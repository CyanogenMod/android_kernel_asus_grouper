/*
 * drivers/power/bq27541_battery.c
 *
 * Gas Gauge driver for TI's BQ27541
 *
 * Copyright (c) 2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <asm/unaligned.h>
#include <linux/miscdevice.h>
#include <mach/gpio.h>
#include "../../arch/arm/mach-tegra/gpio-names.h"
#include "../../arch/arm/mach-tegra/wakeups-t3.h"

#define SMBUS_RETRY                                     (0)
#define BAT_IN_DET                                        TEGRA_GPIO_PN4
#define GPIOPIN_BATTERY_DETECT	         BAT_IN_DET
#define GPIOPIN_LOW_BATTERY_DETECT	  TEGRA_GPIO_PS4
#define BATTERY_POLLING_RATE	(60)
#define DELAY_FOR_CORRECT_CHARGER_STATUS	(5)
#define TEMP_KELVIN_TO_CELCIUS		(2731)
#define MAXIMAL_VALID_BATTERY_TEMP	(200)
#define USB_NO_Cable	0
#define USB_DETECT_CABLE	1
#define USB_SHIFT	0
#define AC_SHIFT	1
#define USB_Cable ((1 << (USB_SHIFT)) | (USB_DETECT_CABLE))
#define USB_AC_Adapter ((1 << (AC_SHIFT)) | (USB_DETECT_CABLE))
#define USB_CALBE_DETECT_MASK (USB_Cable  | USB_DETECT_CABLE)

#define BATTERY_MANUFACTURER_SIZE	12
#define BATTERY_NAME_SIZE		8

/* Battery flags bit definitions */
#define BATT_STS_DSG		0x0001
#define BATT_STS_FC			0x0200

/* Debug Message */
#define BAT_NOTICE(format, arg...)	\
	printk(KERN_NOTICE "%s " format , __FUNCTION__ , ## arg)

#define BAT_ERR(format, arg...)		\
	printk(KERN_ERR format , ## arg)

/* Global variable */
unsigned battery_cable_status = 0;
unsigned battery_driver_ready = 0;
static int ac_on ;
static int usb_on ;
static unsigned int 	battery_current;
static unsigned int  battery_remaining_capacity;
static atomic_t device_count;
struct workqueue_struct *battery_work_queue = NULL;

/* Functions declaration */
static int bq27541_get_psp(int reg_offset, enum power_supply_property psp,union power_supply_propval *val);
static int bq27541_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);
extern unsigned  get_usb_cable_status(void);

module_param(battery_current, uint, 0644);
module_param(battery_remaining_capacity, uint, 0644);

#define BQ27541_DATA(_psp, _addr, _min_value, _max_value)	\
	{								\
		.psp = POWER_SUPPLY_PROP_##_psp,	\
		.addr = _addr,				\
		.min_value = _min_value,		\
		.max_value = _max_value,	\
	}

enum {
       REG_MANUFACTURER_DATA,
	REG_STATE_OF_HEALTH,
	REG_TEMPERATURE,
	REG_VOLTAGE,
	REG_CURRENT,
	REG_TIME_TO_EMPTY,
	REG_TIME_TO_FULL,
	REG_STATUS,
	REG_CAPACITY,
	REG_SERIAL_NUMBER,
	REG_MAX
};

typedef enum {
	Charger_Type_Battery = 0,
	Charger_Type_AC,
	Charger_Type_USB,
	Charger_Type_Num,
	Charger_Type_Force32 = 0x7FFFFFFF
} Charger_Type;

static struct bq27541_device_data {
	enum power_supply_property psp;
	u8 addr;
	int min_value;
	int max_value;
} bq27541_data[] = {
       [REG_MANUFACTURER_DATA]	= BQ27541_DATA(PRESENT, 0, 0, 65535),
       [REG_STATE_OF_HEALTH]		= BQ27541_DATA(HEALTH, 0, 0, 65535),
	[REG_TEMPERATURE]			= BQ27541_DATA(TEMP, 0x06, 0, 65535),
	[REG_VOLTAGE]				= BQ27541_DATA(VOLTAGE_NOW, 0x08, 0, 6000),
	[REG_CURRENT]				= BQ27541_DATA(CURRENT_NOW, 0x14, -32768, 32767),
	[REG_TIME_TO_EMPTY]			= BQ27541_DATA(TIME_TO_EMPTY_AVG, 0x16, 0, 65535),
	[REG_TIME_TO_FULL]			= BQ27541_DATA(TIME_TO_FULL_AVG, 0x18, 0, 65535),
	[REG_STATUS]				= BQ27541_DATA(STATUS, 0x0a, 0, 65535),
	[REG_CAPACITY]				= BQ27541_DATA(CAPACITY, 0x2c, 0, 100),
};

static enum power_supply_property bq27541_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

void check_cabe_type(void)
{
      if(battery_cable_status == USB_AC_Adapter) {
		 ac_on = 1;
	        usb_on = 0;
	}
	else if(battery_cable_status  == USB_Cable) {
		usb_on = 1;
		ac_on = 0;
	}
	else {
		ac_on = 0;
		usb_on = 0;
	}
}

static enum power_supply_property power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	int ret=0;
	switch (psp) {

	case POWER_SUPPLY_PROP_ONLINE:
		   if(psy->type == POWER_SUPPLY_TYPE_MAINS &&  ac_on)
			val->intval =  1;
		   else if (psy->type == POWER_SUPPLY_TYPE_USB && usb_on)
			val->intval =  1;
		   else
			val->intval = 0;
		break;

	default:
		return -EINVAL;
	}
	return ret;
}

static char *supply_list[] = {
	"battery",
	"ac",
#ifndef REMOVE_USB_POWER_SUPPLY
	"usb",
#endif
};

static struct power_supply bq27541_supply[] = {
	{
		.name		= "battery",
		.type		= POWER_SUPPLY_TYPE_BATTERY,
		.properties	= bq27541_properties,
		.num_properties = ARRAY_SIZE(bq27541_properties),
		.get_property	= bq27541_get_property,
       },
	{
		.name		= "ac",
		.type		= POWER_SUPPLY_TYPE_MAINS,
		.supplied_to	= supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = power_properties,
		.num_properties = ARRAY_SIZE(power_properties),
		.get_property	= power_get_property,
	},
#ifndef REMOVE_USB_POWER_SUPPLY
	{
		.name		= "usb",
		.type		= POWER_SUPPLY_TYPE_USB,
		.supplied_to	= supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties =power_properties,
		.num_properties = ARRAY_SIZE(power_properties),
		.get_property = power_get_property,
	},
#endif
};

static struct bq27541_device_info {
	struct i2c_client *client;
	struct delayed_work battery_stress_test;
	struct delayed_work status_poll_work;
	struct delayed_work low_low_bat_work;
	struct miscdevice battery_misc;
	struct wake_lock low_battery_wake_lock;
	int smbus_status;
	int battery_present;
	int low_battery_present;
	int gpio_battery_detect;
	int gpio_low_battery_detect;
	int irq_low_battery_detect;
	int irq_battery_detect;
	int bat_status;
	int bat_temp;
	int bat_vol;
	int bat_current;
	int bat_capacity;
	unsigned int old_capacity;
	unsigned int cap_err;
	unsigned int old_temperature;
	unsigned int temp_err;
	unsigned int prj_id;
	spinlock_t lock;
} *bq27541_device;

static int bq27541_read_i2c(u8 reg, int *rt_value, int b_single)
{
	struct i2c_client *client = bq27541_device->client;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		if (!b_single)
			msg->len = 2;
		else
			msg->len = 1;

		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			if (!b_single)
				*rt_value = get_unaligned_le16(data);
			else
				*rt_value = data[0];

			return 0;
		}
	}
	return err;
}

int bq27541_smbus_read_data(int reg_offset,int byte,int *rt_value)
{
     s32 ret=-EINVAL;
     int count=0;

	do {
		ret = bq27541_read_i2c(bq27541_data[reg_offset].addr, rt_value, 0);
	} while((ret<0)&&(++count<=SMBUS_RETRY));
	return ret;
}

int bq27541_smbus_write_data(int reg_offset,int byte, unsigned int value)
{
     s32 ret = -EINVAL;
     int count=0;

	do{
		if(byte){
			ret = i2c_smbus_write_byte_data(bq27541_device->client,bq27541_data[reg_offset].addr,value&0xFF);
		}
		else{
			ret = i2c_smbus_write_word_data(bq27541_device->client,bq27541_data[reg_offset].addr,value&0xFFFF);
		}
	}while((ret<0) && (++count<=SMBUS_RETRY));
	return ret;
}

static ssize_t show_battery_smbus_status(struct device *dev, struct device_attribute *devattr, char *buf)
{
	int status=!bq27541_device->smbus_status;
	return sprintf(buf, "%d\n", status);
}
static DEVICE_ATTR(battery_smbus, S_IWUSR | S_IRUGO, show_battery_smbus_status,NULL);

static struct attribute *battery_smbus_attributes[] = {

	&dev_attr_battery_smbus.attr,
	NULL
};

static const struct attribute_group battery_smbus_group = {
	.attrs = battery_smbus_attributes,
};

static void battery_status_poll(struct work_struct *work)
{
       struct bq27541_device_info *batt_dev = container_of(work, struct bq27541_device_info, status_poll_work.work);

	if(!battery_driver_ready)
		BAT_NOTICE("battery driver not ready\n");

	power_supply_changed(&bq27541_supply[Charger_Type_Battery]);

	/* Schedule next polling */
	queue_delayed_work(battery_work_queue, &batt_dev->status_poll_work, BATTERY_POLLING_RATE*HZ);
}

static void low_low_battery_check(struct work_struct *work)
{
	cancel_delayed_work(&bq27541_device->status_poll_work);
	queue_delayed_work(battery_work_queue,&bq27541_device->status_poll_work, 0.1*HZ);
	msleep(2000);
	enable_irq(bq27541_device->irq_low_battery_detect);
}

static irqreturn_t low_battery_detect_isr(int irq, void *dev_id)
{
	disable_irq_nosync(bq27541_device->irq_low_battery_detect);
	bq27541_device->low_battery_present = gpio_get_value(bq27541_device->gpio_low_battery_detect);
	BAT_NOTICE("gpio LL_BAT_T30=%d\n", bq27541_device->low_battery_present);
	wake_lock_timeout(&bq27541_device->low_battery_wake_lock, 10*HZ);
	queue_delayed_work(battery_work_queue, &bq27541_device->low_low_bat_work, 0.1*HZ);
	return IRQ_HANDLED;
}

static int setup_low_battery_irq(void)
{
	unsigned gpio = GPIOPIN_LOW_BATTERY_DETECT;
	int ret, irq = gpio_to_irq(gpio);

	ret = gpio_request(gpio, "low_battery_detect");
	if (ret < 0) {
		BAT_ERR("gpio LL_BAT_T30 request failed\n");
		goto fail;
	}

	tegra_gpio_enable(gpio);

	ret = gpio_direction_input(gpio);
	if (ret < 0) {
		BAT_ERR("gpio LL_BAT_T30 unavaliable for input\n");
		goto fail_gpio;
	}

	ret = request_irq(irq, low_battery_detect_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"bq27541-battery (low battery)", NULL);
	if (ret < 0) {
		BAT_ERR("gpio LL_BAT_T30 irq request failed\n");
		goto fail_irq;
	}

	bq27541_device->low_battery_present = gpio_get_value(gpio);
	bq27541_device->irq_low_battery_detect = gpio_to_irq(gpio);
	BAT_NOTICE("irq=%d, LL_BAT_T30=%d\n", irq, bq27541_device->low_battery_present);

	enable_irq_wake(bq27541_device->irq_low_battery_detect);

fail_irq:
fail_gpio:
	gpio_free(gpio);
fail:
	return ret;
}

int battery_callback(unsigned usb_cable_state)
{
	int old_cable_status;

	if(!battery_driver_ready) {
		BAT_NOTICE("battery driver not ready\n");
		return 0;
	}

	old_cable_status = battery_cable_status;
	battery_cable_status = usb_cable_state;

       printk("========================================================\n");
	printk("battery_callback  usb_cable_state = %x\n", usb_cable_state) ;
       printk("========================================================\n");

	check_cabe_type();

	if(! battery_cable_status) {
		if (old_cable_status == USB_AC_Adapter) {
			power_supply_changed(&bq27541_supply[Charger_Type_AC]);
		}
		#ifndef REMOVE_USB_POWER_SUPPLY
		else if ( old_cable_status == USB_Cable) {
			power_supply_changed(&bq27541_supply[Charger_Type_USB]);
		}
		#endif
	}
	#ifndef REMOVE_USB_POWER_SUPPLY
	else if (battery_cable_status == USB_Cable) {
		power_supply_changed(&bq27541_supply[Charger_Type_USB]);
	}
	#endif
	else if (battery_cable_status == USB_AC_Adapter) {
		power_supply_changed(&bq27541_supply[Charger_Type_AC]);
	}
	cancel_delayed_work(&bq27541_device->status_poll_work);
	queue_delayed_work(battery_work_queue, &bq27541_device->status_poll_work, 2*HZ);

	return 1;
}
EXPORT_SYMBOL(battery_callback);

static int bq27541_get_health(enum power_supply_property psp,
	union power_supply_propval *val)
{
	if (psp == POWER_SUPPLY_PROP_PRESENT) {
		val->intval = 1;
	}
	else if (psp == POWER_SUPPLY_PROP_HEALTH) {
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
	}
	return 0;
}

static int bq27541_get_psp(int reg_offset, enum power_supply_property psp,
	union power_supply_propval *val)
{
	s32 ret;
	int smb_retry=0;
	int rt_value=0;
	static char *status_text[] = {"Unknown", "Charging", "Discharging", "Not charging", "Full"};

	bq27541_device->smbus_status = bq27541_smbus_read_data(reg_offset, 0, &rt_value);

	if (bq27541_device->smbus_status < 0) {
		dev_err(&bq27541_device->client->dev,
			"%s: i2c read for %d failed\n", __func__, reg_offset);

		if(psp == POWER_SUPPLY_PROP_TEMP && (++bq27541_device->temp_err<=3) && (bq27541_device->old_temperature!=0xFF)){
			val->intval = bq27541_device->old_temperature;
			BAT_NOTICE("temperature read fail, err= %u use old temp=%u\n", bq27541_device->temp_err, val->intval);
			return 0;
		}
		else
			return -EINVAL;
	}
	if (psp == POWER_SUPPLY_PROP_VOLTAGE_NOW) {
		val->intval = bq27541_device->bat_vol = rt_value;
		BAT_NOTICE("voltage_now= %u mV\n", val->intval);
	}
	if (psp == POWER_SUPPLY_PROP_STATUS) {
		ret = bq27541_device->bat_status = rt_value;

		if (ac_on || usb_on) {            /* Charging detected */
			if (bq27541_device->old_capacity == 100)
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
		} else if (ret & BATT_STS_FC) {                          /* Full-charged condition reached */
			if (!ac_on)
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_FULL;
		} else if (ret & BATT_STS_DSG) {                       /* Discharging detected */
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		} else {
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
		BAT_NOTICE("status: %s ret= 0x%04x\n", status_text[val->intval], ret);

	} else if (psp == POWER_SUPPLY_PROP_TEMP) {
		ret = bq27541_device->bat_temp = rt_value;
		ret -= TEMP_KELVIN_TO_CELCIUS;

		if ((ret/10) >= MAXIMAL_VALID_BATTERY_TEMP && bq27541_device->temp_err == 0) {
			ret=300;
			BAT_NOTICE("[Warning] set temp=30 (0.1�XC)");
			WARN_ON(1);
			bq27541_device->temp_err++;
		}
		else {
			bq27541_device->temp_err = 0;
		}

		bq27541_device->old_temperature = val->intval = ret;
		BAT_NOTICE("temperature= %u (0.1�XC)\n", val->intval);
	}
	return 0;
}

static int bq27541_get_capacity(union power_supply_propval *val)
{
	s32 ret;
	s32 temp_capacity;
	int smb_retry=0;

	do {
		bq27541_device->smbus_status = bq27541_smbus_read_data(REG_CAPACITY, 0 ,&bq27541_device->bat_capacity);
	} while((bq27541_device->smbus_status < 0) && ( ++smb_retry <= SMBUS_RETRY));

	if (bq27541_device->smbus_status < 0) {
		dev_err(&bq27541_device->client->dev, "%s: i2c read for %d "
			"failed bq27541_device->cap_err=%u\n", __func__, REG_CAPACITY, bq27541_device->cap_err);

		if(bq27541_device->cap_err>5 || (bq27541_device->old_capacity == 0xFF)) {
			return -EINVAL;
		} else {
			val->intval = bq27541_device->old_capacity;
			bq27541_device->cap_err++;
			BAT_NOTICE("cap_err=%u use old capacity=%u\n", bq27541_device->cap_err, val->intval);
			return 0;
		}
	}

	ret = bq27541_device->bat_capacity;

	temp_capacity = ((ret >= 100) ? 100 : ret);

	/* start: for mapping %99 to 100%. Lose 84%*/
	if(temp_capacity==99)
		temp_capacity=100;
	if(temp_capacity >=84 && temp_capacity <=98)
		temp_capacity++;
	/* for mapping %99 to 100% */

	 /* lose 26% 47% 58%,69%,79% */
	if(temp_capacity >70 && temp_capacity <80)
		temp_capacity-=1;
	else if(temp_capacity >60&& temp_capacity <=70)
		temp_capacity-=2;
	else if(temp_capacity >50&& temp_capacity <=60)
		temp_capacity-=3;
	else if(temp_capacity >30&& temp_capacity <=50)
		temp_capacity-=4;
	else if(temp_capacity >=0&& temp_capacity <=30)
		temp_capacity-=5;

	/*Re-check capacity to avoid  that temp_capacity <0*/
	temp_capacity = ((temp_capacity <0) ? 0 : temp_capacity);
	val->intval = temp_capacity;

	bq27541_device->old_capacity = val->intval;
	bq27541_device->cap_err=0;

	BAT_NOTICE("= %u%% ret= %u\n", val->intval, ret);
	return 0;
}

static int bq27541_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	u8 count;
	switch (psp) {
		case POWER_SUPPLY_PROP_PRESENT:
		case POWER_SUPPLY_PROP_HEALTH:
			if (bq27541_get_health(psp, val))
				goto error;
			break;

		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;

		case POWER_SUPPLY_PROP_CAPACITY:
			if (bq27541_get_capacity(val))
				goto error;
			break;

		case POWER_SUPPLY_PROP_STATUS:
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		case POWER_SUPPLY_PROP_CURRENT_NOW:
		case POWER_SUPPLY_PROP_TEMP:
		case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		case POWER_SUPPLY_PROP_SERIAL_NUMBER:
			for (count = 0; count < REG_MAX; count++) {
				if (psp == bq27541_data[count].psp)
					break;
			}

			if (bq27541_get_psp(count, psp, val))
				return -EINVAL;
			break;

		default:
			dev_err(&bq27541_device->client->dev,
				"%s: INVALID property psp=%u\n", __func__,psp);
			return -EINVAL;
	}

	return 0;

error:

	return -EINVAL;
}

#include "stress_test.c"
static int bq27541_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret, i=0;

	BAT_NOTICE("+ client->addr= %02x\n", client->addr);

	bq27541_device = kzalloc(sizeof(*bq27541_device), GFP_KERNEL);
	if (!bq27541_device)
		return -ENOMEM;

	memset(bq27541_device, 0, sizeof(*bq27541_device));
	bq27541_device->client = client;
	i2c_set_clientdata(client, bq27541_device);
       bq27541_device->smbus_status = 0;
       bq27541_device->cap_err = 0;
	bq27541_device->temp_err = 0;
	bq27541_device->old_capacity = 0xFF;
       bq27541_device->old_temperature = 0xFF;
	bq27541_device->gpio_low_battery_detect = GPIOPIN_LOW_BATTERY_DETECT;

	for (i = 0; i < ARRAY_SIZE(bq27541_supply); i++) {
		ret = power_supply_register(&client->dev, &bq27541_supply[i]);
		if (ret) {
			BAT_ERR("Failed to register power supply\n");
			while (i--)
				power_supply_unregister(&bq27541_supply[i]);
				kfree(bq27541_device);
			return ret;
		}
	}

	battery_work_queue = create_singlethread_workqueue("battery_workqueue");
	INIT_DELAYED_WORK(&bq27541_device->status_poll_work, battery_status_poll);
	INIT_DELAYED_WORK(&bq27541_device->low_low_bat_work, low_low_battery_check);
       INIT_DELAYED_WORK(&bq27541_device->battery_stress_test, battery_strees_test);
	cancel_delayed_work(&bq27541_device->status_poll_work);

	spin_lock_init(&bq27541_device->lock);
	wake_lock_init(&bq27541_device->low_battery_wake_lock, WAKE_LOCK_SUSPEND, "low_battery_detection");

	/* Register sysfs */
	ret = sysfs_create_group(&client->dev.kobj, &battery_smbus_group);
	if (ret) {
		dev_err(&client->dev, "bq27541_probe: unable to create the sysfs\n");
	}

	/* Misc device registration */
       bq27541_device->battery_misc.minor = MISC_DYNAMIC_MINOR;
	bq27541_device->battery_misc.name = "battery";
	bq27541_device->battery_misc.fops   = &battery_fops;
       ret = misc_register(&bq27541_device->battery_misc);
	if(ret) {
		pr_info("Cannot register bq27541 miscdev (err=%d)\n", ret);
	}

	setup_low_battery_irq();

	battery_driver_ready = 1;

	battery_cable_status = get_usb_cable_status();
	queue_delayed_work(battery_work_queue, &bq27541_device->status_poll_work, 15*HZ);

	BAT_NOTICE("- %s driver registered\n", client->name);

	return 0;
}

static int bq27541_remove(struct i2c_client *client)
{
	struct bq27541_device_info *bq27541_device;
	int i=0;
	bq27541_device = i2c_get_clientdata(client);
	for (i = 0; i < ARRAY_SIZE(bq27541_supply); i++) {
		power_supply_unregister(&bq27541_supply[i]);
	}
	if (bq27541_device) {
		wake_lock_destroy(&bq27541_device->low_battery_wake_lock);
		kfree(bq27541_device);
		bq27541_device = NULL;
	}
	return 0;
}

#if defined (CONFIG_PM)
static int bq27541_suspend(struct i2c_client *client, pm_message_t state)
{
	cancel_delayed_work_sync(&bq27541_device->status_poll_work);
	flush_workqueue(battery_work_queue);;
	return 0;
}

/* any smbus transaction will wake up pad */
static int bq27541_resume(struct i2c_client *client)
{
	cancel_delayed_work(&bq27541_device->status_poll_work);
	queue_delayed_work(battery_work_queue,&bq27541_device->status_poll_work, 5*HZ);
	return 0;
}
#endif

static const struct i2c_device_id bq27541_id[] = {
	{ "bq27541-battery", 0 },
	{},
};

static struct i2c_driver bq27541_battery_driver = {
	.probe	= bq27541_probe,
	.remove 	= bq27541_remove,
#if defined (CONFIG_PM)
	.suspend	= bq27541_suspend,
	.resume 	= bq27541_resume,
#endif
	.id_table	= bq27541_id,
	.driver = {
		.name = "bq27541-battery",
	},
};
static int __init bq27541_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&bq27541_battery_driver);
	if (ret)
		dev_err(&bq27541_device->client->dev,
			"%s: i2c_add_driver failed\n", __func__);

	return ret;
}
module_init(bq27541_battery_init);

static void __exit bq27541_battery_exit(void)
{
	i2c_del_driver(&bq27541_battery_driver);
}
module_exit(bq27541_battery_exit);

MODULE_AUTHOR("NVIDIA Corporation");
MODULE_DESCRIPTION("bq27541 battery monitor driver");
MODULE_LICENSE("GPL");

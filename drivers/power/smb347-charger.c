/*
 * drivers/power/smb347-charger.c
 *
 * Battery charger driver for smb347 from summit microelectronics
 *
 * Copyright (c) 2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/smb347-charger.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/usb/otg.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <mach/board-grouper-misc.h>
#include "../../arch/arm/mach-tegra/gpio-names.h"

#define smb347_CHARGE		0x00
#define smb347_CHRG_CRNTS	0x01
#define smb347_VRS_FUNC		0x02
#define smb347_FLOAT_VLTG	0x03
#define smb347_CHRG_CTRL	0x04
#define smb347_STAT_TIME_CTRL	0x05
#define smb347_PIN_CTRL		0x06
#define smb347_THERM_CTRL	0x07
#define smb347_SYSOK_USB3	0x08
#define smb347_CTRL_REG		0x09

#define smb347_OTG_TLIM_REG	0x0A
#define smb347_HRD_SFT_TEMP	0x0B
#define smb347_FAULT_INTR	0x0C
#define smb347_STS_INTR_1	0x0D
#define smb347_I2C_ADDR	0x0E
#define smb347_IN_CLTG_DET	0x10
#define smb347_STS_INTR_2	0x11

/* Command registers */
#define smb347_CMD_REG		0x30
#define smb347_CMD_REG_B	0x31
#define smb347_CMD_REG_c	0x33

/* Interrupt Status registers */
#define smb347_INTR_STS_A	0x35
#define smb347_INTR_STS_B	0x36
#define smb347_INTR_STS_C	0x37
#define smb347_INTR_STS_D	0x38
#define smb347_INTR_STS_E	0x39
#define smb347_INTR_STS_F	0x3A

/* Status registers */
#define smb347_STS_REG_A	0x3B
#define smb347_STS_REG_B	0x3C
#define smb347_STS_REG_C	0x3D
#define smb347_STS_REG_D	0x3E
#define smb347_STS_REG_E	0x3F

#define smb347_ENABLE_WRITE	1
#define smb347_DISABLE_WRITE	0
#define ENABLE_WRT_ACCESS	0x80
#define ENABLE_APSD		0x04
#define HC_MODE		0x01
#define USB_5_9_CUR		0x02
#define PIN_CTRL		0x10
#define PIN_ACT_LOW	0x20
#define THERM_CTRL		0x10
#define BATTERY_MISSING		0x10
#define CHARGING		0x06
#define DEDICATED_CHARGER	0x02
#define CHRG_DOWNSTRM_PORT	0x04
#define ENABLE_CHARGE		0x02
#define ENABLE_CHARGER		1
#define DISABLE_CHARGER		0
#define USBIN		0x80
#define APSD_OK		0x08
#define APSD_RESULT		0x07
#define APSD_CDP		0x01
#define APSD_DCP		0x02
#define APSD_OTHER		0x03
#define APSD_SDP		0x04
#define APSD_HOST_MODE_CHARGING		0x06
#define USB_30		0x20
#define DCIN_OV_UV_STS		0x50
#define DELAY_FOR_CURR_LIMIT_RECONF (60)
#define ADAPTER_PROTECT_DELAY (4*HZ)
#define GPIO_AC_OK		TEGRA_GPIO_PV1

/* Functions declaration */
static int smb347_configure_charger(struct i2c_client *client, int value);
static int smb347_configure_interrupts(struct i2c_client *client);
extern int battery_callback(unsigned usb_cable_state);
/* Enable or disable the callback for the battery driver. */
#define TOUCH_CALLBACK_ENABLED 1
#ifdef TOUCH_CALLBACK_ENABLED
extern void touch_callback(unsigned cable_status);
#endif

static ssize_t smb347_reg_show(struct device *dev, struct device_attribute *attr, char *buf);

/* Global variables */
static struct smb347_charger *charger;
static struct workqueue_struct *smb347_wq;
struct wake_lock charger_wakelock;
static unsigned int project_id;
static unsigned int pcba_ver;
static int gpio_dock_in = 0;

// tmtmtm: also modify 'export KBUILD_BUILD_USER=timur-usbhost-fi-2013-01-01
//static int fixed_install_mode = 0;
extern int fixed_install_mode;
volatile int smb347_deep_sleep = 0;  // imported by ehci-tegra.c
static volatile int host_mode_charging_state = 0;
static volatile int lastExternalPowerState = 0;
static volatile int lastOtgState = 0;
static volatile int lastChargeSlaveDevicesState = 0;
static volatile int hostmode_waiting_for_power = 0;

/* Sysfs interface */
static DEVICE_ATTR(reg_status, S_IWUSR | S_IRUGO, smb347_reg_show, NULL);

static struct attribute *smb347_attributes[] = {
	&dev_attr_reg_status.attr,
NULL
};

static const struct attribute_group smb347_group = {
	.attrs = smb347_attributes,
};

static int smb347_read(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int smb347_write(struct i2c_client *client, int reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int smb347_update_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret, retval;

	retval = smb347_read(client, reg);
	if (retval < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, retval);
		return retval;
	}

	ret = smb347_write(client, reg, retval | value);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	return ret;
}

static int smb347_clear_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret, retval;

	retval = smb347_read(client, reg);
	if (retval < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, retval);
		return retval;
	}

	ret = smb347_write(client, reg, retval & (~value));
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	return ret;
}

int smb347_volatile_writes(struct i2c_client *client, uint8_t value)
{
	int ret = 0;

	if (value == smb347_ENABLE_WRITE) {
		/* Enable volatile write to config registers */
		ret = smb347_update_reg(client, smb347_CMD_REG,
						ENABLE_WRT_ACCESS);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing"
				"register 0x%02x\n", __func__, smb347_CMD_REG);
			return ret;
		}
	} else {
		ret = smb347_read(client, smb347_CMD_REG);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return ret;
		}

		ret = smb347_write(client, smb347_CMD_REG, ret & (~(1<<7)));
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return ret;
		}
	}
	return ret;
}

static void smb347_clear_interrupts(struct i2c_client *client)
{
	uint8_t val, buf[6];

	val = i2c_smbus_read_i2c_block_data(client, smb347_INTR_STS_A, 6, buf);
	if (val < 0)
		dev_err(&client->dev, "%s(): Failed in clearing interrupts\n",
								__func__);
}

static int smb347_configure_otg(struct i2c_client *client, int enableOTG, int chargeSlaves, int stopChargeSlaves)
{
	int ret = 0;

    printk("smb347_configure_otg %d %d %d %d\n",enableOTG, chargeSlaves, stopChargeSlaves, lastOtgState);

	/*Enable volatile writes to registers*/
	ret = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s error in configuring otg..\n",
								__func__);
		goto error;
	}

	// tmtmtm: we will never charge slave devices in fixed_install_mode
	if(!fixed_install_mode) {
		if(chargeSlaves) {
			if(!lastChargeSlaveDevicesState) {
				/* Configure INOK to be active high */
				//printk("smb347_configure_otg INOK to be active high\n");
				ret = smb347_update_reg(client, smb347_SYSOK_USB3, 0x01);
				if (ret < 0) {
					dev_err(&client->dev, "%s: err %d\n", __func__, ret);
					goto error;
				}

				/* Change "OTG output current limit" to 250mA */
				//printk("smb347_configure_otg charge slaves 250mA\n");
				ret = smb347_read(client, smb347_OTG_TLIM_REG);
				if (ret < 0) {
					dev_err(&client->dev, "%s: err %d\n", __func__, ret);
					goto error;
				}
				ret = smb347_write(client, smb347_OTG_TLIM_REG, (ret & (~(1<<3))));
				if (ret < 0) {
					dev_err(&client->dev, "%s: err %d\n", __func__, ret);
					goto error;
				}
			}
		}
	}

	if(enableOTG>0) {
		if(!lastOtgState) {
			printk("smb347_configure_otg enable host mode\n");
			ret = smb347_update_reg(client, smb347_CMD_REG, 0x10);
			if (ret < 0) {
				dev_err(&client->dev, "%s: Failed in writing register"
					  "0x%02x\n", __func__, smb347_CMD_REG);
				goto error;
			}
			lastOtgState = 1;
		}
	} else if(enableOTG==0) {
		if(lastOtgState) {
			printk("smb347_configure_otg disable host mode\n");
			ret = smb347_read(client, smb347_CMD_REG);
			if (ret < 0) {
				dev_err(&client->dev, "%s: err %d\n", __func__, ret);
				goto error;
			}

			ret = smb347_write(client, smb347_CMD_REG, (ret & (~(1<<4))));
			if (ret < 0) {
				dev_err(&client->dev, "%s: err %d\n", __func__, ret);
				goto error;
			}
			lastOtgState=0;
		}
	}

	// tmtmtm: we will never charge slave devices in fixed_install_mode
	if(!fixed_install_mode) {
		if(chargeSlaves) {
			if(!lastChargeSlaveDevicesState) {
				/* Change "OTG output current limit" from 250mA to 750mA */
				//printk("smb347_configure_otg charge slaves 750mA\n");
				ret = smb347_update_reg(client, smb347_OTG_TLIM_REG, 0x08);
				if (ret < 0) {
					dev_err(&client->dev, "%s: Failed in writing register"
						"0x%02x\n", __func__, smb347_OTG_TLIM_REG);
					goto error;
				}
				lastChargeSlaveDevicesState = 1;
				printk("smb347_configure_otg lastChargeSlaveDevicesState=%d\n",lastChargeSlaveDevicesState);
			}
		}
		else
		if(stopChargeSlaves) {
			if(lastChargeSlaveDevicesState) {
				//printk("smb347_configure_otg stop charging slaves\n");
				/* Configure INOK to be active low */
				ret = smb347_read(client, smb347_SYSOK_USB3);
				if (ret < 0) {
					dev_err(&client->dev, "%s: err %d\n", __func__, ret);
					goto error;
				}

				ret = smb347_write(client, smb347_SYSOK_USB3, (ret & (~(1))));
				if (ret < 0) {
					dev_err(&client->dev, "%s: err %d\n", __func__, ret);
					goto error;
				}
				lastChargeSlaveDevicesState = 0;
				printk("smb347_configure_otg lastChargeSlaveDevicesState=%d\n",lastChargeSlaveDevicesState);
			}
		}
	}

	/* Disable volatile writes to registers */
	ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s error in configuring OTG..\n",
								__func__);
	       goto error;
	}
error:
	return ret;
}

static int smb347_configure_charger(struct i2c_client *client, int value)
{
	int ret = 0;

	/* Enable volatile writes to registers */
	ret = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}

	if (value) {
		if(!host_mode_charging_state) {
			printk("smb347_configure_charger accept external power\n");
			ret = smb347_update_reg(client, smb347_CMD_REG, ENABLE_CHARGE);
			if (ret < 0) {
				dev_err(&client->dev, "%s(): Failed in writing register"
						"0x%02x\n", __func__, smb347_CMD_REG);
				goto error;
			}
		}
		/* Configure THERM ctrl */
		/*
		ret = smb347_update_reg(client, smb347_THERM_CTRL, THERM_CTRL);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}
		*/
	} else {
		// tmtmtm: make sure to NEVER call this in fixed_install_mode
		printk("smb347_configure_charger do not charge; fixed_install_mode=%d\n",fixed_install_mode);
		ret = smb347_read(client, smb347_CMD_REG);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}

		ret = smb347_write(client, smb347_CMD_REG, (ret & (~(1<<1))));
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}
	}
	/* Disable volatile writes to registers */
	ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}
error:
	if(ret!=0)
		printk(KERN_INFO "smb347_configure_charger ERROR %d\n",ret);
	return ret;
}

static int smb347_charger_enable(bool enable)
{
	struct i2c_client *client = charger->client;
	u8 ret = 0;

	printk("smb347_charger_enable %d\n",enable);
	if (enable) {
		/*Pin Controls -active low */
		ret = smb347_update_reg(client, smb347_PIN_CTRL, PIN_ACT_LOW);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed to"
						"enable charger\n", __func__);
			return ret;
		}
	} else {
		/*Pin Controls -active high */
		ret = smb347_clear_reg(client, smb347_PIN_CTRL, PIN_ACT_LOW);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed to"
						"disable charger\n", __func__);
			return ret;
		}
	}
	return ret;
}


static int
smb347_set_InputCurrentlimit(struct i2c_client *client, u32 current_limit)
{
	int ret = 0, retval;
	u8 setting = 0;

	if (charger->curr_limit == current_limit)
		return  ret;

	wake_lock(&charger_wakelock);
	/* Enable volatile writes to registers */
	ret = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}

	/* disable charger */
	smb347_charger_enable(0);

	/* AICL disable */
	retval = smb347_read(client, smb347_VRS_FUNC);
	if (retval < 0) {
		dev_err(&client->dev, "%s(): Failed in reading 0x%02x",
				__func__, smb347_VRS_FUNC);
		goto error;
	}

	setting = retval & (~(BIT(4)));
	printk(KERN_INFO "[charger] Disable AICL, retval=%x setting=%x\n",
		retval, setting);
	ret = smb347_write(client, smb347_VRS_FUNC, setting);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing 0x%02x to register"
			"0x%02x\n", __func__, setting, smb347_VRS_FUNC);
		goto error;
	}

	/* set input current limit */
	retval = smb347_read(client, smb347_CHRG_CRNTS);
	if (retval < 0) {
		dev_err(&client->dev, "%s(): Failed in reading 0x%02x",
			__func__, smb347_CHRG_CRNTS);
		goto error;
	}
	setting = retval & 0xF0;
	if (current_limit > 900)
		setting |= 0x06;
	else
		setting |= 0x03;

	printk(KERN_INFO "[charger] set cahrger limmit, limit=%u retval =%x setting=%x\n",
		current_limit, retval, setting);

	ret = smb347_write(client, smb347_CHRG_CRNTS, setting);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing 0x%02x to register"
			"0x%02x\n", __func__, setting, smb347_CHRG_CRNTS);
		goto error;
	}

	if (current_limit > 900) {
		charger->time_of_1800mA_limit = jiffies;
		charger->curr_limit = 1800;
	} else{
		charger->time_of_1800mA_limit = 0;
		charger->curr_limit = 900;
	}

	/* AICL enable */
	retval = smb347_read(client, smb347_VRS_FUNC);
	if (retval < 0) {
		dev_err(&client->dev, "%s(): Failed in reading 0x%02x",
				__func__, smb347_VRS_FUNC);
		goto error;
	}

	setting = retval | BIT(4);
	printk(KERN_INFO "[charger] re-enable AICL, setting=%x\n", setting);
	msleep(20);
	ret = smb347_write(client, smb347_VRS_FUNC, setting);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing 0x%02x to register"
			"0x%02x\n", __func__, setting, smb347_VRS_FUNC);
			goto error;
	}

	/* enable charger */
	smb347_charger_enable(1);

	/* Disable volatile writes to registers */
	ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}

error:
	if(ret!=0)
		printk(KERN_INFO "smb347_set_InputCurrentlimit ERROR %d\n",ret);
	wake_unlock(&charger_wakelock);
	return ret;
}

static void smb347_set_curr_limit_work_func(struct work_struct *work)
{
	smb347_set_InputCurrentlimit(charger->client, 1800);
}

static void smb347_test_fail_clear_work_function(struct work_struct *work)
{
	charger->test_1800mA_fail = 0;
}

static irqreturn_t smb347_inok_isr(int irq, void *dev_id)
{
	struct smb347_charger *smb = dev_id;

	queue_delayed_work(smb347_wq, &smb->inok_isr_work, 0.6*HZ);

	return IRQ_HANDLED;
}

static irqreturn_t smb347_dockin_isr(int irq, void *dev_id)
{
	struct smb347_charger *smb = dev_id;

	queue_delayed_work(smb347_wq, &smb->dockin_isr_work, 0*HZ);

	return IRQ_HANDLED;
}

static int smb347_inok_irq(struct smb347_charger *smb)
{
	int err = 0 ;
	unsigned gpio = GPIO_AC_OK;
	unsigned irq_num = gpio_to_irq(gpio);

	err = gpio_request(gpio, "smb347_inok");
	if (err) {
		printk("gpio %d request failed \n", gpio);
		goto err1;
	}

	tegra_gpio_enable(gpio);

	err = gpio_direction_input(gpio);
	if (err) {
		printk("gpio %d unavaliable for input \n", gpio);
		goto err2;
	}

	err = request_irq(irq_num, smb347_inok_isr, IRQF_TRIGGER_FALLING |IRQF_TRIGGER_RISING,
		"smb347_inok", smb);
	if (err < 0) {
		printk("%s irq %d request failed \n","smb347_inok", irq_num);
		goto err2 ;
	}
	printk("GPIO pin irq %d requested ok, smb347_INOK = %s\n", irq_num, gpio_get_value(gpio)? "H":"L");

	return 0 ;

err2:
	gpio_free(gpio);
err1:
	//if(err!=0)
	//	printk(KERN_INFO "smb347_inok_irq ERROR %d\n",err);
	return err;
}

static int smb347_dockin_irq(struct smb347_charger *smb)
{
	int err = 0 ;
	unsigned gpio = gpio_dock_in;
	unsigned irq_num = gpio_to_irq(gpio);

	err = gpio_request(gpio, "smb347_dockin");
	if (err) {
		printk("gpio %d request failed \n", gpio);
		goto err1;
	}

	tegra_gpio_enable(gpio);

	err = gpio_direction_input(gpio);
	if (err) {
		printk("gpio %d unavaliable for input \n", gpio);
		goto err2;
	}

	err = request_irq(irq_num, smb347_dockin_isr, IRQF_SHARED|IRQF_TRIGGER_FALLING |IRQF_TRIGGER_RISING,
		"smb347_dockin", smb);
	if (err < 0) {
		printk("%s irq %d request failed \n","smb347_dockin", irq_num);
		goto err2 ;
	}

	enable_irq_wake(irq_num);
	printk("GPIO pin irq %d requested ok, smb347_DOCK_IN# = %s\n", irq_num, gpio_get_value(gpio)? "H":"L");

	return 0;

err2:
	gpio_free(gpio);
err1:
	return err;
}


int register_callback(charging_callback_t cb, void *args)
{
	struct smb347_charger *charger_data = charger;
	if (!charger_data)
		return -ENODEV;

	charger_data->charger_cb = cb;
	charger_data->charger_cb_data = args;
	return 0;
}
EXPORT_SYMBOL_GPL(register_callback);

int smb347_hc_mode_callback(bool enable, int cur)
{
	struct i2c_client *client = charger->client;
	u8 ret = 0;

	if((pcba_ver > GROUPER_PCBA_ER2) && (project_id == GROUPER_PROJECT_NAKASI))
		return 0;

	if (charger->suspend_ongoing)
		return 0;

	//printk("smb347_hc_mode_callback+\n");

	/* Enable volatile writes to registers */
	ret = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
							__func__);
		goto error;
	}

	if(enable) {
		/* Force switch to HC mode */
		ret = smb347_update_reg(client, smb347_CMD_REG_B,
						HC_MODE);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing"
				"register 0x%02x\n", __func__, smb347_CMD_REG_B);
			return ret;
		}

		/* Change to i2c register control */
		ret = smb347_clear_reg(client, smb347_PIN_CTRL, PIN_CTRL);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing"
				"register 0x%02x\n", __func__, smb347_PIN_CTRL);
			return ret;
		}
	}
	else
	{
		/* USB 2.0 input current limit (ICL) */
		ret = smb347_clear_reg(client, smb347_SYSOK_USB3, USB_30);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing"
				"register 0x%02x\n", __func__, smb347_SYSOK_USB3);
			return ret;
		}

		/* Switch back to USB mode */
		ret = smb347_clear_reg(client, smb347_CMD_REG_B, HC_MODE);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing"
				"register 0x%02x\n", __func__, smb347_CMD_REG_B);
			return ret;
		}

		if(cur) {
			/* USB 500mA */
			ret = smb347_update_reg(client, smb347_CMD_REG_B, USB_5_9_CUR);
			if (ret < 0) {
				dev_err(&client->dev, "%s(): Failed in writing"
					"register 0x%02x\n", __func__, smb347_CMD_REG_B);
				return ret;
			}
		} else {
			/* USB 100mA */
			ret = smb347_clear_reg(client, smb347_CMD_REG_B, USB_5_9_CUR);
			if (ret < 0) {
				dev_err(&client->dev, "%s(): Failed in writing"
					"register 0x%02x\n", __func__, smb347_CMD_REG_B);
				return ret;
			}
		}

		/* Disable auto power source detection (APSD) */
		//printk("smb347_hc_mode_callback Disable auto power source detection\n");
		ret = smb347_clear_reg(client, smb347_CHRG_CTRL, ENABLE_APSD);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing"
				"register 0x%02x\n", __func__, smb347_CHRG_CTRL);
			return ret;
		}

		/* Change to i2c register control */
		ret = smb347_clear_reg(client, smb347_PIN_CTRL, PIN_CTRL);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing"
				"register 0x%02x\n", __func__, smb347_PIN_CTRL);
			return ret;
		}
	}

	 /* Disable volatile writes to registers */
	ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}

	//printk("smb347_hc_mode_callback-\n");
	return ret;

error:
	//if(ret!=0)
	//	printk(KERN_INFO "smb347_hc_mode_callback ERROR %d\n",ret);
	return ret;
}
EXPORT_SYMBOL_GPL(smb347_hc_mode_callback);

int smb347_battery_online(void)
{
	int val, ret;
	struct i2c_client *client = charger->client;

	/* Enable volatile writes to registers */
	ret = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}

	val = smb347_read(client, smb347_INTR_STS_B);
	if (val < 0) {
		dev_err(&client->dev, "%s(): Failed in reading register"
				"0x%02x\n", __func__, smb347_INTR_STS_B);
		return val;
	}
	if (val & BATTERY_MISSING)
		return 0;
	else
		return 1;

	 /* Disable volatile writes to registers */
	ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}

error:
	return ret;
}

static int smb347_configure_interrupts(struct i2c_client *client)
{
	int ret = 0;

	/* Enable volatile writes to registers */
	ret = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}
	/* Setting: Fault assert STAT IRQ */
	ret = smb347_update_reg(client, smb347_FAULT_INTR, 0x00);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing register"
				"0x%02x\n", __func__, smb347_CMD_REG);
		goto error;
	}
	/* Setting: Status assert STAT IRQ */
	ret = smb347_update_reg(client, smb347_STS_INTR_1, 0x14);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		goto error;
	}

	 /* Disable volatile writes to registers */
	ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}

error:
	return ret;
}

static int cable_type_detect(void);

static void smb347_otg_status(enum usb_otg_state to, enum usb_otg_state from, void *data)
{
	struct i2c_client *client = charger->client;
	int ret;
	int newExternalPowerState=0;

	printk("smb347_otg_status from=%d to=%d lastOtgState=%d lastExternalPowerState=%d lastChargeSlaveDevicesState=%d fixed_install_mode=%d\n",
		from,to,lastOtgState,lastExternalPowerState,lastChargeSlaveDevicesState,fixed_install_mode);

    if(to==10) {
    	// only when going suspend (OTG PULL)
        // small sleep, so that ehci-tegra #### tegra_usb_resume can run first
        // and use host_mode_charging_state's current value (probably charging), 
        // before we call cable_type_detect() (when it will likely switch to not charging)
        // FIXME: but is tegra_usb_resume not only called on OTG PLUG?
        // FIXME: do I mean "so that tegra_ehci_irq() can run first" ?
	    schedule_timeout_interruptible(msecs_to_jiffies(100));
        // when doing this pause, smb347_resume() will call cable_type_detect() before we do below
	}
	
	cable_type_detect();

	if (to == OTG_STATE_A_HOST) {
		if(charger->cur_cable_type==1 || charger->cur_cable_type==3)
			newExternalPowerState = 1;

		if(!newExternalPowerState) {
			// no external power
			if(fixed_install_mode) {
				// allow battery to be charged
				printk("smb347_otg_status allow battery to be charged\n");
				ret = smb347_configure_charger(client, 1);
				if (ret < 0)
					dev_err(&client->dev, "%s() error in configuring"
						"otg..\n", __func__);
				// disableOTG, dont chargeSlaves, don't stopChargeSlaves
				printk("smb347_otg_status disableOTG, dont chargeSlaves, don't stopChargeSlaves\n");
				ret = smb347_configure_otg(client, 0, 0, 0);
				if (ret < 0)
					dev_err(&client->dev, "%s() error in configuring"
						"otg..\n", __func__);
			} else {
				// tmtmtm: mobile-mode: we need to be careful NOT to disable charger detection too early
				// once we start charging slaves ourselfs, we will not be able to detect ext power coming in

				// also: why are we waiting here, if inok_isr_work_function is called on power
				//       we actually depend on it to arrive in parallel

		        // make external power detectable in case it is coming back
		        printk("smb347_otg_status make external power detectable\n");
		        ret = smb347_configure_interrupts(client);
		        if (ret < 0)
			        dev_err(&client->dev, "%s() error in configuring"
						        "otg..\n", __func__);

				printk("smb347_otg_status waiting for external power...\n");
				// if power is detected, inok_isr_work_function will strike after aprox 1500 ms
				schedule_timeout_interruptible(msecs_to_jiffies(500));
				schedule_timeout_interruptible(msecs_to_jiffies(500));
				schedule_timeout_interruptible(msecs_to_jiffies(400));
				schedule_timeout_interruptible(msecs_to_jiffies(400));
				if(charger->cur_cable_type==1 || charger->cur_cable_type==3)
					newExternalPowerState = 1;
				if(!newExternalPowerState) {
    				cable_type_detect();
    				if(charger->cur_cable_type==1 || charger->cur_cable_type==3)
    					newExternalPowerState = 1;
    		    }
				printk("smb347_otg_status waiting for external power done %d\n",newExternalPowerState);

				if(!newExternalPowerState) {
					// battery will NOT be charged
					ret = smb347_configure_charger(client, 0);
					if (ret < 0)
						dev_err(&client->dev, "%s() error in configuring"
							"otg..\n", __func__);
					// enableOTG, chargeSlaves, don't stopChargeSlaves
					ret = smb347_configure_otg(client, 1, 1, 0);
					if (ret < 0)
						dev_err(&client->dev, "%s() error in configuring"
							"otg..\n", __func__);
				}
			}
		}

		if(newExternalPowerState) {
			// allow battery to be charged
			printk("smb347_otg_status allow battery to be charged\n");
			ret = smb347_configure_charger(client, 1);
			if (ret < 0)
				dev_err(&client->dev, "%s() error in configuring"
					"otg..\n", __func__);
			// enableOTG, don't chargeSlaves, don't stopChargeSlaves
			printk("smb347_otg_status enableOTG, dont chargeSlaves, don't stopChargeSlaves\n");
			ret = smb347_configure_otg(client, 1, 0, 0);
			if (ret < 0)
				dev_err(&client->dev, "%s() error in configuring"
					"otg..\n", __func__);
		}

	} else if (to == OTG_STATE_A_SUSPEND) {

		if(from == OTG_STATE_A_HOST) {
			// disable host-mode and stop slave-charging
			printk("smb347_otg_status disable host-mode and stop slave-charging\n");
			ret = smb347_configure_otg(client, 0, 0, lastChargeSlaveDevicesState);
			if (ret < 0)
				dev_err(&client->dev, "%s() error in configuring"
					"otg..\n", __func__);
			// allow battery to be charged
			printk("smb347_otg_status allow battery to be charged\n");
			ret = smb347_configure_charger(client, 1);
			if (ret < 0)
				dev_err(&client->dev, "%s() error in configuring"
					"otg..\n", __func__);
		}
	}

	//if(!newExternalPowerState /*&& !lastChargeSlaveDevicesState*/) {
		// make external power detectable in case it is coming back
		printk("smb347_otg_status make external power detectable\n");
		ret = smb347_configure_interrupts(client);
		if (ret < 0)
			dev_err(&client->dev, "%s() error in configuring"
						"otg..\n", __func__);
	//}

	lastExternalPowerState = newExternalPowerState;
	printk("smb347_otg_status DONE lastOtgState=%d externalPowerState=%d chargeSlaveDevicesState=%d\n",
		lastOtgState,lastExternalPowerState,lastChargeSlaveDevicesState);
}

/* workqueue function */
static int cable_type_detect(void)
{
	struct i2c_client *client = charger->client;
	u8 retval;
	int  success = 0;
	int ac_ok = GPIO_AC_OK;
	int dock_in = gpio_dock_in;

	printk(KERN_INFO "cable_type_detect()\n");
	/*
	printk("cable_type_detect %d %lu %d %x jiffies=%lu %lu+\n",
	charger->old_cable_type,
	charger->time_of_1800mA_limit,
	gpio_get_value(gpio),
	time_after(charger->time_of_1800mA_limit+(4*HZ), jiffies ),
	jiffies,
	charger->time_of_1800mA_limit+(ADAPTER_PROTECT_DELAY*HZ));
	*/

	if((pcba_ver <= GROUPER_PCBA_ER2) && (project_id == GROUPER_PROJECT_NAKASI)) {
		printk(KERN_INFO "cable_type_detect() wrong\n");
		return 0;
	}

	host_mode_charging_state = 0;
	mutex_lock(&charger->cable_lock);

	if ((charger->old_cable_type == ac_cable) &&
	charger->time_of_1800mA_limit && gpio_get_value(ac_ok) &&
	time_after(charger->time_of_1800mA_limit+
					ADAPTER_PROTECT_DELAY, jiffies)) {
		printk(KERN_INFO "cable_type_detect() charger->test_1800mA_fail\n");
		smb347_set_InputCurrentlimit(client, 900);
		charger->test_1800mA_fail = 1;
		queue_delayed_work(smb347_wq,
				&charger->test_fail_clear_work, 1*HZ);
	}

	if (gpio_get_value(ac_ok)) {
		printk(KERN_INFO "INOK=H no power\n");
		charger->cur_cable_type = non_cable;
		smb347_set_InputCurrentlimit(client, 900);
		success = battery_callback(non_cable);
#ifdef TOUCH_CALLBACK_ENABLED
               touch_callback(non_cable);
#endif
		wake_unlock(&charger_wakelock);
	} else {
		printk(KERN_INFO "INOK=L\n");
		retval = smb347_read(client, smb347_INTR_STS_E);
		SMB_NOTICE("Reg39 : 0x%02x\n", retval);
		if (!(retval & DCIN_OV_UV_STS) && !gpio_get_value(dock_in)) {
			SMB_NOTICE("DC_IN\n");
			success = battery_callback(ac_cable);

			// tmtmtm
			charger->cur_cable_type = ac_cable;
			if(fixed_install_mode) {
				host_mode_charging_state = 1;
				printk(KERN_INFO "cable_type_detect() enabled host_mode_charging_state on DC_IN ######\n");
			}

		} else {
			/* cable type dection */
			retval = smb347_read(client, smb347_STS_REG_E);
			SMB_NOTICE("Reg3F : 0x%02x\n", retval);
			if (retval & USBIN) {
				SMB_NOTICE("USB_IN\n");
				retval = smb347_read(client, smb347_STS_REG_D);
				SMB_NOTICE("Reg3E : 0x%02x\n", retval);
				if (retval & APSD_OK) {
						retval &= APSD_RESULT;
					if (retval == APSD_CDP) {
						printk(KERN_INFO "Cable: CDP\n");
						charger->cur_cable_type = ac_cable;
						success = battery_callback(ac_cable);
#ifdef TOUCH_CALLBACK_ENABLED
	                                    touch_callback(ac_cable);
#endif
					} else if (retval == APSD_DCP) {
						printk(KERN_INFO "Cable: DCP\n");
						charger->cur_cable_type = ac_cable;
						success = battery_callback(ac_cable);
#ifdef TOUCH_CALLBACK_ENABLED
	                                    touch_callback(ac_cable);
#endif
					} else if (retval == APSD_OTHER) {
						charger->cur_cable_type = ac_cable;
						success = battery_callback(ac_cable);
#ifdef TOUCH_CALLBACK_ENABLED
	                                   touch_callback(ac_cable);
#endif
						printk(KERN_INFO "Cable: OTHER\n");
					} else if (retval == APSD_SDP) {
						printk(KERN_INFO "Cable: SDP\n");
						charger->cur_cable_type = usb_cable;
						success = battery_callback(usb_cable);
#ifdef TOUCH_CALLBACK_ENABLED
	                                    touch_callback(usb_cable);
#endif
					} else if(retval == APSD_HOST_MODE_CHARGING) {	// tmtmtm
						printk(KERN_INFO "Cable: host mode charging\n");
						charger->cur_cable_type = usb_cable;
						success = battery_callback(usb_cable);
						host_mode_charging_state = 1;					// tmtmtm
#ifdef TOUCH_CALLBACK_ENABLED
	                                    touch_callback(usb_cable);
#endif
					} else {
						charger->cur_cable_type = unknow_cable;
						printk(KERN_INFO "Unkown Plug In Cable type !\n");
						if (gpio_get_value(dock_in)) {
							charger->cur_cable_type = usb_cable;
							success = battery_callback(usb_cable);
						}
					}
				} else {
					charger->cur_cable_type = unknow_cable;
					printk(KERN_INFO "APSD not completed\n");
				}
			} else {
				charger->cur_cable_type = unknow_cable;
				printk(KERN_INFO "USBIN=0\n");

				// tmtmtm: battery tab keeps stating "Charging (AC)"
				if(fixed_install_mode) {
					host_mode_charging_state = 0;
					printk(KERN_INFO "cable_type_detect() disabled host_mode_charging_state ############\n");
				}
				success = battery_callback(non_cable);
			}
		}
	}

	if (charger->cur_cable_type == ac_cable &&
		charger->old_cable_type != ac_cable &&
		charger->test_1800mA_fail == 0) {
		wake_lock(&charger_wakelock);
		queue_delayed_work(smb347_wq, &charger->curr_limit_work,
					DELAY_FOR_CURR_LIMIT_RECONF*HZ);
	}
	charger->old_cable_type = charger->cur_cable_type;

	mutex_unlock(&charger->cable_lock);
	return success;
}

static void inok_isr_work_function(struct work_struct *dat)
{
	struct i2c_client *client = charger->client;

	// called on power loss/gain, but also if just a bare (non-powered) OTG adapter is pulled
	printk("inok_isr_work_function lastOtgState=%d lastExternalPowerState=%d lastChargeSlaveDevicesState=%d\n",
		lastOtgState,lastExternalPowerState,lastChargeSlaveDevicesState);

	if(lastOtgState>0 && lastExternalPowerState>0) {
		// we used to be in externally powered host mode
		// this means external power was just lost
		cancel_delayed_work(&charger->curr_limit_work);
		cancel_delayed_work(&charger->inok_isr_work);

		// tmtmtm: no external power: in fixed_install_mode we prepare for power to come back
		if(fixed_install_mode) {
			smb347_clear_interrupts(client);

			// stop host-mode, don't chargeSlaves, don't stopChargeSlaves
			printk("inok_isr_work_function fixed_install stop host-mode, don't chargeSlaves, don't stopChargeSlaves\n");
			if(smb347_configure_otg(client, 0, 0, 0)<0)
				dev_err(&client->dev, "%s() error in configuring"
					"otg..\n", __func__);

			// enable external power detection
			printk("inok_isr_work_function fixed_install make external power detectable\n");
			if(smb347_configure_interrupts(client)<0)
				dev_err(&client->dev, "%s() error in configuring"
							"otg..\n", __func__);

			lastExternalPowerState = 0;
			printk("inok_isr_work_function fixed_install make host aware it is now discharging\n");
			// make device aware it is now discharging
			// tmtmtm: notwending ???
			cable_type_detect();

		} else {
			printk("inok_isr_work_function lost external power in host mode; charge slave devices\n");

			// normally, smb347_otg_status() is called whenever the OTG adapter is pulled or plugged
			// here, external power was lost while the OTG adapter remained plugged
			// we call smb347_otg_status() now, to activate self-charging of slave devices
			// so we can continue host mode in OTG mode
			// if we would NOT call smb347_otg_status() here, slave devices would stay without power now

// tmtmtm: we don't want to call this, if OTG-adapter is pulled (not just power)
			smb347_otg_status(OTG_STATE_A_HOST,OTG_STATE_A_HOST,NULL);
		}

        if(!lastExternalPowerState) {
	        // make external power detectable in case it is coming back
	        printk("inok_isr_work_function make external power detectable1\n");
	        int ret = smb347_configure_interrupts(client);
	        if (ret < 0)
		        dev_err(&client->dev, "%s() error in configuring"
					        "otg..\n", __func__);
        }

		printk("inok_isr_work_function done lastOtgState=%d lastExternalPowerState=%d lastChargeSlaveDevicesState=%d\n",
		    lastOtgState,lastExternalPowerState,lastChargeSlaveDevicesState);
		return;
	}

	// we were NOT in externally powered host mode
	cable_type_detect();
	if(charger->cur_cable_type!=1 && charger->cur_cable_type!=3) {
		// still no power incoming
		printk("inok_isr_work_function no power lastExternalPowerState=%d\n",lastExternalPowerState);
		if(lastExternalPowerState) {
			cancel_delayed_work(&charger->curr_limit_work);
			cancel_delayed_work(&charger->inok_isr_work);
			smb347_clear_interrupts(client);

			// make device aware it is now discharging
			lastExternalPowerState = 0;
		}

        // make external power detectable
        printk("inok_isr_work_function make external power detectable2\n");
	    // 2013-01-28: crash here after
        int ret = smb347_configure_interrupts(client);
        if (ret < 0)
	        dev_err(&client->dev, "%s() error in configuring"
				        "otg..\n", __func__);
        printk("inok_isr_work_function make external power detectable2 done\n");
		return;
	}

	// power is incoming
	lastExternalPowerState = 1;

	// host_mode_charging_state may have been set by cable_type_detect()
	if(host_mode_charging_state>0 && lastOtgState==0) {
		printk("inok_isr_work_function external power available, start host mode\n");
		if(smb347_configure_otg(client, 1, 0, 0)<0)
			dev_err(&client->dev, "%s() error in configuring"
				"otg..\n", __func__);
	}

	//smb347_clear_interrupts(client);      // FIXME???
	printk("inok_isr_work_function external power available lastOtgState=%d\n",lastOtgState);
}

static void dockin_isr_work_function(struct work_struct *dat)
{
	//struct i2c_client *client = charger->client;

	int dock_in = gpio_dock_in;
	int ac_ok = GPIO_AC_OK;

	wake_lock(&charger->wake_lock_dockin);
	mutex_lock(&charger->dockin_lock);

	if (gpio_get_value(dock_in)) {
		if (!gpio_get_value(ac_ok)) {
			SMB_NOTICE("dc_in=H & ac_ok=L\n");
			cable_type_detect();
		}
	} else {
		if (!gpio_get_value(ac_ok)) {
			SMB_NOTICE("dc_in=L & ac_ok=L\n");
			msleep(40);
			cable_type_detect();
		}
	}

	mutex_unlock(&charger->dockin_lock);
	wake_unlock(&charger->wake_lock_dockin);
}

/* Sysfs function */
static ssize_t smb347_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = charger->client;
	uint8_t config_reg[14], cmd_reg[1], status_reg[10];
	int i, ret = 0;

	ret += i2c_smbus_read_i2c_block_data(client, smb347_CHARGE, 15, config_reg)
	     + i2c_smbus_read_i2c_block_data(client, smb347_CMD_REG, 2, cmd_reg)
	     + i2c_smbus_read_i2c_block_data(client, smb347_INTR_STS_A, 11, status_reg);

	if (ret < 0)
		SMB_ERR("failed to read charger reg !\n");

	SMB_INFO("smb347 Registers\n");
	SMB_INFO("------------------\n");
	for(i=0;i<=14;i++)
		SMB_INFO("Reg[%02xh]=0x%02x\n", i, config_reg[i]);
	for(i=0;i<=1;i++)
		SMB_INFO("Reg[%02xh]=0x%02x\n", 48+i, cmd_reg[i]);
	for(i=0;i<=10;i++)
		SMB_INFO("Reg[%02xh]=0x%02x\n", 53+i, status_reg[i]);

	return sprintf(buf, "Reg[06h]=0x%02x\n"
		"Reg[08h]=0x%02x\n"
		"Reg[30h]=0x%02x\n"
		"Reg[31h]=0x%02x\n"
		"Reg[39h]=0x%02x\n"
		"Reg[3dh]=0x%02x\n"
		"Reg[3eh]=0x%02x\n"
		"Reg[3fh]=0x%02x\n",
		config_reg[6],
		config_reg[8],
		cmd_reg[0],
		cmd_reg[1],
		status_reg[4],
		status_reg[8],
		status_reg[9],
		status_reg[10]);

}

static void smb347_default_setback(void)
{
	struct i2c_client *client = charger->client;
	int err;

	/* Enable volatile writes to registers */
	err = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
	if (err < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n", __func__);
	}
	err = smb347_update_reg(client, smb347_PIN_CTRL, PIN_CTRL);
	if (err < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, err);
	}
	err = smb347_update_reg(client, smb347_CHRG_CTRL, ENABLE_APSD);
	if (err < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, err);
	}
	 /* Disable volatile writes to registers */
	err = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
	if (err < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n", __func__);
	}
}

static int __devinit smb347_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int ret;
	//int irq_num;
	//uint8_t buf[15];

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	charger = kzalloc(sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	charger->client = client;
	charger->dev = &client->dev;
	i2c_set_clientdata(client, charger);

	/* Restore default setting: APSD Enable & 5/1/HC mode Pin control */
	smb347_default_setback();

	ret = sysfs_create_group(&client->dev.kobj, &smb347_group);
	if (ret) {
		dev_err(&client->dev, "smb347_probe: unable to create the sysfs\n");
	}

	mutex_init(&charger->cable_lock);
	mutex_init(&charger->dockin_lock);

	wake_lock_init(&charger->wake_lock_dockin, WAKE_LOCK_SUSPEND, "wake_lock_dockin");

	smb347_wq = create_singlethread_workqueue("smb347_wq");
	INIT_DELAYED_WORK_DEFERRABLE(&charger->inok_isr_work, inok_isr_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&charger->dockin_isr_work, dockin_isr_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&charger->cable_det_work, cable_type_detect);

	wake_lock_init(&charger_wakelock, WAKE_LOCK_SUSPEND,
			"charger_configuration");
	INIT_DELAYED_WORK(&charger->curr_limit_work,
			smb347_set_curr_limit_work_func);
	INIT_DELAYED_WORK(&charger->test_fail_clear_work,
			smb347_test_fail_clear_work_function);
	charger->curr_limit = UINT_MAX;
	smb347_set_InputCurrentlimit(charger->client, 900);
	charger->cur_cable_type = non_cable;
	charger->old_cable_type = non_cable;
	charger->test_1800mA_fail = 0;

	ret = smb347_dockin_irq(charger);
	if (ret) {
		dev_err(&client->dev, "%s(): Failed in requesting DOCK_IN# pin isr\n",
				__func__);
		goto error;
	}

	ret = smb347_inok_irq(charger);
	if (ret) {
		dev_err(&client->dev, "%s(): Failed in requesting ACOK# pin isr\n",
				__func__);
		goto error;
	}

	queue_delayed_work(smb347_wq, &charger->cable_det_work, 0.5*HZ);
	ret = register_otg_callback(smb347_otg_status, charger);
	if (ret < 0)
		goto error;

	return 0;
error:
	kfree(charger);
	return ret;
}

static int __devexit smb347_remove(struct i2c_client *client)
{
	struct smb347_charger *charger = i2c_get_clientdata(client);

	kfree(charger);
	return 0;
}

static int smb347_suspend(struct i2c_client *client)
{
	charger->suspend_ongoing = 1;
	smb347_deep_sleep = 1;  // tmtmtm

	printk("smb347_suspend+\n");
	flush_workqueue(smb347_wq);
	printk("smb347_suspend-\n");
	return 0;
}

static int smb347_resume(struct i2c_client *client)
{
	charger->suspend_ongoing = 0;

	printk("smb347_resume+\n");
	cable_type_detect();
	printk("smb347_resume-\n");
	return 0;
}


static int smb347_shutdown(struct i2c_client *client)
{
	int ret;
	printk("smb347_shutdown+\n");

	/* Disable OTG */
	ret = smb347_configure_otg(client, 0, 0, lastChargeSlaveDevicesState);
	if (ret < 0)
		dev_err(&client->dev, "%s() error in configuring"
			"otg..\n", __func__);

	/* configure charger */
	ret = smb347_configure_charger(client, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s() error in configuring"
			"otg..\n", __func__);

	printk("smb347_shutdown-\n");
	return 0;
}

static const struct i2c_device_id smb347_id[] = {
	{ "smb347", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, smb347_id);

static struct i2c_driver smb347_i2c_driver = {
	.driver	= {
		.name	= "smb347",
	},
	.probe		= smb347_probe,
	.remove		= __devexit_p(smb347_remove),
	.suspend 		= smb347_suspend,
	.resume 		= smb347_resume,
	.shutdown	= smb347_shutdown,
	.id_table	= smb347_id,
};

static int __init smb347_init(void)
{
	project_id = grouper_get_project_id();
	pcba_ver = grouper_query_pcba_revision();
	u32 project_info = grouper_get_project_id();

	if (project_info == GROUPER_PROJECT_NAKASI_3G)
		gpio_dock_in = TEGRA_GPIO_PO5;
	else
		gpio_dock_in = TEGRA_GPIO_PU4;

	SMB_NOTICE("project_id=%x, pcba_ver=%d, dock_in_gpio=%d\n",
		project_id, pcba_ver, gpio_dock_in);

	return i2c_add_driver(&smb347_i2c_driver);
}
module_init(smb347_init);

static void __exit smb347_exit(void)
{
	i2c_del_driver(&smb347_i2c_driver);
}
module_exit(smb347_exit);

MODULE_AUTHOR("Syed Rafiuddin <srafiuddin@nvidia.com>");
MODULE_DESCRIPTION("smb347 Battery-Charger");
MODULE_LICENSE("GPL");


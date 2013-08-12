/*
 * arch/arm/mach-tegra/board-grouper-sdhci.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2012 NVIDIA Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/wlan_plat.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>

#include <asm/mach-types.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/sdhci.h>

#include "gpio-names.h"
#include "board.h"
#include "board-grouper.h"

#define KAI_SD_CD	TEGRA_GPIO_PI5
#define GROUPER_WLAN_PWR	TEGRA_GPIO_PD4
#define GROUPER_WLAN_RST	TEGRA_GPIO_PD3
#define GROUPER_WLAN_WOW	TEGRA_GPIO_PO4

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;
static int grouper_wifi_status_register(void (*callback)(int, void *), void *);

static int grouper_wifi_reset(int on);
static int grouper_wifi_power(int on);
static int grouper_wifi_set_carddetect(int val);

/* Customized Locale table : OPTIONAL feature */
#define WLC_CNTRY_BUF_SZ        4
typedef struct cntry_locales_custom {
	char iso_abbrev[WLC_CNTRY_BUF_SZ];
	char custom_locale[WLC_CNTRY_BUF_SZ];
	int  custom_locale_rev;
} cntry_locales_custom_t;

static cntry_locales_custom_t grouper_wifi_translate_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement */
	{"RU", "XY", 4},
	{"IR", "XY", 4}
};

static void *grouper_wifi_get_country_code(char *ccode)
{
	int size = ARRAY_SIZE(grouper_wifi_translate_custom_table);
	int i;

	if (!ccode)
		return NULL;

	for (i = 0; i < size; i++)
		if (strcmp(ccode, grouper_wifi_translate_custom_table[i].iso_abbrev) == 0)
			return &grouper_wifi_translate_custom_table[i];
	return NULL;
}

static struct wifi_platform_data grouper_wifi_control = {
	.set_power	= grouper_wifi_power,
	.set_reset	= grouper_wifi_reset,
	.set_carddetect	= grouper_wifi_set_carddetect,
	.get_country_code = grouper_wifi_get_country_code,
};

static struct resource wifi_resource[] = {
	[0] = {
		.name	= "bcmdhd_wlan_irq",
		.start	= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PO4),
		.end	= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PO4),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};

static struct platform_device grouper_wifi_device = {
	.name		= "bcmdhd_wlan",
	.id		= 1,
	.num_resources	= 1,
	.resource	= wifi_resource,
	.dev		= {
		.platform_data = &grouper_wifi_control,
	},
};

static struct resource sdhci_resource2[] = {
	[0] = {
		.start	= INT_SDMMC3,
		.end	= INT_SDMMC3,
		.flags	= IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC3_BASE,
		.end	= TEGRA_SDMMC3_BASE + TEGRA_SDMMC3_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource sdhci_resource3[] = {
	[0] = {
		.start	= INT_SDMMC4,
		.end	= INT_SDMMC4,
		.flags	= IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC4_BASE,
		.end	= TEGRA_SDMMC4_BASE + TEGRA_SDMMC4_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct embedded_sdio_data embedded_sdio_data2 = {
	.cccr   = {
		.sdio_vsn       = 2,
		.multi_block    = 1,
		.low_speed      = 0,
		.wide_bus       = 0,
		.high_power     = 1,
		.high_speed     = 1,
	},
	.cis  = {
		.vendor         = 0x02d0,
		.device         = 0x4330,
	},
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data2 = {
	.mmc_data = {
		.register_status_notify	= grouper_wifi_status_register,
		.embedded_sdio = &embedded_sdio_data2,
		.built_in = 1,
	},
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
/*	.tap_delay = 6,
	.is_voltage_switch_supported = false,
	.vdd_rail_name = NULL,
	.slot_rail_name = NULL,
	.vdd_max_uv = -1,
	.vdd_min_uv = -1,
	.max_clk = 0,
	.is_8bit_supported = false, */
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data3 = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.is_8bit = 1,
	.tap_delay = 0x0F,
	.mmc_data = {
		.built_in = 1,
	}
/*	.tap_delay = 6,
	.is_voltage_switch_supported = false,
	.vdd_rail_name = NULL,
	.slot_rail_name = NULL,
	.vdd_max_uv = -1,
	.vdd_min_uv = -1,
	.max_clk = 48000000,
	.is_8bit_supported = true, */
};

static struct platform_device tegra_sdhci_device2 = {
	.name		= "sdhci-tegra",
	.id		= 2,
	.resource	= sdhci_resource2,
	.num_resources	= ARRAY_SIZE(sdhci_resource2),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data2,
	},
};

static struct platform_device tegra_sdhci_device3 = {
	.name		= "sdhci-tegra",
	.id		= 3,
	.resource	= sdhci_resource3,
	.num_resources	= ARRAY_SIZE(sdhci_resource3),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data3,
	},
};

static int grouper_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}
static int grouper_wifi_set_carddetect(int val)
{
	pr_debug("%s: %d\n", __func__, val);
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
	pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}

static int grouper_wifi_power(int on)
{
	pr_err("Powering %s wifi\n", (on ? "on" : "off"));

	gpio_set_value(GROUPER_WLAN_PWR, on);
	mdelay(100);
	gpio_set_value(GROUPER_WLAN_RST, on);
	mdelay(200);

	return 0;
}

static int grouper_wifi_reset(int on)
{
	pr_err("%s: do nothing\n", __func__);
	return 0;
}

static int __init grouper_wifi_init(void)
{
	int rc;

	rc = gpio_request(GROUPER_WLAN_PWR, "wlan_power");
	if (rc)
		pr_err("WLAN_PWR gpio request failed:%d\n", rc);
	rc = gpio_request(GROUPER_WLAN_RST, "wlan_rst");
	if (rc)
		pr_err("WLAN_RST gpio request failed:%d\n", rc);
	rc = gpio_request(GROUPER_WLAN_WOW, "bcmsdh_sdmmc");
	if (rc)
		pr_err("WLAN_WOW gpio request failed:%d\n", rc);

	tegra_gpio_enable(GROUPER_WLAN_PWR);
	tegra_gpio_enable(GROUPER_WLAN_RST);
	tegra_gpio_enable(GROUPER_WLAN_WOW);

	rc = gpio_direction_output(GROUPER_WLAN_PWR, 0);
	if (rc)
		pr_err("WLAN_PWR gpio direction configuration failed:%d\n", rc);
	gpio_direction_output(GROUPER_WLAN_RST, 0);
	if (rc)
		pr_err("WLAN_RST gpio direction configuration failed:%d\n", rc);
	rc = gpio_direction_input(GROUPER_WLAN_WOW);
	if (rc)
		pr_err("WLAN_WOW gpio direction configuration failed:%d\n", rc);

	platform_device_register(&grouper_wifi_device);
	return 0;
}

int __init grouper_sdhci_init(void)
{
	platform_device_register(&tegra_sdhci_device3);
	platform_device_register(&tegra_sdhci_device2);

	grouper_wifi_init();
	return 0;
}

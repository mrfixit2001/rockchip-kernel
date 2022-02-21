/* Copyright (c) 2008 -2014 Espressif System.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *
 * init , call sdio_init or spi_init
 *
 */

#include "esp_pub.h"
#include "esp_sif.h"
#include "esp_debug.h"
#include "esp_version.h"
#include "esp_file.h"
#include <linux/kthread.h>

int esp_common_init(void)
{
	int ret;
#if (defined(CONFIG_DEBUG_FS) && defined(DEBUGFS_BOOTMODE)) || defined(ESP_CLASS)
	if (sif_get_esp_run() != 0) {
		return 0;
	}
#endif
#ifdef ESP_USE_SDIO
	ret = esp_sdio_init();
#endif
#ifdef ESP_USE_SPI
	ret = esp_spi_init();
#endif
#if (defined(CONFIG_DEBUG_FS) && defined(DEBUGFS_BOOTMODE)) || defined(ESP_CLASS)
	if (ret == 0)
		sif_record_esp_run(1);
#endif
	return ret;
}

void esp_common_exit(void)
{
#if (defined(CONFIG_DEBUG_FS) && defined(DEBUGFS_BOOTMODE)) || defined(ESP_CLASS)
	if (sif_get_esp_run() == 0) {
		return;
	}
#endif
#ifdef ESP_USE_SDIO
	esp_sdio_exit();
#endif
#ifdef ESP_USE_SPI
	esp_spi_exit();
#endif
#if (defined(CONFIG_DEBUG_FS) && defined(DEBUGFS_BOOTMODE)) || defined(ESP_CLASS)
	sif_record_esp_run(0);
#endif
}

static int /*__init*/ _esp_init(void)
{
        u64 ver;
	int edf_ret = 0;

#ifdef DRIVER_VER
        ver = DRIVER_VER;
        esp_dbg(ESP_SHOW, "***** EAGLE DRIVER VER:%llx*****\n", ver);
#endif
        edf_ret = esp_debugfs_init();    /* if failed, continue */
	if (edf_ret == 0) {
#if defined(CONFIG_DEBUG_FS) && defined(DEBUGFS_BOOTMODE)
		dbgfs_bootmode_init();
#endif
		esp_dump_var("esp_msg_level", NULL, &esp_msg_level, ESP_U32);

#ifdef ESP_ANDROID_LOGGER
		esp_dump_var("log_off", NULL, &log_off, ESP_U32);
#endif /* ESP_ANDROID_LOGGER */
	}
#ifdef ESP_CLASS
	esp_class_init();
#endif
	request_init_conf();

	return esp_common_init();
}


static int wifi_init_thread(void *data)
{
	_esp_init();

	return 0;
}

#include <linux/rfkill-wlan.h>
extern int get_wifi_chip_type(void);
int esp_init(void)
{
	// MRFIXIT: don't init the driver if it's not defined in the device tree as the sdio chip
	int type = get_wifi_chip_type();
	struct task_struct *kthread = NULL;
	if(type != WIFI_ESP8089)
		return 0;

	// MRFIXIT: This driver init locks the kernel up when trying to initialize, let's fix that with a thread
	kthread = kthread_run(wifi_init_thread, NULL, "wifi_init_thread");
	if (IS_ERR(kthread))
		pr_err("create wifi_init_thread failed.\n");

	return 0;
}

static void /*__exit */ esp_exit(void)
{
	esp_debugfs_exit();
#ifdef ESP_CLASS
	esp_class_deinit();
#endif

	esp_common_exit();
}

int rockchip_wifi_init_module_esp8089(void)
{
	return esp_init();
}

void rockchip_wifi_exit_module_esp8089(void)
{
	esp_exit(); 
}

// Always lateinit this driver to avoid boot delays
#ifndef CONFIG_WL_ROCKCHIP
late_initcall(esp_init);
module_exit(esp_exit);
#else

late_initcall(rockchip_wifi_init_module_esp8089);
module_exit(rockchip_wifi_exit_module_esp8089);
#if IS_BUILTIN(CONFIG_ESP8089)
EXPORT_SYMBOL(rockchip_wifi_init_module_esp8089);
EXPORT_SYMBOL(rockchip_wifi_exit_module_esp8089);
#endif

#endif

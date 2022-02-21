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

#include <linux/module.h>
#include "esp_pub.h"
#include "esp_sif.h"
#include "esp_debug.h"
#include "esp_version.h"
#include "esp_file.h"
#include "esp_mem.h"

#include "esp_slab.h"
#include "esp_log.h"
#include "version.h"

#include <linux/kthread.h>

#define RETRY_COUNT 10

static int esp_mem_init(void)
{
	int err = 0;
	int retry;

	logi("%s VERSION [%s]\n", __func__, PREALLOC_VERSION);

#ifdef ESP_SLAB
	retry = RETRY_COUNT;
	do {
		err = esp_slab_init();
		if (err) 
			loge("%s esp_slab_init failed %d, retry %d\n", __func__, err, retry);
		else 
			break;

	} while (--retry > 0);

	if (retry <= 0)
		goto _err_slab;
#endif

#ifdef ESP_PRE_MEM
	retry = RETRY_COUNT;
	do {
		err = esp_indi_pre_mem_init();
		if (err)
			loge("%s esp_indi_pre_mem__init failed %d, retry %d\n", __func__, err, retry);
		else 
			break;

	} while (--retry > 0);

	if (retry <= 0)
		goto _err_mem;
#endif
	logi("%s complete \n", __func__);
	return 0;

#ifdef ESP_PRE_MEM
_err_mem:
#endif
#ifdef ESP_SLAB
	esp_slab_deinit();
_err_slab:
#endif
	return err;	
}

static void esp_mem_exit(void)
{
	logi("%s enter \n", __func__);
#ifdef ESP_SLAB
	esp_slab_deinit();
#endif
#ifdef ESP_PRE_MEM
	esp_indi_pre_mem_deinit();
#endif
	logi("%s complete \n", __func__);
}

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
	esp_mem_init();

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
	esp_mem_exit();

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

#ifdef CONFIG_WIFI_BUILD_MODULE
module_init(rockchip_wifi_init_module_esp8089);
module_exit(rockchip_wifi_exit_module_esp8089);
#else

// Always lateinit this driver to avoid boot delays
#ifndef CONFIG_WL_ROCKCHIP
late_initcall(esp_init);
module_exit(esp_exit);
#else
#ifdef CONFIG_WIFI_LOAD_DRIVER_WHEN_KERNEL_BOOTUP
late_initcall(rockchip_wifi_init_module_esp8089);
module_exit(rockchip_wifi_exit_module_esp8089);
#else
module_init(rockchip_wifi_init_module_esp8089);
module_exit(rockchip_wifi_exit_module_esp8089);
#endif
#if IS_BUILTIN(CONFIG_ESP8089)
EXPORT_SYMBOL(rockchip_wifi_init_module_esp8089);
EXPORT_SYMBOL(rockchip_wifi_exit_module_esp8089);
#endif
#endif

#endif

/* SPDX-License-Identifier: GPL-2.0 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/rfkill-wlan.h>

extern int get_wifi_chip_type(void);

static ssize_t wifi_chip_read(struct class *cls, struct class_attribute *attr, char *_buf)
{
	int count = 0;
	int type = get_wifi_chip_type();

	if(type == WIFI_ESP8089) {
		count = sprintf(_buf, "%s", "ESP8089");
		printk("Current WiFi chip is ESP8089.\n");
	}

	if(type == WIFI_RK901) {
		count = sprintf(_buf, "%s", "APRK901");
		printk("Current WiFi chip is APRK901.\n");
	}

	if(type == WIFI_RK903) {
		count = sprintf(_buf, "%s", "APRK903");
		printk("Current WiFi chip is APRK903.\n");
	}

	if(type == WIFI_AP6256) {
		count = sprintf(_buf, "%s", "AP6256");
		printk("Current WiFi chip is AP6256.\n");
	}

	if(type == WIFI_AP6359SA) {
		count = sprintf(_buf, "%s", "AP6359SA");
		printk("Current WiFi chip is AP6359SA.\n");
	}

	if(type == WIFI_AP6181) {
		count = sprintf(_buf, "%s", "AP6181");
		printk("Current WiFi chip is AP6181.\n");
	}

	if(type == WIFI_AP6210) {
		count = sprintf(_buf, "%s", "AP6210");
		printk("Current WiFi chip is AP6210.\n");
	}

        if(type == WIFI_AP6212) {
		count = sprintf(_buf, "%s", "AP6212");
		printk("Current WiFi chip is AP6212.\n");
        }
	
	if(type == WIFI_AP6234) {
		count = sprintf(_buf, "%s", "AP6234");
		printk("Current WiFi chip is AP6234.\n");
	}

	if (type == WIFI_AP6255) {
		count = sprintf(_buf, "%s", "AP6255");
		printk("Current WiFi chip is AP6255.\n");
	}

	if(type == WIFI_AP6330) {
		count = sprintf(_buf, "%s", "AP6330");
		printk("Current WiFi chip is AP6330.\n");
	}
	
	if(type == WIFI_AP6335) {
		count = sprintf(_buf, "%s", "AP6335");
		printk("Current WiFi chip is AP6335.\n");
	}

        if(type == WIFI_AP6354) {
		count = sprintf(_buf, "%s", "AP6354");
		printk("Current WiFi chip is AP6354.\n");
        }

	if(type == WIFI_AP6441) {
		count = sprintf(_buf, "%s", "AP6441");
		printk("Current WiFi chip is AP6441.\n");
	}

	if(type == WIFI_AP6476) {
		count = sprintf(_buf, "%s", "AP6476");
		printk("Current WiFi chip is AP6476.\n");
	}

	if(type == WIFI_AP6493) {
		count = sprintf(_buf, "%s", "AP6493");
		printk("Current WiFi chip is AP6493.\n");
	}

	if(type == WIFI_RTL8189ES) {
		count = sprintf(_buf, "%s", "RTL8189ES");
		printk("Current WiFi chip is RTL8189ES.\n");
	}

        if(type == WIFI_RTL8189FS) {
		count = sprintf(_buf, "%s", "RTL8189FS");
		printk("Current WiFi chip is RTL8189FS.\n");
        }

	if(type == WIFI_RTL8723BS) {
		count = sprintf(_buf, "%s", "RTL8723BS");
		printk("Current WiFi chip is RTL8723BS.\n");
	}

	if(type == WIFI_RTL8723CS) {
		count = sprintf(_buf, "%s", "RTL8723CS");
		printk("Current WiFi chip is RTL8723CS.\n");
	}

	if(type == WIFI_RTL8723DS) {
		count = sprintf(_buf, "%s", "RTL8723DS");
		printk("Current WiFi chip is RTL8723DS.\n");
	}

	if(type == WIFI_RTL8822BS) {
		count = sprintf(_buf, "%s", "RTL8822BS");
		printk("Current WiFi chip is RTL8822BS.\n");
	}

	if(type == WIFI_RTL8822BE) {
		count = sprintf(_buf, "%s", "RTL8822BE");
		printk("Current WiFi chip is RTL8822BE.\n");
	}

        if(type == WIFI_SSV6051) {
		count = sprintf(_buf, "%s", "SSV6051");
		printk("Current WiFi chip is SSV6051.\n");
        }
	return count;
}

static ssize_t wifi_power_write(struct class *cls, struct class_attribute *attr, const char *_buf, size_t _count)
{
    int poweren = 0;
    poweren = simple_strtol(_buf, NULL, 10);
    printk("%s: poweren = %d\n", __func__, poweren);
    if(poweren > 0) {
        rockchip_wifi_power(1);
    } else {
        rockchip_wifi_power(0);
    }

return _count;
}

extern int rockchip_wifi_init_module(void);
extern void rockchip_wifi_exit_module(void);
extern int rockchip_wifi_init_module_rkwifi(void);
extern void rockchip_wifi_exit_module_rkwifi(void);
extern int rockchip_wifi_init_module_rtkwifi(void);
extern void rockchip_wifi_exit_module_rtkwifi(void);
extern int rockchip_wifi_init_module_esp8089(void);
extern void rockchip_wifi_exit_module_esp8089(void);

static struct semaphore driver_sem;
#ifdef CONFIG_WIFI_LOAD_DRIVER_WHEN_KERNEL_BOOTUP
static int wifi_driver_insmod = 1;
#else
static int wifi_driver_insmod = 0;
#endif

static int wifi_init_exit_module(int enable)
{
	int ret = 0;
// Only call these functions when the wlan drivers are builtin, because as modules the functions cannot be 'exported' and called from here
#if !defined(CONFIG_WIFI_BUILD_MODULE) && defined(CONFIG_WIFI_LOAD_DRIVER_WHEN_KERNEL_BOOTUP)
	int type = 0;
	type = get_wifi_chip_type();
#if IS_BUILTIN(CONFIG_BCMDHD) || IS_BUILTIN(CONFIG_CYW_BCMDHD)
	if (type < WIFI_AP6XXX_SERIES) {
		if (enable > 0)
			ret = rockchip_wifi_init_module_rkwifi();
		else
			rockchip_wifi_exit_module_rkwifi();
		return ret;
	}
#endif
#if IS_BUILTIN(CONFIG_ESP8089)
	if (type == WIFI_ESP8089) {
		if (enable > 0)
			ret = rockchip_wifi_init_module_esp8089();
		else
			rockchip_wifi_exit_module_esp8089();
		return ret;

	}
#endif
#if 	IS_BUILTIN(CONFIG_RTL8189ES) || IS_BUILTIN(CONFIG_RTL8189FS) || IS_BUILTIN(CONFIG_RTL8723BS) || \
	IS_BUILTIN(CONFIG_RTL8723CS) || IS_BUILTIN(CONFIG_RTL8723DS) || IS_BUILTIN(CONFIG_RTL8822BS) || \
	IS_BUILTIN(CONFIG_RTL8822BE)
	if (type > WIFI_ESP8089 && type < WIFI_RTL_SERIES) {
		if (enable > 0)
			ret = rockchip_wifi_init_module_rtkwifi();
		else
			rockchip_wifi_exit_module_rtkwifi();
		return ret;
	}
#endif
#if IS_BUILTIN(CONFIG_SSV6051)
	if (type == WIFI_SSV6051) {
		// Not yet supported
		//if (enable > 0)
		//	ret = rockchip_wifi_init_module_rtkwifi();
		//else
		//	rockchip_wifi_exit_module_rtkwifi();
		//return ret;
		return 0;
	}
#endif
#endif
	return ret;
}

static ssize_t wifi_driver_write(struct class *cls, struct class_attribute *attr, const char *_buf, size_t _count)
{
    int enable = 0, ret = 0;
    
    down(&driver_sem);
    enable = simple_strtol(_buf, NULL, 10);
    //printk("%s: enable = %d\n", __func__, enable);
    if (wifi_driver_insmod == enable) {
        printk("%s: wifi driver already %s\n", __func__, enable? "insmod":"rmmod");
    	up(&driver_sem);
        return _count;
    }
    if(enable > 0) {
        ret = wifi_init_exit_module(enable);
        if (ret >= 0)
            wifi_driver_insmod = enable;
    } else {
        wifi_init_exit_module(enable);
        wifi_driver_insmod = enable;
    }   

    up(&driver_sem);
    //printk("%s: ret = %d\n", __func__, ret);
    return _count; 
}

static struct class *rkwifi_class = NULL;
static CLASS_ATTR(chip, 0664, wifi_chip_read, NULL);
static CLASS_ATTR(power, 0660, NULL, wifi_power_write);
static CLASS_ATTR(driver, 0660, NULL, wifi_driver_write);
#ifdef CONFIG_WIFI_LOAD_DRIVER_WHEN_KERNEL_BOOTUP
static CLASS_ATTR(preload, 0660, NULL, NULL);
#endif

int rkwifi_sysif_init(void)
{
    int ret;
    
    printk("Rockchip WiFi SYS interface (V1.00) ... \n");
    
    rkwifi_class = NULL;
    
    rkwifi_class = class_create(THIS_MODULE, "rkwifi");
    if (IS_ERR(rkwifi_class)) 
    {   
        printk("Create class rkwifi_class failed.\n");
        return -ENOMEM;
    }
    
    ret =  class_create_file(rkwifi_class, &class_attr_chip);
    ret =  class_create_file(rkwifi_class, &class_attr_power);
    ret =  class_create_file(rkwifi_class, &class_attr_driver);
#ifdef CONFIG_WIFI_LOAD_DRIVER_WHEN_KERNEL_BOOTUP
    ret =  class_create_file(rkwifi_class, &class_attr_preload);
#endif
    sema_init(&driver_sem, 1);
    
    return 0;
}

void rkwifi_sysif_exit(void)
{
    // need to remove the sys files and class
    class_remove_file(rkwifi_class, &class_attr_chip);
    class_remove_file(rkwifi_class, &class_attr_power);
    class_remove_file(rkwifi_class, &class_attr_driver);
#ifdef CONFIG_WIFI_LOAD_DRIVER_WHEN_KERNEL_BOOTUP
    class_remove_file(rkwifi_class, &class_attr_preload);
#endif
    class_destroy(rkwifi_class);
    
    rkwifi_class = NULL;
}

module_init(rkwifi_sysif_init);
module_exit(rkwifi_sysif_exit);

MODULE_AUTHOR("Yongle Lai & gwl");
MODULE_DESCRIPTION("WiFi SYS @ Rockchip");
MODULE_LICENSE("GPL");
MODULE_SOFTDEP("pre: wlan-platdata");

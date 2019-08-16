/*
 * LED Disk Activity Inverted Trigger - LED stays ON when no activity
 *
 * Copyright 2006 Openedhand Ltd.
 *
 * Author: Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/leds.h>
#include "../leds.h"

#define BLINK_DELAY 30

DEFINE_LED_TRIGGER(ledtrig_disk_invert);
DEFINE_LED_TRIGGER(ledtrig_ide_invert);

void ledtrig_disk_activity_inverted(void)
{
	unsigned long blink_delay = BLINK_DELAY;

	led_trigger_blink_oneshot(ledtrig_disk_invert,
				  &blink_delay, &blink_delay, 1);
}
EXPORT_SYMBOL(ledtrig_disk_activity_inverted);

static int __init ledtrig_disk_inverted_init(void)
{
	led_trigger_register_simple("disk-activity-inverted", &ledtrig_disk_invert);
	led_trigger_register_simple("ide-disk-inverted", &ledtrig_ide_invert);

	return 0;
}

static void __exit ledtrig_disk_inverted_exit(void)
{
	led_trigger_unregister_simple(ledtrig_disk_invert);
	led_trigger_unregister_simple(ledtrig_ide_invert);
}

module_init(ledtrig_disk_inverted_init);

MODULE_AUTHOR("Richard Purdie <rpurdie@openedhand.com>");
MODULE_DESCRIPTION("LED Disk Activity Trigger");
MODULE_LICENSE("GPL");

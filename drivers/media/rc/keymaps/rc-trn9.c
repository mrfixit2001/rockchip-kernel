/* Keytable for T-Chip TRN9 IR Remote Controller
 *
 * Copyright (c) 2018 Omegamoon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <media/rc-map.h>
#include <linux/module.h>

static struct rc_map_table trn9[] = {
	{ 0x0014, KEY_POWER },
	{ 0x0013, KEY_MENU },
	{ 0x0003, KEY_UP },
	{ 0x0002, KEY_DOWN },
	{ 0x000e, KEY_LEFT },
	{ 0x001a, KEY_RIGHT },
	{ 0x0007, KEY_OK },
	{ 0x0058, KEY_VOLUMEDOWN },
	{ 0x005c, KEY_MUTE },
	{ 0x000b, KEY_VOLUMEUP },
	{ 0x0001, KEY_BACK },
	{ 0x0048, KEY_HOME },
};

static struct rc_map_list trn9_map = {
	.map = {
		.scan    = trn9,
		.size    = ARRAY_SIZE(trn9),
		.rc_type = RC_TYPE_NEC,
		.name    = RC_MAP_TRN9,
	}
};

static int __init init_rc_map_trn9(void)
{
	return rc_map_register(&trn9_map);
}

static void __exit exit_rc_map_trn9(void)
{
	rc_map_unregister(&trn9_map);
}

module_init(init_rc_map_trn9)
module_exit(exit_rc_map_trn9)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Omegamoon");

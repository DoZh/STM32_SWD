/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements a basic command interpreter for GDB 'monitor'
 * commands.
 */

#include "general.h"
#include "exception.h"
#include "command.h"
#include "gdb_packet.h"
#include "target.h"
#include "morse.h"



typedef bool (*cmd_handler)(target *t, int argc, const char **argv);

struct command_s {
	const char *cmd;
	cmd_handler handler;

	const char *help;
};

//static bool cmd_swdp_scan(void);
static bool cmd_targets(void);

static bool cmd_connect_srst(target *t, int argc, const char **argv);
static bool cmd_hard_srst(void);
#ifdef PLATFORM_HAS_POWER_SWITCH
static bool cmd_target_power(target *t, int argc, const char **argv);
#endif

#ifdef PLATFORM_HAS_DEBUG
static bool cmd_debug_bmp(target *t, int argc, const char **argv);
#endif

const struct command_s cmd_list[] = {
	{"swdp_scan", (cmd_handler)cmd_swdp_scan, "Scan SW-DP for devices" },
	{"targets", (cmd_handler)cmd_targets, "Display list of available targets" },
	{"connect_srst", (cmd_handler)cmd_connect_srst, "Configure connect under SRST: (enable|disable)" },
	{"hard_srst", (cmd_handler)cmd_hard_srst, "Force a pulse on the hard SRST line - disconnects target" },
#ifdef PLATFORM_HAS_POWER_SWITCH
	{"tpwr", (cmd_handler)cmd_target_power, "Supplies power to the target: (enable|disable)"},
#endif
#ifdef PLATFORM_HAS_DEBUG
	{"debug_bmp", (cmd_handler)cmd_debug_bmp, "Output BMP \"debug\" strings to the second vcom: (enable|disable)"},
#endif
	{NULL, NULL, NULL}
};

static bool connect_assert_srst;
#ifdef PLATFORM_HAS_DEBUG
bool debug_bmp;
#endif


bool cmd_swdp_scan(void)
{
//	printf("Target voltage: %s\n", platform_target_voltage());
	printf("Target voltage: %s\n", "!!!Void!!!");
	if(connect_assert_srst)
		platform_srst_set_val(true); /* will be deasserted after attach */

	int devs = -1;
	volatile struct exception e;
	TRY_CATCH (e, EXCEPTION_ALL) {
		devs = adiv5_swdp_scan();
	}
	switch (e.type) {
	case EXCEPTION_TIMEOUT:
		printf("Timeout during scan. Is target stuck in WFI?\n");
		break;
	case EXCEPTION_ERROR:
		printf("Exception: %s\n", e.msg);
		break;
	}

	if(devs <= 0) {
		platform_srst_set_val(false);
		printf("SW-DP scan failed!\n");
		return false;
	}

	cmd_targets();
	//morse(NULL, false);
	return true;

}

static void display_target(int i, target *t, void *context)
{
	(void)context;
	printf("%2d   %c  %s\n", i, target_attached(t)?'*':' ', target_driver_name(t));
}

bool cmd_targets(void)
{
	printf("Available Targets:\n");
	printf("No. Att Driver\n");
	if (!target_foreach(display_target, NULL)) {
		printf("No usable targets found.\n");
		return false;
	}

	return true;
}



static bool cmd_connect_srst(target *t, int argc, const char **argv)
{
	(void)t;
	if (argc == 1)
		printf("Assert SRST during connect: %s\n",
			 connect_assert_srst ? "enabled" : "disabled");
	else
		connect_assert_srst = !strcmp(argv[1], "enable");
	return true;
}

static bool cmd_hard_srst(void)
{
	target_list_free();
	platform_srst_set_val(true);
	platform_srst_set_val(false);
	return true;
}

#ifdef PLATFORM_HAS_POWER_SWITCH
static bool cmd_target_power(target *t, int argc, const char **argv)
{
	(void)t;
	if (argc == 1)
		printf("Target Power: %s\n",
			 platform_target_get_power() ? "enabled" : "disabled");
	else
		platform_target_set_power(!strncmp(argv[1], "enable", strlen(argv[1])));
	return true;
}
#endif



#ifdef PLATFORM_HAS_DEBUG
static bool cmd_debug_bmp(target *t, int argc, const char **argv)
{
	(void)t;
	if (argc > 1) {
		debug_bmp = !strcmp(argv[1], "enable");
	}
	printf("Debug mode is %s\n",
		 debug_bmp ? "enabled" : "disabled");
	return true;
}
#endif

/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2015 Gareth McMullin <gareth@blacksphere.co.nz>
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
#ifndef __TIMING_STM32_H
#define __TIMING_STM32_H


#define SWD_DELAY_PRE_SAMPLING 100
#define SWD_DELAY_POST_SAMPLING 100

#include "stdint.h"

struct platform_timeout {
	uint32_t time;
};

uint32_t platform_time_ms(void);

extern uint8_t running_status;

void platform_timing_init(void);

void platform_delay_swd(uint32_t us);

#endif


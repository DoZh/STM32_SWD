/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2015  Black Sphere Technologies Ltd.
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
#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f4xx_hal.h"

#define INLINE_GPIO

#define gpio_set_val(port, pin, val) do {	\
	if(val)					\
		gpio_set((port), (pin));	\
	else					\
		gpio_clear((port), (pin));	\
} while(0)

#ifdef INLINE_GPIO
static inline void _gpio_set(GPIO_TypeDef* gpioport, uint16_t gpios)
{
	gpioport->BSRR = gpios;
}
#define gpio_set _gpio_set

static inline void _gpio_clear(GPIO_TypeDef* gpioport, uint16_t gpios)
{
	gpioport->BSRR = (uint32_t)gpios << 16U;
}
#define gpio_clear _gpio_clear

static inline uint16_t _gpio_get(GPIO_TypeDef* gpioport, uint16_t gpios)
{
	uint16_t bitstatus;
	
	if((gpioport->IDR & gpios) != (uint32_t)GPIO_PIN_RESET)
  {
    bitstatus = 1;
  }
  else
  {
    bitstatus = 0;
  }
  return bitstatus;
}
#define gpio_get _gpio_get
#endif

void gpio_mode_setup(GPIO_TypeDef* gpioport, uint8_t mode, uint8_t pull_up_down,
		     uint16_t gpios);

#endif


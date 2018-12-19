/*
 *  Copyright (C) 2016, Loongson Technology Corporation Limited, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#ifndef __LS2K_GPIO_H__
#define __LS2K_GPIO_H__

#include <ls2k.h>

#undef GPIO_DEBUG

#define LS2K_GPIO_MAX 16

#define readl(addr) (*(volatile unsigned int *)CKSEG1ADDR(addr))
#define writel(val,addr) *(volatile unsigned int *)CKSEG1ADDR(addr) = (val)

static const char *ls2k_gpio_list[LS2K_GPIO_MAX];

static int ls2k_gpio_request(unsigned gpio, const char *label)
{
	if (gpio >= LS2K_GPIO_MAX)
		return -EINVAL;

	if (ls2k_gpio_list[gpio])
		return -EBUSY;

	if (label)
		ls2k_gpio_list[gpio] = label;
	else
		ls2k_gpio_list[gpio] = "busy";

	return 0;
}

static void ls2k_gpio_free(unsigned gpio)
{
	ls2k_gpio_list[gpio] = NULL;
}

static inline int ls2k_gpio_get_value(unsigned gpio)
{
	unsigned long gpio_in = LS2K_GPIO_IN_REG;

	return readl(gpio_in) & (1 << gpio);
}
static inline void ls2k_gpio_set_value(unsigned gpio, int value)
{
	unsigned long gpio_out = LS2K_GPIO_OUT_REG;
	unsigned tmp;

	tmp = readl(gpio_out) & ~(1 << gpio);
	if (value)
		tmp |= 1 << gpio;
	writel(tmp, gpio_out);
}

static inline int ls2k_gpio_direction_input(unsigned gpio)
{
	unsigned long gpio_dir = LS2K_GPIO_OE_REG;

	if (gpio >= LS2K_GPIO_MAX)
		return -EINVAL;

	writel(readl(gpio_dir) | (1 << gpio), gpio_dir);

	return 0;
}

static inline int ls2k_gpio_direction_output(unsigned gpio, int value)
{
	unsigned long gpio_dir = LS2K_GPIO_OE_REG;

	if (gpio >= LS2K_GPIO_MAX)
		return -EINVAL;

	gpio_set_value(gpio, value);
	writel(readl(gpio_dir) & ~(1 << gpio), gpio_dir);

	return 0;
}

static inline int ls2k_gpio_to_irq(unsigned gpio)
{
	return -EINVAL;
}

static inline int ls2k_irq_to_gpio(unsigned irq)
{
	return -EINVAL;
}

#endif

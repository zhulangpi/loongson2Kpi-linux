/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __LS2K_GPIO_H
#define __LS2K_GPIO_H

#ifndef gpio_to_irq
#define gpio_to_irq				__gpio_to_irq
#endif

static inline int irq_to_gpio(unsigned irq)
{
	return -EINVAL;
}

#include <asm-generic/gpio.h>

enum ls2k_gpio {

	LS2K_GPIO_PIN_0,
	LS2K_GPIO_PIN_1,
	LS2K_GPIO_PIN_2,
	LS2K_GPIO_PIN_3,
	LS2K_GPIO_PIN_4,
	LS2K_GPIO_PIN_5,
	LS2K_GPIO_PIN_6,
	LS2K_GPIO_PIN_7,
	LS2K_GPIO_PIN_8,
	LS2K_GPIO_PIN_9,
	LS2K_GPIO_PIN_10,
	LS2K_GPIO_PIN_11,
	LS2K_GPIO_PIN_12,
	LS2K_GPIO_PIN_13,
	LS2K_GPIO_PIN_14,

	LS2K_GPIO_PIN_16 = 16,
	LS2K_GPIO_PIN_17,
	LS2K_GPIO_PIN_18,
	LS2K_GPIO_PIN_19,
	LS2K_GPIO_PIN_20,
	LS2K_GPIO_PIN_21,
	LS2K_GPIO_PIN_22,
	LS2K_GPIO_PIN_23,
	LS2K_GPIO_PIN_24,
	LS2K_GPIO_PIN_25,
	LS2K_GPIO_PIN_26,
	LS2K_GPIO_PIN_27,
	LS2K_GPIO_PIN_28,
	LS2K_GPIO_PIN_29,
	LS2K_GPIO_PIN_30,
	LS2K_GPIO_PIN_31,

	LS2K_GPIO_PIN_32 = 32,
	LS2K_GPIO_PIN_33,
	LS2K_GPIO_PIN_34,
	LS2K_GPIO_PIN_35,
	LS2K_GPIO_PIN_36,
	LS2K_GPIO_PIN_37,
	LS2K_GPIO_PIN_38,
	LS2K_GPIO_PIN_39,
	LS2K_GPIO_PIN_40,
	LS2K_GPIO_PIN_41,

	LS2K_GPIO_PIN_44 = 44 ,
	LS2K_GPIO_PIN_45,
	LS2K_GPIO_PIN_46,
	LS2K_GPIO_PIN_47,
	LS2K_GPIO_PIN_48,
	LS2K_GPIO_PIN_49,
	LS2K_GPIO_PIN_50,
	LS2K_GPIO_PIN_51,
	LS2K_GPIO_PIN_52,
	LS2K_GPIO_PIN_53,
	LS2K_GPIO_PIN_54,
	LS2K_GPIO_PIN_55,
	LS2K_GPIO_PIN_56,
	LS2K_GPIO_PIN_57,
	LS2K_GPIO_PIN_58,
	LS2K_GPIO_PIN_59,
	LS2K_GPIO_PIN_60,
	LS2K_GPIO_PIN_61,
	LS2K_GPIO_PIN_62,
	LS2K_GPIO_PIN_63,

};


int gpio_get_value(unsigned gpio);
void gpio_set_value(unsigned int gpio, int value);
#endif				

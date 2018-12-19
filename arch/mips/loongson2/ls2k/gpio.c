/*
 * =====================================================================================
 *
 *       Filename:  gpio.c
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  04/15/2017 10:37:08 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  hp (Huang Pei), huangpei@loongson.cn
 *        Company:  Loongson Corp.
 *
 * =====================================================================================
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/err.h>
#include <asm/types.h>
#include <loongson.h>
#include <linux/gpio.h>
#include <asm/gpio.h>

#define LS2K_GPIO_MAX 64


#define ls2k_readq(addr) (*(volatile unsigned long *)CKSEG1ADDR(addr))
#define ls2k_writeq(val,addr) *(volatile unsigned long *)CKSEG1ADDR(addr) = (val)

int gpio_get_value(unsigned gpio)
{
	unsigned long gpio_in = LS2K_GPIO_IN_REG;

	return (ls2k_readq(gpio_in) >> gpio) & 1;
}
EXPORT_SYMBOL(gpio_get_value);

void gpio_set_value(unsigned gpio, int value)
{
	unsigned long gpio_out = LS2K_GPIO_OUT_REG;
	unsigned long tmp;

	tmp = ls2k_readq(gpio_out) & ~(1ULL << gpio);
	if (value)
		tmp |= 1ULL << gpio;
	ls2k_writeq(tmp, gpio_out);
}
EXPORT_SYMBOL(gpio_set_value);

static const char *ls2k_gpio_list[LS2K_GPIO_MAX];

static int ls2k_gpio_request(struct gpio_chip *chip,  unsigned gpio)
{
	if (gpio >= LS2K_GPIO_MAX)
		return -EINVAL;

	return 0;
}

static void ls2k_gpio_free(struct gpio_chip *chip, unsigned gpio)
{
	ls2k_gpio_list[gpio] = NULL;
}

static inline int ls2k_gpio_get_value(struct gpio_chip *chip, unsigned gpio)
{
	unsigned long gpio_in = LS2K_GPIO_IN_REG;

	return (ls2k_readq(gpio_in) >> gpio) & 1;
}

static inline void ls2k_gpio_set_value(struct gpio_chip *chip, unsigned gpio, int value)
{
	unsigned long gpio_out = LS2K_GPIO_OUT_REG;
	unsigned long tmp;

	tmp = ls2k_readq(gpio_out) & ~(1ULL << gpio);
	if (value)
		tmp |= 1ULL << gpio;
	ls2k_writeq(tmp, gpio_out);
}

static inline int ls2k_gpio_direction_input(struct gpio_chip *chip, unsigned gpio)
{
	unsigned long gpio_dir = LS2K_GPIO_OE_REG;

	if (gpio >= LS2K_GPIO_MAX)
		return -EINVAL;

	ls2k_writeq(ls2k_readq(gpio_dir) | (1ULL << gpio), gpio_dir);

	return 0;
}

static inline int ls2k_gpio_direction_output(struct gpio_chip *chip, unsigned gpio, int value)
{
	unsigned long gpio_dir = LS2K_GPIO_OE_REG;

	if (gpio >= LS2K_GPIO_MAX)
		return -EINVAL;

	ls2k_gpio_set_value(chip, gpio, value);
	ls2k_writeq(ls2k_readq(gpio_dir) & ~(1ULL << gpio), gpio_dir);

	return 0;
}

static struct gpio_chip ls2k_chip = {
	.label			= "ls2k",
	.free			= ls2k_gpio_free,
	.request		= ls2k_gpio_request,
	.direction_input	= ls2k_gpio_direction_input,
	.get			= ls2k_gpio_get_value,
	.direction_output	= ls2k_gpio_direction_output,
	.set			= ls2k_gpio_set_value,
	.base			= 0,
	.ngpio			= 64,
};

static int __init ls2k_gpio_setup(void)
{
	return gpiochip_add(&ls2k_chip);
}
arch_initcall(ls2k_gpio_setup);

/*
 * Copyright (C) 2009 Lemote, Inc.
 * Author: Wu Zhangjin <wuzhangjin@gmail.com>
 *
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __ASM_MACH_LOONGSON3_LOONGSON_H
#define __ASM_MACH_LOONGSON3_LOONGSON_H

#include <linux/io.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/kconfig.h>
#include <boot_param.h>

/* machine-specific reboot/halt operation */
extern void mach_prepare_reboot(void);
extern void mach_prepare_shutdown(void);

/* environment arguments from bootloader */
extern u32 cpu_clock_freq;
extern u32 memsize, highmemsize;

/* loongson-specific command line, env and memory initialization */
extern void __init prom_init_memory(void);
extern void __init prom_init_cmdline(void);
extern void __init prom_init_machtype(void);
extern void __init prom_init_env(void);

/* irq operation functions */
extern void mach_irq_dispatch(unsigned int pending);


#include <linux/interrupt.h>

#define LOONGSON_FLASH_BASE	0x1c000000
#define LOONGSON_FLASH_SIZE	0x02000000	/* 32M */
#define LOONGSON_FLASH_TOP	(LOONGSON_FLASH_BASE+LOONGSON_FLASH_SIZE-1)

#define LOONGSON_LIO0_BASE	0x1e000000
#define LOONGSON_LIO0_SIZE	0x01C00000	/* 28M */
#define LOONGSON_LIO0_TOP	(LOONGSON_LIO0_BASE+LOONGSON_LIO0_SIZE-1)

#define LOONGSON_BOOT_BASE	0x1fc00000
#define LOONGSON_BOOT_SIZE	0x00100000	/* 1M */
#define LOONGSON_BOOT_TOP	(LOONGSON_BOOT_BASE+LOONGSON_BOOT_SIZE-1)

#define LOONGSON_PCIIO_BASE	0x18000000
#define LOONGSON_PCIIO_SIZE	0x02000000	/* 1M */

#define IO_base_regs_addr	((int)0xbfe10000)
#define INT_router_regs_uart0			*(volatile unsigned char *)(IO_base_regs_addr + 0x1400)
#define INT_router_regs_gmac0			*(volatile unsigned char *)(IO_base_regs_addr + 0x140c)
#define INT_router_regs_gpu			*(volatile unsigned char *)(IO_base_regs_addr + 0x141d)
#define INT_router_regs_ahci			*(volatile unsigned char *)(IO_base_regs_addr + 0x1413)
#define INT_router_regs_ohci			*(volatile unsigned char *)(IO_base_regs_addr + 0x1453)


#define IO_control_regs_Intisr			*(volatile unsigned int *)(IO_base_regs_addr + 0x1420)
#define IO_control_regs_Inten			*(volatile unsigned int *)(IO_base_regs_addr + 0x1424)
#define IO_control_regs_Intenset		*(volatile unsigned int *)(IO_base_regs_addr + 0x1428)
#define IO_control_regs_Intenclr		*(volatile unsigned int *)(IO_base_regs_addr + 0x142c)
#define IO_control_regs_Intedge			*(volatile unsigned int *)(IO_base_regs_addr + 0x1434)
#define IO_control_regs_Intisr_hi		*(volatile unsigned int *)(IO_base_regs_addr + 0x1460)
#define IO_control_regs_Inten_hi		*(volatile unsigned int *)(IO_base_regs_addr + 0x1464)
#define IO_control_regs_Intenset_hi		*(volatile unsigned int *)(IO_base_regs_addr + 0x1468)
#define IO_control_regs_Intenclr_hi		*(volatile unsigned int *)(IO_base_regs_addr + 0x146c)
#define IO_control_regs_Intedge_hi		*(volatile unsigned int *)(IO_base_regs_addr + 0x1474)
#define IO_control_regs_CORE0_INTISR		*(volatile unsigned int *)(IO_base_regs_addr + 0x1040)
#define IO_control_regs_CORE0_INTISR_HI		*(volatile unsigned int *)(IO_base_regs_addr + 0x1048)
#define IO_control_regs_CORE1_INTISR		*(volatile unsigned int *)(IO_base_regs_addr + 0x1140)
#define IO_control_regs_CORE1_INTISR_HI		*(volatile unsigned int *)(IO_base_regs_addr + 0x1148)

struct ls2k_int_ctrl_regs {
	volatile unsigned char int_entry[32];
	volatile unsigned int int_isr;
	volatile unsigned int int_en;
	volatile unsigned int int_set;
	volatile unsigned int int_clr;
	volatile unsigned int int_pol;
	volatile unsigned int int_edge;
	volatile unsigned int int_bounce;
	volatile unsigned int int_auto;
};

enum {
UART0_3,
UART4_7,
};



void loongson_suspend_lowlevel(void);


#define CONF_BASE		(int)(0xbfe10000)
#define LS2K_GPIO_OE_REG	(CONF_BASE + 0x0500)
#define LS2K_GPIO_OUT_REG	(CONF_BASE + 0x0510)
#define LS2K_GPIO_IN_REG	(CONF_BASE + 0x0520)
#define LS2K_GPIO_INT_REG	(CONF_BASE + 0x0530)


#endif /* __ASM_MACH_LOONGSON3_LOONGSON_H */

/*
 * Copyright (C) 2007 Lemote Inc. & Insititute of Computing Technology
 * Author: Fuxin Zhang, zhangfx@lemote.com
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 */
#include <linux/module.h>

#include <asm/wbflush.h>

#include <loongson.h>
#include <linux/spinlock.h>
DEFINE_SPINLOCK(ls2k_io_lock);
EXPORT_SYMBOL(ls2k_io_lock);

#ifdef CONFIG_VT
#include <linux/console.h>
#include <linux/screen_info.h>
#endif

static void wbflush_loongson(void)
{
	asm(".set\tpush\n\t"
	    ".set\tnoreorder\n\t"
	    ".set mips3\n\t"
	    "sync\n\t"
	    "nop\n\t"
	    ".set\tpop\n\t"
	    ".set mips0\n\t");
}

void (*__wbflush)(void) = wbflush_loongson;
EXPORT_SYMBOL(__wbflush);

void __init plat_mem_setup(void)
{
#ifdef CONFIG_DMA_NONCOHERENT
	unsigned long val;
	unsigned long addr = CKSEG1ADDR(0x1fe10000);

	val = readq((void*)(addr + 0x0420));
	val &= 0xffffff8fffffffe; //pcie, usb, hda, gmac
	writeq(val, (void *)(addr + 0x0420));

	val = readq((void*)(addr + 0x0430));
	val &= 0xffffffffffffff3; //dc, gpu
	writeq(val, (void *)(addr + 0x0430));

	val = readq((void *)(addr + 0x0450));
	val &= 0xffffffffffffbff; //sata
	writeq(val, (void *)(addr + 0x0450));
#endif

	


#ifdef CONFIG_VT
#if defined(CONFIG_VGA_CONSOLE)
	conswitchp = &vga_con;

	screen_info = (struct screen_info) {
		.orig_x			= 0,
		.orig_y			= 25,
		.orig_video_cols	= 80,
		.orig_video_lines	= 25,
		.orig_video_isVGA	= VIDEO_TYPE_VGAC,
		.orig_video_points	= 16,
	};
#elif defined(CONFIG_DUMMY_CONSOLE)
	conswitchp = &dummy_con;
#endif
#endif
}

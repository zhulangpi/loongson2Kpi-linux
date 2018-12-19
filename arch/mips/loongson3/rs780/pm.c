/*
*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/suspend.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/cpu.h>
#include <linux/i8042.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/delay.h>

#include <asm/i8259.h>
#include <asm/mipsregs.h>
#include <asm/bootinfo.h>
#include <asm/tlbflush.h>

#include <loongson.h>
extern void prom_printf(char *fmt, ...);
extern void acpi_sleep_prepare(void);
extern void acpi_sleep_complete(void);
extern void acpi_registers_setup(void);
extern void rs780_irq_router_init(void);
void mach_suspend_rs780(suspend_state_t state)
{
	if (state == PM_SUSPEND_MEM) {
		acpi_sleep_prepare();
	}
}

void mach_resume_rs780(suspend_state_t state)
{
	if (state == PM_SUSPEND_MEM) {
		rs780_irq_router_init();
		acpi_registers_setup();
		acpi_sleep_complete();
	}
}

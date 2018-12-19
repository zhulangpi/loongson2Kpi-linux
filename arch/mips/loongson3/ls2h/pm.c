/*
*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/suspend.h>
#include <linux/pm.h>
#include <linux/cpu.h>

void setup_wakeup_events(void)
{
}

int wakeup_loongson(void)
{
	return 0;
}
void mach_suspend_ls2h(suspend_state_t state)
{

}
extern void ls2h_irq_router_init(void);
extern void ls2h_early_config(void);

void mach_resume_ls2h(suspend_state_t state)
{
	if (state == PM_SUSPEND_MEM) {
		ls2h_early_config();
		ls2h_irq_router_init();
	}
}

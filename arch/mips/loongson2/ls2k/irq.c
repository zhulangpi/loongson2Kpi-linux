/*
 *  Copyright (C) 2013, Loongson Technology Corporation Limited, Inc.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 */
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/irq_cpu.h>
#include <asm/mipsregs.h>
#include <asm/delay.h>
#include <boot_param.h>
#include <linux/cpumask.h>
#include <irq.h>
#include <loongson.h>
#include <ls2k_int.h>
#include <ls2k.h>

extern void loongson3_ipi_interrupt(struct pt_regs *regs);
extern void _ls2k_init_irq(u32 irq_base);
extern void _ls2k_irq_dispatch(void);
extern unsigned long long lpc_reg_base;


void __init ls2k_init_irq(void)
{
	clear_c0_status(ST0_IM | ST0_BEV);
	local_irq_disable();

	_ls2k_init_irq(LS2K_IRQ_BASE);

#if 1
	{
		volatile unsigned char *p = (unsigned char *)(IO_base_regs_addr+0x1400);
		int i;
		for(i=0;i<0x20;i++)
		p[i] = 0x11;

		for(i=0x40;i<0x60;i++)
		p[i] = 0x11;
	}
#endif

	INT_router_regs_uart0 = 0x11;    //uart0 IP2

	INT_router_regs_gmac0 = 0x81;     //IP5

	INT_router_regs_gpu = 0x81;    //IP5

	INT_router_regs_ahci = 0x81;     //IP5

	INT_router_regs_ohci = 0x21;     //IP3

}

asmlinkage void plat_irq_dispatch(void)
{
	/*
	 * CPU interrupts used on Loongson3A-2H:
	 *
	 *	0 - Software interrupt 0 (unused)
	 *	1 - Software interrupt 1 (unused)
	 *	2 - CPU UART & LPC
	 *	3 - Loongson2H South Bridge
	 *	6 - IPI interrupt
	 *	7 - Timer and Performance Counter
	 */
	unsigned int pending;

	pending = read_c0_cause() & read_c0_status() & ST0_IM;
	if (!pending)
		return;

	if (pending & CAUSEF_IP7) {
		do_IRQ(MIPS_CPU_IRQ_BASE + 7);
	}
#ifdef CONFIG_SMP
	if (pending & CAUSEF_IP6) {
		loongson3_ipi_interrupt(NULL);
	}
#endif
	if (pending & CAUSEF_IP2) {
		_ls2k_irq_dispatch();
	}
	if (pending & CAUSEF_IP3) {
		_ls2k_irq_dispatch();
	}
	if (pending & CAUSEF_IP5) {
		_ls2k_irq_dispatch();
	}
	if (pending & (~(CAUSEF_IP7 | CAUSEF_IP6 | CAUSEF_IP2 | CAUSEF_IP3 | CAUSEF_IP5))) {
		spurious_interrupt();
	}
}

struct irqaction cascade_irqaction = {
	.handler = no_action,
	.name = "cascade",
	.flags = IRQF_NO_THREAD,
};

void __init arch_init_irq(void)
{
	/*
	 * Clear all of the interrupts while we change the able around a bit.
	 * int-handler is not on bootstrap
	 */
	clear_c0_status(ST0_IM | ST0_BEV);


	mips_cpu_irq_init();
	/* machine specific irq init */
	ls2k_init_irq();

	set_c0_status(STATUSF_IP2 | STATUSF_IP3 | STATUSF_IP6);

	setup_irq(MIPS_CPU_IRQ_BASE + 2, &cascade_irqaction);
	setup_irq(MIPS_CPU_IRQ_BASE + 3, &cascade_irqaction);
	setup_irq(MIPS_CPU_IRQ_BASE + 5, &cascade_irqaction);
}

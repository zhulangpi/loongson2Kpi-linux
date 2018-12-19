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
#include <asm/smp.h>
#include <irq.h>
#include <loongson.h>
#include <ls2h/ls2h_int.h>
#include <ls2h/ls2h.h>

extern void loongson3_ipi_interrupt(struct pt_regs *regs);
extern void _ls2h_init_irq(u32 irq_base);
extern void _ls2h_irq_dispatch(void);
extern unsigned long long lpc_reg_base;

DEFINE_SPINLOCK(ls3_lpc_lock);

static void disable_lpc_irq(struct irq_data *d)
{
	unsigned long flags;

	spin_lock_irqsave(&ls3_lpc_lock, flags);
	ls2h_readl(lpc_reg_base + 0x4) &= ~(0x1 << (d->irq));
	spin_unlock_irqrestore(&ls3_lpc_lock, flags);
}

static void enable_lpc_irq(struct irq_data *d)
{
	unsigned long flags;

	spin_lock_irqsave(&ls3_lpc_lock, flags);

	ls2h_readl(lpc_reg_base + 0x4) |= (1 << (d->irq));

	spin_unlock_irqrestore(&ls3_lpc_lock, flags);
}

static struct irq_chip loongson_lpc_irq_chip = {
	.name		= "LOONGSON LPC",
	.irq_disable	= disable_lpc_irq,
	.irq_mask		= disable_lpc_irq,
	.irq_enable		= enable_lpc_irq,
	.irq_unmask		= enable_lpc_irq,
};

static void ls3_lpc_irq_init(void)
{
	/* Enable the LPC interrupt */
	ls2h_readl(lpc_reg_base + 0x0) = 0x80000000;

	/* set the 18-bit interrpt enable bit for keyboard and mouse */
	ls2h_readl(lpc_reg_base + 0x4) = (1 << 1) | (1 << 12);

	/* clear all 18-bit interrpt bit */
	ls2h_readl(lpc_reg_base + 0xc) = 0x3ffff;

	/* added for KBC attached on ls3 LPC controler  */
	irq_set_chip_and_handler(1, &loongson_lpc_irq_chip, handle_level_irq);
	irq_set_chip_and_handler(12, &loongson_lpc_irq_chip, handle_level_irq);
}

void __init ls2h_init_irq(void)
{
	clear_c0_status(ST0_IM | ST0_BEV);
	local_irq_disable();

	_ls2h_init_irq(LS2H_IRQ_BASE);

	/* Route INTn0 to IP3 */
	LOONGSON_INT_ROUTER_ENTRY(0)= LOONGSON_INT_CORE0_INT1;

	/* Route the LPC interrupt to Core0 IP2 */
	LOONGSON_INT_ROUTER_LPC = LOONGSON_INT_CORE0_INT0;

	/* Enable UART and INT0 interrupts */
	LOONGSON_INT_ROUTER_INTENSET = (1 << 10) | (1 << 0);

	ls3_lpc_irq_init();
}

#if     defined(CONFIG_CPU_LOONGSON3)&&defined(CONFIG_SUSPEND)
extern void ls2h_resume_irq_init(void);
void ls2h_irq_router_init(void)
{
	/* Route INTn0 to IP3 */
	LOONGSON_INT_ROUTER_ENTRY(0) = LOONGSON_INT_CORE0_INT1;

	/* Route the LPC interrupt to Core0 IP2 */
	LOONGSON_INT_ROUTER_LPC = LOONGSON_INT_CORE0_INT0;

	/* Enable UART and INT0 interrupts */
	LOONGSON_INT_ROUTER_INTENSET = (1 << 10) | (1 << 0);

	ls2h_resume_irq_init();

	/* Enable the LPC interrupt */
	ls2h_readl(lpc_reg_base + 0x0) = 0x80000000;
	/* set the 18-bit interrpt enable bit for keyboard and mouse */
	ls2h_readl(lpc_reg_base + 0x4) = (1 << 1) | (1 << 12);
	/* clear all 18-bit interrpt bit */
	ls2h_readl(lpc_reg_base + 0xc) = 0x3ffff;

}
#endif
asmlinkage void ls2h_irq_dispatch(unsigned int pending)
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
	unsigned int irq, irqs0;

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
		irqs0 = ls2h_readl(lpc_reg_base + 0x4) & ls2h_readl(lpc_reg_base + 0x8);
		if (irqs0) {
			while ((irq = ffs(irqs0)) != 0) {
				do_IRQ(irq - 1);
				irqs0 &= ~(1 << (irq-1));
        		}
		} else {
			do_IRQ(LOONGSON_UART_IRQ);
		}
	}
	if (pending & CAUSEF_IP3) {
		_ls2h_irq_dispatch();
	}
	if (pending & (~(CAUSEF_IP7 | CAUSEF_IP6 | CAUSEF_IP2 | CAUSEF_IP3))) {
		spurious_interrupt();
	}
}

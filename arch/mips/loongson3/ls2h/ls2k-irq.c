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
#include <linux/types.h>
#include <linux/interrupt.h>

#include <asm/bootinfo.h>
#include <asm/io.h>
#include <ls2h/ls2h.h>
#include <ls2h/ls2h_int.h>

extern int loongson3_send_irq_by_ipi(int cpu,int irqs);
extern unsigned long long lpc_reg_base;
static struct ls2h_int_ctrl_regs volatile *int_ctrl_regs
	= (struct ls2h_int_ctrl_regs volatile *)
	(CKSEG1ADDR(LS2H_INT_REG_BASE));

DEFINE_RAW_SPINLOCK(ls2h_irq_lock);
DEFINE_RAW_SPINLOCK(lpc_irq_lock);

#define LS2H_INT0_LPC_EN 1 << 13

void ack_ls2h_board_irq(struct irq_data *d)
{
	unsigned long flags;
	
	unsigned long irq_nr = d->irq;

	raw_spin_lock_irqsave(&ls2h_irq_lock, flags);

	irq_nr -= LS2H_IRQ_BASE;
	(int_ctrl_regs + (irq_nr >> 5))->int_clr |= (1 << (irq_nr & 0x1f));

	raw_spin_unlock_irqrestore(&ls2h_irq_lock, flags);
}

void disable_ls2h_board_irq(struct irq_data *d)
{
	unsigned long flags;

	unsigned long irq_nr = d->irq;
	
	raw_spin_lock_irqsave(&ls2h_irq_lock, flags);

	irq_nr -= LS2H_IRQ_BASE;
	(int_ctrl_regs + (irq_nr >> 5))->int_en &= ~(1 << (irq_nr & 0x1f));

	raw_spin_unlock_irqrestore(&ls2h_irq_lock, flags);
}

void enable_ls2h_board_irq(struct irq_data *d)
{
	unsigned long flags;

	unsigned long irq_nr = d->irq;
	
	raw_spin_lock_irqsave(&ls2h_irq_lock, flags);

	irq_nr -= LS2H_IRQ_BASE;
	(int_ctrl_regs + (irq_nr >> 5))->int_en |= (1 << (irq_nr & 0x1f));

	raw_spin_unlock_irqrestore(&ls2h_irq_lock, flags);
}

static struct irq_chip ls2h_board_irq_chip = {
	.name		= "LS2H BOARD",
	.irq_ack		= ack_ls2h_board_irq,
	.irq_mask		= disable_ls2h_board_irq,
	.irq_mask_ack	= disable_ls2h_board_irq,
	.irq_unmask		= enable_ls2h_board_irq,
	.irq_eoi		= enable_ls2h_board_irq,
};

static void __ls2h_irq_dispatch(int n, int intstatus)
{
	int irq;
	static unsigned int core_num = 0;

	irq = ffs(intstatus);
	if (!irq) {
		pr_info("Unknow n: %d intstatus %x \n", n, intstatus);
		spurious_interrupt();
	} else {
		core_num = (core_num + 1) % cores_per_package;
		if (core_num == 0 || !cpu_online(core_num))
			do_IRQ(n * 32 + LS2H_IRQ_BASE + irq - 1);
		else
			loongson3_send_irq_by_ipi(core_num, (n * 32 + LS2H_IRQ_BASE + irq - 1));
	}
}

void _ls2h_irq_dispatch(void)
{
	int intstatus, i, irq, irqs0;

	for (i = 0; i < 5; i++)
		if ((intstatus = (int_ctrl_regs + i)->int_isr) != 0) {
			if ((i == 0) && (intstatus == LS2H_INT0_LPC_EN)) {
				irqs0 = ls2h_readl(lpc_reg_base + 0x4) & ls2h_readl(lpc_reg_base + 0x8);
				while ((irq = ffs(irqs0)) != 0) {
					do_IRQ(irq - 1);
					irqs0 &= ~(1 << (irq-1));
				}
			} else
				__ls2h_irq_dispatch(i, intstatus);
		}
}

#if     defined(CONFIG_CPU_LOONGSON3)&&defined(CONFIG_SUSPEND)
void ls2h_resume_irq_init(void)
{

	/* uart, keyboard, and mouse are active high */
	(int_ctrl_regs + 0)->int_edge	= 0x00000000;
	(int_ctrl_regs + 0)->int_pol	= 0xff7fffff;
	(int_ctrl_regs + 0)->int_clr	= 0x00000000;
	(int_ctrl_regs + 0)->int_en	= 0x00ffffff;

	(int_ctrl_regs + 1)->int_edge	= 0x00000000;
	(int_ctrl_regs + 1)->int_pol	= 0xfeffffff;
	(int_ctrl_regs + 1)->int_clr	= 0x00000000;
	(int_ctrl_regs + 1)->int_en	= 0x03ffffff;

	(int_ctrl_regs + 2)->int_edge	= 0x00000000;
	(int_ctrl_regs + 2)->int_pol	= 0xffffffff;
	(int_ctrl_regs + 2)->int_clr	= 0x00000000;
	(int_ctrl_regs + 2)->int_en	= 0x00000000;
}
#endif
void _ls2h_init_irq(u32 irq_base)
{
	u32 i;

	/* uart, keyboard, and mouse are active high */
	(int_ctrl_regs + 0)->int_edge	= 0x00000000;
	(int_ctrl_regs + 0)->int_pol	= 0xff7fffff;
	(int_ctrl_regs + 0)->int_clr	= 0x00000000;
	(int_ctrl_regs + 0)->int_en	= 0x00ffffff;

	(int_ctrl_regs + 1)->int_edge	= 0x00000000;
	(int_ctrl_regs + 1)->int_pol	= 0xfeffffff;
	(int_ctrl_regs + 1)->int_clr	= 0x00000000;
	(int_ctrl_regs + 1)->int_en	= 0x03ffffff;

	(int_ctrl_regs + 2)->int_edge	= 0x00000000;
	(int_ctrl_regs + 2)->int_pol	= 0xffffffff;
	(int_ctrl_regs + 2)->int_clr	= 0x00000000;
	(int_ctrl_regs + 2)->int_en	= 0x00000000;

	for (i = irq_base; i <= LS2H_LAST_IRQ; i++)
		irq_set_chip_and_handler(i, &ls2h_board_irq_chip,
					 handle_level_irq);
}

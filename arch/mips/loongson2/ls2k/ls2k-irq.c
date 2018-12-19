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
#include <linux/module.h>

#include <asm/bootinfo.h>
#include <asm/io.h>
#include <loongson.h>
#include <ls2k.h>
#include <ls2k_int.h>

static int irqbalance;
module_param(irqbalance, int, 0664);
static unsigned int int_auto[2], int_bounce[2];

static struct ls2k_int_ctrl_regs volatile *int_ctrl_regs
	= (struct ls2k_int_ctrl_regs volatile *)
	(CKSEG1ADDR(LS2K_INT_REG_BASE));


static int param_set_intparam(const char *val, struct kernel_param *kp)
{
 *(volatile int *)kp->arg = simple_strtoul(val,0,0);
 (int_ctrl_regs + 0)->int_auto = int_auto[0];
 (int_ctrl_regs + 0)->int_bounce = int_bounce[0];
 (int_ctrl_regs + 1)->int_auto = int_auto[1];
 (int_ctrl_regs + 1)->int_bounce = int_bounce[1];

 return 0;
}


int param_get_intparam(char *buffer, const struct kernel_param *kp)
{
	/* Y and N chosen as being relatively non-coder friendly */
	return sprintf(buffer, "0x%x", *(unsigned int *)kp->arg);
}
module_param_call(int_auto0, param_set_intparam, param_get_intparam, &int_auto[0], 0644);
module_param_call(int_auto1, param_set_intparam, param_get_intparam, &int_auto[1], 0644);
module_param_call(int_bounce0, param_set_intparam, param_get_intparam, &int_bounce[0], 0644);
module_param_call(int_bounce1, param_set_intparam, param_get_intparam, &int_bounce[1], 0644);


DEFINE_RAW_SPINLOCK(ls2k_irq_lock);

void ack_ls2k_board_irq(struct irq_data *d)
{
	unsigned long flags;
	
	unsigned long irq_nr = d->irq;

	raw_spin_lock_irqsave(&ls2k_irq_lock, flags);

	irq_nr -= LS2K_IRQ_BASE;
	(int_ctrl_regs + (irq_nr >> 5))->int_clr = (1 << (irq_nr & 0x1f));

	raw_spin_unlock_irqrestore(&ls2k_irq_lock, flags);
}

void disable_ls2k_board_irq(struct irq_data *d)
{
	unsigned long flags;

	unsigned long irq_nr = d->irq;
	
	raw_spin_lock_irqsave(&ls2k_irq_lock, flags);

	irq_nr -= LS2K_IRQ_BASE;
	(int_ctrl_regs + (irq_nr >> 5))->int_clr = (1 << (irq_nr & 0x1f));

	raw_spin_unlock_irqrestore(&ls2k_irq_lock, flags);
}

void enable_ls2k_board_irq(struct irq_data *d)
{
	unsigned long flags;

	unsigned long irq_nr = d->irq;
	
	raw_spin_lock_irqsave(&ls2k_irq_lock, flags);

	irq_nr -= LS2K_IRQ_BASE;
	(int_ctrl_regs + (irq_nr >> 5))->int_set = (1 << (irq_nr & 0x1f));

	raw_spin_unlock_irqrestore(&ls2k_irq_lock, flags);
}

int ls2k_set_irq_affinity(struct irq_data *d, const struct cpumask *affinity,
		bool force)
{
	cpumask_t tmask;
	unsigned int cpu;
        volatile unsigned char *entry;
	unsigned long *mask;
	int off, sel;
	int irq_nr = d->irq - LS2K_IRQ_BASE;
	off = (irq_nr & 0x1f);
	sel = (irq_nr >> 5);

	cpumask_copy(&tmask, affinity);

	for_each_cpu(cpu, affinity) {
		if (!cpu_online(cpu))
			cpu_clear(cpu, tmask);
	}

	if (cpus_empty(tmask))
		cpu_set(0, tmask);

	cpumask_copy(d->affinity, &tmask);

	mask = cpumask_bits(&tmask);
	entry = &(int_ctrl_regs + sel)->int_entry[off];
	switch(*mask&3)
	{
		case 1:
			int_auto[sel] &= ~(1 << off);
			int_bounce[sel] &= ~(1 << off);
			writeb((readb(entry) & 0xf0)|0x01, entry);
		break;
		case 2:
			int_auto[sel] &= ~(1 << off);
			int_bounce[sel] &= ~(1 << off);
			writeb((readb(entry) & 0xf0)|0x02, entry);
		break;
		case 3:
			if(irqbalance&1)
			 int_auto[sel] |= (1 << off);
			else
			 int_auto[sel] &= ~(1 << off);

			if(irqbalance&2)
			 int_bounce[sel] |= (1 << off);
			else
			 int_bounce[sel] &= ~(1 << off);
			writeb((readb(entry) & 0xf0)|0x03, entry);
		break;
	}
			(int_ctrl_regs + sel)->int_auto = int_auto[sel];
			(int_ctrl_regs + sel)->int_bounce = int_bounce[sel];


	return IRQ_SET_MASK_OK_NOCOPY;
}

static struct irq_chip ls2k_board_irq_chip = {
	.name		= "LS2K BOARD",
	.irq_ack		= ack_ls2k_board_irq,
	.irq_mask		= disable_ls2k_board_irq,
	.irq_disable		= disable_ls2k_board_irq,
	.irq_mask_ack	= disable_ls2k_board_irq,
	.irq_unmask		= enable_ls2k_board_irq,
	.irq_eoi		= enable_ls2k_board_irq,
	.irq_set_affinity	= ls2k_set_irq_affinity,
};


static void __ls2k_irq_dispatch(int n, int intstatus)
{
	int irq;

	if(!intstatus) return;
	irq = ffs(intstatus);
	do_IRQ(n * 32 + LS2K_IRQ_BASE + irq - 1);
}

void _ls2k_irq_dispatch(void)
{
#if 0
	int intstatus, i;

	for (i = 0; i < 2; i++)
		if ((intstatus = (int_ctrl_regs + i)->int_isr) != 0) {
			__ls2k_irq_dispatch(i, intstatus);
		}
#else
	 int cpu = smp_processor_id();
	 if(!cpu)
	 {
		 __ls2k_irq_dispatch(0, IO_control_regs_CORE0_INTISR);
		 __ls2k_irq_dispatch(1, IO_control_regs_CORE0_INTISR_HI);
	 }
	 else
	 {
		 __ls2k_irq_dispatch(0, IO_control_regs_CORE1_INTISR);
		 __ls2k_irq_dispatch(1, IO_control_regs_CORE1_INTISR_HI);
	 }
#endif
}

void _ls2k_init_irq(u32 irq_base)
{
	u32 i;

	/* uart, keyboard, and mouse are active high */
	(int_ctrl_regs + 0)->int_clr	= -1;
	(int_ctrl_regs + 0)->int_auto	= int_auto[0];
	(int_ctrl_regs + 0)->int_bounce	= int_bounce[0];
	(int_ctrl_regs + 0)->int_edge	= (~LS2K_IRQ_MASK) & 0xffffffff;

	(int_ctrl_regs + 1)->int_clr	= -1;
	(int_ctrl_regs + 1)->int_auto	= int_auto[1];
	(int_ctrl_regs + 1)->int_bounce	= int_bounce[1];
	(int_ctrl_regs + 1)->int_edge	= (((~LS2K_IRQ_MASK)>>32)|(0x1f << 12)) & 0xffffffff;


	for (i = irq_base; i <= LS2K_LAST_IRQ; i++)
	{	if((1<<(i - irq_base)) & LS2K_IRQ_MASK)
		irq_set_chip_and_handler(i, &ls2k_board_irq_chip,
					 handle_level_irq);
#if 0
		{
			const DECLARE_BITMAP(cpumask, NR_CPUS) = {1};
			__irq_set_affinity(i, to_cpumask(cpumask),  1);
		}
#endif
	}

	/*config msi window*/
	writeq(0x000000001fe10000ULL, (void *)CKSEG1ADDR(0x1fe12500));
	writeq(0xffffffffffff0000ULL, (void *)CKSEG1ADDR(0x1fe12540));
	writeq(0x000000001fe10081ULL, (void *)CKSEG1ADDR(0x1fe12580));

}

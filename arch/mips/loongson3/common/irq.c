/*
 * Copyright (C) 2007 Lemote Inc. & Insititute of Computing Technology
 * Author: Fuxin Zhang, zhangfx@lemote.com
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 */
#include <linux/delay.h>
#include <linux/interrupt.h>

#include <asm/irq_cpu.h>
#include <asm/i8259.h>

#include <loongson.h>
#include <boot_param.h>
#include <irq.h>
#include <south-bridge.h>

extern unsigned long long smp_group[4];

static struct irqaction cascade_irqaction = {
	.handler = no_action,
	.name = "cascade",
};

/*
 * the first level int-handler will jump here if it is a bonito irq
 */
void bonito_irqdispatch(void)
{
	u32 int_status;
	int i;

	/* workaround the IO dma problem: let cpu looping to allow DMA finish */
	int_status = LOONGSON_INTISR;
	while (int_status & (1 << 10)) {
		udelay(1);
		int_status = LOONGSON_INTISR;
	}

	/* Get pending sources, masked by current enables */
	int_status = LOONGSON_INTISR & LOONGSON_INTEN;

	if (int_status) {
		i = __ffs(int_status);
		do_IRQ(LOONGSON_IRQ_BASE + i);
	}
}

asmlinkage void plat_irq_dispatch(void)
{
	unsigned int pending;

	pending = read_c0_cause() & read_c0_status() & ST0_IM;

	/* machine-specific plat_irq_dispatch */
	ls_south_bridge->sb_irq_dispatch(pending);
}

static inline void mask_loongson_irq(struct irq_data *d)
{
	clear_c0_status(0x100 << (d->irq - MIPS_CPU_IRQ_BASE));
	irq_disable_hazard();

	if (d->irq == LOONGSON_UART_IRQ) {
		int cpu = smp_processor_id();

		int node_id = cpu / cores_per_node;
		int core_id = cpu % cores_per_node;
		u64 intenclr_addr = smp_group[node_id] |
			(u64)(&LOONGSON_INT_ROUTER_INTENCLR);
		u64 introuter_lpc_addr = smp_group[node_id] |
			(u64)(&LOONGSON_INT_ROUTER_LPC);

		*(volatile u32 *)intenclr_addr = 1 << 10;
		*(volatile u8 *)introuter_lpc_addr = 0x10 + (1<<core_id);
	}
}

static inline void unmask_loongson_irq(struct irq_data *d)
{
	if (d->irq == LOONGSON_UART_IRQ) {
		int cpu = smp_processor_id();

		int node_id = cpu / cores_per_node;
		int core_id = cpu % cores_per_node;
		u64 intenset_addr = smp_group[node_id] |
			(u64)(&LOONGSON_INT_ROUTER_INTENSET);
		u64 introuter_lpc_addr = smp_group[node_id] |
			(u64)(&LOONGSON_INT_ROUTER_LPC);

		*(volatile u32 *)intenset_addr = 1 << 10;
		*(volatile u8 *)introuter_lpc_addr = 0x10 + (1<<core_id);
	}

	set_c0_status(0x100 << (d->irq - MIPS_CPU_IRQ_BASE));
	irq_enable_hazard();
}

/* For MIPS IRQs which shared by all cores */
static struct irq_chip loongson_irq_chip = {
	.name		= "Loongson",
	.irq_ack	= mask_loongson_irq,
	.irq_mask	= mask_loongson_irq,
	.irq_mask_ack	= mask_loongson_irq,
	.irq_unmask	= unmask_loongson_irq,
	.irq_eoi	= unmask_loongson_irq,
};

void __init arch_init_irq(void)
{
	/*
	 * Clear all of the interrupts while we change the able around a bit.
	 * int-handler is not on bootstrap
	 */
	clear_c0_status(ST0_IM | ST0_BEV);

	/* no steer */
	LOONGSON_INTSTEER = 0;

	/*
	 * Mask out all interrupt by writing "1" to all bit position in
	 * the interrupt reset reg.
	 */
	LOONGSON_INTENCLR = ~0;

	mips_cpu_irq_init();
	/* machine specific irq init */
	ls_south_bridge->sb_init_irq();

	setup_irq(LOONGSON_I8259_IRQ, &cascade_irqaction);

	irq_set_chip_and_handler(LOONGSON_UART_IRQ,
		&loongson_irq_chip, handle_level_irq);
	
	set_c0_status(STATUSF_IP2 | STATUSF_IP3 | STATUSF_IP6);
}

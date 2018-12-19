#include <loongson.h>
#include <irq.h>
#include <linux/interrupt.h>
#include <linux/module.h>

#include <asm/irq_cpu.h>
#include <asm/i8259.h>
#include <asm/mipsregs.h>

extern void loongson3_ipi_interrupt(struct pt_regs *regs);
extern void loongson3_send_irq_by_ipi(int cpu, int irqs);
int ht_irq_mask[8];
static int bootcore_int_mask;
static int bootcore_int_mask2;

void dispatch_msi_irq(void)
{
	int cpu = smp_processor_id();
	/*dispatch ht irqs for smi*/
#define HT_irq_vector_regs(n)	*(volatile unsigned int *)(ht_control_base + 0x80 +(n)*4)
	int i, irqs, irq;
	int mask, start, end;
	int cpumask;

#ifdef CONFIG_PCI_MSI_IRQ_BALANCE
		start = 1;
		end = 7;
		mask = 0x11111111<<cpu;
	        if(cpu == boot_cpu_id)
                mask |= bootcore_int_mask;
                cpumask = 0xff;
#else
		start = cpu? cpu*2:1;
		end = cpu?cpu*2+1:7;
		mask = 0xffffffff;
                cpumask=cpu?(3<<(cpu*2)):(bootcore_int_mask2|3);
#endif


	for(i=start;i<=end && cpumask;i++)
	{
                if(!(cpumask&(1<<i))) continue;
		cpumask &= ~(1<<i);
		irqs = 0;
		irqs |= HT_irq_vector_regs(i) & mask;
		irqs |= HT_irq_vector_regs(i) & mask;
		irqs |= HT_irq_vector_regs(i) & mask;
		if(!irqs) continue;
		HT_irq_vector_regs(i) = irqs;
		HT_irq_vector_regs(i) = irqs;
		HT_irq_vector_regs(i) = irqs;
		__sync();
//		irqs &= *(volatile unsigned int *)(HT_control_regs_base + 0xA0 + i*4);
		irqs &= ht_irq_mask[i];
		while((irq = ffs(irqs)) != 0){
			/*maybe reenter*/
			do_IRQ(64+i*32+irq-1);
			irqs &= ~(1<<(irq-1));
		}
	}
}

static unsigned int local_irq = 1<<0 | 1<<1 | 1<<2 | 1<<7 | 1<<8 | 1<<12;
core_param(local_irq, local_irq, uint, 0664);

static void ht_irqdispatch(void)
{
	unsigned int irq, irq0, irq1;
	static unsigned int dest_cpu = 0;

	int cpu = smp_processor_id();
	int mask;

#ifdef CONFIG_PCI_MSI_IRQ_BALANCE
		mask = 0x11111111<<cpu;
	        if(cpu == boot_cpu_id)
                mask |= bootcore_int_mask;
#else
		mask = 0xffffffff;
#endif

	irq = 0;
	irq |= LOONGSON_HT1_INT_VECTOR(0) & mask;
	irq |= LOONGSON_HT1_INT_VECTOR(0) & mask;
	irq |= LOONGSON_HT1_INT_VECTOR(0) & mask;

	LOONGSON_HT1_INT_VECTOR(0) = irq;
	LOONGSON_HT1_INT_VECTOR(0) = irq;
	LOONGSON_HT1_INT_VECTOR(0) = irq;
	__sync();

	irq &= ht_irq_mask[0];

	if(irq)
	{

		irq0 = irq & local_irq;  /* handled by local core */
		irq1 = irq & ~local_irq; /* balanced by other cores */

		if (dest_cpu == cpu || !cpu_online(dest_cpu))
			irq0 |= irq1;
		else if(irq1)
			loongson3_send_irq_by_ipi(dest_cpu, irq1);

		dest_cpu = (dest_cpu + 1) % cores_per_package;

		while((irq = ffs(irq0)) != 0){
			do_IRQ(irq-1);
			irq0 &= ~(1<<(irq-1));
		}
	}

	dispatch_msi_irq();
}

void rs780_irq_dispatch(unsigned int pending)
{
	if (pending & CAUSEF_IP7)
		do_IRQ(LOONGSON_TIMER_IRQ);
#if defined(CONFIG_SMP)
	if (pending & CAUSEF_IP6)
		loongson3_ipi_interrupt(NULL);
#endif
	if (pending & CAUSEF_IP3)
		ht_irqdispatch();
	if (pending & CAUSEF_IP2)
		do_IRQ(LOONGSON_UART_IRQ);
	if (pending & (~(CAUSEF_IP7 | CAUSEF_IP6 | CAUSEF_IP2 | CAUSEF_IP3))) {
		printk(KERN_ERR "%s : spurious interrupt 0x%x\n", __func__, pending);
		spurious_interrupt();
	}
}

void irq_router_init(void)
{
	int i;

	/* route LPC int to cpu core0 int 0 */
	LOONGSON_INT_ROUTER_LPC = LOONGSON_INT_COREx_INTy(boot_cpu_id, 0);
	/* route HT1 int0 ~ int7 to cpu core0 INT1*/
#ifdef CONFIG_PCI_MSI_IRQ_BALANCE
	if(setup_max_cpus>=4)
	*(volatile int *)(LOONGSON_HT1_CFG_BASE+0x58) = 0x200;
        else
	*(volatile int *)(LOONGSON_HT1_CFG_BASE+0x58) = 0x0;
#endif
	for (i = 0; i <= 3; i++)
	{
		if(i<setup_max_cpus)
		LOONGSON_INT_ROUTER_HT1(i) = LOONGSON_INT_COREx_INTy(i, 1);
		else
		{
		LOONGSON_INT_ROUTER_HT1(i) = LOONGSON_INT_COREx_INTy(boot_cpu_id, 1);
                bootcore_int_mask |= 0x11111111<<i;
                bootcore_int_mask2 |= 0x3<<(2*i);
		}
	}

	for (i = 4; i < 8; i++)
		LOONGSON_INT_ROUTER_HT1(i) = LOONGSON_INT_COREx_INTy(boot_cpu_id, 1);
	/* enable HT1 interrupt */
	LOONGSON_HT1_INTN_EN(0) = 0xffffffff;
	/* enable router interrupt intenset */
	LOONGSON_INT_ROUTER_INTENSET = LOONGSON_INT_ROUTER_INTEN | (0xffff << 16) | 0x1 << 10;
	__sync();
}

void __init rs780_init_irq(void)
{
	clear_c0_status(ST0_IM | ST0_BEV);

	irq_router_init();
	init_i8259_irqs();

}

void rs780_irq_router_init(void)
{
	int i;

	/* route LPC int to cpu core0 int 0 */
	if (cores_per_node == 4)
		LOONGSON_INT_ROUTER_LPC = LOONGSON_INT_COREx_INTy(boot_cpu_id, 0);
	else
		LOONGSON_INT_ROUTER_LPC = LOONGSON_INT_COREx_INTy(1, 0);
	/* route HT1 int0 ~ int7 to cpu core0 INT1*/
	for (i = 0; i < 8; i++) {
		if (cores_per_node == 4)
			LOONGSON_INT_ROUTER_HT1(i) = LOONGSON_INT_COREx_INTy(boot_cpu_id, 1);
		else
			LOONGSON_INT_ROUTER_HT1(i) = LOONGSON_INT_COREx_INTy(1, 1);
	}
	/* enable HT1 interrupt */
	LOONGSON_HT1_INTN_EN(0) = 0xffffffff;
	/* enable router interrupt intenset */
	LOONGSON_INT_ROUTER_INTENSET = LOONGSON_INT_ROUTER_INTEN | (0xffff << 16) | 0x1 << 10;
}
#ifdef CONFIG_HOTPLUG_CPU

void fixup_irqs(void)
{
#ifdef CONFIG_PCI_MSI_IRQ_BALANCE
	int cpu = smp_processor_id();
	if(cpu < 4)
	{
		LOONGSON_INT_ROUTER_HT1(cpu) = LOONGSON_INT_COREx_INTy(boot_cpu_id, 1);
		bootcore_int_mask |= 0x11111111<<cpu;
                bootcore_int_mask2 |= 0x3<<(2*cpu);
	}
#endif
	irq_cpu_offline();
	clear_c0_status(ST0_IM);
}

#endif

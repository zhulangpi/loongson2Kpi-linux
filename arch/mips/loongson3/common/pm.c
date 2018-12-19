/*
 * loongson-specific suspend support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/suspend.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <asm/mipsregs.h>
#include <asm/bootinfo.h>
#include <boot_param.h>
extern void prom_printf(char *fmt, ...);
extern void mach_resume_rs780(suspend_state_t state);
extern void mach_suspend_rs780(suspend_state_t state);
extern void mach_resume_ls2h(suspend_state_t state);
extern void mach_suspend_ls2h(suspend_state_t state);
u32 loongson_nr_nodes;
u64 loongson_suspend_addr;
u32 loongson_scache_ways;
u32 loongson_scache_sets;
u32 loongson_scache_linesz;
u32 loongson_pcache_ways;
u32 loongson_pcache_sets;
u32 loongson_pcache_linesz;

static u64 loongson_chipcfg[1] = {0xffffffffbfc00180};
#define LOONGSON_CHIPCFG(id) (*(volatile u32 *)(loongson_chipcfg[id]))

#define RTC_PORT(x)     (0x70 + (x))

static inline unsigned char CMOS_READ(unsigned long addr)
{
	outb_p(addr, RTC_PORT(0));
	return inb_p(RTC_PORT(1));
}

static inline void CMOS_WRITE(unsigned char data, unsigned long addr)
{
	outb_p(addr, RTC_PORT(0));
	outb_p(data, RTC_PORT(1));
}

#define HT_uncache_enable_reg0	*(volatile unsigned int *)(ht_control_base + 0xF0)
#define HT_uncache_base_reg0	*(volatile unsigned int *)(ht_control_base + 0xF4)
#define HT_uncache_enable_reg1	*(volatile unsigned int *)(ht_control_base + 0xF8)
#define HT_uncache_base_reg1	*(volatile unsigned int *)(ht_control_base + 0xFC)

extern enum loongson_cpu_type cputype;
static unsigned int __maybe_unused cached_autoplug_enabled;

void cmos_write64(uint64_t data, unsigned long addr)
{
	int i;
	unsigned char * bytes = (unsigned char *)&data;

	for (i=0; i<8; i++)
		CMOS_WRITE(bytes[i], addr + i);
}

/*
 * Setup the board-specific events for waking up loongson from wait mode
 */
void __weak setup_wakeup_events(void)
{
}

/*
 * Check wakeup events
 */
int __weak wakeup_loongson(void)
{
	return 1;
}
struct loongson_registers {
	u32 config4;
	u32 config6;
	u64 pgd;
	u64 kpgd;
	u32 pwctl;
	u64 pwbase;
	u64 pwsize;
	u64 pwfield;
};

static struct loongson_registers loongson_regs;
/*save 3a2000 and 3a3000 new feature register*/
static void mach_save_register(void)
{
	loongson_regs.config4 = read_c0_config4();
	loongson_regs.config6 = read_c0_config6();

	loongson_regs.pgd = read_c0_pgd();
	loongson_regs.kpgd = read_c0_kpgd();
	loongson_regs.pwctl = read_c0_pwctl();
	loongson_regs.pwbase = read_c0_pwbase();
	loongson_regs.pwsize = read_c0_pwsize();
	loongson_regs.pwfield = read_c0_pwfield();
}
/*resume 3a2000 and 3a3000 new feature register*/
static void mach_resume_register(void)
{
	write_c0_config4(loongson_regs.config4);
	write_c0_config6(loongson_regs.config6);

	write_c0_pgd(loongson_regs.pgd);
	write_c0_kpgd(loongson_regs.kpgd);
	write_c0_pwctl(loongson_regs.pwctl);
	write_c0_pwbase(loongson_regs.pwbase);
	write_c0_pwsize(loongson_regs.pwsize);
	write_c0_pwfield(loongson_regs.pwfield);
}
/*
 * If the events are really what we want to wakeup the CPU, wake it up
 * otherwise put the CPU asleep again.
 */
static void wait_for_wakeup_events(void)
{
	while (!wakeup_loongson())
		switch(cputype) {
		case Loongson_3A:
		default:
			LOONGSON_CHIPCFG(0) &= ~0x7;
			break;
		}
}

/*
 * Stop all perf counters
 *
 * $24 is the control register of Loongson perf counter
 */
static inline void stop_perf_counters(void)
{
	__write_64bit_c0_register($25, 0, 0xc0000000);
	__write_64bit_c0_register($25, 2, 0x40000000);
}


static void loongson_suspend_enter(void)
{
	static unsigned int cached_cpu_freq;

	/* setup wakeup events via enabling the IRQs */
	setup_wakeup_events();

	stop_perf_counters();

	switch(cputype) {
	case Loongson_3A:
	default:
		cached_cpu_freq = LOONGSON_CHIPCFG(0);
		/* Put CPU into wait mode */
		LOONGSON_CHIPCFG(0) &= ~0x7;
		/* wait for the given events to wakeup cpu from wait mode */
		wait_for_wakeup_events();
		LOONGSON_CHIPCFG(0) = cached_cpu_freq;
		break;
	}
	mmiowb();
}
void mach_suspend(suspend_state_t state);
void mach_resume(suspend_state_t state);

void loongson_suspend_lowlevel(void);
static int loongson_pm_enter(suspend_state_t state)
{
	int tmp;

	if (state == PM_SUSPEND_MEM) {
		if (board_type == RS780E)
			mach_suspend_rs780(state);
		else if (board_type == LS2H)
			mach_suspend_ls2h(state);

		tmp = (read_c0_prid() & 0xf);
		if (((tmp == PRID_REV_LOONGSON3A2000) || (tmp == PRID_REV_LOONGSON3A3000)))
		{
			mach_save_register();
		}
	}
	/* processor specific suspend */
	switch(state){
		case PM_SUSPEND_STANDBY:
			loongson_suspend_enter();
			break;
		case PM_SUSPEND_MEM:
#ifdef CONFIG_CPU_LOONGSON3
		loongson_nr_nodes = nr_nodes_loongson;
		loongson_suspend_addr = 0xffffffffbfc00500;
		loongson_pcache_ways = cpu_data[0].dcache.ways;
		loongson_pcache_sets = cpu_data[0].dcache.sets;
		loongson_pcache_linesz = cpu_data[0].dcache.linesz;
		loongson_scache_ways = cpu_data[0].scache.ways;
		loongson_scache_sets = cpu_data[0].scache.sets*4;
		loongson_scache_linesz = cpu_data[0].scache.linesz;
		loongson_suspend_lowlevel();
		cmos_write64(0x0, 0x40);  /* clear pc in cmos */
		cmos_write64(0x0, 0x48);  /* clear sp in cmos */
#else
		loongson_suspend_enter();
#endif
			break;
	}
	if (state == PM_SUSPEND_MEM) {

		tmp = (read_c0_prid() & 0xf);
		if (((tmp == PRID_REV_LOONGSON3A2000) || (tmp == PRID_REV_LOONGSON3A3000))) {
			mach_resume_register();
		}

		if (board_type == RS780E) {
			mach_resume_rs780(state);
		}
		else if (board_type == LS2H)
			mach_resume_ls2h(state);
	}
	return 0;
}

static int loongson_pm_valid_state(suspend_state_t state)
{
	switch (state) {
		case PM_SUSPEND_ON:
			return 1;

		case PM_SUSPEND_STANDBY:
		case PM_SUSPEND_MEM:
			return 1;
		default:
			return 0;
	}
}

static int loongson_pm_begin(suspend_state_t state)
{
#ifdef CONFIG_LOONGSON3_CPUAUTOPLUG
	extern int autoplug_enabled;
	cached_autoplug_enabled = autoplug_enabled;
	autoplug_enabled = 0;
#endif
	return 0;
}
void __cpuinit disable_unused_cpus(void);
static void loongson_pm_wake(void)
{
#ifdef CONFIG_CPU_LOONGSON3
	disable_unused_cpus();
#endif
}

static void loongson_pm_end(void)
{
#ifdef CONFIG_LOONGSON3_CPUAUTOPLUG
	extern int autoplug_enabled;
	autoplug_enabled = cached_autoplug_enabled;
#endif
}

static const struct platform_suspend_ops loongson_pm_ops = {
	.valid	= loongson_pm_valid_state,
	.begin	= loongson_pm_begin,
	.enter	= loongson_pm_enter,
	.wake	= loongson_pm_wake,
	.end	= loongson_pm_end,
};

static int __init loongson_pm_init(void)
{
	suspend_set_ops(&loongson_pm_ops);

	return 0;
}
arch_initcall(loongson_pm_init);

/*
 * Linux performance counter support for MIPS.
 *
 * Copyright (C) 2010 MIPS Technologies, Inc.
 * Copyright (C) 2011 Cavium Networks, Inc.
 * Author: Deng-Cheng Zhu
 *
 * This code is based on the implementation for ARM, which is in turn
 * based on the sparc64 perf event code and the x86 code. Performance
 * counter access is based on the MIPS Oprofile code. And the callchain
 * support references the code of MIPS stacktrace.c.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/cpumask.h>
#include <linux/interrupt.h>
#include <linux/smp.h>
#include <linux/kernel.h>
#include <linux/perf_event.h>
#include <linux/uaccess.h>

#include <asm/irq.h>
#include <asm/irq_regs.h>
#include <asm/stacktrace.h>
#include <asm/time.h> /* For perf_irq */

#define MIPS_MAX_HWEVENTS 4

struct cpu_hw_events {
	/* Array of events on this cpu. */
	struct perf_event	*events[MIPS_MAX_HWEVENTS];

	/*
	 * Set the bit (indexed by the counter number) when the counter
	 * is used for an event.
	 */
	unsigned long		used_mask[BITS_TO_LONGS(MIPS_MAX_HWEVENTS)];

	/*
	 * Software copy of the control register for each performance counter.
	 * MIPS CPUs vary in performance counters. They use this differently,
	 * and even may not use it.
	 */
	unsigned int		saved_ctrl[MIPS_MAX_HWEVENTS];
};
DEFINE_PER_CPU(struct cpu_hw_events, cpu_hw_events) = {
	.saved_ctrl = {0},
};

/* The description of MIPS performance events. */
struct mips_perf_event {
	unsigned int event_id;
	/*
	 * MIPS performance counters are indexed starting from 0.
	 * CNTR_EVEN indicates the indexes of the counters to be used are
	 * even numbers.
	 */
	unsigned int cntr_mask;
	#define CNTR_EVEN	0x55555555
	#define CNTR_ODD	0xaaaaaaaa
	#define CNTR_ALL	0xffffffff
#ifdef CONFIG_MIPS_MT_SMP
	enum {
		T  = 0,
		V  = 1,
		P  = 2,
	} range;
#else
	#define T
	#define V
	#define P
#endif
};

static struct mips_perf_event raw_event;
static DEFINE_MUTEX(raw_event_mutex);

#define UNSUPPORTED_PERF_EVENT_ID 0xffffffff
#define C(x) PERF_COUNT_HW_CACHE_##x

struct mips_pmu {
	u64		max_period;
	u64		valid_count;
	u64		overflow;
	const char	*name;
	int		irq;
	u64		(*read_counter)(unsigned int idx);
	void		(*write_counter)(unsigned int idx, u64 val);
	const struct mips_perf_event *(*map_raw_event)(u64 config);
	const struct mips_perf_event (*general_event_map)[PERF_COUNT_HW_MAX];
	const struct mips_perf_event (*cache_event_map)
				[PERF_COUNT_HW_CACHE_MAX]
				[PERF_COUNT_HW_CACHE_OP_MAX]
				[PERF_COUNT_HW_CACHE_RESULT_MAX];
	unsigned int	num_counters;
};

static struct mips_pmu mipspmu;

#define M_CONFIG1_PC	(1 << 4)

#define M_PERFCTL_EXL			(1	<<  0)
#define M_PERFCTL_KERNEL		(1 	<<  1)
#define M_PERFCTL_SUPERVISOR		(1 	<<  2)
#define M_PERFCTL_USER			(1 	<<  3)
#define M_PERFCTL_INTERRUPT_ENABLE	(1	<<  4)
#define M_PERFCTL_EVENT(event)		(((event) & 0x3ff)  << 5)
#define M_PERFCTL_VPEID(vpe)		((vpe)		<< 16)
#define M_PERFCTL_MT_EN(filter)		((filter) << 20)
#define    M_TC_EN_ALL			M_PERFCTL_MT_EN(0)
#define    M_TC_EN_VPE			M_PERFCTL_MT_EN(1)
#define    M_TC_EN_TC			M_PERFCTL_MT_EN(2)
#define M_PERFCTL_TCID(tcid)		((tcid)	<< 22)
#define M_PERFCTL_WIDE			(1	<< 30)
#define M_PERFCTL_MORE			(1	<< 31)

#define M_PERFCTL_COUNT_EVENT_WHENEVER	(M_PERFCTL_EXL |		\
					M_PERFCTL_KERNEL |		\
					M_PERFCTL_USER |		\
					M_PERFCTL_SUPERVISOR |		\
					M_PERFCTL_INTERRUPT_ENABLE)

#ifdef CONFIG_MIPS_MT_SMP
#define M_PERFCTL_CONFIG_MASK		0x3fff801f
#else
#define M_PERFCTL_CONFIG_MASK		0x1f
#endif
#define M_PERFCTL_EVENT_MASK		0x7fe0


#ifdef CONFIG_MIPS_MT_SMP
static int cpu_has_mipsmt_pertccounters;

static DEFINE_RWLOCK(pmuint_rwlock);

/*
 * FIXME: For VSMP, vpe_id() is redefined for Perf-events, because
 * cpu_data[cpuid].vpe_id reports 0 for _both_ CPUs.
 */
#if defined(CONFIG_HW_PERF_EVENTS)
#define vpe_id()	(cpu_has_mipsmt_pertccounters ? \
			0 : smp_processor_id())
#else
#define vpe_id()	(cpu_has_mipsmt_pertccounters ? \
			0 : cpu_data[smp_processor_id()].vpe_id)
#endif

/* Copied from op_model_mipsxx.c */
static unsigned int vpe_shift(void)
{
	if (num_possible_cpus() > 1)
		return 1;

	return 0;
}

static unsigned int counters_total_to_per_cpu(unsigned int counters)
{
	return counters >> vpe_shift();
}

#else /* !CONFIG_MIPS_MT_SMP */
#define vpe_id()	0

#endif /* CONFIG_MIPS_MT_SMP */

static void resume_local_counters(void);
static void pause_local_counters(void);
static irqreturn_t mipsxx_pmu_handle_irq(int, void *);
static int mipsxx_pmu_handle_shared_irq(void);

static unsigned int mipsxx_pmu_swizzle_perf_idx(unsigned int idx)
{
	if (vpe_id() == 1)
		idx = (idx + 2) & 3;
	return idx;
}

static u64 mipsxx_pmu_read_counter(unsigned int idx)
{
	idx = mipsxx_pmu_swizzle_perf_idx(idx);

	switch (idx) {
	case 0:
		/*
		 * The counters are unsigned, we must cast to truncate
		 * off the high bits.
		 */
		return (u32)read_c0_perfcntr0();
	case 1:
		return (u32)read_c0_perfcntr1();
	case 2:
		return (u32)read_c0_perfcntr2();
	case 3:
		return (u32)read_c0_perfcntr3();
	default:
		WARN_ONCE(1, "Invalid performance counter number (%d)\n", idx);
		return 0;
	}
}

static u64 mipsxx_pmu_read_counter_64(unsigned int idx)
{
	idx = mipsxx_pmu_swizzle_perf_idx(idx);

	switch (idx) {
	case 0:
		return read_c0_perfcntr0_64();
	case 1:
		return read_c0_perfcntr1_64();
	case 2:
		return read_c0_perfcntr2_64();
	case 3:
		return read_c0_perfcntr3_64();
	default:
		WARN_ONCE(1, "Invalid performance counter number (%d)\n", idx);
		return 0;
	}
}

static u64 mipsxx_pmu_read_counter_48(unsigned int idx)
{
	idx = mipsxx_pmu_swizzle_perf_idx(idx);

	switch (idx) {
	case 0:
		return read_c0_perfcntr0_64() & 0x0000ffffffffffff;
	case 1:
		return read_c0_perfcntr1_64() & 0x0000ffffffffffff;
	case 2:
		return read_c0_perfcntr2_64() & 0x0000ffffffffffff;
	case 3:
		return read_c0_perfcntr3_64() & 0x0000ffffffffffff;
	default:
		WARN_ONCE(1, "Invalid performance counter number (%d)\n", idx);
		return 0;
	}
}
static void mipsxx_pmu_write_counter(unsigned int idx, u64 val)
{
	idx = mipsxx_pmu_swizzle_perf_idx(idx);

	switch (idx) {
	case 0:
		write_c0_perfcntr0(val);
		return;
	case 1:
		write_c0_perfcntr1(val);
		return;
	case 2:
		write_c0_perfcntr2(val);
		return;
	case 3:
		write_c0_perfcntr3(val);
		return;
	}
}

static void mipsxx_pmu_write_counter_64(unsigned int idx, u64 val)
{
	idx = mipsxx_pmu_swizzle_perf_idx(idx);

	switch (idx) {
	case 0:
		write_c0_perfcntr0_64(val);
		return;
	case 1:
		write_c0_perfcntr1_64(val);
		return;
	case 2:
		write_c0_perfcntr2_64(val);
		return;
	case 3:
		write_c0_perfcntr3_64(val);
		return;
	}
}

static void mipsxx_pmu_write_counter_48(unsigned int idx, u64 val)
{
	idx = mipsxx_pmu_swizzle_perf_idx(idx);

	switch (idx) {
	case 0:
		write_c0_perfcntr0_64(val & 0x0000ffffffffffff);
		return;
	case 1:
		write_c0_perfcntr1_64(val & 0x0000ffffffffffff);
		return;
	case 2:
		write_c0_perfcntr2_64(val & 0x0000ffffffffffff);
		return;
	case 3:
		write_c0_perfcntr3_64(val & 0x0000ffffffffffff);
		return;
	}
}
static unsigned int mipsxx_pmu_read_control(unsigned int idx)
{
	idx = mipsxx_pmu_swizzle_perf_idx(idx);

	switch (idx) {
	case 0:
		return read_c0_perfctrl0();
	case 1:
		return read_c0_perfctrl1();
	case 2:
		return read_c0_perfctrl2();
	case 3:
		return read_c0_perfctrl3();
	default:
		WARN_ONCE(1, "Invalid performance counter number (%d)\n", idx);
		return 0;
	}
}

static void mipsxx_pmu_write_control(unsigned int idx, unsigned int val)
{
	idx = mipsxx_pmu_swizzle_perf_idx(idx);

	switch (idx) {
	case 0:
		write_c0_perfctrl0(val);
		return;
	case 1:
		write_c0_perfctrl1(val);
		return;
	case 2:
		write_c0_perfctrl2(val);
		return;
	case 3:
		write_c0_perfctrl3(val);
		return;
	}
}

static int mipsxx_pmu_alloc_counter(struct cpu_hw_events *cpuc,
				    struct hw_perf_event *hwc)
{
	int i;
	unsigned long cntr_mask;
	/*
	 * We only need to care the counter mask. The range has been
	 * checked definitely.
	 */
	if (((read_c0_prid() & 0xf) == PRID_REV_LOONGSON3A2000) || ((read_c0_prid() & 0xf) == PRID_REV_LOONGSON3A3000))
		cntr_mask = (hwc->event_base >> 10) & 0xffff;
	else
		cntr_mask = (hwc->event_base >> 8) & 0xffff;


	for (i = mipspmu.num_counters - 1; i >= 0; i--) {
		/*
		 * Note that some MIPS perf events can be counted by both
		 * even and odd counters, wheresas many other are only by
		 * even _or_ odd counters. This introduces an issue that
		 * when the former kind of event takes the counter the
		 * latter kind of event wants to use, then the "counter
		 * allocation" for the latter event will fail. In fact if
		 * they can be dynamically swapped, they both feel happy.
		 * But here we leave this issue alone for now.
		 */
		if (test_bit(i, &cntr_mask) &&
			!test_and_set_bit(i, cpuc->used_mask)) { 
                return i;
		}
	}

	return -EAGAIN;
}

static void mipsxx_pmu_enable_event(struct hw_perf_event *evt, int idx)
{
	struct cpu_hw_events *cpuc = &__get_cpu_var(cpu_hw_events);

	WARN_ON(idx < 0 || idx >= mipspmu.num_counters);

	if (((read_c0_prid() & 0xf) == PRID_REV_LOONGSON3A2000) || ((read_c0_prid() & 0xf) == PRID_REV_LOONGSON3A3000))
		cpuc->saved_ctrl[idx] = M_PERFCTL_EVENT(evt->event_base & 0x3ff) |
			(evt->config_base & M_PERFCTL_CONFIG_MASK) |
			/* Make sure interrupt enabled. */
			M_PERFCTL_INTERRUPT_ENABLE;
	else
		cpuc->saved_ctrl[idx] = M_PERFCTL_EVENT(evt->event_base & 0xff) |
			(evt->config_base & M_PERFCTL_CONFIG_MASK) |
			/* Make sure interrupt enabled. */
			M_PERFCTL_INTERRUPT_ENABLE;
	/*
	 * We do not actually let the counter run. Leave it until start().
	 */
}

static void mipsxx_pmu_disable_event(int idx)
{
	struct cpu_hw_events *cpuc = &__get_cpu_var(cpu_hw_events);
	unsigned long flags;

	WARN_ON(idx < 0 || idx >= mipspmu.num_counters);

	local_irq_save(flags);
	cpuc->saved_ctrl[idx] = mipsxx_pmu_read_control(idx) &
		~M_PERFCTL_COUNT_EVENT_WHENEVER;
	mipsxx_pmu_write_control(idx, cpuc->saved_ctrl[idx]);
	local_irq_restore(flags);
}

static int mipspmu_event_set_period(struct perf_event *event,
				    struct hw_perf_event *hwc,
				    int idx)
{
	u64 left = local64_read(&hwc->period_left);
	u64 period = hwc->sample_period;
	int ret = 0;
	if (((read_c0_prid() & 0xf) == PRID_REV_LOONGSON3A2000) || ((read_c0_prid() & 0xf) == PRID_REV_LOONGSON3A3000)){
		if (unlikely((left + period) & (1ULL << 47))) {
			/* left underflowed by more than period. */
			left = period;
			local64_set(&hwc->period_left, left);
			hwc->last_period = period;
			ret = 1;
		} else	if (unlikely((left + period) <= period)) {
			/* left underflowed by less than period. */
			left += period;
			local64_set(&hwc->period_left, left);
			hwc->last_period = period;
			ret = 1;
		}
	}
	else{
		if (unlikely((left + period) & (1ULL << 63))) {

			/* left underflowed by more than period. */
			left = period;
			local64_set(&hwc->period_left, left);
			hwc->last_period = period;
			ret = 1;
		} else	if (unlikely((left + period) <= period)) {
			/* left underflowed by less than period. */
			left += period;
			local64_set(&hwc->period_left, left);
			hwc->last_period = period;
			ret = 1;
		}
	}
	if (left > mipspmu.max_period) {
		left = mipspmu.max_period;
		local64_set(&hwc->period_left, left);
	}

	local64_set(&hwc->prev_count, mipspmu.overflow - left);

	mipsxx_pmu_write_control(idx, M_PERFCTL_EVENT(hwc->event_base & 0x3ff)); 
	mipspmu.write_counter(idx, mipspmu.overflow - left);

	perf_event_update_userpage(event);

	return ret;
}

static void mipspmu_event_update(struct perf_event *event,
				 struct hw_perf_event *hwc,
				 int idx)
{
	u64 prev_raw_count, new_raw_count;
	u64 delta;

again:
	prev_raw_count = local64_read(&hwc->prev_count);
	new_raw_count = mipspmu.read_counter(idx);

	if (local64_cmpxchg(&hwc->prev_count, prev_raw_count,
				new_raw_count) != prev_raw_count)
		goto again;

	delta = new_raw_count - prev_raw_count;

	local64_add(delta, &event->count);
	local64_sub(delta, &hwc->period_left);
}

static void mipspmu_start(struct perf_event *event, int flags)
{
	struct hw_perf_event *hwc = &event->hw;

	if (flags & PERF_EF_RELOAD)
		WARN_ON_ONCE(!(hwc->state & PERF_HES_UPTODATE));

	hwc->state = 0;

	/* Set the period for the event. */
	mipspmu_event_set_period(event, hwc, hwc->idx);

	/* Enable the event. */
	mipsxx_pmu_enable_event(hwc, hwc->idx);
}

static void mipspmu_stop(struct perf_event *event, int flags)
{
	struct hw_perf_event *hwc = &event->hw;

	if (!(hwc->state & PERF_HES_STOPPED)) {
		/* We are working on a local event. */
		mipsxx_pmu_disable_event(hwc->idx);
		barrier();
		mipspmu_event_update(event, hwc, hwc->idx);
		hwc->state |= PERF_HES_STOPPED | PERF_HES_UPTODATE;
	}
}

static int mipspmu_add(struct perf_event *event, int flags)
{
	struct cpu_hw_events *cpuc = &__get_cpu_var(cpu_hw_events);
	struct hw_perf_event *hwc = &event->hw;
	int idx;
	int err = 0;

	perf_pmu_disable(event->pmu);

	/* To look for a free counter for this event. */
	idx = mipsxx_pmu_alloc_counter(cpuc, hwc);
	if (idx < 0) {
		err = idx;
		goto out;
	}

	/*
	 * If there is an event in the counter we are going to use then
	 * make sure it is disabled.
	 */
	event->hw.idx = idx;
	mipsxx_pmu_disable_event(idx);
	cpuc->events[idx] = event;

	hwc->state = PERF_HES_STOPPED | PERF_HES_UPTODATE;
	if (flags & PERF_EF_START)
		mipspmu_start(event, PERF_EF_RELOAD);

	/* Propagate our changes to the userspace mapping. */
	perf_event_update_userpage(event);

out:
	perf_pmu_enable(event->pmu);
	return err;
}

static void mipspmu_del(struct perf_event *event, int flags)
{
	struct cpu_hw_events *cpuc = &__get_cpu_var(cpu_hw_events);
	struct hw_perf_event *hwc = &event->hw;
	int idx = hwc->idx;


	WARN_ON(idx < 0 || idx >= mipspmu.num_counters);

	mipspmu_stop(event, PERF_EF_UPDATE);
	cpuc->events[idx] = NULL;
	clear_bit(idx, cpuc->used_mask);

	perf_event_update_userpage(event);
}

static void mipspmu_read(struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;

	/* Don't read disabled counters! */
	if (hwc->idx < 0)
		return;

	mipspmu_event_update(event, hwc, hwc->idx);
}

static void mipspmu_enable(struct pmu *pmu)
{
#ifdef CONFIG_MIPS_MT_SMP
	write_unlock(&pmuint_rwlock);
#endif
	resume_local_counters();
}

/*
 * MIPS performance counters can be per-TC. The control registers can
 * not be directly accessed accross CPUs. Hence if we want to do global
 * control, we need cross CPU calls. on_each_cpu() can help us, but we
 * can not make sure this function is called with interrupts enabled. So
 * here we pause local counters and then grab a rwlock and leave the
 * counters on other CPUs alone. If any counter interrupt raises while
 * we own the write lock, simply pause local counters on that CPU and
 * spin in the handler. Also we know we won't be switched to another
 * CPU after pausing local counters and before grabbing the lock.
 */
static void mipspmu_disable(struct pmu *pmu)
{
	pause_local_counters();
#ifdef CONFIG_MIPS_MT_SMP
	write_lock(&pmuint_rwlock);
#endif
}

static atomic_t active_events = ATOMIC_INIT(0);
static DEFINE_MUTEX(pmu_reserve_mutex);
static int (*save_perf_irq)(void);

static int mipspmu_get_irq(void)
{
	int err;

	if (mipspmu.irq >= 0) {
		/* Request my own irq handler. */
		err = request_irq(mipspmu.irq, mipsxx_pmu_handle_irq,
			IRQF_PERCPU | IRQF_NOBALANCING,
			"mips_perf_pmu", NULL);
		if (err) {
			pr_warning("Unable to request IRQ%d for MIPS "
			   "performance counters!\n", mipspmu.irq);
		}
	} else if (cp0_perfcount_irq < 0) {
		/*
		 * We are sharing the irq number with the timer interrupt.
		 */
		save_perf_irq = perf_irq;
		perf_irq = mipsxx_pmu_handle_shared_irq;
		err = 0;
	} else {
		pr_warning("The platform hasn't properly defined its "
			"interrupt controller.\n");
		err = -ENOENT;
	}

	return err;
}

static void mipspmu_free_irq(void)
{
	if (mipspmu.irq >= 0)
		free_irq(mipspmu.irq, NULL);
	else if (cp0_perfcount_irq < 0)
		perf_irq = save_perf_irq;
}

/*
 * mipsxx/rm9000/loongson2 have different performance counters, they have
 * specific low-level init routines.
 */
static void reset_counters(void *arg);
static int __hw_perf_event_init(struct perf_event *event);

static void hw_perf_event_destroy(struct perf_event *event)
{
	if (atomic_dec_and_mutex_lock(&active_events,
				&pmu_reserve_mutex)) {
		/*
		 * We must not call the destroy function with interrupts
		 * disabled.
		 */
		on_each_cpu(reset_counters,
			(void *)(long)mipspmu.num_counters, 1);
		mipspmu_free_irq();
		mutex_unlock(&pmu_reserve_mutex);
	}
}

static int mipspmu_event_init(struct perf_event *event)
{
	int err = 0;

	/* does not support taken branch sampling */
	if (has_branch_stack(event))
		return -EOPNOTSUPP;

	switch (event->attr.type) {
	case PERF_TYPE_RAW:
	case PERF_TYPE_HARDWARE:
	case PERF_TYPE_HW_CACHE:
		break;

	default:
		return -ENOENT;
	}

	if (event->cpu >= nr_cpumask_bits ||
	    (event->cpu >= 0 && !cpu_online(event->cpu)))
		return -ENODEV;

	if (!atomic_inc_not_zero(&active_events)) {
		mutex_lock(&pmu_reserve_mutex);
		if (atomic_read(&active_events) == 0)
			err = mipspmu_get_irq();

		if (!err)
			atomic_inc(&active_events);
		mutex_unlock(&pmu_reserve_mutex);
	}

	if (err)
		return err;

	return __hw_perf_event_init(event);
}

static struct pmu pmu = {
	.pmu_enable	= mipspmu_enable,
	.pmu_disable	= mipspmu_disable,
	.event_init	= mipspmu_event_init,
	.add		= mipspmu_add,
	.del		= mipspmu_del,
	.start		= mipspmu_start,
	.stop		= mipspmu_stop,
	.read		= mipspmu_read,
};

static unsigned int mipspmu_perf_event_encode(const struct mips_perf_event *pev)
{
/*
 * Top 8 bits for range, next 16 bits for cntr_mask, lowest 8 bits for
 * event_id.
 */
#if defined(CONFIG_MIPS_MT_SMP)
	return ((unsigned int)pev->range << 24) |
		(pev->cntr_mask & 0xffff00) |
		(pev->event_id & 0xff);
#else
	if (((read_c0_prid() & 0xf) == PRID_REV_LOONGSON3A2000) || ((read_c0_prid() & 0xf) == PRID_REV_LOONGSON3A3000))
	
		return (pev->cntr_mask & 0xfffc00) | 
			(pev->event_id & 0x3ff);
	else
		return (pev->cntr_mask & 0xffff00) |
			(pev->event_id & 0xff);
#endif
}

static const struct mips_perf_event *mipspmu_map_general_event(int idx)
{
	const struct mips_perf_event *pev;

	pev = ((*mipspmu.general_event_map)[idx].event_id ==
		UNSUPPORTED_PERF_EVENT_ID ? ERR_PTR(-EOPNOTSUPP) :
		&(*mipspmu.general_event_map)[idx]);

	return pev;
}

static const struct mips_perf_event *mipspmu_map_cache_event(u64 config)
{
	unsigned int cache_type, cache_op, cache_result;
	const struct mips_perf_event *pev;

	cache_type = (config >> 0) & 0xff;
	if (cache_type >= PERF_COUNT_HW_CACHE_MAX)
		return ERR_PTR(-EINVAL);

	cache_op = (config >> 8) & 0xff;
	if (cache_op >= PERF_COUNT_HW_CACHE_OP_MAX)
		return ERR_PTR(-EINVAL);

	cache_result = (config >> 16) & 0xff;
	if (cache_result >= PERF_COUNT_HW_CACHE_RESULT_MAX)
		return ERR_PTR(-EINVAL);

	pev = &((*mipspmu.cache_event_map)
					[cache_type]
					[cache_op]
					[cache_result]);

	if (pev->event_id == UNSUPPORTED_PERF_EVENT_ID)
		return ERR_PTR(-EOPNOTSUPP);

	return pev;

}

static int validate_group(struct perf_event *event)
{
	struct perf_event *sibling, *leader = event->group_leader;
	struct cpu_hw_events fake_cpuc;

	memset(&fake_cpuc, 0, sizeof(fake_cpuc));

	if (mipsxx_pmu_alloc_counter(&fake_cpuc, &leader->hw) < 0)
		return -EINVAL;

	list_for_each_entry(sibling, &leader->sibling_list, group_entry) {
		if (mipsxx_pmu_alloc_counter(&fake_cpuc, &sibling->hw) < 0)
			return -EINVAL;
	}

	if (mipsxx_pmu_alloc_counter(&fake_cpuc, &event->hw) < 0)
		return -EINVAL;

	return 0;
}

/* This is needed by specific irq handlers in perf_event_*.c */
static void handle_associated_event(struct cpu_hw_events *cpuc,
				    int idx, struct perf_sample_data *data,
				    struct pt_regs *regs)
{
	struct perf_event *event = cpuc->events[idx];
	struct hw_perf_event *hwc = &event->hw;


	mipspmu_event_update(event, hwc, idx);
	data->period = event->hw.last_period;
	if (!mipspmu_event_set_period(event, hwc, idx))
		return;

	if (perf_event_overflow(event, data, regs))
		mipsxx_pmu_disable_event(idx);
}


static int __n_counters(void)
{
	if (!(read_c0_config1() & M_CONFIG1_PC))
		return 0;
	if (!(read_c0_perfctrl0() & M_PERFCTL_MORE))
		return 1;
	if (!(read_c0_perfctrl1() & M_PERFCTL_MORE))
		return 2;
	if (!(read_c0_perfctrl2() & M_PERFCTL_MORE))
		return 3;

	return 4;
}

static int n_counters(void)
{
	int counters;

	switch (current_cpu_type()) {
	case CPU_R10000:
		counters = 2;
		break;

	case CPU_R12000:
	case CPU_R14000:
		counters = 4;
		break;

	default:
		counters = __n_counters();
	}


	return counters;
}
static void loongson3_reset_counters(void* arg)
{
	int counters = (int)(long)arg;	
	switch (counters) {
		case 4:
			mipsxx_pmu_write_control(3, 0);
			mipspmu.write_counter(3, 0);
			mipsxx_pmu_write_control(3, 127<<5);
			mipspmu.write_counter(3, 0);
			mipsxx_pmu_write_control(3, 191<<5);
			mipspmu.write_counter(3, 0);
			mipsxx_pmu_write_control(3, 255<<5);
			mipspmu.write_counter(3, 0);
			mipsxx_pmu_write_control(3, 319<<5);
			mipspmu.write_counter(3, 0);
			mipsxx_pmu_write_control(3, 383<<5);
			mipspmu.write_counter(3, 0);
			mipsxx_pmu_write_control(3, 575<<5);
			mipspmu.write_counter(3, 0);
		case 3:
			mipsxx_pmu_write_control(2, 0);
			mipspmu.write_counter(2, 0);
			mipsxx_pmu_write_control(2, 127<<5);
			mipspmu.write_counter(2, 0);
			mipsxx_pmu_write_control(2, 191<<5);
			mipspmu.write_counter(2, 0);
			mipsxx_pmu_write_control(2, 255<<5);
			mipspmu.write_counter(2, 0);
			mipsxx_pmu_write_control(2, 319<<5);
			mipspmu.write_counter(2, 0);
			mipsxx_pmu_write_control(2, 383<<5);
			mipspmu.write_counter(2, 0);
			mipsxx_pmu_write_control(2, 575<<5);
			mipspmu.write_counter(2, 0);
		case 2:
			mipsxx_pmu_write_control(1, 0);
			mipspmu.write_counter(1, 0);
			mipsxx_pmu_write_control(1, 127<<5);
			mipspmu.write_counter(1, 0);
			mipsxx_pmu_write_control(1, 191<<5);
			mipspmu.write_counter(1, 0);
			mipsxx_pmu_write_control(1, 255<<5);
			mipspmu.write_counter(1, 0);
			mipsxx_pmu_write_control(1, 319<<5);
			mipspmu.write_counter(1, 0);
			mipsxx_pmu_write_control(1, 383<<5);
			mipspmu.write_counter(1, 0);
			mipsxx_pmu_write_control(1, 575<<5);
			mipspmu.write_counter(1, 0);
		case 1:
			mipsxx_pmu_write_control(0, 0);
			mipspmu.write_counter(0, 0);
			mipsxx_pmu_write_control(0, 127<<5);
			mipspmu.write_counter(0, 0);
			mipsxx_pmu_write_control(0, 191<<5);
			mipspmu.write_counter(0, 0);
			mipsxx_pmu_write_control(0, 255<<5);
			mipspmu.write_counter(0, 0);
			mipsxx_pmu_write_control(0, 319<<5);
			mipspmu.write_counter(0, 0);
			mipsxx_pmu_write_control(0, 383<<5);
			mipspmu.write_counter(0, 0);
			mipsxx_pmu_write_control(0, 575<<5);
			mipspmu.write_counter(0, 0);
	}
}
static void reset_counters(void *arg)
{
	int counters = (int)(long)arg;
	if (((read_c0_prid() & 0xf) == PRID_REV_LOONGSON3A2000) || ((read_c0_prid() & 0xf) == PRID_REV_LOONGSON3A3000)){
		loongson3_reset_counters(arg);
		return;
	}

	switch (counters) {
		case 4:
			mipspmu.write_counter(3, 0);
			mipsxx_pmu_write_control(3, 0);
		case 3:
			mipspmu.write_counter(2, 0);
			mipsxx_pmu_write_control(2, 0);
		case 2:
			mipspmu.write_counter(1, 0);
			mipsxx_pmu_write_control(1, 0);
		case 1:
			mipspmu.write_counter(0, 0);
			mipsxx_pmu_write_control(0, 0);
	}

}

/* 24K/34K/1004K cores can share the same event map. */
static const struct mips_perf_event mipsxxcore_event_map
				[PERF_COUNT_HW_MAX] = {
	[PERF_COUNT_HW_CPU_CYCLES] = { 0x00, CNTR_EVEN | CNTR_ODD, P },
	[PERF_COUNT_HW_INSTRUCTIONS] = { 0x01, CNTR_EVEN | CNTR_ODD, T },
	[PERF_COUNT_HW_CACHE_REFERENCES] = { UNSUPPORTED_PERF_EVENT_ID },
	[PERF_COUNT_HW_CACHE_MISSES] = { UNSUPPORTED_PERF_EVENT_ID },
	[PERF_COUNT_HW_BRANCH_INSTRUCTIONS] = { 0x02, CNTR_EVEN, T },
	[PERF_COUNT_HW_BRANCH_MISSES] = { 0x02, CNTR_ODD, T },
	[PERF_COUNT_HW_BUS_CYCLES] = { UNSUPPORTED_PERF_EVENT_ID },
};

/* 74K core has different branch event code. */
static const struct mips_perf_event mipsxx74Kcore_event_map
				[PERF_COUNT_HW_MAX] = {
	[PERF_COUNT_HW_CPU_CYCLES] = { 0x00, CNTR_EVEN | CNTR_ODD, P },
	[PERF_COUNT_HW_INSTRUCTIONS] = { 0x01, CNTR_EVEN | CNTR_ODD, T },
	[PERF_COUNT_HW_CACHE_REFERENCES] = { UNSUPPORTED_PERF_EVENT_ID },
	[PERF_COUNT_HW_CACHE_MISSES] = { UNSUPPORTED_PERF_EVENT_ID },
	[PERF_COUNT_HW_BRANCH_INSTRUCTIONS] = { 0x27, CNTR_EVEN, T },
	[PERF_COUNT_HW_BRANCH_MISSES] = { 0x27, CNTR_ODD, T },
	[PERF_COUNT_HW_BUS_CYCLES] = { UNSUPPORTED_PERF_EVENT_ID },
};

static const struct mips_perf_event octeon_event_map[PERF_COUNT_HW_MAX] = {
	[PERF_COUNT_HW_CPU_CYCLES] = { 0x01, CNTR_ALL },
	[PERF_COUNT_HW_INSTRUCTIONS] = { 0x03, CNTR_ALL },
	[PERF_COUNT_HW_CACHE_REFERENCES] = { 0x2b, CNTR_ALL },
	[PERF_COUNT_HW_CACHE_MISSES] = { 0x2e, CNTR_ALL  },
	[PERF_COUNT_HW_BRANCH_INSTRUCTIONS] = { 0x08, CNTR_ALL },
	[PERF_COUNT_HW_BRANCH_MISSES] = { 0x09, CNTR_ALL },
	[PERF_COUNT_HW_BUS_CYCLES] = { 0x25, CNTR_ALL },
};

static const struct mips_perf_event loongson3_event_map
[PERF_COUNT_HW_MAX] = {
	[PERF_COUNT_HW_CPU_CYCLES] = { 0x00, CNTR_EVEN, P },
	[PERF_COUNT_HW_INSTRUCTIONS] = { 0x00, CNTR_ODD, T },
	[PERF_COUNT_HW_CACHE_REFERENCES] = { UNSUPPORTED_PERF_EVENT_ID },
	[PERF_COUNT_HW_CACHE_MISSES] = { UNSUPPORTED_PERF_EVENT_ID },
	[PERF_COUNT_HW_BRANCH_INSTRUCTIONS] = { 0x01, CNTR_EVEN, T },
	[PERF_COUNT_HW_BRANCH_MISSES] = { 0x01, CNTR_ODD, T },
	[PERF_COUNT_HW_BUS_CYCLES] = { UNSUPPORTED_PERF_EVENT_ID },
	[PERF_COUNT_HW_STALLED_CYCLES_FRONTEND] = { UNSUPPORTED_PERF_EVENT_ID },
	[PERF_COUNT_HW_STALLED_CYCLES_BACKEND] = { UNSUPPORTED_PERF_EVENT_ID },

#if defined(CONFIG_CPU_LOONGSON3_GS464E)
	[PERF_COUNT_HW_JUMP_INSTRUCTIONS] = { 0x02, CNTR_EVEN, T },
	[PERF_COUNT_HW_JR31_INSTRUCTIONS] = { 0x03, CNTR_EVEN, T },
	[PERF_COUNT_HW_ICACHE_MISSES] = { 0x04, CNTR_EVEN, T },
	[PERF_COUNT_HW_ALU1_ISSUED] = { 0x05, CNTR_EVEN, T },
	[PERF_COUNT_HW_MEM_ISSUED] = { 0x06, CNTR_EVEN, T },
	[PERF_COUNT_HW_FALU1_ISSUED] = { 0x07, CNTR_EVEN, T },
	[PERF_COUNT_HW_BHT_BRANCH_INSTRUCTIONS] = { 0x08, CNTR_EVEN, T },
	[PERF_COUNT_HW_MEM_READ] = { 0x09, CNTR_EVEN, T },
	[PERF_COUNT_HW_FQUEUE_FULL] = { 0x0a, CNTR_EVEN, T },
	[PERF_COUNT_HW_ROQ_FULL] = { 0x0b, CNTR_EVEN, T },
	[PERF_COUNT_HW_CP0_QUEUE_FULL] = { 0x0c, CNTR_EVEN, T },
	[PERF_COUNT_HW_TLB_REFILL] = { 0x0d, CNTR_EVEN, T },
	[PERF_COUNT_HW_EXCEPTION] = { 0x0e, CNTR_EVEN, T },
	[PERF_COUNT_HW_INTERNAL_EXCEPTION] = { 0x0f, CNTR_EVEN, T },

	[PERF_COUNT_HW_JR_MISPREDICTED] = { 0x02, CNTR_ODD, T },
	[PERF_COUNT_HW_JR31_MISPREDICTED] = { 0x03, CNTR_ODD, T },
	[PERF_COUNT_HW_DCACHE_MISSES] = { 0x04, CNTR_ODD, T },
	[PERF_COUNT_HW_ALU2_ISSUED] = { 0x05, CNTR_ODD, T },
	[PERF_COUNT_HW_FALU2_ISSUED] = { 0x06, CNTR_ODD, T },
	[PERF_COUNT_HW_UNCACHED_ACCESS] = { 0x07, CNTR_ODD, T },
	[PERF_COUNT_HW_BHT_MISPREDICTED] = { 0x08, CNTR_ODD, T },
	[PERF_COUNT_HW_MEM_WRITE] = { 0x09, CNTR_ODD, T },
	[PERF_COUNT_HW_FTQ_FULL] = { 0x0a, CNTR_ODD, T },
	[PERF_COUNT_HW_BRANCH_QUEUE_FULL] = { 0x0b, CNTR_ODD, T },
	[PERF_COUNT_HW_ITLB_MISSES] = { 0x0c, CNTR_ODD, T },
	[PERF_COUNT_HW_TOTAL_EXCEPTIONS] = { 0x0d, CNTR_ODD, T },
	[PERF_COUNT_HW_LOAD_SPECULATION_MISSES] = { 0x0e, CNTR_ODD, T },
	[PERF_COUNT_HW_CP0Q_FORWARD_VALID] = { 0x0f, CNTR_ODD, T },
#endif
};

static const struct mips_perf_event loongson3a2000_event_map
				[PERF_COUNT_HW_MAX] = {
	[PERF_COUNT_HW_CPU_CYCLES] = { 0x80, CNTR_ALL },
	[PERF_COUNT_HW_INSTRUCTIONS] = { 0x81, CNTR_ALL },
	[PERF_COUNT_HW_CACHE_REFERENCES] = { 0x17, CNTR_ALL },
	[PERF_COUNT_HW_CACHE_MISSES] = { 0x18, CNTR_ALL },
	[PERF_COUNT_HW_BRANCH_INSTRUCTIONS] = { 0x94, CNTR_ALL },
	[PERF_COUNT_HW_BRANCH_MISSES] = { 0x9c, CNTR_ALL },
	[PERF_COUNT_HW_BUS_CYCLES] = { UNSUPPORTED_PERF_EVENT_ID },
	[PERF_COUNT_HW_STALLED_CYCLES_FRONTEND] = { 0x3, CNTR_ALL },
	[PERF_COUNT_HW_STALLED_CYCLES_BACKEND] = { UNSUPPORTED_PERF_EVENT_ID },
#if defined(CONFIG_CPU_LOONGSON3_GS464E)
    /*fetch*/
	[PERF_COUNT_HW_INSTQ_EMPTY] = { 0x01, CNTR_ALL },
	[PERF_COUNT_HW_INSTQ_INSTRUCTIONS] = { 0x02, CNTR_ALL },
	[PERF_COUNT_HW_INSTQ_1] = { 0x04, CNTR_ALL },
	[PERF_COUNT_HW_INSTQ_2] = { 0x05, CNTR_ALL },
	[PERF_COUNT_HW_INSTQ_3] = { 0x06, CNTR_ALL },
	[PERF_COUNT_HW_INSTQ_4] = { 0x07, CNTR_ALL },
	[PERF_COUNT_HW_INSTQ_5] = { 0x08, CNTR_ALL },
	[PERF_COUNT_HW_INSTQ_6] = { 0x09, CNTR_ALL },
	[PERF_COUNT_HW_INSTQ_7] = { 0x0a, CNTR_ALL },
	[PERF_COUNT_HW_INSTQ_8] = { 0x0b, CNTR_ALL },
	[PERF_COUNT_HW_INSTQ_LESSTHAN_8] = { 0x0c, CNTR_ALL },
	[PERF_COUNT_HW_INSTQ_FULL] = { 0x0d, CNTR_ALL },
	[PERF_COUNT_HW_DECODE_INST] = { 0x0e, CNTR_ALL },
	[PERF_COUNT_HW_LOOP_BUFFER_INST] = { 0x0f, CNTR_ALL },
	[PERF_COUNT_HW_LOOP_FIND] = { 0x10, CNTR_ALL },
	[PERF_COUNT_HW_LOOP_TRIGGER] = { 0x11, CNTR_ALL },
	[PERF_COUNT_HW_DECODE_BRANCH_0] = { 0x12, CNTR_ALL },
	[PERF_COUNT_HW_DECODE_BRANCH_1] = { 0x13, CNTR_ALL },
	[PERF_COUNT_HW_DECODE_BRANCH_2] = { 0x14, CNTR_ALL },
	[PERF_COUNT_HW_ICACHE_MISSES_BLOCK] = { 0x15, CNTR_ALL },
	[PERF_COUNT_HW_BRBTB_TAKEN_BRANCH_MISSES] = { 0x16, CNTR_ALL },
	[PERF_COUNT_HW_ICACHE_REPLACE] = { 0x19, CNTR_ALL },
	[PERF_COUNT_HW_ITLB_MISS_TLB_HIT] = { 0x1a, CNTR_ALL },
	[PERF_COUNT_HW_ITLB_FLUSHED] = { 0x1b, CNTR_ALL },
    /*rmap*/
	[PERF_COUNT_HW_RESOURCE_ALLOC_BLOCKED] = { 0x40, CNTR_ALL },
	[PERF_COUNT_HW_GR_BLOCKED] = { 0x41, CNTR_ALL },
	[PERF_COUNT_HW_GR_PSEUDO_BLOCKED] = { 0x42, CNTR_ALL },
	[PERF_COUNT_HW_FR_BLOCKED] = { 0x43, CNTR_ALL },
	[PERF_COUNT_HW_FR_PSEUDO_BLOCKED] = { 0x44, CNTR_ALL },
	[PERF_COUNT_HW_FCR_BLOCKED] = { 0x45, CNTR_ALL },
	[PERF_COUNT_HW_FCR_PSEUDO_BLOCKED] = { 0x46, CNTR_ALL },
	[PERF_COUNT_HW_ACC_BLOCKED] = { 0x47, CNTR_ALL },
	[PERF_COUNT_HW_ACC_PSEUDO_BLOCKED] = { 0x48, CNTR_ALL },
	[PERF_COUNT_HW_DSPCTRL_BLOCKED] = { 0x49, CNTR_ALL },
	[PERF_COUNT_HW_DSPCTRL_PSEUDO_BLOCKED] = { 0x4a, CNTR_ALL },
	[PERF_COUNT_HW_BRQ_BLOCKED] = { 0x4b, CNTR_ALL },
	[PERF_COUNT_HW_BRQ_PSEUDO_BLOCKED] = { 0x4c, CNTR_ALL },
	[PERF_COUNT_HW_FXQ_BLOCKED] = { 0x4d, CNTR_ALL },
	[PERF_COUNT_HW_FXQ_PSEUDO_BLOCKED] = { 0x4e, CNTR_ALL },
	[PERF_COUNT_HW_FTQ_BLOCKED] = { 0x4f, CNTR_ALL },
	[PERF_COUNT_HW_FTQ_PSEUDO_BLOCKED] = { 0x50, CNTR_ALL },
	[PERF_COUNT_HW_MMQ_BLOCKED] = { 0x51, CNTR_ALL },
	[PERF_COUNT_HW_MMQ_PSEUDO_BLOCKED] = { 0x52, CNTR_ALL },
	[PERF_COUNT_HW_CP0Q_BLOCKED] = { 0x53, CNTR_ALL },
	[PERF_COUNT_HW_CP0Q_PSEUDO_BLOCKED] = { 0x54, CNTR_ALL },
	[PERF_COUNT_HW_ROQ_BLOCKED] = { 0x55, CNTR_ALL },
	[PERF_COUNT_HW_NOP_INST] = { 0x56, CNTR_ALL },
	[PERF_COUNT_HW_REGMAP_ISSUED] = { 0x57, CNTR_ALL },
	[PERF_COUNT_HW_EXCEPTIONS] = { 0x58, CNTR_ALL },
	[PERF_COUNT_HW_BRANCH_MISSES_OVERHEAD] = { 0x59, CNTR_ALL },
    /*roq*/
	[PERF_COUNT_HW_ALU_COMMITTED] = { 0x82, CNTR_ALL },
	[PERF_COUNT_HW_FALU_COMMITTED] = { 0x83, CNTR_ALL },
	[PERF_COUNT_HW_MEMORY_SWAP_COMMITTED] = { 0x84, CNTR_ALL },
	[PERF_COUNT_HW_LOAD_COMMITTED] = { 0x85, CNTR_ALL },
	[PERF_COUNT_HW_STORE_COMMITTED] = { 0x86, CNTR_ALL },
	[PERF_COUNT_HW_LL_COMMITTED] = { 0x87, CNTR_ALL },
	[PERF_COUNT_HW_SC_COMMITTED] = { 0x88, CNTR_ALL },
	[PERF_COUNT_HW_UNALIGNED_LOAD_COMMITTED] = { 0x89, CNTR_ALL },
	[PERF_COUNT_HW_UNALIGNED_STORE_COMMITTED] = { 0x8a, CNTR_ALL },
	[PERF_COUNT_HW_EXCEPTIONS_AND_INTERRUPTS] = { 0x8b, CNTR_ALL },
	[PERF_COUNT_HW_INTERRUPTS] = { 0x8c, CNTR_ALL },
	[PERF_COUNT_HW_ROQ_INTERRUPT] = { 0x8d, CNTR_ALL },
	[PERF_COUNT_HW_ROQ_INTERRUPT_INST] = { 0x8e, CNTR_ALL },
	[PERF_COUNT_HW_VM_EXCEPTIONS] = { 0x8f, CNTR_ALL },
	[PERF_COUNT_HW_ADDRESS_FAULT_EXCEPTIONS] = { 0x90, CNTR_ALL },
	[PERF_COUNT_HW_TLB_EXCEPTIONS] = { 0x91, CNTR_ALL },
	[PERF_COUNT_HW_TLB_REFILL_EXCEPTIONS] = { 0x92, CNTR_ALL },
	[PERF_COUNT_HW_TLB_REFILL_HANDLE_TIME] = { 0x93, CNTR_ALL },
	[PERF_COUNT_HW_JUMP_REGISTER] = { 0x95, CNTR_ALL },
	[PERF_COUNT_HW_JUMP_AND_LINK] = { 0x96, CNTR_ALL },
	[PERF_COUNT_HW_BRANCH_AND_LINK] = { 0x97, CNTR_ALL },
	[PERF_COUNT_HW_BHT_BRANCH] = { 0x98, CNTR_ALL },
	[PERF_COUNT_HW_LIKELY_BRANCH] = { 0x99, CNTR_ALL },
	[PERF_COUNT_HW_NOT_TAKEN_BRANCH] = { 0x9a, CNTR_ALL },
	[PERF_COUNT_HW_TAKEN_BRANCH] = { 0x9b, CNTR_ALL },
	[PERF_COUNT_HW_JUMP_REGISTER_MISSES] = { 0x9d, CNTR_ALL },
	[PERF_COUNT_HW_JUMP_AND_LINK_MISSES] = { 0x9e, CNTR_ALL },
	[PERF_COUNT_HW_BRANCH_AND_LINK_MISSES] = { 0x9f, CNTR_ALL },
	[PERF_COUNT_HW_BHT_BRANCH_MISSES] = { 0xa0, CNTR_ALL },
	[PERF_COUNT_HW_LIKELY_BRANCH_MISSES] = { 0xa1, CNTR_ALL },
	[PERF_COUNT_HW_NOT_TAKEN_MISSES] = { 0xa2, CNTR_ALL },
	[PERF_COUNT_HW_TAKEN_MISSES] = { 0xa3, CNTR_ALL },
    /*fix*/
	[PERF_COUNT_HW_FXQ_NO_ISSUE] = { 0xc0, CNTR_ALL },
	[PERF_COUNT_HW_FXQ_ISSUE_OPERAND] = { 0xc1, CNTR_ALL },
	[PERF_COUNT_HW_FXQ_FU0_OPERAND] = { 0xc2, CNTR_ALL },
	[PERF_COUNT_HW_FXQ_FU1_OPERAND] = { 0xc3, CNTR_ALL },
	[PERF_COUNT_HW_FU0_FIXED_MUL] = { 0xc4, CNTR_ALL },
	[PERF_COUNT_HW_FU0_FIXED_DIV] = { 0xc5, CNTR_ALL },
	[PERF_COUNT_HW_FU1_FIXED_MUL] = { 0xc6, CNTR_ALL },
	[PERF_COUNT_HW_FU1_FIXED_DIV] = { 0xc7, CNTR_ALL },
    /*float*/
	[PERF_COUNT_HW_FTQ_NO_ISSUE] = { 0x100, CNTR_ALL },
	[PERF_COUNT_HW_FTQ_ISSUE_OPERAND] = { 0x101, CNTR_ALL },
	[PERF_COUNT_HW_FTQ_FU3_OPERAND] = { 0x102, CNTR_ALL },
	[PERF_COUNT_HW_FTQ_FU4_OPERAND] = { 0x103, CNTR_ALL },
	[PERF_COUNT_HW_FU3_EMPTY_FU4_FULL] = { 0x104, CNTR_ALL },
	[PERF_COUNT_HW_FU4_EMPTY_FU3_FULL] = { 0x105, CNTR_ALL },
	[PERF_COUNT_HW_SCALAR_FLOAT_OPERAND] = { 0x106, CNTR_ALL },
	[PERF_COUNT_HW_GS_ALU_INST] = { 0x107, CNTR_ALL },
	[PERF_COUNT_HW_FIXED_VECTOR_ISSUE_64] = { 0x108, CNTR_ALL },
	[PERF_COUNT_HW_VECTOR_ISSUE_128] = { 0x109, CNTR_ALL },
	[PERF_COUNT_HW_FIXED_VECTOR_ISSUE_128] = { 0x10a, CNTR_ALL },
	[PERF_COUNT_HW_FLOAT_VECTOR_ISSUE_128] = { 0x10b, CNTR_ALL },
	[PERF_COUNT_HW_VECTOR_ISSUE_256] = { 0x10c, CNTR_ALL },
	[PERF_COUNT_HW_FIXED_VECTOR_ISSUE_256] = { 0x10d, CNTR_ALL },
	[PERF_COUNT_HW_FLOAT_VECTOR_ISSUE_256] = { 0x10e, CNTR_ALL },
	[PERF_COUNT_HW_FU3_VECTOR_FIXED_DIV] = { 0x10f, CNTR_ALL },
	[PERF_COUNT_HW_FU3_FLOAT_DIV_SQRT] = { 0x110, CNTR_ALL },
	[PERF_COUNT_HW_FU4_VECTOR_FIXED_DIV] = { 0x111, CNTR_ALL },
	[PERF_COUNT_HW_FU4_FLOAT_DIV_SQRT] = { 0x112, CNTR_ALL },
    /*memory*/
	[PERF_COUNT_HW_MMQ_NO_ISSUE] = { 0x140, CNTR_ALL },
	[PERF_COUNT_HW_MMQ_ISSUE_OPERAND] = { 0x141, CNTR_ALL },
	[PERF_COUNT_HW_MMQ_FU2_INST] = { 0x142, CNTR_ALL },
	[PERF_COUNT_HW_MMQ_FU5_INST] = { 0x143, CNTR_ALL },
	[PERF_COUNT_HW_LOAD_ISSUE] = { 0x144, CNTR_ALL },
	[PERF_COUNT_HW_STORE_ISSUE] = { 0x145, CNTR_ALL },
	[PERF_COUNT_HW_SRC_FLOAT_MEM_INST] = { 0x146, CNTR_ALL },
	[PERF_COUNT_HW_VECTOR_MEM_ISSUE] = { 0x147, CNTR_ALL },
	[PERF_COUNT_HW_FIX_FLOAT_SHIFT_ISSUE] = { 0x148, CNTR_ALL },
	[PERF_COUNT_HW_WAIT_FIRST_BLOCK_CYCLES] = { 0x149, CNTR_ALL },
	[PERF_COUNT_HW_SYNC_BLOCK_CYCLES] = { 0x14a, CNTR_ALL },
	[PERF_COUNT_HW_STALL_ISSUE_BLOCK_CYCLES] = { 0x14b, CNTR_ALL },
	[PERF_COUNT_HW_SOFTWARE_PREFETCH_TOTAL] = { 0x14c, CNTR_ALL },
	[PERF_COUNT_HW_DMEMREF_BLOCK_DCACHEWRITE] = { 0x14d, CNTR_ALL },
	[PERF_COUNT_HW_DMEMREF_BANK_CLASH] = { 0x14e, CNTR_ALL },
	[PERF_COUNT_HW_REFILL_BLOCK_DMEMREF] = { 0x14f, CNTR_ALL },
	[PERF_COUNT_HW_DCACHEWRITE_NO_CANCEL] = { 0x150, CNTR_ALL },
	[PERF_COUNT_HW_DCACHEWRITE0_AND_1_VALID] = { 0x151, CNTR_ALL },
	[PERF_COUNT_HW_SC_WRITE_DCACHE] = { 0x152, CNTR_ALL },
	[PERF_COUNT_HW_STORE_DCACHE_MISS] = { 0x153, CNTR_ALL },
	[PERF_COUNT_HW_STORE_DCACHE_SHARED_MISS] = { 0x154, CNTR_ALL },
    /*cache2mem*/
	[PERF_COUNT_HW_STORE_DCACHE_HIT] = { 0x155, CNTR_ALL },
	[PERF_COUNT_HW_LOAD_HIT] = { 0x156, CNTR_ALL },
	[PERF_COUNT_HW_FWDBUS2] = { 0x157, CNTR_ALL },
	[PERF_COUNT_HW_FWDBUS5] = { 0x158, CNTR_ALL },
	[PERF_COUNT_HW_FWDBUS_TOTAL] = { 0x159, CNTR_ALL },
	[PERF_COUNT_HW_DWAITSTORE] = { 0x15a, CNTR_ALL },
	[PERF_COUNT_HW_MISPEC] = { 0x15b, CNTR_ALL },
	[PERF_COUNT_HW_DCACHEWRITE_CANCEL] = { 0x15c, CNTR_ALL },
	[PERF_COUNT_HW_CP0Q_DMEMREAD] = { 0x15d, CNTR_ALL },
	[PERF_COUNT_HW_CP0Q_DUNCACHE] = { 0x15e, CNTR_ALL },
	[PERF_COUNT_HW_RESBUS2_OCCUPY_RESBUS5] = { 0x15f, CNTR_ALL },
	[PERF_COUNT_HW_SW_PREFETCH_L1DCACHE_HIT] = { 0x160, CNTR_ALL },
	[PERF_COUNT_HW_STORE_SW_PREFETCH_L1DCACHE_HIT] = { 0x161, CNTR_ALL },
	[PERF_COUNT_HW_STORE_SW_PREFETCH_L1DCACHE_MISS] = { 0x162, CNTR_ALL },
	[PERF_COUNT_HW_LOAD_SW_PREFETCH_L1DCACHE_HIT] = { 0x163, CNTR_ALL },
	[PERF_COUNT_HW_LOAD_SW_PREFETCH_L1DCACHE_MISS] = { 0x164, CNTR_ALL },
	[PERF_COUNT_HW_STORE_L1DCACHE_MISS_SHARE_STATE] = { 0x165, CNTR_ALL },
	[PERF_COUNT_HW_SPECFWDBUS2] = { 0x166, CNTR_ALL },
	[PERF_COUNT_HW_SPECFWDBUS5] = { 0x167, CNTR_ALL },
	[PERF_COUNT_HW_SPECFWDBUS2_TOTAL] = { 0x168, CNTR_ALL },
	[PERF_COUNT_HW_DATA_LOAD_VCACHE_ACCESS_REQ] = { 0x180, CNTR_ALL },
	[PERF_COUNT_HW_DATA_STORE_VCACHE_ACCESS_REQ] = { 0x181, CNTR_ALL },
	[PERF_COUNT_HW_DATA_VCACHE_ACCESS_REQ] = { 0x182, CNTR_ALL },
	[PERF_COUNT_HW_INST_VCACHE_ACCESS_REQ] = { 0x183, CNTR_ALL },
	[PERF_COUNT_HW_VCACHE_ACCESS] = { 0x184, CNTR_ALL },
	[PERF_COUNT_HW_SW_PREFETCH_ACCESS_VCACHE] = { 0x185, CNTR_ALL },
	[PERF_COUNT_HW_VCACHE_LOAD_HIT] = { 0x186, CNTR_ALL },
	[PERF_COUNT_HW_VCACHE_STORE_HIT] = { 0x187, CNTR_ALL },
	[PERF_COUNT_HW_VCACHE_DATA_HIT] = { 0x188, CNTR_ALL },
	[PERF_COUNT_HW_VCACHE_INST_HIT] = { 0x189, CNTR_ALL },
	[PERF_COUNT_HW_VCACHE_HIT] = { 0x18a, CNTR_ALL },
	[PERF_COUNT_HW_VCACHE_SW_PREFETCH_HIT] = { 0x18b, CNTR_ALL },
	[PERF_COUNT_HW_VCACHE_LOAD_MISS] = { 0x18c, CNTR_ALL },
	[PERF_COUNT_HW_VCACHE_STORE_MISS] = { 0x18d, CNTR_ALL },
	[PERF_COUNT_HW_VCACHE_DATA_MISS] = { 0x18e, CNTR_ALL },
	[PERF_COUNT_HW_VCACHE_INST_MISS] = { 0x18f, CNTR_ALL },
	[PERF_COUNT_HW_VCACHE_MISS] = { 0x190, CNTR_ALL },
	[PERF_COUNT_HW_VCACHE_SW_PREFETCH_MISS] = { 0x191, CNTR_ALL },
	[PERF_COUNT_HW_VCACHE_EXTREQ_INVALID] = { 0x192, CNTR_ALL },
	[PERF_COUNT_HW_VCACHE_WTBK_DEGRADE] = { 0x193, CNTR_ALL },
	[PERF_COUNT_HW_VCACHE_INV_INVALID] = { 0x194, CNTR_ALL },
	[PERF_COUNT_HW_VCACHE_INVWTBK_INVALID] = { 0x195, CNTR_ALL },
	[PERF_COUNT_HW_AR_REQUEST_ISSUE] = { 0x196, CNTR_ALL },
	[PERF_COUNT_HW_AW_REQUEST_ISSUE] = { 0x197, CNTR_ALL },
	[PERF_COUNT_HW_AW_DATA_REQUEST] = { 0x198, CNTR_ALL },
	[PERF_COUNT_HW_AR_BLOCKED_AW_UNDONE] = { 0x199, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_WTBK_REQUEST] = { 0x19a, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_INVWTBK_REQUEST] = { 0x19b, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_INV_REQUEST] = { 0x19c, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_INV_CLASS_REQUEST] = { 0x19d, CNTR_ALL },
	[PERF_COUNT_HW_REFILL_TOTAL] = { 0x19e, CNTR_ALL },
	[PERF_COUNT_HW_REFILL_ICACHE] = { 0x19f, CNTR_ALL },
	[PERF_COUNT_HW_REFILL_DCACHE] = { 0x1a0, CNTR_ALL },
	[PERF_COUNT_HW_REPLACE_REFILL] = { 0x1a1, CNTR_ALL },
	[PERF_COUNT_HW_REFILL_DCACHE_SHARED] = { 0x1a2, CNTR_ALL },
	[PERF_COUNT_HW_REFILL_DCACHE_EXC] = { 0x1a3, CNTR_ALL },
	[PERF_COUNT_HW_REFILL_DATA_TOTAL] = { 0x1a4, CNTR_ALL },
	[PERF_COUNT_HW_REFILL_INST_TOTAL] = { 0x1a5, CNTR_ALL },
	[PERF_COUNT_HW_DCACHE_REPLACE_VALID_BLOCK] = { 0x1a6, CNTR_ALL },
	[PERF_COUNT_HW_DCACHE_REPLACE_SHARED_BLOCK] = { 0x1a7, CNTR_ALL },
	[PERF_COUNT_HW_DCACHE_REPLACE_EXC_BLOCK] = { 0x1a8, CNTR_ALL },
	[PERF_COUNT_HW_DCACHE_REPLACE_DIRTY_BLOCK] = { 0x1a9, CNTR_ALL },
	[PERF_COUNT_HW_ICACHE_REPLACE_VALID_DATA] = { 0x1aa, CNTR_ALL },
	[PERF_COUNT_HW_VCACHE_REPLACE] = { 0x1ab, CNTR_ALL },
	[PERF_COUNT_HW_VCACHE_REPLACE_VALID_BLOCK] = { 0x1ac, CNTR_ALL },
	[PERF_COUNT_HW_VCACHE_REPLACE_SHARED_BLOCK] = { 0x1ad, CNTR_ALL },
	[PERF_COUNT_HW_VCACHE_REPLACE_EXC_BLOCK] = { 0x1ae, CNTR_ALL },
	[PERF_COUNT_HW_VCACHE_REPLACE_DIRTY_BLOCK] = { 0x1af, CNTR_ALL },
	[PERF_COUNT_HW_VCACHE_REPLACE_VALID_DCBLOCK] = { 0x1b0, CNTR_ALL },
	[PERF_COUNT_HW_VCACHE_REPLACE_VALID_ICBLOCK] = { 0x1b1, CNTR_ALL },
	[PERF_COUNT_HW_SCACHE_LOAD_NOT_RETURN] = { 0x1b2, CNTR_ALL },
	[PERF_COUNT_HW_SCACHE_STORE_NOT_RETURN] = { 0x1b3, CNTR_ALL },
	[PERF_COUNT_HW_SCACHE_ICACHEREQ_NOT_RETURN] = { 0x1b4, CNTR_ALL },
	[PERF_COUNT_HW_SCREAD_TOTAL] = { 0x1b5, CNTR_ALL },
	[PERF_COUNT_HW_SCREAD_LOAD] = { 0x1b6, CNTR_ALL },
	[PERF_COUNT_HW_SCREAD_STORE] = { 0x1b7, CNTR_ALL },
	[PERF_COUNT_HW_SCREAD_DATA_ACCESS] = { 0x1b8, CNTR_ALL },
	[PERF_COUNT_HW_SCREAD_INST_ACCESS] = { 0x1b9, CNTR_ALL },
	[PERF_COUNT_HW_SCREAD_NPREFETCH] = { 0x1ba, CNTR_ALL },
	[PERF_COUNT_HW_SCREAD_NPREFETCH_DATA_LOAD] = { 0x1bb, CNTR_ALL },
	[PERF_COUNT_HW_SCREAD_NPREFETCH_STORE] = { 0x1bc, CNTR_ALL },
	[PERF_COUNT_HW_SCREAD_NPREFETCH_DATA_ACCESS] = { 0x1bd, CNTR_ALL },
	[PERF_COUNT_HW_SCREAD_NPREFETCH_INST_ACCESS] = { 0x1be, CNTR_ALL },
	[PERF_COUNT_HW_SCREAD_PREFETCH] = { 0x1bf, CNTR_ALL },
	[PERF_COUNT_HW_SCREAD_LOAD_PREFETCH] = { 0x1c0, CNTR_ALL },
	[PERF_COUNT_HW_SCREAD_STORE_PREFETCH] = { 0x1c1, CNTR_ALL },
	[PERF_COUNT_HW_SCREAD_PREFETCH_DATA_ACCESS] = { 0x1c2, CNTR_ALL },
	[PERF_COUNT_HW_SCREAD_PREFETCH_INST_ACCESS] = { 0x1c3, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_SW_PREFETCH_REQUEST] = { 0x1c4, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_SCWRITE] = { 0x1c5, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_REPLACE_SCWRITE] = { 0x1c6, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_INVALID_SCWRITE] = { 0x1c7, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_REPLACE_VALID_SCWRITE] = { 0x1c8, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_ACCEPT_REQ] = { 0x1c9, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_ACCEPT_LOAD] = { 0x1ca, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_ACCEPT_SIORE] = { 0x1cb, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_DATA_ACCESS] = { 0x1cc, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_INST_ACCESS] = { 0x1cd, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_NON_EMPTY] = { 0x1ce, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_COMMON_ACCESS_OCCUPY] = { 0x1cf, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_FETCH_ACCESS_OCCUPY] = { 0x1d0, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_EXTERNAL_REQ_OCCUPY] = { 0x1d1, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_PREFETCH_REQ_OCCUPY] = { 0x1d2, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_OCCUPY_CYCLES] = { 0x1d3, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_COMMON_ACCESS_CYCLES] = { 0x1d4, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_FETCH_ACCESS_CYCLES] = { 0x1d5, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_EXTERNAL_REQ_CYCLES] = { 0x1d6, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_PREFETCH_REQ_CYCLES] = { 0x1d7, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_FULL_COUNT] = { 0x1d8, CNTR_ALL },
	[PERF_COUNT_HW_LOAD_MISSQ_PREFETCH] = { 0x1d9, CNTR_ALL },
	[PERF_COUNT_HW_LOAD_MISSQ_PRE_SCREF] = { 0x1da, CNTR_ALL },
	[PERF_COUNT_HW_LOAD_MISSQ_PRE_RDY] = { 0x1db, CNTR_ALL },
	[PERF_COUNT_HW_LOAD_MISSQ_PRE_WAIT] = { 0x1dc, CNTR_ALL },
	[PERF_COUNT_HW_STORE_MISSQ_PRE_SCREF_LOAD] = { 0x1dd, CNTR_ALL },
	[PERF_COUNT_HW_STORE_MISSQ_PRE_RDY_SHARD] = { 0x1de, CNTR_ALL },
	[PERF_COUNT_HW_STORE_MISSQ_PRE_WAIT_LOAD] = { 0x1df, CNTR_ALL },
	[PERF_COUNT_HW_STORE_MISSQ_PRE_SCREF_STORE] = { 0x1e0, CNTR_ALL },
	[PERF_COUNT_HW_STORE_MISSQ_PRE_RDY_EXC] = { 0x1e1, CNTR_ALL },
	[PERF_COUNT_HW_STORE_MISSQ_PRE_WAIT_STORE] = { 0x1e2, CNTR_ALL },
	[PERF_COUNT_HW_STORE_MISSQ_PREFETCH] = { 0x1e3, CNTR_ALL },
	[PERF_COUNT_HW_STORE_MISSQ_VALID_PREFETCH] = { 0x1e4, CNTR_ALL },
	[PERF_COUNT_HW_ALL_REQ_MISSQ_PREFETCH] = { 0x1e5, CNTR_ALL },
	[PERF_COUNT_HW_ALL_REQ_MISSQ_PRE_SCREF] = { 0x1e6, CNTR_ALL },
	[PERF_COUNT_HW_ALL_REQ_MISSQ_PRE_RDY] = { 0x1e7, CNTR_ALL },
	[PERF_COUNT_HW_ALL_REQ_MISSQ_PRE_WAIT] = { 0x1e8, CNTR_ALL },
	[PERF_COUNT_HW_FETCH_MISSQ_PREFETCH] = { 0x1e9, CNTR_ALL },
	[PERF_COUNT_HW_FETCH_PRE_SCREF] = { 0x1ea, CNTR_ALL },
	[PERF_COUNT_HW_FETCH_PRE_RDY] = { 0x1eb, CNTR_ALL },
	[PERF_COUNT_HW_FETCH_PRE_WAIT] = { 0x1ec, CNTR_ALL },
	[PERF_COUNT_HW_DATA_FETCH_PRE_RDY] = { 0x1ef, CNTR_ALL },
	[PERF_COUNT_HW_DATA_FETCH_PRE_WAIT] = { 0x1f0, CNTR_ALL },
	[PERF_COUNT_HW_HW_LOAD_PRE_SCACHE_CANCEL] = { 0x1f1, CNTR_ALL },
	[PERF_COUNT_HW_HW_STORE_PRE_SCACHE_CANCEL] = { 0x1f2, CNTR_ALL },
	[PERF_COUNT_HW_HW_DATA_PRE_SCACHE_CANCEL] = { 0x1f3, CNTR_ALL },
	[PERF_COUNT_HW_HW_FETCH_PRE_SCACHE_CANCEL] = { 0x1f4, CNTR_ALL },
	[PERF_COUNT_HW_HW_PREFETCH_SCACHE_CANCEL] = { 0x1f5, CNTR_ALL },
	[PERF_COUNT_HW_HW_LOAD_PREFETCH] = { 0x1f6, CNTR_ALL },
	[PERF_COUNT_HW_HW_STORE_PREFETCH] = { 0x1f7, CNTR_ALL },
	[PERF_COUNT_HW_HW_DATA_PREFETCH] = { 0x1f8, CNTR_ALL },
	[PERF_COUNT_HW_HW_INST_PREFETCH] = { 0x1f9, CNTR_ALL },
	[PERF_COUNT_HW_HW_PREFETCH] = { 0x1fa, CNTR_ALL },
	[PERF_COUNT_HW_TAGGED_LOAD_PREFETCH] = { 0x1fb, CNTR_ALL },
	[PERF_COUNT_HW_MISS_LOAD_PREFETCH] = { 0x1fc, CNTR_ALL },
	[PERF_COUNT_HW_TAGGED_STORE_PREFETCH] = { 0x1fd, CNTR_ALL },
	[PERF_COUNT_HW_MISS_STORE_PREFETCH] = { 0x1fe, CNTR_ALL },
	[PERF_COUNT_HW_TAGGED_DATA_PREFETCH] = { 0x1ff, CNTR_ALL },
	[PERF_COUNT_HW_MISS_DATA_PREFETCH] = { 0x200, CNTR_ALL },
	[PERF_COUNT_HW_TAGGED_INST_PREFETCH] = { 0x201, CNTR_ALL },
	[PERF_COUNT_HW_MISS_INST_PREFETCH] = { 0x202, CNTR_ALL },
	[PERF_COUNT_HW_TAGGED_PREFETCH] = { 0x203, CNTR_ALL },
	[PERF_COUNT_HW_MISS_PREFETCH] = { 0x204, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_AC_LOAD_PREFETCH] = { 0x205, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_AC_STORE_PREFETCH] = { 0x206, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_AC_DATA_PREFETCH] = { 0x207, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_AC_INST_PREFETCH] = { 0x208, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_AC_PREFETCH] = { 0x209, CNTR_ALL },
	[PERF_COUNT_HW_SCACHE_LOAD_PREFETCH] = { 0x20a, CNTR_ALL },
	[PERF_COUNT_HW_SCACHE_STORE_PREFETCH] = { 0x20b, CNTR_ALL },
	[PERF_COUNT_HW_SCACHE_DATA_PREFETCH] = { 0x20c, CNTR_ALL },
	[PERF_COUNT_HW_SCACHE_INST_PREFETCH] = { 0x20d, CNTR_ALL },
	[PERF_COUNT_HW_SCACHE_VALID_PREFETCH] = { 0x20e, CNTR_ALL },
	[PERF_COUNT_HW_PRE_RDY_LOAD_PREFETCH] = { 0x20f, CNTR_ALL },
	[PERF_COUNT_HW_PRE_RDY_STORE_PREFETCH] = { 0x210, CNTR_ALL },
	[PERF_COUNT_HW_PRE_RDY_DATA_PREFETCH] = { 0x211, CNTR_ALL },
	[PERF_COUNT_HW_PRE_RDY_INST_PREFETCH] = { 0x212, CNTR_ALL },
	[PERF_COUNT_HW_PRE_RDY_PREFETCH] = { 0x213, CNTR_ALL },
	[PERF_COUNT_HW_PRE_RDY_LOAD_REQUEST] = { 0x214, CNTR_ALL },
	[PERF_COUNT_HW_PRE_RDY_STORE_REQUEST] = { 0x215, CNTR_ALL },
	[PERF_COUNT_HW_PRE_RDY_DATA_REQUEST] = { 0x216, CNTR_ALL },
	[PERF_COUNT_HW_PRE_RDY_INST_REQUEST] = { 0x217, CNTR_ALL },
	[PERF_COUNT_HW_PRE_RDY_REQUEST] = { 0x218, CNTR_ALL },
	[PERF_COUNT_HW_PRE_SCREF_HIT_LOAD_REQ] = { 0x219, CNTR_ALL },
	[PERF_COUNT_HW_PRE_SCREF_HIT_STORE_REQ] = { 0x21b, CNTR_ALL },
	[PERF_COUNT_HW_PRE_SCREF_HIT_DATA_REQ] = { 0x21c, CNTR_ALL },
	[PERF_COUNT_HW_PRE_SCREF_HIT_INST_REQ] = { 0x21d, CNTR_ALL },
	[PERF_COUNT_HW_PRE_SCREF_HIT_PREFETCH_REQ] = { 0x21e, CNTR_ALL },
	[PERF_COUNT_HW_PRE_SCREF_HIT_LOAD_PREFETCH] = { 0x21f, CNTR_ALL },
	[PERF_COUNT_HW_PRE_SCREF_HIT_STORE_PREFETCH] = { 0x220, CNTR_ALL },
	[PERF_COUNT_HW_PRE_SCREF_RDY_DATA_REQ] = { 0x221, CNTR_ALL },
	[PERF_COUNT_HW_PRE_SCREF_RDY_INST_REQ] = { 0x222, CNTR_ALL },
	[PERF_COUNT_HW_PRE_SCREF_RDY_PREFETCH] = { 0x223, CNTR_ALL },
	[PERF_COUNT_HW_PRE_SCREF_MISS_LOAD] = { 0x224, CNTR_ALL },
	[PERF_COUNT_HW_PRE_WAIT_HIT_LOAD_PREFETCH] = { 0x229, CNTR_ALL },
	[PERF_COUNT_HW_PRE_WAIT_HIT_STORE_PREFETCH] = { 0x22a, CNTR_ALL },
	[PERF_COUNT_HW_PRE_WAIT_HIT_DATA_PREFETCH] = { 0x22b, CNTR_ALL },
	[PERF_COUNT_HW_PRE_WAIT_HIT_INST_PREFETCH] = { 0x22c, CNTR_ALL },
	[PERF_COUNT_HW_PRE_WAIT_HIT_PREFETCH] = { 0x22d, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_REPLACE_PRE_WAIT_PREFEETCH] = { 0x22e, CNTR_ALL },
	[PERF_COUNT_HW_MISSQ_REPLACE_PRE_RDY_PREFEETCH] = { 0x22f, CNTR_ALL },
	[PERF_COUNT_HW_PREFETCH_INV] = { 0x230, CNTR_ALL },
	[PERF_COUNT_HW_LOAD_PREFETCH_OCCUPY] = { 0x231, CNTR_ALL },
	[PERF_COUNT_HW_LOAD_PREFETCH_ISOCCUPY] = { 0x232, CNTR_ALL },
	[PERF_COUNT_HW_STORE_PREFETCH_OCCUPY] = { 0x233, CNTR_ALL },
	[PERF_COUNT_HW_STORE_PREFETCH_ISOCCUPY] = { 0x234, CNTR_ALL },
	[PERF_COUNT_HW_DATA_PREFETCH_OCCUPY] = { 0x235, CNTR_ALL },
	[PERF_COUNT_HW_DATA_PREFETCH_ISOCCUPY] = { 0x236, CNTR_ALL },
	[PERF_COUNT_HW_INST_PREFETCH_OCCUPY] = { 0x237, CNTR_ALL },
	[PERF_COUNT_HW_INST_PREFETCH_ISOCCUPY] = { 0x238, CNTR_ALL },
	[PERF_COUNT_HW_LOAD_PRE_SCREF_PRE_RDY_HIT] = { 0x239, CNTR_ALL },
	[PERF_COUNT_HW_STORE_PRE_SCREF_PRE_RDY_HIT] = { 0x23a, CNTR_ALL },
	[PERF_COUNT_HW_DATA_PRE_SCREF_PRE_RDY_HIT] = { 0x23b, CNTR_ALL },
	[PERF_COUNT_HW_INST_PRE_SCREF_PRE_RDY_HIT] = { 0x23c, CNTR_ALL },
	[PERF_COUNT_HW_PREFETCH_PRE_SCREF_PRE_RDY_HIT] = { 0x23d, CNTR_ALL },
#endif
};
/* 24K/34K/1004K cores can share the same cache event map. */
static const struct mips_perf_event mipsxxcore_cache_map
				[PERF_COUNT_HW_CACHE_MAX]
				[PERF_COUNT_HW_CACHE_OP_MAX]
				[PERF_COUNT_HW_CACHE_RESULT_MAX] = {
[C(L1D)] = {
	/*
	 * Like some other architectures (e.g. ARM), the performance
	 * counters don't differentiate between read and write
	 * accesses/misses, so this isn't strictly correct, but it's the
	 * best we can do. Writes and reads get combined.
	 */
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { 0x0a, CNTR_EVEN, T },
		[C(RESULT_MISS)]	= { 0x0b, CNTR_EVEN | CNTR_ODD, T },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { 0x0a, CNTR_EVEN, T },
		[C(RESULT_MISS)]	= { 0x0b, CNTR_EVEN | CNTR_ODD, T },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(L1I)] = {
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { 0x09, CNTR_EVEN, T },
		[C(RESULT_MISS)]	= { 0x09, CNTR_ODD, T },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { 0x09, CNTR_EVEN, T },
		[C(RESULT_MISS)]	= { 0x09, CNTR_ODD, T },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { 0x14, CNTR_EVEN, T },
		/*
		 * Note that MIPS has only "hit" events countable for
		 * the prefetch operation.
		 */
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(LL)] = {
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { 0x15, CNTR_ODD, P },
		[C(RESULT_MISS)]	= { 0x16, CNTR_EVEN, P },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { 0x15, CNTR_ODD, P },
		[C(RESULT_MISS)]	= { 0x16, CNTR_EVEN, P },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(DTLB)] = {
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { 0x06, CNTR_EVEN, T },
		[C(RESULT_MISS)]	= { 0x06, CNTR_ODD, T },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { 0x06, CNTR_EVEN, T },
		[C(RESULT_MISS)]	= { 0x06, CNTR_ODD, T },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(ITLB)] = {
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { 0x05, CNTR_EVEN, T },
		[C(RESULT_MISS)]	= { 0x05, CNTR_ODD, T },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { 0x05, CNTR_EVEN, T },
		[C(RESULT_MISS)]	= { 0x05, CNTR_ODD, T },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(BPU)] = {
	/* Using the same code for *HW_BRANCH* */
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { 0x02, CNTR_EVEN, T },
		[C(RESULT_MISS)]	= { 0x02, CNTR_ODD, T },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { 0x02, CNTR_EVEN, T },
		[C(RESULT_MISS)]	= { 0x02, CNTR_ODD, T },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(NODE)] = {
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
};

/* 74K core has completely different cache event map. */
static const struct mips_perf_event mipsxx74Kcore_cache_map
				[PERF_COUNT_HW_CACHE_MAX]
				[PERF_COUNT_HW_CACHE_OP_MAX]
				[PERF_COUNT_HW_CACHE_RESULT_MAX] = {
[C(L1D)] = {
	/*
	 * Like some other architectures (e.g. ARM), the performance
	 * counters don't differentiate between read and write
	 * accesses/misses, so this isn't strictly correct, but it's the
	 * best we can do. Writes and reads get combined.
	 */
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { 0x17, CNTR_ODD, T },
		[C(RESULT_MISS)]	= { 0x18, CNTR_ODD, T },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { 0x17, CNTR_ODD, T },
		[C(RESULT_MISS)]	= { 0x18, CNTR_ODD, T },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(L1I)] = {
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { 0x06, CNTR_EVEN, T },
		[C(RESULT_MISS)]	= { 0x06, CNTR_ODD, T },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { 0x06, CNTR_EVEN, T },
		[C(RESULT_MISS)]	= { 0x06, CNTR_ODD, T },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { 0x34, CNTR_EVEN, T },
		/*
		 * Note that MIPS has only "hit" events countable for
		 * the prefetch operation.
		 */
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(LL)] = {
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { 0x1c, CNTR_ODD, P },
		[C(RESULT_MISS)]	= { 0x1d, CNTR_EVEN | CNTR_ODD, P },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { 0x1c, CNTR_ODD, P },
		[C(RESULT_MISS)]	= { 0x1d, CNTR_EVEN | CNTR_ODD, P },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(DTLB)] = {
	/* 74K core does not have specific DTLB events. */
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(ITLB)] = {
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { 0x04, CNTR_EVEN, T },
		[C(RESULT_MISS)]	= { 0x04, CNTR_ODD, T },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { 0x04, CNTR_EVEN, T },
		[C(RESULT_MISS)]	= { 0x04, CNTR_ODD, T },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(BPU)] = {
	/* Using the same code for *HW_BRANCH* */
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { 0x27, CNTR_EVEN, T },
		[C(RESULT_MISS)]	= { 0x27, CNTR_ODD, T },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { 0x27, CNTR_EVEN, T },
		[C(RESULT_MISS)]	= { 0x27, CNTR_ODD, T },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(NODE)] = {
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
};


static const struct mips_perf_event octeon_cache_map
				[PERF_COUNT_HW_CACHE_MAX]
				[PERF_COUNT_HW_CACHE_OP_MAX]
				[PERF_COUNT_HW_CACHE_RESULT_MAX] = {
[C(L1D)] = {
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { 0x2b, CNTR_ALL },
		[C(RESULT_MISS)]	= { 0x2e, CNTR_ALL },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { 0x30, CNTR_ALL },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(L1I)] = {
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { 0x18, CNTR_ALL },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { 0x19, CNTR_ALL },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(LL)] = {
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(DTLB)] = {
	/*
	 * Only general DTLB misses are counted use the same event for
	 * read and write.
	 */
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { 0x35, CNTR_ALL },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { 0x35, CNTR_ALL },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(ITLB)] = {
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { 0x37, CNTR_ALL },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(BPU)] = {
	/* Using the same code for *HW_BRANCH* */
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
};

static const struct mips_perf_event loongson3a2000_cache_map
				[PERF_COUNT_HW_CACHE_MAX]
				[PERF_COUNT_HW_CACHE_OP_MAX]
				[PERF_COUNT_HW_CACHE_RESULT_MAX] = {
[C(L1D)] = {
	/*
	 * Like some other architectures (e.g. ARM), the performance
	 * counters don't differentiate between read and write
	 * accesses/misses, so this isn't strictly correct, but it's the
	 * best we can do. Writes and reads get combined.
	 */
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { 0x04, CNTR_ODD },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { 0x04, CNTR_ODD },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(L1I)] = {
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { 0x17, CNTR_ALL },
		[C(RESULT_MISS)]	= { 0x18, CNTR_ALL },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		/*
		 * Note that MIPS has only "hit" events countable for
		 * the prefetch operation.
		 */
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(LL)] = {
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { 0x1b6, CNTR_ALL },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { 0x1b7, CNTR_ALL },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { 0x1bf, CNTR_ALL },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(DTLB)] = {
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { 0x92, CNTR_ODD },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(ITLB)] = {
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { 0x1a, CNTR_ALL },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(BPU)] = {
	/* Using the same code for *HW_BRANCH* */
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { 0x02, CNTR_EVEN },
		[C(RESULT_MISS)]	= { 0x02, CNTR_ODD },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { 0x02, CNTR_EVEN },
		[C(RESULT_MISS)]	= { 0x02, CNTR_ODD },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(NODE)] = {
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
};

static const struct mips_perf_event loongson3_cache_map
				[PERF_COUNT_HW_CACHE_MAX]
				[PERF_COUNT_HW_CACHE_OP_MAX]
				[PERF_COUNT_HW_CACHE_RESULT_MAX] = {
[C(L1D)] = {
	/*
	 * Like some other architectures (e.g. ARM), the performance
	 * counters don't differentiate between read and write
	 * accesses/misses, so this isn't strictly correct, but it's the
	 * best we can do. Writes and reads get combined.
	 */
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { 0x04, CNTR_ODD, T },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { 0x04, CNTR_ODD, T },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(L1I)] = {
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { 0x04, CNTR_EVEN, T },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { 0x04, CNTR_EVEN, T },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		/*
		 * Note that MIPS has only "hit" events countable for
		 * the prefetch operation.
		 */
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(LL)] = {
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(DTLB)] = {
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { 0x09, CNTR_ODD, P },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { 0x09, CNTR_ODD, P },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(ITLB)] = {
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { 0x0c, CNTR_ODD, P },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { 0x0c, CNTR_ODD, P },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(BPU)] = {
	/* Using the same code for *HW_BRANCH* */
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { 0x02, CNTR_EVEN, T },
		[C(RESULT_MISS)]	= { 0x02, CNTR_ODD, T },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { 0x02, CNTR_EVEN, T },
		[C(RESULT_MISS)]	= { 0x02, CNTR_ODD, T },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
[C(NODE)] = {
	[C(OP_READ)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_WRITE)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
	[C(OP_PREFETCH)] = {
		[C(RESULT_ACCESS)]	= { UNSUPPORTED_PERF_EVENT_ID },
		[C(RESULT_MISS)]	= { UNSUPPORTED_PERF_EVENT_ID },
	},
},
};
#ifdef CONFIG_MIPS_MT_SMP
static void check_and_calc_range(struct perf_event *event,
				 const struct mips_perf_event *pev)
{
	struct hw_perf_event *hwc = &event->hw;

	if (event->cpu >= 0) {
		if (pev->range > V) {
			/*
			 * The user selected an event that is processor
			 * wide, while expecting it to be VPE wide.
			 */
			hwc->config_base |= M_TC_EN_ALL;
		} else {
			/*
			 * FIXME: cpu_data[event->cpu].vpe_id reports 0
			 * for both CPUs.
			 */
			hwc->config_base |= M_PERFCTL_VPEID(event->cpu);
			hwc->config_base |= M_TC_EN_VPE;
		}
	} else
		hwc->config_base |= M_TC_EN_ALL;
}
#else
static void check_and_calc_range(struct perf_event *event,
				 const struct mips_perf_event *pev)
{
}
#endif

static int __hw_perf_event_init(struct perf_event *event)
{
	struct perf_event_attr *attr = &event->attr;
	struct hw_perf_event *hwc = &event->hw;
	const struct mips_perf_event *pev;
	int err;

	/* Returning MIPS event descriptor for generic perf event. */
	if (PERF_TYPE_HARDWARE == event->attr.type) {
		if (event->attr.config >= PERF_COUNT_HW_MAX)
			return -EINVAL;
		pev = mipspmu_map_general_event(event->attr.config);
	} else if (PERF_TYPE_HW_CACHE == event->attr.type) {
		pev = mipspmu_map_cache_event(event->attr.config);
	} else if (PERF_TYPE_RAW == event->attr.type) {
		/* We are working on the global raw event. */
		mutex_lock(&raw_event_mutex);
		pev = mipspmu.map_raw_event(event->attr.config);
	} else {
		/* The event type is not (yet) supported. */
		return -EOPNOTSUPP;
	}

	if (IS_ERR(pev)) {
		if (PERF_TYPE_RAW == event->attr.type)
			mutex_unlock(&raw_event_mutex);
		return PTR_ERR(pev);
	}

	/*
	 * We allow max flexibility on how each individual counter shared
	 * by the single CPU operates (the mode exclusion and the range).
	 */
	hwc->config_base = M_PERFCTL_INTERRUPT_ENABLE;

	/* Calculate range bits and validate it. */
	if (num_possible_cpus() > 1)
		check_and_calc_range(event, pev);

	hwc->event_base = mipspmu_perf_event_encode(pev);
	if (PERF_TYPE_RAW == event->attr.type)
		mutex_unlock(&raw_event_mutex);

	if (!attr->exclude_user)
		hwc->config_base |= M_PERFCTL_USER;
	if (!attr->exclude_kernel) {
		hwc->config_base |= M_PERFCTL_KERNEL;
		/* MIPS kernel mode: KSU == 00b || EXL == 1 || ERL == 1 */
		hwc->config_base |= M_PERFCTL_EXL;
	}
	if (!attr->exclude_hv)
		hwc->config_base |= M_PERFCTL_SUPERVISOR;

	hwc->config_base &= M_PERFCTL_CONFIG_MASK;
	/*
	 * The event can belong to another cpu. We do not assign a local
	 * counter for it for now.
	 */
	hwc->idx = -1;
	hwc->config = 0;

	if (!hwc->sample_period) {
		hwc->sample_period  = mipspmu.max_period;
		hwc->last_period    = hwc->sample_period;
		local64_set(&hwc->period_left, hwc->sample_period);
	}

	err = 0;
	if (event->group_leader != event)
		err = validate_group(event);

	event->destroy = hw_perf_event_destroy;

	if (err)
		event->destroy(event);

	return err;
}

static void pause_local_counters(void)
{
	struct cpu_hw_events *cpuc = &__get_cpu_var(cpu_hw_events);
	int ctr = mipspmu.num_counters;
	unsigned long flags;

	local_irq_save(flags);
	do {
		ctr--;
		cpuc->saved_ctrl[ctr] = mipsxx_pmu_read_control(ctr);
		mipsxx_pmu_write_control(ctr, cpuc->saved_ctrl[ctr] &
					 ~M_PERFCTL_COUNT_EVENT_WHENEVER);
	} while (ctr > 0);
	local_irq_restore(flags);
}

static void resume_local_counters(void)
{
	struct cpu_hw_events *cpuc = &__get_cpu_var(cpu_hw_events);
	int ctr = mipspmu.num_counters;

	do {
		ctr--;
		mipsxx_pmu_write_control(ctr, cpuc->saved_ctrl[ctr]);
	} while (ctr > 0);
}

static int mipsxx_pmu_handle_shared_irq(void)
{
	struct cpu_hw_events *cpuc = &__get_cpu_var(cpu_hw_events);
	struct perf_sample_data data;
	unsigned int counters = mipspmu.num_counters;
	u64 counter;
	int handled = IRQ_NONE;
	struct pt_regs *regs;

	if (cpu_has_perf_cntr_intr_bit && !(read_c0_cause() & CAUSEF_PCI))
		return handled;
	/*
	 * First we pause the local counters, so that when we are locked
	 * here, the counters are all paused. When it gets locked due to
	 * perf_disable(), the timer interrupt handler will be delayed.
	 *
	 * See also mipsxx_pmu_start().
	 */
	pause_local_counters();
#ifdef CONFIG_MIPS_MT_SMP
	read_lock(&pmuint_rwlock);
#endif

	regs = get_irq_regs();

	perf_sample_data_init(&data, 0, 0);

	switch (counters) {
#define HANDLE_COUNTER(n)						\
	case n + 1:							\
		if (test_bit(n, cpuc->used_mask)) {			\
			counter = mipspmu.read_counter(n);		\
			if (counter & mipspmu.overflow) {		\
				handle_associated_event(cpuc, n, &data, regs); \
				handled = IRQ_HANDLED;			\
			}						\
		}
	HANDLE_COUNTER(3)
	HANDLE_COUNTER(2)
	HANDLE_COUNTER(1)
	HANDLE_COUNTER(0)
	}

	/*
	 * Do all the work for the pending perf events. We can do this
	 * in here because the performance counter interrupt is a regular
	 * interrupt, not NMI.
	 */
	if (handled == IRQ_HANDLED)
		irq_work_run();

#ifdef CONFIG_MIPS_MT_SMP
	read_unlock(&pmuint_rwlock);
#endif
	resume_local_counters();
	return handled;
}

static irqreturn_t mipsxx_pmu_handle_irq(int irq, void *dev)
{
	return mipsxx_pmu_handle_shared_irq();
}

/* 24K */
#define IS_BOTH_COUNTERS_24K_EVENT(b)					\
	((b) == 0 || (b) == 1 || (b) == 11)

/* 34K */
#define IS_BOTH_COUNTERS_34K_EVENT(b)					\
	((b) == 0 || (b) == 1 || (b) == 11)
#ifdef CONFIG_MIPS_MT_SMP
#define IS_RANGE_P_34K_EVENT(r, b)					\
	((b) == 0 || (r) == 18 || (b) == 21 || (b) == 22 ||		\
	 (b) == 25 || (b) == 39 || (r) == 44 || (r) == 174 ||		\
	 (r) == 176 || ((b) >= 50 && (b) <= 55) ||			\
	 ((b) >= 64 && (b) <= 67))
#define IS_RANGE_V_34K_EVENT(r)	((r) == 47)
#endif

/* 74K */
#define IS_BOTH_COUNTERS_74K_EVENT(b)					\
	((b) == 0 || (b) == 1)

/* 1004K */
#define IS_BOTH_COUNTERS_1004K_EVENT(b)					\
	((b) == 0 || (b) == 1 || (b) == 11)
#ifdef CONFIG_MIPS_MT_SMP
#define IS_RANGE_P_1004K_EVENT(r, b)					\
	((b) == 0 || (r) == 18 || (b) == 21 || (b) == 22 ||		\
	 (b) == 25 || (b) == 36 || (b) == 39 || (r) == 44 ||		\
	 (r) == 174 || (r) == 176 || ((b) >= 50 && (b) <= 59) ||	\
	 (r) == 188 || (b) == 61 || (b) == 62 ||			\
	 ((b) >= 64 && (b) <= 67))
#define IS_RANGE_V_1004K_EVENT(r)	((r) == 47)
#endif

/*
 * User can use 0-255 raw events, where 0-127 for the events of even
 * counters, and 128-255 for odd counters. Note that bit 7 is used to
 * indicate the parity. So, for example, when user wants to take the
 * Event Num of 15 for odd counters (by referring to the user manual),
 * then 128 needs to be added to 15 as the input for the event config,
 * i.e., 143 (0x8F) to be used.
 */
static const struct mips_perf_event *mipsxx_pmu_map_raw_event(u64 config)
{
	unsigned int raw_id = config & 0xff;
	unsigned int base_id = raw_id & 0x7f;

	raw_event.event_id = base_id;

	switch (current_cpu_type()) {
	case CPU_24K:
		if (IS_BOTH_COUNTERS_24K_EVENT(base_id))
			raw_event.cntr_mask = CNTR_EVEN | CNTR_ODD;
		else
			raw_event.cntr_mask =
				raw_id > 127 ? CNTR_ODD : CNTR_EVEN;
#ifdef CONFIG_MIPS_MT_SMP
		/*
		 * This is actually doing nothing. Non-multithreading
		 * CPUs will not check and calculate the range.
		 */
		raw_event.range = P;
#endif
		break;
	case CPU_34K:
		if (IS_BOTH_COUNTERS_34K_EVENT(base_id))
			raw_event.cntr_mask = CNTR_EVEN | CNTR_ODD;
		else
			raw_event.cntr_mask =
				raw_id > 127 ? CNTR_ODD : CNTR_EVEN;
#ifdef CONFIG_MIPS_MT_SMP
		if (IS_RANGE_P_34K_EVENT(raw_id, base_id))
			raw_event.range = P;
		else if (unlikely(IS_RANGE_V_34K_EVENT(raw_id)))
			raw_event.range = V;
		else
			raw_event.range = T;
#endif
		break;
	case CPU_74K:
		if (IS_BOTH_COUNTERS_74K_EVENT(base_id))
			raw_event.cntr_mask = CNTR_EVEN | CNTR_ODD;
		else
			raw_event.cntr_mask =
				raw_id > 127 ? CNTR_ODD : CNTR_EVEN;
#ifdef CONFIG_MIPS_MT_SMP
		raw_event.range = P;
#endif
		break;
	case CPU_1004K:
		if (IS_BOTH_COUNTERS_1004K_EVENT(base_id))
			raw_event.cntr_mask = CNTR_EVEN | CNTR_ODD;
		else
			raw_event.cntr_mask =
				raw_id > 127 ? CNTR_ODD : CNTR_EVEN;
#ifdef CONFIG_MIPS_MT_SMP
		if (IS_RANGE_P_1004K_EVENT(raw_id, base_id))
			raw_event.range = P;
		else if (unlikely(IS_RANGE_V_1004K_EVENT(raw_id)))
			raw_event.range = V;
		else
			raw_event.range = T;
#endif
		break;

	case CPU_LOONGSON3:
		raw_event.cntr_mask =
			raw_id > 127 ? CNTR_ODD : CNTR_EVEN;
		break;

	}

	return &raw_event;
}

static const struct mips_perf_event *octeon_pmu_map_raw_event(u64 config)
{
	unsigned int raw_id = config & 0xff;
	unsigned int base_id = raw_id & 0x7f;


	raw_event.cntr_mask = CNTR_ALL;
	raw_event.event_id = base_id;

	if (current_cpu_type() == CPU_CAVIUM_OCTEON2) {
		if (base_id > 0x42)
			return ERR_PTR(-EOPNOTSUPP);
	} else {
		if (base_id > 0x3a)
			return ERR_PTR(-EOPNOTSUPP);
	}

	switch (base_id) {
	case 0x00:
	case 0x0f:
	case 0x1e:
	case 0x1f:
	case 0x2f:
	case 0x34:
	case 0x3b ... 0x3f:
		return ERR_PTR(-EOPNOTSUPP);
	default:
		break;
	}

	return &raw_event;
}

static const struct mips_perf_event *loongson3a2000_pmu_map_raw_event(u64 config)
{

	unsigned int evt_id = config & 0x3ff;


	raw_event.cntr_mask = CNTR_ALL;
	raw_event.event_id = evt_id;

	if ((evt_id >= 1 && evt_id < 28) || 
		(evt_id >= 64 && evt_id < 90) ||
		(evt_id >= 128 && evt_id < 164) ||
		(evt_id >= 192 && evt_id < 200) ||
		(evt_id >= 256 && evt_id < 274) ||
		(evt_id >= 320 && evt_id < 358) ||
		(evt_id >= 384 && evt_id < 574))
		return &raw_event;
	else
		return ERR_PTR(-EOPNOTSUPP);
}

static int __init
init_hw_perf_events(void)
{
	int counters, irq;
	int counter_bits;

	counters = n_counters();
	if (counters == 0) {
		pr_cont("No available PMU.\n");
		return -ENODEV;
	}

#ifdef CONFIG_MIPS_MT_SMP
	cpu_has_mipsmt_pertccounters = read_c0_config7() & (1<<19);
	if (!cpu_has_mipsmt_pertccounters)
		counters = counters_total_to_per_cpu(counters);
#endif

#ifdef MSC01E_INT_BASE
	if (cpu_has_veic) {
		/*
		 * Using platform specific interrupt controller defines.
		 */
		irq = MSC01E_INT_BASE + MSC01E_INT_PERFCTR;
	} else {
#endif
		if ((cp0_perfcount_irq >= 0) &&
				(cp0_compare_irq != cp0_perfcount_irq))
			irq = MIPS_CPU_IRQ_BASE + cp0_perfcount_irq;
		else
			irq = -1;
#ifdef MSC01E_INT_BASE
	}
#endif

	mipspmu.map_raw_event = mipsxx_pmu_map_raw_event;

	switch (current_cpu_type()) {
	case CPU_24K:
		mipspmu.name = "mips/24K";
		mipspmu.general_event_map = &mipsxxcore_event_map;
		mipspmu.cache_event_map = &mipsxxcore_cache_map;
		break;
	case CPU_34K:
		mipspmu.name = "mips/34K";
		mipspmu.general_event_map = &mipsxxcore_event_map;
		mipspmu.cache_event_map = &mipsxxcore_cache_map;
		break;
	case CPU_74K:
		mipspmu.name = "mips/74K";
		mipspmu.general_event_map = &mipsxx74Kcore_event_map;
		mipspmu.cache_event_map = &mipsxx74Kcore_cache_map;
		break;
	case CPU_1004K:
		mipspmu.name = "mips/1004K";
		mipspmu.general_event_map = &mipsxxcore_event_map;
		mipspmu.cache_event_map = &mipsxxcore_cache_map;
		break;
	case CPU_LOONGSON1:
		mipspmu.name = "mips/loongson1";
		mipspmu.general_event_map = &mipsxxcore_event_map;
		mipspmu.cache_event_map = &mipsxxcore_cache_map;
		break;
	case CPU_CAVIUM_OCTEON:
	case CPU_CAVIUM_OCTEON_PLUS:
	case CPU_CAVIUM_OCTEON2:
		mipspmu.name = "octeon";
		mipspmu.general_event_map = &octeon_event_map;
		mipspmu.cache_event_map = &octeon_cache_map;
		mipspmu.map_raw_event = octeon_pmu_map_raw_event;
		break;
	case CPU_LOONGSON3:
		if (((read_c0_prid() & 0xf) == PRID_REV_LOONGSON3A2000)){

			mipspmu.name = "loongson3a2000";
			mipspmu.general_event_map = &loongson3a2000_event_map;
			mipspmu.cache_event_map = &loongson3a2000_cache_map;
			mipspmu.map_raw_event = loongson3a2000_pmu_map_raw_event;
			counters = 4;
		}
		else if (((read_c0_prid() & 0xf) == PRID_REV_LOONGSON3A3000)){
			mipspmu.name = "loongson3a3000";
			mipspmu.general_event_map = &loongson3a2000_event_map;
			mipspmu.cache_event_map = &loongson3a2000_cache_map;
			mipspmu.map_raw_event = loongson3a2000_pmu_map_raw_event;
			counters = 4;
		}
		else{
			mipspmu.name = "loongson3";
			mipspmu.general_event_map = &loongson3_event_map;
			mipspmu.cache_event_map = &loongson3_cache_map;
			counters = 2;
		}
		break;
	default:
		pr_cont("Either hardware does not support performance "
			"counters, or not yet implemented.\n");
		return -ENODEV;
	}

	mipspmu.num_counters = counters;
	mipspmu.irq = irq;

	if (read_c0_perfctrl0() & M_PERFCTL_WIDE) {
		if (((current_cpu_data.processor_id & PRID_REV_MASK) == PRID_REV_LOONGSON3A2000) ||
                    ((current_cpu_data.processor_id & PRID_REV_MASK) == PRID_REV_LOONGSON3A3000)) {
			mipspmu.write_counter = mipsxx_pmu_write_counter_48;
			mipspmu.read_counter = mipsxx_pmu_read_counter_48;
			mipspmu.max_period = (1ULL << 47) - 1;
			mipspmu.valid_count = (1ULL << 47) - 1;
			mipspmu.overflow = 1ULL << 47;
			counter_bits = 48;

		} else {
			mipspmu.write_counter = mipsxx_pmu_write_counter_64;
			mipspmu.read_counter = mipsxx_pmu_read_counter_64;
			mipspmu.max_period = (1ULL << 63) - 1;
			mipspmu.valid_count = (1ULL << 63) - 1;
			mipspmu.overflow = 1ULL << 63;
			counter_bits = 64;
		}
	} else {
		mipspmu.max_period = (1ULL << 31) - 1;
		mipspmu.valid_count = (1ULL << 31) - 1;
		mipspmu.overflow = 1ULL << 31;
		mipspmu.read_counter = mipsxx_pmu_read_counter;
		mipspmu.write_counter = mipsxx_pmu_write_counter;
		counter_bits = 32;
	}

	on_each_cpu(reset_counters, (void *)(long)counters, 1);

	pr_cont("%s PMU enabled, %d %d-bit counters available to each "
		"CPU, irq %d%s\n", mipspmu.name, counters, counter_bits, irq,
		irq < 0 ? " (share with timer interrupt)" : "");

	perf_pmu_register(&pmu, "cpu", PERF_TYPE_RAW);

	return 0;
}
early_initcall(init_hw_perf_events);

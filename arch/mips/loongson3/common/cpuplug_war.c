/*
 * CPU Autoplug driver for the Loongson-3 processors
 *
 * Copyright (C) 2010 - 2012 Lemote Inc.
 * Author: Huacai Chen, chenhc@lemote.com
 *         Xiaofu Meng, mengxiaofu@ict.ac.cn
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/sched.h>
#include <linux/tick.h>
#include <linux/kernel_stat.h>
#include <linux/platform_device.h>
#include <asm/clock.h>
#include <loongson.h>

void increase_cores(int cur_cpus)
{
	int target_cpu;

	if (cur_cpus >= nr_cpus_loongson)
		return;

	target_cpu = cur_cpus;
	lock_device_hotplug();
	cpu_up(target_cpu);
	unlock_device_hotplug();
}

void decrease_cores(int cur_cpus)
{
	int target_cpu;

	if (cur_cpus < nr_cpus_online)
		return;

	target_cpu = cur_cpus;
	lock_device_hotplug();
	cpu_down(target_cpu);
	unlock_device_hotplug();
}

static int __init cpuplug_init_war(void)
{
	int i;
	for (i = nr_cpus_online; i< nr_cpus_loongson; i++)
		if (reserved_cpus_mask & (1<<cpu_logical_map(i)))
			decrease_cores(i);
	return i;
}

late_initcall(cpuplug_init_war);

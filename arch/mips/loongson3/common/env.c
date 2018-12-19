/*
 * Based on Ocelot Linux port, which is
 * Copyright 2001 MontaVista Software Inc.
 * Author: jsun@mvista.com or jsun@junsun.net
 *
 * Copyright 2003 ICT CAS
 * Author: Michael Guo <guoyi@ict.ac.cn>
 *
 * Copyright (C) 2007 Lemote Inc. & Insititute of Computing Technology
 * Author: Fuxin Zhang, zhangfx@lemote.com
 *
 * Copyright (C) 2009 Lemote Inc.
 * Author: Wu Zhangjin, wuzhangjin@gmail.com
 *
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/module.h>
#include <asm/bootinfo.h>
#include <loongson.h>
#include <south-bridge.h>
#include <boot_param.h>
#include <ls2h/ls2h.h>

struct boot_params *boot_p;
struct loongson_params *loongson_p;

struct efi_cpuinfo_loongson *ecpu;
struct efi_memory_map_loongson *emap;
struct system_loongson *esys;
struct irq_source_routing_table *eirq_source;

u64 ht_control_base;
u64 io_base_regs_addr;
u64 pci_mem_start_addr, pci_mem_end_addr;
u64 loongson_pciio_base;
u64 vgabios_addr;
#if     defined(CONFIG_CPU_LOONGSON3)&&defined(CONFIG_SUSPEND)
u64 suspend_addr;
#endif
u64 poweroff_addr, restart_addr;
u32 nr_cpus_online;
u32 pcidev_max_func_num;

u64 loongson_chipcfg[MAX_PACKAGES];

unsigned long long smp_group[4];
unsigned int has_systab = 0;
unsigned long systab_addr;

u16 boot_cpu_id;
u16 reserved_cpus_mask;
u32 dma64_supported;
enum loongson_cpu_type cputype;
enum board_type board_type;
EXPORT_SYMBOL(board_type);
u32 nr_cpus_loongson = NR_CPUS;
u32 nr_nodes_loongson = MAX_NUMNODES;
int cores_per_node;
int cores_per_package;
unsigned long uma_vram_addr;
unsigned long uma_vram_size;
EXPORT_SYMBOL(cores_per_node);

u32 cpu_clock_freq;
EXPORT_SYMBOL(cpu_clock_freq);

unsigned long long lpc_reg_base;
struct interface_info *einter;
struct board_devices *eboard;
struct loongson_special_attribute *especial;
extern char *bios_vendor;
extern char *bios_release_date;
extern char *board_manufacturer;
extern char _bios_info[];
extern char _board_info[];
extern char __bios_info[];
extern char __board_info[];
extern char _bios_release_date[];
extern unsigned short biosrom_size;
extern struct south_bridge ls2h_south_bridge;
extern struct south_bridge rs780_south_bridge;

#define parse_even_earlier(res, option, p)				\
do {									\
	unsigned int tmp __maybe_unused;				\
									\
	if (strncmp(option, (char *)p, strlen(option)) == 0)		\
		tmp = kstrtou32((char *)p + strlen(option"="), 10, &res); \
} while (0)

void __init prom_init_env(void)
{
	/* pmon passes arguments in 32bit pointers */
	unsigned int processor_id;
	int cpu, tmp, i = 0, num = 0;
	char *bios_info;
	char *board_info;

#ifndef CONFIG_UEFI_FIRMWARE_INTERFACE
	int *_prom_envp;
	long l;
	extern u32 memsize, highmemsize;

	/* firmware arguments are initialized in head.S */
	_prom_envp = (int *)fw_arg2;

	l = (long)*_prom_envp;
	while (l != 0) {
		parse_even_earlier(cpu_clock_freq, "cpuclock", l);
		parse_even_earlier(memsize, "memsize", l);
		parse_even_earlier(highmemsize, "highmemsize", l);
		_prom_envp++;
		l = (long)*_prom_envp;
	}
	if (memsize == 0)
		memsize = 256;
#else
	/* firmware arguments are initialized in head.S */
	boot_p = (struct boot_params *)fw_arg2;
	loongson_p = &(boot_p->efi.smbios.lp);

	ecpu	= (struct efi_cpuinfo_loongson *)((u64)loongson_p + loongson_p->cpu_offset);
	emap	= (struct efi_memory_map_loongson *)((u64)loongson_p + loongson_p->memory_offset);
	eirq_source = (struct irq_source_routing_table *)((u64)loongson_p + loongson_p->irq_offset);
	einter = (struct interface_info *)
		((u64)loongson_p + loongson_p->interface_offset);
	eboard = (struct board_devices *)
		((u64)loongson_p + loongson_p->boarddev_table_offset);
	especial = (struct loongson_special_attribute *)
		((u64)loongson_p + loongson_p->special_offset);

	cputype = ecpu->cputype;
	if (cputype == Loongson_3A) {
		cores_per_node = 4;
		cores_per_package = 4;
		smp_group[0] = 0x900000003ff01000;
		smp_group[1] = 0x900010003ff01000;
		smp_group[2] = 0x900020003ff01000;
		smp_group[3] = 0x900030003ff01000;
		ht_control_base = 0x90000EFDFB000000;
		io_base_regs_addr = 0x900000003ff00000;
		loongson_chipcfg[0] = 0x900000001fe00180;
		loongson_chipcfg[1] = 0x900010001fe00180;
		loongson_chipcfg[2] = 0x900020001fe00180;
		loongson_chipcfg[3] = 0x900030001fe00180;
	}
	else if (cputype == Loongson_3B) {
		cores_per_node = 4;
		cores_per_package = 8;
		smp_group[0] = 0x900000003ff01000;
		smp_group[1] = 0x900010003ff05000;
		smp_group[2] = 0x900020003ff09000;
		smp_group[3] = 0x900030003ff0d000;
		ht_control_base = 0x90001EFDFB000000;
		loongson_chipcfg[0] = 0x900000001fe001d0;
		loongson_chipcfg[1] = 0x900020001fe001d0;
		loongson_chipcfg[2] = 0x900040001fe001d0;
		loongson_chipcfg[3] = 0x900060001fe001d0;
	}
	else {
		cores_per_node = 1;
		cores_per_package = 1;
		loongson_chipcfg[0] = 0x900000001fe00180;
	}
	tmp = (read_c0_prid() & 0xf);
	if ((ecpu->cputype == Loongson_3A) && ((tmp == PRID_REV_LOONGSON3A2000) || (tmp == PRID_REV_LOONGSON3A3000))) {
		loongson_chipcfg[0] = 0x900000001fe001d0;
		loongson_chipcfg[1] = 0x900010001fe001d0;
		loongson_chipcfg[2] = 0x900020001fe001d0;
		loongson_chipcfg[3] = 0x900030001fe001d0;
	}

	nr_cpus_loongson = ecpu->nr_cpus;
	cpu_clock_freq = ecpu->cpu_clock_freq;
	boot_cpu_id = ecpu->cpu_startup_core_id;
	reserved_cpus_mask = ecpu->reserved_cores_mask;
	/* fixup reserved cpus mask */
	if (reserved_cpus_mask == 0 &&nr_cpus_loongson != 16) {
		pr_info("WARNING: reserved_cpus_mask NOT set!\n");
		switch (nr_cpus_loongson) {
		case 4:
			reserved_cpus_mask = 0xfff0;
			break;
		case 6:
			reserved_cpus_mask = 0xff11;
			break;
		case 8:
			reserved_cpus_mask = 0xff00;
			break;
		default:
			reserved_cpus_mask = 0xfff0;
			break;
		}
	}
	pr_info("boot_cpu_id: %d, reserved_cpus_mask: %x\n",
			boot_cpu_id, reserved_cpus_mask);

	if (nr_cpus_loongson > NR_CPUS || nr_cpus_loongson == 0)
		nr_cpus_loongson = NR_CPUS;
	nr_nodes_loongson = (nr_cpus_loongson + cores_per_node - 1) / cores_per_node;

	for (nr_cpus_online = 0, cpu = 0; cpu < nr_cpus_loongson; cpu++) {
		if (reserved_cpus_mask & (1<<cpu))
			continue;
		nr_cpus_online++;
	}
	pr_info("nr_cpus_online = %d\n", nr_cpus_online);

	/* For unified kernel, NR_CPUS is the maximum possible value,
	 * nr_cpus_loongson is the really present value */
	tmp = nr_cpus_online;
	while (i < nr_cpus_loongson) {
		if (reserved_cpus_mask & (1<<i)) {
			/* Reserved physical CPU cores */
			__cpu_number_map[i] = tmp;
			__cpu_logical_map[tmp] = i;
			cpu_data[tmp].core = i % cores_per_package;
			cpu_data[tmp].package = i / cores_per_package;
			tmp++;
		} else {
			__cpu_number_map[i] = num;
			__cpu_logical_map[num] = i;
			cpu_data[num].core = i % cores_per_package;
			cpu_data[num].package = i / cores_per_package;
			num++;
		}
		i++;
	}

	if (strstr(eboard->name,"2H")) {
		board_type = LS2H;
	} else {
		board_type = RS780E;
	}

	dma64_supported = 0;
	if (((especial->resource[0].flags & DMA64_SUPPORTED) || (eirq_source->dma_mask_bits == 64))
			&& (board_type == RS780E)) {
		dma64_supported = 1;
		pr_info("BIOS dma flags:0x%x\n", especial->resource[0].flags);
		pr_info("BIOS dma_mask_bits:%d\n", eirq_source->dma_mask_bits);
		pr_info("BIOS enable 64bit dma configuration\n");
	}

	/*
	 * 3B6C(3BITX) mainboard contains 2 nodes, and each node
	 * owns 3 cores. 3BITX uses 3B1500(Prid 0x6307) processor.
	 * 3BITX needs to disaply and use 6 cores.
	 */
	if ((ecpu->cputype == Loongson_3B) && ((read_c0_prid() & 0xf) == PRID_REV_LOONGSON3B_R2)) {
		if ((strstr(eboard->name, "3B6C") || (nr_cpus_online == 6)) &&
				(nr_cpus_loongson == 8)) {
			nr_cpus_loongson = 6;
			cores_per_node = 3;
			cores_per_package = 6;
		}
	}

	if(strstr(eboard->name,"2H")){
		south_bridge_register(&ls2h_south_bridge);
		board_type = LS2H;
		pcidev_max_func_num = 1;
	}
	else{
		south_bridge_register(&rs780_south_bridge);
		board_type = RS780E;
		pcidev_max_func_num = 7;
	}

	pci_mem_start_addr = eirq_source->pci_mem_start_addr;
	pci_mem_end_addr = eirq_source->pci_mem_end_addr;
	loongson_pciio_base = eirq_source->pci_io_start_addr;
	if (!loongson_pciio_base) {
		if (cputype == Loongson_3A)
			loongson_pciio_base = 0xefdfc000000;
		else if (cputype == Loongson_3B)
			loongson_pciio_base = 0x1efdfc000000;
		else
			loongson_pciio_base = 0x1ff00000;
	}
	pr_info("BIOS loongson_pci_io_base:0x%llx\n", loongson_pciio_base);

	poweroff_addr = boot_p->reset_system.Shutdown;
	restart_addr = boot_p->reset_system.ResetWarm;
#if     defined(CONFIG_CPU_LOONGSON3)&&defined(CONFIG_SUSPEND)
	suspend_addr = boot_p->reset_system.ResetCold;
	pr_info("Shutdown Addr: %llx Reset Addr: %llx Suspend Addr: %llx\n", poweroff_addr, restart_addr,suspend_addr);
#else
	pr_info("Shutdown Addr: %llx Reset Addr: %llx\n", poweroff_addr, restart_addr);
#endif
	if (board_type == LS2H) {
		tmp = ls2h_readl(LS2H_GPIO_IN_REG);
		tmp = (tmp >> 8) & 0xf;
		if (tmp == LS3A2H_BOARD_VER_2_2) {
			loongson_pciio_base = 0x1bf00000;
			lpc_reg_base = LS2H_LPC_REG_BASE;
		} else {
			loongson_pciio_base = 0x1ff00000;
			lpc_reg_base = LS3_LPC_REG_BASE;
		}
	}

	/* parse bios info */
	strcpy(_bios_info, einter->description);
	bios_info = _bios_info;
	bios_vendor = strsep(&bios_info, "-");
	strsep(&bios_info, "-");
	strsep(&bios_info, "-");
	bios_release_date = strsep(&bios_info, "-");
	if (!bios_release_date)
		strcpy(_bios_release_date, especial->special_name);

	/* parse board info */
	strcpy(_board_info, eboard->name);
	board_info = _board_info;
	board_manufacturer = strsep(&board_info, "-");

	strcpy(__bios_info, einter->description);
	strcpy(__board_info, eboard->name);

	biosrom_size = einter->size;

	vgabios_addr = boot_p->efi.smbios.vga_bios;
#endif
	if (cpu_clock_freq == 0) {
		processor_id = (&current_cpu_data)->processor_id;
		switch (processor_id & PRID_REV_MASK) {
		case PRID_REV_LOONGSON2E:
			cpu_clock_freq = 533080000;
			break;
		case PRID_REV_LOONGSON2F:
			cpu_clock_freq = 797000000;
			break;
		case PRID_REV_LOONGSON3A:
			cpu_clock_freq = 900000000;
			break;
		case PRID_REV_LOONGSON3B_R1:
		case PRID_REV_LOONGSON3B_R2:
			cpu_clock_freq = 1000000000;
			break;
		default:
			cpu_clock_freq = 100000000;
			break;
		}
	}
	pr_info("CpuClock = %u\n", cpu_clock_freq);
}

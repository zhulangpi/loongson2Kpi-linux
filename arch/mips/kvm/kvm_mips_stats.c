/*
* This file is subject to the terms and conditions of the GNU General Public
* License.  See the file "COPYING" in the main directory of this archive
* for more details.
*
* KVM/MIPS: COP0 access histogram
*
* Copyright (C) 2012  MIPS Technologies, Inc.  All rights reserved.
* Authors: Sanjay Lal <sanjayl@kymasys.com>
*/

#ifndef CONFIG_KVM_MIPS_LOONGSON3
#include <linux/kvm_host.h>

char *kvm_mips_exit_types_str[MAX_KVM_MIPS_EXIT_TYPES] = {
	"WAIT",
	"CACHE",
	"Signal",
	"Interrupt",
	"COP0/1 Unusable",
	"TLB Mod",
	"TLB Miss (LD)",
	"TLB Miss (ST)",
	"Address Err (ST)",
	"Address Error (LD)",
	"System Call",
	"Reserved Inst",
	"Break Inst",
	"D-Cache Flushes",
};

char *kvm_cop0_str[N_MIPS_COPROC_REGS] = {
	"Index",
	"Random",
	"EntryLo0",
	"EntryLo1",
	"Context",
	"PG Mask",
	"Wired",
	"HWREna",
	"BadVAddr",
	"Count",
	"EntryHI",
	"Compare",
	"Status",
	"Cause",
	"EXC PC",
	"PRID",
	"Config",
	"LLAddr",
	"Watch Lo",
	"Watch Hi",
	"X Context",
	"Reserved",
	"Impl Dep",
	"Debug",
	"DEPC",
	"PerfCnt",
	"ErrCtl",
	"CacheErr",
	"TagLo",
	"TagHi",
	"ErrorEPC",
	"DESAVE"
};

int kvm_mips_dump_stats(struct kvm_vcpu *vcpu)
{
#ifdef CONFIG_KVM_MIPS_DEBUG_COP0_COUNTERS
	int i, j;

	printk("\nKVM VCPU[%d] COP0 Access Profile:\n", vcpu->vcpu_id);
	for (i = 0; i < N_MIPS_COPROC_REGS; i++) {
		for (j = 0; j < N_MIPS_COPROC_SEL; j++) {
			if (vcpu->arch.cop0->stat[i][j])
				printk("%s[%d]: %lu\n", kvm_cop0_str[i], j,
				       vcpu->arch.cop0->stat[i][j]);
		}
	}
#endif

	return 0;
}
#else
#include "linux/module.h"
#include "linux/list.h"
#include "linux/init.h"
#include "linux/kernel.h"
#include "linux/sched.h"
#include "linux/types.h"
#include "linux/kthread.h"
#include "linux/proc_fs.h"
#include "statistic.h"

static char* kvm_exit_name[] = {
	"int_exits",
	"mod_exits",
	"mmio_exits",
	"mm_exits",
	"adel_exits",
	"ades_exits",
	"ibe_exits",
	"dbe_exits",
	"tlbl_exits",
	"tlbs_exits",
	"syscall_exits",
	"bp_exits",
	"ri_exits",
	"cpu_exits",
	"cpu_mfc0_exits",
	"cpu_dmfc0_exits",
	"cpu_mtc0_exits",
	"cpu_dmtc0_exits",
	"cpu_xi_exits",
	"cpu_tlbr_exits",
	"cpu_tlbwi_exits",
	"cpu_tlbwr_exits",
	"cpu_tlbp_exits",
	"cpu_eret_exits",
	"cpu_kernel_cache_exits",
	"cpu_user_cache_exits",
	"cpu_cp1_exits",
	"cpu_cp2_exits",
	"ov_exits",
	"tr_exits",
	"tr_insertswapper_exits",
	"tr_gethostvaddr_exits",
	"tr_insertpgcurrent_exits",
	"tr_tlbflushall_exits",
	"tr_tlbflushpage_exits",
	"fpe_exits",
	"c2e_exits",
	"mdmx_exits",
	"watch_exits",
	"mcheck_exits",
	"mt_exits",
	"dsp_exits",
	"tlbmiss_exits",
	"kernel_miss_exits",
	"user_miss_exits",
	"reserved_exits",
	"write cp0_wired",
	0
};

extern int kvminfo_proc_init(void);
extern void kvminfo_proc_destroy(int qemu_pid);
static struct proc_dir_entry *cpuinfo_file = NULL;

static struct kvm_vcpu *vcpu_mips_global_bak[100] = {0};
static int vcpu_count = 0;
static struct proc_dir_entry *sec_dir = NULL;
static int sec_dir_is_build = 0;
static char qemu_mips_proc_filename[100] = {0};
static char del_qemu_mips_proc_filename[100] = {0};

static int vcpu_mips_pid[100] = {0};
static int vcpu_mips_count = 0;

void kvmmips_stat_init(struct kvm_vcpu* vcpu)
{
	int i;

	sprintf(qemu_mips_proc_filename,"kvm_exit_info_%d",current->tgid);
	vcpu_mips_pid[vcpu_mips_count] = current->tgid;
	vcpu_mips_global_bak[vcpu_mips_count] = vcpu;

	for (i = 0; i < __NUMBER_OF_KVM_EXIT_TYPES; i++) {
		vcpu->arch.statistic.each_exits[i] = 0;
	}

	for (i = 0; i < 32; i++) {
		vcpu->arch.statistic.cp0_uses[i] = 0;
	}

	if (kvminfo_proc_init()) {
		printk("kvmmips: proc init error!\n");
	}
}

void kvmmips_stat_exits(struct kvm_vcpu* vcpu, int type)
{
	if (type < 0 || type >= __NUMBER_OF_KVM_EXIT_TYPES) {
		printk("kvmmips: statistic error !!\n");
		return;
	}

	vcpu->arch.statistic.each_exits[type]++;
}

void kvmmips_stat_cp0_uses(struct kvm_vcpu* vcpu, int type)
{
	if (type < 0 || type >= 32) {
		printk("kvmmips: statistic error !!\n");
		return;
	}

	vcpu->arch.statistic.cp0_uses[type]++;
}

void kvmmips_print_stat(struct kvm_vcpu* vcpu)
{
	int i;

	for (i = 0; i < __NUMBER_OF_KVM_EXIT_TYPES; i++) {
		printk("%s : %d\n", kvm_exit_name[i], vcpu->arch.statistic.each_exits[i]);
	}

	for (i = 0; i < 32; i++) {
		printk("%d   ", vcpu->arch.statistic.cp0_uses[i]);
		if (i % 8 == 7) {
			printk("\n");
		}
	}
}

int kvminfo_proc_init(void)
{
	if( sec_dir_is_build == 0 ) {
		const char *d = "kvm_exit_info";
		sec_dir = proc_mkdir(d,NULL);
		sec_dir_is_build = sec_dir_is_build + 1;
	}

	int rv = 0;
	vcpu_mips_count++;

	if(cpuinfo_file == NULL){
		rv = -ENOMEM;
		remove_proc_entry(qemu_mips_proc_filename, sec_dir);
	}

	return rv;
}

void kvminfo_proc_destroy(int qemu_pid)
{

	sprintf(del_qemu_mips_proc_filename,"kvm_exit_info_%d",qemu_pid);
}
#endif /* CONFIG_KVM_MIPS_LOONGSON3 */

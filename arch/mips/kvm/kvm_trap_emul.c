/*
* This file is subject to the terms and conditions of the GNU General Public
* License.  See the file "COPYING" in the main directory of this archive
* for more details.
*
* KVM/MIPS: Deliver/Emulate exceptions to the guest kernel
*
* Copyright (C) 2012  MIPS Technologies, Inc.  All rights reserved.
* Authors: Sanjay Lal <sanjayl@kymasys.com>
*/

#ifndef CONFIG_KVM_MIPS_LOONGSON3
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/vmalloc.h>

#include <linux/kvm_host.h>

#include "kvm_mips_opcode.h"
#include "kvm_mips_int.h"

static gpa_t kvm_trap_emul_gva_to_gpa_cb(gva_t gva)
{
	gpa_t gpa;
	uint32_t kseg = KSEGX(gva);

	if ((kseg == CKSEG0) || (kseg == CKSEG1))
		gpa = CPHYSADDR(gva);
	else {
		printk("%s: cannot find GPA for GVA: %#lx\n", __func__, gva);
		kvm_mips_dump_host_tlbs();
		gpa = KVM_INVALID_ADDR;
	}

#ifdef DEBUG
	kvm_debug("%s: gva %#lx, gpa: %#llx\n", __func__, gva, gpa);
#endif

	return gpa;
}


static int kvm_trap_emul_handle_cop_unusable(struct kvm_vcpu *vcpu)
{
	struct kvm_run *run = vcpu->run;
	uint32_t __user *opc = (uint32_t __user *) vcpu->arch.pc;
	unsigned long cause = vcpu->arch.host_cp0_cause;
	enum emulation_result er = EMULATE_DONE;
	int ret = RESUME_GUEST;

	if (((cause & CAUSEF_CE) >> CAUSEB_CE) == 1) {
		er = kvm_mips_emulate_fpu_exc(cause, opc, run, vcpu);
	} else
		er = kvm_mips_emulate_inst(cause, opc, run, vcpu);

	switch (er) {
	case EMULATE_DONE:
		ret = RESUME_GUEST;
		break;

	case EMULATE_FAIL:
		run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
		ret = RESUME_HOST;
		break;

	case EMULATE_WAIT:
		run->exit_reason = KVM_EXIT_INTR;
		ret = RESUME_HOST;
		break;

	default:
		BUG();
	}
	return ret;
}

static int kvm_trap_emul_handle_tlb_mod(struct kvm_vcpu *vcpu)
{
	struct kvm_run *run = vcpu->run;
	uint32_t __user *opc = (uint32_t __user *) vcpu->arch.pc;
	unsigned long badvaddr = vcpu->arch.host_cp0_badvaddr;
	unsigned long cause = vcpu->arch.host_cp0_cause;
	enum emulation_result er = EMULATE_DONE;
	int ret = RESUME_GUEST;

	if (KVM_GUEST_KSEGX(badvaddr) < KVM_GUEST_KSEG0
	    || KVM_GUEST_KSEGX(badvaddr) == KVM_GUEST_KSEG23) {
#ifdef DEBUG
		kvm_debug
		    ("USER/KSEG23 ADDR TLB MOD fault: cause %#lx, PC: %p, BadVaddr: %#lx\n",
		     cause, opc, badvaddr);
#endif
		er = kvm_mips_handle_tlbmod(cause, opc, run, vcpu);

		if (er == EMULATE_DONE)
			ret = RESUME_GUEST;
		else {
			run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
			ret = RESUME_HOST;
		}
	} else if (KVM_GUEST_KSEGX(badvaddr) == KVM_GUEST_KSEG0) {
		/* XXXKYMA: The guest kernel does not expect to get this fault when we are not
		 * using HIGHMEM. Need to address this in a HIGHMEM kernel
		 */
		printk
		    ("TLB MOD fault not handled, cause %#lx, PC: %p, BadVaddr: %#lx\n",
		     cause, opc, badvaddr);
		kvm_mips_dump_host_tlbs();
		kvm_arch_vcpu_dump_regs(vcpu);
		run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
		ret = RESUME_HOST;
	} else {
		printk
		    ("Illegal TLB Mod fault address , cause %#lx, PC: %p, BadVaddr: %#lx\n",
		     cause, opc, badvaddr);
		kvm_mips_dump_host_tlbs();
		kvm_arch_vcpu_dump_regs(vcpu);
		run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
		ret = RESUME_HOST;
	}
	return ret;
}

static int kvm_trap_emul_handle_tlb_st_miss(struct kvm_vcpu *vcpu)
{
	struct kvm_run *run = vcpu->run;
	uint32_t __user *opc = (uint32_t __user *) vcpu->arch.pc;
	unsigned long badvaddr = vcpu->arch.host_cp0_badvaddr;
	unsigned long cause = vcpu->arch.host_cp0_cause;
	enum emulation_result er = EMULATE_DONE;
	int ret = RESUME_GUEST;

	if (((badvaddr & PAGE_MASK) == KVM_GUEST_COMMPAGE_ADDR)
	    && KVM_GUEST_KERNEL_MODE(vcpu)) {
		if (kvm_mips_handle_commpage_tlb_fault(badvaddr, vcpu) < 0) {
			run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
			ret = RESUME_HOST;
		}
	} else if (KVM_GUEST_KSEGX(badvaddr) < KVM_GUEST_KSEG0
		   || KVM_GUEST_KSEGX(badvaddr) == KVM_GUEST_KSEG23) {
#ifdef DEBUG
		kvm_debug
		    ("USER ADDR TLB LD fault: cause %#lx, PC: %p, BadVaddr: %#lx\n",
		     cause, opc, badvaddr);
#endif
		er = kvm_mips_handle_tlbmiss(cause, opc, run, vcpu);
		if (er == EMULATE_DONE)
			ret = RESUME_GUEST;
		else {
			run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
			ret = RESUME_HOST;
		}
	} else if (KVM_GUEST_KSEGX(badvaddr) == KVM_GUEST_KSEG0) {
		/* All KSEG0 faults are handled by KVM, as the guest kernel does not
		 * expect to ever get them
		 */
		if (kvm_mips_handle_kseg0_tlb_fault
		    (vcpu->arch.host_cp0_badvaddr, vcpu) < 0) {
			run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
			ret = RESUME_HOST;
		}
	} else {
		kvm_err
		    ("Illegal TLB LD fault address , cause %#lx, PC: %p, BadVaddr: %#lx\n",
		     cause, opc, badvaddr);
		kvm_mips_dump_host_tlbs();
		kvm_arch_vcpu_dump_regs(vcpu);
		run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
		ret = RESUME_HOST;
	}
	return ret;
}

static int kvm_trap_emul_handle_tlb_ld_miss(struct kvm_vcpu *vcpu)
{
	struct kvm_run *run = vcpu->run;
	uint32_t __user *opc = (uint32_t __user *) vcpu->arch.pc;
	unsigned long badvaddr = vcpu->arch.host_cp0_badvaddr;
	unsigned long cause = vcpu->arch.host_cp0_cause;
	enum emulation_result er = EMULATE_DONE;
	int ret = RESUME_GUEST;

	if (((badvaddr & PAGE_MASK) == KVM_GUEST_COMMPAGE_ADDR)
	    && KVM_GUEST_KERNEL_MODE(vcpu)) {
		if (kvm_mips_handle_commpage_tlb_fault(badvaddr, vcpu) < 0) {
			run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
			ret = RESUME_HOST;
		}
	} else if (KVM_GUEST_KSEGX(badvaddr) < KVM_GUEST_KSEG0
		   || KVM_GUEST_KSEGX(badvaddr) == KVM_GUEST_KSEG23) {
#ifdef DEBUG
		kvm_debug("USER ADDR TLB ST fault: PC: %#lx, BadVaddr: %#lx\n",
			  vcpu->arch.pc, badvaddr);
#endif

		/* User Address (UA) fault, this could happen if
		 * (1) TLB entry not present/valid in both Guest and shadow host TLBs, in this
		 *     case we pass on the fault to the guest kernel and let it handle it.
		 * (2) TLB entry is present in the Guest TLB but not in the shadow, in this
		 *     case we inject the TLB from the Guest TLB into the shadow host TLB
		 */

		er = kvm_mips_handle_tlbmiss(cause, opc, run, vcpu);
		if (er == EMULATE_DONE)
			ret = RESUME_GUEST;
		else {
			run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
			ret = RESUME_HOST;
		}
	} else if (KVM_GUEST_KSEGX(badvaddr) == KVM_GUEST_KSEG0) {
		if (kvm_mips_handle_kseg0_tlb_fault
		    (vcpu->arch.host_cp0_badvaddr, vcpu) < 0) {
			run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
			ret = RESUME_HOST;
		}
	} else {
		printk
		    ("Illegal TLB ST fault address , cause %#lx, PC: %p, BadVaddr: %#lx\n",
		     cause, opc, badvaddr);
		kvm_mips_dump_host_tlbs();
		kvm_arch_vcpu_dump_regs(vcpu);
		run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
		ret = RESUME_HOST;
	}
	return ret;
}

static int kvm_trap_emul_handle_addr_err_st(struct kvm_vcpu *vcpu)
{
	struct kvm_run *run = vcpu->run;
	uint32_t __user *opc = (uint32_t __user *) vcpu->arch.pc;
	unsigned long badvaddr = vcpu->arch.host_cp0_badvaddr;
	unsigned long cause = vcpu->arch.host_cp0_cause;
	enum emulation_result er = EMULATE_DONE;
	int ret = RESUME_GUEST;

	if (KVM_GUEST_KERNEL_MODE(vcpu)
	    && (KSEGX(badvaddr) == CKSEG0 || KSEGX(badvaddr) == CKSEG1)) {
#ifdef DEBUG
		kvm_debug("Emulate Store to MMIO space\n");
#endif
		er = kvm_mips_emulate_inst(cause, opc, run, vcpu);
		if (er == EMULATE_FAIL) {
			printk("Emulate Store to MMIO space failed\n");
			run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
			ret = RESUME_HOST;
		} else {
			run->exit_reason = KVM_EXIT_MMIO;
			ret = RESUME_HOST;
		}
	} else {
		printk
		    ("Address Error (STORE): cause %#lx, PC: %p, BadVaddr: %#lx\n",
		     cause, opc, badvaddr);
		run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
		ret = RESUME_HOST;
	}
	return ret;
}

static int kvm_trap_emul_handle_addr_err_ld(struct kvm_vcpu *vcpu)
{
	struct kvm_run *run = vcpu->run;
	uint32_t __user *opc = (uint32_t __user *) vcpu->arch.pc;
	unsigned long badvaddr = vcpu->arch.host_cp0_badvaddr;
	unsigned long cause = vcpu->arch.host_cp0_cause;
	enum emulation_result er = EMULATE_DONE;
	int ret = RESUME_GUEST;

	if (KSEGX(badvaddr) == CKSEG0 || KSEGX(badvaddr) == CKSEG1) {
#ifdef DEBUG
		kvm_debug("Emulate Load from MMIO space @ %#lx\n", badvaddr);
#endif
		er = kvm_mips_emulate_inst(cause, opc, run, vcpu);
		if (er == EMULATE_FAIL) {
			printk("Emulate Load from MMIO space failed\n");
			run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
			ret = RESUME_HOST;
		} else {
			run->exit_reason = KVM_EXIT_MMIO;
			ret = RESUME_HOST;
		}
	} else {
		printk
		    ("Address Error (LOAD): cause %#lx, PC: %p, BadVaddr: %#lx\n",
		     cause, opc, badvaddr);
		run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
		ret = RESUME_HOST;
		er = EMULATE_FAIL;
	}
	return ret;
}

static int kvm_trap_emul_handle_syscall(struct kvm_vcpu *vcpu)
{
	struct kvm_run *run = vcpu->run;
	uint32_t __user *opc = (uint32_t __user *) vcpu->arch.pc;
	unsigned long cause = vcpu->arch.host_cp0_cause;
	enum emulation_result er = EMULATE_DONE;
	int ret = RESUME_GUEST;

	er = kvm_mips_emulate_syscall(cause, opc, run, vcpu);
	if (er == EMULATE_DONE)
		ret = RESUME_GUEST;
	else {
		run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
		ret = RESUME_HOST;
	}
	return ret;
}

static int kvm_trap_emul_handle_res_inst(struct kvm_vcpu *vcpu)
{
	struct kvm_run *run = vcpu->run;
	uint32_t __user *opc = (uint32_t __user *) vcpu->arch.pc;
	unsigned long cause = vcpu->arch.host_cp0_cause;
	enum emulation_result er = EMULATE_DONE;
	int ret = RESUME_GUEST;

	er = kvm_mips_handle_ri(cause, opc, run, vcpu);
	if (er == EMULATE_DONE)
		ret = RESUME_GUEST;
	else {
		run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
		ret = RESUME_HOST;
	}
	return ret;
}

static int kvm_trap_emul_handle_break(struct kvm_vcpu *vcpu)
{
	struct kvm_run *run = vcpu->run;
	uint32_t __user *opc = (uint32_t __user *) vcpu->arch.pc;
	unsigned long cause = vcpu->arch.host_cp0_cause;
	enum emulation_result er = EMULATE_DONE;
	int ret = RESUME_GUEST;

	er = kvm_mips_emulate_bp_exc(cause, opc, run, vcpu);
	if (er == EMULATE_DONE)
		ret = RESUME_GUEST;
	else {
		run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
		ret = RESUME_HOST;
	}
	return ret;
}

static int kvm_trap_emul_vm_init(struct kvm *kvm)
{
	return 0;
}

static int kvm_trap_emul_vcpu_init(struct kvm_vcpu *vcpu)
{
	return 0;
}

static int kvm_trap_emul_vcpu_setup(struct kvm_vcpu *vcpu)
{
	struct mips_coproc *cop0 = vcpu->arch.cop0;
	uint32_t config1;
	int vcpu_id = vcpu->vcpu_id;

	/* Arch specific stuff, set up config registers properly so that the
	 * guest will come up as expected, for now we simulate a
	 * MIPS 24kc
	 */
	kvm_write_c0_guest_prid(cop0, 0x00019300);
	kvm_write_c0_guest_config(cop0,
				  MIPS_CONFIG0 | (0x1 << CP0C0_AR) |
				  (MMU_TYPE_R4000 << CP0C0_MT));

	/* Read the cache characteristics from the host Config1 Register */
	config1 = (read_c0_config1() & ~0x7f);

	/* Set up MMU size */
	config1 &= ~(0x3f << 25);
	config1 |= ((KVM_MIPS_GUEST_TLB_SIZE - 1) << 25);

	/* We unset some bits that we aren't emulating */
	config1 &=
	    ~((1 << CP0C1_C2) | (1 << CP0C1_MD) | (1 << CP0C1_PC) |
	      (1 << CP0C1_WR) | (1 << CP0C1_CA));
	kvm_write_c0_guest_config1(cop0, config1);

	kvm_write_c0_guest_config2(cop0, MIPS_CONFIG2);
	/* MIPS_CONFIG2 | (read_c0_config2() & 0xfff) */
	kvm_write_c0_guest_config3(cop0,
				   MIPS_CONFIG3 | (0 << CP0C3_VInt) | (1 <<
								       CP0C3_ULRI));

	/* Set Wait IE/IXMT Ignore in Config7, IAR, AR */
	kvm_write_c0_guest_config7(cop0, (MIPS_CONF7_WII) | (1 << 10));

	/* Setup IntCtl defaults, compatibilty mode for timer interrupts (HW5) */
	kvm_write_c0_guest_intctl(cop0, 0xFC000000);

	/* Put in vcpu id as CPUNum into Ebase Reg to handle SMP Guests */
	kvm_write_c0_guest_ebase(cop0, KVM_GUEST_KSEG0 | (vcpu_id & 0xFF));

	return 0;
}

static struct kvm_mips_callbacks kvm_trap_emul_callbacks = {
	/* exit handlers */
	.handle_cop_unusable = kvm_trap_emul_handle_cop_unusable,
	.handle_tlb_mod = kvm_trap_emul_handle_tlb_mod,
	.handle_tlb_st_miss = kvm_trap_emul_handle_tlb_st_miss,
	.handle_tlb_ld_miss = kvm_trap_emul_handle_tlb_ld_miss,
	.handle_addr_err_st = kvm_trap_emul_handle_addr_err_st,
	.handle_addr_err_ld = kvm_trap_emul_handle_addr_err_ld,
	.handle_syscall = kvm_trap_emul_handle_syscall,
	.handle_res_inst = kvm_trap_emul_handle_res_inst,
	.handle_break = kvm_trap_emul_handle_break,

	.vm_init = kvm_trap_emul_vm_init,
	.vcpu_init = kvm_trap_emul_vcpu_init,
	.vcpu_setup = kvm_trap_emul_vcpu_setup,
	.gva_to_gpa = kvm_trap_emul_gva_to_gpa_cb,
	.queue_timer_int = kvm_mips_queue_timer_int_cb,
	.dequeue_timer_int = kvm_mips_dequeue_timer_int_cb,
	.queue_io_int = kvm_mips_queue_io_int_cb,
	.dequeue_io_int = kvm_mips_dequeue_io_int_cb,
	.irq_deliver = kvm_mips_irq_deliver_cb,
	.irq_clear = kvm_mips_irq_clear_cb,
};

int kvm_mips_emulation_init(struct kvm_mips_callbacks **install_callbacks)
{
	*install_callbacks = &kvm_trap_emul_callbacks;
	return 0;
}
#else
#include <linux/kvm_host.h>
#include <linux/slab.h>
#include <linux/time.h>

#include <asm/ptrace.h>
#include <asm/kvm_host.h>
#include <asm/kvm_loongson.h>
#include <asm/kvm_asm.h>
#include <asm/kvm_mips.h>

#include "statistic.h"

u64* kvmmips_get_cp0_reg(struct kvm_vcpu* vcpu_ptr, int reg, int sel)
{
	switch (reg)
	{
	case 0:
		return &(vcpu_ptr->arch.cp0->cp0_index);
	case 1:
		return &(vcpu_ptr->arch.cp0->cp0_random);
	case 2:
		return &(vcpu_ptr->arch.cp0->cp0_entrylo0);
	case 3:
		return &(vcpu_ptr->arch.cp0->cp0_entrylo1);
	case 4:
		return &(vcpu_ptr->arch.cp0->cp0_context);
	case 5:
		return &(vcpu_ptr->arch.cp0->cp0_pagemask);
	case 6:
		return &(vcpu_ptr->arch.cp0->cp0_wired);
	case 7:
		return &(vcpu_ptr->arch.cp0->cp0_hwrena);
	case 8:
		return &(vcpu_ptr->arch.cp0->cp0_badvaddr);
	case 9:
		return &(vcpu_ptr->arch.cp0->cp0_count);
	case 10:
		return &(vcpu_ptr->arch.cp0->cp0_entryhi);
	case 11:
		return &(vcpu_ptr->arch.cp0->cp0_compare);
	case 12:
		switch (sel)
		{
		case 0:
			return &(vcpu_ptr->arch.cp0->cp0_status);
		case 1:
			return &(vcpu_ptr->arch.cp0->cp0_intctl);
		case 2:
			return &(vcpu_ptr->arch.cp0->cp0_srsctl);
		case 3:
			return &(vcpu_ptr->arch.cp0->cp0_srsmap);
		default:
			break;
		}
		break;
	case 13:
		return &(vcpu_ptr->arch.cp0->cp0_cause);
	case 14:
		return &(vcpu_ptr->arch.cp0->cp0_epc);
	case 15:
		switch (sel)
		{
		case 0:
			return &(vcpu_ptr->arch.cp0->cp0_prid);
		case 1:
			return &(vcpu_ptr->arch.cp0->cp0_ebase);
		default:
			break;
		}
		break;
	case 16:
		switch (sel)
		{
		case 0:
			return &(vcpu_ptr->arch.cp0->cp0_config);
		case 1:
			return &(vcpu_ptr->arch.cp0->cp0_config1);
		case 2:
			return &(vcpu_ptr->arch.cp0->cp0_config2);
		case 3:
			return &(vcpu_ptr->arch.cp0->cp0_config3);
		default:
			break;
		}
		break;
	case 17:
		return &(vcpu_ptr->arch.cp0->cp0_lladdr);
	case 18:
		return &(vcpu_ptr->arch.cp0->cp0_watchlo);
	case 19:
		return &(vcpu_ptr->arch.cp0->cp0_watchhi);
	case 20:
		return &(vcpu_ptr->arch.cp0->cp0_xcontext);
	case 21:
		break;
	case 22:
		return &(vcpu_ptr->arch.cp0->cp0_diagnostic);
	case 23:
		return &(vcpu_ptr->arch.cp0->cp0_debug);
	case 24:
		return &(vcpu_ptr->arch.cp0->cp0_depc);
	case 25:
		switch (sel)
		{
		case 0:
			return &(vcpu_ptr->arch.cp0->cp0_perfctl);
		case 1:
			return &(vcpu_ptr->arch.cp0->cp0_perfcnt);
		default:
			break;
		}
		break;
	case 26:
		return &(vcpu_ptr->arch.cp0->cp0_ecc);
	case 27:
		return &(vcpu_ptr->arch.cp0->cp0_cacheerr);
	case 28:
		switch (sel)
		{
		case 0:
			return &(vcpu_ptr->arch.cp0->cp0_taglo);
		case 1:
			return &(vcpu_ptr->arch.cp0->cp0_datalo);
		default:
			break;
		}
		break;
	case 29:
		switch (sel)
		{
		case 0:
			return &(vcpu_ptr->arch.cp0->cp0_taghi);
		case 1:
			return &(vcpu_ptr->arch.cp0->cp0_datahi);
		default:
			break;
		}
		break;
	case 30:
		return &(vcpu_ptr->arch.cp0->cp0_errorepc);
	case 31:
		return &(vcpu_ptr->arch.cp0->cp0_desave);
	default:
		break;
	}

	return 0;
}

u64 kvmmips_sign_extern(u32 temp)
{
	u64 result = (temp & 0x80000000) ?
			temp | 0xffffffff00000000 : (temp & 0xffffffff);
	return result;
}

u64 kvmmips_sign_extern16(u32 temp)
{
	u64 result = (temp & 0x8000) ?
			temp | 0xffffffffffff0000 : (temp & 0xffff);
	return result;
}

/* inject exception to guest OS */
void kvmmips_queue_exception(struct kvm_vcpu* vcpu_ptr, int exception_type)
{
	/* queue interrupt should not tackle here */
	if (exception_type <= KVMMIPS_EXCEPTION_INT
		|| exception_type >= KVMMIPS_EXCEPTION_TOP) {
		return;
	}
	vcpu_ptr->arch.pending_exceptions |= (u64)0x1 << exception_type;
}

void kvmmips_dequeue_exception(struct kvm_vcpu* vcpu_ptr, int exception_type)
{
	if (exception_type <= KVMMIPS_EXCEPTION_INT
		|| exception_type >= KVMMIPS_EXCEPTION_TOP) {
		return;
	}
	vcpu_ptr->arch.pending_exceptions &= ~((u64)0x1 << exception_type);
}

#define KVMMIPS_CAUSE_IP_BASE 8
#define KVMMIPS_CAUSE_IP_MASK 0x0000ff00
void kvmmips_queue_interrupt(struct kvm_vcpu* vcpu_ptr, struct kvm_interrupt* interrupt)
{
	vcpu_ptr->arch.pending_irqs |= 0x1 << (interrupt->irq + KVMMIPS_CAUSE_IP_BASE);
	vcpu_ptr->arch.pending_exceptions |= 0x1 << KVMMIPS_EXCEPTION_INT;
}

void kvmmips_dequeue_interrupt(struct kvm_vcpu* vcpu_ptr, struct kvm_interrupt* interrupt)
{
	vcpu_ptr->arch.pending_irqs &= ~(0x1 << (interrupt->irq + KVMMIPS_CAUSE_IP_BASE));
	if (vcpu_ptr->arch.pending_irqs == 0) {
		vcpu_ptr->arch.cp0->cp0_cause &= ~KVMMIPS_CAUSE_IP_MASK;
		vcpu_ptr->arch.pending_exceptions &= ~(0x1 << KVMMIPS_EXCEPTION_INT);
	}
}

#define KVMMIPS_CAUSE_BD_MASK		0x80000000
#define KVMMIPS_CAUSE_CE_MASK		0x30000000
#define KVMMIPS_CAUSE_EXCCODE_MASK	0x7c
int kvmmips_exception_deliver(struct kvm_vcpu* vcpu_ptr, int exception_type)
{
	/* ebase register is always in 0x80000000 ~ 0x90000000 in hardware
	 * but in software it can in other area
	 */
	/**TODO: ebase initial value should be changed*/
#ifdef CONFIG_KVM64_SUPPORT
	u64 ebase = (vcpu_ptr->arch.cp0->cp0_ebase & 0xfffff000) | 0x4000000080000000;
#else
	u64 ebase = kvmmips_sign_extern(vcpu_ptr->arch.cp0->cp0_ebase & 0xfffff000)
			| 0xffffffffc0000000;
#endif

	if (vcpu_ptr->arch.pending_exceptions == 0) {
		//guest os doesn't tackle interrupts
		return 0;
	}

	if (exception_type == KVMMIPS_EXCEPTION_TLBMISS) {
		//you should not change cp0_epc register when context is in exceptions
		if (!(vcpu_ptr->arch.cp0->cp0_status & 0x2)) {
			vcpu_ptr->arch.cp0->cp0_epc = vcpu_ptr->arch.pc;
			vcpu_ptr->arch.cp0->cp0_status |= 0x2;
			//cause(bd) pass to guest cp0
			vcpu_ptr->arch.cp0->cp0_cause &= ~KVMMIPS_CAUSE_BD_MASK;
			vcpu_ptr->arch.cp0->cp0_cause |=
				vcpu_ptr->arch.temp_cp0_cause & KVMMIPS_CAUSE_BD_MASK;
		}
		vcpu_ptr->arch.pc = ebase;

		//cause(ce) pass to guest cp0
		vcpu_ptr->arch.cp0->cp0_cause &= ~KVMMIPS_CAUSE_CE_MASK;
		vcpu_ptr->arch.cp0->cp0_cause |=
			vcpu_ptr->arch.temp_cp0_cause & KVMMIPS_CAUSE_CE_MASK;

		//set badvaddr  ,entryhi and pagemask
		vcpu_ptr->arch.cp0->cp0_badvaddr = vcpu_ptr->arch.temp_cp0_badvaddr;
		vcpu_ptr->arch.cp0->cp0_entryhi =
			((vcpu_ptr->arch.temp_cp0_entryhi) & (PAGE_MASK << 1))
			| (vcpu_ptr->arch.cp0->cp0_entryhi & 0xff);

		//compute context and xcontext
		//TODO: define something
		vcpu_ptr->arch.cp0->cp0_context &= ~(u64)0x7fffff;
		vcpu_ptr->arch.cp0->cp0_context |=
			((vcpu_ptr->arch.temp_cp0_badvaddr & 0xffffe000) >> 9) & 0x7ffff0;

		return 1;
	}

	if (exception_type > KVMMIPS_EXCEPTION_INT &&
		exception_type <= KVMMIPS_EXCEPTION_TLBS) {
		//you should not change cp0_epc register when context is in exceptions
		if (!(vcpu_ptr->arch.cp0->cp0_status & 0x2)) {
			vcpu_ptr->arch.cp0->cp0_epc = vcpu_ptr->arch.pc;
			vcpu_ptr->arch.cp0->cp0_status |= 0x2;
			//cause(bd) pass to guest cp0
			vcpu_ptr->arch.cp0->cp0_cause &= ~KVMMIPS_CAUSE_BD_MASK;
			vcpu_ptr->arch.cp0->cp0_cause |=
				vcpu_ptr->arch.temp_cp0_cause & KVMMIPS_CAUSE_BD_MASK;
		}
		vcpu_ptr->arch.pc = ebase + 0x180;

		//set cause(exccode)
		vcpu_ptr->arch.cp0->cp0_cause &= ~KVMMIPS_CAUSE_EXCCODE_MASK;
		vcpu_ptr->arch.cp0->cp0_cause |= exception_type << 2;

		//cause(ce) pass to guest cp0
		vcpu_ptr->arch.cp0->cp0_cause &= ~KVMMIPS_CAUSE_CE_MASK;
		vcpu_ptr->arch.cp0->cp0_cause |= vcpu_ptr->arch.temp_cp0_cause & KVMMIPS_CAUSE_CE_MASK;

		//set badvaddr  ,entryhi and pagemask
		vcpu_ptr->arch.cp0->cp0_badvaddr = vcpu_ptr->arch.temp_cp0_badvaddr;
		vcpu_ptr->arch.cp0->cp0_entryhi =
			((vcpu_ptr->arch.temp_cp0_entryhi) & (PAGE_MASK << 1))
			| (vcpu_ptr->arch.cp0->cp0_entryhi & 0xff);

		//compute context and xcontext
		//TODO: define something
		vcpu_ptr->arch.cp0->cp0_context &= ~(u64)0x7fffff;
		vcpu_ptr->arch.cp0->cp0_context |=
			((vcpu_ptr->arch.temp_cp0_badvaddr & 0xffffe000) >> 9) & 0x7ffff0;

		return 1;
	}

	if (exception_type == KVMMIPS_EXCEPTION_INT) {
		//when cause(ie) = 1 && cause(erl) = 0 && cause(exl) = 0 ,it can import interrupt
		//TODO:to be considered
		//cause(ip) and cause(ti) should be set
		if (vcpu_ptr->arch.pending_irqs & 0x8000) {
			//ip7 is timer interrupt
			vcpu_ptr->arch.cp0->cp0_cause |= 0x1 << 30;
		} else
			vcpu_ptr->arch.cp0->cp0_cause &= ~(u64)(0x1 << 30);

		vcpu_ptr->arch.cp0->cp0_cause &= ~KVMMIPS_CAUSE_IP_MASK;
		vcpu_ptr->arch.cp0->cp0_cause |= vcpu_ptr->arch.pending_irqs & KVMMIPS_CAUSE_IP_MASK;

		//TODO: The IP mask function is diabled to avoid bug
		vcpu_ptr->arch.cp0->cp0_status |= 0xff00;

		if ((vcpu_ptr->arch.cp0->cp0_status & 0x7) == 0x1) {
			vcpu_ptr->arch.cp0->cp0_epc = vcpu_ptr->arch.pc;
			vcpu_ptr->arch.pc = ebase + 0x180;
			vcpu_ptr->arch.cp0->cp0_status |= 0x2;

			//set cause(exccode)
			vcpu_ptr->arch.cp0->cp0_cause &= ~KVMMIPS_CAUSE_EXCCODE_MASK;
			vcpu_ptr->arch.cp0->cp0_cause |= exception_type  << 2;

			//cause(ce) and cause(bd) pass to guest cp0
			vcpu_ptr->arch.cp0->cp0_cause &= ~KVMMIPS_CAUSE_BD_MASK;
			vcpu_ptr->arch.cp0->cp0_cause |=
				vcpu_ptr->arch.temp_cp0_cause & KVMMIPS_CAUSE_BD_MASK;
			vcpu_ptr->arch.cp0->cp0_cause &= ~KVMMIPS_CAUSE_CE_MASK;
			vcpu_ptr->arch.cp0->cp0_cause |=
				vcpu_ptr->arch.temp_cp0_cause & KVMMIPS_CAUSE_CE_MASK;
		}

		return 0;
	}

	if (exception_type > KVMMIPS_EXCEPTION_TLBS &&
		exception_type < KVMMIPS_EXCEPTION_TLBMISS) {
		//you should not change cp0_epc register when context is in exceptions
		if (!(vcpu_ptr->arch.cp0->cp0_status & 0x2)) {
			vcpu_ptr->arch.cp0->cp0_epc = vcpu_ptr->arch.pc;
			vcpu_ptr->arch.cp0->cp0_status |= 0x2;
			//cause(bd) pass to guest cp0
			vcpu_ptr->arch.cp0->cp0_cause &= ~KVMMIPS_CAUSE_BD_MASK;
			vcpu_ptr->arch.cp0->cp0_cause |=
				vcpu_ptr->arch.temp_cp0_cause & KVMMIPS_CAUSE_BD_MASK;
		}
		vcpu_ptr->arch.pc = ebase + 0x180;

		//set cause(exccode)
		vcpu_ptr->arch.cp0->cp0_cause &= ~KVMMIPS_CAUSE_EXCCODE_MASK;
		vcpu_ptr->arch.cp0->cp0_cause |= exception_type  << 2;

		//cause(ce) pass to guest cp0
		vcpu_ptr->arch.cp0->cp0_cause &= ~KVMMIPS_CAUSE_CE_MASK;
		vcpu_ptr->arch.cp0->cp0_cause |=
			vcpu_ptr->arch.temp_cp0_cause & KVMMIPS_CAUSE_CE_MASK;

		vcpu_ptr->arch.cp0->cp0_badvaddr = vcpu_ptr->arch.temp_cp0_badvaddr;

		return 1;
	}

	//other should be considered
	//TODO: consider srsctl, xtlbrefill
	return 0;
}

static int kvmmips_priority[] = {4,32,2,30,6,12,13,8,9,10,11,15,5,3,1,7,18,22,23,24,25,26,0};
void kvmmips_deliver_exceptions(struct kvm_vcpu* vcpu_ptr)
{
	int i,temp;
	if (vcpu_ptr->arch.pending_exceptions == 0) {
		//no pending_exceptions
		return;
	}

	for (i = 0; i < (sizeof(kvmmips_priority) / 4); i++) {
		temp = kvmmips_priority[i];
		if ((vcpu_ptr->arch.pending_exceptions >> temp)	& 0x1) {
			if (kvmmips_exception_deliver(vcpu_ptr,temp)) {
				//clear the exception
				vcpu_ptr->arch.pending_exceptions &= ~((u64)0x1 << temp);
				break;
			}
		}
	}
}

/* real exception tackle */
extern asmlinkage void handle_kvm_int(void);
extern asmlinkage void handle_kvm_mod(void);
extern asmlinkage void handle_kvm_adel(void);
extern asmlinkage void handle_kvm_ades(void);
extern asmlinkage void handle_kvm_ibe(void);
extern asmlinkage void handle_kvm_dbe(void);
extern asmlinkage void handle_kvm_tlbl(void);
extern asmlinkage void handle_kvm_tlbs(void);
extern asmlinkage void handle_kvm_syscall(void);
extern asmlinkage void handle_kvm_bp(void);
extern asmlinkage void handle_kvm_ri(void);
extern asmlinkage void handle_kvm_cpu(void);
extern asmlinkage void handle_kvm_ov(void);
extern asmlinkage void handle_kvm_tr(void);
extern asmlinkage void handle_kvm_fpe(void);
extern asmlinkage void handle_kvm_c2e(void);
extern asmlinkage void handle_kvm_mdmx(void);
extern asmlinkage void handle_kvm_watch(void);
extern asmlinkage void handle_kvm_mcheck(void);
extern asmlinkage void handle_kvm_mt(void);
extern asmlinkage void handle_kvm_dsp(void);
extern asmlinkage void handle_kvm_tlbmiss(void);
extern asmlinkage void handle_kvm_reserved(void);

//temp for inplement
struct kvm_vcpu* vcpu_mips_global[NR_CPUS] = {0};
struct kvm_run*  run_mips_global[NR_CPUS] = {0};
//temp for storing regs
u64 kvm_temp_global;

void kvmmips_disable_preempt(void)
{
	preempt_disable();
}

void kvmmips_enable_preempt(void)
{
	preempt_enable();
}

asmlinkage int do_kvm_int(struct kvm_run* run_ptr , struct kvm_vcpu* vcpu_ptr)
{
	/* account info */
	kvmmips_stat_exits(vcpu_ptr, INT_EXITS);

	run_ptr->exit_reason = KVM_EXIT_UNKNOWN;
	run_ptr->ready_for_interrupt_injection = 1;

	if (need_resched()) {
		cond_resched();
	}

	//TODO: reduce RETURN_TO_HOST
	return RETURN_TO_HOST;

	//TODO: 3A port
	//return to host to ack IO interrupts.
	if(vcpu_ptr->arch.temp_cp0_cause & 0x00007c00)
		return RETURN_TO_HOST;
	else
		return RETURN_TO_GUEST;
}

extern int kvmmips_emulate_mmio(struct kvm_run* run_ptr, struct kvm_vcpu* vcpu_ptr, u64 vaddr);
extern void kvmmips_emulate_branch(struct kvm_run* run_ptr, struct kvm_vcpu* vcpu_ptr);
extern int kvmmips_emulate_mm(struct kvm_run* run_ptr, struct kvm_vcpu* vcpu_ptr, u64 vaddr);
asmlinkage int do_kvm_adel(struct kvm_run* run_ptr , struct kvm_vcpu* vcpu_ptr)
{
	u64 vaddr = vcpu_ptr->arch.temp_cp0_badvaddr;

	run_ptr->exit_reason = KVM_EXIT_UNKNOWN;
	run_ptr->ready_for_interrupt_injection = 1;

	if (((vaddr & 0xfffffffff0000000) == 0xffffffffb0000000) &&
		((vaddr & 0xfffffffffff00000) != 0xffffffffbfc00000)) {
		/* account info */
		kvmmips_stat_exits(vcpu_ptr, MMIO_EXITS);

		//trans vaddr to normal I/O address
		kvmmips_emulate_mmio(run_ptr, vcpu_ptr, (vaddr & ~0xffffffffe0000000));

		if(!(vcpu_ptr->arch.temp_cp0_cause & CAUSEF_BD))
			vcpu_ptr->arch.pc += 4;
		else
			kvmmips_emulate_branch(run_ptr, vcpu_ptr);

		//TODO: return value should be considered
		return RETURN_TO_HOST;
	}

#if 1
	if ((vaddr & 0xffffffff00000000) == 0x9800000000000000) {
		/** account info*/
		kvmmips_stat_exits(vcpu_ptr, MM_EXITS);

		//trans vaddr to normal I/O address
		kvmmips_emulate_mm(run_ptr, vcpu_ptr, vaddr);

		if(!(vcpu_ptr->arch.temp_cp0_cause & CAUSEF_BD))
			vcpu_ptr->arch.pc += 4;
		else
			kvmmips_emulate_branch(run_ptr, vcpu_ptr);

		//TODO: return value should be considered
		return RETURN_TO_GUEST;
	}
#endif

	/* account info */
	kvmmips_stat_exits(vcpu_ptr, ADEL_EXITS);

	/* TODO: inject to guest OS */
	kvmmips_queue_exception(vcpu_ptr, KVMMIPS_EXCEPTION_ADEL);

	return RETURN_TO_GUEST;
}

asmlinkage int do_kvm_ades(struct kvm_run* run_ptr , struct kvm_vcpu* vcpu_ptr)
{
	u64 vaddr = vcpu_ptr->arch.temp_cp0_badvaddr;

	run_ptr->exit_reason = KVM_EXIT_UNKNOWN;
	run_ptr->ready_for_interrupt_injection = 1;

	if (((vaddr & 0xfffffffff0000000) == 0xffffffffb0000000) &&
		((vaddr & 0xfffffffffff00000) != 0xffffffffbfc00000)) {
		/* account info */
		kvmmips_stat_exits(vcpu_ptr, MMIO_EXITS);

		//trans vaddr to normal I/O address
		kvmmips_emulate_mmio(run_ptr, vcpu_ptr, (vaddr & ~0xffffffffe0000000));

		if(!(vcpu_ptr->arch.temp_cp0_cause & CAUSEF_BD))
			vcpu_ptr->arch.pc += 4;
		else
			kvmmips_emulate_branch(run_ptr, vcpu_ptr);

		//TODO: return value should be considered
		return RETURN_TO_HOST;
	}

#if 1
	if ((vaddr & 0xffffffff00000000) == 0x9800000000000000) {
		/* account info */
		kvmmips_stat_exits(vcpu_ptr, MM_EXITS);

		//trans vaddr to normal I/O address
		kvmmips_emulate_mm(run_ptr, vcpu_ptr, vaddr);

		if(!(vcpu_ptr->arch.temp_cp0_cause & CAUSEF_BD))
			vcpu_ptr->arch.pc += 4;
		else
			kvmmips_emulate_branch(run_ptr, vcpu_ptr);

		//TODO: return value should be considered
		return RETURN_TO_GUEST;
	}
#endif

	/* account info */
	kvmmips_stat_exits(vcpu_ptr, ADES_EXITS);
	/* TODO: inject to guest OS */
	kvmmips_queue_exception(vcpu_ptr, KVMMIPS_EXCEPTION_ADES);

	return RETURN_TO_GUEST;
}

asmlinkage int do_kvm_ibe(struct kvm_run* run_ptr , struct kvm_vcpu* vcpu_ptr)
{
	run_ptr->exit_reason = KVM_EXIT_UNKNOWN;
	run_ptr->ready_for_interrupt_injection = 1;

	/* account info */
	kvmmips_stat_exits(vcpu_ptr, IBE_EXITS);
	/* TODO: inject to guest OS */
	kvmmips_queue_exception(vcpu_ptr, KVMMIPS_EXCEPTION_IBE);

	return RETURN_TO_GUEST;
}

asmlinkage int do_kvm_dbe(struct kvm_run* run_ptr , struct kvm_vcpu* vcpu_ptr)
{
	run_ptr->exit_reason = KVM_EXIT_UNKNOWN;
	run_ptr->ready_for_interrupt_injection = 1;
	/* account info */
	kvmmips_stat_exits(vcpu_ptr, DBE_EXITS);

	/* TODO: inject to guest OS */
	kvmmips_queue_exception(vcpu_ptr, KVMMIPS_EXCEPTION_DBE);

	return RETURN_TO_GUEST;
}

asmlinkage int do_kvm_syscall(struct kvm_run* run_ptr , struct kvm_vcpu* vcpu_ptr)
{
	run_ptr->exit_reason = KVM_EXIT_UNKNOWN;
	run_ptr->ready_for_interrupt_injection = 1;

	/* account info */
	kvmmips_stat_exits(vcpu_ptr, SYSCALL_EXITS);

	/* TODO: inject to guest OS */
	kvmmips_queue_exception(vcpu_ptr, KVMMIPS_EXCEPTION_SYSCALL);

	return RETURN_TO_GUEST;
}

asmlinkage int do_kvm_bp(struct kvm_run* run_ptr , struct kvm_vcpu* vcpu_ptr)
{
	run_ptr->exit_reason = KVM_EXIT_UNKNOWN;
	run_ptr->ready_for_interrupt_injection = 1;
	/* account info */
	kvmmips_stat_exits(vcpu_ptr, BP_EXITS);

	/* TODO: inject to guest OS */
	kvmmips_queue_exception(vcpu_ptr, KVMMIPS_EXCEPTION_BP);

	return RETURN_TO_GUEST;
}

asmlinkage int do_kvm_ri(struct kvm_run* run_ptr , struct kvm_vcpu* vcpu_ptr)
{
	run_ptr->exit_reason = KVM_EXIT_UNKNOWN;
	run_ptr->ready_for_interrupt_injection = 1;
	/* account info */
	kvmmips_stat_exits(vcpu_ptr, RI_EXITS);

	/* TODO: inject to guest OS */
	kvmmips_queue_exception(vcpu_ptr, KVMMIPS_EXCEPTION_RI);

	return RETURN_TO_GUEST;
}

#define KVMMIPS_CACHE_EXEC(reg,offset)  \
			__asm__ __volatile__(			\
				".set push\n"			\
				".set noreorder\n"		\
				".set mips3\n"			\
				"cache %1, 0x000(%0)\n" 	\
				".set pop\n"			\
				:				\
				:"r"(offset),			\
				 "i"(reg)			\
			);

extern u32 kvmmips_get_kernel_instruction(struct kvm_run* run_ptr, struct kvm_vcpu* vcpu_ptr);
/* emulate branch in case of exceptions which ocurs in branch delay */
void kvmmips_emulate_branch(struct kvm_run* run_ptr, struct kvm_vcpu* vcpu_ptr)
{
	unsigned int opcode;
	int reg,reg1,reg2;
	u64 temp, temp_cause = vcpu_ptr->arch.temp_cp0_cause;
	//to get the jump opcode
	vcpu_ptr->arch.temp_cp0_cause &= ~CAUSEF_BD;
	opcode = kvmmips_get_kernel_instruction(run_ptr, vcpu_ptr);
	vcpu_ptr->arch.temp_cp0_cause = temp_cause;

	switch ((opcode & OPCODE1) >> OPCODE1_OFFSET) {
	case 0:
		switch (opcode & 0x3f) {
		case 8:	//jr
			reg = (opcode & OPCODE2) >> OPCODE2_OFFSET;
			vcpu_ptr->arch.pc = vcpu_ptr->arch.gpr[reg];
			break;
		case 9: //jalr
			reg = (opcode & OPCODE2) >> OPCODE2_OFFSET;
			temp = vcpu_ptr->arch.gpr[reg];
			vcpu_ptr->arch.gpr[31] = vcpu_ptr->arch.pc + 8;
			vcpu_ptr->arch.pc = temp;
			break;
		default:
			printk("There is other branch ops should be emulate: %x\n", opcode);
			break;
		}
		break;
	case 1:
		switch ((opcode & OP1) >> OP1_OFFSET) {
		case 0://bltz
		case 2://bltzl
			reg = (opcode & OPCODE2) >> OPCODE2_OFFSET;
			if ((s64)vcpu_ptr->arch.gpr[reg] < 0) {
				u32 broffset = opcode & OFFSET_ID;
				vcpu_ptr->arch.pc =
					(u64)((s64)(kvmmips_sign_extern16(broffset) << 2)
					+ (s64)vcpu_ptr->arch.pc);
			} else {
				vcpu_ptr->arch.pc += 8;
			}
			break;
		case 1://bgez
		case 3://bgezl
			reg = (opcode & OPCODE2) >> OPCODE2_OFFSET;
			if ((s64)vcpu_ptr->arch.gpr[reg] >= 0) {
				u32 broffset = opcode & OFFSET_ID;
				vcpu_ptr->arch.pc =
					(u64)((s64)(kvmmips_sign_extern16(broffset) << 2)
					+ (s64)vcpu_ptr->arch.pc);
			} else {
				vcpu_ptr->arch.pc += 8;
			}
			break;
		case 16://bltzal
		case 18://bltzall
			reg = (opcode & OPCODE2) >> OPCODE2_OFFSET;
			if ((s64)vcpu_ptr->arch.gpr[reg] < 0) {
				u32 broffset = opcode & OFFSET_ID;
				vcpu_ptr->arch.gpr[31] = vcpu_ptr->arch.pc + 8;
				vcpu_ptr->arch.pc =
					(u64)((s64)(kvmmips_sign_extern16(broffset) << 2)
					+ (s64)vcpu_ptr->arch.pc);
			} else {
				vcpu_ptr->arch.pc += 8;
			}
		case 17://bgezal
		case 19://bgezall
			reg = (opcode & OPCODE2) >> OPCODE2_OFFSET;
			if ((s64)vcpu_ptr->arch.gpr[reg] >= 0) {
				u32 broffset = opcode & OFFSET_ID;
				vcpu_ptr->arch.gpr[31] = vcpu_ptr->arch.pc + 8;
				vcpu_ptr->arch.pc =
					(u64)((s64)(kvmmips_sign_extern16(broffset) << 2)
					+ (s64)vcpu_ptr->arch.pc);
			} else {
				vcpu_ptr->arch.pc += 8;
			}

		default:
			printk("There is other branch ops should be emulated: %x\n", opcode);
			break;
		}
		break;
	case 2: //j
		vcpu_ptr->arch.pc =
			(vcpu_ptr->arch.pc & ~0x0fffffff) | ((opcode & 0x03ffffff) << 2);
		break;
	case 3: //jal
		vcpu_ptr->arch.gpr[31] = vcpu_ptr->arch.pc + 8;
		vcpu_ptr->arch.pc =
			(vcpu_ptr->arch.pc & ~0x0fffffff) | ((opcode & 0x03ffffff) << 2);
		break;
	case 4: //beq
	case 20: //beql
		reg1 = (opcode & OPCODE2) >> OPCODE2_OFFSET;
		reg2 = (opcode & OP1) >> OP1_OFFSET;
		if (vcpu_ptr->arch.gpr[reg1] == vcpu_ptr->arch.gpr[reg2]) {
			u32 broffset = opcode & OFFSET_ID;
			vcpu_ptr->arch.pc =
				(u64)((s64)(kvmmips_sign_extern16(broffset) << 2)
				+ (s64)vcpu_ptr->arch.pc);
		} else {
			vcpu_ptr->arch.pc += 8;
		}
		break;
	case 5: //bne
	case 21: //bneql
		reg1 = (opcode & OPCODE2) >> OPCODE2_OFFSET;
		reg2 = (opcode & OP1) >> OP1_OFFSET;
		if (vcpu_ptr->arch.gpr[reg1] != vcpu_ptr->arch.gpr[reg2]) {
			u32 broffset = opcode & OFFSET_ID;
			vcpu_ptr->arch.pc =
				(u64)((s64)(kvmmips_sign_extern16(broffset) << 2)
				+ (s64)vcpu_ptr->arch.pc);
		} else {
			vcpu_ptr->arch.pc += 8;
		}
		break;
	case 6: //blez
	case 22: //blezl
		reg1 = (opcode & OPCODE2) >> OPCODE2_OFFSET;
		if ((s64)vcpu_ptr->arch.gpr[reg1] <= 0) {
			u32 broffset = opcode & OFFSET_ID;
			vcpu_ptr->arch.pc =
				(u64)((s64)(kvmmips_sign_extern16(broffset) << 2)
				+ (s64)vcpu_ptr->arch.pc);
		} else {
			vcpu_ptr->arch.pc += 8;
		}
		break;
	case 7://bgtz
	case 23: //bgtzl
		reg1 = (opcode & OPCODE2) >> OPCODE2_OFFSET;
		if ((s64)vcpu_ptr->arch.gpr[reg1] > 0) {
			u32 broffset = opcode & OFFSET_ID;
			vcpu_ptr->arch.pc =
				(u64)((s64)(kvmmips_sign_extern16(broffset) << 2)
				+ (s64)vcpu_ptr->arch.pc);
		} else {
			vcpu_ptr->arch.pc += 8;
		}
		break;
	case 17:
		if (((opcode & OPCODE2) >> OPCODE2_OFFSET) == 8) {
			reg = (opcode & OP1) >> OP1_OFFSET;
			switch (reg & 0x3) {
			case 0: //bc1f
			case 2: //bc1fl
				if (!((reg & ~0x3) >> 2)) {
					u32 broffset = opcode & OFFSET_ID;
					vcpu_ptr->arch.pc =
						(u64)((s64)(kvmmips_sign_extern16(broffset) << 2)
						+ (s64)vcpu_ptr->arch.pc);
				} else {
					vcpu_ptr->arch.pc += 8;
				}
				break;
			case 1: //bc1t
			case 3: //bc1tl
				if (((reg & ~0x3) >> 2) == 1) {
					u32 broffset = opcode & OFFSET_ID;
					vcpu_ptr->arch.pc =
						(u64)((s64)(kvmmips_sign_extern16(broffset) << 2)
						+ (s64)vcpu_ptr->arch.pc);
				} else {
					vcpu_ptr->arch.pc += 8;
				}
				break;
			default:
				printk("There is other branch ops should be emulated: %x\n",opcode);
				break;
			}
		} else {
			printk("There is other branch ops should be emulated: %x\n",opcode);
		}
		break;
	case 18:
		if (((opcode & OPCODE2) >> OPCODE2_OFFSET) == 8) {
			reg = (opcode & OP1) >> OP1_OFFSET;
			switch (reg & 0x3) {
			case 0: //bc2f
			case 2: //bc2fl
				if (!((reg & ~0x3) >> 2)) {
					u32 broffset = opcode & OFFSET_ID;
					vcpu_ptr->arch.pc =
						(u64)((s64)(kvmmips_sign_extern16(broffset) << 2)
						+ (s64)vcpu_ptr->arch.pc);
				} else {
					vcpu_ptr->arch.pc += 8;
				}
				break;
			case 1: //bc2t
			case 3: //bc2tl
				if (((reg & ~0x3) >> 2) == 1) {
					u32 broffset = opcode & OFFSET_ID;
					vcpu_ptr->arch.pc =
						(u64)((s64)(kvmmips_sign_extern16(broffset) << 2)
						+ (s64)vcpu_ptr->arch.pc);
				} else {
					vcpu_ptr->arch.pc += 8;
				}
				break;
			default:
				printk("There is other branch ops should be emulated: %x\n",opcode);
				break;
			}
		} else {
			printk("There is other branch ops should be emulated: %x\n",opcode);
		}
		break;
	default:
		printk("There is other branch ops should be emulated: %x, pc:%llx\n",
			opcode, vcpu_ptr->arch.pc);
		break;
	}
}

//TODO: done : <wait> ,<cache>
extern void kvmmips_update_shadow_tlb(struct kvmmips_vcpu_loongson* vcpu_ptr,
					u32 index, u64 old_entryhi);
asmlinkage int do_kvm_cpu(struct kvm_run* run_ptr, struct kvm_vcpu* vcpu_ptr)
{
	unsigned int opcode;
	unsigned int cpid;
	unsigned long __maybe_unused flags;
	int emulated = EMULATE_FAIL;
	int result = RETURN_TO_GUEST;
	int is_eret = 0;

	run_ptr->exit_reason = KVM_EXIT_UNKNOWN;
	run_ptr->ready_for_interrupt_injection = 1;

	/* account info */
	kvmmips_stat_exits(vcpu_ptr, CPU_EXITS);

	cpid = (vcpu_ptr->arch.temp_cp0_cause >> CAUSEB_CE) & 3;

	switch (cpid) {
	case 0:
		/* get opcode by epc */
		opcode = kvmmips_get_kernel_instruction(run_ptr, vcpu_ptr);

		if ((opcode & OPCODE1) >> OPCODE1_OFFSET == 16) {
			int reg1;
			int reg2;
			int sel;
			u64* real_reg;

			struct tlbe* temp_tlb;
			int index;
			u64 old_entryhi;
			switch((opcode & OPCODE2) >> OPCODE2_OFFSET) {
			case MFC0:
				reg1 = (opcode & OP1) >> OP1_OFFSET;
				reg2 = (opcode & OP2) >> OP2_OFFSET;
				sel  = opcode & XI_ID;

				kvmmips_stat_exits(vcpu_ptr, CPU_MFC0_EXITS);
				kvmmips_stat_cp0_uses(vcpu_ptr, reg2);
				real_reg = kvmmips_get_cp0_reg(vcpu_ptr , reg2 , sel);
				if (!real_reg)
				{
					emulated = EMULATE_FAIL;
					break;
				}
				vcpu_ptr->arch.gpr[reg1] =
					kvmmips_sign_extern((*real_reg) & 0xffffffff);
				emulated = EMULATE_DONE;
				break;
			case DMFC0:
				reg1 = (opcode & OP1) >> OP1_OFFSET;
				reg2 = (opcode & OP2) >> OP2_OFFSET;
				sel  = opcode & XI_ID;

				kvmmips_stat_exits(vcpu_ptr, CPU_DMFC0_EXITS);
				kvmmips_stat_cp0_uses(vcpu_ptr, reg2);
				real_reg = kvmmips_get_cp0_reg(vcpu_ptr , reg2 , sel);
				if (!real_reg)
				{
					emulated = EMULATE_FAIL;
					break;
				}
				vcpu_ptr->arch.gpr[reg1] = *real_reg;
				emulated = EMULATE_DONE;
				break;
			case MTC0:
				reg1 = (opcode & OP1) >> OP1_OFFSET;
				reg2 = (opcode & OP2) >> OP2_OFFSET;
				sel  = opcode & XI_ID;

				kvmmips_stat_exits(vcpu_ptr, CPU_MTC0_EXITS);
				kvmmips_stat_cp0_uses(vcpu_ptr, reg2);
				real_reg = kvmmips_get_cp0_reg(vcpu_ptr , reg2 , sel);
				if (!real_reg)
				{
					emulated = EMULATE_FAIL;
					break;
				}
				*real_reg =
					kvmmips_sign_extern(vcpu_ptr->arch.gpr[reg1] & 0xffffffff);
				//TODO:
				if (reg2 == 11) { //CP0_Compare
					struct kvm_interrupt inter;
					inter.irq = 7;
					kvmmips_dequeue_interrupt(vcpu_ptr,&inter);
				}
				if (reg2 == 22) {
					if(*real_reg != 4) {
						printk("ERROR: %s:%s:%d value(0x%llx)\n",
							__FILE__, __func__, __LINE__, *real_reg);
						while(1);
					}
					write_c0_diag(4);
				}
				if (reg2 == 6) { //CP0_Wired
					kvmmips_stat_exits(vcpu_ptr, WRITE_WIRED_NR);
				}
				emulated = EMULATE_DONE;
				break;
			case DMTC0:
				reg1 = (opcode & OP1) >> OP1_OFFSET;
				reg2 = (opcode & OP2) >> OP2_OFFSET;
				sel  = opcode & XI_ID;

				kvmmips_stat_exits(vcpu_ptr, CPU_DMTC0_EXITS);
				kvmmips_stat_cp0_uses(vcpu_ptr, reg2);
				real_reg = kvmmips_get_cp0_reg(vcpu_ptr , reg2 , sel);
				if (!real_reg)
				{
					emulated = EMULATE_FAIL;
					break;
				}
				*real_reg = vcpu_ptr->arch.gpr[reg1];
				//TODO:
				if (reg2 == 11) { //CP0_Compare
					struct kvm_interrupt inter;
					inter.irq = 7;
					kvmmips_dequeue_interrupt(vcpu_ptr,&inter);
				}
				if (reg2 == 22) {
					if(*real_reg != 4) {
						printk("ERROR: %s:%s:%d value(0x%llx)\n",
							__FILE__, __func__, __LINE__, *real_reg);
						while(1);
					}
					write_c0_diag(4);
				}
				if (reg2 == 6) { //CP0_Wired
					kvmmips_stat_exits(vcpu_ptr, WRITE_WIRED_NR);
					printk("ERROR %s:%s:%d: write CP0_Wired: epc(0x%lx)\n",
						__FILE__, __func__, __LINE__,
						(unsigned long)vcpu_ptr->arch.temp_cp0_epc);
				}
				emulated = EMULATE_DONE;
				break;
			case RDPGPR:
				/*not implemented in our cpu*/
				printk("Not implemented in our cpu: rdpgpr\n");
				emulated = EMULATE_FAIL;
				break;
			case XI:
				kvmmips_stat_exits(vcpu_ptr, CPU_XI_EXITS);
				switch (opcode & XI_ID) {
				case DI:
					reg1 = (opcode & OP1) >> OP1_OFFSET;
					//get status reg
					real_reg = kvmmips_get_cp0_reg(vcpu_ptr , 12,0);
					if (!real_reg)
					{
						emulated = EMULATE_FAIL;
						break;
					}

					if (*real_reg & 0x80000000)
						vcpu_ptr->arch.gpr[reg1] = *real_reg | 0xffffffff00000000;
					else
						vcpu_ptr->arch.gpr[reg1] = *real_reg & 0x00000000ffffffff;

					//disable interrupt
					*real_reg &= ~1;
					emulated = EMULATE_DONE;
					break;
				case EI:
					reg1 = (opcode & OP1) >> OP1_OFFSET;
					//get status reg
					real_reg = kvmmips_get_cp0_reg(vcpu_ptr , 12,0);
					if (!real_reg)
					{
						emulated = EMULATE_FAIL;
						break;
					}
					if (*real_reg & 0x80000000)
						vcpu_ptr->arch.gpr[reg1] = *real_reg | 0xffffffff00000000;
					else
						vcpu_ptr->arch.gpr[reg1] = *real_reg & 0x00000000ffffffff;

					//enable interrupt
					*real_reg |= 1;
					emulated = EMULATE_DONE;

					break;
				default:
					emulated = EMULATE_FAIL;
					printk("not implement tlb instruct\n");
				}
				break;
			case WRPGPR:
				printk("Not implemented in our cpu : wrpgpr\n");
				/*not implemented in our cpu*/
				emulated = EMULATE_FAIL;
				break;
			case TLBX:
				switch (opcode & TLB_ID) {
				case TLBR:
					printk("ERROR %s:%s:%d: this is a tlb instruction, epc(0x%lx)\n",
						__FILE__, __func__, __LINE__,
						(unsigned long)vcpu_ptr->arch.temp_cp0_epc);
					while(1);
					//TODO: origin
					kvmmips_stat_exits(vcpu_ptr, CPU_TLBR_EXITS);
					real_reg = kvmmips_get_cp0_reg(vcpu_ptr, 0, 0);
					if (!real_reg)
					{
						emulated = EMULATE_FAIL;
						break;
					}
					index = *real_reg;

					if (index >= to_loongson(vcpu_ptr)->guest_tlb_size)
					{
						index %= to_loongson(vcpu_ptr)->guest_tlb_size;
						*real_reg = index;
					}
					temp_tlb = &(to_loongson(vcpu_ptr)->guest_tlb[index]);
					vcpu_ptr->arch.cp0->cp0_entryhi  = temp_tlb->mask0;
					vcpu_ptr->arch.cp0->cp0_pagemask  = temp_tlb->mask1;
					vcpu_ptr->arch.cp0->cp0_entrylo0 = temp_tlb->mask2;
					vcpu_ptr->arch.cp0->cp0_entrylo1 = temp_tlb->mask3;

					emulated = EMULATE_DONE;
					break;
				case TLBWI:
					printk("ERROR %s:%s:%d: this is a tlb instruction, epc(0x%lx)\n",
						__FILE__, __func__, __LINE__,
						(unsigned long)vcpu_ptr->arch.temp_cp0_epc);
					while(1);

					//TODO: origin
					kvmmips_stat_exits(vcpu_ptr, CPU_TLBWI_EXITS);
					real_reg = kvmmips_get_cp0_reg(vcpu_ptr, 0, 0);
					if (!real_reg)
					{
						emulated = EMULATE_FAIL;
						break;
					}
					index = *real_reg;

					if (index >= to_loongson(vcpu_ptr)->guest_tlb_size)
					{
						index %= to_loongson(vcpu_ptr)->guest_tlb_size;
						*real_reg = index;
					}
					temp_tlb = &(to_loongson(vcpu_ptr)->guest_tlb[index]);
					old_entryhi = temp_tlb->mask0;
					temp_tlb->mask0 = vcpu_ptr->arch.cp0->cp0_entryhi;
					temp_tlb->mask1 = vcpu_ptr->arch.cp0->cp0_pagemask;
					temp_tlb->mask2 = vcpu_ptr->arch.cp0->cp0_entrylo0;
					temp_tlb->mask3 = vcpu_ptr->arch.cp0->cp0_entrylo1;
					kvmmips_update_shadow_tlb(to_loongson(vcpu_ptr), index, old_entryhi);
					emulated = EMULATE_DONE;
					break;
				case TLBWR:
					printk("ERROR %s:%s:%d: this is a tlb instruction, epc(0x%lx)\n",
						__FILE__, __func__, __LINE__,
						(unsigned long)vcpu_ptr->arch.temp_cp0_epc);
					while(1);

					//TODO: origin
					kvmmips_stat_exits(vcpu_ptr, CPU_TLBWR_EXITS);
					real_reg = kvmmips_get_cp0_reg(vcpu_ptr, 1, 0);
					if (!real_reg)
					{
						emulated = EMULATE_FAIL;
						break;
					}
					index = *real_reg;

					if (index >= to_loongson(vcpu_ptr)->guest_tlb_size)
					{
						index %= to_loongson(vcpu_ptr)->guest_tlb_size;
						*real_reg = index++;
					}
					temp_tlb = &(to_loongson(vcpu_ptr)->guest_tlb[index]);
					old_entryhi = temp_tlb->mask0;
					temp_tlb->mask0 = vcpu_ptr->arch.cp0->cp0_entryhi;
					temp_tlb->mask1 = vcpu_ptr->arch.cp0->cp0_pagemask;
					temp_tlb->mask2 = vcpu_ptr->arch.cp0->cp0_entrylo0;
					temp_tlb->mask3 = vcpu_ptr->arch.cp0->cp0_entrylo1;
					kvmmips_update_shadow_tlb(to_loongson(vcpu_ptr), index, old_entryhi);

					emulated = EMULATE_DONE;
					break;
				case TLBP:
					printk("ERROR %s:%s:%d: this is a tlb instruction, epc(0x%lx)\n",
						__FILE__, __func__, __LINE__,
						(unsigned long)vcpu_ptr->arch.temp_cp0_epc);
					while(1);

					//TODO: origin
					kvmmips_stat_exits(vcpu_ptr, CPU_TLBP_EXITS);
					for (index = 0; index < to_loongson(vcpu_ptr)->guest_tlb_size; index++)
					{
						//TODO: define something
						temp_tlb = &(to_loongson(vcpu_ptr)->guest_tlb[index]);
						if (((temp_tlb->mask0 & (~0x7fff)) ==
							(vcpu_ptr->arch.cp0->cp0_entryhi  & (~0x7fff))) &&
							((temp_tlb->mask2 & 0x1) == 1 || (temp_tlb->mask3 & 0x1) == 1 ||
							((vcpu_ptr->arch.cp0->cp0_entryhi & 0xff) == (temp_tlb->mask0 & 0xff))))
							break;
					}
					real_reg = kvmmips_get_cp0_reg(vcpu_ptr, 0, 0);
					if (!real_reg)
					{
						emulated = EMULATE_FAIL;
						break;
					}

					/* TODO: should be considered more when search failed */
					if (index < to_loongson(vcpu_ptr)->guest_tlb_size)
						*real_reg = index;
					else
						*real_reg = -1;

					emulated = EMULATE_DONE;
					break;
				case ERET:
					kvmmips_stat_exits(vcpu_ptr, CPU_ERET_EXITS);
					if (vcpu_ptr->arch.cp0->cp0_status & 0x4)
						vcpu_ptr->arch.cp0->cp0_status &= ~0x4;
					else
						vcpu_ptr->arch.cp0->cp0_status &= ~0x2;
					//TODO: this only be fit for 32 bit guest os
#ifdef CONFIG_KVM64_SUPPORT
					vcpu_ptr->arch.pc = vcpu_ptr->arch.cp0->cp0_epc;
#else
					vcpu_ptr->arch.pc =
						kvmmips_sign_extern(vcpu_ptr->arch.cp0->cp0_epc & 0xffffffff);
#endif
					is_eret = 1;
					emulated = EMULATE_DONE;
					break;
				default:
					printk("Unimplemented tlb instruction!\n");
					emulated = EMULATE_FAIL;
					break;
				}
				break;
			default:
				printk("%s:%d: opcode(0x%x)\n", __func__, __LINE__, opcode);
				break;
			}
			/* return the symbols or kvm_run */
			if (vcpu_ptr->arch.temp_cp0_cause & CAUSEF_BD) {
				printk("AN EXCEPTION OCCUR IN BRANCH STOP, pc:%llx, %s:%d\n",
					vcpu_ptr->arch.pc, __func__, __LINE__);
			} else {
				if(!is_eret)
					vcpu_ptr->arch.pc += 4;
			}
		} else {
			if ((opcode & OPCODE1) >> OPCODE1_OFFSET == 47) {
				goto cache_tackle;
			}
			printk("ERROR: %s:%d: opcode(0x%x) pc: %llx  epc: %llx\n",
				__func__, __LINE__, opcode,
				vcpu_ptr->arch.pc, vcpu_ptr->arch.temp_cp0_epc);
			while(1);
			/* TODO: maybe inject to guest OS */
			kvmmips_queue_exception(vcpu_ptr, KVMMIPS_EXCEPTION_CPU);
			if (vcpu_ptr->arch.temp_cp0_cause & CAUSEF_BD) {
				printk("AN EXCEPTION OCCUR IN BRANCH STOP, pc:%llx, %s:%d\n",
					vcpu_ptr->arch.pc, __func__, __LINE__);
			} else {
				vcpu_ptr->arch.pc += 4;
			}
			break;
		}

		if (emulated == EMULATE_DONE)
		{
			if (signal_pending(current))
			{
				run_ptr->exit_reason = KVM_EXIT_INTR;
			}
			result = RETURN_TO_GUEST;
		} else {
			printk("%s:%d: opcode(0x%x) pc: %llx  epc: %llx\n", __func__, __LINE__,
				opcode, vcpu_ptr->arch.pc, vcpu_ptr->arch.temp_cp0_epc);
			printk("Emulate instrcution error!\n");
			result = RETURN_TO_HOST;
			run_ptr->exit_reason = KVM_EXIT_UNKNOWN;
		}
		break;
	case 1:
		/* maybe inject to guest OS */
		kvmmips_queue_exception(vcpu_ptr, KVMMIPS_EXCEPTION_CPU);
		kvmmips_stat_exits(vcpu_ptr, CPU_CP1_EXITS);
		break;
	case 2:
		/* maybe inject to guest OS */
		kvmmips_queue_exception(vcpu_ptr, KVMMIPS_EXCEPTION_CPU);
		kvmmips_stat_exits(vcpu_ptr, CPU_CP2_EXITS);
		break;
	case 3:
		opcode = kvmmips_get_kernel_instruction(run_ptr, vcpu_ptr);
cache_tackle:
		printk("ERROR: %s:%s:%d: epc(0x%lx), opecode(0x%lx)\n",
			__FILE__, __func__, __LINE__,
			(unsigned long)vcpu_ptr->arch.temp_cp0_epc, (unsigned long)opcode);
		while(1);
		/* when add tackling cache ops in user mode attribute, it may not be used */
		if ((opcode & OPCODE1) >> OPCODE1_OFFSET == 47) {
			if (vcpu_ptr->arch.temp_cp0_cause & CAUSEF_BD) {
				printk("AN EXCEPTION OCCUR IN BRANCH STOP, pc:%llx, %s:%d\n",
					vcpu_ptr->arch.pc, __func__, __LINE__);
			} else {
				vcpu_ptr->arch.pc += 4;
			}
		}
		break;
	default:
		opcode = kvmmips_get_kernel_instruction(run_ptr, vcpu_ptr);
		printk("%s:%d: opcode(0x%x)\n", __func__, __LINE__, opcode);
		break;
	}

	return result;
	/* TODO: the cp0 unavailable exception */
}

asmlinkage int do_kvm_ov(struct kvm_run* run_ptr, struct kvm_vcpu* vcpu_ptr)
{
	run_ptr->exit_reason = KVM_EXIT_UNKNOWN;
	run_ptr->ready_for_interrupt_injection = 1;
	/* account info */
	kvmmips_stat_exits(vcpu_ptr, OV_EXITS);

	/* TODO: inject to guest OS */
	kvmmips_queue_exception(vcpu_ptr, KVMMIPS_EXCEPTION_OV);

	return RETURN_TO_GUEST;
}

u64 kvmmips_get_host_vaddr(struct kvm_run* run_ptr, struct kvm_vcpu* vcpu_ptr, u64 guest_vaddr)
{
	u64 guest_paddr;
	u64 guest_pfn;
	u64 host_vaddr;

	if((guest_vaddr & 0xfffffffff0000000) == 0x4000000080000000)
		guest_paddr = guest_vaddr & 0x0fffffff;
	else if((guest_vaddr & 0xffffffff00000000) == 0x4000000100000000)
		guest_paddr = guest_vaddr & 0x0ffffffffULL;
	else {
		printk(KERN_ERR "[KVM-ERROR]kvmmips: (0x%lx) is not guest kernel vaddr!\n",
			(unsigned long)guest_vaddr);
		return -1;
	}

	guest_pfn = guest_paddr >> PAGE_SHIFT;

	if (kvm_is_visible_gfn(vcpu_ptr->kvm, guest_pfn))
	{
		struct page *new_page;

		new_page = gfn_to_page(vcpu_ptr->kvm, guest_pfn);
		if (is_error_page(new_page))
		{
			printk(KERN_ERR "[KVM-ERROR]kvmmips: Couldn't get guest page for gfn %lx!\n",
				(unsigned long)guest_pfn);
			kvm_release_page_clean(new_page);
			return -1;
		}
		host_vaddr = 0x9800000000000000 |
			page_to_phys(new_page) | (guest_vaddr & ((1 << PAGE_SHIFT) -1));

		/* release the count of page */
		kvm_release_page_clean(new_page);
		return host_vaddr;
	} else {
		printk(KERN_ERR "[KVM-ERROR]kvmmips: invisible gfn %lx!\n",
			(unsigned long)guest_pfn);
		return -1;
	}
}

extern void local_flush_tlb_all(void);
extern void kvmmips_local_flush_tlb_page(unsigned long page);
asmlinkage int do_kvm_tr(struct kvm_run* run_ptr , struct kvm_vcpu* vcpu_ptr)
{
	u64 temp;

	run_ptr->exit_reason = KVM_EXIT_UNKNOWN;
	run_ptr->ready_for_interrupt_injection = 1;
	/* account info */
	kvmmips_stat_exits(vcpu_ptr, TR_EXITS);

	switch(vcpu_ptr->arch.gpr[2]) {
	case 10001: //guest share paravirt_cp0 with hypervisor: BYTE
		temp = kvmmips_get_host_vaddr(run_ptr, vcpu_ptr, vcpu_ptr->arch.gpr[4]);

		vcpu_ptr->arch.cp0_origin = vcpu_ptr->arch.gpr[4];
		*(struct kvmmips_cp0_reg*)temp = *(vcpu_ptr->arch.cp0);

		//release origin cp0
		kfree(vcpu_ptr->arch.cp0);
		vcpu_ptr->arch.is_cp0_released = 1;

		//set arch.cp0 to new cp0
		vcpu_ptr->arch.cp0 = (struct kvmmips_cp0_reg*)temp;
		break;
	case 10002: //insert swapper_pg_dir to hypervisor: BYTE
		to_loongson(vcpu_ptr)->swapper_pg_dir =
			(pgd_t*)kvmmips_get_host_vaddr(run_ptr, vcpu_ptr, vcpu_ptr->arch.gpr[4]);
		kvmmips_stat_exits(vcpu_ptr, TR_INSERTSWAPPER_EXITS);
		break;
	case 10003: //get host address: BYTE
		vcpu_ptr->arch.gpr[2] =
			kvmmips_get_host_vaddr(run_ptr, vcpu_ptr, vcpu_ptr->arch.gpr[4]);
		kvmmips_stat_exits(vcpu_ptr, TR_GETHOSTVADDR_EXITS);
		break;
	case 10004: //insert pgd_current to hypervisor: BYTE
		to_loongson(vcpu_ptr)->pgd_current =
			(unsigned long)kvmmips_get_host_vaddr(run_ptr, vcpu_ptr, vcpu_ptr->arch.gpr[4]);
		kvmmips_stat_exits(vcpu_ptr, TR_INSERTPGCURRENT_EXITS);
		break;
	case 10005:
		local_flush_tlb_all();
		kvmmips_stat_exits(vcpu_ptr, TR_TLBFLUSHALL_EXITS);
		break;
	case 10006:
		kvmmips_local_flush_tlb_page(vcpu_ptr->arch.gpr[4]);
		kvmmips_stat_exits(vcpu_ptr, TR_TLBFLUSHPAGE_EXITS);
		break;
	default:
		break;
	}
	vcpu_ptr->arch.pc += 4;

	return RETURN_TO_GUEST;
}

asmlinkage int do_kvm_fpe(struct kvm_run* run_ptr, struct kvm_vcpu* vcpu_ptr)
{
	run_ptr->exit_reason = KVM_EXIT_UNKNOWN;
	run_ptr->ready_for_interrupt_injection = 1;
	/* account info */
	kvmmips_stat_exits(vcpu_ptr, FPE_EXITS);

	/* TODO: inject to guest OS */
	kvmmips_queue_exception(vcpu_ptr, KVMMIPS_EXCEPTION_FPE);

	return RETURN_TO_GUEST;
}

asmlinkage int do_kvm_c2e(struct kvm_run* run_ptr, struct kvm_vcpu* vcpu_ptr)
{
	run_ptr->exit_reason = KVM_EXIT_UNKNOWN;
	run_ptr->ready_for_interrupt_injection = 1;
	/* account info */
	kvmmips_stat_exits(vcpu_ptr, C2E_EXITS);

	/* TODO: inject to guest OS */
	kvmmips_queue_exception(vcpu_ptr, KVMMIPS_EXCEPTION_C2E);

	return RETURN_TO_GUEST;
}

asmlinkage int do_kvm_mdmx(struct kvm_run* run_ptr, struct kvm_vcpu* vcpu_ptr)
{
	run_ptr->exit_reason = KVM_EXIT_UNKNOWN;
	run_ptr->ready_for_interrupt_injection = 1;
	/* account info */
	kvmmips_stat_exits(vcpu_ptr, MDMX_EXITS);

	/* TODO: inject to guest OS */
	kvmmips_queue_exception(vcpu_ptr, KVMMIPS_EXCEPTION_MDMX);

	return RETURN_TO_GUEST;
}

asmlinkage int do_kvm_watch(struct kvm_run* run_ptr , struct kvm_vcpu* vcpu_ptr)
{
	run_ptr->exit_reason = KVM_EXIT_UNKNOWN;
	run_ptr->ready_for_interrupt_injection = 1;
	/* account info */
	kvmmips_stat_exits(vcpu_ptr, WATCH_EXITS);

	/* TODO: inject to guest OS */
	kvmmips_queue_exception(vcpu_ptr, KVMMIPS_EXCEPTION_WATCH);

	return RETURN_TO_GUEST;
}

asmlinkage int do_kvm_mcheck(struct kvm_run* run_ptr , struct kvm_vcpu* vcpu_ptr)
{
	run_ptr->exit_reason = KVM_EXIT_UNKNOWN;
	run_ptr->ready_for_interrupt_injection = 1;
	/* account info */
	kvmmips_stat_exits(vcpu_ptr, MCHECK_EXITS);

	/* TODO: inject to guest OS */
	kvmmips_queue_exception(vcpu_ptr, KVMMIPS_EXCEPTION_MCHECK);

	return RETURN_TO_GUEST;
}

asmlinkage int do_kvm_mt(struct kvm_run* run_ptr , struct kvm_vcpu* vcpu_ptr)
{
	run_ptr->exit_reason = KVM_EXIT_UNKNOWN;
	run_ptr->ready_for_interrupt_injection = 1;
	/* account info */
	kvmmips_stat_exits(vcpu_ptr, MT_EXITS);

	/* TODO: inject to guest OS */
	kvmmips_queue_exception(vcpu_ptr, KVMMIPS_EXCEPTION_MT);

	return RETURN_TO_GUEST;
}

asmlinkage int do_kvm_dsp(struct kvm_run* run_ptr, struct kvm_vcpu* vcpu_ptr)
{
	run_ptr->exit_reason = KVM_EXIT_UNKNOWN;
	run_ptr->ready_for_interrupt_injection = 1;
	/* account info */
	kvmmips_stat_exits(vcpu_ptr, DSP_EXITS);

	/* TODO: inject to guest OS */
	kvmmips_queue_exception(vcpu_ptr, KVMMIPS_EXCEPTION_DSP);

	return RETURN_TO_GUEST;
}

asmlinkage int do_kvm_reserved(struct kvm_run* run_ptr , struct kvm_vcpu* vcpu_ptr)
{
	run_ptr->exit_reason = KVM_EXIT_UNKNOWN;
	run_ptr->ready_for_interrupt_injection = 1;
	/* account info */
	kvmmips_stat_exits(vcpu_ptr, RESERVED_EXITS);

	/* TODO: inject to guest OS */
	kvmmips_queue_exception(vcpu_ptr, KVMMIPS_EXCEPTION_TOP);

	return RETURN_TO_GUEST;
}

extern void kvmmips_set_except_vector(int n, void *addr);
extern void kvmmips_set_tlbmiss_handler(unsigned long value);
extern void kvmmips_restore_host_exception(void);
extern void kvmmips_inject_exception_generic(void);
extern void kvmmips_set_vcpu_mips_global(int index, void* value);

/* temp to avoid long jump error in loongson_interrupt.S */
void  kvmmips_restore_host(void)
{
	kvmmips_restore_host_exception();
}

void kvmmips_init_except_vector(void)
{
	int i;

	printk("Inject exception begin!\n");

	/* inject tlb miss exception */
	kvmmips_set_tlbmiss_handler((unsigned long)handle_kvm_tlbmiss);

	for (i = 0; i <= 31; i++)
		kvmmips_set_except_vector(i, handle_kvm_reserved);

	/* change the normal cp* unavailable exception */
	kvmmips_set_except_vector(0, handle_kvm_int);
	kvmmips_set_except_vector(1, handle_kvm_mod);
	kvmmips_set_except_vector(2, handle_kvm_tlbl);
	kvmmips_set_except_vector(3, handle_kvm_tlbs);
	kvmmips_set_except_vector(4, handle_kvm_adel);
	kvmmips_set_except_vector(5, handle_kvm_ades);
	kvmmips_set_except_vector(6, handle_kvm_ibe);
	kvmmips_set_except_vector(7, handle_kvm_dbe);
	kvmmips_set_except_vector(8, handle_kvm_syscall);
	kvmmips_set_except_vector(9, handle_kvm_bp);
	kvmmips_set_except_vector(10, handle_kvm_ri);
	kvmmips_set_except_vector(11, handle_kvm_cpu);
	kvmmips_set_except_vector(12, handle_kvm_ov);
	kvmmips_set_except_vector(13, handle_kvm_tr);
	kvmmips_set_except_vector(15, handle_kvm_fpe);
	kvmmips_set_except_vector(18, handle_kvm_c2e);
	kvmmips_set_except_vector(22, handle_kvm_mdmx);
	kvmmips_set_except_vector(23, handle_kvm_watch);
	kvmmips_set_except_vector(24, handle_kvm_mcheck);
	kvmmips_set_except_vector(25, handle_kvm_mt);
	kvmmips_set_except_vector(26, handle_kvm_dsp);
}

extern void getnstimeofday(struct timespec *ts);
extern void kvmmips_set_vcpu_kernel_pg_dir(int index,
		struct kvmmips_vcpu_loongson* vcpu_loongson);
void  kvmmips_inject_exception(int index, void* vcpu_ptr)
{
	/* sync time */
	struct timespec ts;
	getnstimeofday(&ts);
	((struct kvm_vcpu*)vcpu_ptr)->arch.cp0->xtime_sec = ts.tv_sec;
	((struct kvm_vcpu*)vcpu_ptr)->arch.cp0->xtime_nsec = ts.tv_nsec;

	//set kvmmips_vcpu_mips_global
	kvmmips_set_vcpu_mips_global(index / 8, vcpu_ptr);

	kvmmips_set_vcpu_kernel_pg_dir(index / 8, to_loongson(vcpu_ptr));

	kvmmips_inject_exception_generic();
}
#endif /* CONFIG_KVM_MIPS_LOONGSON3 */

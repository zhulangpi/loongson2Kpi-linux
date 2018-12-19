/*
* This file is subject to the terms and conditions of the GNU General Public
* License.  See the file "COPYING" in the main directory of this archive
* for more details.
*
* KVM/MIPS TLB handling, this file is part of the Linux host kernel so that
* TLB handlers run from KSEG0
*
* Copyright (C) 2012  MIPS Technologies, Inc.  All rights reserved.
* Authors: Sanjay Lal <sanjayl@kymasys.com>
*/

#ifndef CONFIG_KVM_MIPS_LOONGSON3
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/kvm_host.h>
#include <linux/srcu.h>


#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mmu_context.h>
#include <asm/pgtable.h>
#include <asm/cacheflush.h>

#undef CONFIG_MIPS_MT
#include <asm/r4kcache.h>
#define CONFIG_MIPS_MT

#define KVM_GUEST_PC_TLB    0
#define KVM_GUEST_SP_TLB    1

#define PRIx64 "llx"

/* Use VZ EntryHi.EHINV to invalidate TLB entries */
#define UNIQUE_ENTRYHI(idx) (CKSEG0 + ((idx) << (PAGE_SHIFT + 1)))

atomic_t kvm_mips_instance;
EXPORT_SYMBOL(kvm_mips_instance);

/* These function pointers are initialized once the KVM module is loaded */
pfn_t(*kvm_mips_gfn_to_pfn) (struct kvm *kvm, gfn_t gfn);
EXPORT_SYMBOL(kvm_mips_gfn_to_pfn);

void (*kvm_mips_release_pfn_clean) (pfn_t pfn);
EXPORT_SYMBOL(kvm_mips_release_pfn_clean);

bool(*kvm_mips_is_error_pfn) (pfn_t pfn);
EXPORT_SYMBOL(kvm_mips_is_error_pfn);

uint32_t kvm_mips_get_kernel_asid(struct kvm_vcpu *vcpu)
{
	return vcpu->arch.guest_kernel_asid[smp_processor_id()] & ASID_MASK;
}


uint32_t kvm_mips_get_user_asid(struct kvm_vcpu *vcpu)
{
	return vcpu->arch.guest_user_asid[smp_processor_id()] & ASID_MASK;
}

inline uint32_t kvm_mips_get_commpage_asid (struct kvm_vcpu *vcpu)
{
	return vcpu->kvm->arch.commpage_tlb;
}


/*
 * Structure defining an tlb entry data set.
 */

void kvm_mips_dump_host_tlbs(void)
{
	unsigned long old_entryhi;
	unsigned long old_pagemask;
	struct kvm_mips_tlb tlb;
	unsigned long flags;
	int i;

	local_irq_save(flags);

	old_entryhi = read_c0_entryhi();
	old_pagemask = read_c0_pagemask();

	printk("HOST TLBs:\n");
	printk("ASID: %#lx\n", read_c0_entryhi() & ASID_MASK);

	for (i = 0; i < current_cpu_data.tlbsize; i++) {
		write_c0_index(i);
		mtc0_tlbw_hazard();

		tlb_read();
		tlbw_use_hazard();

		tlb.tlb_hi = read_c0_entryhi();
		tlb.tlb_lo0 = read_c0_entrylo0();
		tlb.tlb_lo1 = read_c0_entrylo1();
		tlb.tlb_mask = read_c0_pagemask();

		printk("TLB%c%3d Hi 0x%08lx ",
		       (tlb.tlb_lo0 | tlb.tlb_lo1) & MIPS3_PG_V ? ' ' : '*',
		       i, tlb.tlb_hi);
		printk("Lo0=0x%09" PRIx64 " %c%c attr %lx ",
		       (uint64_t) mips3_tlbpfn_to_paddr(tlb.tlb_lo0),
		       (tlb.tlb_lo0 & MIPS3_PG_D) ? 'D' : ' ',
		       (tlb.tlb_lo0 & MIPS3_PG_G) ? 'G' : ' ',
		       (tlb.tlb_lo0 >> 3) & 7);
		printk("Lo1=0x%09" PRIx64 " %c%c attr %lx sz=%lx\n",
		       (uint64_t) mips3_tlbpfn_to_paddr(tlb.tlb_lo1),
		       (tlb.tlb_lo1 & MIPS3_PG_D) ? 'D' : ' ',
		       (tlb.tlb_lo1 & MIPS3_PG_G) ? 'G' : ' ',
		       (tlb.tlb_lo1 >> 3) & 7, tlb.tlb_mask);
	}
	write_c0_entryhi(old_entryhi);
	write_c0_pagemask(old_pagemask);
	mtc0_tlbw_hazard();
	local_irq_restore(flags);
}

void kvm_mips_dump_guest_tlbs(struct kvm_vcpu *vcpu)
{
	struct mips_coproc *cop0 = vcpu->arch.cop0;
	struct kvm_mips_tlb tlb;
	int i;

	printk("Guest TLBs:\n");
	printk("Guest EntryHi: %#lx\n", kvm_read_c0_guest_entryhi(cop0));

	for (i = 0; i < KVM_MIPS_GUEST_TLB_SIZE; i++) {
		tlb = vcpu->arch.guest_tlb[i];
		printk("TLB%c%3d Hi 0x%08lx ",
		       (tlb.tlb_lo0 | tlb.tlb_lo1) & MIPS3_PG_V ? ' ' : '*',
		       i, tlb.tlb_hi);
		printk("Lo0=0x%09" PRIx64 " %c%c attr %lx ",
		       (uint64_t) mips3_tlbpfn_to_paddr(tlb.tlb_lo0),
		       (tlb.tlb_lo0 & MIPS3_PG_D) ? 'D' : ' ',
		       (tlb.tlb_lo0 & MIPS3_PG_G) ? 'G' : ' ',
		       (tlb.tlb_lo0 >> 3) & 7);
		printk("Lo1=0x%09" PRIx64 " %c%c attr %lx sz=%lx\n",
		       (uint64_t) mips3_tlbpfn_to_paddr(tlb.tlb_lo1),
		       (tlb.tlb_lo1 & MIPS3_PG_D) ? 'D' : ' ',
		       (tlb.tlb_lo1 & MIPS3_PG_G) ? 'G' : ' ',
		       (tlb.tlb_lo1 >> 3) & 7, tlb.tlb_mask);
	}
}

void kvm_mips_dump_shadow_tlbs(struct kvm_vcpu *vcpu)
{
	int i;
	volatile struct kvm_mips_tlb tlb;

	printk("Shadow TLBs:\n");
	for (i = 0; i < KVM_MIPS_GUEST_TLB_SIZE; i++) {
		tlb = vcpu->arch.shadow_tlb[smp_processor_id()][i];
		printk("TLB%c%3d Hi 0x%08lx ",
		       (tlb.tlb_lo0 | tlb.tlb_lo1) & MIPS3_PG_V ? ' ' : '*',
		       i, tlb.tlb_hi);
		printk("Lo0=0x%09" PRIx64 " %c%c attr %lx ",
		       (uint64_t) mips3_tlbpfn_to_paddr(tlb.tlb_lo0),
		       (tlb.tlb_lo0 & MIPS3_PG_D) ? 'D' : ' ',
		       (tlb.tlb_lo0 & MIPS3_PG_G) ? 'G' : ' ',
		       (tlb.tlb_lo0 >> 3) & 7);
		printk("Lo1=0x%09" PRIx64 " %c%c attr %lx sz=%lx\n",
		       (uint64_t) mips3_tlbpfn_to_paddr(tlb.tlb_lo1),
		       (tlb.tlb_lo1 & MIPS3_PG_D) ? 'D' : ' ',
		       (tlb.tlb_lo1 & MIPS3_PG_G) ? 'G' : ' ',
		       (tlb.tlb_lo1 >> 3) & 7, tlb.tlb_mask);
	}
}

static int kvm_mips_map_page(struct kvm *kvm, gfn_t gfn)
{
	int srcu_idx, err = 0;
	pfn_t pfn;

	if (kvm->arch.guest_pmap[gfn] != KVM_INVALID_PAGE)
		return 0;

        srcu_idx = srcu_read_lock(&kvm->srcu);
	pfn = kvm_mips_gfn_to_pfn(kvm, gfn);

	if (kvm_mips_is_error_pfn(pfn)) {
		kvm_err("Couldn't get pfn for gfn %#" PRIx64 "!\n", gfn);
		err = -EFAULT;
		goto out;
	}

	kvm->arch.guest_pmap[gfn] = pfn;
out:
	srcu_read_unlock(&kvm->srcu, srcu_idx);
	return err;
}

/* Translate guest KSEG0 addresses to Host PA */
unsigned long kvm_mips_translate_guest_kseg0_to_hpa(struct kvm_vcpu *vcpu,
	unsigned long gva)
{
	gfn_t gfn;
	uint32_t offset = gva & ~PAGE_MASK;
	struct kvm *kvm = vcpu->kvm;

	if (KVM_GUEST_KSEGX(gva) != KVM_GUEST_KSEG0) {
		kvm_err("%s/%p: Invalid gva: %#lx\n", __func__,
			__builtin_return_address(0), gva);
		return KVM_INVALID_PAGE;
	}

	gfn = (KVM_GUEST_CPHYSADDR(gva) >> PAGE_SHIFT);

	if (gfn >= kvm->arch.guest_pmap_npages) {
		kvm_err("%s: Invalid gfn: %#llx, GVA: %#lx\n", __func__, gfn,
			gva);
		return KVM_INVALID_PAGE;
	}

	if (kvm_mips_map_page(vcpu->kvm, gfn) < 0)
		return KVM_INVALID_ADDR;

	return (kvm->arch.guest_pmap[gfn] << PAGE_SHIFT) + offset;
}

/* XXXKYMA: Must be called with interrupts disabled */
/* set flush_dcache_mask == 0 if no dcache flush required */
int
kvm_mips_host_tlb_write(struct kvm_vcpu *vcpu, unsigned long entryhi,
	unsigned long entrylo0, unsigned long entrylo1, int flush_dcache_mask)
{
	unsigned long flags;
	unsigned long old_entryhi;
	volatile int idx;

	local_irq_save(flags);


	old_entryhi = read_c0_entryhi();
	write_c0_entryhi(entryhi);
	mtc0_tlbw_hazard();

	tlb_probe();
	tlb_probe_hazard();
	idx = read_c0_index();

	if (idx > current_cpu_data.tlbsize) {
		kvm_err("%s: Invalid Index: %d\n", __func__, idx);
		kvm_mips_dump_host_tlbs();
		return -1;
	}

	if (idx < 0) {
		idx = read_c0_random() % current_cpu_data.tlbsize;
		write_c0_index(idx);
		mtc0_tlbw_hazard();
	}
	write_c0_entrylo0(entrylo0);
	write_c0_entrylo1(entrylo1);
	mtc0_tlbw_hazard();

	tlb_write_indexed();
	tlbw_use_hazard();

#ifdef DEBUG
	if (debug) {
		kvm_debug("@ %#lx idx: %2d [entryhi(R): %#lx] "
			  "entrylo0(R): 0x%08lx, entrylo1(R): 0x%08lx\n",
			  vcpu->arch.pc, idx, read_c0_entryhi(),
			  read_c0_entrylo0(), read_c0_entrylo1());
	}
#endif

	/* Flush D-cache */
	if (flush_dcache_mask) {
		if (entrylo0 & MIPS3_PG_V) {
			++vcpu->stat.flush_dcache_exits;
			flush_data_cache_page((entryhi & VPN2_MASK) & ~flush_dcache_mask);
		}
		if (entrylo1 & MIPS3_PG_V) {
			++vcpu->stat.flush_dcache_exits;
			flush_data_cache_page(((entryhi & VPN2_MASK) & ~flush_dcache_mask) |
				(0x1 << PAGE_SHIFT));
		}
	}

	/* Restore old ASID */
	write_c0_entryhi(old_entryhi);
	mtc0_tlbw_hazard();
	tlbw_use_hazard();
	local_irq_restore(flags);
	return 0;
}


/* XXXKYMA: Must be called with interrupts disabled */
int kvm_mips_handle_kseg0_tlb_fault(unsigned long badvaddr,
	struct kvm_vcpu *vcpu)
{
	gfn_t gfn;
	pfn_t pfn0, pfn1;
	unsigned long vaddr = 0;
	unsigned long entryhi = 0, entrylo0 = 0, entrylo1 = 0;
	int even;
	struct kvm *kvm = vcpu->kvm;
	const int flush_dcache_mask = 0;


	if (KVM_GUEST_KSEGX(badvaddr) != KVM_GUEST_KSEG0) {
		kvm_err("%s: Invalid BadVaddr: %#lx\n", __func__, badvaddr);
		kvm_mips_dump_host_tlbs();
		return -1;
	}

	gfn = (KVM_GUEST_CPHYSADDR(badvaddr) >> PAGE_SHIFT);
	if (gfn >= kvm->arch.guest_pmap_npages) {
		kvm_err("%s: Invalid gfn: %#llx, BadVaddr: %#lx\n", __func__,
			gfn, badvaddr);
		kvm_mips_dump_host_tlbs();
		return -1;
	}
	even = !(gfn & 0x1);
	vaddr = badvaddr & (PAGE_MASK << 1);

	if (kvm_mips_map_page(vcpu->kvm, gfn) < 0)
		return -1;

	if (kvm_mips_map_page(vcpu->kvm, gfn ^ 0x1) < 0)
		return -1;

	if (even) {
		pfn0 = kvm->arch.guest_pmap[gfn];
		pfn1 = kvm->arch.guest_pmap[gfn ^ 0x1];
	} else {
		pfn0 = kvm->arch.guest_pmap[gfn ^ 0x1];
		pfn1 = kvm->arch.guest_pmap[gfn];
	}

	entryhi = (vaddr | kvm_mips_get_kernel_asid(vcpu));
	entrylo0 = mips3_paddr_to_tlbpfn(pfn0 << PAGE_SHIFT) | (0x3 << 3) | (1 << 2) |
			(0x1 << 1);
	entrylo1 = mips3_paddr_to_tlbpfn(pfn1 << PAGE_SHIFT) | (0x3 << 3) | (1 << 2) |
			(0x1 << 1);

	return kvm_mips_host_tlb_write(vcpu, entryhi, entrylo0, entrylo1,
				       flush_dcache_mask);
}

int kvm_mips_handle_commpage_tlb_fault(unsigned long badvaddr,
	struct kvm_vcpu *vcpu)
{
	pfn_t pfn0, pfn1;
	unsigned long flags, old_entryhi = 0, vaddr = 0;
	unsigned long entrylo0 = 0, entrylo1 = 0;


	pfn0 = CPHYSADDR(vcpu->arch.kseg0_commpage) >> PAGE_SHIFT;
	pfn1 = 0;
	entrylo0 = mips3_paddr_to_tlbpfn(pfn0 << PAGE_SHIFT) | (0x3 << 3) | (1 << 2) |
			(0x1 << 1);
	entrylo1 = 0;

	local_irq_save(flags);

	old_entryhi = read_c0_entryhi();
	vaddr = badvaddr & (PAGE_MASK << 1);
	write_c0_entryhi(vaddr | kvm_mips_get_kernel_asid(vcpu));
	mtc0_tlbw_hazard();
	write_c0_entrylo0(entrylo0);
	mtc0_tlbw_hazard();
	write_c0_entrylo1(entrylo1);
	mtc0_tlbw_hazard();
	write_c0_index(kvm_mips_get_commpage_asid(vcpu));
	mtc0_tlbw_hazard();
	tlb_write_indexed();
	mtc0_tlbw_hazard();
	tlbw_use_hazard();

#ifdef DEBUG
	kvm_debug ("@ %#lx idx: %2d [entryhi(R): %#lx] entrylo0 (R): 0x%08lx, entrylo1(R): 0x%08lx\n",
	     vcpu->arch.pc, read_c0_index(), read_c0_entryhi(),
	     read_c0_entrylo0(), read_c0_entrylo1());
#endif

	/* Restore old ASID */
	write_c0_entryhi(old_entryhi);
	mtc0_tlbw_hazard();
	tlbw_use_hazard();
	local_irq_restore(flags);

	return 0;
}

int
kvm_mips_handle_mapped_seg_tlb_fault(struct kvm_vcpu *vcpu,
	struct kvm_mips_tlb *tlb, unsigned long *hpa0, unsigned long *hpa1)
{
	unsigned long entryhi = 0, entrylo0 = 0, entrylo1 = 0;
	struct kvm *kvm = vcpu->kvm;
	pfn_t pfn0, pfn1;


	if ((tlb->tlb_hi & VPN2_MASK) == 0) {
		pfn0 = 0;
		pfn1 = 0;
	} else {
		if (kvm_mips_map_page(kvm, mips3_tlbpfn_to_paddr(tlb->tlb_lo0) >> PAGE_SHIFT) < 0)
			return -1;

		if (kvm_mips_map_page(kvm, mips3_tlbpfn_to_paddr(tlb->tlb_lo1) >> PAGE_SHIFT) < 0)
			return -1;

		pfn0 = kvm->arch.guest_pmap[mips3_tlbpfn_to_paddr(tlb->tlb_lo0) >> PAGE_SHIFT];
		pfn1 = kvm->arch.guest_pmap[mips3_tlbpfn_to_paddr(tlb->tlb_lo1) >> PAGE_SHIFT];
	}

	if (hpa0)
		*hpa0 = pfn0 << PAGE_SHIFT;

	if (hpa1)
		*hpa1 = pfn1 << PAGE_SHIFT;

	/* Get attributes from the Guest TLB */
	entryhi = (tlb->tlb_hi & VPN2_MASK) | (KVM_GUEST_KERNEL_MODE(vcpu) ?
			kvm_mips_get_kernel_asid(vcpu) : kvm_mips_get_user_asid(vcpu));
	entrylo0 = mips3_paddr_to_tlbpfn(pfn0 << PAGE_SHIFT) | (0x3 << 3) |
			(tlb->tlb_lo0 & MIPS3_PG_D) | (tlb->tlb_lo0 & MIPS3_PG_V);
	entrylo1 = mips3_paddr_to_tlbpfn(pfn1 << PAGE_SHIFT) | (0x3 << 3) |
			(tlb->tlb_lo1 & MIPS3_PG_D) | (tlb->tlb_lo1 & MIPS3_PG_V);

#ifdef DEBUG
	kvm_debug("@ %#lx tlb_lo0: 0x%08lx tlb_lo1: 0x%08lx\n", vcpu->arch.pc,
		  tlb->tlb_lo0, tlb->tlb_lo1);
#endif

	return kvm_mips_host_tlb_write(vcpu, entryhi, entrylo0, entrylo1,
				       tlb->tlb_mask);
}

int kvm_mips_guest_tlb_lookup(struct kvm_vcpu *vcpu, unsigned long entryhi)
{
	int i;
	int index = -1;
	struct kvm_mips_tlb *tlb = vcpu->arch.guest_tlb;


	for (i = 0; i < KVM_MIPS_GUEST_TLB_SIZE; i++) {
		if (((TLB_VPN2(tlb[i]) & ~tlb[i].tlb_mask) == ((entryhi & VPN2_MASK) & ~tlb[i].tlb_mask)) &&
			(TLB_IS_GLOBAL(tlb[i]) || (TLB_ASID(tlb[i]) == (entryhi & ASID_MASK)))) {
			index = i;
			break;
		}
	}

#ifdef DEBUG
	kvm_debug("%s: entryhi: %#lx, index: %d lo0: %#lx, lo1: %#lx\n",
		  __func__, entryhi, index, tlb[i].tlb_lo0, tlb[i].tlb_lo1);
#endif

	return index;
}

int kvm_mips_host_tlb_lookup(struct kvm_vcpu *vcpu, unsigned long vaddr)
{
	unsigned long old_entryhi, flags;
	volatile int idx;


	local_irq_save(flags);

	old_entryhi = read_c0_entryhi();

	if (KVM_GUEST_KERNEL_MODE(vcpu))
		write_c0_entryhi((vaddr & VPN2_MASK) | kvm_mips_get_kernel_asid(vcpu));
	else {
		write_c0_entryhi((vaddr & VPN2_MASK) | kvm_mips_get_user_asid(vcpu));
	}

	mtc0_tlbw_hazard();

	tlb_probe();
	tlb_probe_hazard();
	idx = read_c0_index();

	/* Restore old ASID */
	write_c0_entryhi(old_entryhi);
	mtc0_tlbw_hazard();
	tlbw_use_hazard();

	local_irq_restore(flags);

#ifdef DEBUG
	kvm_debug("Host TLB lookup, %#lx, idx: %2d\n", vaddr, idx);
#endif

	return idx;
}

int kvm_mips_host_tlb_inv(struct kvm_vcpu *vcpu, unsigned long va)
{
	int idx;
	unsigned long flags, old_entryhi;

	local_irq_save(flags);


	old_entryhi = read_c0_entryhi();

	write_c0_entryhi((va & VPN2_MASK) | kvm_mips_get_user_asid(vcpu));
	mtc0_tlbw_hazard();

	tlb_probe();
	tlb_probe_hazard();
	idx = read_c0_index();

	if (idx >= current_cpu_data.tlbsize)
		BUG();

	if (idx > 0) {
		write_c0_entryhi(UNIQUE_ENTRYHI(idx));
		mtc0_tlbw_hazard();

		write_c0_entrylo0(0);
		mtc0_tlbw_hazard();

		write_c0_entrylo1(0);
		mtc0_tlbw_hazard();

		tlb_write_indexed();
		mtc0_tlbw_hazard();
	}

	write_c0_entryhi(old_entryhi);
	mtc0_tlbw_hazard();
	tlbw_use_hazard();

	local_irq_restore(flags);

#ifdef DEBUG
	if (idx > 0) {
		kvm_debug("%s: Invalidated entryhi %#lx @ idx %d\n", __func__,
			  (va & VPN2_MASK) | (vcpu->arch.asid_map[va & ASID_MASK] & ASID_MASK), idx);
	}
#endif

	return 0;
}

/* XXXKYMA: Fix Guest USER/KERNEL no longer share the same ASID*/
int kvm_mips_host_tlb_inv_index(struct kvm_vcpu *vcpu, int index)
{
	unsigned long flags, old_entryhi;

	if (index >= current_cpu_data.tlbsize)
		BUG();

	local_irq_save(flags);


	old_entryhi = read_c0_entryhi();

	write_c0_entryhi(UNIQUE_ENTRYHI(index));
	mtc0_tlbw_hazard();

	write_c0_index(index);
	mtc0_tlbw_hazard();

	write_c0_entrylo0(0);
	mtc0_tlbw_hazard();

	write_c0_entrylo1(0);
	mtc0_tlbw_hazard();

	tlb_write_indexed();
	mtc0_tlbw_hazard();
	tlbw_use_hazard();

	write_c0_entryhi(old_entryhi);
	mtc0_tlbw_hazard();
	tlbw_use_hazard();

	local_irq_restore(flags);

	return 0;
}

void kvm_mips_flush_host_tlb(int skip_kseg0)
{
	unsigned long flags;
	unsigned long old_entryhi, entryhi;
	unsigned long old_pagemask;
	int entry = 0;
	int maxentry = current_cpu_data.tlbsize;


	local_irq_save(flags);

	old_entryhi = read_c0_entryhi();
	old_pagemask = read_c0_pagemask();

	/* Blast 'em all away. */
	for (entry = 0; entry < maxentry; entry++) {

		write_c0_index(entry);
		mtc0_tlbw_hazard();

		if (skip_kseg0) {
			tlb_read();
			tlbw_use_hazard();

			entryhi = read_c0_entryhi();

			/* Don't blow away guest kernel entries */
			if (KVM_GUEST_KSEGX(entryhi) == KVM_GUEST_KSEG0) {
				continue;
			}
		}

		/* Make sure all entries differ. */
		write_c0_entryhi(UNIQUE_ENTRYHI(entry));
		mtc0_tlbw_hazard();
		write_c0_entrylo0(0);
		mtc0_tlbw_hazard();
		write_c0_entrylo1(0);
		mtc0_tlbw_hazard();

		tlb_write_indexed();
		mtc0_tlbw_hazard();
	}

	tlbw_use_hazard();

	write_c0_entryhi(old_entryhi);
	write_c0_pagemask(old_pagemask);
	mtc0_tlbw_hazard();
	tlbw_use_hazard();

	local_irq_restore(flags);
}

void
kvm_get_new_mmu_context(struct mm_struct *mm, unsigned long cpu,
			struct kvm_vcpu *vcpu)
{
	unsigned long asid = asid_cache(cpu);

	if (!((asid += ASID_INC) & ASID_MASK)) {
		if (cpu_has_vtag_icache) {
			flush_icache_all();
		}

		kvm_local_flush_tlb_all();      /* start new asid cycle */

		if (!asid)      /* fix version if needed */
			asid = ASID_FIRST_VERSION;
	}

	cpu_context(cpu, mm) = asid_cache(cpu) = asid;
}

void kvm_shadow_tlb_put(struct kvm_vcpu *vcpu)
{
	unsigned long flags;
	unsigned long old_entryhi;
	unsigned long old_pagemask;
	int entry = 0;
	int cpu = smp_processor_id();

	local_irq_save(flags);

	old_entryhi = read_c0_entryhi();
	old_pagemask = read_c0_pagemask();

	for (entry = 0; entry < current_cpu_data.tlbsize; entry++) {
		write_c0_index(entry);
		mtc0_tlbw_hazard();
		tlb_read();
		tlbw_use_hazard();

		vcpu->arch.shadow_tlb[cpu][entry].tlb_hi = read_c0_entryhi();
		vcpu->arch.shadow_tlb[cpu][entry].tlb_lo0 = read_c0_entrylo0();
		vcpu->arch.shadow_tlb[cpu][entry].tlb_lo1 = read_c0_entrylo1();
		vcpu->arch.shadow_tlb[cpu][entry].tlb_mask = read_c0_pagemask();
	}

	write_c0_entryhi(old_entryhi);
	write_c0_pagemask(old_pagemask);
	mtc0_tlbw_hazard();

	local_irq_restore(flags);

}

void kvm_shadow_tlb_load(struct kvm_vcpu *vcpu)
{
	unsigned long flags;
	unsigned long old_ctx;
	int entry;
	int cpu = smp_processor_id();

	local_irq_save(flags);

	old_ctx = read_c0_entryhi();

	for (entry = 0; entry < current_cpu_data.tlbsize; entry++) {
		write_c0_entryhi(vcpu->arch.shadow_tlb[cpu][entry].tlb_hi);
		mtc0_tlbw_hazard();
		write_c0_entrylo0(vcpu->arch.shadow_tlb[cpu][entry].tlb_lo0);
		write_c0_entrylo1(vcpu->arch.shadow_tlb[cpu][entry].tlb_lo1);

		write_c0_index(entry);
		mtc0_tlbw_hazard();

		tlb_write_indexed();
		tlbw_use_hazard();
	}

	tlbw_use_hazard();
	write_c0_entryhi(old_ctx);
	mtc0_tlbw_hazard();
	local_irq_restore(flags);
}


void kvm_local_flush_tlb_all(void)
{
	unsigned long flags;
	unsigned long old_ctx;
	int entry = 0;

	local_irq_save(flags);
	/* Save old context and create impossible VPN2 value */
	old_ctx = read_c0_entryhi();
	write_c0_entrylo0(0);
	write_c0_entrylo1(0);

	/* Blast 'em all away. */
	while (entry < current_cpu_data.tlbsize) {
		/* Make sure all entries differ. */
		write_c0_entryhi(UNIQUE_ENTRYHI(entry));
		write_c0_index(entry);
		mtc0_tlbw_hazard();
		tlb_write_indexed();
		entry++;
	}
	tlbw_use_hazard();
	write_c0_entryhi(old_ctx);
	mtc0_tlbw_hazard();

	local_irq_restore(flags);
}

void kvm_mips_init_shadow_tlb(struct kvm_vcpu *vcpu)
{
	int cpu, entry;

	for_each_possible_cpu(cpu) {
		for (entry = 0; entry < current_cpu_data.tlbsize; entry++) {
			vcpu->arch.shadow_tlb[cpu][entry].tlb_hi =
			    UNIQUE_ENTRYHI(entry);
			vcpu->arch.shadow_tlb[cpu][entry].tlb_lo0 = 0x0;
			vcpu->arch.shadow_tlb[cpu][entry].tlb_lo1 = 0x0;
			vcpu->arch.shadow_tlb[cpu][entry].tlb_mask =
			    read_c0_pagemask();
#ifdef DEBUG
			kvm_debug
			    ("shadow_tlb[%d][%d]: tlb_hi: %#lx, lo0: %#lx, lo1: %#lx\n",
			     cpu, entry,
			     vcpu->arch.shadow_tlb[cpu][entry].tlb_hi,
			     vcpu->arch.shadow_tlb[cpu][entry].tlb_lo0,
			     vcpu->arch.shadow_tlb[cpu][entry].tlb_lo1);
#endif
		}
	}
}

/* Restore ASID once we are scheduled back after preemption */
void kvm_arch_vcpu_load(struct kvm_vcpu *vcpu, int cpu)
{
	unsigned long flags;
	int newasid = 0;

#ifdef DEBUG
	kvm_debug("%s: vcpu %p, cpu: %d\n", __func__, vcpu, cpu);
#endif

	/* Alocate new kernel and user ASIDs if needed */

	local_irq_save(flags);

	if (((vcpu->arch.
	      guest_kernel_asid[cpu] ^ asid_cache(cpu)) & ASID_VERSION_MASK)) {
		kvm_get_new_mmu_context(&vcpu->arch.guest_kernel_mm, cpu, vcpu);
		vcpu->arch.guest_kernel_asid[cpu] =
		    vcpu->arch.guest_kernel_mm.context.asid[cpu];
		kvm_get_new_mmu_context(&vcpu->arch.guest_user_mm, cpu, vcpu);
		vcpu->arch.guest_user_asid[cpu] =
		    vcpu->arch.guest_user_mm.context.asid[cpu];
		newasid++;

		kvm_info("[%d]: cpu_context: %#lx\n", cpu,
			 cpu_context(cpu, current->mm));
		kvm_info("[%d]: Allocated new ASID for Guest Kernel: %#x\n",
			 cpu, vcpu->arch.guest_kernel_asid[cpu]);
		kvm_info("[%d]: Allocated new ASID for Guest User: %#x\n", cpu,
			 vcpu->arch.guest_user_asid[cpu]);
	}

	if (vcpu->arch.last_sched_cpu != cpu) {
		kvm_info("[%d->%d]KVM VCPU[%d] switch\n",
			 vcpu->arch.last_sched_cpu, cpu, vcpu->vcpu_id);
	}

	/* Only reload shadow host TLB if new ASIDs haven't been allocated */
#if 0
	if ((atomic_read(&kvm_mips_instance) > 1) && !newasid) {
		kvm_mips_flush_host_tlb(0);
		kvm_shadow_tlb_load(vcpu);
	}
#endif

	if (!newasid) {
		/* If we preempted while the guest was executing, then reload the pre-empted ASID */
		if (current->flags & PF_VCPU) {
			write_c0_entryhi(vcpu->arch.
					 preempt_entryhi & ASID_MASK);
			ehb();
		}
	} else {
		/* New ASIDs were allocated for the VM */

		/* Were we in guest context? If so then the pre-empted ASID is no longer
		 * valid, we need to set it to what it should be based on the mode of
		 * the Guest (Kernel/User)
		 */
		if (current->flags & PF_VCPU) {
			if (KVM_GUEST_KERNEL_MODE(vcpu))
				write_c0_entryhi(vcpu->arch.
						 guest_kernel_asid[cpu] &
						 ASID_MASK);
			else
				write_c0_entryhi(vcpu->arch.
						 guest_user_asid[cpu] &
						 ASID_MASK);
			ehb();
		}
	}

	local_irq_restore(flags);

}

/* ASID can change if another task is scheduled during preemption */
void kvm_arch_vcpu_put(struct kvm_vcpu *vcpu)
{
	unsigned long flags;
	uint32_t cpu;

	local_irq_save(flags);

	cpu = smp_processor_id();


	vcpu->arch.preempt_entryhi = read_c0_entryhi();
	vcpu->arch.last_sched_cpu = cpu;

#if 0
	if ((atomic_read(&kvm_mips_instance) > 1)) {
		kvm_shadow_tlb_put(vcpu);
	}
#endif

	if (((cpu_context(cpu, current->mm) ^ asid_cache(cpu)) &
	     ASID_VERSION_MASK)) {
		kvm_debug("%s: Dropping MMU Context:  %#lx\n", __func__,
			  cpu_context(cpu, current->mm));
		drop_mmu_context(current->mm, cpu);
	}
	write_c0_entryhi(cpu_asid(cpu, current->mm));
	ehb();

	local_irq_restore(flags);
}

uint32_t kvm_get_inst(uint32_t *opc, struct kvm_vcpu *vcpu)
{
	struct mips_coproc *cop0 = vcpu->arch.cop0;
	unsigned long paddr, flags;
	uint32_t inst;
	int index;

	if (KVM_GUEST_KSEGX((unsigned long) opc) < KVM_GUEST_KSEG0 ||
	    KVM_GUEST_KSEGX((unsigned long) opc) == KVM_GUEST_KSEG23) {
		local_irq_save(flags);
		index = kvm_mips_host_tlb_lookup(vcpu, (unsigned long) opc);
		if (index >= 0) {
			inst = *(opc);
		} else {
			index =
			    kvm_mips_guest_tlb_lookup(vcpu,
						      ((unsigned long) opc & VPN2_MASK)
						      |
						      (kvm_read_c0_guest_entryhi
						       (cop0) & ASID_MASK));
			if (index < 0) {
				kvm_err
				    ("%s: get_user_failed for %p, vcpu: %p, ASID: %#lx\n",
				     __func__, opc, vcpu, read_c0_entryhi());
				kvm_mips_dump_host_tlbs();
				local_irq_restore(flags);
				return KVM_INVALID_INST;
			}
			kvm_mips_handle_mapped_seg_tlb_fault(vcpu,
							     &vcpu->arch.
							     guest_tlb[index],
							     NULL, NULL);
			inst = *(opc);
		}
		local_irq_restore(flags);
	} else if (KVM_GUEST_KSEGX(opc) == KVM_GUEST_KSEG0) {
		paddr =
		    kvm_mips_translate_guest_kseg0_to_hpa(vcpu,
							 (unsigned long) opc);
		inst = *(uint32_t *) CKSEG0ADDR(paddr);
	} else {
		kvm_err("%s: illegal address: %p\n", __func__, opc);
		return KVM_INVALID_INST;
	}

	return inst;
}

EXPORT_SYMBOL(kvm_local_flush_tlb_all);
EXPORT_SYMBOL(kvm_shadow_tlb_put);
EXPORT_SYMBOL(kvm_mips_handle_mapped_seg_tlb_fault);
EXPORT_SYMBOL(kvm_mips_handle_commpage_tlb_fault);
EXPORT_SYMBOL(kvm_mips_init_shadow_tlb);
EXPORT_SYMBOL(kvm_mips_dump_host_tlbs);
EXPORT_SYMBOL(kvm_mips_handle_kseg0_tlb_fault);
EXPORT_SYMBOL(kvm_mips_host_tlb_lookup);
EXPORT_SYMBOL(kvm_mips_flush_host_tlb);
EXPORT_SYMBOL(kvm_mips_guest_tlb_lookup);
EXPORT_SYMBOL(kvm_mips_host_tlb_inv);
EXPORT_SYMBOL(kvm_mips_translate_guest_kseg0_to_hpa);
EXPORT_SYMBOL(kvm_shadow_tlb_load);
EXPORT_SYMBOL(kvm_mips_dump_shadow_tlbs);
EXPORT_SYMBOL(kvm_mips_dump_guest_tlbs);
EXPORT_SYMBOL(kvm_get_inst);
EXPORT_SYMBOL(kvm_arch_vcpu_load);
EXPORT_SYMBOL(kvm_arch_vcpu_put);
#else
#include <linux/kvm_host.h>
#include <linux/highmem.h>
#include <linux/slab.h>

#include <asm/r4kcache.h>
#include <asm/ptrace.h>
#include <asm/kvm_loongson.h>
#include <asm/kvm_mips.h>
#include <asm/kvm_asm.h>
#include <linux/mm.h>

#include "loongson_tlb.h"
#include "statistic.h"

#define FLUSH_ITLB write_c0_diag(0x104)
#define KERNEL_TLB_SIZE 32

#ifdef CONFIG_KERNEL_PAGE_SIZE_4KB
#define KERNEL_PAGE_SHIFT	12
#endif
#ifdef CONFIG_KERNEL_PAGE_SIZE_8KB
#define KERNEL_PAGE_SHIFT	13
#endif
#ifdef CONFIG_KERNEL_PAGE_SIZE_16KB
#define KERNEL_PAGE_SHIFT	14
#endif
#ifdef CONFIG_KERNEL_PAGE_SIZE_32KB
#define KERNEL_PAGE_SHIFT	15
#endif
#ifdef CONFIG_KERNEL_PAGE_SIZE_64KB
#define KERNEL_PAGE_SHIFT	16
#endif
#ifdef CONFIG_KERNEL_PAGE_SIZE_16M
#define KERNEL_PAGE_SHIFT	24
#endif
#define KERNEL_PAGE_SIZE	(_AC(1,UL) << KERNEL_PAGE_SHIFT)
#define KERNEL_PAGE_MASK	(~((1 << KERNEL_PAGE_SHIFT) - 1))

extern void local_flush_tlb_all(void);

u32 kvmmips_get_kernel_instruction(struct kvm_run* run_ptr, struct kvm_vcpu* vcpu_ptr)
{
	struct page* new_page;
	u64 basic_addr = 0;
	u64 epc_gfn;
	u64 epc = vcpu_ptr->arch.temp_cp0_cause & CAUSEF_BD ? vcpu_ptr->arch.temp_cp0_epc + 4 :
					vcpu_ptr->arch.temp_cp0_epc;

#ifdef CONFIG_KVM64_SUPPORT
	if ((epc & 0xfffffffff0000000) != 0x4000000080000000) {
#else
	if ((epc & 0xfffffffff0000000) != 0xffffffffc0000000) {
#endif
		//the instruction should be in guest os' kernel mode
		return 0;
	}

#ifdef CONFIG_KVM64_SUPPORT
	epc = epc & ~0xffffffffe0000000;
#else
	epc = epc & ~0xffffffffc0000000;
#endif
	epc_gfn = epc >> PAGE_SHIFT;
	//TODO: it is 0x9000000008000000 because we map  c0000000 ~ c8000000 to 08000000 ~ 10000000
	if (kvm_is_visible_gfn(vcpu_ptr->kvm,epc_gfn))
	{
		new_page = gfn_to_page(vcpu_ptr->kvm,epc_gfn);
		if (is_error_page(new_page))
		{
			printk(KERN_ERR "kvmmips: Couldn't get guest page for gfn %lx!\n", (unsigned long)0x0);
			kvm_release_page_clean(new_page);
			return 0;
		}
		basic_addr = page_to_phys(new_page);

		/* release page, for it will not use */
		kvm_release_page_clean(new_page);
	}

	basic_addr |= 0x9800000000000000;
	epc = (epc & ((1 << PAGE_SHIFT) -1)) | basic_addr;

	return *(volatile u32*)epc;
}

/* TODO: it supports that I/O in qemu store value in little endian */
int kvmmips_handle_load(struct kvm_run *run_ptr, struct kvm_vcpu *vcpu_ptr,
			unsigned int rt, unsigned int bytes, u64 vaddr, int is_sign_extend)
{
	if (bytes > sizeof(run_ptr->mmio.data)) {
		printk(KERN_ERR "%s: bad MMIO length: %d\n", __func__,
			run_ptr->mmio.len);
	}

	run_ptr->mmio.phys_addr = vaddr;
	run_ptr->mmio.len = bytes;
	run_ptr->mmio.is_write = 0;
	vcpu_ptr->mmio_needed = 1;
	vcpu_ptr->mmio_is_write = 0;
	vcpu_ptr->arch.io_gpr = rt;
	vcpu_ptr->arch.mmio_sign_extend = is_sign_extend;

	return EMULATE_DO_MMIO;
}

void kvmmips_complete_mmio_load(struct kvm_run *run_ptr, struct kvm_vcpu *vcpu_ptr)
{
	u64 gpr = 0;

	if (run_ptr->mmio.len > sizeof(gpr)) {
		printk(KERN_ERR "bad MMIO length: %d\n", run_ptr->mmio.len);
		return;
	}

	switch (run_ptr->mmio.len) {
	case 8:
		gpr = *(u64 *)run_ptr->mmio.data;
		break;
	case 4:
		gpr = *(u32 *)run_ptr->mmio.data;
		break;
	case 2:
		gpr = *(u16 *)run_ptr->mmio.data;
		break;
	case 1:
		gpr = *(u8 *)run_ptr->mmio.data;
		break;
	}

	if (vcpu_ptr->arch.mmio_sign_extend) {
		switch (run_ptr->mmio.len) {
		case 4:
			gpr = (s64)(s32)gpr;
			break;
		case 2:
			gpr = (s64)(s16)gpr;
			break;
		case 1:
			gpr = (s64)(s8)gpr;
			break;
		}
	}

	/* TODO: now not consider fpr and other */
	vcpu_ptr->arch.gpr[vcpu_ptr->arch.io_gpr % 32] = gpr;
}

int kvmmips_handle_store(struct kvm_run *run_ptr, struct kvm_vcpu *vcpu_ptr,
			unsigned int rt, unsigned int bytes, u64 vaddr)
{
	void *data = run_ptr->mmio.data;
	u64   val  = vcpu_ptr->arch.gpr[rt];
	if (bytes > sizeof(run_ptr->mmio.data)) {
		printk(KERN_ERR "%s: bad MMIO length: %d\n", __func__,
			run_ptr->mmio.len);
	}

	run_ptr->mmio.phys_addr = vaddr;
	run_ptr->mmio.len = bytes;
	run_ptr->mmio.is_write = 1;
	vcpu_ptr->mmio_needed = 1;
	vcpu_ptr->mmio_is_write = 1;

	switch (bytes) {
		case 8: *(u64 *)data = val; break;
		case 4: *(u32 *)data = val; break;
		case 2: *(u16 *)data = val; break;
		case 1: *(u8  *)data = val; break;
	}

	return EMULATE_DO_MMIO;
}

int kvmmips_emulate_mmio(struct kvm_run* run_ptr, struct kvm_vcpu* vcpu_ptr, u64 vaddr)
{
	u32 opcode = kvmmips_get_kernel_instruction(run_ptr, vcpu_ptr);

	/* TODO: vaddr use or not should be considered */
	switch ((opcode & OPCODE1) >> OPCODE1_OFFSET) {
	case LB:
		kvmmips_handle_load(run_ptr, vcpu_ptr, (opcode & OP1) >> OP1_OFFSET, 1, vaddr, 1);
		break;
	case LBU:
		kvmmips_handle_load(run_ptr, vcpu_ptr, (opcode & OP1) >> OP1_OFFSET, 1, vaddr, 0);
		break;
	case LH:
		kvmmips_handle_load(run_ptr, vcpu_ptr, (opcode & OP1) >> OP1_OFFSET, 2, vaddr, 1);
		break;
	case LHU:
		kvmmips_handle_load(run_ptr, vcpu_ptr, (opcode & OP1) >> OP1_OFFSET, 2, vaddr, 0);
		break;
	case LW:
	case LWL:
	case LL:
		/* TODO: LL should be considered more */
		kvmmips_handle_load(run_ptr, vcpu_ptr, (opcode & OP1) >> OP1_OFFSET, 4, vaddr, 1);
		break;
	case LWR:
		/* TODO:suppose that address in badvaddr is real address and LWL,
		 * LWR shall use together, LWR do nothing */
		break;
	case LWU:
		kvmmips_handle_load(run_ptr, vcpu_ptr, (opcode & OP1) >> OP1_OFFSET, 4, vaddr, 0);
		break;
	case LD:
	case LLD:
	case LDL:
		kvmmips_handle_load(run_ptr, vcpu_ptr, (opcode & OP1) >> OP1_OFFSET, 8, vaddr, 1);
		break;
	case LDR:
		/* TODO:suppose that address in badvaddr is real address and LWL,
		 * LWR shall use together, LWR do nothing */
		break;
	case SB:
		kvmmips_handle_store(run_ptr, vcpu_ptr, (opcode & OP1) >> OP1_OFFSET, 1, vaddr);
		break;
	case SH:
		kvmmips_handle_store(run_ptr, vcpu_ptr, (opcode & OP1) >> OP1_OFFSET, 2, vaddr);
		break;
	case SW:
	case SWL:
	case SC:
		/* TODO: SC should be considered more */
		kvmmips_handle_store(run_ptr, vcpu_ptr, (opcode & OP1) >> OP1_OFFSET, 4, vaddr);
		break;
	case SWR:
		/* TODO:suppose that address in badvaddr is real address and LWL,
		 * LWR shall use together, LWR do nothing */
		break;
	case SD:
	case SDL:
	case SCD:
		/* TODO: SC should be considered more */
		kvmmips_handle_store(run_ptr, vcpu_ptr, (opcode & OP1) >> OP1_OFFSET, 8, vaddr);
		break;
	case SDR:
		/* TODO:suppose that address in badvaddr is real address and LWL,
		 * LWR shall use together, LWR do nothing */
		break;
	default:
		break;
	}

	run_ptr->exit_reason = KVM_EXIT_MMIO;
	return 0;
}

int kvmmips_emulate_mm(struct kvm_run* run_ptr, struct kvm_vcpu* vcpu_ptr, u64 vaddr)
{
	u32 opcode = kvmmips_get_kernel_instruction(run_ptr, vcpu_ptr);

	/* TODO: vaddr use or not should be considered */
	switch ((opcode & OPCODE1) >> OPCODE1_OFFSET) {
	case LB:
		vcpu_ptr->arch.gpr[(opcode & OP1) >> OP1_OFFSET] = (s64)(s8)(*(u8*)vaddr);
		break;
	case LBU:
		kvmmips_handle_load(run_ptr, vcpu_ptr, (opcode & OP1) >> OP1_OFFSET, 1, vaddr, 0);
		vcpu_ptr->arch.gpr[(opcode & OP1) >> OP1_OFFSET] = (*(u8*)vaddr);
		break;
	case LH:
		vcpu_ptr->arch.gpr[(opcode & OP1) >> OP1_OFFSET] = (s64)(s16)(*(u16*)vaddr);
		break;
	case LHU:
		kvmmips_handle_load(run_ptr, vcpu_ptr, (opcode & OP1) >> OP1_OFFSET, 2, vaddr, 0);
		vcpu_ptr->arch.gpr[(opcode & OP1) >> OP1_OFFSET] = (*(u16*)vaddr);
		break;
	case LW:
	case LWL:
	case LL:
		/* TODO: LL should be considered more */
		vcpu_ptr->arch.gpr[(opcode & OP1) >> OP1_OFFSET] = (s64)(s32)(*(u32*)vaddr);
		break;
	case LWR:
		/* TODO:suppose that address in badvaddr is real address and LWL,
		 * LWR shall use together,LWR do nothing */
		break;
	case LWU:
		vcpu_ptr->arch.gpr[(opcode & OP1) >> OP1_OFFSET] = (*(u32*)vaddr);
		break;
	case LD:
	case LLD:
	case LDL:
		vcpu_ptr->arch.gpr[(opcode & OP1) >> OP1_OFFSET] = (*(u64*)vaddr);
		break;
	case LDR:
		/* TODO:suppose that address in badvaddr is real address and LWL,
		 * LWR shall use together, LWR do nothing */
		break;
	case SB:
		*(u8*)vaddr = vcpu_ptr->arch.gpr[(opcode & OP1) >> OP1_OFFSET];
		break;
	case SH:
		*(u16*)vaddr = vcpu_ptr->arch.gpr[(opcode & OP1) >> OP1_OFFSET];
		break;
	case SW:
	case SWL:
	case SC:
		/* TODO: SC should be considered more*/
		*(u32*)vaddr = vcpu_ptr->arch.gpr[(opcode & OP1) >> OP1_OFFSET];
		break;
	case SWR:
		/* TODO:suppose that address in badvaddr is real address and LWL,
		 * LWR shall use together, LWR do nothing */
		break;
	case SD:
	case SDL:
	case SCD:
		/* TODO: SC should be considered more */
		*(u64*)vaddr = vcpu_ptr->arch.gpr[(opcode & OP1) >> OP1_OFFSET];
		break;
	case SDR:
		/* TODO: suppose that address in badvaddr is real address and LWL,
		 * LWR shall use together, LWR do nothing */
		break;
	default:
		break;
	}

	run_ptr->exit_reason = KVM_EXIT_MMIO;
	return 0;
}

extern void kvmmips_emulate_branch(struct kvm_run* run_ptr, struct kvm_vcpu* vcpu_ptr);
asmlinkage int do_kvm_tlbmiss(struct kvm_run* run_ptr , struct kvm_vcpu* vcpu_ptr)
{
	u64 vaddr = vcpu_ptr->arch.temp_cp0_badvaddr;

	printk("[KVM-ERROR]:%s %d: vaddr(0x%lx)\n", __func__, __LINE__, (unsigned long)vaddr);
	while(1);

	return RETURN_TO_GUEST;
}

int kvmmips_handle_pte_fault(struct kvmmips_vcpu_loongson* vcpu_ptr, pte_t *pte, u64 vaddr, int mls)
{
	u64 gpaddr[2], gfn[2];
	struct page *new_page[2];
	pgprot_t page_prot;
	pte_t entry0, entry1;
	int i = 0;

	u64 temp = vaddr & (PAGE_MASK << 1);

	if (((vaddr & 0xffffffff00000000) == 0x4000000100000000) || ((vaddr & 0xffffffff00000000) == 0x4000000200000000)) {
		gpaddr[0] = temp & ~(u64)0xffffffff00000000;
		gpaddr[1] = (temp & ~(u64)0xffffffff00000000) | (0x1 << PAGE_SHIFT);
	} else {
		gpaddr[0] = temp & ~(u64)0xffffffffe0000000;
		gpaddr[1] = (temp & ~(u64)0xffffffffe0000000) | (0x1 << PAGE_SHIFT);
	}

	gfn[0] = gpaddr[0] >> PAGE_SHIFT;
	gfn[1] = gpaddr[1] >> PAGE_SHIFT;

	for (i = 0; i < 2; i++)
	{
		if (kvm_is_visible_gfn(vcpu_ptr->vcpu.kvm, gfn[i]))
		{
			new_page[i] = gfn_to_page(vcpu_ptr->vcpu.kvm, gfn[i]);
			if (is_error_page(new_page[i]))
			{
				printk(KERN_ERR "kvmmips: Couldn't get guest page for gfn %lx!\n", (unsigned long)gfn[i]);
				kvm_release_page_clean(new_page[i]);
				return -1;
			}

			/* release the count of page*/
			kvm_release_page_clean(new_page[i]);
		}
	}

	page_prot.pgprot = 0x3ff; //for kernel G=1
	entry0 = mk_pte(new_page[0], page_prot);
	set_pte(pte, entry0);
	entry1 = mk_pte(new_page[1], page_prot);
	set_pte(pte + 1, entry1);

#ifndef CONFIG_KVM_MULTICLIENT
	kvmmips_update_mmu_cache(vaddr, pte);
#else
	for (i = 0;i < vcpu_ptr->shadow_tlb_size;i++) {
		if ((vcpu_ptr->shadow_tlb[i].mask0 & ~((0x1 << (PAGE_SHIFT + 1)) - 1))
			== (vcpu_ptr->vcpu.arch.temp_cp0_entryhi & ~((0x1 << (PAGE_SHIFT + 1)) - 1))) {
			break;
		}
	}

	/* save to shadow tlb */
	if (i >= vcpu_ptr->shadow_tlb_size) {
		printk("ERROR!!!! in %s cannot find entryhi:%llx, vaddr:%llx\n",
			__func__, vcpu_ptr->vcpu.arch.temp_cp0_entryhi, vaddr);
		for (i = 0; i < vcpu_ptr->shadow_tlb_size; i++) {
			printk("%llx  ", vcpu_ptr->shadow_tlb[i].mask0);
			if (i % 8 == 7)
				printk("\n");
		}
		return 1;
	}
#if defined(CONFIG_64BIT_PHYS_ADDR) && defined(CONFIG_CPU_MIPS32)
	vcpu_ptr->shadow_tlb[i].mask2 = pte->pte_high;
	pte++;
	vcpu_ptr->shadow_tlb[i].mask3 = pte->pte_high;
#else
	vcpu_ptr->shadow_tlb[i].mask2 = pte_to_entrylo(pte_val(*pte++));
	vcpu_ptr->shadow_tlb[i].mask3 = pte_to_entrylo(pte_val(*pte));
#endif
#endif
	return 1;
}

void kvm_print_tlb(void);

void kvmmips_do_page_fault(struct kvm_run* run_ptr, struct kvm_vcpu* vcpu_ptr, int mls)
{
	u64 vaddr = vcpu_ptr->arch.temp_cp0_entryhi & (PAGE_MASK << 1);
	pgd_t *pgd;
	pmd_t *pmd;
	pte_t *pte;

	run_ptr->exit_reason = KVM_EXIT_UNKNOWN;
	run_ptr->ready_for_interrupt_injection = 1;

	pgd = kvmmips_pgd_offset(to_loongson(vcpu_ptr)->kvm_pg_dir, vaddr);
	pmd = (pmd_t*)pgd;
	pte = kvmmips_pte_alloc_map(NULL, pmd, vaddr);

	kvmmips_handle_pte_fault(to_loongson(vcpu_ptr), pte, vaddr, mls);
}

int kvmmips_handle_kernel_address(struct kvm_run* run_ptr, struct kvm_vcpu* vcpu_ptr, int mls)
{
	u64 vaddr = vcpu_ptr->arch.temp_cp0_entryhi;

	run_ptr->exit_reason = KVM_EXIT_UNKNOWN;
	run_ptr->ready_for_interrupt_injection = 1;

	if (((vaddr & 0xfffffffff0000000) == 0x4000000080000000)
		|| ((vaddr & 0xfffffffff0000000) == 0x40000000a0000000)
		|| ((vaddr & 0xfffffffffff00000) == 0x40000000bfc00000)
		|| ((vaddr & 0xffffffff00000000) == 0x4000000100000000)
		|| ((vaddr & 0xffffffff00000000) == 0x4000000200000000)) {
#ifndef CONFIG_KVM_MULTICLIENT
		u64 temp_entryhi = read_c0_entryhi();
		write_c0_entryhi(vcpu_ptr->arch.temp_cp0_entryhi); //restore entryhi
#endif
		kvmmips_do_page_fault(run_ptr, vcpu_ptr, mls);
#ifndef CONFIG_KVM_MULTICLIENT
		write_c0_entryhi(temp_entryhi);
#endif
		return 1; //is kernel address
	} else {
#ifdef KVM_MM_DEBUG
		if(vaddr >= 0x4000000400000000)
			printk("\n!!!%s %d: epc(0x%lx) cause(0x%lx) vaddr(0x%lx) mls(%d)\n", __func__, __LINE__,
				vcpu_ptr->arch.temp_cp0_epc, vcpu_ptr->arch.temp_cp0_cause, vaddr, mls);
		else {
			printk("\n!!!%s %d: epc(0x%lx) cause(0x%lx) vaddr(0x%lx) mls(%d)\n", __func__, __LINE__,
				vcpu_ptr->arch.temp_cp0_epc, vcpu_ptr->arch.temp_cp0_cause, vaddr, mls);
		}
#endif
		return 0;
	}
}

int kvmmips_check_pte_rwm(struct kvm_run* run_ptr, struct kvm_vcpu* vcpu_ptr, u8 check, u8 new)
{
	u64 vaddr = vcpu_ptr->arch.temp_cp0_entryhi;
	pgd_t *temp_pgdp;
	pgd_t *pgd;
	pmd_t *pmd;
	pte_t *pte;
	unsigned int check_result;

	run_ptr->exit_reason = KVM_EXIT_UNKNOWN;
	run_ptr->ready_for_interrupt_injection = 1;

	if(vaddr >= 0x4000000400000000ULL)
		temp_pgdp = to_loongson(vcpu_ptr)->swapper_pg_dir;
	else if(vaddr < 0x4000000000000000ULL)
		temp_pgdp = (pgd_t *)to_loongson(vcpu_ptr)->pgd_current;
	else {
		printk("===[ERROR]: %s %s %d\n", __FILE__, __func__, __LINE__);
		while(1);
	}

	pgd = kvmmips_pgd_offset(temp_pgdp, vaddr);
	pmd = (pmd_t*)pgd;
	pte = kvmmips_pte_alloc_map(NULL, pmd, vaddr);

	check_result = (pte_val(*pte) & check) ^ check;

	if(!check_result) {
		u64 entrylo0, entrylo1;

		pte_val(*pte) |= new;
		pte = (pte_t *)(((unsigned long)pte | 0x8) ^ 0x8);

		entrylo0 = pte_val(*pte++) >> ilog2(_PAGE_GLOBAL);
		entrylo1 = pte_val(*pte) >> ilog2(_PAGE_GLOBAL);

#ifndef CONFIG_KVM_MULTICLIENT
		/* cannot fit for the multi clients */
	{
		int idx, temp_idx;
		u64 temp_entryhi;
		temp_entryhi = read_c0_entryhi();
		temp_idx = read_c0_index();

		write_c0_entryhi(vcpu_ptr->arch.temp_cp0_entryhi);
		mtc0_tlbw_hazard();
		tlb_probe();
		tlb_probe_hazard();

		idx = read_c0_index();
		write_c0_entrylo0(entrylo0);
		write_c0_entrylo1(entrylo1);
		mtc0_tlbw_hazard();

		if (idx < 0)
			tlb_write_random();
		else
			tlb_write_indexed();
		tlbw_use_hazard();
		FLUSH_ITLB;

		write_c0_entryhi(temp_entryhi);
		write_c0_index(temp_idx);
	}
#else
		/* save to shadow tlb */
	{
		int i;
		for (i = 0;i < to_loongson(vcpu_ptr)->shadow_tlb_size;i++) {
			if (((to_loongson(vcpu_ptr)->shadow_tlb[i].mask0 & ~((0x1 << (PAGE_SHIFT + 1)) - 1))
				== (to_loongson(vcpu_ptr)->vcpu.arch.temp_cp0_entryhi & ~((0x1 << (PAGE_SHIFT + 1)) - 1)))
				&& ((to_loongson(vcpu_ptr)->shadow_tlb[i].mask0 & 0xff)
					== (to_loongson(vcpu_ptr)->vcpu.arch.temp_cp0_entryhi & 0xff))) {
				break;
			}
		}

		/* save to shadow tlb */
		if (i >= to_loongson(vcpu_ptr)->shadow_tlb_size) {
			printk("ERROR!!!! in %s cannot find entryhi:%llx, vaddr:%llx\n",
				__func__, to_loongson(vcpu_ptr)->vcpu.arch.temp_cp0_entryhi, vaddr);
			for (i = 0; i < to_loongson(vcpu_ptr)->shadow_tlb_size; i++) {
				printk("%llx  ", to_loongson(vcpu_ptr)->shadow_tlb[i].mask0);
				if (i % 8 == 7)
					printk("\n");
			}
			return 1;
		}
		to_loongson(vcpu_ptr)->shadow_tlb[i].mask0 = vcpu_ptr->arch.temp_cp0_entryhi;
		to_loongson(vcpu_ptr)->shadow_tlb[i].mask1 = read_c0_pagemask();
		to_loongson(vcpu_ptr)->shadow_tlb[i].mask2 = entrylo0;
		to_loongson(vcpu_ptr)->shadow_tlb[i].mask3 = entrylo1;
	}
#endif
		return 1;
	}

	return 0;
}

asmlinkage int do_kvm_mod(struct kvm_run* run_ptr, struct kvm_vcpu* vcpu_ptr)
{
	if (kvmmips_handle_kernel_address(run_ptr, vcpu_ptr, 0))
		return RETURN_TO_GUEST;

	/* account info */
	kvmmips_stat_exits(vcpu_ptr, MOD_EXITS);

	/* TODO: inject to guest os*/
	if (kvmmips_check_pte_rwm(run_ptr, vcpu_ptr, 0x4, 0xd8))
		return RETURN_TO_GUEST;

	kvmmips_queue_exception(vcpu_ptr, KVMMIPS_EXCEPTION_MOD);
	return RETURN_TO_GUEST;
}

asmlinkage int do_kvm_tlbl(struct kvm_run* run_ptr, struct kvm_vcpu* vcpu_ptr)
{
	if (kvmmips_handle_kernel_address(run_ptr, vcpu_ptr, 1))
		return RETURN_TO_GUEST;

	/* account info */
	kvmmips_stat_exits(vcpu_ptr, TLBL_EXITS);

	/* TODO: inject to guest OS */
	if (kvmmips_check_pte_rwm(run_ptr, vcpu_ptr, 0x3, 0x48))
		return RETURN_TO_GUEST;

	kvmmips_queue_exception(vcpu_ptr, KVMMIPS_EXCEPTION_TLBL);

	return RETURN_TO_GUEST;
}

asmlinkage int do_kvm_tlbs(struct kvm_run* run_ptr , struct kvm_vcpu* vcpu_ptr)
{
	if (kvmmips_handle_kernel_address(run_ptr, vcpu_ptr, 2))
		return RETURN_TO_GUEST;

	/* account info */
	kvmmips_stat_exits(vcpu_ptr, TLBS_EXITS);

	/* TODO: inject to guest OS */
	if(kvmmips_check_pte_rwm(run_ptr, vcpu_ptr, 0x5, 0xd8))
		return RETURN_TO_GUEST;

	kvmmips_queue_exception(vcpu_ptr, KVMMIPS_EXCEPTION_TLBS);

	return RETURN_TO_GUEST;
}

void kvmmips_loongson_tlb_setup(struct kvmmips_vcpu_loongson *vcpu_loongson)
{
	return;
}

void kvmmips_local_flush_tlb_all(void)
{
	local_flush_tlb_all();
}

#ifndef CONFIG_KVM_MULTICLIENT
int kvmmips_loongson_tlb_init(struct kvmmips_vcpu_loongson *vcpu_loongson)
{
	return 0;
}
#else
void kvmmips_tlbsave(struct kvm_vcpu * vcpu_ptr)
{
	struct kvmmips_vcpu_loongson* vcpu_loongson = to_loongson(vcpu_ptr);
	int i;
	struct tlbe * temp_tlbe;
	u32 temp_index, temp_pagemask;
	u64 temp_entryhi, temp_entrylo0, temp_entrylo1;

	temp_index = read_c0_index();
	temp_entryhi = read_c0_entryhi();
	temp_entrylo0 = read_c0_entrylo0();
	temp_entrylo1 = read_c0_entrylo1();
	temp_pagemask = read_c0_pagemask();

	for (i = 0; i < vcpu_loongson->shadow_tlb_size; i++) {
		temp_tlbe = &vcpu_loongson->shadow_tlb[i];

		write_c0_index(i);
		tlb_read();

		temp_tlbe->mask0 = read_c0_entryhi();
		temp_tlbe->mask1 = read_c0_pagemask();
		temp_tlbe->mask2 = read_c0_entrylo0();
		temp_tlbe->mask3 = read_c0_entrylo1();
	}

	write_c0_index(temp_index);
	write_c0_entryhi(temp_entryhi);
	write_c0_entrylo0(temp_entrylo0);
	write_c0_entrylo1(temp_entrylo1);
	write_c0_pagemask(temp_pagemask);
}

void kvmmips_finish_tlbload(struct kvm_vcpu * vcpu_ptr)
{
	struct kvmmips_vcpu_loongson* vcpu_loongson = to_loongson(vcpu_ptr);
	int i;
	struct tlbe * temp_tlbe;
	u32 index, pagemask;
	u64 entryhi, entrylo0, entrylo1;
	u32 temp_index, temp_pagemask;
	u64 temp_entryhi, temp_entrylo0, temp_entrylo1;

	temp_index = read_c0_index();
	temp_entryhi = read_c0_entryhi();
	temp_entrylo0 = read_c0_entrylo0();
	temp_entrylo1 = read_c0_entrylo1();
	temp_pagemask = read_c0_pagemask();

	for (i = 0; i < vcpu_loongson->shadow_tlb_size; i++) {
		temp_tlbe = &vcpu_loongson->shadow_tlb[i];
		index = i;
		entryhi = temp_tlbe->mask0;
		pagemask = (u32)(temp_tlbe->mask1 & 0xffffffff);
		entrylo0 = temp_tlbe->mask2;
		entrylo1 = temp_tlbe->mask3;

		write_c0_index(index);
		write_c0_entryhi(entryhi);
		write_c0_entrylo0(entrylo0);
		write_c0_entrylo1(entrylo1);
		write_c0_pagemask(pagemask);
		tlb_write_indexed();
		write_c0_pagemask(0); //workround for 3a tlb bug
	}

	FLUSH_ITLB;

	write_c0_index(temp_index);
	write_c0_entryhi(temp_entryhi);
	write_c0_entrylo0(temp_entrylo0);
	write_c0_entrylo1(temp_entrylo1);
	write_c0_pagemask(temp_pagemask);
}

int kvmmips_loongson_tlb_init(struct kvmmips_vcpu_loongson *vcpu_loongson)
{
	vcpu_loongson->shadow_tlb_size = KVM_LOONGSON_TLB_SIZE;
	vcpu_loongson->shadow_tlb =
		kzalloc(sizeof(struct tlbe) * KVM_LOONGSON_TLB_SIZE, GFP_KERNEL);
	if (vcpu_loongson->shadow_tlb == NULL)
		goto err_out_shadow;

	return 0;

err_out_shadow:
	kfree(vcpu_loongson->shadow_tlb);

	return -1;
}
#endif
#endif /* CONFIG_KVM_MIPS_LOONGSON3 */

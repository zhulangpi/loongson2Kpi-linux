/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * Copyright (C) 2016-2018 Zhang Shuangshuang <zhangshuangshuang@loongson.cn>
 *
 */

#ifndef __ASM_KVM_LOONGSON_H__
#define __ASM_KVM_LOONGSON_H__

#include <linux/kvm_host.h>

/*
 * Tlb entry of loongson, in fact it's entryhi, pagemask, entrylo0, entrylo1,
 * not exactly the same as hardware.
 */
struct tlbe {
	u64 mask0;
	u64 mask1;
	u64 mask2;
	u64 mask3;
};

struct kvmmips_page {
	struct page* page[2];
};

/* vcpu of loongson */
struct kvmmips_vcpu_loongson {
	struct tlbe* guest_tlb;
	struct tlbe* shadow_tlb;
	struct kvmmips_page* shadow_pages;

	unsigned int guest_tlb_size;
	unsigned int shadow_tlb_size;
	unsigned int guest_tlb_nv;
	unsigned int guest_kernel_index;

	/* fixed kernel address page table */
	pgd_t* kvm_pg_dir;

	/* mapped kernel address page table */
	pgd_t* swapper_pg_dir;

	/* mapped user address page table */
	unsigned long pgd_current;
	struct kvm_vcpu vcpu;
};

static inline struct kvmmips_vcpu_loongson *to_loongson(struct kvm_vcpu * vcpu)
{
	return container_of(vcpu, struct kvmmips_vcpu_loongson, vcpu);
}

#endif /*__ASM_KVM_LOONGSON_H__*/

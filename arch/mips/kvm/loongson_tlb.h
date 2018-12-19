/*
 * Copyright (C) 2016-2018 Zhang Shuangshuang <zhangshuangshuang@loongson.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation.
 */

#ifndef __KVM_LOONGSON_TLB_H__
#define __KVM_LOONGSON_TLB_H__

#ifndef __ASSEMBLY__
#include <asm/kvm_loongson.h>
#include <asm/kvm_asm.h>

#define KVM_LOONGSON_TLB_SIZE	64
#define KVM_TLB_PAGE_SHIFT	13
#define KVM_TLB_PAGE_MASK	((1 << KVM_TLB_PAGE_SHIFT) - 1)

extern void kvmmips_loongson_tlb_setup(struct kvmmips_vcpu_loongson *vcpu_loongson);
extern int kvmmips_loongson_tlb_init(struct kvmmips_vcpu_loongson *vcpu_loongson);
extern void kvmmips_loongson_tlb_load(struct kvm_vcpu *vcpu, int cpu);
extern void kvmmips_complete_mmio_load(struct kvm_run *run_ptr, struct kvm_vcpu *vcpu_ptr);
extern int get_tlb_entry_num(void);
#endif

/* tell support multi client or not */
#define CONFIG_KVM_MULTICLIENT

#endif /* __KVM_LOONGSON_TLB_H__ */

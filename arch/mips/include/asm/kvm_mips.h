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

#ifndef __MIPS_KVM_MIPS_H__
#define __MIPS_KVM_MIPS_H__

enum emulation_result {
	EMULATE_DONE,         /* no further processing */
	EMULATE_DO_MMIO,      /* kvm_run filled with MMIO request */
	EMULATE_DO_DCR,       /* kvm_run filled with DCR request */
	EMULATE_FAIL,         /* can't emulate this instruction */
	EMULATE_AGAIN,        /* something went wrong. go again */
};

extern int kvmmips_core_check_processor_compat(void);
extern struct kvm_vcpu *kvmmips_core_vcpu_create(struct kvm *kvm, unsigned int id);
extern int kvmmips_core_vcpu_setup(struct kvm_vcpu *vcpu);
extern void kvmmips_core_vcpu_load(struct kvm_vcpu *vcpu, int cpu);
extern void kvmmips_deliver_exceptions(struct kvm_vcpu* vcpu_ptr);
extern void kvmmips_queue_exception(struct kvm_vcpu* vcpu_ptr, int exception_type);
extern void kvmmips_dequeue_exception(struct kvm_vcpu* vcpu_ptr, int exception_type);
extern void kvmmips_queue_interrupt(struct kvm_vcpu* vcpu_ptr, struct kvm_interrupt* interrupt);
extern void kvmmips_dequeue_interrupt(struct kvm_vcpu* vcpu_ptr, struct kvm_interrupt* interrupt);
extern int __kvmmips_vcpu_run(struct kvm_run *run, struct kvm_vcpu *vcpu);
extern void kvmmips_init_except_vector(void);

#endif /* __MIPS_KVM_MIPS_H__ */

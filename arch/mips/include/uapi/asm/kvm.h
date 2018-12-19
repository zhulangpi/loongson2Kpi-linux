/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2012  MIPS Technologies, Inc.  All rights reserved.
 * Copyright (C) 2013 Cavium, Inc.
 * Authors: Sanjay Lal <sanjayl@kymasys.com>
 */

#ifndef __LINUX_KVM_MIPS_H
#define __LINUX_KVM_MIPS_H

#include <linux/types.h>

/*
 * KVM MIPS specific structures and definitions.
 *
 * Some parts derived from the x86 version of this file.
 */

/*
 * for KVM_GET_REGS and KVM_SET_REGS
 *
 * If Config[AT] is zero (32-bit CPU), the register contents are
 * stored in the lower 32-bits of the struct kvm_regs fields and sign
 * extended to 64-bits.
 */
#ifndef CONFIG_KVM_MIPS_LOONGSON3
struct kvm_regs {
	/* out (KVM_GET_REGS) / in (KVM_SET_REGS) */
	__u64 gpr[32];
	__u64 hi;
	__u64 lo;
	__u64 pc;
};
#else
struct kvm_regs {
	__u64 cp0_index;
	__u64 cp0_random;
	__u64 cp0_entrylo0;
	__u64 cp0_entrylo1;
	__u64 cp0_context;
	__u64 cp0_pagemask;
	__u64 cp0_pagegrain;
	__u64 cp0_wired;
	__u64 cp0_hwrena;
	__u64 cp0_badvaddr;
	__u64 cp0_count;
	__u64 cp0_entryhi;
	__u64 cp0_compare;
	__u64 cp0_status;
	__u64 cp0_intctl;
	__u64 cp0_srsctl;
	__u64 cp0_srsmap;
	__u64 cp0_cause;
	__u64 cp0_epc;
	__u64 cp0_prid;
	__u64 cp0_ebase;
	__u64 cp0_config;
	__u64 cp0_config1;
	__u64 cp0_config2;
	__u64 cp0_config3;
	__u64 cp0_lladdr;
	__u64 cp0_watchlo;
	__u64 cp0_watchhi;
	__u64 cp0_xcontext;
	__u64 cp0_diagnostic;
	__u64 cp0_debug;
	__u64 cp0_depc;
	__u64 cp0_perfcnt;
	__u64 cp0_perfctl;
	__u64 cp0_ecc;
	__u64 cp0_cacheerr;
	__u64 cp0_taglo;
	__u64 cp0_datalo;
	__u64 cp0_taghi;
	__u64 cp0_datahi;
	__u64 cp0_errorepc;
	__u64 cp0_desave;

	__u64 pc;	/* pc pointer, can't be seen in real hardware */
	__u64 gpr[32];	/* general registers */

	__u64 fpr[32];	/* fpu registers */
	__u64 host_stack;
	__u64 fcr;	/* fcsr registers */
	__u64 save_fpr;	/* indicates save fpr or not */
	__u64 hi;
	__u64 lo;

	/* temp host cp0 reg backup for exception */
	__u64 temp_cp0_cause;
	__u64 temp_cp0_epc;
	__u64 temp_cp0_badvaddr;
	__u64 temp_cp0_pagemask;
	__u64 temp_cp0_wired;
	__u64 temp_cp0_entryhi;
	__u64 temp_cp0_context;

	__u64 host_cp0_entryhi;
	__u64 host_cp0_pagemask;
	__u64 host_cp0_wired;

	/* mmio related */
	__u64 io_gpr;
	__u64 mmio_sign_extend;

	/* pending the exceptions */
	__u64 pending_exceptions;

	/* pending the irq */
	__u64 pending_irqs;
	/* others should be added when programming */

	/* counter */
	__u64 count_start;
	__u64 count_end;

	/* last compare and count_previous for multi vm */
	__u64 last_compare;
	__u64 count_previous;
	__u64 cp0_origin;
	__u64 cp1_fcr;
};
#endif

/*
 * for KVM_GET_FPU and KVM_SET_FPU
 *
 * If Status[FR] is zero (32-bit FPU), the upper 32-bits of the FPRs
 * are zero filled.
 */
struct kvm_fpu {
	__u64 fpr[32];
	__u32 fir;
	__u32 fccr;
	__u32 fexr;
	__u32 fenr;
	__u32 fcsr;
	__u32 pad;
};


/*
 * For MIPS, we use KVM_SET_ONE_REG and KVM_GET_ONE_REG to access CP0
 * registers.  The id field is broken down as follows:
 *
 *  bits[2..0]   - Register 'sel' index.
 *  bits[7..3]   - Register 'rd'  index.
 *  bits[15..8]  - Must be zero.
 *  bits[31..16] - 1 -> CP0 registers.
 *  bits[51..32] - Must be zero.
 *  bits[63..52] - As per linux/kvm.h
 *
 * Other sets registers may be added in the future.  Each set would
 * have its own identifier in bits[31..16].
 *
 * The registers defined in struct kvm_regs are also accessible, the
 * id values for these are below.
 */

#define KVM_REG_MIPS_R0 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 0)
#define KVM_REG_MIPS_R1 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 1)
#define KVM_REG_MIPS_R2 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 2)
#define KVM_REG_MIPS_R3 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 3)
#define KVM_REG_MIPS_R4 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 4)
#define KVM_REG_MIPS_R5 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 5)
#define KVM_REG_MIPS_R6 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 6)
#define KVM_REG_MIPS_R7 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 7)
#define KVM_REG_MIPS_R8 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 8)
#define KVM_REG_MIPS_R9 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 9)
#define KVM_REG_MIPS_R10 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 10)
#define KVM_REG_MIPS_R11 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 11)
#define KVM_REG_MIPS_R12 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 12)
#define KVM_REG_MIPS_R13 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 13)
#define KVM_REG_MIPS_R14 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 14)
#define KVM_REG_MIPS_R15 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 15)
#define KVM_REG_MIPS_R16 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 16)
#define KVM_REG_MIPS_R17 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 17)
#define KVM_REG_MIPS_R18 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 18)
#define KVM_REG_MIPS_R19 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 19)
#define KVM_REG_MIPS_R20 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 20)
#define KVM_REG_MIPS_R21 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 21)
#define KVM_REG_MIPS_R22 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 22)
#define KVM_REG_MIPS_R23 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 23)
#define KVM_REG_MIPS_R24 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 24)
#define KVM_REG_MIPS_R25 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 25)
#define KVM_REG_MIPS_R26 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 26)
#define KVM_REG_MIPS_R27 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 27)
#define KVM_REG_MIPS_R28 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 28)
#define KVM_REG_MIPS_R29 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 29)
#define KVM_REG_MIPS_R30 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 30)
#define KVM_REG_MIPS_R31 (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 31)

#define KVM_REG_MIPS_HI (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 32)
#define KVM_REG_MIPS_LO (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 33)
#define KVM_REG_MIPS_PC (KVM_REG_MIPS | KVM_REG_SIZE_U64 | 34)

/*
 * KVM MIPS specific structures and definitions
 *
 */
struct kvm_debug_exit_arch {
	__u64 epc;
};

/* for KVM_SET_GUEST_DEBUG */
struct kvm_guest_debug_arch {
};

/* definition of registers in kvm_run */
struct kvm_sync_regs {
};

/* dummy definition */
struct kvm_sregs {
};

struct kvm_mips_interrupt {
	/* in */
	__u32 cpu;
	__u32 irq;
};

#endif /* __LINUX_KVM_MIPS_H */

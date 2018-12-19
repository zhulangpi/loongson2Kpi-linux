/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2003 Ralf Baechle
 */
#ifndef _ASM_ASMMACRO_H
#define _ASM_ASMMACRO_H

#include <asm/hazards.h>

#ifdef CONFIG_32BIT
#include <asm/asmmacro-32.h>
#endif
#ifdef CONFIG_64BIT
#include <asm/asmmacro-64.h>
#endif
#ifdef CONFIG_MIPS_MT_SMTC
#include <asm/mipsmtregs.h>
#endif

#ifdef CONFIG_MIPS_MT_SMTC
	.macro	local_irq_enable reg=t0
	mfc0	\reg, CP0_TCSTATUS
	ori	\reg, \reg, TCSTATUS_IXMT
	xori	\reg, \reg, TCSTATUS_IXMT
	mtc0	\reg, CP0_TCSTATUS
	_ehb
	.endm

	.macro	local_irq_disable reg=t0
	mfc0	\reg, CP0_TCSTATUS
	ori	\reg, \reg, TCSTATUS_IXMT
	mtc0	\reg, CP0_TCSTATUS
	_ehb
	.endm
#elif defined(CONFIG_CPU_MIPSR2)
	.macro	local_irq_enable reg=t0
	ei
	irq_enable_hazard
	.endm

	.macro	local_irq_disable reg=t0
	di
	irq_disable_hazard
	.endm
#else
#ifdef CONFIG_PARA_VIRT
	.macro	local_irq_enable reg=t0 temp=t1
	.set noat
	PTR_LA	\reg,	paravirt_cp0
	LONG_ADDIU	\reg,	PV_CP0_STATUS
	LONG_L	\reg,	0(\reg)
	.set at
#else
	.macro	local_irq_enable reg=t0
	mfc0	\reg, CP0_STATUS
#endif
	ori	\reg, \reg, 1
#ifdef CONFIG_PARA_VIRT
	.set noat
	PTR_LA	\temp,	paravirt_cp0
	LONG_ADDIU	\temp,	PV_CP0_STATUS
	LONG_S	\reg,	0(\temp)
	.set at
#else
	mtc0	\reg, CP0_STATUS
#endif
	irq_enable_hazard
	.endm

#ifdef CONFIG_PARA_VIRT
	.macro	local_irq_disable reg=t0 temp=t1
	.set noat
	PTR_LA	\reg,	paravirt_cp0
	LONG_ADDIU	\reg,	PV_CP0_STATUS
	LONG_L	\reg,	0(\reg)
	.set at
#else
	.macro	local_irq_disable reg=t0
	mfc0	\reg, CP0_STATUS
#endif
	ori	\reg, \reg, 1
	xori	\reg, \reg, 1
#ifdef CONFIG_PARA_VIRT
	.set noat
	PTR_LA	\temp,	paravirt_cp0
	LONG_ADDIU	\temp,	PV_CP0_STATUS
	LONG_S	\reg,	0(\temp)
	.set at
#else
	mtc0	\reg, CP0_STATUS
#endif
	irq_disable_hazard
	.endm
#endif /* CONFIG_MIPS_MT_SMTC */

/*
 * Temporary until all gas have MT ASE support
 */
	.macro	DMT	reg=0
	.word	0x41600bc1 | (\reg << 16)
	.endm

	.macro	EMT	reg=0
	.word	0x41600be1 | (\reg << 16)
	.endm

	.macro	DVPE	reg=0
	.word	0x41600001 | (\reg << 16)
	.endm

	.macro	EVPE	reg=0
	.word	0x41600021 | (\reg << 16)
	.endm

	.macro	MFTR	rt=0, rd=0, u=0, sel=0
	 .word	0x41000000 | (\rt << 16) | (\rd << 11) | (\u << 5) | (\sel)
	.endm

	.macro	MTTR	rt=0, rd=0, u=0, sel=0
	 .word	0x41800000 | (\rt << 16) | (\rd << 11) | (\u << 5) | (\sel)
	.endm

#endif /* _ASM_ASMMACRO_H */

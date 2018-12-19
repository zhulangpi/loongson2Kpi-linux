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

#ifndef __MIPS_KVM_ASM_H__
#define __MIPS_KVM_ASM_H__

#ifdef MFC0
#undef MFC0
#endif

#ifdef MTC0
#undef MTC0
#endif

/* used for decode the instruction */
#define OPCODE1		0xFC000000
#define OPCODE1_OFFSET	26
#define OPCODE2		0x03E00000
#define OPCODE2_OFFSET	21

#define MFC0		0
#define DMFC0		1
#define CFC0		2
#define MTC0		4
#define DMTC0		5
#define RDPGPR		10
#define XI		11
#define WRPGPR		14
#define TLBX		16

#define LDL		26
#define LDR		27
#define LB		32
#define LH		33
#define LWL		34
#define LW		35
#define LBU		36
#define LHU		37
#define LWR		38
#define LWU		39
#define SB		40
#define SH		41
#define SWL		42
#define SW		43
#define SDL		44
#define SDR		45
#define SWR		46
#define LL		48
#define LLD		52
#define LD		55
#define SC		56
#define SCD		60
#define SD		63
#define OFFSET_ID	0xFFFF

#define XI_ID		0x0000001F
#define DI		0
#define EI		32

#define TLB_ID		0x0000001F
#define TLBR		1
#define TLBWI		2
#define TLBWR		6
#define TLBP		8
#define ERET		24

#define OP1		0x001F0000
#define OP1_OFFSET	16
#define OP2		0x0000F800
#define OP2_OFFSET	11

#endif  /* __MIPS_KVM_ASM_H__ */

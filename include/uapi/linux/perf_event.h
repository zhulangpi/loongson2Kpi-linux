/*
 * Performance events:
 *
 *    Copyright (C) 2008-2009, Thomas Gleixner <tglx@linutronix.de>
 *    Copyright (C) 2008-2011, Red Hat, Inc., Ingo Molnar
 *    Copyright (C) 2008-2011, Red Hat, Inc., Peter Zijlstra
 *
 * Data type definitions, declarations, prototypes.
 *
 *    Started by: Thomas Gleixner and Ingo Molnar
 *
 * For licencing details see kernel-base/COPYING
 */
#ifndef _UAPI_LINUX_PERF_EVENT_H
#define _UAPI_LINUX_PERF_EVENT_H

#include <linux/types.h>
#include <linux/ioctl.h>
#include <asm/byteorder.h>

/*
 * User-space ABI bits:
 */

/*
 * attr.type
 */
enum perf_type_id {
	PERF_TYPE_HARDWARE			= 0,
	PERF_TYPE_SOFTWARE			= 1,
	PERF_TYPE_TRACEPOINT			= 2,
	PERF_TYPE_HW_CACHE			= 3,
	PERF_TYPE_RAW				= 4,
	PERF_TYPE_BREAKPOINT			= 5,

	PERF_TYPE_MAX,				/* non-ABI */
};

/*
 * Generalized performance event event_id types, used by the
 * attr.event_id parameter of the sys_perf_event_open()
 * syscall:
 */
enum perf_hw_id {
	/*
	 * Common hardware events, generalized by the kernel:
	 */
	PERF_COUNT_HW_CPU_CYCLES		= 0,
	PERF_COUNT_HW_INSTRUCTIONS		= 1,
	PERF_COUNT_HW_CACHE_REFERENCES		= 2,
	PERF_COUNT_HW_CACHE_MISSES		= 3,
	PERF_COUNT_HW_BRANCH_INSTRUCTIONS	= 4,
	PERF_COUNT_HW_BRANCH_MISSES		= 5,
	PERF_COUNT_HW_BUS_CYCLES		= 6,
	PERF_COUNT_HW_STALLED_CYCLES_FRONTEND	= 7,
	PERF_COUNT_HW_STALLED_CYCLES_BACKEND	= 8,
	PERF_COUNT_HW_REF_CPU_CYCLES		= 9,

#if defined(CONFIG_CPU_LOONGSON3_GS464E)
	/*loongson-3A1000 hardware events*/
	PERF_COUNT_HW_JUMP_INSTRUCTIONS     = 9,
	PERF_COUNT_HW_JR31_INSTRUCTIONS     = 10,
	PERF_COUNT_HW_ICACHE_MISSES     = 11,
	PERF_COUNT_HW_ALU1_ISSUED       = 12,
	PERF_COUNT_HW_MEM_ISSUED        = 13,
	PERF_COUNT_HW_FALU1_ISSUED      = 14,
	PERF_COUNT_HW_BHT_BRANCH_INSTRUCTIONS   = 15,
	PERF_COUNT_HW_MEM_READ          = 16,
	PERF_COUNT_HW_FQUEUE_FULL       = 17,
	PERF_COUNT_HW_ROQ_FULL          = 18,
	PERF_COUNT_HW_CP0_QUEUE_FULL        = 19,
	PERF_COUNT_HW_TLB_REFILL        = 20,
	PERF_COUNT_HW_EXCEPTION         = 21,
	PERF_COUNT_HW_INTERNAL_EXCEPTION    = 22,

	PERF_COUNT_HW_JR_MISPREDICTED       = 23,
	PERF_COUNT_HW_JR31_MISPREDICTED     = 24,
	PERF_COUNT_HW_DCACHE_MISSES     = 25,
	PERF_COUNT_HW_ALU2_ISSUED       = 26,
	PERF_COUNT_HW_FALU2_ISSUED      = 27,
	PERF_COUNT_HW_UNCACHED_ACCESS       = 28,
	PERF_COUNT_HW_BHT_MISPREDICTED      = 29,
	PERF_COUNT_HW_MEM_WRITE         = 30,
	PERF_COUNT_HW_FTQ_FULL          = 31,
	PERF_COUNT_HW_BRANCH_QUEUE_FULL     = 32,
	PERF_COUNT_HW_ITLB_MISSES       = 33,
	PERF_COUNT_HW_TOTAL_EXCEPTIONS      = 34,
	PERF_COUNT_HW_LOAD_SPECULATION_MISSES   = 35,
	PERF_COUNT_HW_CP0Q_FORWARD_VALID    = 36,

	/*loongson-3A2000 hardware events*/
	/*fetch*/
	PERF_COUNT_HW_INSTQ_EMPTY               = 9, 
	PERF_COUNT_HW_INSTQ_INSTRUCTIONS                = 10,
	PERF_COUNT_HW_INSTQ_1           = 11,
	PERF_COUNT_HW_INSTQ_2           = 12,
	PERF_COUNT_HW_INSTQ_3           = 13,
	PERF_COUNT_HW_INSTQ_4           = 14,
	PERF_COUNT_HW_INSTQ_5           = 15,
	PERF_COUNT_HW_INSTQ_6           = 16,
	PERF_COUNT_HW_INSTQ_7           = 17,
	PERF_COUNT_HW_INSTQ_8           = 18,
	PERF_COUNT_HW_INSTQ_LESSTHAN_8          = 19,
	PERF_COUNT_HW_INSTQ_FULL                = 20,
	PERF_COUNT_HW_DECODE_INST               = 21,
	PERF_COUNT_HW_LOOP_BUFFER_INST          = 22,
	PERF_COUNT_HW_LOOP_FIND         = 23,
	PERF_COUNT_HW_LOOP_TRIGGER              = 24,
	PERF_COUNT_HW_DECODE_BRANCH_0           = 25,
	PERF_COUNT_HW_DECODE_BRANCH_1           = 26,
	PERF_COUNT_HW_DECODE_BRANCH_2           = 27,
	PERF_COUNT_HW_ICACHE_MISSES_BLOCK               = 28,
	PERF_COUNT_HW_BRBTB_TAKEN_BRANCH_MISSES         = 29,
	PERF_COUNT_HW_ICACHE_REPLACE            = 30,
	PERF_COUNT_HW_ITLB_MISS_TLB_HIT         = 31,
	PERF_COUNT_HW_ITLB_FLUSHED              = 32,
	/*rmap*/
	PERF_COUNT_HW_RESOURCE_ALLOC_BLOCKED            = 33,
	PERF_COUNT_HW_GR_BLOCKED                = 34,
	PERF_COUNT_HW_GR_PSEUDO_BLOCKED         = 35,
	PERF_COUNT_HW_FR_BLOCKED                = 36,
	PERF_COUNT_HW_FR_PSEUDO_BLOCKED         = 37,
	PERF_COUNT_HW_FCR_BLOCKED               = 38,
	PERF_COUNT_HW_FCR_PSEUDO_BLOCKED                = 39,
	PERF_COUNT_HW_ACC_BLOCKED               = 40,
	PERF_COUNT_HW_ACC_PSEUDO_BLOCKED                = 41,
	PERF_COUNT_HW_DSPCTRL_BLOCKED           = 42,
	PERF_COUNT_HW_DSPCTRL_PSEUDO_BLOCKED            = 43,
	PERF_COUNT_HW_BRQ_BLOCKED               = 44,
	PERF_COUNT_HW_BRQ_PSEUDO_BLOCKED                = 45,
	PERF_COUNT_HW_FXQ_BLOCKED               = 46,
	PERF_COUNT_HW_FXQ_PSEUDO_BLOCKED                = 47,
	PERF_COUNT_HW_FTQ_BLOCKED               = 48,
	PERF_COUNT_HW_FTQ_PSEUDO_BLOCKED                = 49,
	PERF_COUNT_HW_MMQ_BLOCKED               = 50,
	PERF_COUNT_HW_MMQ_PSEUDO_BLOCKED                = 51,
	PERF_COUNT_HW_CP0Q_BLOCKED              = 52,
	PERF_COUNT_HW_CP0Q_PSEUDO_BLOCKED               = 53,
	PERF_COUNT_HW_ROQ_BLOCKED               = 54,
	PERF_COUNT_HW_NOP_INST          = 55,
	PERF_COUNT_HW_REGMAP_ISSUED             = 56,
	PERF_COUNT_HW_EXCEPTIONS                = 57,
	PERF_COUNT_HW_BRANCH_MISSES_OVERHEAD            = 58,
	/*roq*/
	PERF_COUNT_HW_ALU_COMMITTED             = 59,
	PERF_COUNT_HW_FALU_COMMITTED            = 60,
	PERF_COUNT_HW_MEMORY_SWAP_COMMITTED             = 61,
	PERF_COUNT_HW_LOAD_COMMITTED            = 62,
	PERF_COUNT_HW_STORE_COMMITTED           = 63,
	PERF_COUNT_HW_LL_COMMITTED              = 64,
	PERF_COUNT_HW_SC_COMMITTED              = 65,
	PERF_COUNT_HW_UNALIGNED_LOAD_COMMITTED          = 66,
	PERF_COUNT_HW_UNALIGNED_STORE_COMMITTED         = 67,
	PERF_COUNT_HW_EXCEPTIONS_AND_INTERRUPTS         = 68,
	PERF_COUNT_HW_INTERRUPTS                = 69,
	PERF_COUNT_HW_ROQ_INTERRUPT             = 70,
	PERF_COUNT_HW_ROQ_INTERRUPT_INST                = 71,
	PERF_COUNT_HW_VM_EXCEPTIONS             = 72,
	PERF_COUNT_HW_ADDRESS_FAULT_EXCEPTIONS          = 73,
	PERF_COUNT_HW_TLB_EXCEPTIONS            = 74,
	PERF_COUNT_HW_TLB_REFILL_EXCEPTIONS             = 75,
	PERF_COUNT_HW_TLB_REFILL_HANDLE_TIME            = 76,
	PERF_COUNT_HW_JUMP_REGISTER             = 77,
	PERF_COUNT_HW_JUMP_AND_LINK             = 78,
	PERF_COUNT_HW_BRANCH_AND_LINK           = 79,
	PERF_COUNT_HW_BHT_BRANCH                = 80,
	PERF_COUNT_HW_LIKELY_BRANCH             = 81,
	PERF_COUNT_HW_NOT_TAKEN_BRANCH          = 82,
	PERF_COUNT_HW_TAKEN_BRANCH              = 83,
	PERF_COUNT_HW_JUMP_REGISTER_MISSES              = 84,
	PERF_COUNT_HW_JUMP_AND_LINK_MISSES              = 85,
	PERF_COUNT_HW_BRANCH_AND_LINK_MISSES            = 86,
	PERF_COUNT_HW_BHT_BRANCH_MISSES         = 87,
	PERF_COUNT_HW_LIKELY_BRANCH_MISSES              = 88,
	PERF_COUNT_HW_NOT_TAKEN_MISSES          = 89,
	PERF_COUNT_HW_TAKEN_MISSES              = 90,
	/*fix*/
	PERF_COUNT_HW_FXQ_NO_ISSUE              = 91,
	PERF_COUNT_HW_FXQ_ISSUE_OPERAND         = 92,
	PERF_COUNT_HW_FXQ_FU0_OPERAND           = 93,
	PERF_COUNT_HW_FXQ_FU1_OPERAND           = 94,
	PERF_COUNT_HW_FU0_FIXED_MUL             = 95,
	PERF_COUNT_HW_FU0_FIXED_DIV             = 96,
	PERF_COUNT_HW_FU1_FIXED_MUL             = 97,
	PERF_COUNT_HW_FU1_FIXED_DIV             = 98,
	/*float*/
	PERF_COUNT_HW_FTQ_NO_ISSUE              = 99,
	PERF_COUNT_HW_FTQ_ISSUE_OPERAND         = 100,
	PERF_COUNT_HW_FTQ_FU3_OPERAND           = 101,
	PERF_COUNT_HW_FTQ_FU4_OPERAND           = 102,
	PERF_COUNT_HW_FU3_EMPTY_FU4_FULL                = 103,
	PERF_COUNT_HW_FU4_EMPTY_FU3_FULL                = 104,
	PERF_COUNT_HW_SCALAR_FLOAT_OPERAND              = 105,
	PERF_COUNT_HW_GS_ALU_INST               = 106,
	PERF_COUNT_HW_FIXED_VECTOR_ISSUE_64             = 107,
	PERF_COUNT_HW_VECTOR_ISSUE_128          = 108,
	PERF_COUNT_HW_FIXED_VECTOR_ISSUE_128            = 109,
	PERF_COUNT_HW_FLOAT_VECTOR_ISSUE_128            = 110,
	PERF_COUNT_HW_VECTOR_ISSUE_256          = 111,
	PERF_COUNT_HW_FIXED_VECTOR_ISSUE_256            = 112,
	PERF_COUNT_HW_FLOAT_VECTOR_ISSUE_256            = 113,
	PERF_COUNT_HW_FU3_VECTOR_FIXED_DIV              = 114,
	PERF_COUNT_HW_FU3_FLOAT_DIV_SQRT                = 115,
	PERF_COUNT_HW_FU4_VECTOR_FIXED_DIV              = 116,
	PERF_COUNT_HW_FU4_FLOAT_DIV_SQRT                = 117,
	/*memory*/
	PERF_COUNT_HW_MMQ_NO_ISSUE              = 118,
	PERF_COUNT_HW_MMQ_ISSUE_OPERAND         = 119,
	PERF_COUNT_HW_MMQ_FU2_INST              = 120,
	PERF_COUNT_HW_MMQ_FU5_INST              = 121,
	PERF_COUNT_HW_LOAD_ISSUE                = 122,
	PERF_COUNT_HW_STORE_ISSUE               = 123,
	PERF_COUNT_HW_SRC_FLOAT_MEM_INST                = 124,
	PERF_COUNT_HW_VECTOR_MEM_ISSUE          = 125,
	PERF_COUNT_HW_FIX_FLOAT_SHIFT_ISSUE             = 126,
	PERF_COUNT_HW_WAIT_FIRST_BLOCK_CYCLES           = 127,
	PERF_COUNT_HW_SYNC_BLOCK_CYCLES         = 128,
	PERF_COUNT_HW_STALL_ISSUE_BLOCK_CYCLES          = 129,
	PERF_COUNT_HW_SOFTWARE_PREFETCH_TOTAL           = 130,
	PERF_COUNT_HW_DMEMREF_BLOCK_DCACHEWRITE         = 131,
	PERF_COUNT_HW_DMEMREF_BANK_CLASH                = 132,
	PERF_COUNT_HW_REFILL_BLOCK_DMEMREF              = 133,
	PERF_COUNT_HW_DCACHEWRITE_NO_CANCEL             = 134,
	PERF_COUNT_HW_DCACHEWRITE0_AND_1_VALID          = 135,
	PERF_COUNT_HW_SC_WRITE_DCACHE           = 136,
	PERF_COUNT_HW_STORE_DCACHE_MISS         = 137,
	PERF_COUNT_HW_STORE_DCACHE_SHARED_MISS          = 138,
	/*cache2mem*/
	PERF_COUNT_HW_STORE_DCACHE_HIT          = 139,
	PERF_COUNT_HW_LOAD_HIT          = 140,
	PERF_COUNT_HW_FWDBUS2           = 141,
	PERF_COUNT_HW_FWDBUS5           = 142,
	PERF_COUNT_HW_FWDBUS_TOTAL              = 143,
	PERF_COUNT_HW_DWAITSTORE                = 144,
	PERF_COUNT_HW_MISPEC            = 145,
	PERF_COUNT_HW_DCACHEWRITE_CANCEL                = 146,
	PERF_COUNT_HW_CP0Q_DMEMREAD             = 147,
	PERF_COUNT_HW_CP0Q_DUNCACHE             = 148,
	PERF_COUNT_HW_RESBUS2_OCCUPY_RESBUS5            = 149,
	PERF_COUNT_HW_SW_PREFETCH_L1DCACHE_HIT          = 150,
	PERF_COUNT_HW_STORE_SW_PREFETCH_L1DCACHE_HIT            = 151,
	PERF_COUNT_HW_STORE_SW_PREFETCH_L1DCACHE_MISS           = 152,
	PERF_COUNT_HW_LOAD_SW_PREFETCH_L1DCACHE_HIT             = 153,
	PERF_COUNT_HW_LOAD_SW_PREFETCH_L1DCACHE_MISS            = 154,
	PERF_COUNT_HW_STORE_L1DCACHE_MISS_SHARE_STATE           = 155,
	PERF_COUNT_HW_SPECFWDBUS2               = 156,
	PERF_COUNT_HW_SPECFWDBUS5               = 157,
	PERF_COUNT_HW_SPECFWDBUS2_TOTAL         = 158,
	PERF_COUNT_HW_DATA_LOAD_VCACHE_ACCESS_REQ               = 159,
	PERF_COUNT_HW_DATA_STORE_VCACHE_ACCESS_REQ              = 160,
	PERF_COUNT_HW_DATA_VCACHE_ACCESS_REQ            = 161,
	PERF_COUNT_HW_INST_VCACHE_ACCESS_REQ            = 162,
	PERF_COUNT_HW_VCACHE_ACCESS             = 163,
	PERF_COUNT_HW_SW_PREFETCH_ACCESS_VCACHE         = 164,
	PERF_COUNT_HW_VCACHE_LOAD_HIT           = 165,
	PERF_COUNT_HW_VCACHE_STORE_HIT          = 166,
	PERF_COUNT_HW_VCACHE_DATA_HIT           = 167,
	PERF_COUNT_HW_VCACHE_INST_HIT           = 168,
	PERF_COUNT_HW_VCACHE_HIT                = 169,
	PERF_COUNT_HW_VCACHE_SW_PREFETCH_HIT            = 170,
	PERF_COUNT_HW_VCACHE_LOAD_MISS          = 171,
	PERF_COUNT_HW_VCACHE_STORE_MISS         = 172,
	PERF_COUNT_HW_VCACHE_DATA_MISS          = 173,
	PERF_COUNT_HW_VCACHE_INST_MISS          = 174,
	PERF_COUNT_HW_VCACHE_MISS               = 175,
	PERF_COUNT_HW_VCACHE_SW_PREFETCH_MISS           = 176,
	PERF_COUNT_HW_VCACHE_EXTREQ_INVALID             = 177,
	PERF_COUNT_HW_VCACHE_WTBK_DEGRADE               = 178,
	PERF_COUNT_HW_VCACHE_INV_INVALID                = 179,
	PERF_COUNT_HW_VCACHE_INVWTBK_INVALID            = 180,
	PERF_COUNT_HW_AR_REQUEST_ISSUE          = 181,
	PERF_COUNT_HW_AW_REQUEST_ISSUE          = 182,
	PERF_COUNT_HW_AW_DATA_REQUEST           = 183,
	PERF_COUNT_HW_AR_BLOCKED_AW_UNDONE              = 184,
	PERF_COUNT_HW_MISSQ_WTBK_REQUEST                = 185,
	PERF_COUNT_HW_MISSQ_INVWTBK_REQUEST             = 186,
	PERF_COUNT_HW_MISSQ_INV_REQUEST         = 187,
	PERF_COUNT_HW_MISSQ_INV_CLASS_REQUEST           = 188,
	PERF_COUNT_HW_REFILL_TOTAL              = 189,
	PERF_COUNT_HW_REFILL_ICACHE             = 190,
	PERF_COUNT_HW_REFILL_DCACHE             = 191,
	PERF_COUNT_HW_REPLACE_REFILL            = 192,
	PERF_COUNT_HW_REFILL_DCACHE_SHARED              = 193,
	PERF_COUNT_HW_REFILL_DCACHE_EXC         = 194,
	PERF_COUNT_HW_REFILL_DATA_TOTAL         = 195,
	PERF_COUNT_HW_REFILL_INST_TOTAL         = 196,
	PERF_COUNT_HW_DCACHE_REPLACE_VALID_BLOCK                = 197,
	PERF_COUNT_HW_DCACHE_REPLACE_SHARED_BLOCK               = 198,
	PERF_COUNT_HW_DCACHE_REPLACE_EXC_BLOCK          = 199,
	PERF_COUNT_HW_DCACHE_REPLACE_DIRTY_BLOCK                = 200,
	PERF_COUNT_HW_ICACHE_REPLACE_VALID_DATA         = 201,
	PERF_COUNT_HW_VCACHE_REPLACE            = 202,
	PERF_COUNT_HW_VCACHE_REPLACE_VALID_BLOCK                = 203,
	PERF_COUNT_HW_VCACHE_REPLACE_SHARED_BLOCK               = 204,
	PERF_COUNT_HW_VCACHE_REPLACE_EXC_BLOCK          = 205,
	PERF_COUNT_HW_VCACHE_REPLACE_DIRTY_BLOCK                = 206,
	PERF_COUNT_HW_VCACHE_REPLACE_VALID_DCBLOCK              = 207,
	PERF_COUNT_HW_VCACHE_REPLACE_VALID_ICBLOCK              = 208,
	PERF_COUNT_HW_SCACHE_LOAD_NOT_RETURN            = 209,
	PERF_COUNT_HW_SCACHE_STORE_NOT_RETURN           = 210,
	PERF_COUNT_HW_SCACHE_ICACHEREQ_NOT_RETURN               = 211,
	PERF_COUNT_HW_SCREAD_TOTAL              = 212,
	PERF_COUNT_HW_SCREAD_LOAD               = 213,
	PERF_COUNT_HW_SCREAD_STORE              = 214,
	PERF_COUNT_HW_SCREAD_DATA_ACCESS                = 215,
	PERF_COUNT_HW_SCREAD_INST_ACCESS                = 216,
	PERF_COUNT_HW_SCREAD_NPREFETCH          = 217,
	PERF_COUNT_HW_SCREAD_NPREFETCH_DATA_LOAD                = 218,
	PERF_COUNT_HW_SCREAD_NPREFETCH_STORE            = 219,
	PERF_COUNT_HW_SCREAD_NPREFETCH_DATA_ACCESS              = 220,
	PERF_COUNT_HW_SCREAD_NPREFETCH_INST_ACCESS              = 221,
	PERF_COUNT_HW_SCREAD_PREFETCH           = 222,
	PERF_COUNT_HW_SCREAD_LOAD_PREFETCH              = 223,
	PERF_COUNT_HW_SCREAD_STORE_PREFETCH             = 224,
	PERF_COUNT_HW_SCREAD_PREFETCH_DATA_ACCESS               = 225,
	PERF_COUNT_HW_SCREAD_PREFETCH_INST_ACCESS               = 226,
	PERF_COUNT_HW_MISSQ_SW_PREFETCH_REQUEST         = 227,
	PERF_COUNT_HW_MISSQ_SCWRITE             = 228,
	PERF_COUNT_HW_MISSQ_REPLACE_SCWRITE             = 229,
	PERF_COUNT_HW_MISSQ_INVALID_SCWRITE             = 230,
	PERF_COUNT_HW_MISSQ_REPLACE_VALID_SCWRITE               = 231,
	PERF_COUNT_HW_MISSQ_ACCEPT_REQ          = 232,
	PERF_COUNT_HW_MISSQ_ACCEPT_LOAD         = 233,
	PERF_COUNT_HW_MISSQ_ACCEPT_SIORE                = 234,
	PERF_COUNT_HW_MISSQ_DATA_ACCESS         = 235,
	PERF_COUNT_HW_MISSQ_INST_ACCESS         = 236,
	PERF_COUNT_HW_MISSQ_NON_EMPTY           = 237,
	PERF_COUNT_HW_MISSQ_COMMON_ACCESS_OCCUPY                = 238,
	PERF_COUNT_HW_MISSQ_FETCH_ACCESS_OCCUPY         = 239,
	PERF_COUNT_HW_MISSQ_EXTERNAL_REQ_OCCUPY         = 240,
	PERF_COUNT_HW_MISSQ_PREFETCH_REQ_OCCUPY         = 241,
	PERF_COUNT_HW_MISSQ_OCCUPY_CYCLES               = 242,
	PERF_COUNT_HW_MISSQ_COMMON_ACCESS_CYCLES                = 243,
	PERF_COUNT_HW_MISSQ_FETCH_ACCESS_CYCLES         = 244,
	PERF_COUNT_HW_MISSQ_EXTERNAL_REQ_CYCLES         = 245,
	PERF_COUNT_HW_MISSQ_PREFETCH_REQ_CYCLES         = 246,
	PERF_COUNT_HW_MISSQ_FULL_COUNT          = 247,
	PERF_COUNT_HW_LOAD_MISSQ_PREFETCH               = 248,
	PERF_COUNT_HW_LOAD_MISSQ_PRE_SCREF              = 249,
	PERF_COUNT_HW_LOAD_MISSQ_PRE_RDY                = 250,
	PERF_COUNT_HW_LOAD_MISSQ_PRE_WAIT               = 251,
	PERF_COUNT_HW_STORE_MISSQ_PRE_SCREF_LOAD                = 252,
	PERF_COUNT_HW_STORE_MISSQ_PRE_RDY_SHARD         = 253,
	PERF_COUNT_HW_STORE_MISSQ_PRE_WAIT_LOAD         = 254,
	PERF_COUNT_HW_STORE_MISSQ_PRE_SCREF_STORE               = 255,
	PERF_COUNT_HW_STORE_MISSQ_PRE_RDY_EXC           = 256,
	PERF_COUNT_HW_STORE_MISSQ_PRE_WAIT_STORE                = 257,
	PERF_COUNT_HW_STORE_MISSQ_PREFETCH              = 258,
	PERF_COUNT_HW_STORE_MISSQ_VALID_PREFETCH                = 259,
	PERF_COUNT_HW_ALL_REQ_MISSQ_PREFETCH            = 260,
	PERF_COUNT_HW_ALL_REQ_MISSQ_PRE_SCREF           = 261,
	PERF_COUNT_HW_ALL_REQ_MISSQ_PRE_RDY             = 262,
	PERF_COUNT_HW_ALL_REQ_MISSQ_PRE_WAIT            = 263,
	PERF_COUNT_HW_FETCH_MISSQ_PREFETCH              = 264,
	PERF_COUNT_HW_FETCH_PRE_SCREF           = 265,
	PERF_COUNT_HW_FETCH_PRE_RDY             = 266,
	PERF_COUNT_HW_FETCH_PRE_WAIT            = 267,
	PERF_COUNT_HW_DATA_FETCH_PRE_RDY                = 268,
	PERF_COUNT_HW_DATA_FETCH_PRE_WAIT               = 269,
	PERF_COUNT_HW_HW_LOAD_PRE_SCACHE_CANCEL         = 270,
	PERF_COUNT_HW_HW_STORE_PRE_SCACHE_CANCEL                = 271,
	PERF_COUNT_HW_HW_DATA_PRE_SCACHE_CANCEL         = 272,
	PERF_COUNT_HW_HW_FETCH_PRE_SCACHE_CANCEL                = 273,
	PERF_COUNT_HW_HW_PREFETCH_SCACHE_CANCEL         = 274,
	PERF_COUNT_HW_HW_LOAD_PREFETCH          = 275,
	PERF_COUNT_HW_HW_STORE_PREFETCH         = 276,
	PERF_COUNT_HW_HW_DATA_PREFETCH          = 277,
	PERF_COUNT_HW_HW_INST_PREFETCH          = 278,
	PERF_COUNT_HW_HW_PREFETCH               = 279,
	PERF_COUNT_HW_TAGGED_LOAD_PREFETCH              = 280,
	PERF_COUNT_HW_MISS_LOAD_PREFETCH                = 281,
	PERF_COUNT_HW_TAGGED_STORE_PREFETCH             = 282,
	PERF_COUNT_HW_MISS_STORE_PREFETCH               = 283,
	PERF_COUNT_HW_TAGGED_DATA_PREFETCH              = 284,
	PERF_COUNT_HW_MISS_DATA_PREFETCH                = 285,
	PERF_COUNT_HW_TAGGED_INST_PREFETCH              = 286,
	PERF_COUNT_HW_MISS_INST_PREFETCH                = 287,
	PERF_COUNT_HW_TAGGED_PREFETCH           = 288,
	PERF_COUNT_HW_MISS_PREFETCH             = 289,
	PERF_COUNT_HW_MISSQ_AC_LOAD_PREFETCH            = 290,
	PERF_COUNT_HW_MISSQ_AC_STORE_PREFETCH           = 291,
	PERF_COUNT_HW_MISSQ_AC_DATA_PREFETCH            = 292,
	PERF_COUNT_HW_MISSQ_AC_INST_PREFETCH            = 293,
	PERF_COUNT_HW_MISSQ_AC_PREFETCH         = 294,
	PERF_COUNT_HW_SCACHE_LOAD_PREFETCH              = 295,
	PERF_COUNT_HW_SCACHE_STORE_PREFETCH             = 296,
	PERF_COUNT_HW_SCACHE_DATA_PREFETCH              = 297,
	PERF_COUNT_HW_SCACHE_INST_PREFETCH              = 298,
	PERF_COUNT_HW_SCACHE_VALID_PREFETCH             = 299,
	PERF_COUNT_HW_PRE_RDY_LOAD_PREFETCH             = 300,
	PERF_COUNT_HW_PRE_RDY_STORE_PREFETCH            = 301,
	PERF_COUNT_HW_PRE_RDY_DATA_PREFETCH             = 302,
	PERF_COUNT_HW_PRE_RDY_INST_PREFETCH             = 303,
	PERF_COUNT_HW_PRE_RDY_PREFETCH          = 304,
	PERF_COUNT_HW_PRE_RDY_LOAD_REQUEST              = 305,
	PERF_COUNT_HW_PRE_RDY_STORE_REQUEST             = 306,
	PERF_COUNT_HW_PRE_RDY_DATA_REQUEST              = 307,
	PERF_COUNT_HW_PRE_RDY_INST_REQUEST              = 308,
	PERF_COUNT_HW_PRE_RDY_REQUEST           = 309,
	PERF_COUNT_HW_PRE_SCREF_HIT_LOAD_REQ            = 310,
	PERF_COUNT_HW_PRE_SCREF_HIT_STORE_REQ           = 311,
	PERF_COUNT_HW_PRE_SCREF_HIT_DATA_REQ            = 312,
	PERF_COUNT_HW_PRE_SCREF_HIT_INST_REQ            = 313,
	PERF_COUNT_HW_PRE_SCREF_HIT_PREFETCH_REQ                = 314,
	PERF_COUNT_HW_PRE_SCREF_HIT_LOAD_PREFETCH               = 315,
	PERF_COUNT_HW_PRE_SCREF_HIT_STORE_PREFETCH              = 316,
	PERF_COUNT_HW_PRE_SCREF_RDY_DATA_REQ            = 317,
	PERF_COUNT_HW_PRE_SCREF_RDY_INST_REQ            = 318,
	PERF_COUNT_HW_PRE_SCREF_RDY_PREFETCH            = 319,
	PERF_COUNT_HW_PRE_SCREF_MISS_LOAD               = 320,
	PERF_COUNT_HW_PRE_WAIT_HIT_LOAD_PREFETCH                = 321,
	PERF_COUNT_HW_PRE_WAIT_HIT_STORE_PREFETCH               = 322,
	PERF_COUNT_HW_PRE_WAIT_HIT_DATA_PREFETCH                = 323,
	PERF_COUNT_HW_PRE_WAIT_HIT_INST_PREFETCH                = 324,
	PERF_COUNT_HW_PRE_WAIT_HIT_PREFETCH             = 325,
	PERF_COUNT_HW_MISSQ_REPLACE_PRE_WAIT_PREFEETCH          = 326,
	PERF_COUNT_HW_MISSQ_REPLACE_PRE_RDY_PREFEETCH           = 327,
	PERF_COUNT_HW_PREFETCH_INV              = 328,
	PERF_COUNT_HW_LOAD_PREFETCH_OCCUPY              = 329,
	PERF_COUNT_HW_LOAD_PREFETCH_ISOCCUPY            = 330,
	PERF_COUNT_HW_STORE_PREFETCH_OCCUPY             = 331,
	PERF_COUNT_HW_STORE_PREFETCH_ISOCCUPY           = 332,
	PERF_COUNT_HW_DATA_PREFETCH_OCCUPY              = 333,
	PERF_COUNT_HW_DATA_PREFETCH_ISOCCUPY            = 334,
	PERF_COUNT_HW_INST_PREFETCH_OCCUPY              = 335,
	PERF_COUNT_HW_INST_PREFETCH_ISOCCUPY            = 336,
	PERF_COUNT_HW_LOAD_PRE_SCREF_PRE_RDY_HIT                = 337,
	PERF_COUNT_HW_STORE_PRE_SCREF_PRE_RDY_HIT               = 338,
	PERF_COUNT_HW_DATA_PRE_SCREF_PRE_RDY_HIT                = 339,
	PERF_COUNT_HW_INST_PRE_SCREF_PRE_RDY_HIT                = 340,
	PERF_COUNT_HW_PREFETCH_PRE_SCREF_PRE_RDY_HIT            = 341,

	/*loongson-3A1000 hardware events*/
	PERF_COUNT_HW_JUMP_INSTRUCTIONS     = 342,
	PERF_COUNT_HW_JR31_INSTRUCTIONS     = 343,
	PERF_COUNT_HW_ICACHE_MISSES     = 344,
	PERF_COUNT_HW_ALU1_ISSUED       = 345,
	PERF_COUNT_HW_MEM_ISSUED        = 346,
	PERF_COUNT_HW_FALU1_ISSUED      = 347,
	PERF_COUNT_HW_BHT_BRANCH_INSTRUCTIONS   = 348,
	PERF_COUNT_HW_MEM_READ          = 349,
	PERF_COUNT_HW_FQUEUE_FULL       = 350,
	PERF_COUNT_HW_ROQ_FULL          = 351,
	PERF_COUNT_HW_CP0_QUEUE_FULL        = 352,
	PERF_COUNT_HW_TLB_REFILL        = 353,
	PERF_COUNT_HW_EXCEPTION         = 354,
	PERF_COUNT_HW_INTERNAL_EXCEPTION    = 355,

	PERF_COUNT_HW_JR_MISPREDICTED       = 356,
	PERF_COUNT_HW_JR31_MISPREDICTED     = 357,
	PERF_COUNT_HW_DCACHE_MISSES     = 358,
	PERF_COUNT_HW_ALU2_ISSUED       = 359,
	PERF_COUNT_HW_FALU2_ISSUED      = 360,
	PERF_COUNT_HW_UNCACHED_ACCESS       = 361,
	PERF_COUNT_HW_BHT_MISPREDICTED      = 362,
	PERF_COUNT_HW_MEM_WRITE         = 363,
	PERF_COUNT_HW_FTQ_FULL          = 364,
	PERF_COUNT_HW_BRANCH_QUEUE_FULL     = 365,
	PERF_COUNT_HW_ITLB_MISSES       = 366,
	PERF_COUNT_HW_TOTAL_EXCEPTIONS      = 367,
	PERF_COUNT_HW_LOAD_SPECULATION_MISSES   = 368,
	PERF_COUNT_HW_CP0Q_FORWARD_VALID    = 369,
#endif
	PERF_COUNT_HW_MAX,			/* non-ABI */

};

/*
 * Generalized hardware cache events:
 *
 *       { L1-D, L1-I, LLC, ITLB, DTLB, BPU, NODE } x
 *       { read, write, prefetch } x
 *       { accesses, misses }
 */
enum perf_hw_cache_id {
	PERF_COUNT_HW_CACHE_L1D			= 0,
	PERF_COUNT_HW_CACHE_L1I			= 1,
	PERF_COUNT_HW_CACHE_LL			= 2,
	PERF_COUNT_HW_CACHE_DTLB		= 3,
	PERF_COUNT_HW_CACHE_ITLB		= 4,
	PERF_COUNT_HW_CACHE_BPU			= 5,
	PERF_COUNT_HW_CACHE_NODE		= 6,

	PERF_COUNT_HW_CACHE_MAX,		/* non-ABI */
};

enum perf_hw_cache_op_id {
	PERF_COUNT_HW_CACHE_OP_READ		= 0,
	PERF_COUNT_HW_CACHE_OP_WRITE		= 1,
	PERF_COUNT_HW_CACHE_OP_PREFETCH		= 2,

	PERF_COUNT_HW_CACHE_OP_MAX,		/* non-ABI */
};

enum perf_hw_cache_op_result_id {
	PERF_COUNT_HW_CACHE_RESULT_ACCESS	= 0,
	PERF_COUNT_HW_CACHE_RESULT_MISS		= 1,

	PERF_COUNT_HW_CACHE_RESULT_MAX,		/* non-ABI */
};

/*
 * Special "software" events provided by the kernel, even if the hardware
 * does not support performance events. These events measure various
 * physical and sw events of the kernel (and allow the profiling of them as
 * well):
 */
enum perf_sw_ids {
	PERF_COUNT_SW_CPU_CLOCK			= 0,
	PERF_COUNT_SW_TASK_CLOCK		= 1,
	PERF_COUNT_SW_PAGE_FAULTS		= 2,
	PERF_COUNT_SW_CONTEXT_SWITCHES		= 3,
	PERF_COUNT_SW_CPU_MIGRATIONS		= 4,
	PERF_COUNT_SW_PAGE_FAULTS_MIN		= 5,
	PERF_COUNT_SW_PAGE_FAULTS_MAJ		= 6,
	PERF_COUNT_SW_ALIGNMENT_FAULTS		= 7,
	PERF_COUNT_SW_EMULATION_FAULTS		= 8,

	PERF_COUNT_SW_MAX,			/* non-ABI */
};

/*
 * Bits that can be set in attr.sample_type to request information
 * in the overflow packets.
 */
enum perf_event_sample_format {
	PERF_SAMPLE_IP				= 1U << 0,
	PERF_SAMPLE_TID				= 1U << 1,
	PERF_SAMPLE_TIME			= 1U << 2,
	PERF_SAMPLE_ADDR			= 1U << 3,
	PERF_SAMPLE_READ			= 1U << 4,
	PERF_SAMPLE_CALLCHAIN			= 1U << 5,
	PERF_SAMPLE_ID				= 1U << 6,
	PERF_SAMPLE_CPU				= 1U << 7,
	PERF_SAMPLE_PERIOD			= 1U << 8,
	PERF_SAMPLE_STREAM_ID			= 1U << 9,
	PERF_SAMPLE_RAW				= 1U << 10,
	PERF_SAMPLE_BRANCH_STACK		= 1U << 11,
	PERF_SAMPLE_REGS_USER			= 1U << 12,
	PERF_SAMPLE_STACK_USER			= 1U << 13,
	PERF_SAMPLE_WEIGHT			= 1U << 14,
	PERF_SAMPLE_DATA_SRC			= 1U << 15,

	PERF_SAMPLE_MAX = 1U << 16,		/* non-ABI */
};

/*
 * values to program into branch_sample_type when PERF_SAMPLE_BRANCH is set
 *
 * If the user does not pass priv level information via branch_sample_type,
 * the kernel uses the event's priv level. Branch and event priv levels do
 * not have to match. Branch priv level is checked for permissions.
 *
 * The branch types can be combined, however BRANCH_ANY covers all types
 * of branches and therefore it supersedes all the other types.
 */
enum perf_branch_sample_type {
	PERF_SAMPLE_BRANCH_USER		= 1U << 0, /* user branches */
	PERF_SAMPLE_BRANCH_KERNEL	= 1U << 1, /* kernel branches */
	PERF_SAMPLE_BRANCH_HV		= 1U << 2, /* hypervisor branches */

	PERF_SAMPLE_BRANCH_ANY		= 1U << 3, /* any branch types */
	PERF_SAMPLE_BRANCH_ANY_CALL	= 1U << 4, /* any call branch */
	PERF_SAMPLE_BRANCH_ANY_RETURN	= 1U << 5, /* any return branch */
	PERF_SAMPLE_BRANCH_IND_CALL	= 1U << 6, /* indirect calls */

	PERF_SAMPLE_BRANCH_MAX		= 1U << 7, /* non-ABI */
};

#define PERF_SAMPLE_BRANCH_PLM_ALL \
	(PERF_SAMPLE_BRANCH_USER|\
	 PERF_SAMPLE_BRANCH_KERNEL|\
	 PERF_SAMPLE_BRANCH_HV)

/*
 * Values to determine ABI of the registers dump.
 */
enum perf_sample_regs_abi {
	PERF_SAMPLE_REGS_ABI_NONE	= 0,
	PERF_SAMPLE_REGS_ABI_32		= 1,
	PERF_SAMPLE_REGS_ABI_64		= 2,
};

/*
 * The format of the data returned by read() on a perf event fd,
 * as specified by attr.read_format:
 *
 * struct read_format {
 *	{ u64		value;
 *	  { u64		time_enabled; } && PERF_FORMAT_TOTAL_TIME_ENABLED
 *	  { u64		time_running; } && PERF_FORMAT_TOTAL_TIME_RUNNING
 *	  { u64		id;           } && PERF_FORMAT_ID
 *	} && !PERF_FORMAT_GROUP
 *
 *	{ u64		nr;
 *	  { u64		time_enabled; } && PERF_FORMAT_TOTAL_TIME_ENABLED
 *	  { u64		time_running; } && PERF_FORMAT_TOTAL_TIME_RUNNING
 *	  { u64		value;
 *	    { u64	id;           } && PERF_FORMAT_ID
 *	  }		cntr[nr];
 *	} && PERF_FORMAT_GROUP
 * };
 */
enum perf_event_read_format {
	PERF_FORMAT_TOTAL_TIME_ENABLED		= 1U << 0,
	PERF_FORMAT_TOTAL_TIME_RUNNING		= 1U << 1,
	PERF_FORMAT_ID				= 1U << 2,
	PERF_FORMAT_GROUP			= 1U << 3,

	PERF_FORMAT_MAX = 1U << 4,		/* non-ABI */
};

#define PERF_ATTR_SIZE_VER0	64	/* sizeof first published struct */
#define PERF_ATTR_SIZE_VER1	72	/* add: config2 */
#define PERF_ATTR_SIZE_VER2	80	/* add: branch_sample_type */
#define PERF_ATTR_SIZE_VER3	96	/* add: sample_regs_user */
					/* add: sample_stack_user */

/*
 * Hardware event_id to monitor via a performance monitoring event:
 */
struct perf_event_attr {

	/*
	 * Major type: hardware/software/tracepoint/etc.
	 */
	__u32			type;

	/*
	 * Size of the attr structure, for fwd/bwd compat.
	 */
	__u32			size;

	/*
	 * Type specific configuration information.
	 */
	__u64			config;

	union {
		__u64		sample_period;
		__u64		sample_freq;
	};

	__u64			sample_type;
	__u64			read_format;

	__u64			disabled       :  1, /* off by default        */
				inherit	       :  1, /* children inherit it   */
				pinned	       :  1, /* must always be on PMU */
				exclusive      :  1, /* only group on PMU     */
				exclude_user   :  1, /* don't count user      */
				exclude_kernel :  1, /* ditto kernel          */
				exclude_hv     :  1, /* ditto hypervisor      */
				exclude_idle   :  1, /* don't count when idle */
				mmap           :  1, /* include mmap data     */
				comm	       :  1, /* include comm data     */
				freq           :  1, /* use freq, not period  */
				inherit_stat   :  1, /* per task counts       */
				enable_on_exec :  1, /* next exec enables     */
				task           :  1, /* trace fork/exit       */
				watermark      :  1, /* wakeup_watermark      */
				/*
				 * precise_ip:
				 *
				 *  0 - SAMPLE_IP can have arbitrary skid
				 *  1 - SAMPLE_IP must have constant skid
				 *  2 - SAMPLE_IP requested to have 0 skid
				 *  3 - SAMPLE_IP must have 0 skid
				 *
				 *  See also PERF_RECORD_MISC_EXACT_IP
				 */
				precise_ip     :  2, /* skid constraint       */
				mmap_data      :  1, /* non-exec mmap data    */
				sample_id_all  :  1, /* sample_type all events */

				exclude_host   :  1, /* don't count in host   */
				exclude_guest  :  1, /* don't count in guest  */

				exclude_callchain_kernel : 1, /* exclude kernel callchains */
				exclude_callchain_user   : 1, /* exclude user callchains */

				__reserved_1   : 41;

	union {
		__u32		wakeup_events;	  /* wakeup every n events */
		__u32		wakeup_watermark; /* bytes before wakeup   */
	};

	__u32			bp_type;
	union {
		__u64		bp_addr;
		__u64		config1; /* extension of config */
	};
	union {
		__u64		bp_len;
		__u64		config2; /* extension of config1 */
	};
	__u64	branch_sample_type; /* enum perf_branch_sample_type */

	/*
	 * Defines set of user regs to dump on samples.
	 * See asm/perf_regs.h for details.
	 */
	__u64	sample_regs_user;

	/*
	 * Defines size of the user stack to dump on samples.
	 */
	__u32	sample_stack_user;

	/* Align to u64. */
	__u32	__reserved_2;
};

#define perf_flags(attr)	(*(&(attr)->read_format + 1))

/*
 * Ioctls that can be done on a perf event fd:
 */
#define PERF_EVENT_IOC_ENABLE		_IO ('$', 0)
#define PERF_EVENT_IOC_DISABLE		_IO ('$', 1)
#define PERF_EVENT_IOC_REFRESH		_IO ('$', 2)
#define PERF_EVENT_IOC_RESET		_IO ('$', 3)
#define PERF_EVENT_IOC_PERIOD		_IOW('$', 4, __u64)
#define PERF_EVENT_IOC_SET_OUTPUT	_IO ('$', 5)
#define PERF_EVENT_IOC_SET_FILTER	_IOW('$', 6, char *)

enum perf_event_ioc_flags {
	PERF_IOC_FLAG_GROUP		= 1U << 0,
};

/*
 * Structure of the page that can be mapped via mmap
 */
struct perf_event_mmap_page {
	__u32	version;		/* version number of this structure */
	__u32	compat_version;		/* lowest version this is compat with */

	/*
	 * Bits needed to read the hw events in user-space.
	 *
	 *   u32 seq, time_mult, time_shift, idx, width;
	 *   u64 count, enabled, running;
	 *   u64 cyc, time_offset;
	 *   s64 pmc = 0;
	 *
	 *   do {
	 *     seq = pc->lock;
	 *     barrier()
	 *
	 *     enabled = pc->time_enabled;
	 *     running = pc->time_running;
	 *
	 *     if (pc->cap_usr_time && enabled != running) {
	 *       cyc = rdtsc();
	 *       time_offset = pc->time_offset;
	 *       time_mult   = pc->time_mult;
	 *       time_shift  = pc->time_shift;
	 *     }
	 *
	 *     idx = pc->index;
	 *     count = pc->offset;
	 *     if (pc->cap_usr_rdpmc && idx) {
	 *       width = pc->pmc_width;
	 *       pmc = rdpmc(idx - 1);
	 *     }
	 *
	 *     barrier();
	 *   } while (pc->lock != seq);
	 *
	 * NOTE: for obvious reason this only works on self-monitoring
	 *       processes.
	 */
	__u32	lock;			/* seqlock for synchronization */
	__u32	index;			/* hardware event identifier */
	__s64	offset;			/* add to hardware event value */
	__u64	time_enabled;		/* time event active */
	__u64	time_running;		/* time event on cpu */
	union {
		__u64	capabilities;
		__u64	cap_usr_time  : 1,
			cap_usr_rdpmc : 1,
			cap_____res   : 62;
	};

	/*
	 * If cap_usr_rdpmc this field provides the bit-width of the value
	 * read using the rdpmc() or equivalent instruction. This can be used
	 * to sign extend the result like:
	 *
	 *   pmc <<= 64 - width;
	 *   pmc >>= 64 - width; // signed shift right
	 *   count += pmc;
	 */
	__u16	pmc_width;

	/*
	 * If cap_usr_time the below fields can be used to compute the time
	 * delta since time_enabled (in ns) using rdtsc or similar.
	 *
	 *   u64 quot, rem;
	 *   u64 delta;
	 *
	 *   quot = (cyc >> time_shift);
	 *   rem = cyc & ((1 << time_shift) - 1);
	 *   delta = time_offset + quot * time_mult +
	 *              ((rem * time_mult) >> time_shift);
	 *
	 * Where time_offset,time_mult,time_shift and cyc are read in the
	 * seqcount loop described above. This delta can then be added to
	 * enabled and possible running (if idx), improving the scaling:
	 *
	 *   enabled += delta;
	 *   if (idx)
	 *     running += delta;
	 *
	 *   quot = count / running;
	 *   rem  = count % running;
	 *   count = quot * enabled + (rem * enabled) / running;
	 */
	__u16	time_shift;
	__u32	time_mult;
	__u64	time_offset;

		/*
		 * Hole for extension of the self monitor capabilities
		 */

	__u64	__reserved[120];	/* align to 1k */

	/*
	 * Control data for the mmap() data buffer.
	 *
	 * User-space reading the @data_head value should issue an smp_rmb(),
	 * after reading this value.
	 *
	 * When the mapping is PROT_WRITE the @data_tail value should be
	 * written by userspace to reflect the last read data, after issueing
	 * an smp_mb() to separate the data read from the ->data_tail store.
	 * In this case the kernel will not over-write unread data.
	 *
	 * See perf_output_put_handle() for the data ordering.
	 */
	__u64   data_head;		/* head in the data section */
	__u64	data_tail;		/* user-space written tail */
};

#define PERF_RECORD_MISC_CPUMODE_MASK		(7 << 0)
#define PERF_RECORD_MISC_CPUMODE_UNKNOWN	(0 << 0)
#define PERF_RECORD_MISC_KERNEL			(1 << 0)
#define PERF_RECORD_MISC_USER			(2 << 0)
#define PERF_RECORD_MISC_HYPERVISOR		(3 << 0)
#define PERF_RECORD_MISC_GUEST_KERNEL		(4 << 0)
#define PERF_RECORD_MISC_GUEST_USER		(5 << 0)

#define PERF_RECORD_MISC_MMAP_DATA		(1 << 13)
/*
 * Indicates that the content of PERF_SAMPLE_IP points to
 * the actual instruction that triggered the event. See also
 * perf_event_attr::precise_ip.
 */
#define PERF_RECORD_MISC_EXACT_IP		(1 << 14)
/*
 * Reserve the last bit to indicate some extended misc field
 */
#define PERF_RECORD_MISC_EXT_RESERVED		(1 << 15)

struct perf_event_header {
	__u32	type;
	__u16	misc;
	__u16	size;
};

enum perf_event_type {

	/*
	 * If perf_event_attr.sample_id_all is set then all event types will
	 * have the sample_type selected fields related to where/when
	 * (identity) an event took place (TID, TIME, ID, CPU, STREAM_ID)
	 * described in PERF_RECORD_SAMPLE below, it will be stashed just after
	 * the perf_event_header and the fields already present for the existing
	 * fields, i.e. at the end of the payload. That way a newer perf.data
	 * file will be supported by older perf tools, with these new optional
	 * fields being ignored.
	 *
	 * The MMAP events record the PROT_EXEC mappings so that we can
	 * correlate userspace IPs to code. They have the following structure:
	 *
	 * struct {
	 *	struct perf_event_header	header;
	 *
	 *	u32				pid, tid;
	 *	u64				addr;
	 *	u64				len;
	 *	u64				pgoff;
	 *	char				filename[];
	 * };
	 */
	PERF_RECORD_MMAP			= 1,

	/*
	 * struct {
	 *	struct perf_event_header	header;
	 *	u64				id;
	 *	u64				lost;
	 * };
	 */
	PERF_RECORD_LOST			= 2,

	/*
	 * struct {
	 *	struct perf_event_header	header;
	 *
	 *	u32				pid, tid;
	 *	char				comm[];
	 * };
	 */
	PERF_RECORD_COMM			= 3,

	/*
	 * struct {
	 *	struct perf_event_header	header;
	 *	u32				pid, ppid;
	 *	u32				tid, ptid;
	 *	u64				time;
	 * };
	 */
	PERF_RECORD_EXIT			= 4,

	/*
	 * struct {
	 *	struct perf_event_header	header;
	 *	u64				time;
	 *	u64				id;
	 *	u64				stream_id;
	 * };
	 */
	PERF_RECORD_THROTTLE			= 5,
	PERF_RECORD_UNTHROTTLE			= 6,

	/*
	 * struct {
	 *	struct perf_event_header	header;
	 *	u32				pid, ppid;
	 *	u32				tid, ptid;
	 *	u64				time;
	 * };
	 */
	PERF_RECORD_FORK			= 7,

	/*
	 * struct {
	 *	struct perf_event_header	header;
	 *	u32				pid, tid;
	 *
	 *	struct read_format		values;
	 * };
	 */
	PERF_RECORD_READ			= 8,

	/*
	 * struct {
	 *	struct perf_event_header	header;
	 *
	 *	{ u64			ip;	  } && PERF_SAMPLE_IP
	 *	{ u32			pid, tid; } && PERF_SAMPLE_TID
	 *	{ u64			time;     } && PERF_SAMPLE_TIME
	 *	{ u64			addr;     } && PERF_SAMPLE_ADDR
	 *	{ u64			id;	  } && PERF_SAMPLE_ID
	 *	{ u64			stream_id;} && PERF_SAMPLE_STREAM_ID
	 *	{ u32			cpu, res; } && PERF_SAMPLE_CPU
	 *	{ u64			period;   } && PERF_SAMPLE_PERIOD
	 *
	 *	{ struct read_format	values;	  } && PERF_SAMPLE_READ
	 *
	 *	{ u64			nr,
	 *	  u64			ips[nr];  } && PERF_SAMPLE_CALLCHAIN
	 *
	 *	#
	 *	# The RAW record below is opaque data wrt the ABI
	 *	#
	 *	# That is, the ABI doesn't make any promises wrt to
	 *	# the stability of its content, it may vary depending
	 *	# on event, hardware, kernel version and phase of
	 *	# the moon.
	 *	#
	 *	# In other words, PERF_SAMPLE_RAW contents are not an ABI.
	 *	#
	 *
	 *	{ u32			size;
	 *	  char                  data[size];}&& PERF_SAMPLE_RAW
	 *
	 *	{ u64                   nr;
	 *        { u64 from, to, flags } lbr[nr];} && PERF_SAMPLE_BRANCH_STACK
	 *
	 * 	{ u64			abi; # enum perf_sample_regs_abi
	 * 	  u64			regs[weight(mask)]; } && PERF_SAMPLE_REGS_USER
	 *
	 * 	{ u64			size;
	 * 	  char			data[size];
	 * 	  u64			dyn_size; } && PERF_SAMPLE_STACK_USER
	 *
	 *	{ u64			weight;   } && PERF_SAMPLE_WEIGHT
	 *	{ u64			data_src;     } && PERF_SAMPLE_DATA_SRC
	 * };
	 */
	PERF_RECORD_SAMPLE			= 9,

	PERF_RECORD_MAX,			/* non-ABI */
};

#define PERF_MAX_STACK_DEPTH		127

enum perf_callchain_context {
	PERF_CONTEXT_HV			= (__u64)-32,
	PERF_CONTEXT_KERNEL		= (__u64)-128,
	PERF_CONTEXT_USER		= (__u64)-512,

	PERF_CONTEXT_GUEST		= (__u64)-2048,
	PERF_CONTEXT_GUEST_KERNEL	= (__u64)-2176,
	PERF_CONTEXT_GUEST_USER		= (__u64)-2560,

	PERF_CONTEXT_MAX		= (__u64)-4095,
};

#define PERF_FLAG_FD_NO_GROUP		(1U << 0)
#define PERF_FLAG_FD_OUTPUT		(1U << 1)
#define PERF_FLAG_PID_CGROUP		(1U << 2) /* pid=cgroup id, per-cpu mode only */

union perf_mem_data_src {
	__u64 val;
	struct {
		__u64   mem_op:5,	/* type of opcode */
			mem_lvl:14,	/* memory hierarchy level */
			mem_snoop:5,	/* snoop mode */
			mem_lock:2,	/* lock instr */
			mem_dtlb:7,	/* tlb access */
			mem_rsvd:31;
	};
};

/* type of opcode (load/store/prefetch,code) */
#define PERF_MEM_OP_NA		0x01 /* not available */
#define PERF_MEM_OP_LOAD	0x02 /* load instruction */
#define PERF_MEM_OP_STORE	0x04 /* store instruction */
#define PERF_MEM_OP_PFETCH	0x08 /* prefetch */
#define PERF_MEM_OP_EXEC	0x10 /* code (execution) */
#define PERF_MEM_OP_SHIFT	0

/* memory hierarchy (memory level, hit or miss) */
#define PERF_MEM_LVL_NA		0x01  /* not available */
#define PERF_MEM_LVL_HIT	0x02  /* hit level */
#define PERF_MEM_LVL_MISS	0x04  /* miss level  */
#define PERF_MEM_LVL_L1		0x08  /* L1 */
#define PERF_MEM_LVL_LFB	0x10  /* Line Fill Buffer */
#define PERF_MEM_LVL_L2		0x20  /* L2 */
#define PERF_MEM_LVL_L3		0x40  /* L3 */
#define PERF_MEM_LVL_LOC_RAM	0x80  /* Local DRAM */
#define PERF_MEM_LVL_REM_RAM1	0x100 /* Remote DRAM (1 hop) */
#define PERF_MEM_LVL_REM_RAM2	0x200 /* Remote DRAM (2 hops) */
#define PERF_MEM_LVL_REM_CCE1	0x400 /* Remote Cache (1 hop) */
#define PERF_MEM_LVL_REM_CCE2	0x800 /* Remote Cache (2 hops) */
#define PERF_MEM_LVL_IO		0x1000 /* I/O memory */
#define PERF_MEM_LVL_UNC	0x2000 /* Uncached memory */
#define PERF_MEM_LVL_SHIFT	5

/* snoop mode */
#define PERF_MEM_SNOOP_NA	0x01 /* not available */
#define PERF_MEM_SNOOP_NONE	0x02 /* no snoop */
#define PERF_MEM_SNOOP_HIT	0x04 /* snoop hit */
#define PERF_MEM_SNOOP_MISS	0x08 /* snoop miss */
#define PERF_MEM_SNOOP_HITM	0x10 /* snoop hit modified */
#define PERF_MEM_SNOOP_SHIFT	19

/* locked instruction */
#define PERF_MEM_LOCK_NA	0x01 /* not available */
#define PERF_MEM_LOCK_LOCKED	0x02 /* locked transaction */
#define PERF_MEM_LOCK_SHIFT	24

/* TLB access */
#define PERF_MEM_TLB_NA		0x01 /* not available */
#define PERF_MEM_TLB_HIT	0x02 /* hit level */
#define PERF_MEM_TLB_MISS	0x04 /* miss level */
#define PERF_MEM_TLB_L1		0x08 /* L1 */
#define PERF_MEM_TLB_L2		0x10 /* L2 */
#define PERF_MEM_TLB_WK		0x20 /* Hardware Walker*/
#define PERF_MEM_TLB_OS		0x40 /* OS fault handler */
#define PERF_MEM_TLB_SHIFT	26

#define PERF_MEM_S(a, s) \
	(((u64)PERF_MEM_##a##_##s) << PERF_MEM_##a##_SHIFT)

#endif /* _UAPI_LINUX_PERF_EVENT_H */

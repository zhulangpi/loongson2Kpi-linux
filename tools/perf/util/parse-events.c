#include <linux/hw_breakpoint.h>
#include "util.h"
#include "../perf.h"
#include "evlist.h"
#include "evsel.h"
#include "parse-options.h"
#include "parse-events.h"
#include "exec_cmd.h"
#include "string.h"
#include "symbol.h"
#include "cache.h"
#include "header.h"
#include <lk/debugfs.h>
#include "parse-events-bison.h"
#define YY_EXTRA_TYPE int
#include "parse-events-flex.h"
#include "pmu.h"

#define MAX_NAME_LEN 100

struct event_symbol {
	const char	*symbol;
	const char	*alias;
};

#ifdef PARSER_DEBUG
extern int parse_events_debug;
#endif
int parse_events_parse(void *data, void *scanner);

static struct event_symbol event_symbols_hw[PERF_COUNT_HW_MAX] = {
	[PERF_COUNT_HW_CPU_CYCLES] = {
		.symbol = "cpu-cycles",
		.alias  = "cycles",
	},
	[PERF_COUNT_HW_INSTRUCTIONS] = {
		.symbol = "instructions",
		.alias  = "",
	},
	[PERF_COUNT_HW_CACHE_REFERENCES] = {
		.symbol = "cache-references",
		.alias  = "",
	},
	[PERF_COUNT_HW_CACHE_MISSES] = {
		.symbol = "cache-misses",
		.alias  = "",
	},
	[PERF_COUNT_HW_BRANCH_INSTRUCTIONS] = {
		.symbol = "branch-instructions",
		.alias  = "branches",
	},
	[PERF_COUNT_HW_BRANCH_MISSES] = {
		.symbol = "branch-misses",
		.alias  = "",
	},
	[PERF_COUNT_HW_BUS_CYCLES] = {
		.symbol = "bus-cycles",
		.alias  = "",
	},
	[PERF_COUNT_HW_STALLED_CYCLES_FRONTEND] = {
		.symbol = "stalled-cycles-frontend",
		.alias  = "idle-cycles-frontend",
	},
	[PERF_COUNT_HW_STALLED_CYCLES_BACKEND] = {
		.symbol = "stalled-cycles-backend",
		.alias  = "idle-cycles-backend",
	},
	[PERF_COUNT_HW_REF_CPU_CYCLES] = {
		.symbol = "ref-cycles",
		.alias  = "",
	},


/*loongson3A2000 hardware events*/
#if defined(CONFIG_CPU_LOONGSON3_GS464E)
    /*fetch*/
		[PERF_COUNT_HW_INSTQ_EMPTY] = {
	      	 	 .symbol = "Inst-queue-empty",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_INSTQ_INSTRUCTIONS] = {
	      	 	 .symbol = "Inst-queue-instructions",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_INSTQ_1] = {
	      	 	 .symbol = "Inst-queue-1",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_INSTQ_2] = {
	      	 	 .symbol = "Inst-queue-2",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_INSTQ_3] = {
	      	 	 .symbol = "Inst-queue-3",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_INSTQ_4] = {
	      	 	 .symbol = "Inst-queue-4",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_INSTQ_5] = {
	      	 	 .symbol = "Inst-queue-5",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_INSTQ_6] = {
	      	 	 .symbol = "Inst-queue-6",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_INSTQ_7] = {
	      	 	 .symbol = "Inst-queue-7",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_INSTQ_8] = {
	      	 	 .symbol = "Inst-queue-8",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_INSTQ_LESSTHAN_8] = {
	      	 	 .symbol = "Inst-queue-less-than-8",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_INSTQ_FULL] = {
	      	 	 .symbol = "Inst-queue-full",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_DECODE_INST] = {
	      	 	 .symbol = "decode-instructions",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_LOOP_BUFFER_INST] = {
	      	 	 .symbol = "Loop-buffer-instructions",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_LOOP_FIND] = {
	      	 	 .symbol = "Loop-find",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_LOOP_TRIGGER] = {
	      	 	 .symbol = "Loop-trigger",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_DECODE_BRANCH_0] = {
	      	 	 .symbol = "Decode-branch-0",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_DECODE_BRANCH_1] = {
	      	 	 .symbol = "Decode-branch-1",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_DECODE_BRANCH_2] = {
	      	 	 .symbol = "Decode-branch-2",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_ICACHE_MISSES_BLOCK] = {
	      	 	 .symbol = "Icache-misses",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_BRBTB_TAKEN_BRANCH_MISSES] = {
	      	 	 .symbol = "BrBTB-taken-branch-misses",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_ICACHE_REPLACE] = {
	      	 	 .symbol = "Icache-replace/refill",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_ITLB_MISS_TLB_HIT] = {
	      	 	 .symbol = "Itlb-miss-tlb-hit",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_ITLB_FLUSHED] = {
	      	 	 .symbol = "Itlb-flushed",
	       	 	 .alias  = "",
		},
    /*rmap*/
		[PERF_COUNT_HW_RESOURCE_ALLOC_BLOCKED] = {
    	  	 	 .symbol = "Resource-alloc-blocked",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_GR_BLOCKED] = {
    	  	 	 .symbol = "GR-blocked",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_GR_PSEUDO_BLOCKED] = {
    	  	 	 .symbol = "GR-pseudo-blocked",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FR_BLOCKED] = {
    	  	 	 .symbol = "FR-blocked",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FR_PSEUDO_BLOCKED] = {
    	  	 	 .symbol = "FR-pseudo-blocked",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FCR_BLOCKED] = {
    	  	 	 .symbol = "FCR-blocked",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FCR_PSEUDO_BLOCKED] = {
    	  	 	 .symbol = "FCR-pseudo-blocked",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_ACC_BLOCKED] = {
    	  	 	 .symbol = "ACC-blocked",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_ACC_PSEUDO_BLOCKED] = {
    	  	 	 .symbol = "ACC-pseudo-blocked",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_DSPCTRL_BLOCKED] = {
    	  	 	 .symbol = "DSPCTRL-blocked",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_DSPCTRL_PSEUDO_BLOCKED] = {
    	  	 	 .symbol = "DSPCTRL-pseudo-blocked",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_BRQ_BLOCKED] = {
    	  	 	 .symbol = "BRQ-blocked",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_BRQ_PSEUDO_BLOCKED] = {
    	  	 	 .symbol = "BRQ-pseudo-blocked",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FXQ_BLOCKED] = {
    	  	 	 .symbol = "FXQ-blocked",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FXQ_PSEUDO_BLOCKED] = {
    	  	 	 .symbol = "FXQ-pseudo-blocked",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FTQ_BLOCKED] = {
    	  	 	 .symbol = "FTQ-blocked",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FTQ_PSEUDO_BLOCKED] = {
    	  	 	 .symbol = "FTQ-pseudo-blocked",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MMQ_BLOCKED] = {
    	  	 	 .symbol = "MMQ-blocked",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MMQ_PSEUDO_BLOCKED] = {
    	  	 	 .symbol = "MMQ-pseudo-blocked",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_CP0Q_BLOCKED] = {
    	  	 	 .symbol = "CP0Q-blocked",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_CP0Q_PSEUDO_BLOCKED] = {
    	  	 	 .symbol = "CP0Q-pseudo-blocked",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_ROQ_BLOCKED] = {
    	  	 	 .symbol = "ROQ-blocked",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_NOP_INST] = {
    	  	 	 .symbol = "Nop-inst",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_REGMAP_ISSUED] = {
    	  	 	 .symbol = "Regmap-issued",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_EXCEPTIONS] = {
    	  	 	 .symbol = "Exceptions",
    	   	 	 .alias  = "",
		},
		[PERF_COUNT_HW_BRANCH_MISSES_OVERHEAD] = {
    	  	 	 .symbol = "Branch-misses-overhead",
    	   	 	 .alias  = "",
		},
    /*roq*/
		[PERF_COUNT_HW_ALU_COMMITTED] = {
	      	 	 .symbol = "Alu-committed",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FALU_COMMITTED] = {
	      	 	 .symbol = "Falu-committed",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MEMORY_SWAP_COMMITTED] = {
	      	 	 .symbol = "Memory-swap-committed",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_LOAD_COMMITTED] = {
	      	 	 .symbol = "Load-committed",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_STORE_COMMITTED] = {
	      	 	 .symbol = "Store-committed",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_LL_COMMITTED] = {
	      	 	 .symbol = "LL-committed",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SC_COMMITTED] = {
	      	 	 .symbol = "SC-committed",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_UNALIGNED_LOAD_COMMITTED] = {
	      	 	 .symbol = "Unaligned-load-committed",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_UNALIGNED_STORE_COMMITTED] = {
	      	 	 .symbol = "Unaligned-store-committed",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_EXCEPTIONS_AND_INTERRUPTS] = {
	      	 	 .symbol = "Exceptions-and-interrupts",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_INTERRUPTS] = {
	      	 	 .symbol = "Interrupts",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_ROQ_INTERRUPT] = {
	      	 	 .symbol = "ROQ-interrupt",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_ROQ_INTERRUPT_INST] = {
	      	 	 .symbol = "ROQ-interrupt-inst",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VM_EXCEPTIONS] = {
	      	 	 .symbol = "Vm-exceptions",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_ADDRESS_FAULT_EXCEPTIONS] = {
	      	 	 .symbol = "Address-fault-exceptions",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_TLB_EXCEPTIONS] = {
	      	 	 .symbol = "Tlb-exceptions",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_TLB_REFILL_EXCEPTIONS] = {
	      	 	 .symbol = "Tlb-refill-exceptions",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_TLB_REFILL_HANDLE_TIME] = {
	      	 	 .symbol = "Tlb-refill-handle-time",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_JUMP_REGISTER] = {
	      	 	 .symbol = "Jump-register",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_JUMP_AND_LINK] = {
	      	 	 .symbol = "Jump-and-link",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_BRANCH_AND_LINK] = {
	      	 	 .symbol = "Branch-and-link",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_BHT_BRANCH] = {
	      	 	 .symbol = "Bht-branch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_LIKELY_BRANCH] = {
	      	 	 .symbol = "Likely-branch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_NOT_TAKEN_BRANCH] = {
	      	 	 .symbol = "Not-taken-branch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_TAKEN_BRANCH] = {
	      	 	 .symbol = "Taken-branch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_JUMP_REGISTER_MISSES] = {
	      	 	 .symbol = "Jump-register-misses",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_JUMP_AND_LINK_MISSES] = {
	      	 	 .symbol = "Jump-and-link-misses",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_BRANCH_AND_LINK_MISSES] = {
	      	 	 .symbol = "Branch-and-link-misses",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_BHT_BRANCH_MISSES] = {
	      	 	 .symbol = "Bht-branch-misses",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_LIKELY_BRANCH_MISSES] = {
	      	 	 .symbol = "Likely-branch-misses",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_NOT_TAKEN_MISSES] = {
	      	 	 .symbol = "Not-taken-misses",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_TAKEN_MISSES] = {
	      	 	 .symbol = "Taken-misses",
	       	 	 .alias  = "",
		},
    /*fix*/
		[PERF_COUNT_HW_FXQ_NO_ISSUE] = {
	      	 	 .symbol = "Fxq-no-issue",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FXQ_ISSUE_OPERAND] = {
	      	 	 .symbol = "Fxq-issue-operand",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FXQ_FU0_OPERAND] = {
	      	 	 .symbol = "Fxq-fu0-operand",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FXQ_FU1_OPERAND] = {
	      	 	 .symbol = "Fxq-fu1-operand",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FU0_FIXED_MUL] = {
	      	 	 .symbol = "Fu0-fixed-mul",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FU0_FIXED_DIV] = {
	      	 	 .symbol = "Fu0-fixed-div",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FU1_FIXED_MUL] = {
	      	 	 .symbol = "Fu1-fixed-mul",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FU1_FIXED_DIV] = {
	      	 	 .symbol = "Fu1-fixed-div",
	       	 	 .alias  = "",
		},
    /*float*/
		[PERF_COUNT_HW_FTQ_NO_ISSUE] = {
	      	 	 .symbol = "Ftq-no-issue",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FTQ_ISSUE_OPERAND] = {
	      	 	 .symbol = "Ftq-issue-operand",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FTQ_FU3_OPERAND] = {
	      	 	 .symbol = "Ftq-fu3-operand",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FTQ_FU4_OPERAND] = {
	      	 	 .symbol = "Ftq-fu4-operand",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FU3_EMPTY_FU4_FULL] = {
	      	 	 .symbol = "Fu3-empty-fu4-full",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FU4_EMPTY_FU3_FULL] = {
	      	 	 .symbol = "Fu4-empty-fu3-full",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SCALAR_FLOAT_OPERAND] = {
	      	 	 .symbol = "Scalar-float-operand",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_GS_ALU_INST] = {
	      	 	 .symbol = "Gs-alu-inst",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FIXED_VECTOR_ISSUE_64] = {
	      	 	 .symbol = "Fixed-vector-issue-64",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VECTOR_ISSUE_128] = {
	      	 	 .symbol = "Vector-issue-128",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FIXED_VECTOR_ISSUE_128] = {
	      	 	 .symbol = "Fixed-vector-issue-128",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FLOAT_VECTOR_ISSUE_128] = {
	      	 	 .symbol = "Float-vector-issue-128",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VECTOR_ISSUE_256] = {
	      	 	 .symbol = "Vector-issue-256",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FIXED_VECTOR_ISSUE_256] = {
	      	 	 .symbol = "Fixed-vector-issue-256",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FLOAT_VECTOR_ISSUE_256] = {
	      	 	 .symbol = "Float-vector-issue-256",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FU3_VECTOR_FIXED_DIV] = {
	      	 	 .symbol = "Fu3-vector-fixed-div",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FU3_FLOAT_DIV_SQRT] = {
	      	 	 .symbol = "Fu3-float-div-sqrt",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FU4_VECTOR_FIXED_DIV] = {
	      	 	 .symbol = "Fu4-vector-fixed-div",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FU4_FLOAT_DIV_SQRT] = {
	      	 	 .symbol = "Fu4-float-div-sqrt",
	       	 	 .alias  = "",
		},
    /*memory*/
		[PERF_COUNT_HW_MMQ_NO_ISSUE] = {
	      	 	 .symbol = "Mmq-no-issue",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MMQ_ISSUE_OPERAND] = {
	      	 	 .symbol = "Mmq-issue-operand",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MMQ_FU2_INST] = {
	      	 	 .symbol = "Mmq-fu2-inst",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MMQ_FU5_INST] = {
	      	 	 .symbol = "Mmq-fu5-inst",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_LOAD_ISSUE] = {
	      	 	 .symbol = "Load-issue",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_STORE_ISSUE] = {
	      	 	 .symbol = "Store-issue",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SRC_FLOAT_MEM_INST] = {
	      	 	 .symbol = "Src-float-mem-inst",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VECTOR_MEM_ISSUE] = {
	      	 	 .symbol = "Vector-mem-issue",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FIX_FLOAT_SHIFT_ISSUE] = {
	      	 	 .symbol = "Fix-float-shift-issue",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_WAIT_FIRST_BLOCK_CYCLES] = {
	      	 	 .symbol = "Wait-first-block-cycles",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SYNC_BLOCK_CYCLES] = {
	      	 	 .symbol = "Sync-block-cycles",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_STALL_ISSUE_BLOCK_CYCLES] = {
	      	 	 .symbol = "Stall-issue-block-cycles",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SOFTWARE_PREFETCH_TOTAL] = {
	      	 	 .symbol = "Software-prefetch-total",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_DMEMREF_BLOCK_DCACHEWRITE] = {
	      	 	 .symbol = "Dmemref-block-dcachewrite",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_DMEMREF_BANK_CLASH] = {
	      	 	 .symbol = "Dmemref-bank-clash",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_REFILL_BLOCK_DMEMREF] = {
	      	 	 .symbol = "Refill-block-dmemref",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_DCACHEWRITE_NO_CANCEL] = {
	      	 	 .symbol = "Dcachewrite-no-cancel",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_DCACHEWRITE0_AND_1_VALID] = {
	      	 	 .symbol = "Dcachewrite0-and-1-valid",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SC_WRITE_DCACHE] = {
	      	 	 .symbol = "Sc-write-dcache",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_STORE_DCACHE_MISS] = {
	      	 	 .symbol = "Store-dcache-miss",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_STORE_DCACHE_SHARED_MISS] = {
	      	 	 .symbol = "Store-dcache-shared-miss",
	       	 	 .alias  = "",
		},
    /*cache2mem*/
		[PERF_COUNT_HW_STORE_DCACHE_HIT] = {
	      	 	 .symbol = "Store-dcache-hit",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_LOAD_HIT] = {
	      	 	 .symbol = "Load-hit",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FWDBUS2] = {
	      	 	 .symbol = "Fwdbus2",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FWDBUS5] = {
	      	 	 .symbol = "Fwdbus5",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FWDBUS_TOTAL] = {
	      	 	 .symbol = "Fwdbus-total",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_DWAITSTORE] = {
	      	 	 .symbol = "Dwaitstore",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISPEC] = {
	      	 	 .symbol = "Mispec",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_DCACHEWRITE_CANCEL] = {
	      	 	 .symbol = "Dcachewrite-cancel",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_CP0Q_DMEMREAD] = {
	      	 	 .symbol = "Cp0q-dmemread",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_CP0Q_DUNCACHE] = {
	      	 	 .symbol = "Cp0q-duncache",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_RESBUS2_OCCUPY_RESBUS5] = {
	      	 	 .symbol = "Resbus2-occupy-resbus5",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SW_PREFETCH_L1DCACHE_HIT] = {
	      	 	 .symbol = "Sw-prefetch-l1dcache-hit",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_STORE_SW_PREFETCH_L1DCACHE_HIT] = {
	      	 	 .symbol = "Store-sw-prefetch-l1dcache-hit",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_STORE_SW_PREFETCH_L1DCACHE_MISS] = {
	      	 	 .symbol = "Store-sw-prefetch-l1dcache-miss",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_LOAD_SW_PREFETCH_L1DCACHE_HIT] = {
	      	 	 .symbol = "Load-sw-prefetch-l1dcache-hit",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_LOAD_SW_PREFETCH_L1DCACHE_MISS] = {
	      	 	 .symbol = "Load-sw-prefetch-l1dcache-miss",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_STORE_L1DCACHE_MISS_SHARE_STATE] = {
	      	 	 .symbol = "Store-l1dcache-miss-share-state",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SPECFWDBUS2] = {
	      	 	 .symbol = "Specfwdbus2",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SPECFWDBUS5] = {
	      	 	 .symbol = "Specfwdbus5",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SPECFWDBUS2_TOTAL] = {
	      	 	 .symbol = "Specfwdbus-total",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_DATA_LOAD_VCACHE_ACCESS_REQ] = {
	      	 	 .symbol = "Data-load-vcache-access-req",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_DATA_STORE_VCACHE_ACCESS_REQ] = {
	      	 	 .symbol = "Data-store-vcache-access-req",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_DATA_VCACHE_ACCESS_REQ] = {
	      	 	 .symbol = "Data-vcache-access-req",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_INST_VCACHE_ACCESS_REQ] = {
	      	 	 .symbol = "Inst-vcache-access-req",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VCACHE_ACCESS] = {
	      	 	 .symbol = "Vcache-access",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SW_PREFETCH_ACCESS_VCACHE] = {
	      	 	 .symbol = "Sw-prefetch-access-vcache",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VCACHE_LOAD_HIT] = {
	      	 	 .symbol = "Vcache-load-hit",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VCACHE_STORE_HIT] = {
	      	 	 .symbol = "Vcache-store-hit",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VCACHE_DATA_HIT] = {
	      	 	 .symbol = "Vcache-data-hit",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VCACHE_INST_HIT] = {
	      	 	 .symbol = "Vcache-inst-hit",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VCACHE_HIT] = {
	      	 	 .symbol = "Vcache-hit",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VCACHE_SW_PREFETCH_HIT] = {
	      	 	 .symbol = "Vcache-sw-prefetch-hit",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VCACHE_LOAD_MISS] = {
	      	 	 .symbol = "Vcache-load-miss",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VCACHE_STORE_MISS] = {
	      	 	 .symbol = "Vcache-store-miss",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VCACHE_DATA_MISS] = {
	      	 	 .symbol = "Vcache-data-miss",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VCACHE_INST_MISS] = {
	      	 	 .symbol = "Vcache-inst-miss",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VCACHE_MISS] = {
	      	 	 .symbol = "Vcache-miss",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VCACHE_SW_PREFETCH_MISS] = {
	      	 	 .symbol = "Vcache-sw-prefetch-miss",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VCACHE_EXTREQ_INVALID] = {
	      	 	 .symbol = "Vcache-extreq-invalid",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VCACHE_WTBK_DEGRADE] = {
	      	 	 .symbol = "Vcache-wtbk-degrade",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VCACHE_INV_INVALID] = {
	      	 	 .symbol = "Vcache-inv-invalid",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VCACHE_INVWTBK_INVALID] = {
	      	 	 .symbol = "Vcache-invwtbk-invalid",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_AR_REQUEST_ISSUE] = {
	      	 	 .symbol = "Ar-request-issue",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_AW_REQUEST_ISSUE] = {
	      	 	 .symbol = "Aw-request-issue",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_AW_DATA_REQUEST] = {
	      	 	 .symbol = "Aw-data-request",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_AR_BLOCKED_AW_UNDONE] = {
	      	 	 .symbol = "Ar-blocked-aw-undone",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_WTBK_REQUEST] = {
	      	 	 .symbol = "Missq-wtbk-request",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_INVWTBK_REQUEST] = {
	      	 	 .symbol = "Missq-invwtbk-request",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_INV_REQUEST] = {
	      	 	 .symbol = "Missq-inv-request",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_INV_CLASS_REQUEST] = {
	      	 	 .symbol = "Missq-inv-class-request",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_REFILL_TOTAL] = {
	      	 	 .symbol = "Refill-total",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_REFILL_ICACHE] = {
	      	 	 .symbol = "Refill-icache",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_REFILL_DCACHE] = {
	      	 	 .symbol = "Refill-dcache",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_REPLACE_REFILL] = {
	      	 	 .symbol = "Replace-refill",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_REFILL_DCACHE_SHARED] = {
	      	 	 .symbol = "Refill-dcache-shared",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_REFILL_DCACHE_EXC] = {
	      	 	 .symbol = "Refill-dcache-exc",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_REFILL_DATA_TOTAL] = {
	      	 	 .symbol = "Refill-data-total",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_REFILL_INST_TOTAL] = {
	      	 	 .symbol = "Refill-inst-total",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_DCACHE_REPLACE_VALID_BLOCK] = {
	      	 	 .symbol = "Dcache-replace-valid-block",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_DCACHE_REPLACE_SHARED_BLOCK] = {
	      	 	 .symbol = "Dcache-replace-shared-block",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_DCACHE_REPLACE_EXC_BLOCK] = {
	      	 	 .symbol = "Dcache-replace-exc-block",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_DCACHE_REPLACE_DIRTY_BLOCK] = {
	      	 	 .symbol = "Dcache-replace-dirty-block",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_ICACHE_REPLACE_VALID_DATA] = {
	      	 	 .symbol = "Icache-replace-valid-data",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VCACHE_REPLACE] = {
	      	 	 .symbol = "Vcache-replace",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VCACHE_REPLACE_VALID_BLOCK] = {
	      	 	 .symbol = "Vcache-replace-valid-block",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VCACHE_REPLACE_SHARED_BLOCK] = {
	      	 	 .symbol = "Vcache-replace-shared-block",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VCACHE_REPLACE_EXC_BLOCK] = {
	      	 	 .symbol = "Vcache-replace-exc-block",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VCACHE_REPLACE_DIRTY_BLOCK] = {
	      	 	 .symbol = "Vcache-replace-dirty-block",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VCACHE_REPLACE_VALID_DCBLOCK] = {
	      	 	 .symbol = "Vcache-replace-valid-dcblock",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_VCACHE_REPLACE_VALID_ICBLOCK] = {
	      	 	 .symbol = "Vcache-replace-valid-icblock",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SCACHE_LOAD_NOT_RETURN] = {
	      	 	 .symbol = "Scache-load-not-return",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SCACHE_STORE_NOT_RETURN] = {
	      	 	 .symbol = "Scache-store-not-return",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SCACHE_ICACHEREQ_NOT_RETURN] = {
	      	 	 .symbol = "Scache-icachereq-not-return",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SCREAD_TOTAL] = {
	      	 	 .symbol = "Scread-total",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SCREAD_LOAD] = {
	      	 	 .symbol = "Scread-load",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SCREAD_STORE] = {
	      	 	 .symbol = "Scread-store",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SCREAD_DATA_ACCESS] = {
	      	 	 .symbol = "Scread-data-access",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SCREAD_INST_ACCESS] = {
	      	 	 .symbol = "Scread-inst-access",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SCREAD_NPREFETCH] = {
	      	 	 .symbol = "Scread-nprefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SCREAD_NPREFETCH_DATA_LOAD] = {
	      	 	 .symbol = "Scread-nprefetch-data-load",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SCREAD_NPREFETCH_STORE] = {
	      	 	 .symbol = "Scread-nprefetch-store",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SCREAD_NPREFETCH_DATA_ACCESS] = {
	      	 	 .symbol = "Scread-nprefetch-data-access",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SCREAD_NPREFETCH_INST_ACCESS] = {
	      	 	 .symbol = "Scread-nprefetch-inst-access",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SCREAD_PREFETCH] = {
	      	 	 .symbol = "Scread-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SCREAD_LOAD_PREFETCH] = {
	      	 	 .symbol = "Scread-load-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SCREAD_STORE_PREFETCH] = {
	      	 	 .symbol = "Scread-store-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SCREAD_PREFETCH_DATA_ACCESS] = {
	      	 	 .symbol = "Scread-prefetch-data-access",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SCREAD_PREFETCH_INST_ACCESS] = {
	      	 	 .symbol = "Scread-prefetch-inst-access",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_SW_PREFETCH_REQUEST] = {
	      	 	 .symbol = "Missq-sw-prefetch-request",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_SCWRITE] = {
	      	 	 .symbol = "Missq-scwrite",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_REPLACE_SCWRITE] = {
	      	 	 .symbol = "Missq-replace-scwrite",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_INVALID_SCWRITE] = {
	      	 	 .symbol = "Missq-invalid-scwrite",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_REPLACE_VALID_SCWRITE] = {
	      	 	 .symbol = "Missq-replace-valid-scwrite",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_ACCEPT_REQ] = {
	      	 	 .symbol = "Missq-accept-req",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_ACCEPT_LOAD] = {
	      	 	 .symbol = "Missq-accept-load",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_ACCEPT_SIORE] = {
	      	 	 .symbol = "Missq-accept-store",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_DATA_ACCESS] = {
	      	 	 .symbol = "Missq-data-access",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_INST_ACCESS] = {
	      	 	 .symbol = "Missq-inst-access",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_NON_EMPTY] = {
	      	 	 .symbol = "Missq-non-empty",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_COMMON_ACCESS_OCCUPY] = {
	      	 	 .symbol = "Missq-common-access-occupy",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_FETCH_ACCESS_OCCUPY] = {
	      	 	 .symbol = "Missq-fetch-access-occupy",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_EXTERNAL_REQ_OCCUPY] = {
	      	 	 .symbol = "Missq-external-req-occupy",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_PREFETCH_REQ_OCCUPY] = {
	      	 	 .symbol = "Missq-prefetch-req-occupy",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_OCCUPY_CYCLES] = {
	      	 	 .symbol = "Missq-occupy-cycles",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_COMMON_ACCESS_CYCLES] = {
	      	 	 .symbol = "Missq-common-access-cycles",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_FETCH_ACCESS_CYCLES] = {
	      	 	 .symbol = "Missq-fetch-access-cycles",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_EXTERNAL_REQ_CYCLES] = {
	      	 	 .symbol = "Missq-external-req-cycles",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_PREFETCH_REQ_CYCLES] = {
	      	 	 .symbol = "Missq-prefetch-req-cycles",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_FULL_COUNT] = {
	      	 	 .symbol = "Missq-full-count",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_LOAD_MISSQ_PREFETCH] = {
	      	 	 .symbol = "Load-missq-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_LOAD_MISSQ_PRE_SCREF] = {
	      	 	 .symbol = "Load-missq-pre-scref",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_LOAD_MISSQ_PRE_RDY] = {
	      	 	 .symbol = "Load-missq-pre-rdy",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_LOAD_MISSQ_PRE_WAIT] = {
	      	 	 .symbol = "Load-missq-pre-wait",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_STORE_MISSQ_PRE_SCREF_LOAD] = {
	      	 	 .symbol = "Store-missq-pre-scref-load",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_STORE_MISSQ_PRE_RDY_SHARD] = {
	      	 	 .symbol = "Store-missq-pre-rdy-shard",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_STORE_MISSQ_PRE_WAIT_LOAD] = {
	      	 	 .symbol = "Store-missq-pre-wait-load",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_STORE_MISSQ_PRE_SCREF_STORE] = {
	      	 	 .symbol = "Store-missq-pre-scref-store",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_STORE_MISSQ_PRE_RDY_EXC] = {
	      	 	 .symbol = "Store-missq-pre-rdy-exc",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_STORE_MISSQ_PRE_WAIT_STORE] = {
	      	 	 .symbol = "Store-missq-pre-wait-store",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_STORE_MISSQ_PREFETCH] = {
	      	 	 .symbol = "Store-missq-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_STORE_MISSQ_VALID_PREFETCH] = {
	      	 	 .symbol = "Store-missq-valid-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_ALL_REQ_MISSQ_PREFETCH] = {
	      	 	 .symbol = "All-req-missq-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_ALL_REQ_MISSQ_PRE_SCREF] = {
	      	 	 .symbol = "All-req-missq-pre-scref",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_ALL_REQ_MISSQ_PRE_RDY] = {
	      	 	 .symbol = "All-req-missq-pre-rdy",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_ALL_REQ_MISSQ_PRE_WAIT] = {
	      	 	 .symbol = "All-req-missq-pre-wait",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FETCH_MISSQ_PREFETCH] = {
	      	 	 .symbol = "Fetch-missq-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FETCH_PRE_SCREF] = {
	      	 	 .symbol = "Fetch-pre-scref",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FETCH_PRE_RDY] = {
	      	 	 .symbol = "Fetch-pre-rdy",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_FETCH_PRE_WAIT] = {
	      	 	 .symbol = "Fetch-pre-wait",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_DATA_FETCH_PRE_RDY] = {
	      	 	 .symbol = "Data-fetch-pre-rdy",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_DATA_FETCH_PRE_WAIT] = {
	      	 	 .symbol = "Data-fetch-pre-wait",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_HW_LOAD_PRE_SCACHE_CANCEL] = {
	      	 	 .symbol = "Hw-load-pre-scache-cancel",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_HW_STORE_PRE_SCACHE_CANCEL] = {
	      	 	 .symbol = "Hw-store-pre-scache-cancel",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_HW_DATA_PRE_SCACHE_CANCEL] = {
	      	 	 .symbol = "Hw-data-pre-scache-cancel",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_HW_FETCH_PRE_SCACHE_CANCEL] = {
	      	 	 .symbol = "Hw-fetch-pre-scache-cancel",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_HW_PREFETCH_SCACHE_CANCEL] = {
	      	 	 .symbol = "Hw-prefetch-scache-cancel",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_HW_LOAD_PREFETCH] = {
	      	 	 .symbol = "Hw-load-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_HW_STORE_PREFETCH] = {
	      	 	 .symbol = "Hw-store-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_HW_DATA_PREFETCH] = {
	      	 	 .symbol = "Hw-data-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_HW_INST_PREFETCH] = {
	      	 	 .symbol = "Hw-inst-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_HW_PREFETCH] = {
	      	 	 .symbol = "Hw-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_TAGGED_LOAD_PREFETCH] = {
	      	 	 .symbol = "Tagged-load-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISS_LOAD_PREFETCH] = {
	      	 	 .symbol = "Miss-load-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_TAGGED_STORE_PREFETCH] = {
	      	 	 .symbol = "Tagged-store-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISS_STORE_PREFETCH] = {
	      	 	 .symbol = "Miss-store-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_TAGGED_DATA_PREFETCH] = {
	      	 	 .symbol = "Tagged-data-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISS_DATA_PREFETCH] = {
	      	 	 .symbol = "Miss-data-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_TAGGED_INST_PREFETCH] = {
	      	 	 .symbol = "Tagged-inst-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISS_INST_PREFETCH] = {
	      	 	 .symbol = "Miss-inst-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_TAGGED_PREFETCH] = {
	      	 	 .symbol = "Tagged-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISS_PREFETCH] = {
	      	 	 .symbol = "Miss-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_AC_LOAD_PREFETCH] = {
	      	 	 .symbol = "Missq-ac-load-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_AC_STORE_PREFETCH] = {
	      	 	 .symbol = "Missq-ac-store-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_AC_DATA_PREFETCH] = {
	      	 	 .symbol = "Missq-ac-data-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_AC_INST_PREFETCH] = {
	      	 	 .symbol = "Missq-ac-inst-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_AC_PREFETCH] = {
	      	 	 .symbol = "Missq-ac-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SCACHE_LOAD_PREFETCH] = {
	      	 	 .symbol = "Scache-load-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SCACHE_STORE_PREFETCH] = {
	      	 	 .symbol = "Scache-store-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SCACHE_DATA_PREFETCH] = {
	      	 	 .symbol = "Scache-data-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SCACHE_INST_PREFETCH] = {
	      	 	 .symbol = "Scache-inst-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_SCACHE_VALID_PREFETCH] = {
	      	 	 .symbol = "Scache-valid-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_RDY_LOAD_PREFETCH] = {
	      	 	 .symbol = "Pre-rdy-load-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_RDY_STORE_PREFETCH] = {
	      	 	 .symbol = "Pre-rdy-store-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_RDY_DATA_PREFETCH] = {
	      	 	 .symbol = "Pre-rdy-data-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_RDY_INST_PREFETCH] = {
	      	 	 .symbol = "Pre-rdy-inst-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_RDY_PREFETCH] = {
	      	 	 .symbol = "Pre-rdy-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_RDY_LOAD_REQUEST] = {
	      	 	 .symbol = "Pre-rdy-load-request",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_RDY_STORE_REQUEST] = {
	      	 	 .symbol = "Pre-rdy-store-request",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_RDY_DATA_REQUEST] = {
	      	 	 .symbol = "Pre-rdy-data-request",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_RDY_INST_REQUEST] = {
	      	 	 .symbol = "Pre-rdy-inst-request",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_RDY_REQUEST] = {
	      	 	 .symbol = "Pre-rdy-request",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_SCREF_HIT_LOAD_REQ] = {
	      	 	 .symbol = "Pre-scref-hit-load-req",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_SCREF_HIT_STORE_REQ] = {
	      	 	 .symbol = "Pre-scref-hit-store-req",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_SCREF_HIT_DATA_REQ] = {
	      	 	 .symbol = "Pre-scref-hit-data-req",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_SCREF_HIT_INST_REQ] = {
	      	 	 .symbol = "Pre-scref-hit-inst-req",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_SCREF_HIT_PREFETCH_REQ] = {
	      	 	 .symbol = "Pre-scref-hit-prefetch-req",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_SCREF_HIT_LOAD_PREFETCH] = {
	      	 	 .symbol = "Pre-scref-hit-load-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_SCREF_HIT_STORE_PREFETCH] = {
	      	 	 .symbol = "Pre-scref-hit-store-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_SCREF_RDY_DATA_REQ] = {
	      	 	 .symbol = "Pre-scref-rdy-data-req",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_SCREF_RDY_INST_REQ] = {
	      	 	 .symbol = "Pre-scref-rdy-inst-req",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_SCREF_RDY_PREFETCH] = {
	      	 	 .symbol = "Pre-scref-rdy-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_SCREF_MISS_LOAD] = {
	      	 	 .symbol = "Pre-scref-miss-load",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_WAIT_HIT_LOAD_PREFETCH] = {
	      	 	 .symbol = "Pre-wait-hit-load-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_WAIT_HIT_STORE_PREFETCH] = {
	      	 	 .symbol = "Pre-wait-hit-store-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_WAIT_HIT_DATA_PREFETCH] = {
	      	 	 .symbol = "Pre-wait-hit-data-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_WAIT_HIT_INST_PREFETCH] = {
	      	 	 .symbol = "Pre-wait-hit-inst-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PRE_WAIT_HIT_PREFETCH] = {
	      	 	 .symbol = "Pre-wait-hit-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_REPLACE_PRE_WAIT_PREFEETCH] = {
	      	 	 .symbol = "Missq-replace-pre-wait-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_MISSQ_REPLACE_PRE_RDY_PREFEETCH] = {
	      	 	 .symbol = "Missq-replace-pre-rdy-prefetch",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PREFETCH_INV] = {
	      	 	 .symbol = "Prefetch-inv",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_LOAD_PREFETCH_OCCUPY] = {
	      	 	 .symbol = "Load-prefetch-occupy",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_LOAD_PREFETCH_ISOCCUPY] = {
	      	 	 .symbol = "Load-prefetch-isoccupy",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_STORE_PREFETCH_OCCUPY] = {
	      	 	 .symbol = "Store-prefetch-occupy",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_STORE_PREFETCH_ISOCCUPY] = {
	      	 	 .symbol = "Store-prefetch-isoccupy",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_DATA_PREFETCH_OCCUPY] = {
	      	 	 .symbol = "data-prefetch-occupy",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_DATA_PREFETCH_ISOCCUPY] = {
	      	 	 .symbol = "data-prefetch-isoccupy",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_INST_PREFETCH_OCCUPY] = {
	      	 	 .symbol = "inst-prefetch-occupy",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_INST_PREFETCH_ISOCCUPY] = {
	      	 	 .symbol = "inst-prefetch-isoccupy",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_LOAD_PRE_SCREF_PRE_RDY_HIT] = {
	      	 	 .symbol = "Load-pre-scref-pre-rdy-hit",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_STORE_PRE_SCREF_PRE_RDY_HIT] = {
	      	 	 .symbol = "Store-pre-scref-pre-rdy-hit",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_DATA_PRE_SCREF_PRE_RDY_HIT] = {
	      	 	 .symbol = "Data-pre-scref-pre-rdy-hit",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_INST_PRE_SCREF_PRE_RDY_HIT] = {
	      	 	 .symbol = "Inst-pre-scref-pre-rdy-hit",
	       	 	 .alias  = "",
		},
		[PERF_COUNT_HW_PREFETCH_PRE_SCREF_PRE_RDY_HIT] = {
	      	 	 .symbol = "Prefetch-pre-scref-pre-rdy-hit",
	       	 	 .alias  = "",
		},

/*loongson3A1000 hardware events*/
        [PERF_COUNT_HW_JUMP_INSTRUCTIONS] = {
                .symbol = "JR-instructions",
                .alias  = "",
        },
        [PERF_COUNT_HW_JR31_INSTRUCTIONS] = {
                .symbol = "JR(RS=31)-instructions",
                .alias  = "JR31-instructions",
        },
        [PERF_COUNT_HW_ICACHE_MISSES] = {
                .symbol = "instruction-cache-misses",
                .alias  = "icache-misses",
        },
        [PERF_COUNT_HW_ALU1_ISSUED] = {
                .symbol = "ALU1-issued",
                .alias  = "ALU1-operation-issued",
        },
        [PERF_COUNT_HW_MEM_ISSUED] = {
                .symbol = "memory-read/write-issued",
                .alias  = "memory-issued",
        },
        [PERF_COUNT_HW_FALU1_ISSUED] = {
                .symbol = "float-ALU1-operation-issued",
                .alias  = "FALU1-issued",
        },
        [PERF_COUNT_HW_BHT_BRANCH_INSTRUCTIONS] = {
                .symbol = "BHT-branch-instructions",
                .alias  = "",
        },
        [PERF_COUNT_HW_MEM_READ] = {
                .symbol = "memory-read",
                .alias  = "read-from-primary-memory",
        },
        [PERF_COUNT_HW_FQUEUE_FULL] = {
                .symbol = "fix-queue-full",
                .alias  = "",
        },
        [PERF_COUNT_HW_ROQ_FULL] = {
                .symbol = "reorder-queue-full",
                .alias  = "",
        },
        [PERF_COUNT_HW_CP0_QUEUE_FULL] = {
                .symbol = "CP0-queue-full",
                .alias  = "",
        },
        [PERF_COUNT_HW_TLB_REFILL] = {
                .symbol = "TLB-refill-exception",
                .alias  = "",
        },
        [PERF_COUNT_HW_EXCEPTION] = {
                .symbol = "exceptions",
                .alias  = "",
        },
        [PERF_COUNT_HW_INTERNAL_EXCEPTION] = {
                .symbol = "internal-exceptions",
                .alias  = "",
        },

        [PERF_COUNT_HW_JR_MISPREDICTED] = {
                .symbol = "JR-mispredicted",
                .alias  = "",
        },
        [PERF_COUNT_HW_JR31_MISPREDICTED] = {
                .symbol = "JR31-mispredicted",
                .alias  = "",
        },
        [PERF_COUNT_HW_DCACHE_MISSES] = {
                .symbol = "data-cache-misses",
                .alias  = "",
        },
        [PERF_COUNT_HW_ALU2_ISSUED] = {
                .symbol = "ALU2-operation-issued",
                .alias  = "ALU2-issued",
        },
        [PERF_COUNT_HW_FALU2_ISSUED] = {
                .symbol = "FALU2-operation-issued",
                .alias  = "FALU2-issued",
        },
        [PERF_COUNT_HW_UNCACHED_ACCESS] = {
                .symbol = "uncached-accesses",
                .alias  = "",
        },
        [PERF_COUNT_HW_BHT_MISPREDICTED] = {
                .symbol = "BHT-mispredicted",
                .alias  = "",
        },
        [PERF_COUNT_HW_MEM_WRITE] = {
                .symbol = "write-to-memory",
                .alias  = "",
        },
        [PERF_COUNT_HW_FTQ_FULL] = {
                .symbol = "float-queue-full",
                .alias  = "",
        },
        [PERF_COUNT_HW_BRANCH_QUEUE_FULL] = {
                .symbol = "branch-queue-full",
                .alias  = "",
        },
        [PERF_COUNT_HW_ITLB_MISSES] = {
                .symbol = "instruction-TLB-misses",
                .alias  = "ITLB-misses",
        },
        [PERF_COUNT_HW_TOTAL_EXCEPTIONS] = {
                .symbol = "total-exceptions",
                .alias  = "",
        },
        [PERF_COUNT_HW_LOAD_SPECULATION_MISSES] = {
                .symbol = "load-speculation-misses",
                .alias  = "",
        },
        [PERF_COUNT_HW_CP0Q_FORWARD_VALID] = {
                .symbol = "CP0-queue-forward-valid",
                .alias  = "",
        },
#endif
};

static struct event_symbol event_symbols_sw[PERF_COUNT_SW_MAX] = {
	[PERF_COUNT_SW_CPU_CLOCK] = {
		.symbol = "cpu-clock",
		.alias  = "",
	},
	[PERF_COUNT_SW_TASK_CLOCK] = {
		.symbol = "task-clock",
		.alias  = "",
	},
	[PERF_COUNT_SW_PAGE_FAULTS] = {
		.symbol = "page-faults",
		.alias  = "faults",
	},
	[PERF_COUNT_SW_CONTEXT_SWITCHES] = {
		.symbol = "context-switches",
		.alias  = "cs",
	},
	[PERF_COUNT_SW_CPU_MIGRATIONS] = {
		.symbol = "cpu-migrations",
		.alias  = "migrations",
	},
	[PERF_COUNT_SW_PAGE_FAULTS_MIN] = {
		.symbol = "minor-faults",
		.alias  = "",
	},
	[PERF_COUNT_SW_PAGE_FAULTS_MAJ] = {
		.symbol = "major-faults",
		.alias  = "",
	},
	[PERF_COUNT_SW_ALIGNMENT_FAULTS] = {
		.symbol = "alignment-faults",
		.alias  = "",
	},
	[PERF_COUNT_SW_EMULATION_FAULTS] = {
		.symbol = "emulation-faults",
		.alias  = "",
	},
};

#define __PERF_EVENT_FIELD(config, name) \
	((config & PERF_EVENT_##name##_MASK) >> PERF_EVENT_##name##_SHIFT)

#define PERF_EVENT_RAW(config)		__PERF_EVENT_FIELD(config, RAW)
#define PERF_EVENT_CONFIG(config)	__PERF_EVENT_FIELD(config, CONFIG)
#define PERF_EVENT_TYPE(config)		__PERF_EVENT_FIELD(config, TYPE)
#define PERF_EVENT_ID(config)		__PERF_EVENT_FIELD(config, EVENT)

#define for_each_subsystem(sys_dir, sys_dirent, sys_next)	       \
	while (!readdir_r(sys_dir, &sys_dirent, &sys_next) && sys_next)	       \
	if (sys_dirent.d_type == DT_DIR &&				       \
	   (strcmp(sys_dirent.d_name, ".")) &&				       \
	   (strcmp(sys_dirent.d_name, "..")))

static int tp_event_has_id(struct dirent *sys_dir, struct dirent *evt_dir)
{
	char evt_path[MAXPATHLEN];
	int fd;

	snprintf(evt_path, MAXPATHLEN, "%s/%s/%s/id", tracing_events_path,
			sys_dir->d_name, evt_dir->d_name);
	fd = open(evt_path, O_RDONLY);
	if (fd < 0)
		return -EINVAL;
	close(fd);

	return 0;
}

#define for_each_event(sys_dirent, evt_dir, evt_dirent, evt_next)	       \
	while (!readdir_r(evt_dir, &evt_dirent, &evt_next) && evt_next)        \
	if (evt_dirent.d_type == DT_DIR &&				       \
	   (strcmp(evt_dirent.d_name, ".")) &&				       \
	   (strcmp(evt_dirent.d_name, "..")) &&				       \
	   (!tp_event_has_id(&sys_dirent, &evt_dirent)))

#define MAX_EVENT_LENGTH 512


struct tracepoint_path *tracepoint_id_to_path(u64 config)
{
	struct tracepoint_path *path = NULL;
	DIR *sys_dir, *evt_dir;
	struct dirent *sys_next, *evt_next, sys_dirent, evt_dirent;
	char id_buf[24];
	int fd;
	u64 id;
	char evt_path[MAXPATHLEN];
	char dir_path[MAXPATHLEN];

	if (debugfs_valid_mountpoint(tracing_events_path))
		return NULL;

	sys_dir = opendir(tracing_events_path);
	if (!sys_dir)
		return NULL;

	for_each_subsystem(sys_dir, sys_dirent, sys_next) {

		snprintf(dir_path, MAXPATHLEN, "%s/%s", tracing_events_path,
			 sys_dirent.d_name);
		evt_dir = opendir(dir_path);
		if (!evt_dir)
			continue;

		for_each_event(sys_dirent, evt_dir, evt_dirent, evt_next) {

			snprintf(evt_path, MAXPATHLEN, "%s/%s/id", dir_path,
				 evt_dirent.d_name);
			fd = open(evt_path, O_RDONLY);
			if (fd < 0)
				continue;
			if (read(fd, id_buf, sizeof(id_buf)) < 0) {
				close(fd);
				continue;
			}
			close(fd);
			id = atoll(id_buf);
			if (id == config) {
				closedir(evt_dir);
				closedir(sys_dir);
				path = zalloc(sizeof(*path));
				path->system = malloc(MAX_EVENT_LENGTH);
				if (!path->system) {
					free(path);
					return NULL;
				}
				path->name = malloc(MAX_EVENT_LENGTH);
				if (!path->name) {
					free(path->system);
					free(path);
					return NULL;
				}
				strncpy(path->system, sys_dirent.d_name,
					MAX_EVENT_LENGTH);
				strncpy(path->name, evt_dirent.d_name,
					MAX_EVENT_LENGTH);
				return path;
			}
		}
		closedir(evt_dir);
	}

	closedir(sys_dir);
	return NULL;
}

const char *event_type(int type)
{
	switch (type) {
	case PERF_TYPE_HARDWARE:
		return "hardware";

	case PERF_TYPE_SOFTWARE:
		return "software";

	case PERF_TYPE_TRACEPOINT:
		return "tracepoint";

	case PERF_TYPE_HW_CACHE:
		return "hardware-cache";

	default:
		break;
	}

	return "unknown";
}



static int __add_event(struct list_head **_list, int *idx,
		       struct perf_event_attr *attr,
		       char *name, struct cpu_map *cpus)
{
	struct perf_evsel *evsel;
	struct list_head *list = *_list;

	if (!list) {
		list = malloc(sizeof(*list));
		if (!list)
			return -ENOMEM;
		INIT_LIST_HEAD(list);
	}

	event_attr_init(attr);

	evsel = perf_evsel__new(attr, (*idx)++);
	if (!evsel) {
		free(list);
		return -ENOMEM;
	}

	evsel->cpus = cpus;
	if (name)
		evsel->name = strdup(name);
	list_add_tail(&evsel->node, list);
	*_list = list;
	return 0;
}

static int add_event(struct list_head **_list, int *idx,
		     struct perf_event_attr *attr, char *name)
{
	return __add_event(_list, idx, attr, name, NULL);
}

static int parse_aliases(char *str, const char *names[][PERF_EVSEL__MAX_ALIASES], int size)
{
	int i, j;
	int n, longest = -1;

	for (i = 0; i < size; i++) {
		for (j = 0; j < PERF_EVSEL__MAX_ALIASES && names[i][j]; j++) {
			n = strlen(names[i][j]);
			if (n > longest && !strncasecmp(str, names[i][j], n))
				longest = n;
		}
		if (longest > 0)
			return i;
	}

	return -1;
}

int parse_events_add_cache(struct list_head **list, int *idx,
			   char *type, char *op_result1, char *op_result2)
{
	struct perf_event_attr attr;
	char name[MAX_NAME_LEN];
	int cache_type = -1, cache_op = -1, cache_result = -1;
	char *op_result[2] = { op_result1, op_result2 };
	int i, n;

	/*
	 * No fallback - if we cannot get a clear cache type
	 * then bail out:
	 */
	cache_type = parse_aliases(type, perf_evsel__hw_cache,
				   PERF_COUNT_HW_CACHE_MAX);
	if (cache_type == -1)
		return -EINVAL;

	n = snprintf(name, MAX_NAME_LEN, "%s", type);

	for (i = 0; (i < 2) && (op_result[i]); i++) {
		char *str = op_result[i];

		n += snprintf(name + n, MAX_NAME_LEN - n, "-%s", str);

		if (cache_op == -1) {
			cache_op = parse_aliases(str, perf_evsel__hw_cache_op,
						 PERF_COUNT_HW_CACHE_OP_MAX);
			if (cache_op >= 0) {
				if (!perf_evsel__is_cache_op_valid(cache_type, cache_op))
					return -EINVAL;
				continue;
			}
		}

		if (cache_result == -1) {
			cache_result = parse_aliases(str, perf_evsel__hw_cache_result,
						     PERF_COUNT_HW_CACHE_RESULT_MAX);
			if (cache_result >= 0)
				continue;
		}
	}

	/*
	 * Fall back to reads:
	 */
	if (cache_op == -1)
		cache_op = PERF_COUNT_HW_CACHE_OP_READ;

	/*
	 * Fall back to accesses:
	 */
	if (cache_result == -1)
		cache_result = PERF_COUNT_HW_CACHE_RESULT_ACCESS;

	memset(&attr, 0, sizeof(attr));
	attr.config = cache_type | (cache_op << 8) | (cache_result << 16);
	attr.type = PERF_TYPE_HW_CACHE;
	return add_event(list, idx, &attr, name);
}

static int add_tracepoint(struct list_head **listp, int *idx,
			  char *sys_name, char *evt_name)
{
	struct perf_evsel *evsel;
	struct list_head *list = *listp;

	if (!list) {
		list = malloc(sizeof(*list));
		if (!list)
			return -ENOMEM;
		INIT_LIST_HEAD(list);
	}

	evsel = perf_evsel__newtp(sys_name, evt_name, (*idx)++);
	if (!evsel) {
		free(list);
		return -ENOMEM;
	}

	list_add_tail(&evsel->node, list);
	*listp = list;
	return 0;
}

static int add_tracepoint_multi_event(struct list_head **list, int *idx,
				      char *sys_name, char *evt_name)
{
	char evt_path[MAXPATHLEN];
	struct dirent *evt_ent;
	DIR *evt_dir;
	int ret = 0;

	snprintf(evt_path, MAXPATHLEN, "%s/%s", tracing_events_path, sys_name);
	evt_dir = opendir(evt_path);
	if (!evt_dir) {
		perror("Can't open event dir");
		return -1;
	}

	while (!ret && (evt_ent = readdir(evt_dir))) {
		if (!strcmp(evt_ent->d_name, ".")
		    || !strcmp(evt_ent->d_name, "..")
		    || !strcmp(evt_ent->d_name, "enable")
		    || !strcmp(evt_ent->d_name, "filter"))
			continue;

		if (!strglobmatch(evt_ent->d_name, evt_name))
			continue;

		ret = add_tracepoint(list, idx, sys_name, evt_ent->d_name);
	}

	closedir(evt_dir);
	return ret;
}

static int add_tracepoint_event(struct list_head **list, int *idx,
				char *sys_name, char *evt_name)
{
	return strpbrk(evt_name, "*?") ?
	       add_tracepoint_multi_event(list, idx, sys_name, evt_name) :
	       add_tracepoint(list, idx, sys_name, evt_name);
}

static int add_tracepoint_multi_sys(struct list_head **list, int *idx,
				    char *sys_name, char *evt_name)
{
	struct dirent *events_ent;
	DIR *events_dir;
	int ret = 0;

	events_dir = opendir(tracing_events_path);
	if (!events_dir) {
		perror("Can't open event dir");
		return -1;
	}

	while (!ret && (events_ent = readdir(events_dir))) {
		if (!strcmp(events_ent->d_name, ".")
		    || !strcmp(events_ent->d_name, "..")
		    || !strcmp(events_ent->d_name, "enable")
		    || !strcmp(events_ent->d_name, "header_event")
		    || !strcmp(events_ent->d_name, "header_page"))
			continue;

		if (!strglobmatch(events_ent->d_name, sys_name))
			continue;

		ret = add_tracepoint_event(list, idx, events_ent->d_name,
					   evt_name);
	}

	closedir(events_dir);
	return ret;
}

int parse_events_add_tracepoint(struct list_head **list, int *idx,
				char *sys, char *event)
{
	int ret;

	ret = debugfs_valid_mountpoint(tracing_events_path);
	if (ret)
		return ret;

	if (strpbrk(sys, "*?"))
		return add_tracepoint_multi_sys(list, idx, sys, event);
	else
		return add_tracepoint_event(list, idx, sys, event);
}

static int
parse_breakpoint_type(const char *type, struct perf_event_attr *attr)
{
	int i;

	for (i = 0; i < 3; i++) {
		if (!type || !type[i])
			break;

#define CHECK_SET_TYPE(bit)		\
do {					\
	if (attr->bp_type & bit)	\
		return -EINVAL;		\
	else				\
		attr->bp_type |= bit;	\
} while (0)

		switch (type[i]) {
		case 'r':
			CHECK_SET_TYPE(HW_BREAKPOINT_R);
			break;
		case 'w':
			CHECK_SET_TYPE(HW_BREAKPOINT_W);
			break;
		case 'x':
			CHECK_SET_TYPE(HW_BREAKPOINT_X);
			break;
		default:
			return -EINVAL;
		}
	}

#undef CHECK_SET_TYPE

	if (!attr->bp_type) /* Default */
		attr->bp_type = HW_BREAKPOINT_R | HW_BREAKPOINT_W;

	return 0;
}

int parse_events_add_breakpoint(struct list_head **list, int *idx,
				void *ptr, char *type)
{
	struct perf_event_attr attr;

	memset(&attr, 0, sizeof(attr));
	attr.bp_addr = (unsigned long) ptr;

	if (parse_breakpoint_type(type, &attr))
		return -EINVAL;

	/*
	 * We should find a nice way to override the access length
	 * Provide some defaults for now
	 */
	if (attr.bp_type == HW_BREAKPOINT_X)
		attr.bp_len = sizeof(long);
	else
		attr.bp_len = HW_BREAKPOINT_LEN_4;

	attr.type = PERF_TYPE_BREAKPOINT;
	attr.sample_period = 1;

	return add_event(list, idx, &attr, NULL);
}

static int config_term(struct perf_event_attr *attr,
		       struct parse_events_term *term)
{
#define CHECK_TYPE_VAL(type)					\
do {								\
	if (PARSE_EVENTS__TERM_TYPE_ ## type != term->type_val)	\
		return -EINVAL;					\
} while (0)

	switch (term->type_term) {
	case PARSE_EVENTS__TERM_TYPE_CONFIG:
		CHECK_TYPE_VAL(NUM);
		attr->config = term->val.num;
		break;
	case PARSE_EVENTS__TERM_TYPE_CONFIG1:
		CHECK_TYPE_VAL(NUM);
		attr->config1 = term->val.num;
		break;
	case PARSE_EVENTS__TERM_TYPE_CONFIG2:
		CHECK_TYPE_VAL(NUM);
		attr->config2 = term->val.num;
		break;
	case PARSE_EVENTS__TERM_TYPE_SAMPLE_PERIOD:
		CHECK_TYPE_VAL(NUM);
		attr->sample_period = term->val.num;
		break;
	case PARSE_EVENTS__TERM_TYPE_BRANCH_SAMPLE_TYPE:
		/*
		 * TODO uncomment when the field is available
		 * attr->branch_sample_type = term->val.num;
		 */
		break;
	case PARSE_EVENTS__TERM_TYPE_NAME:
		CHECK_TYPE_VAL(STR);
		break;
	default:
		return -EINVAL;
	}

	return 0;
#undef CHECK_TYPE_VAL
}

static int config_attr(struct perf_event_attr *attr,
		       struct list_head *head, int fail)
{
	struct parse_events_term *term;

	list_for_each_entry(term, head, list)
		if (config_term(attr, term) && fail)
			return -EINVAL;

	return 0;
}

int parse_events_add_numeric(struct list_head **list, int *idx,
			     u32 type, u64 config,
			     struct list_head *head_config)
{
	struct perf_event_attr attr;

	memset(&attr, 0, sizeof(attr));
	attr.type = type;
	attr.config = config;

	if (head_config &&
	    config_attr(&attr, head_config, 1))
		return -EINVAL;

	return add_event(list, idx, &attr, NULL);
}

static int parse_events__is_name_term(struct parse_events_term *term)
{
	return term->type_term == PARSE_EVENTS__TERM_TYPE_NAME;
}

static char *pmu_event_name(struct list_head *head_terms)
{
	struct parse_events_term *term;

	list_for_each_entry(term, head_terms, list)
		if (parse_events__is_name_term(term))
			return term->val.str;

	return NULL;
}

int parse_events_add_pmu(struct list_head **list, int *idx,
			 char *name, struct list_head *head_config)
{
	struct perf_event_attr attr;
	struct perf_pmu *pmu;

	pmu = perf_pmu__find(name);
	if (!pmu)
		return -EINVAL;

	memset(&attr, 0, sizeof(attr));

	if (perf_pmu__check_alias(pmu, head_config))
		return -EINVAL;

	/*
	 * Configure hardcoded terms first, no need to check
	 * return value when called with fail == 0 ;)
	 */
	config_attr(&attr, head_config, 0);

	if (perf_pmu__config(pmu, &attr, head_config))
		return -EINVAL;

	return __add_event(list, idx, &attr, pmu_event_name(head_config),
			   pmu->cpus);
}

int parse_events__modifier_group(struct list_head *list,
				 char *event_mod)
{
	return parse_events__modifier_event(list, event_mod, true);
}

void parse_events__set_leader(char *name, struct list_head *list)
{
	struct perf_evsel *leader;

	__perf_evlist__set_leader(list);
	leader = list_entry(list->next, struct perf_evsel, node);
	leader->group_name = name ? strdup(name) : NULL;
}

void parse_events_update_lists(struct list_head *list_event,
			       struct list_head *list_all)
{
	/*
	 * Called for single event definition. Update the
	 * 'all event' list, and reinit the 'single event'
	 * list, for next event definition.
	 */
	list_splice_tail(list_event, list_all);
	free(list_event);
}

struct event_modifier {
	int eu;
	int ek;
	int eh;
	int eH;
	int eG;
	int precise;
	int exclude_GH;
};

static int get_event_modifier(struct event_modifier *mod, char *str,
			       struct perf_evsel *evsel)
{
	int eu = evsel ? evsel->attr.exclude_user : 0;
	int ek = evsel ? evsel->attr.exclude_kernel : 0;
	int eh = evsel ? evsel->attr.exclude_hv : 0;
	int eH = evsel ? evsel->attr.exclude_host : 0;
	int eG = evsel ? evsel->attr.exclude_guest : 0;
	int precise = evsel ? evsel->attr.precise_ip : 0;

	int exclude = eu | ek | eh;
	int exclude_GH = evsel ? evsel->exclude_GH : 0;

	memset(mod, 0, sizeof(*mod));

	while (*str) {
		if (*str == 'u') {
			if (!exclude)
				exclude = eu = ek = eh = 1;
			eu = 0;
		} else if (*str == 'k') {
			if (!exclude)
				exclude = eu = ek = eh = 1;
			ek = 0;
		} else if (*str == 'h') {
			if (!exclude)
				exclude = eu = ek = eh = 1;
			eh = 0;
		} else if (*str == 'G') {
			if (!exclude_GH)
				exclude_GH = eG = eH = 1;
			eG = 0;
		} else if (*str == 'H') {
			if (!exclude_GH)
				exclude_GH = eG = eH = 1;
			eH = 0;
		} else if (*str == 'p') {
			precise++;
			/* use of precise requires exclude_guest */
			if (!exclude_GH)
				eG = 1;
		} else
			break;

		++str;
	}

	/*
	 * precise ip:
	 *
	 *  0 - SAMPLE_IP can have arbitrary skid
	 *  1 - SAMPLE_IP must have constant skid
	 *  2 - SAMPLE_IP requested to have 0 skid
	 *  3 - SAMPLE_IP must have 0 skid
	 *
	 *  See also PERF_RECORD_MISC_EXACT_IP
	 */
	if (precise > 3)
		return -EINVAL;

	mod->eu = eu;
	mod->ek = ek;
	mod->eh = eh;
	mod->eH = eH;
	mod->eG = eG;
	mod->precise = precise;
	mod->exclude_GH = exclude_GH;
	return 0;
}

/*
 * Basic modifier sanity check to validate it contains only one
 * instance of any modifier (apart from 'p') present.
 */
static int check_modifier(char *str)
{
	char *p = str;

	/* The sizeof includes 0 byte as well. */
	if (strlen(str) > (sizeof("ukhGHppp") - 1))
		return -1;

	while (*p) {
		if (*p != 'p' && strchr(p + 1, *p))
			return -1;
		p++;
	}

	return 0;
}

int parse_events__modifier_event(struct list_head *list, char *str, bool add)
{
	struct perf_evsel *evsel;
	struct event_modifier mod;

	if (str == NULL)
		return 0;

	if (check_modifier(str))
		return -EINVAL;

	if (!add && get_event_modifier(&mod, str, NULL))
		return -EINVAL;

	list_for_each_entry(evsel, list, node) {

		if (add && get_event_modifier(&mod, str, evsel))
			return -EINVAL;

		evsel->attr.exclude_user   = mod.eu;
		evsel->attr.exclude_kernel = mod.ek;
		evsel->attr.exclude_hv     = mod.eh;
		evsel->attr.precise_ip     = mod.precise;
		evsel->attr.exclude_host   = mod.eH;
		evsel->attr.exclude_guest  = mod.eG;
		evsel->exclude_GH          = mod.exclude_GH;
	}

	return 0;
}

int parse_events_name(struct list_head *list, char *name)
{
	struct perf_evsel *evsel;

	list_for_each_entry(evsel, list, node) {
		if (!evsel->name)
			evsel->name = strdup(name);
	}

	return 0;
}

static int parse_events__scanner(const char *str, void *data, int start_token)
{
	YY_BUFFER_STATE buffer;
	void *scanner;
	int ret;

	ret = parse_events_lex_init_extra(start_token, &scanner);
	if (ret)
		return ret;

	buffer = parse_events__scan_string(str, scanner);

#ifdef PARSER_DEBUG
	parse_events_debug = 1;
#endif
	ret = parse_events_parse(data, scanner);

	parse_events__flush_buffer(buffer, scanner);
	parse_events__delete_buffer(buffer, scanner);
	parse_events_lex_destroy(scanner);
	return ret;
}

/*
 * parse event config string, return a list of event terms.
 */
int parse_events_terms(struct list_head *terms, const char *str)
{
	struct parse_events_terms data = {
		.terms = NULL,
	};
	int ret;

	ret = parse_events__scanner(str, &data, PE_START_TERMS);
	if (!ret) {
		list_splice(data.terms, terms);
		free(data.terms);
		return 0;
	}

	parse_events__free_terms(data.terms);
	return ret;
}

int parse_events(struct perf_evlist *evlist, const char *str)
{
	struct parse_events_evlist data = {
		.list = LIST_HEAD_INIT(data.list),
		.idx  = evlist->nr_entries,
	};
	int ret;

	ret = parse_events__scanner(str, &data, PE_START_EVENTS);
	if (!ret) {
		int entries = data.idx - evlist->nr_entries;
		perf_evlist__splice_list_tail(evlist, &data.list, entries);
		evlist->nr_groups += data.nr_groups;
		return 0;
	}

	/*
	 * There are 2 users - builtin-record and builtin-test objects.
	 * Both call perf_evlist__delete in case of error, so we dont
	 * need to bother.
	 */
	return ret;
}

int parse_events_option(const struct option *opt, const char *str,
			int unset __maybe_unused)
{
	struct perf_evlist *evlist = *(struct perf_evlist **)opt->value;
	int ret = parse_events(evlist, str);

	if (ret) {
		fprintf(stderr, "invalid or unsupported event: '%s'\n", str);
		fprintf(stderr, "Run 'perf list' for a list of valid events\n");
	}
	return ret;
}

int parse_filter(const struct option *opt, const char *str,
		 int unset __maybe_unused)
{
	struct perf_evlist *evlist = *(struct perf_evlist **)opt->value;
	struct perf_evsel *last = NULL;

	if (evlist->nr_entries > 0)
		last = perf_evlist__last(evlist);

	if (last == NULL || last->attr.type != PERF_TYPE_TRACEPOINT) {
		fprintf(stderr,
			"-F option should follow a -e tracepoint option\n");
		return -1;
	}

	last->filter = strdup(str);
	if (last->filter == NULL) {
		fprintf(stderr, "not enough memory to hold filter string\n");
		return -1;
	}

	return 0;
}

static const char * const event_type_descriptors[] = {
	"Hardware event",
	"Software event",
	"Tracepoint event",
	"Hardware cache event",
	"Raw hardware event descriptor",
	"Hardware breakpoint",
};

/*
 * Print the events from <debugfs_mount_point>/tracing/events
 */

void print_tracepoint_events(const char *subsys_glob, const char *event_glob,
			     bool name_only)
{
	DIR *sys_dir, *evt_dir;
	struct dirent *sys_next, *evt_next, sys_dirent, evt_dirent;
	char evt_path[MAXPATHLEN];
	char dir_path[MAXPATHLEN];

	if (debugfs_valid_mountpoint(tracing_events_path))
		return;

	sys_dir = opendir(tracing_events_path);
	if (!sys_dir)
		return;

	for_each_subsystem(sys_dir, sys_dirent, sys_next) {
		if (subsys_glob != NULL && 
		    !strglobmatch(sys_dirent.d_name, subsys_glob))
			continue;

		snprintf(dir_path, MAXPATHLEN, "%s/%s", tracing_events_path,
			 sys_dirent.d_name);
		evt_dir = opendir(dir_path);
		if (!evt_dir)
			continue;

		for_each_event(sys_dirent, evt_dir, evt_dirent, evt_next) {
			if (event_glob != NULL && 
			    !strglobmatch(evt_dirent.d_name, event_glob))
				continue;

			if (name_only) {
				printf("%s:%s ", sys_dirent.d_name, evt_dirent.d_name);
				continue;
			}

			snprintf(evt_path, MAXPATHLEN, "%s:%s",
				 sys_dirent.d_name, evt_dirent.d_name);
			printf("  %-50s [%s]\n", evt_path,
				event_type_descriptors[PERF_TYPE_TRACEPOINT]);
		}
		closedir(evt_dir);
	}
	closedir(sys_dir);
}

/*
 * Check whether event is in <debugfs_mount_point>/tracing/events
 */

int is_valid_tracepoint(const char *event_string)
{
	DIR *sys_dir, *evt_dir;
	struct dirent *sys_next, *evt_next, sys_dirent, evt_dirent;
	char evt_path[MAXPATHLEN];
	char dir_path[MAXPATHLEN];

	if (debugfs_valid_mountpoint(tracing_events_path))
		return 0;

	sys_dir = opendir(tracing_events_path);
	if (!sys_dir)
		return 0;

	for_each_subsystem(sys_dir, sys_dirent, sys_next) {

		snprintf(dir_path, MAXPATHLEN, "%s/%s", tracing_events_path,
			 sys_dirent.d_name);
		evt_dir = opendir(dir_path);
		if (!evt_dir)
			continue;

		for_each_event(sys_dirent, evt_dir, evt_dirent, evt_next) {
			snprintf(evt_path, MAXPATHLEN, "%s:%s",
				 sys_dirent.d_name, evt_dirent.d_name);
			if (!strcmp(evt_path, event_string)) {
				closedir(evt_dir);
				closedir(sys_dir);
				return 1;
			}
		}
		closedir(evt_dir);
	}
	closedir(sys_dir);
	return 0;
}

static void __print_events_type(u8 type, struct event_symbol *syms,
				unsigned max)
{
	char name[64];
	unsigned i;

	for (i = 0; i < max ; i++, syms++) {
		if (strlen(syms->alias))
			snprintf(name, sizeof(name),  "%s OR %s",
				 syms->symbol, syms->alias);
		else
			snprintf(name, sizeof(name), "%s", syms->symbol);

		printf("  %-50s [%s]\n", name,
			event_type_descriptors[type]);
	}
}

void print_events_type(u8 type)
{
	if (type == PERF_TYPE_SOFTWARE)
		__print_events_type(type, event_symbols_sw, PERF_COUNT_SW_MAX);
	else
		__print_events_type(type, event_symbols_hw, PERF_COUNT_HW_MAX);
}

int print_hwcache_events(const char *event_glob, bool name_only)
{
	unsigned int type, op, i, printed = 0;
	char name[64];

	for (type = 0; type < PERF_COUNT_HW_CACHE_MAX; type++) {
		for (op = 0; op < PERF_COUNT_HW_CACHE_OP_MAX; op++) {
			/* skip invalid cache type */
			if (!perf_evsel__is_cache_op_valid(type, op))
				continue;

			for (i = 0; i < PERF_COUNT_HW_CACHE_RESULT_MAX; i++) {
				__perf_evsel__hw_cache_type_op_res_name(type, op, i,
									name, sizeof(name));
				if (event_glob != NULL && !strglobmatch(name, event_glob))
					continue;

				if (name_only)
					printf("%s ", name);
				else
					printf("  %-50s [%s]\n", name,
					       event_type_descriptors[PERF_TYPE_HW_CACHE]);
				++printed;
			}
		}
	}

	return printed;
}

static void print_symbol_events(const char *event_glob, unsigned type,
				struct event_symbol *syms, unsigned max,
				bool name_only)
{
	unsigned i, printed = 0;
	char name[MAX_NAME_LEN];

	for (i = 0; i < max; i++, syms++) {

		if (event_glob != NULL && 
		    !(strglobmatch(syms->symbol, event_glob) ||
		      (syms->alias && strglobmatch(syms->alias, event_glob))))
			continue;

		if (name_only) {
			printf("%s ", syms->symbol);
			continue;
		}

		if (strlen(syms->alias))
			snprintf(name, MAX_NAME_LEN, "%s OR %s", syms->symbol, syms->alias);
		else
			strncpy(name, syms->symbol, MAX_NAME_LEN);

		printf("  %-50s [%s]\n", name, event_type_descriptors[type]);

		printed++;
	}

	if (printed)
		printf("\n");
}

/*
 * Print the help text for the event symbols:
 */
void print_events(const char *event_glob, bool name_only)
{
	if (!name_only) {
		printf("\n");
		printf("List of pre-defined events (to be used in -e):\n");
	}

	print_symbol_events(event_glob, PERF_TYPE_HARDWARE,
			    event_symbols_hw, PERF_COUNT_HW_MAX, name_only);

	print_symbol_events(event_glob, PERF_TYPE_SOFTWARE,
			    event_symbols_sw, PERF_COUNT_SW_MAX, name_only);

	print_hwcache_events(event_glob, name_only);

	if (event_glob != NULL)
		return;

	if (!name_only) {
		printf("\n");
		printf("  %-50s [%s]\n",
		       "rNNN",
		       event_type_descriptors[PERF_TYPE_RAW]);
		printf("  %-50s [%s]\n",
		       "cpu/t1=v1[,t2=v2,t3 ...]/modifier",
		       event_type_descriptors[PERF_TYPE_RAW]);
		printf("   (see 'man perf-list' on how to encode it)\n");
		printf("\n");

		printf("  %-50s [%s]\n",
		       "mem:<addr>[:access]",
			event_type_descriptors[PERF_TYPE_BREAKPOINT]);
		printf("\n");
	}

	print_tracepoint_events(NULL, NULL, name_only);
}

int parse_events__is_hardcoded_term(struct parse_events_term *term)
{
	return term->type_term != PARSE_EVENTS__TERM_TYPE_USER;
}

static int new_term(struct parse_events_term **_term, int type_val,
		    int type_term, char *config,
		    char *str, u64 num)
{
	struct parse_events_term *term;

	term = zalloc(sizeof(*term));
	if (!term)
		return -ENOMEM;

	INIT_LIST_HEAD(&term->list);
	term->type_val  = type_val;
	term->type_term = type_term;
	term->config = config;

	switch (type_val) {
	case PARSE_EVENTS__TERM_TYPE_NUM:
		term->val.num = num;
		break;
	case PARSE_EVENTS__TERM_TYPE_STR:
		term->val.str = str;
		break;
	default:
		return -EINVAL;
	}

	*_term = term;
	return 0;
}

int parse_events_term__num(struct parse_events_term **term,
			   int type_term, char *config, u64 num)
{
	return new_term(term, PARSE_EVENTS__TERM_TYPE_NUM, type_term,
			config, NULL, num);
}

int parse_events_term__str(struct parse_events_term **term,
			   int type_term, char *config, char *str)
{
	return new_term(term, PARSE_EVENTS__TERM_TYPE_STR, type_term,
			config, str, 0);
}

int parse_events_term__sym_hw(struct parse_events_term **term,
			      char *config, unsigned idx)
{
	struct event_symbol *sym;

	BUG_ON(idx >= PERF_COUNT_HW_MAX);
	sym = &event_symbols_hw[idx];

	if (config)
		return new_term(term, PARSE_EVENTS__TERM_TYPE_STR,
				PARSE_EVENTS__TERM_TYPE_USER, config,
				(char *) sym->symbol, 0);
	else
		return new_term(term, PARSE_EVENTS__TERM_TYPE_STR,
				PARSE_EVENTS__TERM_TYPE_USER,
				(char *) "event", (char *) sym->symbol, 0);
}

int parse_events_term__clone(struct parse_events_term **new,
			     struct parse_events_term *term)
{
	return new_term(new, term->type_val, term->type_term, term->config,
			term->val.str, term->val.num);
}

void parse_events__free_terms(struct list_head *terms)
{
	struct parse_events_term *term, *h;

	list_for_each_entry_safe(term, h, terms, list)
		free(term);

	free(terms);
}

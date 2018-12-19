
%option reentrant
%option bison-bridge
%option prefix="parse_events_"
%option stack

%{
#include <errno.h>
#include "../perf.h"
#include "parse-events-bison.h"
#include "parse-events.h"

char *parse_events_get_text(yyscan_t yyscanner);
YYSTYPE *parse_events_get_lval(yyscan_t yyscanner);

static int __value(YYSTYPE *yylval, char *str, int base, int token)
{
	u64 num;

	errno = 0;
	num = strtoull(str, NULL, base);
	if (errno)
		return PE_ERROR;

	yylval->num = num;
	return token;
}

static int value(yyscan_t scanner, int base)
{
	YYSTYPE *yylval = parse_events_get_lval(scanner);
	char *text = parse_events_get_text(scanner);

	return __value(yylval, text, base, PE_VALUE);
}

static int raw(yyscan_t scanner)
{
	YYSTYPE *yylval = parse_events_get_lval(scanner);
	char *text = parse_events_get_text(scanner);

	return __value(yylval, text + 1, 16, PE_RAW);
}

static int str(yyscan_t scanner, int token)
{
	YYSTYPE *yylval = parse_events_get_lval(scanner);
	char *text = parse_events_get_text(scanner);

	yylval->str = strdup(text);
	return token;
}

static int sym(yyscan_t scanner, int type, int config)
{
	YYSTYPE *yylval = parse_events_get_lval(scanner);

	yylval->num = (type << 16) + config;
	return type == PERF_TYPE_HARDWARE ? PE_VALUE_SYM_HW : PE_VALUE_SYM_SW;
}

static int term(yyscan_t scanner, int type)
{
	YYSTYPE *yylval = parse_events_get_lval(scanner);

	yylval->num = type;
	return PE_TERM;
}

%}

%x mem
%s config
%x event

group		[^,{}/]*[{][^}]*[}][^,{}/]*
event_pmu	[^,{}/]+[/][^/]*[/][^,{}/]*
event		[^,{}/]+

num_dec		[0-9]+
num_hex		0x[a-fA-F0-9]+
num_raw_hex	[a-fA-F0-9]+
name		[a-zA-Z_*?][a-zA-Z0-9_*?]*
name_minus	[a-zA-Z_*?][a-zA-Z0-9\-_*?]*
modifier_event	[ukhpGH]+
modifier_bp	[rwx]{1,3}

%%

%{
	{
		int start_token;

		start_token = parse_events_get_extra(yyscanner);

		if (start_token == PE_START_TERMS)
			BEGIN(config);
		else if (start_token == PE_START_EVENTS)
			BEGIN(event);

		if (start_token) {
			parse_events_set_extra(NULL, yyscanner);
			return start_token;
		}
         }
%}

<event>{

{group}		{
			BEGIN(INITIAL); yyless(0);
		}

{event_pmu}	|
{event}		{
			str(yyscanner, PE_EVENT_NAME);
			BEGIN(INITIAL); yyless(0);
			return PE_EVENT_NAME;
		}

.		|
<<EOF>>		{
			BEGIN(INITIAL); yyless(0);
		}

}

cpu-cycles|cycles				{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_CPU_CYCLES); }
stalled-cycles-frontend|idle-cycles-frontend	{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_STALLED_CYCLES_FRONTEND); }
stalled-cycles-backend|idle-cycles-backend	{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_STALLED_CYCLES_BACKEND); }
instructions					{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_INSTRUCTIONS); }
cache-references				{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_CACHE_REFERENCES); }
cache-misses					{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_CACHE_MISSES); }
branch-instructions|branches			{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_BRANCH_INSTRUCTIONS); }
branch-misses					{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_BRANCH_MISSES); }
bus-cycles					{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_BUS_CYCLES); }
ref-cycles					{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_REF_CPU_CYCLES); }

    /*loongson3a2000 hardware events*/
    /*fetch*/
	/*
Inst-queue-empty    { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_INSTQ_EMPTY); }
Inst-queue-instructions { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_INSTQ_INSTRUCTIONS); }
Inst-queue-1 { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_INSTQ_1); }
Inst-queue-2 { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_INSTQ_2); }
Inst-queue-3 { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_INSTQ_3); }
Inst-queue-4 { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_INSTQ_4); }
Inst-queue-5 { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_INSTQ_5); }
Inst-queue-6 { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_INSTQ_6); }
Inst-queue-7 { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_INSTQ_7); }
Inst-queue-8 { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_INSTQ_8); }
Inst-queue-less-than-8 { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_INSTQ_LESSTHAN_8); }
Inst-queue-full { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_INSTQ_FULL); }
decode-instructions { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DECODE_INST); }
Loop-buffer-instructions { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_LOOP_BUFFER_INST); }
Loop-find { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_LOOP_FIND); }
Loop-trigger { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_LOOP_TRIGGER); }
Decode-branch-0 { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DECODE_BRANCH_0); }
Decode-branch-1 { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DECODE_BRANCH_1); }
Decode-branch-2 { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DECODE_BRANCH_2); }
Icache-misses { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_ICACHE_MISSES_BLOCK); }
BrBTB-taken-branch-misses { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_BRBTB_TAKEN_BRANCH_MISSES); }
Icache-replace/refill { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_ICACHE_REPLACE); }
Itlb-miss-tlb-hit { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_ITLB_MISS_TLB_HIT); }
Itlb-flushed { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_ITLB_FLUSHED); }
	*/
    /*rmap*/
	/*
Resource-alloc-blocked { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_RESOURCE_ALLOC_BLOCKED); }
GR-blocked { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_GR_BLOCKED); }
GR-pseudo-blocked { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_GR_PSEUDO_BLOCKED); }
FR-blocked { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FR_BLOCKED); }
FR-pseudo-blocked { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FR_PSEUDO_BLOCKED); }
FCR-blocked { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FCR_BLOCKED); }
FCR-pseudo-blocked { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FCR_PSEUDO_BLOCKED); }
ACC-blocked { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_ACC_BLOCKED); }
ACC-pseudo-blocked { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_ACC_PSEUDO_BLOCKED); }
DSPCTRL-blocked { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DSPCTRL_BLOCKED); }
DSPCTRL-pseudo-blocked { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DSPCTRL_PSEUDO_BLOCKED); }
BRQ-blocked { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_BRQ_BLOCKED); }
BRQ-pseudo-blocked { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_BRQ_PSEUDO_BLOCKED); }
FXQ-blocked { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FXQ_BLOCKED); }
FXQ-pseudo-blocked { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FXQ_PSEUDO_BLOCKED); }
FTQ-blocked { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FTQ_BLOCKED); }
FTQ-pseudo-blocked { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FTQ_PSEUDO_BLOCKED); }
MMQ-blocked { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MMQ_BLOCKED); }
MMQ-pseudo-blocked { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MMQ_PSEUDO_BLOCKED); }
CP0Q-blocked { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_CP0Q_BLOCKED); }
CP0Q-pseudo-blocked { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_CP0Q_PSEUDO_BLOCKED); }
ROQ-blocked { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_ROQ_BLOCKED); }
Nop-inst { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_NOP_INST); }
Regmap-issued { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_REGMAP_ISSUED); }
Exceptions { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_EXCEPTIONS); }
Branch-misses-overhead { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_BRANCH_MISSES_OVERHEAD); }
	*/
	/*roq*/
	/*
Alu-committed { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_ALU_COMMITTED); }
Falu-committed { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FALU_COMMITTED); }
Memory-swap-committed { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MEMORY_SWAP_COMMITTED); }
Load-committed { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_LOAD_COMMITTED); }
Store-committed { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_STORE_COMMITTED); }
LL-committed { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_LL_COMMITTED); }
SC-committed { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SC_COMMITTED); }
Unaligned-load-committed { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_UNALIGNED_LOAD_COMMITTED); }
Unaligned-store-committed { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_UNALIGNED_STORE_COMMITTED); }
Exceptions-and-interrupts { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_EXCEPTIONS_AND_INTERRUPTS); }
Interrupts { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_INTERRUPTS); }
ROQ-interrupt { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_ROQ_INTERRUPT); }
ROQ-interrupt-inst { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_ROQ_INTERRUPT_INST); }
Vm-exceptions { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VM_EXCEPTIONS); }
Address-fault-exceptions { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_ADDRESS_FAULT_EXCEPTIONS); }
Tlb-exceptions { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_TLB_EXCEPTIONS); }
Tlb-refill-exceptions { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_TLB_REFILL_EXCEPTIONS); }
Tlb-refill-handle-time { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_TLB_REFILL_HANDLE_TIME); }
Jump-register { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_JUMP_REGISTER); }
Jump-and-link { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_JUMP_AND_LINK); }
Branch-and-link { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_BRANCH_AND_LINK); }
Bht-branch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_BHT_BRANCH); }
Likely-branch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_LIKELY_BRANCH); }
Not-taken-branch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_NOT_TAKEN_BRANCH); }
Taken-branch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_TAKEN_BRANCH); }
Jump-register-misses { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_JUMP_REGISTER_MISSES); }
Jump-and-link-misses { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_JUMP_AND_LINK_MISSES); }
Branch-and-link-misses { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_BRANCH_AND_LINK_MISSES); }
Bht-branch-misses { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_BHT_BRANCH_MISSES); }
Likely-branch-misses { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_LIKELY_BRANCH_MISSES); }
Not-taken-misses { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_NOT_TAKEN_MISSES); }
Taken-misses { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_TAKEN_MISSES); }
    */
	/*fix*/
	/*
Fxq-no-issue { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FXQ_NO_ISSUE); }
Fxq-issue-operand { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FXQ_ISSUE_OPERAND); }
Fxq-fu0-operand { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FXQ_FU0_OPERAND); }
Fxq-fu1-operand { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FXQ_FU1_OPERAND); }
Fu0-fixed-mul { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FU0_FIXED_MUL); }
Fu0-fixed-div { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FU0_FIXED_DIV); }
Fu1-fixed-mul { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FU1_FIXED_MUL); }
Fu1-fixed-div { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FU1_FIXED_DIV); }
	*/
	/*float*/
	/*
Ftq-no-issue { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FTQ_NO_ISSUE); }
Ftq-issue-operand { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FTQ_ISSUE_OPERAND); }
Ftq-fu3-operand { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FTQ_FU3_OPERAND); }
Ftq-fu4-operand { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FTQ_FU4_OPERAND); }
Fu3-empty-fu4-full { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FU3_EMPTY_FU4_FULL); }
Fu4-empty-fu3-full { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FU4_EMPTY_FU3_FULL); }
Scalar-float-operand { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SCALAR_FLOAT_OPERAND); }
Gs-alu-inst { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_GS_ALU_INST); }
Fixed-vector-issue-64 { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FIXED_VECTOR_ISSUE_64); }
Vector-issue-128 { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VECTOR_ISSUE_128); }
Fixed-vector-issue-128 { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FIXED_VECTOR_ISSUE_128); }
Float-vector-issue-128 { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FLOAT_VECTOR_ISSUE_128); }
Vector-issue-256 { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VECTOR_ISSUE_256); }
Fixed-vector-issue-256 { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FIXED_VECTOR_ISSUE_256); }
Float-vector-issue-256 { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FLOAT_VECTOR_ISSUE_256); }
Fu3-vector-fixed-div { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FU3_VECTOR_FIXED_DIV); }
Fu3-float-div-sqrt { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FU3_FLOAT_DIV_SQRT); }
Fu4-vector-fixed-div { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FU4_VECTOR_FIXED_DIV); }
Fu4-float-div-sqrt { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FU4_FLOAT_DIV_SQRT); }
    */
	/*memory*/
	/*
Mmq-no-issue { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MMQ_NO_ISSUE); }
Mmq-issue-operand { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MMQ_ISSUE_OPERAND); }
Mmq-fu2-inst { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MMQ_FU2_INST); }
Mmq-fu5-inst { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MMQ_FU5_INST); }
Load-issue { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_LOAD_ISSUE); }
Store-issue { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_STORE_ISSUE); }
Src-float-mem-inst { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SRC_FLOAT_MEM_INST); }
Vector-mem-issue { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VECTOR_MEM_ISSUE); }
Fix-float-shift-issue { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FIX_FLOAT_SHIFT_ISSUE); }
Wait-first-block-cycles { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_WAIT_FIRST_BLOCK_CYCLES); }
Sync-block-cycles { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SYNC_BLOCK_CYCLES); }
Stall-issue-block-cycles { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_STALL_ISSUE_BLOCK_CYCLES); }
Software-prefetch-total { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SOFTWARE_PREFETCH_TOTAL); }
Dmemref-block-dcachewrite { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DMEMREF_BLOCK_DCACHEWRITE); }
Dmemref-bank-clash { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DMEMREF_BANK_CLASH); }
Refill-block-dmemref { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_REFILL_BLOCK_DMEMREF); }
Dcachewrite-no-cancel { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DCACHEWRITE_NO_CANCEL); }
Dcachewrite0-and-1-valid { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DCACHEWRITE0_AND_1_VALID); }
Sc-write-dcache { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SC_WRITE_DCACHE); }
Store-dcache-miss { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_STORE_DCACHE_MISS); }
Store-dcache-shared-miss { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_STORE_DCACHE_SHARED_MISS); }
    */
	/*cache2mem*/
	/*
Store-dcache-hit { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_STORE_DCACHE_HIT); }
Load-hit { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_LOAD_HIT); }
Fwdbus2 { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FWDBUS2); }
Fwdbus5 { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FWDBUS5); }
Fwdbus-total { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FWDBUS_TOTAL); }
Dwaitstore { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DWAITSTORE); }
Mispec { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISPEC); }
Dcachewrite-cancel { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DCACHEWRITE_CANCEL); }
Cp0q-dmemread { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_CP0Q_DMEMREAD); }
Cp0q-duncache { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_CP0Q_DUNCACHE); }
Resbus2-occupy-resbus5 { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_RESBUS2_OCCUPY_RESBUS5); }
Sw-prefetch-l1dcache-hit { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SW_PREFETCH_L1DCACHE_HIT); }
Store-sw-prefetch-l1dcache-hit { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_STORE_SW_PREFETCH_L1DCACHE_HIT); }
Store-sw-prefetch-l1dcache-miss { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_STORE_SW_PREFETCH_L1DCACHE_MISS); }
Load-sw-prefetch-l1dcache-hit { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_LOAD_SW_PREFETCH_L1DCACHE_HIT); }
Load-sw-prefetch-l1dcache-miss { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_LOAD_SW_PREFETCH_L1DCACHE_MISS); }
Store-l1dcache-miss-share-state { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_STORE_L1DCACHE_MISS_SHARE_STATE); }
Specfwdbus2 { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SPECFWDBUS2); }
Specfwdbus5 { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SPECFWDBUS5); }
Specfwdbus-total { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SPECFWDBUS2_TOTAL); }
Data-load-vcache-access-req { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DATA_LOAD_VCACHE_ACCESS_REQ); }
Data-store-vcache-access-req { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DATA_STORE_VCACHE_ACCESS_REQ); }
Data-vcache-access-req { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DATA_VCACHE_ACCESS_REQ); }
Inst-vcache-access-req { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_INST_VCACHE_ACCESS_REQ); }
Vcache-access { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VCACHE_ACCESS); }
Sw-prefetch-access-vcache { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SW_PREFETCH_ACCESS_VCACHE); }
Vcache-load-hit { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VCACHE_LOAD_HIT); }
Vcache-store-hit { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VCACHE_STORE_HIT); }
Vcache-data-hit { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VCACHE_DATA_HIT); }
Vcache-inst-hit { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VCACHE_INST_HIT); }
Vcache-hit { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VCACHE_HIT); }
Vcache-sw-prefetch-hit { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VCACHE_SW_PREFETCH_HIT); }
Vcache-load-miss { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VCACHE_LOAD_MISS); }
Vcache-store-miss { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VCACHE_STORE_MISS); }
Vcache-data-miss { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VCACHE_DATA_MISS); }
Vcache-inst-miss { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VCACHE_INST_MISS); }
Vcache-miss { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VCACHE_MISS); }
Vcache-sw-prefetch-miss { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VCACHE_SW_PREFETCH_MISS); }
Vcache-extreq-invalid { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VCACHE_EXTREQ_INVALID); }
Vcache-wtbk-degrade { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VCACHE_WTBK_DEGRADE); }
Vcache-inv-invalid { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VCACHE_INV_INVALID); }
Vcache-invwtbk-invalid { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VCACHE_INVWTBK_INVALID); }
Ar-request-issue { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_AR_REQUEST_ISSUE); }
Aw-request-issue { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_AW_REQUEST_ISSUE); }
Aw-data-request { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_AW_DATA_REQUEST); }
Ar-blocked-aw-undone { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_AR_BLOCKED_AW_UNDONE); }
Missq-wtbk-request { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_WTBK_REQUEST); }
Missq-invwtbk-request { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_INVWTBK_REQUEST); }
Missq-inv-request { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_INV_REQUEST); }
Missq-inv-class-request { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_INV_CLASS_REQUEST); }
Refill-total { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_REFILL_TOTAL); }
Refill-icache { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_REFILL_ICACHE); }
Refill-dcache { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_REFILL_DCACHE); }
Replace-refill { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_REPLACE_REFILL); }
Refill-dcache-shared { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_REFILL_DCACHE_SHARED); }
Refill-dcache-exc { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_REFILL_DCACHE_EXC); }
Refill-data-total { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_REFILL_DATA_TOTAL); }
Refill-inst-total { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_REFILL_INST_TOTAL); }
Dcache-replace-valid-block { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DCACHE_REPLACE_VALID_BLOCK); }
Dcache-replace-shared-block { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DCACHE_REPLACE_SHARED_BLOCK); }
Dcache-replace-exc-block { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DCACHE_REPLACE_EXC_BLOCK); }
Dcache-replace-dirty-block { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DCACHE_REPLACE_DIRTY_BLOCK); }
Icache-replace-valid-data { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_ICACHE_REPLACE_VALID_DATA); }
Vcache-replace { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VCACHE_REPLACE); }
Vcache-replace-valid-block { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VCACHE_REPLACE_VALID_BLOCK); }
Vcache-replace-shared-block { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VCACHE_REPLACE_SHARED_BLOCK); }
Vcache-replace-exc-block { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VCACHE_REPLACE_EXC_BLOCK); }
Vcache-replace-dirty-block { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VCACHE_REPLACE_DIRTY_BLOCK); }
Vcache-replace-valid-dcblock { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VCACHE_REPLACE_VALID_DCBLOCK); }
Vcache-replace-valid-icblock { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_VCACHE_REPLACE_VALID_ICBLOCK); }
Scache-load-not-return { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SCACHE_LOAD_NOT_RETURN); }
Scache-store-not-return { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SCACHE_STORE_NOT_RETURN); }
Scache-icachereq-not-return { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SCACHE_ICACHEREQ_NOT_RETURN); }
Scread-total { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SCREAD_TOTAL); }
Scread-load { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SCREAD_LOAD); }
Scread-store { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SCREAD_STORE); }
Scread-data-access { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SCREAD_DATA_ACCESS); }
Scread-inst-access { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SCREAD_INST_ACCESS); }
Scread-nprefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SCREAD_NPREFETCH); }
Scread-nprefetch-data-load { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SCREAD_NPREFETCH_DATA_LOAD); }
Scread-nprefetch-store { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SCREAD_NPREFETCH_STORE); }
Scread-nprefetch-data-access { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SCREAD_NPREFETCH_DATA_ACCESS); }
Scread-nprefetch-inst-access { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SCREAD_NPREFETCH_INST_ACCESS); }
Scread-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SCREAD_PREFETCH); }
Scread-load-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SCREAD_LOAD_PREFETCH); }
Scread-store-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SCREAD_STORE_PREFETCH); }
Scread-prefetch-data-access { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SCREAD_PREFETCH_DATA_ACCESS); }
Scread-prefetch-inst-access { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SCREAD_PREFETCH_INST_ACCESS); }
Missq-sw-prefetch-request { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_SW_PREFETCH_REQUEST); }
Missq-scwrite { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_SCWRITE); }
Missq-replace-scwrite { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_REPLACE_SCWRITE); }
Missq-invalid-scwrite { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_INVALID_SCWRITE); }
Missq-replace-valid-scwrite { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_REPLACE_VALID_SCWRITE); }
Missq-accept-req { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_ACCEPT_REQ); }
Missq-accept-load { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_ACCEPT_LOAD); }
Missq-accept-store { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_ACCEPT_SIORE); }
Missq-data-access { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_DATA_ACCESS); }
Missq-inst-access { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_INST_ACCESS); }
Missq-non-empty { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_NON_EMPTY); }
Missq-common-access-occupy { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_COMMON_ACCESS_OCCUPY); }
Missq-fetch-access-occupy { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_FETCH_ACCESS_OCCUPY); }
Missq-external-req-occupy { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_EXTERNAL_REQ_OCCUPY); }
Missq-prefetch-req-occupy { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_PREFETCH_REQ_OCCUPY); }
Missq-occupy-cycles { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_OCCUPY_CYCLES); }
Missq-common-access-cycles { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_COMMON_ACCESS_CYCLES); }
Missq-fetch-access-cycles { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_FETCH_ACCESS_CYCLES); }
Missq-external-req-cycles { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_EXTERNAL_REQ_CYCLES); }
Missq-prefetch-req-cycles { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_PREFETCH_REQ_CYCLES); }
Missq-full-count { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_FULL_COUNT); }
Load-missq-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_LOAD_MISSQ_PREFETCH); }
Load-missq-pre-scref { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_LOAD_MISSQ_PRE_SCREF); }
Load-missq-pre-rdy { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_LOAD_MISSQ_PRE_RDY); }
Load-missq-pre-wait { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_LOAD_MISSQ_PRE_WAIT); }
Store-missq-pre-scref-load { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_STORE_MISSQ_PRE_SCREF_LOAD); }
Store-missq-pre-rdy-shard { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_STORE_MISSQ_PRE_RDY_SHARD); }
Store-missq-pre-wait-load { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_STORE_MISSQ_PRE_WAIT_LOAD); }
Store-missq-pre-scref-store { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_STORE_MISSQ_PRE_SCREF_STORE); }
Store-missq-pre-rdy-exc { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_STORE_MISSQ_PRE_RDY_EXC); }
Store-missq-pre-wait-store { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_STORE_MISSQ_PRE_WAIT_STORE); }
Store-missq-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_STORE_MISSQ_PREFETCH); }
Store-missq-valid-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_STORE_MISSQ_VALID_PREFETCH); }
All-req-missq-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_ALL_REQ_MISSQ_PREFETCH); }
All-req-missq-pre-scref { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_ALL_REQ_MISSQ_PRE_SCREF); }
All-req-missq-pre-rdy { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_ALL_REQ_MISSQ_PRE_RDY); }
All-req-missq-pre-wait { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_ALL_REQ_MISSQ_PRE_WAIT); }
Fetch-missq-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FETCH_MISSQ_PREFETCH); }
Fetch-pre-scref { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FETCH_PRE_SCREF); }
Fetch-pre-rdy { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FETCH_PRE_RDY); }
Fetch-pre-wait { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FETCH_PRE_WAIT); }
Data-fetch-pre-rdy { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DATA_FETCH_PRE_RDY); }
Data-fetch-pre-wait { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DATA_FETCH_PRE_WAIT); }
Hw-load-pre-scache-cancel { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_HW_LOAD_PRE_SCACHE_CANCEL); }
Hw-store-pre-scache-cancel { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_HW_STORE_PRE_SCACHE_CANCEL); }
Hw-data-pre-scache-cancel { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_HW_DATA_PRE_SCACHE_CANCEL); }
Hw-fetch-pre-scache-cancel { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_HW_FETCH_PRE_SCACHE_CANCEL); }
Hw-prefetch-scache-cancel { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_HW_PREFETCH_SCACHE_CANCEL); }
Hw-load-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_HW_LOAD_PREFETCH); }
Hw-store-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_HW_STORE_PREFETCH); }
Hw-data-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_HW_DATA_PREFETCH); }
Hw-inst-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_HW_INST_PREFETCH); }
Hw-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_HW_PREFETCH); }
Tagged-load-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_TAGGED_LOAD_PREFETCH); }
Miss-load-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISS_LOAD_PREFETCH); }
Tagged-store-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_TAGGED_STORE_PREFETCH); }
Miss-store-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISS_STORE_PREFETCH); }
Tagged-data-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_TAGGED_DATA_PREFETCH); }
Miss-data-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISS_DATA_PREFETCH); }
Tagged-inst-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_TAGGED_INST_PREFETCH); }
Miss-inst-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISS_INST_PREFETCH); }
Tagged-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_TAGGED_PREFETCH); }
Miss-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISS_PREFETCH); }
Missq-ac-load-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_AC_LOAD_PREFETCH); }
Missq-ac-store-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_AC_STORE_PREFETCH); }
Missq-ac-data-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_AC_DATA_PREFETCH); }
Missq-ac-inst-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_AC_INST_PREFETCH); }
Missq-ac-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_AC_PREFETCH); }
Scache-load-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SCACHE_LOAD_PREFETCH); }
Scache-store-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SCACHE_STORE_PREFETCH); }
Scache-data-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SCACHE_DATA_PREFETCH); }
Scache-inst-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SCACHE_INST_PREFETCH); }
Scache-valid-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_SCACHE_VALID_PREFETCH); }
Pre-rdy-load-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_RDY_LOAD_PREFETCH); }
Pre-rdy-store-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_RDY_STORE_PREFETCH); }
Pre-rdy-data-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_RDY_DATA_PREFETCH); }
Pre-rdy-inst-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_RDY_INST_PREFETCH); }
Pre-rdy-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_RDY_PREFETCH); }
Pre-rdy-load-request { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_RDY_LOAD_REQUEST); }
Pre-rdy-store-request { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_RDY_STORE_REQUEST); }
Pre-rdy-data-request { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_RDY_DATA_REQUEST); }
Pre-rdy-inst-request { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_RDY_INST_REQUEST); }
Pre-rdy-request { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_RDY_REQUEST); }
Pre-scref-hit-load-req { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_SCREF_HIT_LOAD_REQ); }
Pre-scref-hit-store-req { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_SCREF_HIT_STORE_REQ); }
Pre-scref-hit-data-req { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_SCREF_HIT_DATA_REQ); }
Pre-scref-hit-inst-req { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_SCREF_HIT_INST_REQ); }
Pre-scref-hit-prefetch-req { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_SCREF_HIT_PREFETCH_REQ); }
Pre-scref-hit-load-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_SCREF_HIT_LOAD_PREFETCH); }
Pre-scref-hit-store-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_SCREF_HIT_STORE_PREFETCH); }
Pre-scref-rdy-data-req { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_SCREF_RDY_DATA_REQ); }
Pre-scref-rdy-inst-req { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_SCREF_RDY_INST_REQ); }
Pre-scref-rdy-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_SCREF_RDY_PREFETCH); }
Pre-scref-miss-load { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_SCREF_MISS_LOAD); }
Pre-wait-hit-load-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_WAIT_HIT_LOAD_PREFETCH); }
Pre-wait-hit-store-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_WAIT_HIT_STORE_PREFETCH); }
Pre-wait-hit-data-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_WAIT_HIT_DATA_PREFETCH); }
Pre-wait-hit-inst-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_WAIT_HIT_INST_PREFETCH); }
Pre-wait-hit-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PRE_WAIT_HIT_PREFETCH); }
Missq-replace-pre-wait-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_REPLACE_PRE_WAIT_PREFEETCH); }
Missq-replace-pre-rdy-prefetch { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MISSQ_REPLACE_PRE_RDY_PREFEETCH); }
Prefetch-inv { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PREFETCH_INV); }
Load-prefetch-occupy { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_LOAD_PREFETCH_OCCUPY); }
Load-prefetch-isoccupy { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_LOAD_PREFETCH_ISOCCUPY); }
Store-prefetch-occupy { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_STORE_PREFETCH_OCCUPY); }
Store-prefetch-isoccupy { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_STORE_PREFETCH_ISOCCUPY); }
data-prefetch-occupy { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DATA_PREFETCH_OCCUPY); }
data-prefetch-isoccupy { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DATA_PREFETCH_ISOCCUPY); }
inst-prefetch-occupy { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_INST_PREFETCH_OCCUPY); }
inst-prefetch-isoccupy { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_INST_PREFETCH_ISOCCUPY); }
Load-pre-scref-pre-rdy-hit { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_LOAD_PRE_SCREF_PRE_RDY_HIT); }
Store-pre-scref-pre-rdy-hit { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_STORE_PRE_SCREF_PRE_RDY_HIT); }
Data-pre-scref-pre-rdy-hit { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DATA_PRE_SCREF_PRE_RDY_HIT); }
Inst-pre-scref-pre-rdy-hit { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_INST_PRE_SCREF_PRE_RDY_HIT); }
Prefetch-pre-scref-pre-rdy-hit { return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_PREFETCH_PRE_SCREF_PRE_RDY_HIT); }

    /*loongson3a1000 hardware events*/
	/*
JR-instructions					{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_JUMP_INSTRUCTIONS); }
JR(RS=31)-instructions|JR31-instructions	{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_JR31_INSTRUCTIONS); }
instruction-cache-misses|icache-misses		{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_ICACHE_MISSES); }
ALU1-issued|ALU1-operation-issued		{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_ALU1_ISSUED); }
memory-read/write-issued|memory-issued		{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MEM_ISSUED); }
float-ALU1-operation-issued|FALU1-issued	{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FALU1_ISSUED); }
BHT-branch-instructions 			{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_BHT_BRANCH_INSTRUCTIONS); }
memory-read|read-from-primary-memory		{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MEM_READ); }
fix-queue-full					{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FQUEUE_FULL); }
reorder-queue-full				{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_ROQ_FULL); }
CP0-queue-full					{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_CP0_QUEUE_FULL); }
TLB-refill-exception				{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_TLB_REFILL); }
exceptions					{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_EXCEPTION); }
internal-exceptions				{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_INTERNAL_EXCEPTION); }

JR-mispredicted					{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_JR_MISPREDICTED); }
JR31-mispredicted				{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_JR31_MISPREDICTED); }
data-cache-misses				{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_DCACHE_MISSES); }
ALU2-operation-issued|ALU2-issued		{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_ALU2_ISSUED); }
FALU2-operation-issued|FALU2-issued		{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FALU2_ISSUED); }
uncached-accesses				{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_UNCACHED_ACCESS); }
BHT-mispredicted 				{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_BHT_MISPREDICTED); }
write-to-memory					{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_MEM_WRITE); }
float-queue-full				{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_FTQ_FULL); }
branch-queue-full				{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_BRANCH_QUEUE_FULL); }
instruction-TLB-misses|ITLB-misses		{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_ITLB_MISSES); }
total-exceptions				{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_TOTAL_EXCEPTIONS); }
load-speculation-misses				{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_LOAD_SPECULATION_MISSES); }
CP0-queue-forward-valid				{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_CP0Q_FORWARD_VALID); }
	*/
cpu-clock					{ return sym(yyscanner, PERF_TYPE_SOFTWARE, PERF_COUNT_SW_CPU_CLOCK); }
task-clock					{ return sym(yyscanner, PERF_TYPE_SOFTWARE, PERF_COUNT_SW_TASK_CLOCK); }
page-faults|faults				{ return sym(yyscanner, PERF_TYPE_SOFTWARE, PERF_COUNT_SW_PAGE_FAULTS); }
minor-faults					{ return sym(yyscanner, PERF_TYPE_SOFTWARE, PERF_COUNT_SW_PAGE_FAULTS_MIN); }
major-faults					{ return sym(yyscanner, PERF_TYPE_SOFTWARE, PERF_COUNT_SW_PAGE_FAULTS_MAJ); }
context-switches|cs				{ return sym(yyscanner, PERF_TYPE_SOFTWARE, PERF_COUNT_SW_CONTEXT_SWITCHES); }
cpu-migrations|migrations			{ return sym(yyscanner, PERF_TYPE_SOFTWARE, PERF_COUNT_SW_CPU_MIGRATIONS); }
alignment-faults				{ return sym(yyscanner, PERF_TYPE_SOFTWARE, PERF_COUNT_SW_ALIGNMENT_FAULTS); }
emulation-faults				{ return sym(yyscanner, PERF_TYPE_SOFTWARE, PERF_COUNT_SW_EMULATION_FAULTS); }

L1-dcache|l1-d|l1d|L1-data		|
L1-icache|l1-i|l1i|L1-instruction	|
LLC|L2					|
dTLB|d-tlb|Data-TLB			|
iTLB|i-tlb|Instruction-TLB		|
branch|branches|bpu|btb|bpc		|
node					{ return str(yyscanner, PE_NAME_CACHE_TYPE); }

load|loads|read				|
store|stores|write			|
prefetch|prefetches			|
speculative-read|speculative-load	|
refs|Reference|ops|access		|
misses|miss				{ return str(yyscanner, PE_NAME_CACHE_OP_RESULT); }

<config>{
config			{ return term(yyscanner, PARSE_EVENTS__TERM_TYPE_CONFIG); }
config1			{ return term(yyscanner, PARSE_EVENTS__TERM_TYPE_CONFIG1); }
config2			{ return term(yyscanner, PARSE_EVENTS__TERM_TYPE_CONFIG2); }
name			{ return term(yyscanner, PARSE_EVENTS__TERM_TYPE_NAME); }
period			{ return term(yyscanner, PARSE_EVENTS__TERM_TYPE_SAMPLE_PERIOD); }
branch_type		{ return term(yyscanner, PARSE_EVENTS__TERM_TYPE_BRANCH_SAMPLE_TYPE); }
,			{ return ','; }
"/"			{ BEGIN(INITIAL); return '/'; }
{name_minus}		{ return str(yyscanner, PE_NAME); }
}

mem:			{ BEGIN(mem); return PE_PREFIX_MEM; }
r{num_raw_hex}		{ return raw(yyscanner); }
{num_dec}		{ return value(yyscanner, 10); }
{num_hex}		{ return value(yyscanner, 16); }

{modifier_event}	{ return str(yyscanner, PE_MODIFIER_EVENT); }
{name}			{ return str(yyscanner, PE_NAME); }
"/"			{ BEGIN(config); return '/'; }
-			{ return '-'; }
,			{ BEGIN(event); return ','; }
:			{ return ':'; }
"{"			{ BEGIN(event); return '{'; }
"}"			{ return '}'; }
=			{ return '='; }
\n			{ }

<mem>{
{modifier_bp}		{ return str(yyscanner, PE_MODIFIER_BP); }
:			{ return ':'; }
{num_dec}		{ return value(yyscanner, 10); }
{num_hex}		{ return value(yyscanner, 16); }
	/*
	 * We need to separate 'mem:' scanner part, in order to get specific
	 * modifier bits parsed out. Otherwise we would need to handle PE_NAME
	 * and we'd need to parse it manually. During the escape from <mem>
	 * state we need to put the escaping char back, so we dont miss it.
	 */
.			{ unput(*yytext); BEGIN(INITIAL); }
	/*
	 * We destroy the scanner after reaching EOF,
	 * but anyway just to be sure get back to INIT state.
	 */
<<EOF>>			{ BEGIN(INITIAL); }
}

%%

int parse_events_wrap(void *scanner __maybe_unused)
{
	return 1;
}

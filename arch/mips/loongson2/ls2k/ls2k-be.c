/*
 *  Copyright (C) 2014, Loongson Technology Corporation Limited, Inc.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 */
#include <linux/module.h>
#include <asm/traps.h>
#include <asm/ptrace.h>
#include <asm/inst.h>
#include <asm/branch.h>

/*
 * fix loongson 2h pcie bus error
 */
static int ls2h_be_handler(struct pt_regs *regs, int is_fixup)
{
	int action = MIPS_BE_FATAL;
	union mips_instruction insn;
	unsigned int __user *epc;

	if (is_fixup)
		return MIPS_BE_FIXUP;

	epc = (unsigned int __user *)exception_epc(regs);
	if (likely(__get_user(insn.word, epc) >= 0)) {
		switch (insn.i_format.opcode) {
		case lw_op:
			compute_return_epc(regs);
			regs->regs[insn.i_format.rt] = -1;
			action = MIPS_BE_DISCARD;
			break;
		case sw_op:
			compute_return_epc(regs);
			action = MIPS_BE_DISCARD;
			break;
		}
	}

	return action;
}

static int ls2h_be_init(void)
{
	board_be_handler = ls2h_be_handler;
	return 0;
}

arch_initcall(ls2h_be_init);

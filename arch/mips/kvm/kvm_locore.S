/*
* This file is subject to the terms and conditions of the GNU General Public
* License.  See the file "COPYING" in the main directory of this archive
* for more details.
*
* Main entry point for the guest, exception handling.
*
* Copyright (C) 2012  MIPS Technologies, Inc.  All rights reserved.
* Authors: Sanjay Lal <sanjayl@kymasys.com>
*/

#ifndef CONFIG_KVM_MIPS_LOONGSON3
#include <asm/asm.h>
#include <asm/asmmacro.h>
#include <asm/regdef.h>
#include <asm/mipsregs.h>
#include <asm/stackframe.h>
#include <asm/asm-offsets.h>


#define _C_LABEL(x)     x
#define MIPSX(name)     mips32_ ## name
#define CALLFRAME_SIZ   32

/*
 * VECTOR
 *  exception vector entrypoint
 */
#define VECTOR(x, regmask)      \
    .ent    _C_LABEL(x),0;      \
    EXPORT(x);

#define VECTOR_END(x)      \
    EXPORT(x);

/* Overload, Danger Will Robinson!! */
#define PT_HOST_ASID        PT_BVADDR
#define PT_HOST_USERLOCAL   PT_EPC

#define CP0_DDATA_LO        $28,3
#define CP0_EBASE           $15,1

#define CP0_INTCTL          $12,1
#define CP0_SRSCTL          $12,2
#define CP0_SRSMAP          $12,3
#define CP0_HWRENA          $7,0

/* Resume Flags */
#define RESUME_FLAG_HOST        (1<<1)  /* Resume host? */

#define RESUME_GUEST            0
#define RESUME_HOST             RESUME_FLAG_HOST

/*
 * __kvm_mips_vcpu_run: entry point to the guest
 * a0: run
 * a1: vcpu
 */

FEXPORT(__kvm_mips_vcpu_run)
    .set    push
    .set    noreorder
    .set    noat

    /* k0/k1 not being used in host kernel context */
	addiu  		k1,sp, -PT_SIZE
    LONG_S	    $0, PT_R0(k1)
    LONG_S     	$1, PT_R1(k1)
    LONG_S     	$2, PT_R2(k1)
    LONG_S     	$3, PT_R3(k1)

    LONG_S     	$4, PT_R4(k1)
    LONG_S     	$5, PT_R5(k1)
    LONG_S     	$6, PT_R6(k1)
    LONG_S     	$7, PT_R7(k1)

    LONG_S     	$8,  PT_R8(k1)
    LONG_S     	$9,  PT_R9(k1)
    LONG_S     	$10, PT_R10(k1)
    LONG_S     	$11, PT_R11(k1)
    LONG_S     	$12, PT_R12(k1)
    LONG_S     	$13, PT_R13(k1)
    LONG_S     	$14, PT_R14(k1)
    LONG_S     	$15, PT_R15(k1)
    LONG_S     	$16, PT_R16(k1)
    LONG_S     	$17, PT_R17(k1)

    LONG_S     	$18, PT_R18(k1)
    LONG_S     	$19, PT_R19(k1)
    LONG_S     	$20, PT_R20(k1)
    LONG_S     	$21, PT_R21(k1)
    LONG_S     	$22, PT_R22(k1)
    LONG_S     	$23, PT_R23(k1)
    LONG_S     	$24, PT_R24(k1)
    LONG_S     	$25, PT_R25(k1)

	/* XXXKYMA k0/k1 not saved, not being used if we got here through an ioctl() */

    LONG_S     	$28, PT_R28(k1)
    LONG_S     	$29, PT_R29(k1)
    LONG_S     	$30, PT_R30(k1)
    LONG_S     	$31, PT_R31(k1)

    /* Save hi/lo */
	mflo		v0
	LONG_S		v0, PT_LO(k1)
	mfhi   		v1
	LONG_S		v1, PT_HI(k1)

	/* Save host status */
	mfc0		v0, CP0_STATUS
	LONG_S		v0, PT_STATUS(k1)

	/* Save host ASID, shove it into the BVADDR location */
	mfc0 		v1,CP0_ENTRYHI
	andi		v1, 0xff
	LONG_S		v1, PT_HOST_ASID(k1)

    /* Save DDATA_LO, will be used to store pointer to vcpu */
    mfc0        v1, CP0_DDATA_LO
    LONG_S      v1, PT_HOST_USERLOCAL(k1)

    /* DDATA_LO has pointer to vcpu */
    mtc0        a1,CP0_DDATA_LO

    /* Offset into vcpu->arch */
	addiu		k1, a1, VCPU_HOST_ARCH

    /* Save the host stack to VCPU, used for exception processing when we exit from the Guest */
    LONG_S      sp, VCPU_HOST_STACK(k1)

    /* Save the kernel gp as well */
    LONG_S      gp, VCPU_HOST_GP(k1)

	/* Setup status register for running the guest in UM, interrupts are disabled */
	li			k0,(ST0_EXL | KSU_USER| ST0_BEV)
	mtc0		k0,CP0_STATUS
    ehb

    /* load up the new EBASE */
    LONG_L      k0, VCPU_GUEST_EBASE(k1)
    mtc0        k0,CP0_EBASE

    /* Now that the new EBASE has been loaded, unset BEV, set interrupt mask as it was
     * but make sure that timer interrupts are enabled
     */
    li          k0,(ST0_EXL | KSU_USER | ST0_IE)
    andi        v0, v0, ST0_IM
    or          k0, k0, v0
    mtc0        k0,CP0_STATUS
    ehb


	/* Set Guest EPC */
	LONG_L		t0, VCPU_PC(k1)
	mtc0		t0, CP0_EPC

FEXPORT(__kvm_mips_load_asid)
    /* Set the ASID for the Guest Kernel */
    sll         t0, t0, 1                       /* with kseg0 @ 0x40000000, kernel */
                                                /* addresses shift to 0x80000000 */
    bltz        t0, 1f                          /* If kernel */
	addiu       t1, k1, VCPU_GUEST_KERNEL_ASID  /* (BD)  */
    addiu       t1, k1, VCPU_GUEST_USER_ASID    /* else user */
1:
    /* t1: contains the base of the ASID array, need to get the cpu id  */
    LONG_L      t2, TI_CPU($28)             /* smp_processor_id */
    sll         t2, t2, 2                   /* x4 */
    addu        t3, t1, t2
    LONG_L      k0, (t3)
    andi        k0, k0, 0xff
	mtc0		k0,CP0_ENTRYHI
    ehb

    /* Disable RDHWR access */
    mtc0    zero,  CP0_HWRENA

    /* Now load up the Guest Context from VCPU */
    LONG_L     	$1, VCPU_R1(k1)
    LONG_L     	$2, VCPU_R2(k1)
    LONG_L     	$3, VCPU_R3(k1)

    LONG_L     	$4, VCPU_R4(k1)
    LONG_L     	$5, VCPU_R5(k1)
    LONG_L     	$6, VCPU_R6(k1)
    LONG_L     	$7, VCPU_R7(k1)

    LONG_L     	$8,  VCPU_R8(k1)
    LONG_L     	$9,  VCPU_R9(k1)
    LONG_L     	$10, VCPU_R10(k1)
    LONG_L     	$11, VCPU_R11(k1)
    LONG_L     	$12, VCPU_R12(k1)
    LONG_L     	$13, VCPU_R13(k1)
    LONG_L     	$14, VCPU_R14(k1)
    LONG_L     	$15, VCPU_R15(k1)
    LONG_L     	$16, VCPU_R16(k1)
    LONG_L     	$17, VCPU_R17(k1)
    LONG_L     	$18, VCPU_R18(k1)
    LONG_L     	$19, VCPU_R19(k1)
    LONG_L     	$20, VCPU_R20(k1)
    LONG_L     	$21, VCPU_R21(k1)
    LONG_L     	$22, VCPU_R22(k1)
    LONG_L     	$23, VCPU_R23(k1)
    LONG_L     	$24, VCPU_R24(k1)
    LONG_L     	$25, VCPU_R25(k1)

    /* k0/k1 loaded up later */

    LONG_L     	$28, VCPU_R28(k1)
    LONG_L     	$29, VCPU_R29(k1)
    LONG_L     	$30, VCPU_R30(k1)
    LONG_L     	$31, VCPU_R31(k1)

    /* Restore hi/lo */
	LONG_L		k0, VCPU_LO(k1)
	mtlo		k0

	LONG_L		k0, VCPU_HI(k1)
	mthi   		k0

FEXPORT(__kvm_mips_load_k0k1)
	/* Restore the guest's k0/k1 registers */
    LONG_L     	k0, VCPU_R26(k1)
    LONG_L     	k1, VCPU_R27(k1)

    /* Jump to guest */
	eret
	.set	pop

VECTOR(MIPSX(exception), unknown)
/*
 * Find out what mode we came from and jump to the proper handler.
 */
    .set    push
	.set	noat
    .set    noreorder
    mtc0    k0, CP0_ERROREPC    #01: Save guest k0
    ehb                         #02:

    mfc0    k0, CP0_EBASE       #02: Get EBASE
    srl     k0, k0, 10          #03: Get rid of CPUNum
    sll     k0, k0, 10          #04
    LONG_S  k1, 0x3000(k0)      #05: Save k1 @ offset 0x3000
    addiu   k0, k0, 0x2000      #06: Exception handler is installed @ offset 0x2000
	j	k0				        #07: jump to the function
	nop				        	#08: branch delay slot
	.set	push
VECTOR_END(MIPSX(exceptionEnd))
.end MIPSX(exception)

/*
 * Generic Guest exception handler. We end up here when the guest
 * does something that causes a trap to kernel mode.
 *
 */
NESTED (MIPSX(GuestException), CALLFRAME_SIZ, ra)
    .set    push
    .set    noat
    .set    noreorder

    /* Get the VCPU pointer from DDTATA_LO */
    mfc0        k1, CP0_DDATA_LO
	addiu		k1, k1, VCPU_HOST_ARCH

    /* Start saving Guest context to VCPU */
    LONG_S  $0, VCPU_R0(k1)
    LONG_S  $1, VCPU_R1(k1)
    LONG_S  $2, VCPU_R2(k1)
    LONG_S  $3, VCPU_R3(k1)
    LONG_S  $4, VCPU_R4(k1)
    LONG_S  $5, VCPU_R5(k1)
    LONG_S  $6, VCPU_R6(k1)
    LONG_S  $7, VCPU_R7(k1)
    LONG_S  $8, VCPU_R8(k1)
    LONG_S  $9, VCPU_R9(k1)
    LONG_S  $10, VCPU_R10(k1)
    LONG_S  $11, VCPU_R11(k1)
    LONG_S  $12, VCPU_R12(k1)
    LONG_S  $13, VCPU_R13(k1)
    LONG_S  $14, VCPU_R14(k1)
    LONG_S  $15, VCPU_R15(k1)
    LONG_S  $16, VCPU_R16(k1)
    LONG_S  $17,VCPU_R17(k1)
    LONG_S  $18, VCPU_R18(k1)
    LONG_S  $19, VCPU_R19(k1)
    LONG_S  $20, VCPU_R20(k1)
    LONG_S  $21, VCPU_R21(k1)
    LONG_S  $22, VCPU_R22(k1)
    LONG_S  $23, VCPU_R23(k1)
    LONG_S  $24, VCPU_R24(k1)
    LONG_S  $25, VCPU_R25(k1)

    /* Guest k0/k1 saved later */

    LONG_S  $28, VCPU_R28(k1)
    LONG_S  $29, VCPU_R29(k1)
    LONG_S  $30, VCPU_R30(k1)
    LONG_S  $31, VCPU_R31(k1)

    /* We need to save hi/lo and restore them on
     * the way out
     */
    mfhi    t0
    LONG_S  t0, VCPU_HI(k1)

    mflo    t0
    LONG_S  t0, VCPU_LO(k1)

    /* Finally save guest k0/k1 to VCPU */
    mfc0    t0, CP0_ERROREPC
    LONG_S  t0, VCPU_R26(k1)

    /* Get GUEST k1 and save it in VCPU */
    la      t1, ~0x2ff
    mfc0    t0, CP0_EBASE
    and     t0, t0, t1
    LONG_L  t0, 0x3000(t0)
    LONG_S  t0, VCPU_R27(k1)

    /* Now that context has been saved, we can use other registers */

    /* Restore vcpu */
    mfc0        a1, CP0_DDATA_LO
    move        s1, a1

   /* Restore run (vcpu->run) */
    LONG_L      a0, VCPU_RUN(a1)
    /* Save pointer to run in s0, will be saved by the compiler */
    move        s0, a0


    /* Save Host level EPC, BadVaddr and Cause to VCPU, useful to process the exception */
    mfc0    k0,CP0_EPC
    LONG_S  k0, VCPU_PC(k1)

    mfc0    k0, CP0_BADVADDR
    LONG_S  k0, VCPU_HOST_CP0_BADVADDR(k1)

    mfc0    k0, CP0_CAUSE
    LONG_S  k0, VCPU_HOST_CP0_CAUSE(k1)

    mfc0    k0, CP0_ENTRYHI
    LONG_S  k0, VCPU_HOST_ENTRYHI(k1)

    /* Now restore the host state just enough to run the handlers */

    /* Swtich EBASE to the one used by Linux */
    /* load up the host EBASE */
    mfc0        v0, CP0_STATUS

    .set at
	or          k0, v0, ST0_BEV
    .set noat

    mtc0        k0, CP0_STATUS
    ehb

    LONG_L      k0, VCPU_HOST_EBASE(k1)
    mtc0        k0,CP0_EBASE


    /* Now that the new EBASE has been loaded, unset BEV and KSU_USER */
    .set at
	and         v0, v0, ~(ST0_EXL | KSU_USER | ST0_IE)
    or          v0, v0, ST0_CU0
    .set noat
    mtc0        v0, CP0_STATUS
    ehb

    /* Load up host GP */
    LONG_L  gp, VCPU_HOST_GP(k1)

    /* Need a stack before we can jump to "C" */
    LONG_L  sp, VCPU_HOST_STACK(k1)

    /* Saved host state */
    addiu   sp,sp, -PT_SIZE

    /* XXXKYMA do we need to load the host ASID, maybe not because the
     * kernel entries are marked GLOBAL, need to verify
     */

    /* Restore host DDATA_LO */
    LONG_L      k0, PT_HOST_USERLOCAL(sp)
    mtc0        k0, CP0_DDATA_LO

    /* Restore RDHWR access */
    la      k0, 0x2000000F
    mtc0    k0,  CP0_HWRENA

    /* Jump to handler */
FEXPORT(__kvm_mips_jump_to_handler)
    /* XXXKYMA: not sure if this is safe, how large is the stack?? */
    /* Now jump to the kvm_mips_handle_exit() to see if we can deal with this in the kernel */
    la          t9,kvm_mips_handle_exit
    jalr.hb     t9
    addiu       sp,sp, -CALLFRAME_SIZ           /* BD Slot */

    /* Return from handler Make sure interrupts are disabled */
    di
    ehb

    /* XXXKYMA: k0/k1 could have been blown away if we processed an exception
     * while we were handling the exception from the guest, reload k1
     */
    move        k1, s1
	addiu		k1, k1, VCPU_HOST_ARCH

    /* Check return value, should tell us if we are returning to the host (handle I/O etc)
     * or resuming the guest
     */
    andi        t0, v0, RESUME_HOST
    bnez        t0, __kvm_mips_return_to_host
    nop

__kvm_mips_return_to_guest:
    /* Put the saved pointer to vcpu (s1) back into the DDATA_LO Register */
    mtc0        s1, CP0_DDATA_LO

    /* Load up the Guest EBASE to minimize the window where BEV is set */
    LONG_L      t0, VCPU_GUEST_EBASE(k1)

    /* Switch EBASE back to the one used by KVM */
    mfc0        v1, CP0_STATUS
    .set at
	or          k0, v1, ST0_BEV
    .set noat
    mtc0        k0, CP0_STATUS
    ehb
    mtc0        t0,CP0_EBASE

    /* Setup status register for running guest in UM */
    .set at
    or     v1, v1, (ST0_EXL | KSU_USER | ST0_IE)
    and     v1, v1, ~(ST0_CU0 | ST0_MX)
    .set noat
    mtc0    v1, CP0_STATUS
    ehb


	/* Set Guest EPC */
	LONG_L		t0, VCPU_PC(k1)
	mtc0		t0, CP0_EPC

    /* Set the ASID for the Guest Kernel */
    sll         t0, t0, 1                       /* with kseg0 @ 0x40000000, kernel */
                                                /* addresses shift to 0x80000000 */
    bltz        t0, 1f                          /* If kernel */
	addiu       t1, k1, VCPU_GUEST_KERNEL_ASID  /* (BD)  */
    addiu       t1, k1, VCPU_GUEST_USER_ASID    /* else user */
1:
    /* t1: contains the base of the ASID array, need to get the cpu id  */
    LONG_L      t2, TI_CPU($28)             /* smp_processor_id */
    sll         t2, t2, 2                   /* x4 */
    addu        t3, t1, t2
    LONG_L      k0, (t3)
    andi        k0, k0, 0xff
	mtc0		k0,CP0_ENTRYHI
    ehb

    /* Disable RDHWR access */
    mtc0    zero,  CP0_HWRENA

    /* load the guest context from VCPU and return */
    LONG_L  $0, VCPU_R0(k1)
    LONG_L  $1, VCPU_R1(k1)
    LONG_L  $2, VCPU_R2(k1)
    LONG_L  $3, VCPU_R3(k1)
    LONG_L  $4, VCPU_R4(k1)
    LONG_L  $5, VCPU_R5(k1)
    LONG_L  $6, VCPU_R6(k1)
    LONG_L  $7, VCPU_R7(k1)
    LONG_L  $8, VCPU_R8(k1)
    LONG_L  $9, VCPU_R9(k1)
    LONG_L  $10, VCPU_R10(k1)
    LONG_L  $11, VCPU_R11(k1)
    LONG_L  $12, VCPU_R12(k1)
    LONG_L  $13, VCPU_R13(k1)
    LONG_L  $14, VCPU_R14(k1)
    LONG_L  $15, VCPU_R15(k1)
    LONG_L  $16, VCPU_R16(k1)
    LONG_L  $17, VCPU_R17(k1)
    LONG_L  $18, VCPU_R18(k1)
    LONG_L  $19, VCPU_R19(k1)
    LONG_L  $20, VCPU_R20(k1)
    LONG_L  $21, VCPU_R21(k1)
    LONG_L  $22, VCPU_R22(k1)
    LONG_L  $23, VCPU_R23(k1)
    LONG_L  $24, VCPU_R24(k1)
    LONG_L  $25, VCPU_R25(k1)

    /* $/k1 loaded later */
    LONG_L  $28, VCPU_R28(k1)
    LONG_L  $29, VCPU_R29(k1)
    LONG_L  $30, VCPU_R30(k1)
    LONG_L  $31, VCPU_R31(k1)

FEXPORT(__kvm_mips_skip_guest_restore)
    LONG_L  k0, VCPU_HI(k1)
    mthi    k0

    LONG_L  k0, VCPU_LO(k1)
    mtlo    k0

    LONG_L  k0, VCPU_R26(k1)
    LONG_L  k1, VCPU_R27(k1)

    eret

__kvm_mips_return_to_host:
    /* EBASE is already pointing to Linux */
    LONG_L  k1, VCPU_HOST_STACK(k1)
	addiu  	k1,k1, -PT_SIZE

    /* Restore host DDATA_LO */
    LONG_L      k0, PT_HOST_USERLOCAL(k1)
    mtc0        k0, CP0_DDATA_LO

    /* Restore host ASID */
    LONG_L      k0, PT_HOST_ASID(sp)
    andi        k0, 0xff
    mtc0        k0,CP0_ENTRYHI
    ehb

    /* Load context saved on the host stack */
    LONG_L  $0, PT_R0(k1)
    LONG_L  $1, PT_R1(k1)

    /* r2/v0 is the return code, shift it down by 2 (arithmetic) to recover the err code  */
    sra     k0, v0, 2
    move    $2, k0

    LONG_L  $3, PT_R3(k1)
    LONG_L  $4, PT_R4(k1)
    LONG_L  $5, PT_R5(k1)
    LONG_L  $6, PT_R6(k1)
    LONG_L  $7, PT_R7(k1)
    LONG_L  $8, PT_R8(k1)
    LONG_L  $9, PT_R9(k1)
    LONG_L  $10, PT_R10(k1)
    LONG_L  $11, PT_R11(k1)
    LONG_L  $12, PT_R12(k1)
    LONG_L  $13, PT_R13(k1)
    LONG_L  $14, PT_R14(k1)
    LONG_L  $15, PT_R15(k1)
    LONG_L  $16, PT_R16(k1)
    LONG_L  $17, PT_R17(k1)
    LONG_L  $18, PT_R18(k1)
    LONG_L  $19, PT_R19(k1)
    LONG_L  $20, PT_R20(k1)
    LONG_L  $21, PT_R21(k1)
    LONG_L  $22, PT_R22(k1)
    LONG_L  $23, PT_R23(k1)
    LONG_L  $24, PT_R24(k1)
    LONG_L  $25, PT_R25(k1)

    /* Host k0/k1 were not saved */

    LONG_L  $28, PT_R28(k1)
    LONG_L  $29, PT_R29(k1)
    LONG_L  $30, PT_R30(k1)

    LONG_L  k0, PT_HI(k1)
    mthi    k0

    LONG_L  k0, PT_LO(k1)
    mtlo    k0

    /* Restore RDHWR access */
    la      k0, 0x2000000F
    mtc0    k0,  CP0_HWRENA


    /* Restore RA, which is the address we will return to */
    LONG_L  ra, PT_R31(k1)
    j       ra
    nop

    .set    pop
VECTOR_END(MIPSX(GuestExceptionEnd))
.end MIPSX(GuestException)

MIPSX(exceptions):
	####
	##### The exception handlers.
	#####
	.word _C_LABEL(MIPSX(GuestException))	#  0
	.word _C_LABEL(MIPSX(GuestException))	#  1
	.word _C_LABEL(MIPSX(GuestException))	#  2
	.word _C_LABEL(MIPSX(GuestException))	#  3
	.word _C_LABEL(MIPSX(GuestException))	#  4
	.word _C_LABEL(MIPSX(GuestException))	#  5
	.word _C_LABEL(MIPSX(GuestException))	#  6
	.word _C_LABEL(MIPSX(GuestException))	#  7
	.word _C_LABEL(MIPSX(GuestException))	#  8
	.word _C_LABEL(MIPSX(GuestException))	#  9
	.word _C_LABEL(MIPSX(GuestException))	# 10
	.word _C_LABEL(MIPSX(GuestException))	# 11
	.word _C_LABEL(MIPSX(GuestException))	# 12
	.word _C_LABEL(MIPSX(GuestException))	# 13
	.word _C_LABEL(MIPSX(GuestException))	# 14
	.word _C_LABEL(MIPSX(GuestException))	# 15
	.word _C_LABEL(MIPSX(GuestException))	# 16
	.word _C_LABEL(MIPSX(GuestException))	# 17
	.word _C_LABEL(MIPSX(GuestException))	# 18
	.word _C_LABEL(MIPSX(GuestException))	# 19
	.word _C_LABEL(MIPSX(GuestException))	# 20
	.word _C_LABEL(MIPSX(GuestException))	# 21
	.word _C_LABEL(MIPSX(GuestException))	# 22
	.word _C_LABEL(MIPSX(GuestException))	# 23
	.word _C_LABEL(MIPSX(GuestException))	# 24
	.word _C_LABEL(MIPSX(GuestException))	# 25
	.word _C_LABEL(MIPSX(GuestException))	# 26
	.word _C_LABEL(MIPSX(GuestException))	# 27
	.word _C_LABEL(MIPSX(GuestException))	# 28
	.word _C_LABEL(MIPSX(GuestException))	# 29
	.word _C_LABEL(MIPSX(GuestException))	# 30
	.word _C_LABEL(MIPSX(GuestException))	# 31


/* This routine makes changes to the instruction stream effective to the hardware.
 * It should be called after the instruction stream is written.
 * On return, the new instructions are effective.
 * Inputs:
 * a0 = Start address of new instruction stream
 * a1 = Size, in bytes, of new instruction stream
 */

#define HW_SYNCI_Step       $1
LEAF(MIPSX(SyncICache))
    .set    push
	.set	mips32r2
    beq     a1, zero, 20f
    nop
    addu    a1, a0, a1
    rdhwr   v0, HW_SYNCI_Step
    beq     v0, zero, 20f
    nop

10:
    synci   0(a0)
    addu    a0, a0, v0
    sltu    v1, a0, a1
    bne     v1, zero, 10b
    nop
    sync
20:
    jr.hb   ra
    nop
    .set pop
END(MIPSX(SyncICache))
#else
/* use pt_regs struct to save guest OS entry point to stack */
/* restore guest OS to physical cpu */
#include <linux/init.h>
#include <asm/asm.h>
#include <asm/asmmacro.h>
#include <asm/cacheops.h>
#include <asm/irqflags.h>
#include <asm/regdef.h>
#include <asm/fpregdef.h>
#include <asm/mipsregs.h>
#include <asm/stackframe.h>
#include <asm/war.h>
#include <asm/thread_info.h>

#include "loongson_tlb.h"

	.macro	kvm_fpu_save_16even vcpu tmp=t0
	cfc1	\tmp, fcr31
	sdc1	$f0,  KVM_ARCH_P0(\vcpu)
	sdc1	$f2,  KVM_ARCH_P2(\vcpu)
	sdc1	$f4,  KVM_ARCH_P4(\vcpu)
	sdc1	$f6,  KVM_ARCH_P6(\vcpu)
	sdc1	$f8,  KVM_ARCH_P8(\vcpu)
	sdc1	$f10, KVM_ARCH_P10(\vcpu)
	sdc1	$f12, KVM_ARCH_P12(\vcpu)
	sdc1	$f14, KVM_ARCH_P14(\vcpu)
	sdc1	$f16, KVM_ARCH_P16(\vcpu)
	sdc1	$f18, KVM_ARCH_P18(\vcpu)
	sdc1	$f20, KVM_ARCH_P20(\vcpu)
	sdc1	$f22, KVM_ARCH_P22(\vcpu)
	sdc1	$f24, KVM_ARCH_P24(\vcpu)
	sdc1	$f26, KVM_ARCH_P26(\vcpu)
	sdc1	$f28, KVM_ARCH_P28(\vcpu)
	sdc1	$f30, KVM_ARCH_P30(\vcpu)
	sw	\tmp, KVM_ARCH_C31(\vcpu)
	.endm

	.macro	kvm_fpu_save_16odd vcpu
	sdc1	$f1,  KVM_ARCH_P1(\vcpu)
	sdc1	$f3,  KVM_ARCH_P3(\vcpu)
	sdc1	$f5,  KVM_ARCH_P5(\vcpu)
	sdc1	$f7,  KVM_ARCH_P7(\vcpu)
	sdc1	$f9,  KVM_ARCH_P9(\vcpu)
	sdc1	$f11, KVM_ARCH_P11(\vcpu)
	sdc1	$f13, KVM_ARCH_P13(\vcpu)
	sdc1	$f15, KVM_ARCH_P15(\vcpu)
	sdc1	$f17, KVM_ARCH_P17(\vcpu)
	sdc1	$f19, KVM_ARCH_P19(\vcpu)
	sdc1	$f21, KVM_ARCH_P21(\vcpu)
	sdc1	$f23, KVM_ARCH_P23(\vcpu)
	sdc1	$f25, KVM_ARCH_P25(\vcpu)
	sdc1	$f27, KVM_ARCH_P27(\vcpu)
	sdc1	$f29, KVM_ARCH_P29(\vcpu)
	sdc1	$f31, KVM_ARCH_P31(\vcpu)
	.endm

	.macro	kvm_fpu_save_double vcpu status tmp
	sll	\tmp, \status, 5
	bgez	\tmp, 2f
	kvm_fpu_save_16odd \vcpu
2:
	kvm_fpu_save_16even \vcpu \tmp
	.endm

	.macro	kvm_fpu_restore_16even vcpu tmp=t0
	lw	\tmp, KVM_ARCH_C31(\vcpu)
	ldc1	$f0,  KVM_ARCH_P0(\vcpu)
	ldc1	$f2,  KVM_ARCH_P2(\vcpu)
	ldc1	$f4,  KVM_ARCH_P4(\vcpu)
	ldc1	$f6,  KVM_ARCH_P6(\vcpu)
	ldc1	$f8,  KVM_ARCH_P8(\vcpu)
	ldc1	$f10, KVM_ARCH_P10(\vcpu)
	ldc1	$f12, KVM_ARCH_P12(\vcpu)
	ldc1	$f14, KVM_ARCH_P14(\vcpu)
	ldc1	$f16, KVM_ARCH_P16(\vcpu)
	ldc1	$f18, KVM_ARCH_P18(\vcpu)
	ldc1	$f20, KVM_ARCH_P20(\vcpu)
	ldc1	$f22, KVM_ARCH_P22(\vcpu)
	ldc1	$f24, KVM_ARCH_P24(\vcpu)
	ldc1	$f26, KVM_ARCH_P26(\vcpu)
	ldc1	$f28, KVM_ARCH_P28(\vcpu)
	ldc1	$f30, KVM_ARCH_P30(\vcpu)
	ctc1	\tmp, fcr31
	.endm

	.macro	kvm_fpu_restore_16odd vcpu
	ldc1	$f1,  KVM_ARCH_P1(\vcpu)
	ldc1	$f3,  KVM_ARCH_P3(\vcpu)
	ldc1	$f5,  KVM_ARCH_P5(\vcpu)
	ldc1	$f7,  KVM_ARCH_P7(\vcpu)
	ldc1	$f9,  KVM_ARCH_P9(\vcpu)
	ldc1	$f11, KVM_ARCH_P11(\vcpu)
	ldc1	$f13, KVM_ARCH_P13(\vcpu)
	ldc1	$f15, KVM_ARCH_P15(\vcpu)
	ldc1	$f17, KVM_ARCH_P17(\vcpu)
	ldc1	$f19, KVM_ARCH_P19(\vcpu)
	ldc1	$f21, KVM_ARCH_P21(\vcpu)
	ldc1	$f23, KVM_ARCH_P23(\vcpu)
	ldc1	$f25, KVM_ARCH_P25(\vcpu)
	ldc1	$f27, KVM_ARCH_P27(\vcpu)
	ldc1	$f29, KVM_ARCH_P29(\vcpu)
	ldc1	$f31, KVM_ARCH_P31(\vcpu)
	.endm

	.macro	kvm_fpu_restore_double vcpu status tmp
	sll	\tmp, \status, 5
	bgez	\tmp, 1f				# 16 register mode?

	kvm_fpu_restore_16odd \vcpu
1:	kvm_fpu_restore_16even \vcpu \tmp
	.endm

#ifdef CONFIG_MIPS_MT_SMTC
#define PTEBASE_SHIFT	19	/* TCBIND */
#define CPU_ID_REG CP0_TCBIND
#define CPU_ID_MFC0 mfc0
#elif defined(CONFIG_MIPS_PGD_C0_CONTEXT)
#define PTEBASE_SHIFT	48	/* XCONTEXT */
#define CPU_ID_REG CP0_XCONTEXT
#define CPU_ID_MFC0 MFC0
#else
#define PTEBASE_SHIFT	23	/* CONTEXT */
#define CPU_ID_REG CP0_CONTEXT
#define CPU_ID_MFC0 MFC0
#endif

NESTED(__kvmmips_vcpu_run, PT_SIZE, sp)
	SAVE_ALL
/* a0 point to kvm_run ,a1 point to kvm_vcpu
 * save stack point  to kvm_vcpu
 */
	LONG_S	sp,	KVM_ARCH_HOST_STACK(a1)

/* save kvm_vcpu kvm_run to global variable*/
	CPU_ID_MFC0	a2, CPU_ID_REG
	LONG_SRL	a2, PTEBASE_SHIFT
	LONG_S	a0,	run_mips_global(a2)
	LONG_S	a1,	vcpu_mips_global(a2)

lightweight_exit:

	CPU_ID_MFC0	a2, CPU_ID_REG
	LONG_SRL	a2, PTEBASE_SHIFT
	LONG_L	a1,	vcpu_mips_global(a2)

/* TODO: store host wired, entryhi and pagemask */
	dmfc0	t0, CP0_ENTRYHI
	LONG_S	t0, KVM_ARCH_HOST_CP0_ENTRYHI(a1)
	mfc0	t0, CP0_PAGEMASK
	LONG_S	t0, KVM_ARCH_HOST_CP0_PAGEMASK(a1)
	mfc0	t0, CP0_WIRED
	LONG_S	t0, KVM_ARCH_HOST_CP0_WIRED(a1)

/* inject exception handlers*/
	CPU_ID_MFC0	a2, CPU_ID_REG
	LONG_SRL	a2, PTEBASE_SHIFT
	LONG_L	a1,	vcpu_mips_global(a2)
	CPU_ID_MFC0	a0, CPU_ID_REG
	LONG_SRL	a0, PTEBASE_SHIFT
	jal		kvmmips_inject_exception
	nop

	CPU_ID_MFC0	a2, CPU_ID_REG
	LONG_SRL	a2, PTEBASE_SHIFT
	LONG_L	a1,	vcpu_mips_global(a2)

/* TODO: restore wired, entryhi and pagemask of vcpu */
	LONG_L	t0,	KVM_ARCH_CP0(a1)
	LONG_L	t0,	KVM_ARCH_CP0_ENTRYHI(t0)
	ori		t0, 0x80 #cww!!!
	dmtc0	t0,	CP0_ENTRYHI
	#li		t0, 0x0
	li		t0, 0x1e000
	mtc0	t0,	CP0_PAGEMASK
	li		t0, 0x0
	mtc0	t0,	CP0_WIRED

/* enable cache instruction user mode */
	mfc0	t0, CP0_DIAGNOSTIC
	ori	t0, 0x100
	mtc0	t0, CP0_DIAGNOSTIC

/* #ifndef CONFIG_EASY_TLB */
#ifdef CONFIG_KVM_MULTICLIENT
	CPU_ID_MFC0	a2, CPU_ID_REG
	LONG_SRL	a2, PTEBASE_SHIFT
	LONG_L	a0,	vcpu_mips_global(a2)
	jal		kvmmips_finish_tlbload
	nop
#endif

/* load kvm_vcpu to global variable*/
	CPU_ID_MFC0	a2, CPU_ID_REG
	LONG_SRL	a2, PTEBASE_SHIFT
	LONG_L	a1,	vcpu_mips_global(a2)

/* restore guest fpu registers */
	lw		t1, KVM_ARCH_SAVE_FPR(a1)
	beqz	t1,	2f
	nop

	mfc0	t0,	CP0_STATUS
	li	t1,	ST0_CU1
	or	t0,	t1
	LONG_L	t1,	KVM_ARCH_CP0(a1)
	LONG_L  t1, KVM_ARCH_CP0_STATUS(t1)
	li		t2,	ST0_FR
	and		t1, t2
	or		t0, t1
	mtc0	t0,	CP0_STATUS

	mfc0	t1,	CP0_STATUS
	kvm_fpu_restore_double a1 t1 t0
	li		t1,	0x0
	sw		t1, KVM_ARCH_SAVE_FPR(a1)
2:

/* set guest CP0_STATUS
 * unset CU0; restore guest CP0_STATUS:CU1 and KSU; set EXL and IE for eret
 */
	mfc0	t0,	CP0_STATUS
	li		t1,	~(ST0_CU0 | ST0_CU1 | ST0_FR)
	and		t0, t1
	LONG_L	t1,	KVM_ARCH_CP0(a1)
	LONG_L  t1, KVM_ARCH_CP0_STATUS(t1)
	li		t2,	ST0_CU1 | ST0_FR
	and		t2, t1
	or		t0, t2
#ifdef CONFIG_KVM64_SUPPORT
	ori		t0, 0x4b /*supervisor mode*/
#else
	ori		t0, 0xb
#endif
	mtc0	t0, CP0_STATUS

/* set epc = entry address of guest OS */
	LONG_L	t0,	KVM_ARCH_PC(a1)
	dmtc0	t0,	CP0_EPC

/* restore guest general registers */
	LONG_L	t0, KVM_ARCH_HI(a1)
	mthi	t0
	LONG_L	t0, KVM_ARCH_LO(a1)
	mtlo	t0
	LONG_L	$0,	KVM_ARCH_R0(a1)
	LONG_L	$2,	KVM_ARCH_R2(a1)
	LONG_L	$3,	KVM_ARCH_R3(a1)
	LONG_L	$8,	KVM_ARCH_R8(a1)
	LONG_L	$9,	KVM_ARCH_R9(a1)
	LONG_L	$10,	KVM_ARCH_R10(a1)
	LONG_L	$11,	KVM_ARCH_R11(a1)
	LONG_L	$12,	KVM_ARCH_R12(a1)
	LONG_L	$13,	KVM_ARCH_R13(a1)
	LONG_L	$14,	KVM_ARCH_R14(a1)
	LONG_L	$15,	KVM_ARCH_R15(a1)
	LONG_L	$16,	KVM_ARCH_R16(a1)
	LONG_L	$17,	KVM_ARCH_R17(a1)
	LONG_L	$18,	KVM_ARCH_R18(a1)
	LONG_L	$19,	KVM_ARCH_R19(a1)
	LONG_L	$20,	KVM_ARCH_R20(a1)
	LONG_L	$21,	KVM_ARCH_R21(a1)
	LONG_L	$22,	KVM_ARCH_R22(a1)
	LONG_L	$23,	KVM_ARCH_R23(a1)
	LONG_L	$24,	KVM_ARCH_R24(a1)
	LONG_L	$25,	KVM_ARCH_R25(a1)
	LONG_L	$26,	KVM_ARCH_R26(a1)
	LONG_L	$27,	KVM_ARCH_R27(a1)
	LONG_L	$28,	KVM_ARCH_R28(a1)
	LONG_L	$29,	KVM_ARCH_R29(a1)
	LONG_L	$30,	KVM_ARCH_R30(a1)
	LONG_L	$6,	KVM_ARCH_R6(a1)
	LONG_L	$7,	KVM_ARCH_R7(a1)
	LONG_L	$31,	KVM_ARCH_R31(a1)
	.set noat
	LONG_L	$1,	KVM_ARCH_R1(a1)
	.set at

/* begin counting */
	mfc0	a0,	CP0_COUNT
	sw		a0, KVM_ARCH_COUNT_START(a1)

	LONG_L	$4,	KVM_ARCH_R4(a1)
	LONG_L	$5,	KVM_ARCH_R5(a1)

/* enter guest OS */
	eret
	nop
	nop

END(__kvmmips_vcpu_run)


	.macro  __build_kvm_clear_none
	.endm

	.macro  __build_kvm_fpr_save
	CPU_ID_MFC0	a2, CPU_ID_REG
	LONG_SRL	a2, PTEBASE_SHIFT
	LONG_L	a1,	vcpu_mips_global(a2)

	mfc0	t0,	CP0_STATUS
	mfc0	t2,	CP0_STATUS
	li	t1,	ST0_CU1
	or	t0,	t1
	mtc0	t0,	CP0_STATUS

	/* discard the fpe info here, or it will be wrong */
	cfc1	t1, fcr31
	li		t0, 0x3f000
	and		t0, t1
	beqz	t0, 1f
	nop
	LONG_L	t0,	KVM_ARCH_CP0(a1)
	LONG_S  t1, KVM_ARCH_CP1_FCR(t0)
1:
	li		t0, ~0x3f000
	and		t1,	t0
	ctc1	t1, fcr31

	mfc0	t1,	CP0_STATUS
	kvm_fpu_save_double a1 t1 t0
	mtc0	t2,	CP0_STATUS
	li		t1,	0x1
	sw		t1, KVM_ARCH_SAVE_FPR(a1)
	.endm

	.macro  __build_kvm_fpr_none
	.endm

	.macro  __build_kvm_clear_sti
/* local_irq_enable */
	mfc0	t0,	CP0_STATUS
	ori		t0,	0x1
	mtc0	t0,	CP0_STATUS
	.endm

	.macro  __build_kvm_clear_cli
/* local_irq_disable */
	mfc0	t0,	CP0_STATUS
	ori		t0,	0x1
	xori	t0,	0x1
	mtc0	t0,	CP0_STATUS
	.endm

/* ade is not consider now */
	.macro  __build_kvm_clear_ade
	.endm

	.macro	BUILD_KVMMIPS_HANDLER exception  handler clear restore save
	.align	5
	NESTED(handle_kvm_\exception, PT_SIZE, sp)
/* store vcpu general registers except at, k0, k1 */
	move	k0,	a1
	CPU_ID_MFC0	k1, CPU_ID_REG
	LONG_SRL	k1, PTEBASE_SHIFT
	LONG_L	a1,	vcpu_mips_global(k1)
	LONG_S	$4,	KVM_ARCH_R4(a1)

	dmfc0	k1,	$24
	LONG_S	k1,	KVM_ARCH_R27(a1)
	dmfc0	k1,	$30
	LONG_S	k1,	KVM_ARCH_R26(a1)


/* finish counting */
	mfc0	a0, CP0_COUNT
	sw		a0, KVM_ARCH_COUNT_END(a1)

	LONG_S	$0,	KVM_ARCH_R0(a1)
#	LONG_S	$1,	KVM_ARCH_R1(a1)
	LONG_S	$2,	KVM_ARCH_R2(a1)
	LONG_S	$3,	KVM_ARCH_R3(a1)
	LONG_S	$8,	KVM_ARCH_R8(a1)
	LONG_S	$9,	KVM_ARCH_R9(a1)
	LONG_S	$10,	KVM_ARCH_R10(a1)
	LONG_S	$11,	KVM_ARCH_R11(a1)
	LONG_S	$12,	KVM_ARCH_R12(a1)
	LONG_S	$13,	KVM_ARCH_R13(a1)
	LONG_S	$14,	KVM_ARCH_R14(a1)
	LONG_S	$15,	KVM_ARCH_R15(a1)
	LONG_S	$16,	KVM_ARCH_R16(a1)
	LONG_S	$17,	KVM_ARCH_R17(a1)
	LONG_S	$18,	KVM_ARCH_R18(a1)
	LONG_S	$19,	KVM_ARCH_R19(a1)
	LONG_S	$20,	KVM_ARCH_R20(a1)
	LONG_S	$21,	KVM_ARCH_R21(a1)
	LONG_S	$22,	KVM_ARCH_R22(a1)
	LONG_S	$23,	KVM_ARCH_R23(a1)
	LONG_S	$24,	KVM_ARCH_R24(a1)
	LONG_S	$25,	KVM_ARCH_R25(a1)
#	LONG_S	$26,	KVM_ARCH_R26(a1)
#	LONG_S	$27,	KVM_ARCH_R27(a1)
	LONG_S	$28,	KVM_ARCH_R28(a1)
	LONG_S	$29,	KVM_ARCH_R29(a1)
	LONG_S	$30,	KVM_ARCH_R30(a1)
	LONG_S	$6,	KVM_ARCH_R6(a1)
	LONG_S	$7,	KVM_ARCH_R7(a1)
	LONG_S	$31,	KVM_ARCH_R31(a1)
	move	a0,	k0
	LONG_S	a0,	KVM_ARCH_R5(a1)

	mfhi	t0
	LONG_S	t0,	KVM_ARCH_HI(a1)
	mflo	t0
	LONG_S	t0,	KVM_ARCH_LO(a1)

/* store vcpu pc */
	dmfc0	t0,	CP0_EPC
	LONG_S	t0,	KVM_ARCH_PC(a1)

/* TODO: store guest wired, entryhi and pagemask */

/* TODO:store some cpu info when guest OS running.*/
	mfc0	t0,	CP0_CAUSE
	LONG_S	t0,	KVM_ARCH_TEMP_CP0_CAUSE(a1)
	dmfc0	t0,	CP0_EPC
	LONG_S	t0,	KVM_ARCH_TEMP_CP0_EPC(a1)
	dmfc0	t0,	CP0_BADVADDR
	LONG_S	t0,	KVM_ARCH_TEMP_CP0_BADVADDR(a1)
	dmfc0	t0,	CP0_ENTRYHI
	LONG_S	t0,	KVM_ARCH_TEMP_CP0_ENTRYHI(a1)
	mfc0	t0,	CP0_PAGEMASK
	LONG_S	t0,	KVM_ARCH_TEMP_CP0_PAGEMASK(a1)
	mfc0	t0,	CP0_CONTEXT
	LONG_S	t0,	KVM_ARCH_TEMP_CP0_CONTEXT(a1)

	__build_kvm_fpr_\save

/* restore the host sp and gp */
	LONG_L	sp,	KVM_ARCH_HOST_STACK(a1)
	ori		$28, sp, _THREAD_MASK
	xori	$28, _THREAD_MASK

/* restore host CP0_STATUS
 * set CU0, set KSU to kernel mode, clear EXL and ERL, and restore host CU1 set
 */
	mfc0	t0,	CP0_STATUS
	li		t1,	~(ST0_CU0 | ST0_CU1 | ST0_FR)
	and		t0, t1
	ori	t0,	0x1f
	xori	t0, 0x1f
	li		t1, ST0_CU0
	or		t0, t1
	LONG_L  t1, PT_STATUS(sp)
	li		t2,	ST0_CU1 | ST0_FR
	and		t2, t1
	or		t0, t2
	ori	t0,	0x1
	xori	t0,	0x1
	mtc0	t0, CP0_STATUS

/* TODO: restore host wired, entryhi and pagemask */
	LONG_L	t0, KVM_ARCH_HOST_CP0_ENTRYHI(a1)
	dmtc0	t0, CP0_ENTRYHI
	LONG_L	t0, KVM_ARCH_HOST_CP0_PAGEMASK(a1)
	mtc0	t0, CP0_PAGEMASK
	LONG_L	t0, KVM_ARCH_HOST_CP0_WIRED(a1)
	mtc0	t0, CP0_WIRED

/* restore CP0_DIAGNOSTIC */
	li		t0, 0
	mtc0	t0, CP0_DIAGNOSTIC

/* TODO: flush guest tlb entry */
#ifdef CONFIG_KVM_MULTICLIENT
	CPU_ID_MFC0	a2, CPU_ID_REG
	LONG_SRL	a2, PTEBASE_SHIFT
	LONG_L	a0,	vcpu_mips_global(a2)
#	jal		kvmmips_finish_tlbstore
	jal		kvmmips_tlbsave
	nop
#endif

/* restore host exception handler*/
	jal		kvmmips_restore_host
	nop

/*  load vcpu_mips_global to a0 */
	CPU_ID_MFC0	a2, CPU_ID_REG
	LONG_SRL	a2, PTEBASE_SHIFT
	LONG_L	a0,	vcpu_mips_global(a2)
	jal		kvmmips_update_timing_stats
	nop

/* load run_mips_global to a0, load vcpu_mips_global to a1 */
	CPU_ID_MFC0	a2, CPU_ID_REG
	LONG_SRL	a2, PTEBASE_SHIFT
	LONG_L	a0,	run_mips_global(a2)
	LONG_L	a1,	vcpu_mips_global(a2)

/* store run_mips_global and vcpu_mips_global for it will changed*/
	dsubu	sp,	16
	sd		a0,	0(sp)
	sd		a1,	8(sp)

	__build_kvm_clear_\clear

	jal		do_kvm_\handler
	nop
	move	s0, v0

	__build_kvm_clear_\restore

/* restore run_mips_global and vcpu_mips_global*/
	ld		a0,	0(sp)
	ld		a1,	8(sp)
	daddu	sp,	16
	CPU_ID_MFC0	a2, CPU_ID_REG
	LONG_SRL	a2, PTEBASE_SHIFT
	LONG_S	a0,	run_mips_global(a2)
	LONG_S	a1,	vcpu_mips_global(a2)

/* kvmmips_deliver_exceptions */
	LONG_L	a0,	vcpu_mips_global(a2)
	jal		kvmmips_deliver_exceptions
	nop

/* branch here to switch */
	li		a0,	1  /** RETURN_TO_GUEST*/
	beq		s0,	a0,	lightweight_exit
	nop

/* restore to host */
	RESTORE_ALL

	li		v0, 0
	jr		ra
	nop

	END(handle_kvm_\exception)
	.endm


	BUILD_KVMMIPS_HANDLER int int sti cli save		/* #0  */
	BUILD_KVMMIPS_HANDLER mod mod cli cli none		/* #1  */
	BUILD_KVMMIPS_HANDLER tlbl tlbl sti cli save		/* #2  */
	BUILD_KVMMIPS_HANDLER tlbs tlbs sti cli save		/* #3  */
	BUILD_KVMMIPS_HANDLER adel adel sti cli save		/* #4  */
	BUILD_KVMMIPS_HANDLER ades ades sti cli save		/* #5  */
	BUILD_KVMMIPS_HANDLER ibe ibe cli cli none		/* #6  */
	BUILD_KVMMIPS_HANDLER dbe dbe cli cli none		/* #7  */
	BUILD_KVMMIPS_HANDLER syscall syscall sti cli save	/* #8 */
	BUILD_KVMMIPS_HANDLER bp bp cli cli none		/* #9  */
	BUILD_KVMMIPS_HANDLER ri ri cli cli none		/* #10 */
	BUILD_KVMMIPS_HANDLER cpu cpu sti cli save		/* #11 */
	BUILD_KVMMIPS_HANDLER ov ov cli cli none		/* #12 */
	BUILD_KVMMIPS_HANDLER tr tr sti cli save		/* #13 */
	BUILD_KVMMIPS_HANDLER fpe fpe cli cli save		/* #15 */
	BUILD_KVMMIPS_HANDLER c2e c2e cli cli none		/* #18 */
	BUILD_KVMMIPS_HANDLER mdmx mdmx cli cli none		/* #22 */
#ifdef 	CONFIG_HARDWARE_WATCHPOINTS
	/*
	 * For watch, interrupts will be enabled after the watch
	 * registers are read.
	 */
	BUILD_KVMMIPS_HANDLER watch watch cli cli none		/* #23 */
#else
	BUILD_KVMMIPS_HANDLER watch watch sti cli none		/* #23 */
#endif
	BUILD_KVMMIPS_HANDLER mcheck mcheck cli cli none	/* #24 */
	BUILD_KVMMIPS_HANDLER mt mt cli cli none		/* #25 */
	BUILD_KVMMIPS_HANDLER dsp dsp cli cli none		/* #26 */
	BUILD_KVMMIPS_HANDLER tlbmiss tlbmiss sti cli save	/* #tlbmiss */
	BUILD_KVMMIPS_HANDLER reserved reserved none none none	/* others */

#endif /* CONFIG_KVM_MIPS_LOONGSON3 */

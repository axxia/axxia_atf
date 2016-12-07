/*
 * Copyright (c) 2013-2015, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Assember routines to dump registers to DDR */
/* They  various register name strings, some defined here, some defined in bl31/aarch64/crash_reporting */

#include <asm_macros.S>
#include <context.h>
#include <gic_common.h>
#include <axxia_def.h>

#include <ccn504.h>

#ifdef CONFIG_DATALOGGER


	.global dump_cpu_registers
	.global dump_gic_registers
	.global dump_ccn504_registers


/* Convert x9 register contents to ascii and store to ddr*/
/* X0 has address to store ASCII output */
/* X0 will point to next free ddr location on exit */
/* Clobbers x15, x16, x17 */


func dump_hex
        mov     x15, x30
        mov     x16, #64  /* No of bits to convert to ascii */
1:
        sub     x16, x16, #4
        lsrv    x17, x9, x16
        and     x17, x17, #0xf
        cmp     x17, #0xA
        b.lo    2f
        /* Add by 0x27 in addition to 0x30
         * to get ascii for characters 'a - f'.
         */
        add     x17, x17, #0x27
2:
        add     x17, x17, #0x30
        strb    w17, [x0], #1
        cbnz    x16, 1b
  

        mov x30, x15
        ret     x15
endfunc dump_hex


/* Print string, address in X11, ddr location in X0*/
/* On exit, X0 points to next ddr location */
/* Clobbers x13, x14 */
func dump_str_no_reg
	mov x14, x30

1:	ldrb 	w13, [x11], #1
	cbz 	x13, 2f
	strb	w13, [x0], #1
	b	1b
2:	

	mov x30, x14
	ret x30


endfunc dump_str_no_reg


/*Write register name, then spacer, then reg value to ddr. Repeast for 2 registers */
/* X11 points to string, x0 points to ddr location, x9 and x10 contain reg values*/
/* On exit x11 points to next string, x0 points to next address in ddr */
/* Clobbers x13, x14 */
func dump_str_pair
	mov x14, x30

1:	ldrb 	w13, [x11], #1
	cbz 	x13, 2f
	strb	w13, [x0], #1
	b	1b
2:	mov	w13, ':'
	strb    w13, [x0], #1
	mov	w13, '\t'
	strb    w13, [x0], #1

	bl      dump_hex

	mov	w13, '\n'
	strb    w13, [x0], #1

3:	ldrb 	w13, [x11], #1
	cbz 	x13,4f
	strb	w13, [x0], #1
	b	3b
4:	mov	w13, ':'
	strb    w13, [x0], #1
	mov	w13, '\t'
	strb    w13, [x0], #1


	mov x9, x10
	bl  dump_hex

	mov	w13, '\n'
	strb    w13, [x0], #1

	mov x30, x14
	ret x30
endfunc dump_str_pair

/*Write register name, then spacer, then reg value to ddr.  */
/* X11 points to string, x0 points to ddr location, x9  contains reg value*/
/* On exit x11 points to next string, x0 points to next address in ddr */
/* Clobbers x13, x14 */
func dump_str_single
	mov x14, x30

1:	ldrb 	w13, [x11], #1
	cbz 	x13, 2f
	strb	w13, [x0], #1
	b	1b
2:	mov	w13, ':'
	strb    w13, [x0], #1
	mov	w13, '\t'
	strb    w13, [x0], #1

	bl      dump_hex

	mov	w13, '\n'
	strb    w13, [x0], #1

	mov x30, x14
	ret x30

endfunc dump_str_single



.section .rodata.cpu_reg_name, "aS"
/* Note, most CPU regs are defined in bl31/aarch64/crash_reporting, here are a few extras */
cpu_header:
	.asciz  "\nCore Registers For CPU: " 

x30_reg:
	.asciz	"x30", ""

newline_char:
	.asciz  "\n", ""




/* Dump cpu registers to ddr */
/* Before entry, sp should be set to sp_el3, x0 holds ddr address, x1 contains CPU number */
/* Clobbers x9, x10, x11, x12 */
func dump_cpu_registers
	/* Save link reg */
	mov x12, x30

	/* Print header */
	adr 	x11, cpu_header
	bl	dump_str_no_reg

	mov	x9, x1
	bl 	dump_hex

	adr	x11, newline_char
	bl	dump_str_no_reg



	/* Get ptr to gp_regs names */	
	adr	x11, gp_regs

	/* Read gp reg pair */
	ldp	x9, x10, [sp, #CTX_GPREGS_OFFSET + CTX_GPREG_X0]
	bl 	dump_str_pair
	ldp	x9, x10, [sp, #CTX_GPREGS_OFFSET + CTX_GPREG_X2]
	bl 	dump_str_pair
	ldp	x9, x10, [sp, #CTX_GPREGS_OFFSET + CTX_GPREG_X4]
	bl 	dump_str_pair
	ldp	x9, x10, [sp, #CTX_GPREGS_OFFSET + CTX_GPREG_X6]
	bl 	dump_str_pair
	ldp	x9, x10, [sp, #CTX_GPREGS_OFFSET + CTX_GPREG_X8]
	bl 	dump_str_pair
	ldp	x9, x10, [sp, #CTX_GPREGS_OFFSET + CTX_GPREG_X10]
	bl 	dump_str_pair
	ldp	x9, x10, [sp, #CTX_GPREGS_OFFSET + CTX_GPREG_X12]
	bl 	dump_str_pair
	ldp	x9, x10, [sp, #CTX_GPREGS_OFFSET + CTX_GPREG_X14]
	bl 	dump_str_pair
	ldp	x9, x10, [sp, #CTX_GPREGS_OFFSET + CTX_GPREG_X16]
	bl 	dump_str_pair
	ldp	x9, x10, [sp, #CTX_GPREGS_OFFSET + CTX_GPREG_X18]
	bl 	dump_str_pair
	ldp	x9, x10, [sp, #CTX_GPREGS_OFFSET + CTX_GPREG_X20]
	bl 	dump_str_pair
	ldp	x9, x10, [sp, #CTX_GPREGS_OFFSET + CTX_GPREG_X22]
	bl 	dump_str_pair
	ldp	x9, x10, [sp, #CTX_GPREGS_OFFSET + CTX_GPREG_X24]
	bl 	dump_str_pair
	ldp	x9, x10, [sp, #CTX_GPREGS_OFFSET + CTX_GPREG_X26]
	bl 	dump_str_pair
	ldp	x9, x10, [sp, #CTX_GPREGS_OFFSET + CTX_GPREG_X28]
	bl 	dump_str_pair
	ldr	 x9,     [sp, #CTX_GPREGS_OFFSET + CTX_GPREG_LR]

	adr	x11, x30_reg
	bl	dump_str_single

	/* Dump the el3 sys registers */
	adr	x11, el3_sys_regs

	mrs	x9, scr_el3
	mrs	x10, sctlr_el3
	bl 	dump_str_pair
	mrs	x9, cptr_el3
	mrs	x10, tcr_el3
	bl 	dump_str_pair
	mrs	x9, daif
	mrs	x10, mair_el3
	bl 	dump_str_pair
	mrs	x9, spsr_el3
	mrs	x10, elr_el3
	bl 	dump_str_pair
	mrs	x9, ttbr0_el3
	mrs	x10, esr_el3
	bl 	dump_str_pair
	mrs	x9, far_el3
	bl	dump_str_single

	/* Print the non el3 sys registers */
	adr	x11, non_el3_sys_regs

	mrs	x9, spsr_el1
	mrs	x10, elr_el1
	bl 	dump_str_pair
	mrs	x9, spsr_abt
	mrs	x10, spsr_und
	bl 	dump_str_pair
	mrs	x9, spsr_irq
	mrs	x10, spsr_fiq
	bl 	dump_str_pair
	mrs	x9, sctlr_el1
	mrs	x10, actlr_el1
	bl 	dump_str_pair
	mrs	x9, cpacr_el1
	mrs	x10, csselr_el1
	bl 	dump_str_pair
	mrs	x9, sp_el1
	mrs	x10, esr_el1
	bl 	dump_str_pair
	mrs	x9, ttbr0_el1
	mrs	x10, ttbr1_el1
	bl 	dump_str_pair
	mrs	x9, mair_el1
	mrs	x10, amair_el1
	bl 	dump_str_pair
	mrs	x9, tcr_el1
	mrs	x10, tpidr_el1
	bl 	dump_str_pair
	mrs	x9, tpidr_el0
	mrs	x10, tpidrro_el0
	bl	dump_str_pair
	mrs	x9, dacr32_el2
	mrs	x10, ifsr32_el2
	bl	dump_str_pair
	mrs	x9, par_el1
	mrs	x10, mpidr_el1
	bl 	dump_str_pair
	mrs	x9, afsr0_el1
	mrs	x10, afsr1_el1
	bl 	dump_str_pair
	mrs	x9, contextidr_el1
	mrs	x10, vbar_el1
	bl 	dump_str_pair
	mrs	x9, cntp_ctl_el0
	mrs	x10, cntp_cval_el0
	bl 	dump_str_pair
	mrs	x9, cntv_ctl_el0
	mrs	x10, cntv_cval_el0
	bl 	dump_str_pair
	mrs	x9, cntkctl_el1
	mrs	x10, fpexc32_el2
	bl 	dump_str_pair
	mrs	x9, sp_el0
	bl 	dump_str_single



	/* Exit */
	mov x30, x12
	ret x30
endfunc	dump_cpu_registers



.section .rodata.gic_reg_name, "aS"
/* Applicable only to GICv3 with SRE enabled */
icc_regs:
	.asciz "icc_hppir0_el1", "icc_hppir1_el1", "icc_ctlr_el3", ""

/* Registers common to both GICv2 and GICv3 */
gicd_pend_reg:
	.asciz "gicd_ispendr regs (Offsets 0x200 - 0x278)\n"	\
		" Offset:\t\t\tvalue\n"


gic_header:
	.asciz  "\nGIC Registers\n", ""




/* Dump GIC registers to ddr */
/* Before entry, sp should be set to sp_el3, x0 holds ddr address */
/* Clobbers x9, x10, x11, x12 */
func dump_gic_registers
	/* Save link reg */
	mov x12, x30

	/* Print header */
	adr	x11, gic_header
	bl	dump_str_no_reg

	/* Load the icc reg list to x11 */
	adr	x11, icc_regs
	/* Dump icc regs */
	mrs	x9, ICC_HPPIR0_EL1
	mrs	x10, ICC_HPPIR1_EL1
	bl	dump_str_pair	

	mrs	x9, ICC_CTLR_EL3
	bl 	dump_str_single	

	ldr	x10, =GICC_BASE

	/* Print the GICD_ISPENDR regs */
	
	/* Print GICD header */
	adr	x11, gicd_pend_reg
	bl	dump_str_no_reg

	add	x11, x10, #GICD_ISPENDR  /*x11 contains base address of ispendr regs*/
gicd_ispendr_loop:
	sub	x9, x11, x10
	cmp	x9, #0x280		/* At end of ispendr regs ? */
	b.eq	exit_dump_gic_regs

	ldr	x9, [x11], #8		/* Load x9 with ispendr, move x11 to next ispendr */	
	bl 	dump_hex	        /* Print contents of x9 as hex */
	mov	w9, '\n'
	strb    w9, [x0], #1		/*Print newline */
	
	b	gicd_ispendr_loop

exit_dump_gic_regs:
	mov x30, x12
	ret x30
endfunc dump_gic_registers




.section .rodata.ccn504_reg_name, "aS"

ccn504_header:
	.asciz "\nCCN504 Error Registers \n","" 

ccn504_mn_header:
	.asciz  "\nMN Registers \n", ""

ccn504_mn_regs:
	.asciz  "errint_status", "dvm_domain_ctl", "err_sig_val_63_0", "err_sig_val_127_64", \
		"err_sig_val_191_128", "err_type_31_0", "err_type_63_32", "err_type_95_64",  \
		"err_type_159_128", "err_syndrome_reg0", ""

ccn504_hni_header:
	.asciz  "\nHNI Registers\n", ""

ccn504_err_syn_regs:
	.asciz "err_syndrome_reg0", "err_syndrome_reg1", ""


ccn504_hnf_header:
	.asciz "\nHNF (L3 0 -7) Registers\n", ""

ccn504_hnf_regs:
	.asciz  "snoop_domain_ctl", "err_syndrome_reg0", "err_syndrome_reg1", ""

ccn504_xp_header:
	.asciz  "\nXP 0 - 10  Registers\n", ""



/* Dump CCN-504 error registers to ddr */
/* Before entry, sp should be set to sp_el3, x0 holds ddr address */
/* Clobbers x9, x10, x11, x12 */
func dump_ccn504_registers
	/* Save link reg */
	mov x12, x30

	/* Print header */
	adr	x11, ccn504_header
	bl	dump_str_no_reg

	/*Print MN registers */
	adr	x11, ccn504_mn_header
	bl	dump_str_no_reg

	adr	x11, ccn504_mn_regs
	ldr	x10, =DICKENS_BASE_X9

	add	x10, x10, #ERRINT_STATUS
	ldr	x9, [x10]
	bl	dump_str_single
	add	x10,x10, #DVM_DOMAIN_CTL
	ldr	x9, [x10]
	bl	dump_str_single	
	add	x10,x10, #ERR_SIG_VAL_63_0
	ldr	x9, [x10]
	bl	dump_str_single
	add	x10,x10, #ERR_SIG_VAL_127_64
	ldr	x9, [x10]
	bl	dump_str_single	
	add	x10,x10, #ERR_SIG_VAL_191_128
	ldr	x9, [x10]
	bl	dump_str_single	
	add	x10,x10,  #ERR_TYPE_31_0
	ldr	x9, [x10]
	bl	dump_str_single	
	add	x10,x10, #ERR_TYPE_63_32
	ldr	x9, [x10]
	bl	dump_str_single
	add	x10,x10, #ERR_TYPE_95_64
	ldr	x9, [x10]
	bl	dump_str_single	
	add	x10,x10, #ERR_TYPE_159_128
	ldr	x9, [x10]
	bl	dump_str_single
	add	x10,x10, #ERR_SYNDROME_REG0
	ldr	x9, [x10]
	bl	dump_str_single	


	/* Print HN-I Registers */
	adr	x11, ccn504_hni_header
	bl	dump_str_no_reg

	adr	x11, ccn504_err_syn_regs
	ldr	x10, =(DICKENS_BASE_X9 + CCN_HNI_OFFSET)	
	add	x10, x10, #ERR_SYNDROME_REG0
	ldr	x9, [x10]
	bl	dump_str_single
	add	x10, x10, #ERR_SYNDROME_REG1
	ldr	x9, [x10]
	bl	dump_str_single


	/* Print HNF registers */
	adr	x11, ccn504_hnf_header
	bl	dump_str_no_reg


	adr	x11, ccn504_hnf_regs
	ldr	x10, =(DICKENS_BASE_X9 + CCN_HNF_NODE(0))	
	add	x10, x10, #SNOOP_CTRL
	ldr	x9, [x10]
	bl	dump_str_single
	add	x10, x10, #ERR_SYNDROME_REG0
	ldr	x9, [x10]
	bl	dump_str_single
	add	x10, x10, #ERR_SYNDROME_REG1
	ldr	x9, [x10]
	bl	dump_str_single

	adr	x11, ccn504_hnf_regs
	ldr	x10, =(DICKENS_BASE_X9 + CCN_HNF_NODE(1))	
	add	x10, x10, #SNOOP_CTRL
	ldr	x9, [x10]
	bl	dump_str_single
	add	x10, x10, #ERR_SYNDROME_REG0
	ldr	x9, [x10]
	bl	dump_str_single
	add	x10, x10, #ERR_SYNDROME_REG1
	ldr	x9, [x10]
	bl	dump_str_single

	adr	x11, ccn504_hnf_regs
	ldr	x10, =(DICKENS_BASE_X9 + CCN_HNF_NODE(2))	
	add	x10, x10, #SNOOP_CTRL
	ldr	x9, [x10]
	bl	dump_str_single
	add	x10, x10, #ERR_SYNDROME_REG0
	ldr	x9, [x10]
	bl	dump_str_single
	add	x10, x10, #ERR_SYNDROME_REG1
	ldr	x9, [x10]
	bl	dump_str_single

	adr	x11, ccn504_hnf_regs
	ldr	x10, =(DICKENS_BASE_X9 + CCN_HNF_NODE(3))	
	add	x10, x10, #SNOOP_CTRL
	ldr	x9, [x10]
	bl	dump_str_single
	add	x10, x10, #ERR_SYNDROME_REG0
	ldr	x9, [x10]
	bl	dump_str_single
	add	x10, x10, #ERR_SYNDROME_REG1
	ldr	x9, [x10]
	bl	dump_str_single

	adr	x11, ccn504_hnf_regs
	ldr	x10, =(DICKENS_BASE_X9 + CCN_HNF_NODE(4))	
	add	x10, x10, #SNOOP_CTRL
	ldr	x9, [x10]
	bl	dump_str_single
	add	x10, x10, #ERR_SYNDROME_REG0
	ldr	x9, [x10]
	bl	dump_str_single
	add	x10, x10, #ERR_SYNDROME_REG1
	ldr	x9, [x10]
	bl	dump_str_single

	adr	x11, ccn504_hnf_regs
	ldr	x10, =(DICKENS_BASE_X9 + CCN_HNF_NODE(5))	
	add	x10, x10, #SNOOP_CTRL
	ldr	x9, [x10]
	bl	dump_str_single
	add	x10, x10, #ERR_SYNDROME_REG0
	ldr	x9, [x10]
	bl	dump_str_single
	add	x10, x10, #ERR_SYNDROME_REG1
	ldr	x9, [x10]
	bl	dump_str_single

	adr	x11, ccn504_hnf_regs
	ldr	x10, =(DICKENS_BASE_X9 + CCN_HNF_NODE(6))	
	add	x10, x10, #SNOOP_CTRL
	ldr	x9, [x10]
	bl	dump_str_single
	add	x10, x10, #ERR_SYNDROME_REG0
	ldr	x9, [x10]
	bl	dump_str_single
	add	x10, x10, #ERR_SYNDROME_REG1
	ldr	x9, [x10]
	bl	dump_str_single

	adr	x11, ccn504_hnf_regs
	ldr	x10, =(DICKENS_BASE_X9 + CCN_HNF_NODE(7))	
	add	x10, x10, #SNOOP_CTRL
	ldr	x9, [x10]
	bl	dump_str_single
	add	x10, x10, #ERR_SYNDROME_REG0
	ldr	x9, [x10]
	bl	dump_str_single
	add	x10, x10, #ERR_SYNDROME_REG1
	ldr	x9, [x10]
	bl	dump_str_single

	/* Print XP regs */
	adr	x11, ccn504_xp_header
	bl	dump_str_no_reg
	
	adr	x11, ccn504_err_syn_regs
	ldr	x10, =(DICKENS_BASE_X9 + CCN_XP_NODE(0))	
	add	x10, x10, #ERR_SYNDROME_REG0
	ldr	x9, [x10]
	bl	dump_str_single

	adr	x11, ccn504_err_syn_regs
	ldr	x10, =(DICKENS_BASE_X9 + CCN_XP_NODE(1))	
	add	x10, x10, #ERR_SYNDROME_REG0
	ldr	x9, [x10]
	bl	dump_str_single

	adr	x11, ccn504_err_syn_regs
	ldr	x10, =(DICKENS_BASE_X9 + CCN_XP_NODE(2))	
	add	x10, x10, #ERR_SYNDROME_REG0
	ldr	x9, [x10]
	bl	dump_str_single

	adr	x11, ccn504_err_syn_regs
	ldr	x10, =(DICKENS_BASE_X9 + CCN_XP_NODE(3))	
	add	x10, x10, #ERR_SYNDROME_REG0
	ldr	x9, [x10]
	bl	dump_str_single

	adr	x11, ccn504_err_syn_regs
	ldr	x10, =(DICKENS_BASE_X9 + CCN_XP_NODE(4))	
	add	x10, x10, #ERR_SYNDROME_REG0
	ldr	x9, [x10]
	bl	dump_str_single

	adr	x11, ccn504_err_syn_regs
	ldr	x10, =(DICKENS_BASE_X9 + CCN_XP_NODE(5))	
	add	x10, x10, #ERR_SYNDROME_REG0
	ldr	x9, [x10]
	bl	dump_str_single

	adr	x11, ccn504_err_syn_regs
	ldr	x10, =(DICKENS_BASE_X9 + CCN_XP_NODE(6))	
	add	x10, x10, #ERR_SYNDROME_REG0
	ldr	x9, [x10]
	bl	dump_str_single

	adr	x11, ccn504_err_syn_regs
	ldr	x10, =(DICKENS_BASE_X9 + CCN_XP_NODE(7))	
	add	x10, x10, #ERR_SYNDROME_REG0
	ldr	x9, [x10]
	bl	dump_str_single

	adr	x11, ccn504_err_syn_regs
	ldr	x10, =(DICKENS_BASE_X9 + CCN_XP_NODE(8))	
	add	x10, x10, #ERR_SYNDROME_REG0
	ldr	x9, [x10]
	bl	dump_str_single

	adr	x11, ccn504_err_syn_regs
	ldr	x10, =(DICKENS_BASE_X9 + CCN_XP_NODE(9))	
	add	x10, x10, #ERR_SYNDROME_REG0
	ldr	x9, [x10]
	bl	dump_str_single

	mov x30, x12
	ret x30
endfunc dump_ccn504_registers


#endif //CONFIG_DATALOGGER


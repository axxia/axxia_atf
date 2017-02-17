/*
 * Copyright (c) 2013-2014, ARM Limited and Contributors. All rights reserved.
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

#include <arch.h>
#include <asm_macros.S>
#include <bl_common.h>
#include <cortex_a57.h>
#include <cpu_macros.S>
#include <platform_def.h>
#include <axxia_def.h>

	.globl	plat_crash_console_init
	.globl	plat_crash_console_putc
	.globl	plat_report_exception
	.globl	plat_reset_handler
	.globl	plat_my_core_pos
	.globl  plat_is_my_cpu_primary
	.globl	platform_mem_init
	.globl	plat_secondary_cold_boot_setup
	.globl	plat_flush_dcache_l1
	.globl	plat_flush_dcache_l2
	.globl	plat_flush_dcache

	.globl	axxia_sec_entry_point


	/* Define a crash console for the plaform */
#define AXXIA_CRASH_CONSOLE_BASE	PL011_UART0_BASE

/*---------------------------------------------
 * int plat_crash_console_init(void)
 *
 * Function to initialize the crash console
 * without a C Runtime to print crash report.
 * Clobber list : x0, x1, x2
 *---------------------------------------------
 */
func plat_crash_console_init
	mov_imm	x0, AXXIA_CRASH_CONSOLE_BASE
	mov_imm	x1, PL011_UART_CLK
	mov_imm	x2, PL011_BAUDRATE
	b	console_core_init
endfunc plat_crash_console_init

/*---------------------------------------------
 * int plat_crash_console_putc(int c)
 *
 * Function to print a character on the crash
 * console without a C Runtime.
 * Clobber list : x1, x2
 *---------------------------------------------
 */
func plat_crash_console_putc
	mov_imm	x1, AXXIA_CRASH_CONSOLE_BASE
	b	console_core_putc
endfunc plat_crash_console_putc

/*---------------------------------------------
 * void plat_report_exception(unsigned int type)
 *
 * Function to report an unhandled exception
 * with platform-specific means.
 *---------------------------------------------
 */
func plat_report_exception
	ret
endfunc plat_report_exception

/*---------------------------------------------
 * int platform_get_core_pos(void)
 *
 * Return CPU number 0..16 (cluster * 4 + core)
 *---------------------------------------------
 */
func plat_my_core_pos
	mrs	x0, mpidr_el1
	and	x1, x0, #MPIDR_CPU_MASK
	and	x0, x0, #MPIDR_CLUSTER_MASK
	add	x0, x1, x0, LSR #6
	ret
endfunc plat_my_core_pos

/* -----------------------------------------------------
 * unsigned int plat_is_my_cpu_primary(void);
 *
 * This function checks if this is the Primary CPU
 * -----------------------------------------------------
 */
func plat_is_my_cpu_primary
	mrs	x0, mpidr_el1
	and	x0, x0, #(MPIDR_CLUSTER_MASK | MPIDR_CPU_MASK)
	cmp	x0, #AXXIA_PRIMARY_CPU
	cset	x0, eq
	ret
endfunc plat_is_my_cpu_primary

/* -----------------------------------------------------
 * unsigned long plat_get_my_entrypoint (void);
 *
 * Main job of this routine is to distinguish between
 * a cold and warm boot. If the tegra_sec_entry_point for
 * this CPU is present, then it's a warm boot.
 *
 * -----------------------------------------------------
 */
func plat_get_my_entrypoint
	adr	x1, axxia_sec_entry_point
	ldr	x0, [x1]
	ret
endfunc plat_get_my_entrypoint

/* -----------------------------------------------------
 * void plat_secondary_cold_boot_setup (void);
 *
 * This function performs any platform specific actions
 * needed for a secondary cpu after a cold reset. Right
 * now this is a stub function.
 * -----------------------------------------------------
 */
func plat_secondary_cold_boot_setup
	mov	x0, #0
	ret
endfunc plat_secondary_cold_boot_setup

/*-----------------------------------------------------
 * void platform_mem_init(void)
 *
 * We don't need to carry out any memory initialization
 * on Juno. The Secure RAM is accessible straight away.
 *-----------------------------------------------------
 */
func platform_mem_init
	ret
endfunc platform_mem_init

/*-----------------------------------------------------
 * void plat_flush_dcache_l1
 *
 * clean and invalidate one level cache.
 *
 * x0: cache level
 * x1: 0 flush & invalidate, 1 invalidate only
 * x2~x9: clobbered
 *-----------------------------------------------------
 */
func plat_flush_dcache_l1
	mov	x16, x30
	mov	x0, #0
	mov x1, #0
	bl	plat_flush_dcache
	mov	x30, x16
	ret
endfunc plat_flush_dcache_l1

/*-----------------------------------------------------
 * void plat_flush_dcache_l2
 *
 * clean and invalidate one level cache.
 *
 * x0: cache level
 * x1: 0 flush & invalidate, 1 invalidate only
 * x2~x9: clobbered
 *-----------------------------------------------------
 */
func plat_flush_dcache_l2
	mov	x16, x30
	mov	x0, #1
	mov x1, #0
	bl	plat_flush_dcache
	mov	x30, x16
	ret
endfunc plat_flush_dcache_l2

/*
 * void plat_flush_dcache(int invalidate_only)
 *
 * x0: cache level
 * x1: 0 flush & invalidate, 1 invalidate only
 *
 * clean and invalidate all data cache by SET/WAY.
 */
func plat_flush_dcache
	dsb	sy
	mrs	x10, clidr_el1		/* read clidr_el1 */
	lsr	x11, x10, #24
	and	x11, x11, #0x7		/* x11 <- loc */
	cbz	x11, finished		/* if loc is 0, exit */
	mov	x15, x30
	/* x0  <- cache level */
	/* x10 <- clidr_el1 */
	/* x11 <- loc */
	/* x15 <- return address */

	lsl	x12, x0, #1
	add	x12, x12, x0		/* x0 <- tripled cache level */
	lsr	x12, x10, x12
	and	x12, x12, #7		/* x12 <- cache type */
	cmp	x12, #2
	b.lt	cont			/* skip if no cache or icache */
	bl	plat_flush_dcache_level	/* x1 = 0 flush, 1 invalidate */

cont:
	mov	x0, #0
	msr	csselr_el1, x0		/* restore csselr_el1 */
	dsb	sy
	isb
	mov	x30, x15

finished:
	ret
endfunc plat_flush_dcache

/*
 * void plat_flush_dcache_level(level)
 *
 * clean and invalidate one level cache.
 *
 * x0: cache level
 * x1: 0 flush & invalidate, 1 invalidate only
 * x2~x9: clobbered
 */
func plat_flush_dcache_level
	lsl	x12, x0, #1
	msr	csselr_el1, x12		/* select cache level */
	isb				/* sync change of cssidr_el1 */
	mrs	x6, ccsidr_el1		/* read the new cssidr_el1 */
	and	x2, x6, #7		/* x2 <- log2(cache line size)-4 */
	add	x2, x2, #4		/* x2 <- log2(cache line size) */
	mov	x3, #0x3ff
	and	x3, x3, x6, lsr #3	/* x3 <- max number of #ways */
	clz	w5, w3			/* bit position of #ways */
	mov	x4, #0x7fff
	and	x4, x4, x6, lsr #13	/* x4 <- max number of #sets */
	/* x12 <- cache level << 1 */
	/* x2 <- line length offset */
	/* x3 <- number of cache ways - 1 */
	/* x4 <- number of cache sets - 1 */
	/* x5 <- bit position of #ways */

loop_set:
	mov	x6, x3			/* x6 <- working copy of #ways */
loop_way:
	lsl	x7, x6, x5
	orr	x9, x12, x7		/* map way and level to cisw value */
	lsl	x7, x4, x2
	orr	x9, x9, x7		/* map set number to cisw value */
	tbz	w1, #0, 1f
	dc	isw, x9
	b	2f
1:	dc	cisw, x9		/* clean & invalidate by set/way */
2:	subs	x6, x6, #1		/* decrement the way */
	b.ge	loop_way
	subs	x4, x4, #1		/* decrement the set */
	b.ge	loop_set

	ret
endfunc plat_flush_dcache_level

	.data
	.align 3

	/* --------------------------------------------------
	 * CPU Secure entry point - resume from suspend
	 * --------------------------------------------------
	 */
axxia_sec_entry_point:
	.quad	0

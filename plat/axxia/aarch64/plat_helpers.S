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
#include "../axxia_def.h"

	.globl	plat_crash_console_init
	.globl	plat_crash_console_putc
	.globl	plat_report_exception
	.globl	plat_reset_handler
	.globl	platform_get_core_pos
	.globl	platform_mem_init

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

/*---------------------------------------------
 * void plat_report_exception(unsigned int type)
 *
 * Function to report an unhandled exception
 * with platform-specific means.
 *---------------------------------------------
 */
func plat_report_exception
	ret

/*---------------------------------------------
 * int platform_get_core_pos(void)
 *
 * Return CPU number 0..16 (cluster * 4 + core)
 *---------------------------------------------
 */
func platform_get_core_pos
	and	x1, x0, #MPIDR_CPU_MASK
	and	x0, x0, #MPIDR_CLUSTER_MASK
	add	x0, x1, x0, LSR #6
	ret

/*-----------------------------------------------------
 * void platform_mem_init(void)
 *
 * We don't need to carry out any memory initialization
 * on Juno. The Secure RAM is accessible straight away.
 *-----------------------------------------------------
 */
func platform_mem_init
	ret


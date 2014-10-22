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
#include "../axxia_def.h"

	.globl	platform_is_primary_cpu
	.globl	platform_get_entrypoint
	.globl	platform_cold_boot_init
	.globl	plat_secondary_cold_boot_setup

	/* -----------------------------------------------------
	 * unsigned int platform_is_primary_cpu (unsigned int mpid);
	 *
	 * Given the mpidr say whether this cpu is the primary
	 * cpu (applicable ony after a cold boot)
	 * -----------------------------------------------------
	 */
func platform_is_primary_cpu
	and	x0, x0, #(MPIDR_CLUSTER_MASK | MPIDR_CPU_MASK)
	cmp	x0, #0
	cset	x0, eq
	ret

	/* -----------------------------------------------------
	 * void plat_secondary_cold_boot_setup (void);
	 *
	 * This function performs any platform specific actions
	 * needed for a secondary cpu after a cold reset e.g
	 * mark the cpu's presence, mechanism to place it in a
	 * holding pen etc.
	 * -----------------------------------------------------
	 */
func plat_secondary_cold_boot_setup
	wfe
	ret
cb_panic:
	b	cb_panic


	/* -----------------------------------------------------
	 * void platform_get_entrypoint (unsigned int mpid);
	 *
	 * Main job of this routine is to distinguish between
	 * a cold and warm boot.
	 * On a cold boot the secondaries first wait for the
	 * platform to be initialized after which they are
	 * hotplugged in. The primary proceeds to perform the
	 * platform initialization.
	 * On a warm boot, each cpu jumps to the address in its
	 * mailbox.
	 *
	 * TODO: Not a good idea to save lr in a temp reg
	 * -----------------------------------------------------
	 */
func platform_get_entrypoint
	and	x0, x0, #(MPIDR_CLUSTER_MASK | MPIDR_CPU_MASK)
	cbz	x0, 1f
	mov_imm	x0, SYSCON_BASE
	ldr	x0, [x0, #0xe0]
	mov_imm	x1, 0x10000
2:	sub	x1, x1, #1
	cbnz	x1, 2b
1:	ret	x30


	/* -----------------------------------------------------
	 * void platform_cold_boot_init (bl1_main function);
	 *
	 * Routine called only by the primary cpu after a cold
	 * boot to perform early platform initialization
	 * -----------------------------------------------------
	 */
func platform_cold_boot_init
	mov	x20, x0

	/* ---------------------------------------------
	 * Give ourselves a small coherent stack to
	 * ease the pain of initializing the MMU and
	 * CCI in assembler
	 * ---------------------------------------------
	 */
	mrs	x0, mpidr_el1
	bl	platform_set_coherent_stack

	/* ---------------------------------------------
	 * Architectural init. can be generic e.g.
	 * enabling stack alignment and platform spec-
	 * ific e.g. MMU & page table setup as per the
	 * platform memory map. Perform the latter here
	 * and the former in bl1_main.
	 * ---------------------------------------------
	 */
	bl	bl1_early_platform_setup
	bl	bl1_plat_arch_setup

	/* ---------------------------------------------
	 * Give ourselves a stack allocated in Normal
	 * -IS-WBWA memory
	 * ---------------------------------------------
	 */
	mrs	x0, mpidr_el1
	bl	platform_set_stack

	/* ---------------------------------------------
	 * Jump to the main function. Returning from it
	 * is a terminal error.
	 * ---------------------------------------------
	 */
	blr	x20

cb_init_panic:
	b	cb_init_panic

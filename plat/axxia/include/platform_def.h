/*
 * Copyright (c) 2014, ARM Limited and Contributors. All rights reserved.
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

#ifndef __PLATFORM_DEF_H__
#define __PLATFORM_DEF_H__

#include <arch.h>

/*******************************************************************************
 * Platform binary types for linking
 ******************************************************************************/
#define PLATFORM_LINKER_FORMAT          "elf64-littleaarch64"
#define PLATFORM_LINKER_ARCH            aarch64

/*******************************************************************************
 * Generic platform constants
 ******************************************************************************/

/* Size of cacheable stacks */
#define PLATFORM_STACK_SIZE		0x800

#define FIRMWARE_WELCOME_STR		"Booting Trusted Firmware\n"

/* Trusted Boot Firmware BL2 */
#define BL2_IMAGE_NAME			"bl2.bin"

/* EL3 Runtime Firmware BL3-1 */
#define BL31_IMAGE_NAME			"bl31.bin"

/* Secure Payload BL3-2 (Trusted OS) */
#define BL32_IMAGE_NAME			"bl32.bin"

/* Non-Trusted Firmware BL3-3 */
#define BL33_IMAGE_NAME			"u-boot.bin" /* e.g. UEFI */

/* Firmware Image Package */
#define FIP_IMAGE_NAME			"fip.bin"

#define PLATFORM_CACHE_LINE_SIZE	64
#define PLATFORM_SYSTEM_COUNT           1
#define PLATFORM_CLUSTER_COUNT		8
#define PLATFORM_CORE_COUNT             32
#define PLATFORM_NUM_AFFS		(PLATFORM_SYSTEM_COUNT +  \
					 PLATFORM_CLUSTER_COUNT + \
					 PLATFORM_CORE_COUNT)
#define PLATFORM_MAX_AFFLVL             MPIDR_AFFLVL2
#define MAX_IO_DEVICES			3
#define MAX_IO_HANDLES			4

/*******************************************************************************
 * Platform memory map related constants
 ******************************************************************************/
#define TZROM_BASE			0x80FFFF0000
#define TZROM_SIZE			0x0000008000

#define TZRAM_BASE			0x8031001000
#define TZRAM_SIZE			0x000003f000

/*******************************************************************************
 * BL1 specific defines.
 * BL1 RW data is relocated from ROM to RAM at runtime so we need 2 base
 * addresses.
 ******************************************************************************/
#define BL1_RO_BASE			TZROM_BASE
#define BL1_RO_LIMIT			(TZROM_BASE + TZROM_SIZE)
#define BL1_RW_BASE			TZRAM_BASE
#define BL1_RW_LIMIT			BL31_BASE

/*******************************************************************************
 * BL2 specific defines.
 ******************************************************************************/
#define BL2_BASE			TZRAM_BASE
#define BL2_LIMIT			(TZRAM_BASE + TZRAM_SIZE)

/*******************************************************************************
 * BL3-1 specific defines.
 ******************************************************************************/
#define BL31_BASE			TZRAM_BASE
#define BL31_LIMIT			(TZRAM_BASE + TZRAM_SIZE)

/*******************************************************************************
 * Load address of BL3-3
 ******************************************************************************/
#define NS_IMAGE_OFFSET			0x00000000

/*******************************************************************************
 * Platform specific page table and MMU setup constants
 ******************************************************************************/
#define ADDR_SPACE_SIZE                 (1ULL << 40)
#define MAX_XLAT_TABLES			8
#define MAX_MMAP_REGIONS                16

/*******************************************************************************
 * ID of the secure physical generic timer interrupt used by the TSP
 ******************************************************************************/
#define TSP_IRQ_SEC_PHY_TIMER		IRQ_SEC_PHY_TIMER

/*******************************************************************************
 * Declarations and constants to access the mailboxes safely. Each mailbox is
 * aligned on the biggest cache line size in the platform. This is known only
 * to the platform as it might have a combination of integrated and external
 * caches. Such alignment ensures that two maiboxes do not sit on the same cache
 * line at any cache level. They could belong to different cpus/clusters &
 * get written while being protected by different locks causing corruption of
 * a valid mailbox address.
 ******************************************************************************/
#define CACHE_WRITEBACK_SHIFT		6
#define CACHE_WRITEBACK_GRANULE		(1 << CACHE_WRITEBACK_SHIFT)

#endif /* __PLATFORM_DEF_H__ */

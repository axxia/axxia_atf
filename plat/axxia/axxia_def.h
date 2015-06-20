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

#ifndef __AXXIA_DEF_H__
#define __AXXIA_DEF_H__

/*******************************************************************************
 * Axxia memory map related constants
 ******************************************************************************/

#define DRAM_BASE		0x0000000000
#define DRAM_SIZE		0x0040000000

/* XLF Base Peripherals */
#define DEVICE0_BASE		0x4000000000
#define DEVICE0_SIZE		0x0040000000

/* X9 Base Peripherals */
#define DEVICE1_BASE		0x8000000000
#define DEVICE1_SIZE		0x0040000000

/* AXIS Peripherals */
#define DEVICE2_BASE		0x8080000000
#define DEVICE2_SIZE		0x0040000000

/* GIC-500 */
#define GICC_BASE		0x8001000000
#define GICD_BASE_X9		0x8010000000
#define GICD_BASE_XLF		0x8010010000
#define GICR_BASE_X9		0x8010200000
#define GICR_BASE_XLF		0x8010100000

/* SYSCON */
#define SYSCON_BASE		0x8002C00000

/* DICKENS */
#define DICKENS_BASE_X9         0x8000000000
#define DICKENS_BASE_XLF        0x4000000000

/* NCA */
#define NCA_X9_BASE             0x8031080000
#define NCA_XLF_BASE            0x8020000000

/* CCN-504 */
#define CCN504_BASE		0x0000000000

/* UART0-3 */
#define PL011_UART0_BASE	0x8080000000
#define PL011_UART1_BASE	0x8080010000
#define PL011_UART2_BASE	0x8080020000
#define PL011_UART3_BASE	0x8080030000

#define PL011_BAUDRATE		115200
#define PL011_UART_CLK		24000000

/* Secure Interrupts */
#define IRQ_SEC_PHY_TIMER	10

#endif /* __AXXIA_DEF_H__ */

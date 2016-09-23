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

#ifndef __ASSEMBLY__

typedef enum {
	AXXIA_5600 = 0,
	AXXIA_6700 = 1
} axxia_target_t;

typedef enum {
	AXXIA_SIM = 0,
	AXXIA_EMU = 1,
	AXXIA_HW = 2
} axxia_platform_t;

typedef enum {
	AXXIA_NONE = 0,
	AXXIA_SYSCACHE_ONLY = 1
} axxia_option_t;

typedef struct axxia_configuration {
	axxia_target_t target;
	axxia_platform_t platform;
	axxia_option_t option;
	unsigned int per_clock_hz;
	unsigned int baud_rate;
} axxia_configuration_t;

extern axxia_configuration_t axxia_configuration;

#define IS_5600() (AXXIA_5600 == axxia_configuration.target)
#define IS_6700() (AXXIA_6700 == axxia_configuration.target)
#define IS_SIM()  (AXXIA_SIM == axxia_configuration.platform)
#define IS_EMU()  (AXXIA_EMU == axxia_configuration.platform)
#define IS_HW()   (AXXIA_HW == axxia_configuration.platform)
#define IS_SYSCACHE_ONLY() (AXXIA_SYSCACHE_ONLY == axxia_configuration.option)

#endif

/*******************************************************************************
 * Axxia memory map related constants
 ******************************************************************************/

#define DRAM_BASE		0x0000000000
#define DRAM_SIZE		0x0040000000

/* XLF CCN */
#define XLF_CCN_BASE	        0x4000000000
#define XLF_CCN_SIZE	        0x0040000000

/* X9 CCN and XLF/X9 GIC */
#define DEVICE0_BASE		0x8000000000
#define DEVICE0_SIZE		0x0040000000

/* AXIS Peripherals */
#define DEVICE1_BASE		0x8080000000
#define DEVICE1_SIZE		0x0040000000

/* GIC-500 */
#define GICC_BASE		0x8001000000
#define GICD_BASE_X9		0x8010000000
#define GICD_BASE_XLF		0x8010000000
#define GICR_BASE_X9		0x8010200000
#define GICR_BASE_XLF		0x8010400000

/* SYSCON */
#define SYSCON_BASE		0x8002C00000

/* DICKENS */
#define DICKENS_BASE_X9         0x8000000000
#define DICKENS_BASE_XLF        0x4000000000

/* ELM */
#define ELM_BASE                0x8003C00000

/* NCA */
#define NCA_X9_BASE             0x8031080000
#define NCA_XLF_BASE            0x8020000000

/* TZC */
#define TZC_X9_BASE             0x8004140000
#define TZC_XLF_BASE            0x8005040000

/* CCN-504 */
#define CCN504_BASE		0x0000000000

/* GPREGs and SCBs */
#define MMAP_GPREG              0x8032900000 /* 0x170.0 */
#define MMAP_SCB                0x8032000000 /* 0x170.1 */
#define PERIPH_GPREG            0x8080230000 /* 0x171.0 */
#define PERIPH_SCB              0x8080400000 /* 0x171.1 */

/* DSPs, XLF Only */
#define NCAP                    0x8004000000 /* 0x168.0 */
#define CDC0                    0x8004400000 /* 0x169.0 */
#define CDC1                    0x8004420000 /* 0x169.1 */
#define CDC2                    0x8004440000 /* 0x169.2 */
#define CDC3                    0x8004460000 /* 0x169.3 */

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

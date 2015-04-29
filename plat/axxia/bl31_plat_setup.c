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
#include <arch_helpers.h>
#include <assert.h>
#include <bl31.h>
#include <bl_common.h>
#include <ccn504.h>
#include <arm_gic.h>
#include <console.h>
#include <mmio.h>
#include <platform.h>
#include <debug.h>
#include <stddef.h>
#include "axxia_def.h"
#include "axxia_private.h"

/*******************************************************************************
 * Declarations of linker defined symbols which will help us find the layout
 * of trusted RAM
 ******************************************************************************/
extern unsigned long __RO_START__;
extern unsigned long __RO_END__;

extern unsigned long __COHERENT_RAM_START__;
extern unsigned long __COHERENT_RAM_END__;

/*
 * The next 2 constants identify the extents of the code & RO data region.
 * These addresses are used by the MMU setup code and therefore they must be
 * page-aligned.  It is the responsibility of the linker script to ensure that
 * __RO_START__ and __RO_END__ linker symbols refer to page-aligned addresses.
 */
#define BL31_RO_BASE (unsigned long)(&__RO_START__)
#define BL31_RO_LIMIT (unsigned long)(&__RO_END__)

/*
 * The next 2 constants identify the extents of the coherent memory region.
 * These addresses are used by the MMU setup code and therefore they must be
 * page-aligned.  It is the responsibility of the linker script to ensure that
 * __COHERENT_RAM_START__ and __COHERENT_RAM_END__ linker symbols
 * refer to page-aligned addresses.
 */
#define BL31_COHERENT_RAM_BASE (unsigned long)(&__COHERENT_RAM_START__)
#define BL31_COHERENT_RAM_LIMIT (unsigned long)(&__COHERENT_RAM_END__)

/******************************************************************************
 * Placeholder variables for copying the arguments that have been passed to
 * BL3-1 from BL2.
 ******************************************************************************/
static entry_point_info_t bl32_ep_info;
static entry_point_info_t bl33_ep_info;

/*******************************************************************************
 * Return a pointer to the 'entry_point_info' structure of the next image for
 * the security state specified. BL3-3 corresponds to the non-secure image type
 * while BL3-2 corresponds to the secure image type. A NULL pointer is returned
 * if the image does not exist.
 ******************************************************************************/
entry_point_info_t *bl31_plat_get_next_image_ep_info(uint32_t type)
{
	entry_point_info_t *next_image_info;

	next_image_info = (type == NON_SECURE) ? &bl33_ep_info : &bl32_ep_info;

#if 1
	/* Temp hack to use bl33 (u-boot) pre-loaded in RAM */
	bl33_ep_info.pc = 0x00000000;
	bl33_ep_info.spsr = SPSR_64(MODE_EL2, MODE_SP_ELX, DISABLE_ALL_EXCEPTIONS);
	bl33_ep_info.args.arg0 = read_mpidr() & 0xffff;
	SET_PARAM_HEAD(&bl33_ep_info, PARAM_IMAGE_BINARY, VERSION_1, NON_SECURE);
#endif

	return next_image_info;
}

/*******************************************************************************
 * Perform any BL3-1 specific platform actions. Here is an opportunity to copy
 * parameters passed by the calling EL (S-EL1 in BL2 & S-EL3 in BL1) before they
 * are lost (potentially). This needs to be done before the MMU is initialized
 * so that the memory layout can be used while creating page tables. Also, BL2
 * has flushed this information to memory, so we are guaranteed to pick up good
 * data
 ******************************************************************************/
void bl31_early_platform_setup(bl31_params_t *from_bl2,
			       void *plat_params_from_bl2)
{
	/* Initialize the console to provide early debug support */
	console_init(PL011_UART0_BASE, PL011_UART_CLK, PL011_BAUDRATE);

	/*
	 * Initialise the CCN-504 driver for BL31 so that it is accessible
	 * after a warm boot. BL1 should have already enabled CCI coherency for
	 * this cluster during cold boot.
	 */
	ccn_init(CCN504_BASE);

	/*
	 * Check params passed from BL2 should not be NULL,
	 */
#if 0
	assert(from_bl2 != NULL);
	assert(from_bl2->h.type == PARAM_BL31);
	assert(from_bl2->h.version >= VERSION_1);

	/*
	 * Copy BL3-2 and BL3-3 entry point information.
	 * They are stored in Secure RAM, in BL2's address space.
	 */
	bl32_ep_info = *from_bl2->bl32_ep_info;
	bl33_ep_info = *from_bl2->bl33_ep_info;
#endif
}

const unsigned int axxia_sec_irq[] = {
	0,
};

/*******************************************************************************
 * Initialize the MHU and the GIC.
 ******************************************************************************/
void bl31_platform_setup(void)
{
	/* Initialize the gic cpu and distributor interfaces */
#if 0
	gic_init(GICC_BASE, GICD_BASE, GICR_BASE, axxia_sec_irq, 0);
	gic_setup();
#else
	axxia_gic_setup();
#endif
}

/*******************************************************************************
 * Perform the very early platform specific architectural setup here. At the
 * moment this is only intializes the mmu in a quick and dirty way.
 ******************************************************************************/
void bl31_plat_arch_setup()
{
	configure_mmu_el3(BL31_RO_BASE,
			  BL31_COHERENT_RAM_LIMIT - BL31_RO_BASE,
			  BL31_RO_BASE,
			  BL31_RO_LIMIT,
			  BL31_COHERENT_RAM_BASE,
			  BL31_COHERENT_RAM_LIMIT);
}

void
bl31_plat_enable_mmu(uint32_t flags)
{
	return;
}

#include <gic_v2.h>

void
display_gic(void)
{
#if 0
	/* GICC */
	tf_printf("*** BL31 GICC Registers ***\n");
	tf_printf("      GICC_CTLR: 0x%x\n"
		  "       GICC_PMR: 0x%x\n"
		  "       GICC_BPR: 0x%x\n"
		  "       GICC_IAR: 0x%x\n"
		  "      GICC_EOIR: 0x%x\n"
		  "       GICC_RPR: 0x%x\n"
		  "     GICC_HPPIR: 0x%x\n"
		  "    GICC_AHPPIR: 0x%x\n"
		  "      GICC_IIDR: 0x%x\n"
		  "       GICC_DIR: 0x%x\n",
		  mmio_read_32(GICC_BASE + GICC_CTLR),
		  mmio_read_32(GICC_BASE + GICC_PMR),
		  mmio_read_32(GICC_BASE + GICC_BPR),
		  mmio_read_32(GICC_BASE + GICC_IAR),
		  mmio_read_32(GICC_BASE + GICC_EOIR),
		  mmio_read_32(GICC_BASE + GICC_RPR),
		  mmio_read_32(GICC_BASE + GICC_HPPIR),
		  mmio_read_32(GICC_BASE + GICC_AHPPIR),
		  mmio_read_32(GICC_BASE + GICC_IIDR),
		  mmio_read_32(GICC_BASE + GICC_DIR));

	/* GICD */
	tf_printf("*** BL31 GICD Registers ***\n");
	tf_printf("      GICD_CTLR: 0x%x\n"
		  "     GICD_TYPER: 0x%x\n"
		  "   GICD_IGROUPR: 0x%x\n"
		  " GICD_ISENABLER: 0x%x\n"
		  " GICD_ICENABLER: 0x%x\n"
		  "   GICD_ISPENDR: 0x%x\n"
		  "   GICD_ICPENDR: 0x%x\n"
		  " GICD_ISACTIVER: 0x%x\n"
		  " GICD_ICACTIVER: 0x%x\n"
		  "GICD_IPRIORITYR: 0x%x\n"
		  " GICD_ITARGETSR: 0x%x\n"
		  "     GICD_ICFGR: 0x%x\n"
		  "      GICD_SGIR: 0x%x\n"
		  " GICD_CPENDSGIR: 0x%x\n"
		  " GICD_SPENDSGIR: 0x%x\n",
		  mmio_read_32(GICD_BASE + GICD_CTLR),
		  mmio_read_32(GICD_BASE + GICD_TYPER),
		  mmio_read_32(GICD_BASE + GICD_IGROUPR),
		  mmio_read_32(GICD_BASE + GICD_ISENABLER),
		  mmio_read_32(GICD_BASE + GICD_ICENABLER),
		  mmio_read_32(GICD_BASE + GICD_ISPENDR),
		  mmio_read_32(GICD_BASE + GICD_ICPENDR),
		  mmio_read_32(GICD_BASE + GICD_ISACTIVER),
		  mmio_read_32(GICD_BASE + GICD_ICACTIVER),
		  mmio_read_32(GICD_BASE + GICD_IPRIORITYR),
		  mmio_read_32(GICD_BASE + GICD_ITARGETSR),
		  mmio_read_32(GICD_BASE + GICD_ICFGR),
		  mmio_read_32(GICD_BASE + GICD_SGIR),
		  mmio_read_32(GICD_BASE + GICD_CPENDSGIR),
		  mmio_read_32(GICD_BASE + GICD_SPENDSGIR));

	/* GICR */
	tf_printf("*** BL31 GICR Registers ***\n");
	tf_printf("      GICR_CTLR: 0x%x\n"
		  "     GICD_TYPER: 0x%x\n"
		  "   GICD_IGROUPR: 0x%x\n"
		  " GICD_ISENABLER: 0x%x\n"
		  " GICD_ICENABLER: 0x%x\n"
		  "   GICD_ISPENDR: 0x%x\n"
		  "   GICD_ICPENDR: 0x%x\n"
		  " GICD_ISACTIVER: 0x%x\n"
		  " GICD_ICACTIVER: 0x%x\n"
		  "GICD_IPRIORITYR: 0x%x\n"
		  " GICD_ITARGETSR: 0x%x\n"
		  "     GICD_ICFGR: 0x%x\n"
		  "      GICD_SGIR: 0x%x\n"
		  " GICD_CPENDSGIR: 0x%x\n"
		  " GICD_SPENDSGIR: 0x%x\n",
		  mmio_read_32(GICR_BASE + GICD_CTLR),
		  mmio_read_32(GICR_BASE + GICD_TYPER),
		  mmio_read_32(GICR_BASE + GICD_IGROUPR),
		  mmio_read_32(GICR_BASE + GICD_ISENABLER),
		  mmio_read_32(GICR_BASE + GICD_ICENABLER),
		  mmio_read_32(GICR_BASE + GICD_ISPENDR),
		  mmio_read_32(GICR_BASE + GICD_ICPENDR),
		  mmio_read_32(GICR_BASE + GICD_ISACTIVER),
		  mmio_read_32(GICR_BASE + GICD_ICACTIVER),
		  mmio_read_32(GICR_BASE + GICD_IPRIORITYR),
		  mmio_read_32(GICR_BASE + GICD_ITARGETSR),
		  mmio_read_32(GICR_BASE + GICD_ICFGR),
		  mmio_read_32(GICR_BASE + GICD_SGIR),
		  mmio_read_32(GICR_BASE + GICD_CPENDSGIR),
		  mmio_read_32(GICR_BASE + GICD_SPENDSGIR));
#endif

	return;
}

#include <mmio.h>

int
is_x9(void)
{
	unsigned int pfuse;

	pfuse = mmio_read_32(SYSCON_BASE + 0x34);

	return (0xb == (pfuse & 0x1f));
}

int
is_simulation(void)
{
	unsigned long *nca_e0;

	if (is_x9())
		nca_e0 = (unsigned long *)(NCA_X9_BASE + 0xe0);
	else
		nca_e0 = (unsigned long *)(NCA_XLF_BASE + 0xe0);

	return (0 == *nca_e0);
}

static unsigned long
get_cntr_frq(void)
{
	unsigned long cntfrq;

	__asm__ __volatile__ ("mrs %0, cntfrq_el0" : "=r" (cntfrq));

	return cntfrq;
}

static unsigned long
get_cntr(void)
{
	unsigned long cntpct;

	__asm__ __volatile__ ("isb ; mrs %0, cntpct_el0" : "=r" (cntpct));

	return cntpct;
}

static void
udelay(unsigned long us)
{
	unsigned long frequency;
	unsigned long cycles;
	unsigned long cycle;

	frequency = get_cntr_frq();
	cycles = (us * frequency) / 1000000;
	cycle = get_cntr();
	cycles += cycle;

	do {
		cycle = get_cntr();
	} while (cycle < cycles);

	return;
}

static int
set_l3_state(unsigned int state)
{
	int i;
        unsigned int status;
	int retries;
	unsigned int hnf_offsets[] = {
		0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27
	};
	volatile unsigned long *address;
	unsigned long dickens_base;

	if (is_x9())
		dickens_base = DICKENS_BASE_X9;
	else
		dickens_base = DICKENS_BASE_XLF;

	if (0 != (state & ~0x3))
		return -1;

	for (i = 0; i < (sizeof(hnf_offsets) / sizeof(unsigned int)); ++i) {
		address = (unsigned long *)
			(dickens_base + (0x10000 * hnf_offsets[i]) + 0x10);
		*address = state;
		dsb();
	}

	for (i = 0; i < (sizeof(hnf_offsets) / sizeof(unsigned int)); ++i) {
		retries = 10000;
		address = (unsigned long *)
			(dickens_base + (0x10000 * hnf_offsets[i]) + 0x18);

		do {
			udelay(1);
			status = *address;
		} while ((0 < --retries) && ((state << 2) != (status & 0xf)));

		if (0 == retries)
			return -1;
	}

	return 0;
}

void
flush_l3(void)
{
	int rc;

	rc = set_l3_state(0x1);

	if (0 != rc) {
		printf("Error Setting L3 to SFONLY!\n");

		return;
	}

	rc = set_l3_state(0x3);

	if (0 != rc) {
		printf("Error Setting L3 to FULL!\n");

		return;
	}
}

/*
  ==============================================================================
  Clusters and Coherency
*/

static int number_of_clusters;
static int bit_by_cluster[4];

static int
initialize_cluster_info(void)
{
	number_of_clusters = 3;
	bit_by_cluster[0] = 19;
	bit_by_cluster[1] = 9;
	bit_by_cluster[2] = 1;
	bit_by_cluster[3] = -1;

	return 0;
}

static unsigned long
get_bit_by_cluster(unsigned long cluster)
{
	return bit_by_cluster[cluster];
}

int
set_cluster_coherency(unsigned cluster, unsigned state)
{
	unsigned int sdcr_offsets[] = {
		0x00,		/* This is the DVM */
		0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27
	};
	int i;
	int retries;
	unsigned int mask;
	unsigned int value;
	unsigned long dickens_base;

	if (is_x9())
		dickens_base = DICKENS_BASE_X9;
	else
		dickens_base = DICKENS_BASE_XLF;

	initialize_cluster_info();

	if (1 < cluster)
		return -1;

	printf("%s cluster %d %s the coherency domain.\n",
	       state ? "Adding" : "Removing",
	       cluster,
	       state ? "to" : "from");
	mask = (1 << get_bit_by_cluster(cluster));

	for (i = 0; i < (sizeof(sdcr_offsets) / sizeof(unsigned int)); ++i) {
		unsigned long offset;

		offset = (dickens_base | (sdcr_offsets[i] << 16));

		if (0 == state)
			mmio_write_32((uintptr_t)(offset + 0x220),
				      (unsigned int)mask);
		else
			mmio_write_32((uintptr_t)(offset + 0x210),
				      (unsigned int)mask);

		retries = 1000;

		do {
			--retries;
			value = mmio_read_32(offset + 0x200);

			if (0 == state) {
				if (0 == (mask & value))
					break;
			} else {
				if (mask == (mask & value))
					break;
			}
		} while (0 < retries);

		if (0 == retries)
			return -1;
	}

	return 0;
}

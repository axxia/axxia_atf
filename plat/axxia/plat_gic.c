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

#include <arch_helpers.h>
#include <assert.h>
#include <bl_common.h>
#include <gic_v2.h>
#include <gic_v3.h>
#include <interrupt_mgmt.h>
#include <platform.h>
#include <debug.h>
#include "axxia_def.h"
#include "axxia_private.h"

/* GICv3 distributor control register */
#define CTRL_ARE_NS		(1UL << 5)
#define CTRL_ARE_S		(1UL << 4)
#define ENABLE_GRP1S		(1UL << 2)
#define ENABLE_GRP1NS		(1UL << 1)
#define ENABLE_GRP0S		(1UL << 0)

#define GICD_IGROUPMODR		0xd00

#define GICR_IGROUPR 0x10080
#define GICR_IGROUPMODR 0x10d00
#define GICR_ISENABLER 0x10100

#define ICC_SRE_DFB (1UL << 1)
#define ICC_SRE_DIB (1UL << 2)

DEFINE_RENAME_SYSREG_RW_FUNCS(icc_ctlr_el3, ICC_CTLR_EL3)
DEFINE_RENAME_SYSREG_RW_FUNCS(icc_ctlr_el1, ICC_CTLR_EL1)

/* Value used to initialise Non-Secure irq priorities four at a time */
#define DEFAULT_NS_PRIORITY_X4 \
	(GIC_HIGHEST_NS_PRIORITY | \
	(GIC_HIGHEST_NS_PRIORITY << 8) | \
	(GIC_HIGHEST_NS_PRIORITY << 16) | \
	(GIC_HIGHEST_NS_PRIORITY << 24))


/*******************************************************************************
 * Enable secure interrupts and use FIQs to route them. Disable legacy bypass
 * and set the priority mask register to allow all interrupts to trickle in.
 ******************************************************************************/
void gic_cpuif_setup(void)
{
	unsigned int val, scr_val;
	uintptr_t gicr_base;

	if (is_x9())
		gicr_base = GICR_BASE_X9;
	else
		gicr_base = GICR_BASE_XLF;

	gicr_base = gicv3_get_rdist(gicr_base, read_mpidr());

	if (gicr_base == (uintptr_t)NULL)
		panic();

	val = gicr_read_waker(gicr_base);
	val &= ~WAKER_PS;
	gicr_write_waker(gicr_base, val);
	dsb();

	/* We need to wait for ChildrenAsleep to clear. */
	val = gicr_read_waker(gicr_base);
	while (val & WAKER_CA)
		val = gicr_read_waker(gicr_base);

	/* Mark all 32 PPI interrupts as Group 1 (non-secure) */
	mmio_write_32(gicr_base + GICR_IGROUPR, ~0);
	mmio_write_32(gicr_base + GICR_IGROUPMODR, 0);

	/* Enable SGI 0 */
	mmio_write_32(gicr_base + GICR_ISENABLER, 1);

	val = read_icc_sre_el3();
	val |= ICC_SRE_EN | ICC_SRE_DIB | ICC_SRE_DFB | ICC_SRE_SRE;
	write_icc_sre_el3(val);
	isb();

	write_icc_ctlr_el3(0);
	isb();
	write_icc_ctlr_el1(0);
	isb();
	write_icc_pmr_el1(1<<7);
	isb();

	scr_val = read_scr();
	write_scr(scr_val | SCR_NS_BIT);
	isb();	/* ensure NS=1 takes effect before accessing ICC_SRE_EL2 */

	val = read_icc_sre_el2();
	val |= ICC_SRE_EN | ICC_SRE_DIB | ICC_SRE_DFB | ICC_SRE_SRE;
	write_icc_sre_el2(val);
	isb();

	/* Restore SCR_EL3 */
	write_scr(scr_val);
	isb();	/* ensure NS=0 takes effect immediately */

	write_icc_igrpen1_el3(0x3);
	isb();
}

/*******************************************************************************
 * Place the cpu interface in a state where it can never make a cpu exit wfi as
 * as result of an asserted interrupt. This is critical for powering down a cpu
 ******************************************************************************/
void gic_cpuif_deactivate(uintptr_t gicc_base)
{
	unsigned int val;

	/* Disable secure, non-secure interrupts and disable their bypass */
	val = gicc_read_ctlr(gicc_base);
	val &= ~(ENABLE_GRP0 | ENABLE_GRP1);
	val |= FIQ_BYP_DIS_GRP1 | FIQ_BYP_DIS_GRP0;
	val |= IRQ_BYP_DIS_GRP0 | IRQ_BYP_DIS_GRP1;
	gicc_write_ctlr(gicc_base, val);
}

#if 0
static void gic_set_secure(uintptr_t gicd_base, unsigned id)
{
	/* Set interrupt as Group 0 */
	gicd_clr_igroupr(gicd_base, id);

	/* Set priority to max */
	gicd_set_ipriorityr(gicd_base, id, GIC_HIGHEST_SEC_PRIORITY);
}
#endif

/*******************************************************************************
 * Global gic distributor setup which will be done by the primary cpu after a
 * cold boot. It marks out the secure SPIs, PPIs & SGIs and enables them. It
 * then enables the secure GIC distributor interface.
 ******************************************************************************/
static void gic_distif_setup(uintptr_t gicd_base)
{
	unsigned int ctlr;
	unsigned int i, lines;

	ctlr = gicd_read_ctlr(gicd_base);
	ctlr |= ENABLE_GRP0 | ENABLE_GRP1NS | ENABLE_GRP1S;
	ctlr |= CTRL_ARE_S | CTRL_ARE_NS;
	ctlr |= (1UL << 6);
	gicd_write_ctlr(gicd_base, ctlr);

	/* Mark all lines of SPIs as Group 1 (non-secure) */
	lines = gicd_read_typer(gicd_base) & IT_LINES_NO_MASK;

	for (i = 0; i <= lines; i++) {
		mmio_write_32(gicd_base + GICD_IGROUPR + 4 + i * 4, ~0);
		mmio_write_32(gicd_base + GICD_IGROUPMODR + 4 + i * 4, 0);
	}
}

void axxia_gic_setup(void)
{
	uintptr_t gicd_base_;

	if (is_x9())
		gicd_base_ = GICD_BASE_X9;
	else
		gicd_base_ = GICD_BASE_XLF;

	gic_distif_setup(gicd_base_);
	gic_cpuif_setup();
}

/*******************************************************************************
 * An ARM processor signals interrupt exceptions through the IRQ and FIQ pins.
 * The interrupt controller knows which pin/line it uses to signal a type of
 * interrupt. The platform knows which interrupt controller type is being used
 * in a particular security state e.g. with an ARM GIC, normal world could use
 * the GICv2 features while the secure world could use GICv3 features and vice
 * versa.
 * This function is exported by the platform to let the interrupt management
 * framework determine for a type of interrupt and security state, which line
 * should be used in the SCR_EL3 to control its routing to EL3. The interrupt
 * line is represented as the bit position of the IRQ or FIQ bit in the SCR_EL3.
 ******************************************************************************/
uint32_t plat_interrupt_type_to_line(uint32_t type, uint32_t security_state)
{
  /* FIX THISSSS */
#if 0
	assert(type == INTR_TYPE_S_EL1 ||
	       type == INTR_TYPE_EL3 ||
	       type == INTR_TYPE_NS);

	assert(sec_state_is_valid(security_state));

	/*
	 * We ignore the security state parameter because Juno is GICv2 only
	 * so both normal and secure worlds are using ARM GICv2.
	 */
	return gicv2_interrupt_type_to_line(GICC_BASE, type);
#else
	return __builtin_ctz(SCR_IRQ_BIT);
#endif
}

/*******************************************************************************
 * This function returns the type of the highest priority pending interrupt at
 * the GIC cpu interface. INTR_TYPE_INVAL is returned when there is no
 * interrupt pending.
 ******************************************************************************/
uint32_t plat_ic_get_pending_interrupt_type(void)
{
	uint32_t id;

	id = gicc_read_hppir(GICC_BASE);

	/* Assume that all secure interrupts are S-EL1 interrupts */
	if (id < 1022)
		return INTR_TYPE_S_EL1;

	if (id == GIC_SPURIOUS_INTERRUPT)
		return INTR_TYPE_INVAL;

	return INTR_TYPE_NS;
}

/*******************************************************************************
 * This function returns the id of the highest priority pending interrupt at
 * the GIC cpu interface. INTR_ID_UNAVAILABLE is returned when there is no
 * interrupt pending.
 ******************************************************************************/
uint32_t plat_ic_get_pending_interrupt_id(void)
{
	uint32_t id;

	id = gicc_read_hppir(GICC_BASE);

	if (id < 1022)
		return id;

	if (id == 1023)
		return INTR_ID_UNAVAILABLE;

	/*
	 * Find out which non-secure interrupt it is under the assumption that
	 * the GICC_CTLR.AckCtl bit is 0.
	 */
	return gicc_read_ahppir(GICC_BASE);
}

/*******************************************************************************
 * This functions reads the GIC cpu interface Interrupt Acknowledge register
 * to start handling the pending interrupt. It returns the contents of the IAR.
 ******************************************************************************/
uint32_t plat_ic_acknowledge_interrupt(void)
{
	return gicc_read_IAR(GICC_BASE);
}

/*******************************************************************************
 * This functions writes the GIC cpu interface End Of Interrupt register with
 * the passed value to finish handling the active interrupt
 ******************************************************************************/
void plat_ic_end_of_interrupt(uint32_t id)
{
	gicc_write_EOIR(GICC_BASE, id);
}

/*******************************************************************************
 * This function returns the type of the interrupt id depending upon the group
 * this interrupt has been configured under by the interrupt controller i.e.
 * group0 or group1.
 ******************************************************************************/
uint32_t plat_ic_get_interrupt_type(uint32_t id)
{
	uint32_t group;
	uintptr_t gicd_base_;

	if (is_x9())
		gicd_base_ = GICD_BASE_X9;
	else
		gicd_base_ = GICD_BASE_XLF;

	group = gicd_get_igroupr(gicd_base_, id);

	/* Assume that all secure interrupts are S-EL1 interrupts */
	if (group == GRP0)
		return INTR_TYPE_S_EL1;
	else
		return INTR_TYPE_NS;
}

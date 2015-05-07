/*
 * Copyright (c) 2013, ARM Limited and Contributors. All rights reserved.
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

#include <assert.h>
#include <arch_helpers.h>
#include <debug.h>
#include <ccn504.h>
#include <errno.h>
#include <platform.h>
#include <platform_def.h>
#include <psci.h>
#include <mmio.h>
#include <debug.h>
#include "axxia_def.h"
#include "axxia_private.h"

#define SYSCON_RESET_KEY	(SYSCON_BASE + 0x2000)
#define SYSCON_RESET_CTRL	(SYSCON_BASE + 0x2008)
#define   RSTCTL_RST_CHIP       (1<<1)
#define   RSTCTL_RST_SYS        (1<<0)
#define SYSCON_RESET_HOLD	(SYSCON_BASE + 0x2010)

/*
 * Handler called when an affinity instance is about to be turned on. The level
 * and mpidr determine the affinity instance.
 */
int32_t axxia_affinst_on(uint64_t mpidr,
			 uint64_t sec_entrypoint,
			 uint64_t ns_entrypoint,
			 uint32_t afflvl,
			 uint32_t state)
{
	uint32_t id, hold;

	switch (afflvl) {
	case MPIDR_AFFLVL1:
		id = (mpidr >> MPIDR_AFF1_SHIFT) & MPIDR_AFFLVL_MASK;
		break;
	case MPIDR_AFFLVL0:
		id = platform_get_core_pos(mpidr);
		mmio_write_32(0x8031000000,
			      (0x14000000 |
			       (sec_entrypoint - 0x8031000000) / 4));
		hold = mmio_read_32(SYSCON_RESET_HOLD);
		hold &= ~(1 << id);
		mmio_write_32(SYSCON_RESET_KEY, 0xab);
		mmio_write_32(SYSCON_RESET_HOLD, hold | (1 << id));
		mmio_write_32(SYSCON_RESET_HOLD, hold);
		mmio_write_32(SYSCON_RESET_KEY, 0x00);
		break;
	default:
		WARN("Unsupported affinity level");
	}

	return PSCI_E_SUCCESS;
}

/*
 * Handler called when an affinity instance has just been powered on after
 * being turned off earlier. The level and mpidr determine the affinity
 * instance. The 'state' arg. allows the platform to decide whether the cluster
 * was turned off prior to wakeup and do what's necessary to setup it up
 * correctly.
 */
int32_t axxia_affinst_on_finish(uint64_t mpidr, uint32_t afflvl, uint32_t state)
{
	switch (afflvl) {
	case MPIDR_AFFLVL1:
		ccn_enable_cluster_coherency(mpidr);
		break;
	case MPIDR_AFFLVL0:
		/* Clear entrypoint branch */
		mmio_write_32(0x8031000000, 0);
		/* Enable the gic cpu interface */
		gic_cpuif_setup(GICC_BASE);

		break;
	default:
		WARN("Unsupported affinity level");
	}

	return PSCI_E_SUCCESS;
}

/*
 * Handler called when an affinity instance is about to be turned off. The
 * level and mpidr determine the affinity instance. The 'state' arg. allows the
 * platform to decide whether the cluster is being turned off and take
 * appropriate actions.
 *
 * CAUTION: There is no guarantee that caches will remain turned on across
 * calls to this function as each affinity level is dealt with. So do not write
 * & read global variables across calls. It will be wise to do flush a write to
 * the global to prevent unpredictable results.
 */
static int32_t axxia_affinst_off(uint64_t mpidr, uint32_t afflvl, uint32_t state)
{
	/* Prevent interrupts from spuriously waking up this cpu */
	gic_cpuif_deactivate(GICC_BASE);

	/* Cluster is to be turned off, so disable coherency */
	switch (afflvl) {
	case MPIDR_AFFLVL1:
		ccn_disable_cluster_coherency(mpidr);
		break;
	case MPIDR_AFFLVL0:
		break;
	default:
		WARN("Unsupported affinity level");
		break;
	}

	return PSCI_E_SUCCESS;
}

/*
 * Handler called when an affinity instance is about to be suspended. The level
 * and mpidr determine the affinity instance. The 'state' arg. allows the
 * platform to decide whether the cluster is being turned off and take apt
 * actions. The 'sec_entrypoint' determines the address in BL3-1 from where
 * execution should resume.
 *
 * CAUTION: There is no guarantee that caches will remain turned on across
 * calls to this function as each affinity level is dealt with. So do not write
 * & read global variables across calls. It will be wise to do flush a write to
 * the global to prevent unpredictable results.
 */
static int32_t axxia_affinst_suspend(uint64_t mpidr,
				     uint64_t sec_entrypoint,
				     uint64_t ns_entrypoint,
				     uint32_t afflvl,
				     uint32_t state)
{
	return axxia_affinst_off(mpidr, afflvl, state);
}

static void __dead2 axxia_system_off(void)
{
	/* Best we can do here */
	psci_power_down_wfi();
}

static void __dead2 axxia_system_reset(void)
{
	uint32_t ctrl;

	mmio_write_32(SYSCON_RESET_KEY, 0xab);
	ctrl = mmio_read_32(SYSCON_RESET_CTRL);
	mmio_write_32(SYSCON_RESET_CTRL, ctrl | RSTCTL_RST_SYS);
	/* ...in case it fails */
	psci_power_down_wfi();
}

static const plat_pm_ops_t axxia_ops = {
	.affinst_on		= axxia_affinst_on,
	.affinst_on_finish	= axxia_affinst_on_finish,
	.affinst_off		= axxia_affinst_off,
	.affinst_suspend	= axxia_affinst_suspend,
	.affinst_suspend_finish	= axxia_affinst_on_finish,
	.system_off		= axxia_system_off,
	.system_reset		= axxia_system_reset
};

int32_t platform_setup_pm(const plat_pm_ops_t **plat_ops)
{
	/* Enable cross cluster events */
	mmio_write_32(SYSCON_BASE + 0x14, 0xffff);
	*plat_ops = &axxia_ops;
	return 0;
}

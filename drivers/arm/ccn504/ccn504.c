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
#include <assert.h>
#include <ccn504.h>
#include <mmio.h>

#define MAX_CLUSTERS		4
#define NUM_HNF_NODES		8

static unsigned long ccn_base_addr;
static uint32_t      ccn_cluster_mask[MAX_CLUSTERS];

void ccn_init(unsigned long ccn_base)
{
	ccn_base_addr = ccn_base;
}

static inline unsigned long get_cluster_mask(unsigned long mpidr)
{
	unsigned int cluster_id =
		(mpidr >> MPIDR_AFF1_SHIFT) & MPIDR_AFFLVL_MASK;
	assert(cluster_id < MAX_CLUSTERS);
	return ccn_cluster_mask[cluster_id];
}

static void ccn_set_cluster_coherency(unsigned long mpidr, unsigned long reg)
{
	uint32_t mask = get_cluster_mask(mpidr);
	int i, timeout;

	if (!ccn_base_addr)
		return;

	for (i=0; i < NUM_HNF_NODES; i++) {
		uintptr_t base = ccn_base_addr + CCN_HNF_NODE(i);

		mmio_write_32(base + reg, mask);

		for (timeout=1024; timeout > 0; --timeout) {
			uint32_t state = mmio_read_32(base + SNOOP_CTRL);
			if (reg == SNOOP_DOMAIN_SET && (state & mask) != 0)
				break;
			if (reg == SNOOP_DOMAIN_CLR && (state & mask) == 0)
				break;
		}
	}
}

void ccn_enable_cluster_coherency(unsigned long mpidr)
{
	ccn_set_cluster_coherency(mpidr, SNOOP_DOMAIN_SET);
}

void ccn_disable_cluster_coherency(unsigned long mpidr)
{
	ccn_set_cluster_coherency(mpidr, SNOOP_DOMAIN_CLR);
}

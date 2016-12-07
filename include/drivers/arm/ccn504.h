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

#ifndef __CCN_504_H__
#define __CCN_504_H__

/* Slave interface offsets from PERIPHBASE */
#define CCN_MN_OFFSET		0x000000
#define CCN_DT_OFFSET		0x010000
#define CCN_HNI_OFFSET		0x080000
#define CCN_SBSX_OFFSET		0x100000
#define CCN_HNF_OFFSET		0x200000
#define CCN_XP_OFFSET		0x400000
#define CCN_RNI_OFFSET		0x800000
#define CCN_SBAS_OFFSET		0x810000

#define SNOOP_CTRL		0x200
#define SNOOP_DOMAIN_SET	0x210
#define SNOOP_DOMAIN_CLR	0x220

#define CCN_HNF_NODE(index)	(CCN_HNF_OFFSET + 0x10000*(index))

#define CCN_XP_NODE(index)	(CCN_XP_OFFSET + 0x10000*(index))

#define ERRINT_STATUS		0x8
#define DVM_DOMAIN_CTL		0x200
#define ERR_SIG_VAL_63_0	0x300
#define ERR_SIG_VAL_127_64	0x308
#define ERR_SIG_VAL_191_128	0x310
#define ERR_TYPE_31_0		0x320
#define ERR_TYPE_63_32		0x328
#define ERR_TYPE_95_64		0x330
#define ERR_TYPE_159_128	0x340
#define ERR_SYNDROME_REG0	0x400
#define ERR_SYNDROME_REG1	0x408

#ifndef __ASSEMBLY__

/* Function declarations */

/*
 * The CCI-400 driver must be initialized with the base address of the
 * CCI-400 device in the platform memory map, and the cluster indices for
 * the CCI-400 slave interfaces 3 and 4 respectively. These are the fully
 * coherent ACE slave interfaces of CCI-400.
 * The cluster indices must either be 0 or 1, corresponding to the level 1
 * affinity instance of the mpidr representing the cluster. A negative cluster
 * index indicates that no cluster is present on that slave interface.
 */
void ccn_init(unsigned long cci_base);
void ccn_enable_cluster_coherency(unsigned long mpidr);
void ccn_disable_cluster_coherency(unsigned long mpidr);

#endif /* __ASSEMBLY__ */
#endif /* __CCN_504_H__ */

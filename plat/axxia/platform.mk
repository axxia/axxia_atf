#
# Copyright (c) 2013-2014, ARM Limited and Contributors. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# Neither the name of ARM nor the names of its contributors may be used
# to endorse or promote products derived from this software without specific
# prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

PLAT_INCLUDES		:=	-Iplat/axxia/include/

PLAT_BL_COMMON_SOURCES	:=	drivers/arm/pl011/pl011_console.S	\
				drivers/arm/ccn504/ccn504.c		\
				drivers/io/io_fip.c			\
				drivers/io/io_memmap.c			\
				drivers/io/io_storage.c			\
				lib/aarch64/xlat_tables.c		\
				plat/common/aarch64/plat_common.c	\
				plat/axxia/plat_io_storage.c
ifdef FULL_ATF
BL1_SOURCES		+=	lib/cpus/aarch64/cortex_a57.S		\
				lib/cpus/aarch64/cortex_a53.S		\
				plat/common/aarch64/platform_up_stack.S	\
				plat/axxia/bl1_plat_setup.c		\
				plat/axxia/aarch64/bl1_plat_helpers.S	\
				plat/axxia/aarch64/plat_helpers.S	\
				plat/axxia/aarch64/axxia_common.c
endif

BL2_SOURCES		+=	lib/locks/bakery/bakery_lock.c		\
				plat/common/aarch64/platform_up_stack.S	\
				plat/axxia/bl2_plat_setup.c		\
				plat/axxia/aarch64/plat_helpers.S	\
				plat/axxia/aarch64/axxia_common.c

BL31_SOURCES		+=	drivers/arm/gic/gic_v2.c		\
				drivers/arm/gic/gic_v3.c		\
				plat/axxia/plat_gic.c			\
				lib/cpus/aarch64/cortex_a57.S		\
				lib/cpus/aarch64/cortex_a53.S		\
				plat/common/aarch64/platform_mp_stack.S	\
				plat/axxia/bl31_plat_setup.c		\
				plat/axxia/aarch64/plat_helpers.S	\
				plat/axxia/aarch64/axxia_common.c	\
				plat/axxia/plat_pm.c			\
				plat/axxia/plat_topology.c      \
				plat/axxia/ncr.c

NEED_BL33		:=	no

# Enable workarounds for selected Cortex-A57 erratas.
#ERRATA_A57_806969	:=	0
#ERRATA_A57_813420	:=	0

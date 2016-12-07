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

# Disable the PSCI platform compatibility layer
ENABLE_PLAT_COMPAT := 0

PLAT_INCLUDES := -Iplat/intel/axxia/include

PLAT_BL_COMMON_SOURCES :=                       \
        drivers/arm/pl011/pl011_console.S       \
        drivers/arm/ccn504/ccn504.c             \
        drivers/io/io_fip.c                     \
        drivers/io/io_memmap.c                  \
        drivers/io/io_storage.c                 \
        lib/aarch64/xlat_tables.c               \
        plat/common/aarch64/plat_common.c       \
        plat/intel/axxia/plat_io_storage.c

BL31_SOURCES +=                                            \
        plat/common/aarch64/plat_psci_common.c             \
        plat/intel/axxia/drivers/pwrc/axxia_psci_handler.c \
        plat/intel/axxia/drivers/pwrc/axxia_pwrc.c         \
        drivers/arm/gic/gic_v2.c                           \
        drivers/arm/gic/gic_v3.c                           \
        plat/intel/axxia/plat_gic.c                        \
        lib/cpus/aarch64/cortex_a57.S                      \
        lib/cpus/aarch64/cortex_a53.S                      \
        plat/common/aarch64/platform_mp_stack.S            \
        plat/intel/axxia/bl31_plat_setup.c                 \
        plat/intel/axxia/aarch64/plat_helpers.S            \
        plat/intel/axxia/aarch64/axxia_common.c            \
        plat/intel/axxia/plat_pm.c                         \
        plat/intel/axxia/plat_topology.c                   \
        plat/intel/axxia/ncr.c                             \
        plat/intel/axxia/dsp.c                             \
        plat/intel/axxia/oem_svc.c                         \
        plat/intel/axxia/aarch64/datalogger_dump.S         \
        plat/intel/axxia/ddr_retention.c                   \
        plat/intel/axxia/datalogger.c   

################################################################################
# Axxia only uses BL31
################################################################################

NEED_BL1  := no
NEED_BL2  := no
NEED_BL2U := no
NEED_BL31 := yes
NEED_BL33 := no

# Enable workarounds for selected Cortex-A57 erratas.
#ERRATA_A57_806969      :=      0
#ERRATA_A57_813420      :=      0

# README for the Axxia Changes to the ATF (ARM Trusted Firmware)

## Overview

This repository contains the port of ARM Trusted Firmware for Axxia.

### Branches

The 'main' branch contains two files, this README and COPYING.

To get to a release, simply create a new branch at the release tag.

## Versions and Tags

The tag indicates the version of the Axxia changes and the version of
ARM Trusted Firmware used.  The format is as follows.

atf_&lt;short rev-parse of the commit on master>_axxia_X.Y

X indicates a branch.  It should normally be 1.

Y indicates the version of the Axxia changes based on this ARM Trusted
Firmware version. It increments to indicate changes.

## History

Occasionally, some of the branches get renamed or recreated to clean
up the commit history.  This makes it impossible to update clones.
When this happens, existing clones can no longer be used.  An attempt
is made to communicate this, but because there is no connection
between clones and the repository, it is not possible to be sure
everyone gets notified.  If 'git pull' fails, try a fresh clone.

The original branch is saved in case commit history etc. is needed.
Here are the changes and the names of the saved branches.

The original port for simulation is on the x9 branch.

For releases through axxia_atf_1.1.1.18, use axxia-dev_axxia_atf_1.1.1.18.

After that, use axxia-dev.

## Building ARM Trusted Firmware for Axxia

### Overview

Note that only the bl31 component is used by Axxia.  No other targets
are supported.

The cross compilation tools should be in the PATH, CROSS_COMPILE
should contain the proper prefix, and SYSROOT should point to the
target root file system if required by the tools.  ARCH should be set
to arm64.

For example, after installing the Yocto tools, set up the environment
as follows.

```text
$ export PATH=<tool install path>/sysroots/x86_64-pokysdk-linux/usr/bin/aarch64-poky-linux:$PATH
$ export CROSS_COMPILE=aarch64-poky-linux-
$ export SYSROOT=<tool install path>/sysroots/aarch64-poky-linux
$ export ARCH=arm64
```

### Steps

1 Clone the Axxia ARM Trusted Firmware repository.

```text
$ git clone https://github.com/axxia/axxia_atf.git
```

2 Checkout the "axxia-dev" branch.

```text
$ cd axxia_atf
$ git checkout --track -b axxia-dev origin/axxia-dev
```

3 Build bl31.

Add 'PLAT=axxia USE_COHERENT_MEM=0 CRASH_REPORTING=1', and optionally,
'DEBUG=1' to the make command line.  If 'DEBUG=1' is added, there will
be more output on the console, and more assertions in the code.

A number of CVE work arounds are available.  Details are available on
the ARM website.  By defaults all CVEs are enabled.  Some do not apply
to all cores, and some that apply may not be necessary.

None of the CVEs are applicable to the A53 (6700/XLF).  Here are the
available CVE work arounds, to disable them add
WORKAROUND_CVE_<date>_<number>=0; to enable change =0 to =1.  All CVEs
are disabled by default.

WORKAROUND_CVE_2017_5715 (available from atf_84091c4_axxia_1.35 on)
    -- Internal Commits --
    5c30cf5 "Work around for CVE-2017-5715 on Axxia"
    be6a829 "Change the Default State of WORKAROUND_CVE_2017_5715"

WORKAROUND_CVE_2018_3639 (avaialble from atf_84091c4_axxia_1.38 on)
    -- Internal Commits --
    f5c3e5b "Work around for CVE-2018-3639 on Axxia"
    69fdca2 "Change the Default State of WORKAROUND_CVE_2018_3639"

WORKAROUND_CVE_2017_7564 (available from atf_84091c4_axxia_1.40 on)
    -- Internal Commit --
    1957fc1 "Advisory TFV 2 to CVE-2017-7564"

WORKAROUND_CVE_2017_7563 (available from atf_84091c4_axxia_1.40 on)
    -- Internal Commit --
    96e86c6 "Advisory TFV 3 to CVE-2017-7563"

WORKAROUND_CVE_2017_15031 (available from atf_84091c4_axxia_1.40 on)
CTX_INCLUDE_AARCH32_REGS (available from atf_84091c4_axxia_1.40 on)
    -- Internal Commit --
    4654b4d "Advisory TFV 5 to CVE-2017-15031"

Note that CTX_INCLUDE_AARCH32_REGS should be set to 1 if using
aarch32. As aarch32 is not suppported by the Axxia ATF port, there
should be no reason to set this.

```text
$ make PLAT=axxia USE_COHERENT_MEM=0 CRASH_REPORTING=1 bl31
```

or

```text
$ make DEBUG=1 PLAT=axxia USE_COHERENT_MEM=0 CRASH_REPORTING=1 bl31
```

Prior to version atf_84091c4_axxia_1.7, build as follows.  Note that
earlier versions of the ATF will only work with versions of U-Boot up
to 1.19.

In addition to PLAT=axxia, include the following.

Either AXM5600=1 or AXC6700=1 must be defined.

Either EMULATION=1 or SIMULATION=1 may be defined.

Optionally, DEBUG=1 can be defined to add console output as above.

```text
$ make PLAT=axxia <other defines as described above> bl31
```

4 Create an ELF object of the bl31 binary for inclusion in the SPL.

```text
$ ${CROSS_COMPILE}objcopy -I binary -O elf64-littleaarch64 -B aarch64 \
    --rename-section .data=.monitor build/axxia/release/bl31.bin \
    build/axxia/release/bl31.o
```

Or, if using DEBUG=1,

```text
$ ${CROSS_COMPILE}objcopy -I binary -O elf64-littleaarch64 -B aarch64 \
    --rename-section .data=.monitor build/axxia/debug/bl31.bin \
    build/axxia/debug/bl31.o
```

Copy bl31.o from above to the spl directory (you may need to create
the spl directory!) in the U-Boot source tree after configuring
U-Boot.

## ARM Trusted Firmware: Changes for Axxia

### atf_84091c4_axxia_1.43

* Disable the Secure Cycle Counter.

### atf_84091c4_axxia_1.42

* Update the CVE based on ARM recommendation.

### atf_84091c4_axxia_1.41

* Support the GNU binutils 2.31 and GCC 8.2.

### atf_84091c4_axxia_1.40

* Updates for the latest ARM CVEs.  These include CVE-2017-7564,
  CVE-2017-7563, CVE-2017-15031, and CVE-2018-19440.  Each is optional
  at compile time; see Readme.md for details.
* Implement the SMC v1.1 calling convention.

### atf_84091c4_axxia_1.39

* Clean up klocwork issues, Critical and Error only, and only in code
  added to support Axxia.

### atf_84091c4_axxia_1.38

* Allow non-secure access to the CCN registers.  This is required, for
  example, to implement some work arounds.
* Add work around for CVE-2018-3639 (Spectre Variant 4).  This should
  only be enabled when building for Axxia 5600.  Include
  WORKAROUND_CVE_2018_3639=1 on the make command line to enable the
  work around and WORKAROUND_CVE_2018_3639=0 to disable it.

### atf_84091c4_axxia_1.37

* Enable PMU register access from EL0.

### atf_84091c4_axxia_1.36

* Support unfused 6700 parts.

### atf_84091c4_axxia_1.35

* Add a patch for CVE-2017-5715 (Spectre/Meltdown) on 5600.

### atf_84091c4_axxia_1.34

* Enable cache protection for 5600.
* Support the new B0 version of 6700.  Note that from this version on,
  unfused A0 parts are not supported.

### atf_84091c4_axxia_1.33

* Enable L1/L2 cache ecc correction on 6700.
* Add EDAC suppport for 6700.

### atf_84091c4_axxia_1.32

* Enable L1/L2 cache ecc protection.  Not tested on 6700.
* Add new services for ccn504 for EDAC driver L3 accesses.

### atf_84091c4_axxia_1.31

* Add support for L2 power control.  When enabled, a cluster's L2 will
  be powered down when all cores in the cluster are disabled.
* Change the delay mentioned in the previous version to a cache flush.

### atf_84091c4_axxia_1.30

* Updated support for AXC6700 power management.  Adds a delay when
  powering down

### atf_84091c4_axxia_1.29

* Initial support for AXC6700 power management.

### atf_84091c4_axxia_1.28

* Update the commit log -- no code changes.

### atf_84091c4_axxia_1.27

* If the last DDR retention reset was caused by timer 7, set bit 1 in
  the pscratch register.
* Fix compiler warnings when building with GCC 6.
* Don't reset the ELM trace buffer during DDR retention resets.
* Correct the peripheral clock speed calculation.

### atf_84091c4_axxia_1.26

* Before disabling DSP clusters, flush the L2 cache.

### atf_84091c4_axxia_1.25

* Flush the DSP cluster's L2 cache before disabling the cluster.

### atf_84091c4_axxia_1.24

* Increase the length of the CDC L2CC power down retry loop.  The
  original length was too short in simulation.

### atf_84091c4_axxia_1.23

* Further power management updates.  L2 power control works but is not
  enabled by default pending investigation of the L2 reset logic.
* Flush all caches before initiating a DDR retention reset.
* Add OEM functions to get and set the ACTLR_EL3 and ACTLR_EL2.  This
  is required for performance testing.

### atf_84091c4_axxia_1.22

* Updates to 5600 power management.

### atf_84091c4_axxia_1.21

* Do not power down the L2 cache during power management.

### atf_84091c4_axxia_1.20

* Update power control.  CPU off and on work on 5600.
* Fix error print when controlling DSP clusters on 6700.

### atf_84091c4_axxia_1.19

* Add support for OEM functions.
* Support control of DSP clusters on 6700 using OEM functions.

### atf_84091c4_axxia_1.18

* Use the correct address for CDC3.

### atf_84091c4_axxia_1.17

* Add a function to control the DSPs.  For now, this just enables DSP
  cluster 0.

### atf_84091c4_axxia_1.16

* Updates to the "Run in Syscache" feature.  Works on 5600 but not 6700.

### atf_84091c4_axxia_1.15

* Support the correct coherency bits on 6700.

### atf_84091c4_axxia_1.14

* Remove debug print statements.

### atf_84091c4_axxia_1.13

* Use the ARM physical timer to trigger DDR retention reset instead of
  the watchdog.

### atf_84091c4_axxia_1.12

* DDR retention reset updates.

### atf_84091c4_axxia_1.11

* Use "chip" instead of "system" reset.

### atf_84091c4_axxia_1.10

* Do not use the SFONLY or HAM L3 states.  To flush the L3 cache, use
  ON->OFF->ON instead of ON->SFONLY->ON.

### atf_84091c4_axxia_1.9

* Add the peripheral clock frequency and baud rate to the parameters
  passed from the SPL.

### atf_84091c4_axxia_1.8

* Support the 4th cluster on 5600 hardware.

### atf_84091c4_axxia_1.7

* Simplify the build options.  The SPL will pass options indicating
  the target (5600 or 6700), the platform (simulation, emulation, or
  hardware), and options.
* Add support to boot without initializing system memory.

### atf_84091c4_axxia_1.6

* Fix emulation boot problems introduced in 1.5.

### atf_84091c4_axxia_1.5

* Switch from PSCI 'compat' to 'native' mode.
* Add code to power down cores, caches, and clusters.

### atf_84091c4_axxia_1.4

* Add the DSP clusters to the coherency domain.

### atf_84091c4_axxia_1.3

* Set the counter frequency for secondary cores.

### atf_84091c4_axxia_1.2

* Instead of setting the counter frequency, use the frequency set by
  the SPL.

### atf_84091c4_axxia_1.1

* Updated to the latest ARM Trusted Firmware (commit 84091c4).

### 1.1.1.18

* DDR Retention support.

### 1.1.1.17

* Support for config ring accesses.

### 1.1.1.16

* Get the last group when making interrupts accessible to Linux
  (non-secure).

### 1.1.1.15

* Increase the number of clusters -- required for XLF.

### 1.1.1.14

* Clear the OS Lock for Secondary Cores.

### 1.1.1.13

* Add a compile time option to leave the L3 cache in SFONLY.
* Only issue invalidate cache operations when memory is cacheable.
* Add coherency bits for the new (3.7) XLF emulation images.

### 1.1.1.12

* Only issue clean and invalidate to cachable memory.

### 1.1.1.11

* Changes for X9 Multi-Cluster in Simulation.

### 1.1.1.10

* GIC address updates for XLF.
* Use the correct cluster mask bits for XLF.

### 1.1.1.9

* Add a memory mapping for CCN on XLF.
* Don't enable IGRPEN1 during GIC initialization.  Enabling IGRPEN1
  causes XLF to fail, and isn't necessary on X9.

### 1.1.1.8

* Use the same GIC addresses for simulation and emulation/hardware.

### 1.1.1.7

* Implement PSCI reset.
* Add the other clusters to the coherency domain as needed.

### 1.1.1.6

* Set the cluster coherency bits correctly for emulation.
* Flush the L3 cache in simulation.

### 1.1.1.5

* Enable the L3 cache before starting U-Boot.

### 1.1.1.4

* Handle simulation, emulation, and hardware differences at run time.
* Don't use a scratch register for the jump address, just jump to the
  base of LSM.
* Updated MMU setup for emulation.

### 1.1.1.3

* Add cluster 0 to the coherency domain.

### 1.1.1.2

* Change the 

### 1.1.1.1

* Initial version.

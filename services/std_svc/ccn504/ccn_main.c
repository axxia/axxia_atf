#include <arch.h>
#include <arch_helpers.h>
#include <assert.h>
#include <debug.h>
#include <platform.h>
#include <runtime_svc.h>
#include <std_svc.h>
#include <string.h>
#include <mmio.h>

#include "ccn_private.h"

extern unsigned long dickens_base;

#ifdef CCN_DEBUG
int ccn_make_unsecure(uint64_t x1)
{
	uint64_t ret=1;
	mmio_write_64(dickens_base, 0x1);
	return  ret;
}

int ccn_make_secure(uint64_t x1)
{
	uint64_t ret=1;
	mmio_write_64(dickens_base, 0x0);
	return  ret;
}
#endif

uint64_t write_ccn_nm_errint_status(uint64_t x1)
{
	switch(0xff & x1) {
	case CCN_MN_ERRINT_STATUS__INTREQ__DESSERT:
	case CCN_MN_ERRINT_STATUS__ALL_ERRORS__ENABLE:
	case CCN_MN_ERRINT_STATUS__ALL_ERRORS__DISABLE:
	case CCN_MN_ERRINT_STATUS__CORRECTED_ERRORS_ENABLE:
	case CCN_MN_ERRINT_STATUS__CORRECTED_ERRORS_DISABLE:
	case CCN_MN_ERRINT_STATUS__PMU_EVENTS__ENABLE:
	case CCN_MN_ERRINT_STATUS__PMU_EVENTS__DISABLE:
		mmio_write_64(dickens_base + 8, x1);
		return 1;
	default:

		WARN("Unknown errint_status argument: %lx \n", x1);
		return SMC_UNK;
	}
}

uint64_t read_ccn_nm_errint_status(void)
{
	return mmio_read_64(dickens_base + 8);
}

uint64_t ccn_smc_handler(uint32_t smc_fid,
			  uint64_t x1,
			  uint64_t x2,
			  uint64_t x3,
			  uint64_t x4,
			  void *cookie,
			  void *handle,
			  uint64_t flags)
{

#ifdef CCN_DEBUG
#else
	if (is_caller_secure(flags))
		SMC_RET1(handle, SMC_UNK);
#endif		

	/* Check the fid against the capabilities */
	if (!(ccn_caps & define_ccn_cap(smc_fid)))
		SMC_RET1(handle, SMC_UNK);

	if (((smc_fid >> FUNCID_CC_SHIFT) & FUNCID_CC_MASK) == SMC_32) {

		WARN("Unimplemented CCN Call: 0x%x \n", smc_fid);
		SMC_RET1(handle, SMC_UNK);
	}
	else {

		switch (smc_fid) {
#ifdef CCN_DEBUG
		case CCN_MAKE_SECURE_AARCH64:
			SMC_RET1(handle, ccn_make_secure(x1));

		case CCN_MAKE_UNSECURE_AARCH64:
			SMC_RET1(handle, ccn_make_unsecure(x1));
#endif
		case CCN_MN_READ_ERRINT_AARCH64:
			SMC_RET1(handle, mmio_read_64(dickens_base + 8));

		case CCN_MN_WRITE_ERRINT_AARCH64:
			SMC_RET1(handle, write_ccn_nm_errint_status(x1));

		default:
			break;
		}
	}

	WARN("Unimplemented CCN Call: 0x%x \n", smc_fid);
	SMC_RET1(handle, SMC_UNK);
}

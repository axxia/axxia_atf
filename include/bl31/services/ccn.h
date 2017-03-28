#ifndef __CCN_H__
#define __CCN_H__

#define CCN_MAKE_UNSECURE_AARCH32 		0x84000024
#define CCN_MAKE_UNSECURE_AARCH64		0xc4000024
#define CCN_MAKE_SECURE_AARCH32 		0x84000025
#define CCN_MAKE_SECURE_AARCH64			0xc4000025

#define CCN_MN_READ_ERRINT_AARCH32 		0x84000026
#define CCN_MN_READ_ERRINT_AARCH64		0xc4000026

#define CCN_MN_WRITE_ERRINT_AARCH32 		0x84000027
#define CCN_MN_WRITE_ERRINT_AARCH64		0xc4000027

#define CCN_MN_ERRINT_STATUS__INTREQ__DESSERT           0x11
#define CCN_MN_ERRINT_STATUS__ALL_ERRORS__ENABLE        0x02
#define CCN_MN_ERRINT_STATUS__ALL_ERRORS__DISABLED      0x20
#define CCN_MN_ERRINT_STATUS__ALL_ERRORS__DISABLE       0x22
#define CCN_MN_ERRINT_STATUS__CORRECTED_ERRORS_ENABLE   0x04
#define CCN_MN_ERRINT_STATUS__CORRECTED_ERRORS_DISABLED 0x40
#define CCN_MN_ERRINT_STATUS__CORRECTED_ERRORS_DISABLE  0x44
#define CCN_MN_ERRINT_STATUS__PMU_EVENTS__ENABLE        0x08
#define CCN_MN_ERRINT_STATUS__PMU_EVENTS__DISABLED      0x80
#define CCN_MN_ERRINT_STATUS__PMU_EVENTS__DISABLE       0x88


#define define_ccn_cap(x)		(1 << ( (x - 0x24) & 0x2f))

#ifdef CCN_DEBUG

int ccn_make_secure(uint64_t x1);
int ccn_make_unsecure(uint64_t x1);

#define CCN_GENERIC_CAP	\
			(define_ccn_cap(CCN_MAKE_SECURE_AARCH64)   |		\
			 define_ccn_cap(CCN_MAKE_UNSECURE_AARCH64) |		\
			 define_ccn_cap(CCN_MN_READ_ERRINT_AARCH64)|		\
			 define_ccn_cap(CCN_MN_WRITE_ERRINT_AARCH64))

#else
#define CCN_GENERIC_CAP	\
			(define_ccn_cap(CCN_MN_READ_ERRINT_AARCH64)|		\
			 define_ccn_cap(CCN_MN_WRITE_ERRINT_AARCH64))
#endif

extern unsigned int ccn_caps;

void ccn504_setup();

uint64_t ccn_smc_handler(uint32_t smc_fid,
			  uint64_t x1,
			  uint64_t x2,
			  uint64_t x3,
			  uint64_t x4,
			  void *cookie,
			  void *handle,
			  uint64_t flags);
#endif /*__CCN_H__*/

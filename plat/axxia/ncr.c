/*
 *  Copyright (C) 2009 LSI Corporation
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#if 0
#include <common.h>
#include <asm/io.h>
#include "../axxia-arm/ncp_nca_reg_defines.h"
#endif

#include <stddef.h>
#include <string.h>
#include <debug.h>

extern void
udelay(unsigned long us);


#define     NCP_NCA_CFG_RING_ERROR_STAT_R0                      (0x000000e4)
#define     NCP_NCA_CFG_RING_ERROR_STAT_R1                      (0x000000e8)
#define     NCP_NCA_CFG_RING_NODE_STATUS                        (0x000000ec)
#define     NCP_NCA_CFG_PIO_CDR0                                (0x000000f0)
#define     NCP_NCA_CFG_PIO_CDR1                                (0x000000f4)
#define     NCP_NCA_CFG_PIO_CDR2                                (0x000000f8)
#define     NCP_NCA_CDAR_MEMORY_BASE                            (0x00001000)

#define NCP_NODE_ID(regionId)    (((regionId) >> 16) & 0xffff)
#define NCP_TARGET_ID(regionId)  ((regionId) & 0xffff)
#define NCP_REGION_ID(node, tgt) (((node & 0xffff) << 16 ) | (tgt & 0xffff))

#define WFC_TIMEOUT (400000)

#define LOCK_DOMAIN 0

extern __uint64_t nca_base;
extern int need_nca_swap;

static int ncr_sysmem_mode_disabled = 1;
static int ncr_tracer_disabled = 1;
void ncr_tracer_enable( void ) { ncr_tracer_disabled = 0; }
void ncr_tracer_disable( void ) { ncr_tracer_disabled = 1; }
int ncr_tracer_is_enabled( void ) { return 0 == ncr_tracer_disabled ? 1 : 0; }
void ncr_sysmem_init_mode_enable(void) { ncr_sysmem_mode_disabled = 0; }
void ncr_sysmem_init_mode_disable(void) { ncr_sysmem_mode_disabled = 1; }

static __inline__ __uint32_t ncr_register_read(__uint64_t );
static __inline__ void ncr_register_write(__uint32_t, __uint64_t );



void
acp_failure(const char *file, const char *function, const int line)
{
    tf_printf("\n%s:%s:%d - System Failure\n", file, function, line);
    while (1) ;
    return;
}




static int
__ncr_fail(const char *file, const char *function, const int line)
{
	if (1 == ncr_sysmem_mode_disabled)
		return -1;
	
	tf_printf("Config Ring Access Failed: 0x%08lx 0x%08lx\n",
	       ncr_register_read((nca_base + NCP_NCA_CFG_RING_ERROR_STAT_R0)),
	       ncr_register_read((nca_base + NCP_NCA_CFG_RING_ERROR_STAT_R1)));
	acp_failure(file, function, line);

	return -1;
}

#define ncr_fail(file, func, line) __ncr_fail(file, NULL, line);

#ifdef NCR_TRACER
static int short_read_count = 100;	/* Make sure this isn't in bss. */

void
ncr_trace_read8(__uint32_t region, __uint32_t offset)
{
	if (100 == short_read_count)
		short_read_count = 0;

	if (0 == short_read_count) {
		++short_read_count;
		tf_printf("ncpRead   -w8 0.%u.%u.0x00%08x",
		       NCP_NODE_ID(region), NCP_TARGET_ID(region), offset);
	} else {
		++short_read_count;

		if (64 == short_read_count) {
			tf_printf(" 64\n");
			short_read_count = 0;
		}
	}

	return;
}

void
ncr_trace_read16(__uint32_t region, __uint32_t offset)
{
	tf_printf("ncpRead    0.%u.%u.0x00%08x 1\n",
	       NCP_NODE_ID(region), NCP_TARGET_ID(region), offset);

	return;
}

void
ncr_trace_read32(__uint32_t region, __uint32_t offset)
{
	tf_printf("ncpRead    0.%u.%u.0x00%08x 1\n",
	       NCP_NODE_ID(region), NCP_TARGET_ID(region), offset);

	return;
}

static int short_write_count = 100;	/* Make sure this isn't in bss. */

void
ncr_trace_write8(__uint32_t region, __uint32_t offset, __uint32_t value)
{
	if (100 == short_write_count)
		short_write_count = 0;

	if (0 == short_write_count) {
		++short_write_count;
		tf_printf("ncpWrite  -w8 0.%u.%u.0x00%08x 0x%02x",
		       NCP_NODE_ID(region), NCP_TARGET_ID(region),
		       offset, value);
	} else {
		++ short_write_count;
		tf_printf(" 0x%02x", value);

		if (4 == short_write_count) {
			tf_printf("\n");
			short_write_count = 0;
		}
	}

	return;
}

void
ncr_trace_write16(__uint32_t region,
		  __uint32_t offset, __uint32_t value)
{
	tf_printf("ncpWrite   0.%u.%u.0x00%08x 0x%04x\n",
	       NCP_NODE_ID(region), NCP_TARGET_ID(region), offset, value);

	return;
}

void
ncr_trace_write32(__uint32_t region,
		  __uint32_t offset, __uint32_t value)
{
	tf_printf("ncpWrite   0.%u.%u.0x00%08x 0x%08x\n",
	       NCP_NODE_ID(region), NCP_TARGET_ID(region), offset, value);

	return;
}

void
ncr_trace_modify(__uint32_t region,
		 __uint32_t offset, __uint32_t mask, __uint32_t value)
{
	tf_printf("ncpModify  0.%u.%u.0x00%08x 0x%08x 0x%08x\n",
	       NCP_NODE_ID(region), NCP_TARGET_ID(region), offset, mask, value);

	return;
}

void
ncr_trace_poll(__uint32_t region,
	       __uint32_t loops, __uint32_t delay,
	       __uint32_t offset, __uint32_t mask, __uint32_t value)
{
	tf_printf("ncpPoll -l %u -t %u  0.%u.%u.0x00%08x " \
	       "0x%08x 0x%08x\n",
	       loops, delay,
	       NCP_NODE_ID(region), NCP_TARGET_ID(region), offset, mask, value);

	return;
}

#define NCR_TRACE_READ8(region, offset) do {			\
		if (ncr_tracer_is_enabled()) {			\
			ncr_trace_read8(region, offset); }	\
	} while (0);
#define NCR_TRACE_READ16(region, offset) do {			\
		if (ncr_tracer_is_enabled()) {			\
			ncr_trace_read16(region, offset); }	\
	} while (0);
#define NCR_TRACE_READ32(region, offset) do {			\
		if (ncr_tracer_is_enabled()) {			\
			ncr_trace_read32(region, offset); }	\
	} while (0);
#define NCR_TRACE_WRITE8(region, offset, value) do {			\
		if (ncr_tracer_is_enabled()) {				\
			ncr_trace_write8(region, offset, value); }	\
	} while (0);
#define NCR_TRACE_WRITE16(region, offset, value) do {			\
		if (ncr_tracer_is_enabled()) {				\
			ncr_trace_write16(region, offset, value); }	\
	} while (0);
#define NCR_TRACE_WRITE32(region, offset, value) do {			\
		if (ncr_tracer_is_enabled()) {				\
			ncr_trace_write32(region, offset, value); }	\
	} while (0);
#define NCR_TRACE_MODIFY(region, offset, mask, value) do {		\
		if (ncr_tracer_is_enabled()) {				\
			ncr_trace_modify(region, offset, mask, value); } \
	} while (0);
#define NCR_TRACE_POLL(region, loops, delay, offset, mask, value) do {	\
		if (ncr_tracer_is_enabled()) {				\
			ncr_trace_poll(region, loops, delay, offset, mask, value); } \
	} while (0);
#else
#define NCR_TRACE_READ8(region, offset) {}
#define NCR_TRACE_READ16(region, offset) {}
#define NCR_TRACE_READ32(region, offset) {}
#define NCR_TRACE_WRITE8(region, offset, value) {}
#define NCR_TRACE_WRITE16(region, offset, value) {}
#define NCR_TRACE_WRITE32(region, offset, value) {}
#define NCR_TRACE_MODIFY(region, offset, mask, value) {}
#define NCR_TRACE_POLL(region, loops, delay, offset, mask, value) {}
#endif

/* Note that NCA in this case means nca_axi (0x101.0.0) */

/*
 * Note that these are the little-endian representation of
 * the NCA CDR registers.  The bit-field definitions will 
 * appear swapped with respect to the RDL. 
 */

typedef union {
    __uint32_t raw;
    struct {
        __uint32_t dbs                 : 16;
        __uint32_t cmd_type            : 4;
        __uint32_t cfg_cmpl_int_enable : 1;
        __uint32_t byte_swap_enable    : 1;
        __uint32_t status              : 2;
        __uint32_t local_bit           : 1;
        __uint32_t sysmem_access_type  : 4;
        __uint32_t                     : 2;
        __uint32_t start_done          : 1;
    } __attribute__ ( ( packed ) ) bits;
} __attribute__ ( ( packed ) ) command_data_register_0_t;

typedef union {
    __uint32_t raw;
    struct {
        __uint32_t target_address : 32;
    } __attribute__ ( ( packed ) ) bits;
} __attribute__ ( ( packed ) ) command_data_register_1_t;

typedef union {
    __uint32_t raw;
    struct {
        __uint32_t target_id_address_upper : 8;
        __uint32_t target_node_id          : 8;
        __uint32_t                         : 16;
    } __attribute__ ( ( packed ) ) bits;
} __attribute__ ( ( packed ) ) command_data_register_2_t;



/*
  ------------------------------------------------------------------------------
  ncr_register_read
*/

static __inline__ __uint32_t
ncr_register_read(__uint64_t address)
{
    __uint32_t *p = (__uint32_t *) address;
    volatile __uint32_t val = *p;
    if (need_nca_swap) 
    	return __builtin_bswap32(val);
    else
        return val;
}

/*
  ----------------------------------------------------------------------
  ncr_register_write
*/

static __inline__ void
ncr_register_write( __uint32_t value, __uint64_t address) 
{
    __uint32_t *p = (__uint32_t *) address;
    volatile __uint32_t val;
    if (need_nca_swap) {
        val = __builtin_bswap32(value);
    } else {
        val = value;
    }
	*p = val;
}

/*
  ------------------------------------------------------------------------------
  ncr_lock
*/

static int
ncr_lock(int domain)
{
	__uint32_t offset;
	__uint32_t value;
	int loops = 400000;

	offset=(0xff80 + (domain * 4));

	do {
		value = ncr_register_read(nca_base + offset);
	} while ((0 != value) && (0 < --loops));

	if (0 == loops)
		return -1;
	return 0;
}

/*
  ------------------------------------------------------------------------------
  ncr_unlock
*/

static void
ncr_unlock(int domain)
{
	__uint32_t offset;

	offset=(0xff80 + (domain * 4));
	ncr_register_write(0, (nca_base + offset));
	return;
}

/*
  ======================================================================
  ======================================================================
  Public Interface
  ======================================================================
  ======================================================================
*/

/*
  ----------------------------------------------------------------------
  ncr_read
*/

int
ncr_read(__uint32_t region,
	 __uint32_t address,
	 int count, __uint32_t *buffer)
{
	command_data_register_0_t cdr0;	/* 0x101.0.0xf0 */
	command_data_register_1_t cdr1;	/* 0x101.0.0xf4 */
	command_data_register_2_t cdr2;	/* 0x101.0.0xf8 */
    __uint32_t reg;
	int wfc_timeout = WFC_TIMEOUT;

	switch (NCP_NODE_ID(region)) {
	default:
		if(NCP_NODE_ID(region) >= 0x100) {
			tf_printf("Unhandled read to 0x%lx.0x%lx.0x%lx\n",
			       (unsigned long)NCP_NODE_ID(region),
			       (unsigned long)NCP_TARGET_ID(region),
			       (unsigned long)address);
			return -1;
		}
		/* Actual config ring acces, continue. */
		break;
	}

	if (0 != ncr_lock(LOCK_DOMAIN))
		return -1;

	/*
	  Set up the read command.
	*/

	cdr2.raw = 0;
	cdr2.bits.target_node_id = NCP_NODE_ID( region );
	cdr2.bits.target_id_address_upper = NCP_TARGET_ID( region );

	ncr_register_write( cdr2.raw, ( nca_base + NCP_NCA_CFG_PIO_CDR2 ) );

	cdr1.raw = 0;
	cdr1.bits.target_address = ( address >> 2 );

	ncr_register_write( cdr1.raw, ( nca_base + NCP_NCA_CFG_PIO_CDR1 ) );

	cdr0.raw = 0;
	cdr0.bits.start_done = 1;

	if( 0xff == cdr2.bits.target_id_address_upper ) {
		cdr0.bits.local_bit = 1;
	}

	cdr0.bits.cmd_type = 0x4;

	/* TODO: Verify count... */
	cdr0.bits.dbs = ( (count * 4)  - 1 );
	ncr_register_write( cdr0.raw, ( nca_base + NCP_NCA_CFG_PIO_CDR0 ) );

	/*
	  Wait for completion.
	*/

	do {
		--wfc_timeout;
	     reg = ncr_register_read( ( nca_base + NCP_NCA_CFG_PIO_CDR0 ) ) ;
         /* tf_printf("cdr0=0x%08x\n", reg); */
         if ( (reg & 0x80000000) == 0) break;
	} while( 0 < wfc_timeout);

	if (0 == wfc_timeout) {
		tf_printf("ncr_read(): NCA Lockup!\n");
		ncr_unlock(LOCK_DOMAIN);
		return -1;
	}

	/*
	  Check status.
	*/

	if( 0x3 != ( ( ncr_register_read( ( nca_base + NCP_NCA_CFG_PIO_CDR0 ) ) &
		       0x00c00000 ) >> 22 ) ) {
		ncr_unlock(LOCK_DOMAIN);
		return -1;
	}

	/*
	  Read the data into the buffer.
	*/

	if (NULL != buffer) {
		__uint64_t address;

		address = (nca_base + NCP_NCA_CDAR_MEMORY_BASE);

		while (count) {
			*((__uint32_t *)buffer) =
				ncr_register_read(address);
			address += 4;
			count--;
			buffer++;
		}
	}

	ncr_unlock(LOCK_DOMAIN);
	return 0;
}

/*
  ----------------------------------------------------------------------
  ncr_read32
*/

int
ncr_read32(__uint32_t region, __uint32_t offset, __uint32_t *value)
{
	int rc = 0;

	NCR_TRACE_READ32(region, offset);
	rc = ncr_read(region, offset, 4, value);

	if (0 != rc)
		return ncr_fail(__FILE__, __FUNCTION__, __LINE__);

	return 0;
}


/*
  ------------------------------------------------------------------------------
  ncr_poll
*/

int
ncr_poll( __uint32_t region, __uint32_t offset,
	  __uint32_t mask, __uint32_t desired_value,
	  __uint32_t delay_time, __uint32_t delay_loops )
{
	int i;
	int rc = 0;

	NCR_TRACE_POLL(region, delay_loops, delay_time,
		       offset, mask, desired_value);

	for( i = 0; i < delay_loops; ++ i ) {
		__uint32_t value;

		rc |= ncr_read(region, offset, 4, &value);

		if( ( value & mask ) == desired_value ) {
			break;
		}

		udelay( delay_time );
	}

	if( delay_loops == i ) {
		return ncr_fail(__FILE__, __FUNCTION__, __LINE__);
	}

	if (0 != rc)
		return ncr_fail(__FILE__, __FUNCTION__, __LINE__);

	return 0;
}

/*
  ----------------------------------------------------------------------
  ncr_write
*/

int
ncr_write(__uint32_t region,
	  __uint32_t address,
	  int count, __uint32_t *buffer)
{
	command_data_register_0_t cdr0;
	command_data_register_1_t cdr1;
	command_data_register_2_t cdr2;
	int dbs = ((count * 4) - 1);
	int wfc_timeout = WFC_TIMEOUT;
    __uint32_t reg;

	switch (NCP_NODE_ID(region)) {
	default:
		if(NCP_NODE_ID(region) >= 0x100) {
			tf_printf("Unhandled write to 0x%lx.0x%lx.0x%lx\n",
			       (unsigned long)NCP_NODE_ID(region),
			       (unsigned long)NCP_TARGET_ID(region),
			       (unsigned long)address);
			return -1;
		}
		/* Actual config ring acces, continue. */
		break;
	}

	if (0 != ncr_lock(LOCK_DOMAIN))
		return -1;

	/*
	  Set up the write.
	*/

	cdr2.raw = 0;
	cdr2.bits.target_node_id = NCP_NODE_ID( region );
	cdr2.bits.target_id_address_upper = NCP_TARGET_ID( region );

	ncr_register_write( cdr2.raw, ( nca_base + NCP_NCA_CFG_PIO_CDR2 ) );

	cdr1.raw = 0;
	cdr1.bits.target_address = ( address >> 2 );

	ncr_register_write( cdr1.raw, ( nca_base + NCP_NCA_CFG_PIO_CDR1 ) );

	/*
	  Copy data from the buffer.
	*/

	if (NULL != buffer) {
		__uint64_t offset = (nca_base + NCP_NCA_CDAR_MEMORY_BASE);

		while (count) {
			ncr_register_write(*((__uint32_t *)buffer), offset);
			offset += 4;
			buffer++;
			count--;
		}
	}

	/*
	  Write
	*/

	cdr0.raw = 0;
	cdr0.bits.start_done = 1;

	if( 0xff == cdr2.bits.target_id_address_upper ) {
		cdr0.bits.local_bit = 1;
	}

	cdr0.bits.cmd_type = 0x5;

	/* TODO: Verify number... */
	cdr0.bits.dbs = dbs;
	ncr_register_write( cdr0.raw, ( nca_base + NCP_NCA_CFG_PIO_CDR0 ) );

	/*
	  Wait for completion.
	*/

	do {
		--wfc_timeout;
	     reg = ncr_register_read( ( nca_base + NCP_NCA_CFG_PIO_CDR0 ) ) ;
         /* tf_printf("cdr0=0x%08x\n", reg); */
         if ( (reg & 0x80000000) == 0) break;
	} while( 0 < wfc_timeout);

	if (0 == wfc_timeout) {
		tf_printf("ncr_write(): NCA Lockup!\n");
		ncr_unlock(LOCK_DOMAIN);
		return -1;
	}

	/*
	  Check status.
	*/

	if(0x3 !=
	   ((ncr_register_read((nca_base + NCP_NCA_CFG_PIO_CDR0)) & 0x00c00000) >> 22)) {
		tf_printf("ncr_write( ) failed: 0x%lx, status1=0x%lx, status2=0x%lx\n",
		       ((ncr_register_read((nca_base +
								       NCP_NCA_CFG_PIO_CDR0)) &
					0x00c00000) >> 22),
		       ncr_register_read((nca_base +
								     NCP_NCA_CFG_RING_ERROR_STAT_R0)),
		       ncr_register_read(nca_base +
								     NCP_NCA_CFG_RING_ERROR_STAT_R1));
		ncr_unlock(LOCK_DOMAIN);

		return -1;
	}

	ncr_unlock(LOCK_DOMAIN);
	return 0;
}

/*
  ----------------------------------------------------------------------
  ncr_write32
*/

int
ncr_write32(__uint32_t region, __uint32_t offset, __uint32_t value)
{
	int rc;

	NCR_TRACE_WRITE32(region, offset, value);
	rc = ncr_write(region, offset, 4, &value);

	if (0 != rc)
		return ncr_fail(__FILE__, __FUNCTION__, __LINE__);

	return 0;
}

/*
  ------------------------------------------------------------------------------
  ncr_and
*/

int
ncr_and( __uint32_t region, __uint32_t offset, __uint32_t value )
{
	__uint32_t temp;
	int rc = 0;

	rc |= ncr_read(region, offset, 4, &temp);
	temp &= value;
	rc |= ncr_write(region, offset, 4, &temp);

	if (0 != rc)
		return ncr_fail(__FILE__, __FUNCTION__, __LINE__);

	return 0;
}

/*
  ------------------------------------------------------------------------------
  ncr_or
*/

int
ncr_or( __uint32_t region, __uint32_t offset, __uint32_t value )
{
	__uint32_t temp;
	int rc = 0;

	rc |= ncr_read(region, offset, 4, &temp);
	temp |= value;
	rc |= ncr_write(region, offset, 4, &temp);

	if (0 != rc)
		return ncr_fail(__FILE__, __FUNCTION__, __LINE__);

	return 0;
}

/*
  ----------------------------------------------------------------------
  ncr_modify
*/

int
ncr_modify(__uint32_t region, __uint32_t address, int count,
	   __uint32_t *masks, __uint32_t *values)
{
	command_data_register_0_t cdr0;
	command_data_register_1_t cdr1;
	command_data_register_2_t cdr2;
	__uint64_t data_word_base;
	__uint32_t reg;
	int wfc_timeout = WFC_TIMEOUT;

	if (0 != ncr_lock(LOCK_DOMAIN))
		return -1;

	/*
	  Set up the write.
	*/

	cdr2.raw = 0;
	cdr2.bits.target_node_id = NCP_NODE_ID( region );
	cdr2.bits.target_id_address_upper = NCP_TARGET_ID( region );
	ncr_register_write( cdr2.raw, ( nca_base + NCP_NCA_CFG_PIO_CDR2 ) );

	cdr1.raw = 0;
	cdr1.bits.target_address = ( address >> 2 );
	ncr_register_write( cdr1.raw, ( nca_base + NCP_NCA_CFG_PIO_CDR1 ) );

	/*
	  Copy from buffer to the data words.
	*/

	data_word_base = ( nca_base + NCP_NCA_CDAR_MEMORY_BASE );
	ncr_register_write( count, data_word_base );
	data_word_base += 4;

	while( 0 < count ) {
		ncr_register_write( * ( ( __uint32_t * ) masks ),
				    data_word_base );
		data_word_base += 4;
		ncr_register_write( * ( ( __uint32_t * ) values ),
				    data_word_base );
		data_word_base += 4;
		masks++;
		values++;
		count--;
	}

	cdr0.raw = 0;
	cdr0.bits.start_done = 1;

	if( 0xff == cdr2.bits.target_id_address_upper ) {
		cdr0.bits.local_bit = 1;
	}

	cdr0.bits.cmd_type = 0x8;

	ncr_register_write( cdr0.raw, ( nca_base + NCP_NCA_CFG_PIO_CDR0 ) );

	/*
	  Wait for completion.
	*/

	do {
		--wfc_timeout;
	     reg = ncr_register_read( ( nca_base + NCP_NCA_CFG_PIO_CDR0 ) ) ;
         if ( (reg & 0x80000000) == 0) break;
	} while( 0 < wfc_timeout);

	if (0 == wfc_timeout) {
		tf_printf("ncr_modify(): NCA Lockup!\n");
		ncr_unlock(LOCK_DOMAIN);
		return -1;
	}

	/*
	  Check status.
	*/

	if( 0x3 !=
	    ( ( ncr_register_read( ( nca_base + NCP_NCA_CFG_PIO_CDR0 ) ) &
		0x00c00000 ) >> 22 ) ) {
#ifdef NCR_TRACER
		tf_printf( "ncr_write( ) failed: 0x%x\n",
			( ( ncr_register_read( ( nca_base + NCP_NCA_CFG_PIO_CDR0 ) ) &
			    0x00c00000 ) >> 22 ) );
#endif
		ncr_unlock(LOCK_DOMAIN);
		return -1;
	}

	ncr_unlock(LOCK_DOMAIN);
	return 0;
}

/*
  ----------------------------------------------------------------------
  ncp_modify32
*/

int
ncr_modify32( __uint32_t region, __uint32_t offset,
	      __uint32_t mask, __uint32_t value )
{
	int rc;

	NCR_TRACE_MODIFY(region, offset, mask, value);
	rc = ncr_modify( region, offset, 1, & mask, & value );

	if (0 != rc)
		return ncr_fail(__FILE__, __FUNCTION__, __LINE__);

	return 0;
}

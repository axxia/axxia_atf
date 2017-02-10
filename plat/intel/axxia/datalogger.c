/*
 *  Copyright (C) 2017 Intel Corporation
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

/* Data logging function using timer 7. */
/* Timer 7 generates an FIQ (INT Id 68, 0x44) on timeout */
/* datalogger_int_handler stores Primary CPU core and other registers to DDR  */
/* All other Secondary cores are sent an SGI (SGI 15) which jumps them to this function (INT Id 15 (0xf) */
/* Secondary cores write cpu registers to DDR */
/* DDR retention reset is then called */


#include <gic_v3.h>
#include <platform.h>
#include <debug.h>
#include <string.h>
#include "axxia_def.h"
#include "axxia_private.h"

#include <context.h>
#include <spinlock.h>


#ifdef CONFIG_DATALOGGER


/* Initial timer timeout causes a CPU to enter with int id PRIMARY_CPU_INT_ID */
/* SGI causes CPUs to enter with int id SECONDARY_CPU_INT_ID */
#define PRIMARY_CPU_INT_ID 68
#define SECONDARY_CPU_INT_ID  15


static spinlock_t  coreCountLock;
static volatile unsigned int secondaryCoreCount;


uint64_t datalogger_int_handler(uint32_t id,
				uint32_t flags,
				void *handle,
				void *cookie)
{
	/* CPU ID , including all affinity fields */
	unsigned int cpuNumber;
	unsigned int intID;

	/* SGI INTID = 0xf  to all other cores*/
	unsigned long long sgiValue = 0x000001000f000000;

	unsigned int i;
	unsigned int numberCores = NUM_CORES;
	volatile unsigned int temp;

	/* Read CPU number */
	__asm__ volatile("\t mrs %[value], mpidr_el1 \n" :
			 [value] "=r" (cpuNumber) );
	/* Mask out upper affinity fields, merge cpuid and clusterid */
	cpuNumber = (cpuNumber & 0x00000003) | ((cpuNumber & 0x0000ff00) >> 6);


	/* Get Int ID */
	intID = plat_ic_get_pending_interrupt_id();
	
	if (PRIMARY_CPU_INT_ID == intID) { 
		INFO(" CPU %x In datalogger int handler in BL31 \n",
		     cpuNumber); 

		/* First CPU into function disables timer 7 */
		*(unsigned int*)0x80802200e8 = 0; /* Disable timer7 */	
		*(unsigned int*)0x80802200ec = 1; /* Clear timer7 int */

		/*
		  First CPU into function checks watchdogs and kicks if enabled.
		*/

		/* Enables writes to critical regs */
		*(unsigned int *)(DEVICE0_BASE + 0x2c02000) = 0xab;

		/* Watchdog 0 */
		if ((*(unsigned int *)
		     (DEVICE0_BASE + 0x2c02008) & 0x00000100) != 0) {
			/*
			  If wd0_reset_enable bit set in reset control reg.
			*/

			if ((*(volatile unsigned int *)
			     (DEVICE0_BASE + 0x2200a0) ) != 0x00000000) {
				/*
				  If timers load value is not zero,
				  (re)write load val register to kick
				  watchdog.
				*/
				temp = *(volatile unsigned int *)
					(DEVICE0_BASE + 0x2200a0);
				*(volatile unsigned int *)
					(DEVICE0_BASE + 0x2200a0) = temp;
			}
		}

		/* Watchdog 1*/
		if ((*(unsigned int*)
		     (DEVICE0_BASE + 0x2c02008) & 0x00000200) != 0) {
			/*
			  If wd1_reset_enable bit set in reset control reg.
			*/

			if ((*(volatile unsigned int *)
			     (DEVICE0_BASE + 0x2200c0) ) != 0x00000000) {
				/*
				  If timers load value is not zero,
				  (re)write load val register to kick
				  watchdog.
				*/

				temp = *(volatile unsigned int *)
					(DEVICE0_BASE + 0x2200c0);
				*(volatile unsigned int *)
					(DEVICE0_BASE + 0x2200c0) = temp;
			}
		}

		/* Zero the ddr region used for datalog */
		memset((void *)DATALOGGER_STORE_BASE, 0,
		       (DATALOGGER_STORE_SIZE_PER_CPU * (NUM_CORES + 2)));
		__asm__ volatile("\t DSB sy \n");
		__asm__ volatile("\t ISB  \n");

		/* Dump registers to ddr */
		__asm__ volatile("\t msr	spsel, #1 \n");

		dump_cpu_registers(DATALOGGER_STORE_BASE +
				   (DATALOGGER_STORE_SIZE_PER_CPU * cpuNumber),
				   cpuNumber);   
		
		/* Store GIC registers after CPU registers */
		dump_gic_registers(DATALOGGER_STORE_BASE +
				   (DATALOGGER_STORE_SIZE_PER_CPU *
				    NUM_CORES));

		/* Store CCN-504 registers after GIC registers */
		dump_ccn504_registers((DATALOGGER_STORE_BASE +
				       (DATALOGGER_STORE_SIZE_PER_CPU *
					(NUM_CORES+1))));

		__asm__ volatile("\t msr	spsel, #0 \n");
		secondaryCoreCount = 0;

		/*
		  Send Secure SGI to other CPUs to call them into
		  Secure Monitor icc_sgi0r_el1.
		*/

		__asm__ volatile("\t msr S3_0_C12_C11_7 , %[value] \n" ::
				 [value] "r" (sgiValue) );

		/*
		  Poll until all other CPUs have dumped registers, or
		  loop times out.
		*/

		for (i = 0; i < DATALOGGER_POLL_LOOP_COUNT; i++)
			if (secondaryCoreCount == (numberCores -1))
				break;

		/* Trigger DDR retention reset. */
		INFO("Secondary Core Count %i  \n", secondaryCoreCount);
		INFO("ATF DDR retention reset!\n");

		/*
		  Set retention reset and datalogger_reset bits (bits
		  0 and 1 respectively) in pscratch.
		*/
		mmio_write_32(SYSCON_BASE + 0xdc,
			      (mmio_read_32(SYSCON_BASE + 0xdc) | 3));
		initiate_retention_reset();
		/* initiate_retention_reset should never return */
		ERROR(" Error - returned from initiate_retention_reset(), shoudl never happen !! \n");
	} else if ( SECONDARY_CPU_INT_ID == intID) {
		/* Dump registers to ddr */
		__asm__ volatile("\t msr	spsel, #1 \n");
		dump_cpu_registers(DATALOGGER_STORE_BASE +
				   (DATALOGGER_STORE_SIZE_PER_CPU * cpuNumber),
				   cpuNumber );
		__asm__ volatile("\t msr	spsel, #0 \n");

		spin_lock(&coreCountLock);
		/* Increment secondary core counter. */
		secondaryCoreCount++;
		spin_unlock(&coreCountLock);		

		/* Wait here until reset. */
		for (;;)
			;
	}

	/*
	  Only get here if an incorrect int id was received. Return to caller.
	*/

	return 0;
}

#endif /* CONFIG_DATALOGGER */

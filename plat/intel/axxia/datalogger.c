/*
 *  Copyright (C) 2013 LSI Corporation
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

	unsigned int cpuNumber;  			/* CPU ID , including all affinity fields */
	unsigned int intID;

	unsigned long long sgiValue = 0x000001000f000000; /* SGI INTID = 0xf  to all other cores*/

	unsigned int i;
	unsigned int numberCores = NUM_CORES;
	

	/* Read CPU number */
	__asm__ volatile("\t mrs %[value], mpidr_el1 \n" :[value] "=r" (cpuNumber) );
	cpuNumber = (cpuNumber &0x00000003) | ((cpuNumber & 0x0000ff00) >>6);  /* Mask out upper affinity fields, merge cpuid and clusterid */	

	VERBOSE(" CPU %x In datalogger int handler in BL31 \n", cpuNumber); 


	/* Get Int ID */
	intID = plat_ic_get_pending_interrupt_id();
	
	if ( PRIMARY_CPU_INT_ID == intID)
	{ 
		/* First CPU into function disables timer 7 */
		*(unsigned int*)0x80802200e8 = 0; //Disable timer	
		*(unsigned int*)0x80802200ec = 1; //Clear timer int

		__asm__ volatile("\t ISB  \n");

#if 0
		/* Read int registers to check if secure ints are enabled */
		VERBOSE("GICD_CTLR  %x  \n\r", *(unsigned int*)0x8010000000 );
		VERBOSE("GICR_IGROUPR0  %x  \n\r", *(unsigned int*)0x8010210080 );
		VERBOSE("GICR_ISENABLER0  %x  \n\r", *(unsigned int*)0x8010210100 );
		VERBOSE("GICR_ICENABLER0  %x  \n\r", *(unsigned int*)0x8010210180 );

		/* Check cores have different sp_el3 values */
		__asm__ volatile("\t msr	spsel, #1 \n");
		__asm__ volatile("\t mov %[value], sp \n" :[value] "=r" (el3_stack_pointer)  );
		__asm__ volatile("\t msr	spsel, #0 \n");
		VERBOSE("Primary sp_el3 %x \n\r", el3StackPointer);	

		VERBOSE("Attempting to store regs to DDR \n");
#endif

		/* Dump registers to ddr */
		
		__asm__ volatile("\t msr	spsel, #1 \n");
		dump_cpu_registers(DATALOGGER_STORE_BASE + (DATALOGGER_STORE_SIZE_PER_CPU * cpuNumber), \
				   cpuNumber);   
		
		/* Store GIC registers after CPU registers */
		dump_gic_registers(DATALOGGER_STORE_BASE + (DATALOGGER_STORE_SIZE_PER_CPU * NUM_CORES) ); 

		/* Store CCN-504 registers after GIC registers */
		dump_ccn504_registers( (DATALOGGER_STORE_BASE + (DATALOGGER_STORE_SIZE_PER_CPU * (NUM_CORES+1)) ) );

		__asm__ volatile("\t msr	spsel, #0 \n");

		secondaryCoreCount = 0;
	
		/* Send Secure SGI to other CPUs to call them into Secure Monitor icc_sgi0r_el1*/
		__asm__ volatile("\t msr S3_0_C12_C11_7 , %[value] \n" ::[value] "r" (sgiValue) );

		/* Poll until all other CPUs have dumped registers, or loop times out */
		for (i = 0; i < DATALOGGER_POLL_LOOP_COUNT; i++)
		{
			if  (secondaryCoreCount == (numberCores -1) )
			{
			 	break;
			}		
			
		}		 

		/* Trigger DDR retention reset  */
		VERBOSE("Secondary Core Count %i  \n", secondaryCoreCount); 
		INFO("ATF DDR retention reset!\n"); 

		/* set retention reset bit in pscratch */
		mmio_write_32(SYSCON_BASE + 0xdc, (mmio_read_32(SYSCON_BASE + 0xdc) | 1)  );
		initiate_retention_reset();
		/* initiate_retention_reset shoudl never return */
		ERROR(" Error - returned from initiate_retention_reset(), shoudl never happen !! \n"); 
	}
	else if ( SECONDARY_CPU_INT_ID == intID)
	{
		/* Dump registers to ddr */
		__asm__ volatile("\t msr	spsel, #1 \n");
		dump_cpu_registers(DATALOGGER_STORE_BASE + (DATALOGGER_STORE_SIZE_PER_CPU * cpuNumber), \
				   cpuNumber );   
		__asm__ volatile("\t msr	spsel, #0 \n");

		spin_lock(&coreCountLock);
		secondaryCoreCount++;		/* Increment secondary core counter*/
		spin_unlock(&coreCountLock);		

		while(1) {}  /* Wait here until reset */
		
	}

	/* Only get here if an incorrect int id was received. Return to caller */
	return 0;
}


#endif //CONFIG_DATALOGGER


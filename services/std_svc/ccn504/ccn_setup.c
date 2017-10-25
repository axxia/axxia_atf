#include <bl_common.h>
#include <stddef.h>
#include <axxia_def.h>
#include <axxia_private.h>
#include <debug.h>
#include "ccn_private.h"


unsigned int ccn_caps;
unsigned long dickens_base;



void ccn504_setup(void)
{
	ccn_caps = CCN_GENERIC_CAP;

	if (IS_5600())
		dickens_base = DICKENS_BASE_X9;
	else
		dickens_base = DICKENS_BASE_XLF;
#ifdef CCN_DEBUG
	INFO("DICKENS: 0x%lx\n", dickens_base);
#endif
}

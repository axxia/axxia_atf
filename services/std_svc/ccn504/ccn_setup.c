#include <bl_common.h>
#include <stddef.h>
#include "ccn_private.h"

unsigned int ccn_caps;

void ccn504_setup(void)
{
	ccn_caps = CCN_GENERIC_CAP;
}

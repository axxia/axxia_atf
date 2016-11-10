#include <debug.h>
#include <mmio.h>
#include <delay_timer.h>

#include <axxia_def.h>
#include <axxia_private.h>

#define L2CC_CTRL               (0x2030)
#define     PMU_FREEZALL        (1 << 5)
#define     PMU_ENABLE          (1 << 4)
#define     L2CC_ARRAY_SHUTDOWN (1 << 3)
#define     L2CC_POWER_DOWN_EN  (1 << 2)
#define     L2CC_FLUSH_INIT     (1 << 1)
#define     L2CC_RSTN           (1 << 0)

#define L2CC_STATUS             (0x2034)
#define     L2CC_POWER_DOWN_RDY (1 << 2)
#define     L2CC_FLUSH_COMP     (1 << 1)
#define     L2CC_FLUSH_START    (1 << 0)

#define CDC_CTRL                 (0x238)
#define     ADDR_TRANS_LCK       (1 << 7)
#define     ENABLE_ERR           (1 << 2)
#define     WAKE_ON_INT          (1 << 1)
#define     GATE_CLK             (1 << 0)

static unsigned long current_state = 0;
static unsigned long cdc[] = {	CDC0, CDC1, CDC2, CDC3 };

/*
  ==============================================================================
  ==============================================================================
  Private Functions
  ==============================================================================
  ==============================================================================
*/

static int
enable_dsp_cluster(unsigned int cluster)
{
	if (4 <= cluster)
		return -1;

	/* Reset and enable the DSP cluster's L2 cache. */
	mmio_write_32((cdc[cluster] + 0x2030), 0);
	udelay(100);
	mmio_write_32((cdc[cluster] + 0x2030), 1);

	/* Add the cluster to the coherency domain. */
	if (0 != set_cluster_coherency(8 + cluster, 1))
		return -1;

	return 0;
}

/*
  ------------------------------------------------------------------------------
  disable_dsp_cluster

  Turn off the given cluster.  No attempt is made to preserve the
  cache contents.
*/

static int
disable_dsp_cluster(unsigned int cluster)
{
	unsigned int control;
	unsigned int status;
	int retries;

	if (4 <= cluster)
		return -1;

	/* Remove the cluster from the coherency domain. */
	if (0 != set_cluster_coherency(8 + cluster, 0))
		return -1;

	control = mmio_read_32(cdc[cluster] + L2CC_CTRL);
	control |= L2CC_POWER_DOWN_EN;
	mmio_write_32(cdc[cluster] + L2CC_CTRL, control);

	retries = 150000;

	do {
		status = mmio_read_32(cdc[cluster] + L2CC_STATUS);
	} while ((0 == (status & L2CC_POWER_DOWN_RDY)) && (0 < --retries));

	if (0 == retries) {
		ERROR("Timed Out Powering Down DSP Cluster %d\n", cluster);

		return -1;
	}

	control |= L2CC_ARRAY_SHUTDOWN;
	mmio_write_32(cdc[cluster] + L2CC_CTRL, control);

	control &= ~L2CC_RSTN;
	mmio_write_32(cdc[cluster] + L2CC_CTRL, control);

	control = mmio_read_32(cdc[cluster] + CDC_CTRL);
	control |= GATE_CLK;
	mmio_write_32(cdc[cluster] + CDC_CTRL, control);

	return 0;
}

/*
  ==============================================================================
  ==============================================================================
  Public Functions
  ==============================================================================
  ==============================================================================
*/

unsigned long
get_dsp_state(void)
{
	INFO("Getting the DSP State: current_state=0x%lx\n", current_state);

	return current_state;
}

void
set_dsp_state(unsigned long state)
{
	int i;
	int rc;

	INFO("Setting the DSP State from 0x%lx to 0x%lx\n",
	     current_state, state);

	for (i = 0; i < 4; ++i) {
		int cs;
		int s;

		cs = (0 != (current_state & (1 << i)));
		s = (0 != (state & (1 << i)));

		if (0 == cs && 1 == s)
			rc = enable_dsp_cluster(i);
		else if (1 == cs && 0 == s)
			rc = disable_dsp_cluster(i);
		else
			continue;

		if (0 != rc) {
			ERROR("Changing DSP cluster %d from %d to %d failed!\n",
			      i, cs, s);
		} else {
			current_state &= ~(1 << i);
			current_state |= (state & (1 << i));
		}
	}

	return;
}

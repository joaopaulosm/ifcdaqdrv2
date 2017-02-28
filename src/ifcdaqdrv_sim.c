#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <libudev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <pthread.h>

#include "debug.h"
#include "ifcdaqdrv.h"
#include "ifcdaqdrv_utils.h"
#include "ifcdaqdrv_fmc.h"
#include "ifcdaqdrv_sim.h"
#include "ifcdaqdrv_scope.h"

static const uint32_t decimations[] = {1, 2, 5, 10, 20, 50, 100, 200, 0};
static const uint32_t averages[] = {1, 4, 8, 16, 32, 64, 128, 256, 0};
static const double   valid_clocks[] = {2400e6, 2500e6, 0};

struct ADCSIM_runtime_data runtime_data;

ifcdaqdrv_status ADCSim_register(struct ifcdaqdrv_dev *ifcdevice) {
    int status = 0;
    uint32_t nsamples_max;
    
    ifcdevice->init_adc              = ADCSim_init_adc;
    ifcdevice->get_gain              = ADCSim_get_gain;
    ifcdevice->set_gain              = ADCSim_set_gain;
    ifcdevice->set_nsamples          = ifcdaqdrv_scope_set_nsamples;
    ifcdevice->get_nsamples          = ifcdaqdrv_scope_get_nsamples;
    ifcdevice->set_trigger_threshold = ADCSim_set_trigger_threshold;
    ifcdevice->get_trigger_threshold = ADCSim_get_trigger_threshold;
    ifcdevice->set_clock_frequency   = ADCSim_set_clock_frequency;
    ifcdevice->get_clock_frequency   = ADCSim_get_clock_frequency;
    ifcdevice->set_clock_source      = ADCSim_set_clock_source;
    ifcdevice->get_clock_source      = ADCSim_get_clock_source;
    ifcdevice->set_clock_divisor     = ADCSim_set_clock_divisor;
    ifcdevice->get_clock_divisor     = ADCSim_get_clock_divisor;

    ifcdevice->set_pattern           = ADCSim_set_test_pattern;
    ifcdevice->get_pattern           = ADCSim_get_test_pattern;

    // ifcdevice->read_ai_ch            = ifcdaqdrv_scope_read_ai_ch;
    ifcdevice->read_ai               = ADCSim_read;

    // ifcdevice->normalize_ch          = ADCSim_read_ch;
    // ifcdevice->normalize             = ADCSim_read;

    ifcdevice->mode_switch = ifcdaqdrv_scope_switch_mode;

    ifcdevice->mode        = ifcdaqdrv_acq_mode_sram;
    ifcdevice->sample_size = 2;
    ifcdevice->nchannels   = 8;

    memcpy(ifcdevice->decimations,  decimations,  sizeof(decimations));
    memcpy(ifcdevice->averages,     averages,     sizeof(averages));

    ifcdevice->resolution  = 16;
    memcpy(ifcdevice->valid_clocks, valid_clocks, sizeof(valid_clocks));
    ifcdevice->divisor_max = 125; //1045;
    ifcdevice->divisor_min = 8; //1;

    ifcdevice->sample_resolution = 16;
    ifcdevice->vref_max = 1;

    ifcdevice->armed       = 0;
    ifcdevice->poll_period = 1000;

    nsamples_max = 16 * 1024;

    ifcdevice->sram_size = nsamples_max * ifcdevice->sample_size;
    ifcdevice->smem_size = 256 * 1024 * 1024;

    /* The subsystem lock is used to serialize access to the serial interface
     * since it requires several write/read pci accesses */
    pthread_mutex_init(&ifcdevice->sub_lock, NULL);

    return status;
}

ifcdaqdrv_status ADCSim_init_adc(struct ifcdaqdrv_dev *ifcdevice)
{
	int i;

	// set clock source internal
	ADCSim_set_clock_source(ifcdevice, ifcdaqdrv_clock_internal);

	// set gain to 1
	for (i=0; i<8; i++)
		runtime_data.channel_gain[i] = 1.0;

	// set unsigned data format
	runtime_data.dataformat = ifcdaqdrv_dataformat_unsigned;	

	return status_success;
}


ifcdaqdrv_status ADCSim_set_clock_source(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_clock clock)
{
	switch (clock)
	{
		case ifcdaqdrv_clock_internal:
			ADCSim_set_clock_frequency(ifcdevice, 1000e6);
			ADCSim_set_clock_divisor(ifcdevice, 10);
			runtime_data.clock_source = ifcdaqdrv_clock_internal;
			break;

		case ifcdaqdrv_clock_external:
			ADCSim_set_clock_frequency(ifcdevice, 250e6);
			ADCSim_set_clock_divisor(ifcdevice, 1);
			runtime_data.clock_source = ifcdaqdrv_clock_external;
			break;
	}

	return status_success;
}

ifcdaqdrv_status ADCSim_get_clock_source(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_clock *clock)
{
	if (runtime_data.clock_source == ifcdaqdrv_clock_external)||(runtime_data.clock_source == ifcdaqdrv_clock_internal) 
		return runtime_data.clock_source;
	else
		return ifcdaqdrv_clock_internal;
}



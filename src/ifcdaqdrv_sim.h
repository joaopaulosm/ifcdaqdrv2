#ifndef _IFCDAQDRV_SIM_H_
#define _IFCDAQDRV_SIM_H_

#include "ifcdaqdrv.h"
#include "ifcdaqdrv_utils.h"

/* Just a struct to hold data */
struct ADCSim_runtime_data {
	ifcdaqdrv_clock			clock_source;
	double 					channel_gain[8];
	ifcdaqdrv_dataformat 	dataformat;
}

ifcdaqdrv_status ADCSim_register(struct ifcdaqdrv_dev *ifcdevice);
ifcdaqdrv_status ADCSim_init_adc(struct ifcdaqdrv_dev *ifcdevice);
// ifcdaqdrv_status ADCSim_adc_init_priv(struct ifcdaqdrv_dev *ifcdevice, ADC3110_SBCDEVICE device);

ifcdaqdrv_status ADCSim_set_dataformat(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_dataformat dataformat);

ifcdaqdrv_status ADCSim_set_clock_frequency(struct ifcdaqdrv_dev *ifcdevice, double frequency);
ifcdaqdrv_status ADCSim_get_clock_frequency(struct ifcdaqdrv_dev *ifcdevice, double *frequency);

ifcdaqdrv_status ADCSim_set_clock_divisor(struct ifcdaqdrv_dev *ifcdevice, uint32_t divisor);
ifcdaqdrv_status ADCSim_get_clock_divisor(struct ifcdaqdrv_dev *ifcdevice, uint32_t *divisor);

ifcdaqdrv_status ADCSim_set_clock_source(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_clock clock);
ifcdaqdrv_status ADCSim_get_clock_source(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_clock *clock);

ifcdaqdrv_status ADCSim_set_gain(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, double gain);
ifcdaqdrv_status ADCSim_get_gain(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, double *gain);

ifcdaqdrv_status ADCSim_set_test_pattern(struct ifcdaqdrv_dev *ifcdevice, unsigned channel, ifcdaqdrv_pattern pattern);
ifcdaqdrv_status ADCSim_get_test_pattern(struct ifcdaqdrv_dev *ifcdevice, unsigned channel, ifcdaqdrv_pattern *pattern);

ifcdaqdrv_status ADCSim_read(struct ifcdaqdrv_dev *ifcdevice, void *dst, size_t dst_offset, void *src, size_t src_offset, size_t nelm, size_t channel_nsamples);
ifcdaqdrv_status ADCSim_read_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *res, void *data, size_t offset,
                              size_t nelm);

ifcdaqdrv_status ADCSim_set_trigger_threshold(struct ifcdaqdrv_dev *ifcdevice, int32_t threshold);
ifcdaqdrv_status ADCSim_get_trigger_threshold(struct ifcdaqdrv_dev *ifcdevice, int32_t *threshold);



#endif //_IFCDAQDRV_SIM_H_
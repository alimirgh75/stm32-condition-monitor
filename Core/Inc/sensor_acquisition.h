/*
 * sensor_acquisition.h
 *
 *  Created on: Sep 13, 2026
 *      Author: alimi
 */

#ifndef INC_SENSOR_ACQUISITION_H_
#define INC_SENSOR_ACQUISITION_H_


#include "ism330dhcx.h"

typedef struct
{
	ism330dhcx_t *device;
    uint8_t dma_rx_buffer[ISM330DHCX_SAMPLE_BYTE_COUNT];

    ism330dhcx_raw_sample_t raw_sample;

    volatile uint32_t drdy_event_count;
    uint32_t processed_drdy_event_count;
    volatile uint32_t dma_complete_count;

    volatile bool dma_busy;
    volatile bool dma_complete;
    bool sample_ready;

} sensor_acquisition_t;


void sensor_acquisition_init(
    sensor_acquisition_t *acquisition,
    ism330dhcx_t *device);

void sensor_acquisition_on_drdy(
    sensor_acquisition_t *acquisition);

void sensor_acquisition_on_dma_complete(
    sensor_acquisition_t *acquisition);

ism330dhcx_status_t sensor_acquisition_process(
    sensor_acquisition_t *acquisition);

bool sensor_acquisition_get_sample(
    sensor_acquisition_t *acquisition,
    ism330dhcx_raw_sample_t *sample);

uint32_t sensor_acquisition_get_drdy_count(
    const sensor_acquisition_t *acquisition);

uint32_t sensor_acquisition_get_dma_complete_count(
    const sensor_acquisition_t *acquisition);

#endif /* INC_SENSOR_ACQUISITION_H_ */

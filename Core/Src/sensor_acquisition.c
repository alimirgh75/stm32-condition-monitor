/*
 * sensor_acquisition.c
 *
 *  Created on: Sep 13, 2026
 *      Author: alimi
 */
#include "sensor_acquisition.h"

void sensor_acquisition_init(
    sensor_acquisition_t *acquisition,
    ism330dhcx_t *device)
{
    if ((acquisition == NULL) || (device == NULL))
    {
        return;
    }

    acquisition->device = device;


    acquisition->drdy_event_count = 0U;
    acquisition->processed_drdy_event_count = 0U;
    acquisition->dma_complete_count = 0U;


    acquisition->dma_busy = false;
    acquisition->dma_complete = false;
    acquisition->sample_ready = false;
}


void sensor_acquisition_on_drdy(
    sensor_acquisition_t *acquisition){
    if (acquisition == NULL)
    {
        return;
    }

	++acquisition->drdy_event_count;
}

void sensor_acquisition_on_dma_complete(
    sensor_acquisition_t *acquisition){
    if (acquisition == NULL)
    {
        return;
    }

    acquisition->dma_busy = false;
    acquisition->dma_complete = true;
    ++acquisition->dma_complete_count;

}

ism330dhcx_status_t sensor_acquisition_process(
    sensor_acquisition_t *acquisition)
{
	if ((acquisition == NULL) ||
	    (acquisition->device == NULL))
	{
	    return ISM330DHCX_INVALID_ARGUMENT;
	}



	if ((!acquisition->dma_busy) &&
	    (!acquisition->dma_complete) &&
	    (!acquisition->sample_ready) &&
	    (acquisition->processed_drdy_event_count !=
	     acquisition->drdy_event_count)){



	    const ism330dhcx_status_t dma_status =
	        ism330dhcx_start_sample_read_dma(
	            acquisition->device,
				acquisition->dma_rx_buffer,
	            ISM330DHCX_SAMPLE_BYTE_COUNT);

	    if (dma_status == ISM330DHCX_OK)
	    {
	    	acquisition->dma_busy = true;
	    	++acquisition->processed_drdy_event_count;
	    }
	    else
	    {
	        return dma_status;
	    }

	}



	if (acquisition->dma_complete)
	{
	    const ism330dhcx_status_t decode_status =
	        ism330dhcx_decode_raw_sample(
	            acquisition->dma_rx_buffer,
	            ISM330DHCX_SAMPLE_BYTE_COUNT,
	            &acquisition->raw_sample);

	    if (decode_status != ISM330DHCX_OK)
	    {
	        return decode_status;
	    }

	    acquisition->dma_complete = false;
	    acquisition->sample_ready = true;
	}


	return ISM330DHCX_OK;
}

bool sensor_acquisition_get_sample(
    sensor_acquisition_t *acquisition,
    ism330dhcx_raw_sample_t *sample)
{
    if ((acquisition == NULL) || (sample == NULL))
    {
        return false;
    }

    if (!acquisition->sample_ready)
    {
        return false;
    }

    *sample = acquisition->raw_sample;
    acquisition->sample_ready = false;

    return true;
}
uint32_t sensor_acquisition_get_drdy_count(
    const sensor_acquisition_t *acquisition)
{
    if (acquisition == NULL)
    {
        return 0U;
    }

    return acquisition->drdy_event_count;
}

uint32_t sensor_acquisition_get_dma_complete_count(
    const sensor_acquisition_t *acquisition)
{
    if (acquisition == NULL)
    {
        return 0U;
    }

    return acquisition->dma_complete_count;
}




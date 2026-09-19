/*
 * sensor_analysis_window.c
 *
 *  Created on: Sep 13, 2026
 *      Author: alimi
 */


#include "sensor_analysis_window.h"



void sensor_window_init(sensor_window_t *window)
{
    if (window == NULL)
    {
        return;
    }

    window->index = 0U;
    window->completed_count = 0U;
    window->ready = false;
}
bool sensor_window_push(sensor_window_t *window, const sensor_physical_sample_t *sample)
{
    if ((window == NULL) || (sample == NULL))
    {
        return false;
    }

    if (sensor_window_is_ready(window))
    {
        return false;
    }

    window->samples[window->index] = *sample;

    ++window->index;

    if (window->index >= SENSOR_ANALYSIS_WINDOW_SIZE)
    {
        window->ready = true;
        ++window->completed_count;
    }


    return true;
}
bool sensor_window_is_ready(const sensor_window_t *window)
{
    if (window == NULL)
    {
        return false;
    }

	return window->ready;
}
void sensor_window_release(sensor_window_t *window)
{
    if (window == NULL)
    {
        return;
    }

	window->index = 0U;
	window->ready = false;
}
uint32_t sensor_window_get_completed_count(
    const sensor_window_t *window)
{

    if (window == NULL)
    {
        return 0;
    }

	return window->completed_count;
}

bool accelerometer_calculate_rms(const sensor_window_t *input, ism330dhcx_axes_t * rms_output, ism330dhcx_axes_t centered_output[SENSOR_ANALYSIS_WINDOW_SIZE])
{
    if((input == NULL) || (!sensor_window_is_ready(input)) || (centered_output == NULL) || (rms_output == NULL))
    {

        return false;
    }


    float mean_x = 0.0f;
    float mean_y = 0.0f;
    float mean_z = 0.0f;

    float rms_x = 0.0f;
    float rms_y = 0.0f;
    float rms_z = 0.0f;

    for(uint32_t i = 0U; i<SENSOR_ANALYSIS_WINDOW_SIZE; i++)
    {
        mean_x +=  input->samples[i].data.acceleration_mps2.x;
        mean_y +=  input->samples[i].data.acceleration_mps2.y;
        mean_z +=  input->samples[i].data.acceleration_mps2.z;


    }


    mean_x /= SENSOR_ANALYSIS_WINDOW_SIZE;
    mean_y /= SENSOR_ANALYSIS_WINDOW_SIZE;
    mean_z /= SENSOR_ANALYSIS_WINDOW_SIZE;

    for(uint32_t i = 0U; i<SENSOR_ANALYSIS_WINDOW_SIZE; i++)
    {
    	centered_output[i].x = input->samples[i].data.acceleration_mps2.x - mean_x;
    	rms_x += centered_output[i].x * centered_output[i].x;

    	centered_output[i].y = input->samples[i].data.acceleration_mps2.y - mean_y;
    	rms_y += centered_output[i].y * centered_output[i].y;

    	centered_output[i].z = input->samples[i].data.acceleration_mps2.z - mean_z;
    	rms_z += centered_output[i].z * centered_output[i].z;
    }


    rms_x /= SENSOR_ANALYSIS_WINDOW_SIZE;
    rms_y /= SENSOR_ANALYSIS_WINDOW_SIZE;
    rms_z /= SENSOR_ANALYSIS_WINDOW_SIZE;


    rms_output->x = sqrtf(rms_x);
    rms_output->y = sqrtf(rms_y);
    rms_output->z = sqrtf(rms_z);

    return true;

}

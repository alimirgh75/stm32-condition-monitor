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

bool sensor_window_calculate_acceleration_features(
    const sensor_window_t *input,
    sensor_acceleration_time_features_t *features_output,
    ism330dhcx_axes_t
        centered_output[SENSOR_ANALYSIS_WINDOW_SIZE]){
    if((input == NULL) || (!sensor_window_is_ready(input)) || (centered_output == NULL) || (features_output == NULL))
    {

        return false;
    }


    float mean_x = 0.0f;
    float mean_y = 0.0f;
    float mean_z = 0.0f;

    float rms_x = 0.0f;
    float rms_y = 0.0f;
    float rms_z = 0.0f;


    float peak_x = 0.0f;
    float peak_y = 0.0f;
    float peak_z = 0.0f;

    float magnitude_squared = 0.0f;
    float max_magnitude_squared = 0.0f;

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

    	const float absolute_x = fabsf(centered_output[i].x);

    	if (absolute_x > peak_x)
    	{
    	    peak_x = absolute_x;
    	}

    	centered_output[i].y = input->samples[i].data.acceleration_mps2.y - mean_y;
    	rms_y += centered_output[i].y * centered_output[i].y;

    	const float absolute_y = fabsf(centered_output[i].y);

    	if (absolute_y > peak_y)
    	{
    	    peak_y = absolute_y;
    	}

    	centered_output[i].z = input->samples[i].data.acceleration_mps2.z - mean_z;
    	rms_z += centered_output[i].z * centered_output[i].z;

    	const float absolute_z = fabsf(centered_output[i].z);

    	if (absolute_z > peak_z)
    	{
    	    peak_z = absolute_z;
    	}

    	magnitude_squared = centered_output[i].x * centered_output[i].x +
    						centered_output[i].y * centered_output[i].y +
							centered_output[i].z * centered_output[i].z;

    	if (magnitude_squared > max_magnitude_squared)
    	{
    		max_magnitude_squared = magnitude_squared;
    		features_output->max_magnitude_index = i;
    	}
    }

    features_output->max_magnitude_mps2 = sqrtf(max_magnitude_squared);

    features_output->max_magnitude_timestamp_us = input->samples[features_output->max_magnitude_index].timestamp_us;

    rms_x /= SENSOR_ANALYSIS_WINDOW_SIZE;
    rms_y /= SENSOR_ANALYSIS_WINDOW_SIZE;
    rms_z /= SENSOR_ANALYSIS_WINDOW_SIZE;


    features_output->rms_mps2.x = sqrtf(rms_x);
    features_output->rms_mps2.y = sqrtf(rms_y);
    features_output->rms_mps2.z = sqrtf(rms_z);
    features_output->peak_mps2.x = peak_x;
    features_output->peak_mps2.y = peak_y;
    features_output->peak_mps2.z = peak_z;

    return true;

}

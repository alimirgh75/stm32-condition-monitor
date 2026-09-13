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
bool sensor_window_push(sensor_window_t *window, const ism330dhcx_sample_t *sample)
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

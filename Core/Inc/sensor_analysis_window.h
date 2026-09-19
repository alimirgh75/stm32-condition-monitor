/*
 * sensor_analysis_window.h
 *
 *  Created on: Sep 13, 2026
 *      Author: alimi
 */

#ifndef INC_SENSOR_ANALYSIS_WINDOW_H_
#define INC_SENSOR_ANALYSIS_WINDOW_H_

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "ism330dhcx.h"
#include "sensor_sample.h"

#define SENSOR_ANALYSIS_WINDOW_SIZE 128U

typedef struct
{
	sensor_physical_sample_t samples[SENSOR_ANALYSIS_WINDOW_SIZE];
    uint32_t index;
    uint32_t completed_count;
    bool ready;

} sensor_window_t;



void sensor_window_init(sensor_window_t *window);
bool sensor_window_push(sensor_window_t *window, const sensor_physical_sample_t *sample);
bool sensor_window_is_ready(const sensor_window_t *window);
void sensor_window_release(
    sensor_window_t *window);

uint32_t sensor_window_get_completed_count(
    const sensor_window_t *window);

bool accelerometer_calculate_rms(const sensor_window_t *input, ism330dhcx_axes_t * rms_output, ism330dhcx_axes_t centered_output[SENSOR_ANALYSIS_WINDOW_SIZE]);

#endif /* INC_SENSOR_ANALYSIS_WINDOW_H_ */

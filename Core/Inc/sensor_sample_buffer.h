/*
 * sensor_sample_buffer.h
 *
 *  Created on: Sep 13, 2026
 *      Author: alimi
 */

#ifndef INC_SENSOR_SAMPLE_BUFFER_H_
#define INC_SENSOR_SAMPLE_BUFFER_H_


#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "ism330dhcx.h"
#include "sensor_sample.h"
//Sensor RING BUFFER variables
#define SENSOR_SAMPLE_BUFFER_CAPACITY 32U


typedef struct
{
    sensor_sample_t samples[SENSOR_SAMPLE_BUFFER_CAPACITY];

    uint32_t write_index;
    uint32_t read_index;
    uint32_t count;
    uint32_t overrun_count;

} sensor_sample_buffer_t;

void sensor_sample_buffer_init(
    sensor_sample_buffer_t *buffer);

bool sensor_sample_buffer_push(
    sensor_sample_buffer_t *buffer,
    const sensor_sample_t *sample);

bool sensor_sample_buffer_pop(
    sensor_sample_buffer_t *buffer,
    sensor_sample_t *sample);

uint32_t sensor_sample_buffer_get_count(
    const sensor_sample_buffer_t *buffer);

uint32_t sensor_sample_buffer_get_overrun_count(
    const sensor_sample_buffer_t *buffer);






#endif /* INC_SENSOR_SAMPLE_BUFFER_H_ */

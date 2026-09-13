/*
 * sensor_sample_buffer.c
 *
 *  Created on: Sep 13, 2026
 *      Author: alimi
 */


/*
 *
 * Move:
- sensor_sample_buffer[]
- write_index
- read_index
- count
- overrun_count
- push()
- pop()
 */

#include "sensor_sample_buffer.h"

void sensor_sample_buffer_init(
    sensor_sample_buffer_t *buffer)
{
    if (buffer == NULL)
    {
        return;
    }

    buffer->write_index = 0U;
    buffer->read_index = 0U;
    buffer->count = 0U;
    buffer->overrun_count = 0U;
}

bool sensor_sample_buffer_push(
    sensor_sample_buffer_t *buffer,
    const ism330dhcx_raw_sample_t *sample)
{
    if ((buffer == NULL) || (sample == NULL))
    {
        return false;
    }

    if (buffer->count >= SENSOR_SAMPLE_BUFFER_CAPACITY)
    {
        ++buffer->overrun_count;
        return false;
    }

    buffer->samples[buffer->write_index] = *sample;

    buffer->write_index =
        (buffer->write_index + 1U) %
        SENSOR_SAMPLE_BUFFER_CAPACITY;

    ++buffer->count;

    return true;
}

bool sensor_sample_buffer_pop(
    sensor_sample_buffer_t *buffer,
    ism330dhcx_raw_sample_t *sample)
{
    if ((buffer == NULL) ||
        (sample == NULL) ||
        (buffer->count == 0U))
    {
        return false;
    }

    *sample = buffer->samples[buffer->read_index];

    buffer->read_index =
        (buffer->read_index + 1U) %
        SENSOR_SAMPLE_BUFFER_CAPACITY;

    --buffer->count;

    return true;
}

uint32_t sensor_sample_buffer_get_count(
    const sensor_sample_buffer_t *buffer)
{
    if (buffer == NULL)
    {
        return 0U;
    }

    return buffer->count;
}

uint32_t sensor_sample_buffer_get_overrun_count(
    const sensor_sample_buffer_t *buffer)
{
    if (buffer == NULL)
    {
        return 0U;
    }

    return buffer->overrun_count;
}

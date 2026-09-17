/*
 * sensor_sample.h
 *
 *  Created on: Sep 16, 2026
 *      Author: alimi
 */

#ifndef INC_SENSOR_SAMPLE_H_
#define INC_SENSOR_SAMPLE_H_

#include <stdint.h>
#include "ism330dhcx.h"

typedef struct
{
    ism330dhcx_raw_sample_t data;
    uint32_t timestamp_us;
} sensor_sample_t;

#endif

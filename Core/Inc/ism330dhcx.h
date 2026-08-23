/*
 * ism330dhcx.h
 *
 *  Created on: Aug 12, 2026
 *      Author: alimi
 */


#ifndef ISM330DHCX_H
#define ISM330DHCX_H

#include "stm32l4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

//Motion sensor
#define ISM330DHCX_ADDRESS_7BIT     0x6BU
#define ISM330DHCX_ADDRESS_HAL      (ISM330DHCX_ADDRESS_7BIT << 1U)
#define ISM330DHCX_I2C_ADDRESS_7BIT  0x6BU
#define ISM330DHCX_EXPECTED_ID       0x6BU



//Create four public enum types:

typedef enum
{
    ISM330DHCX_ACCEL_ODR_PWR_DOWN = 0,
	ISM330DHCX_ACCEL_ODR_12_5_HZ,
	ISM330DHCX_ACCEL_ODR_26_HZ,
	ISM330DHCX_ACCEL_ODR_52_HZ,
	ISM330DHCX_ACCEL_ODR_104_HZ,
	ISM330DHCX_ACCEL_ODR_208_HZ,
	ISM330DHCX_ACCEL_ODR_416_HZ,
	ISM330DHCX_ACCEL_ODR_833_HZ,
	ISM330DHCX_ACCEL_ODR_1660_HZ,
	ISM330DHCX_ACCEL_ODR_3330_HZ,
	ISM330DHCX_ACCEL_ODR_6660_HZ,

	ISM330DHCX_ACCEL_ODR_COUNT
} ism330dhcx_accel_odr_t;

typedef enum
{
	ISM330DHCX_ACCEL_RANGE_2G = 0,
	ISM330DHCX_ACCEL_RANGE_16G,
	ISM330DHCX_ACCEL_RANGE_4G,
	ISM330DHCX_ACCEL_RANGE_8G,


	ISM330DHCX_ACCEL_RANGE_COUNT
} ism330dhcx_accel_range_t;


typedef enum
{
    ISM330DHCX_GYRO_ODR_PWR_DOWN = 0,
	ISM330DHCX_GYRO_ODR_12_5_HZ,
	ISM330DHCX_GYRO_ODR_26_HZ,
	ISM330DHCX_GYRO_ODR_52_HZ,
	ISM330DHCX_GYRO_ODR_104_HZ,
	ISM330DHCX_GYRO_ODR_208_HZ,
	ISM330DHCX_GYRO_ODR_416_HZ,
	ISM330DHCX_GYRO_ODR_833_HZ,
	ISM330DHCX_GYRO_ODR_1660_HZ,
	ISM330DHCX_GYRO_ODR_3330_HZ,
	ISM330DHCX_GYRO_ODR_6660_HZ,

	ISM330DHCX_GYRO_ODR_COUNT
} ism330dhcx_gyro_odr_t;

typedef enum
{
    ISM330DHCX_GYRO_125_DPS = 0,
    ISM330DHCX_GYRO_250_DPS,
    ISM330DHCX_GYRO_500_DPS,
    ISM330DHCX_GYRO_1000_DPS,
    ISM330DHCX_GYRO_2000_DPS,
    ISM330DHCX_GYRO_4000_DPS,

    ISM330DHCX_GYRO_DPS_COUNT
} ism330dhcx_gyro_range_t;

typedef enum
{
    ISM330DHCX_OK = 0,
    ISM330DHCX_ERROR,
    ISM330DHCX_INVALID_ARGUMENT,
    ISM330DHCX_WRONG_DEVICE,
    ISM330DHCX_TIMEOUT,
	ISM330DHCX_NOT_CONFIGURED
} ism330dhcx_status_t;

typedef enum
{
    ISM330DHCX_MODE_HIGH_PERFORMANCE = 0,
    ISM330DHCX_MODE_LOW_POWER_NORMAL,

    ISM330DHCX_MODE_COUNT
} ism330dhcx_performance_mode_t;

typedef struct
{
    ism330dhcx_accel_odr_t accel_odr;
    ism330dhcx_accel_range_t accel_range;
    ism330dhcx_performance_mode_t accel_mode;

    ism330dhcx_gyro_odr_t gyro_odr;
    ism330dhcx_gyro_range_t gyro_range;
    ism330dhcx_performance_mode_t gyro_mode;


} ism330dhcx_sensor_config_t;

typedef struct
{
	I2C_HandleTypeDef *i2c;
	uint16_t address;
	uint32_t timeout_ms;

    ism330dhcx_sensor_config_t sensor_config;
    bool sensor_configured;

}ism330dhcx_t;

typedef struct
{
    bool block_data_update;
    bool auto_increment;
} ism330dhcx_interface_config_t;


typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}ism330dhcx_raw_axes_t;

typedef struct
{
    ism330dhcx_raw_axes_t gyro;
    ism330dhcx_raw_axes_t accel;
} ism330dhcx_raw_sample_t;


typedef struct
{
    float x;
    float y;
    float z;
} ism330dhcx_axes_t;

typedef struct
{
    ism330dhcx_axes_t acceleration_mps2;
    ism330dhcx_axes_t angular_rate_dps;
} ism330dhcx_sample_t;

ism330dhcx_status_t ism330dhcx_read_raw_sample(
    const ism330dhcx_t *device,
    ism330dhcx_raw_sample_t *sample);

ism330dhcx_status_t ism330dhcx_init(
		ism330dhcx_t *device,
		I2C_HandleTypeDef *i2c,
		uint8_t address_7bit,
		uint32_t timeout_ms);

ism330dhcx_status_t ism330dhcx_read_device_id(
		const ism330dhcx_t *device,
		uint8_t *device_id);





/*Set the SW_RESET bit in CTRL3_C.
Check until the sensor clears the bit.
Have a finite timeout.
Return a timeout/error instead of hanging.
Not use HAL_Delay().*/
ism330dhcx_status_t ism330dhcx_reset(ism330dhcx_t *device);

ism330dhcx_status_t ism330dhcx_configure_interface(const ism330dhcx_t *device, const ism330dhcx_interface_config_t *config);


ism330dhcx_status_t ism330dhcx_configure_sensor(ism330dhcx_t *device, const ism330dhcx_sensor_config_t *config);


ism330dhcx_status_t ism330dhcx_convert_raw_sample(
    const ism330dhcx_t *device,
    const ism330dhcx_raw_sample_t *raw_sample,
    ism330dhcx_sample_t *sample);

#endif


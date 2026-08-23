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
#define ISM330DHCX_I2C_ADDRESS_LOW_7BIT   0x6AU
#define ISM330DHCX_I2C_ADDRESS_HIGH_7BIT  0x6BU
#define ISM330DHCX_EXPECTED_ID            0x6BU

typedef enum
{
	/** Accelerometer powered down. */
    ISM330DHCX_ACCEL_ODR_PWR_DOWN = 0,

	/** Accelerometer operates at 1.6 Hz  */
	ISM330DHCX_ACCEL_ODR_1_6_HZ,

	/** Accelerometer operates at 12.5 Hz  */
	ISM330DHCX_ACCEL_ODR_12_5_HZ,

	/** Accelerometer operates at 26 Hz  */
	ISM330DHCX_ACCEL_ODR_26_HZ,

	/** Accelerometer operates at 52 Hz  */
	ISM330DHCX_ACCEL_ODR_52_HZ,

	/** Accelerometer operates at 104 Hz  */
	ISM330DHCX_ACCEL_ODR_104_HZ,

	/** Accelerometer operates at 208 Hz  */
	ISM330DHCX_ACCEL_ODR_208_HZ,

	/** Accelerometer operates at 416 Hz  */
	ISM330DHCX_ACCEL_ODR_416_HZ,

	/** Accelerometer operates at 833 Hz  */
	ISM330DHCX_ACCEL_ODR_833_HZ,

	/** Accelerometer operates at 1660 Hz  */
	ISM330DHCX_ACCEL_ODR_1660_HZ,

	/** Accelerometer operates at 3330 Hz  */
	ISM330DHCX_ACCEL_ODR_3330_HZ,

	/** Accelerometer operates at 6660 Hz  */
	ISM330DHCX_ACCEL_ODR_6660_HZ,

	ISM330DHCX_ACCEL_ODR_COUNT
} ism330dhcx_accel_odr_t;

typedef enum
{
	/** Accelerometer operates between -2g and +2g  */
	ISM330DHCX_ACCEL_RANGE_2G = 0,

	/** Accelerometer operates between -16g and +16g  */
	ISM330DHCX_ACCEL_RANGE_16G,

	/** Accelerometer operates between -4g and +4g  */
	ISM330DHCX_ACCEL_RANGE_4G,

	/** Accelerometer operates between -8g and +8g  */
	ISM330DHCX_ACCEL_RANGE_8G,


	ISM330DHCX_ACCEL_RANGE_COUNT
} ism330dhcx_accel_range_t;


typedef enum
{
	/** Gyroscope powered down. */
    ISM330DHCX_GYRO_ODR_PWR_DOWN = 0,

	/** Gyroscope operates at 12.5 Hz  */
	ISM330DHCX_GYRO_ODR_12_5_HZ,

	/** Gyroscope operates at 26 Hz  */
	ISM330DHCX_GYRO_ODR_26_HZ,

	/** Gyroscope operates at 52 Hz  */
	ISM330DHCX_GYRO_ODR_52_HZ,

	/** Gyroscope operates at 104 Hz  */
	ISM330DHCX_GYRO_ODR_104_HZ,

	/** Gyroscope operates at 208 Hz  */
	ISM330DHCX_GYRO_ODR_208_HZ,

	/** Gyroscope operates at 416 Hz  */
	ISM330DHCX_GYRO_ODR_416_HZ,

	/** Gyroscope operates at 833 Hz  */
	ISM330DHCX_GYRO_ODR_833_HZ,

	/** Gyroscope operates at 1660 Hz  */
	ISM330DHCX_GYRO_ODR_1660_HZ,

	/** Gyroscope operates at 3330 Hz  */
	ISM330DHCX_GYRO_ODR_3330_HZ,

	/** Gyroscope operates at 6660 Hz  */
	ISM330DHCX_GYRO_ODR_6660_HZ,

	ISM330DHCX_GYRO_ODR_COUNT
} ism330dhcx_gyro_odr_t;

typedef enum
{
	/** Gyroscope operates at 125 dps sensivity   */
    ISM330DHCX_GYRO_125_DPS = 0,

	/** Gyroscope operates at 250 dps sensivity   */
    ISM330DHCX_GYRO_250_DPS,

	/** Gyroscope operates at 500 dps sensivity   */
    ISM330DHCX_GYRO_500_DPS,

	/** Gyroscope operates at 1000 dps sensivity   */
    ISM330DHCX_GYRO_1000_DPS,

	/** Gyroscope operates at 2000 dps sensivity   */
    ISM330DHCX_GYRO_2000_DPS,

	/** Gyroscope operates at 4000 dps sensivity   */
    ISM330DHCX_GYRO_4000_DPS,

    ISM330DHCX_GYRO_DPS_COUNT
} ism330dhcx_gyro_range_t;

typedef enum
{
    /** Operation completed successfully. */
    ISM330DHCX_OK = 0,

    /** A pointer, enum value, address or length was invalid. */
    ISM330DHCX_INVALID_ARGUMENT,

    /** A bus transaction failed for a reason other than timeout. */
	ISM330DHCX_ERROR,

    /** The I2C transaction or reset operation timed out. */
    ISM330DHCX_TIMEOUT,

    /** WHO_AM_I did not contain the expected value. */
    ISM330DHCX_WRONG_DEVICE,

    /** An operation requiring active configuration was requested too early. */
    ISM330DHCX_NOT_CONFIGURED
} ism330dhcx_status_t;
typedef enum
{

	/** Sensor operates at High performance mode. */
    ISM330DHCX_MODE_HIGH_PERFORMANCE = 0,

	/** Sensor operates at Low-power performance mode. */
    ISM330DHCX_MODE_LOW_POWER_NORMAL,

    ISM330DHCX_MODE_COUNT
} ism330dhcx_performance_mode_t;


/**
 * @brief ISM330DHCX Sensor configuration.
 */
typedef struct
{

    ism330dhcx_accel_odr_t accel_odr;
    ism330dhcx_accel_range_t accel_range;
    ism330dhcx_performance_mode_t accel_mode;

    ism330dhcx_gyro_odr_t gyro_odr;
    ism330dhcx_gyro_range_t gyro_range;
    ism330dhcx_performance_mode_t gyro_mode;


} ism330dhcx_sensor_config_t;


/**
 * @brief ISM330DHCX Device instance/state.
 */
typedef struct
{
	I2C_HandleTypeDef *i2c;
	uint16_t address;
	uint32_t timeout_ms;

    ism330dhcx_sensor_config_t sensor_config;
    bool sensor_configured;

}ism330dhcx_t;



/**
 * @brief ISM330DHCX Interface configuration.
 */
typedef struct
{
    bool block_data_update;
    bool auto_increment;
} ism330dhcx_interface_config_t;

/**
 * @brief Raw signed output values read from the sensor registers.
 */
typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}ism330dhcx_raw_axes_t;


/**
 * @brief ISM330DHCX Raw sample.
 */
typedef struct
{
    ism330dhcx_raw_axes_t gyro;
    ism330dhcx_raw_axes_t accel;
} ism330dhcx_raw_sample_t;

/**
 * @brief ISM330DHCX float converted from signed output values read from the sensor registers.
 */
typedef struct
{
    float x;
    float y;
    float z;
} ism330dhcx_axes_t;

/**
 * @brief Converted sensor sample.
 *
 * Acceleration is expressed in m/s².
 * Angular rate is expressed in degrees per second.
 */

typedef struct
{
    ism330dhcx_axes_t acceleration_mps2;
    ism330dhcx_axes_t angular_rate_dps;
} ism330dhcx_sample_t;



/**
 * @brief Read Raw sensor sample.
 *
 */
ism330dhcx_status_t ism330dhcx_read_raw_sample(
    const ism330dhcx_t *device,
    ism330dhcx_raw_sample_t *sample);

/**
 * @brief Initialize the sensor instance.
 *
 *	device instance
 *	I2C bus
 *	7-bit slave address of the sensor
 *	timeout for the I2C communication in millisecond
 */

ism330dhcx_status_t ism330dhcx_init(
		ism330dhcx_t *device,
		I2C_HandleTypeDef *i2c,
		uint8_t address_7bit,
		uint32_t timeout_ms);


/**
 * @brief Initialize the sensor instance.
 *
 *	device instance
 *	device_id read from the device
 */
ism330dhcx_status_t ism330dhcx_read_device_id(
		const ism330dhcx_t *device,
		uint8_t *device_id);


/**
 * @brief Resets the sensor instance.
 *
 *	device instance
 */
ism330dhcx_status_t ism330dhcx_reset(ism330dhcx_t *device);


/**
 * @brief Configures the sensor interface.
 *
 *	device instance
 *	Interface configuration for ism330dhcx_interface_config_t
 */
ism330dhcx_status_t ism330dhcx_configure_interface(const ism330dhcx_t *device, const ism330dhcx_interface_config_t *config);

/**
 * @brief Configures the sensor interface.
 *
 *	device instance
 *	Sensor configuration for ism330dhcx_sensor_config_t
 */
ism330dhcx_status_t ism330dhcx_configure_sensor(ism330dhcx_t *device, const ism330dhcx_sensor_config_t *config);

/**
 * @brief Converts the raw sample to physical sample.
 *
 *	device instance
 *	Signed raw sample read from the sensor
 *	float Physical sample converted from the Raw sample
 */
ism330dhcx_status_t ism330dhcx_convert_raw_sample(
    const ism330dhcx_t *device,
    const ism330dhcx_raw_sample_t *raw_sample,
    ism330dhcx_sample_t *sample);

#endif


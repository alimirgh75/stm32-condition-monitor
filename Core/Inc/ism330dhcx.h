/*
 * ism330dhcx.h
 *
 *  Created on: Aug 12, 2026
 *      Author: alimi
 */


#ifndef ISM330DHCX_H
#define ISM330DHCX_H

#include "stm32l4xx_hal.h"
#include <stdint.h>

//Motion sensor
#define ISM330DHCX_ADDRESS_7BIT     0x6BU
#define ISM330DHCX_ADDRESS_HAL      (ISM330DHCX_ADDRESS_7BIT << 1U)
#define ISM330DHCX_I2C_ADDRESS_7BIT  0x6BU
#define ISM330DHCX_EXPECTED_ID       0x6BU
#define ISM330DHCX_WHO_AM_I_VALUE   0x6BU



typedef enum
{
	ISM330DHCX_OK =0,
	ISM330DHCX_ERROR,
	ISM330DHCX_INVALID_ARGUMENT,
	ISM330DHCX_WRONG_DEVICE

} ism330dhcx_status_t;


typedef struct
{
	I2C_HandleTypeDef *i2c;
	uint16_t address;
	uint32_t timeout_ms;

}ism330dhcx_t;

ism330dhcx_status_t ism330dhcx_init(
		ism330dhcx_t *device,
		I2C_HandleTypeDef *i2c,
		uint8_t address_7bit,
		uint32_t timeout_ms);

ism330dhcx_status_t ism330dhcx_read_device_id(
		const ism330dhcx_t *device,
		uint8_t *device_id);

#endif


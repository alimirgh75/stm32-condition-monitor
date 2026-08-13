

#include "ism330dhcx.h"


//this is only here cause it's a private detail
#define ISM330DHCX_WHO_AM_I_REG     0x0FU

static ism330dhcx_status_t ism330dhcx_read_register(
    const ism330dhcx_t *device,
    uint8_t register_address,
    uint8_t *data,
    uint16_t length);


ism330dhcx_status_t ism330dhcx_init(
    ism330dhcx_t *device,
    I2C_HandleTypeDef *i2c,
    uint8_t address_7bit,
    uint32_t timeout_ms)
{
	if((device == NULL) || (i2c == NULL)){

		return ISM330DHCX_INVALID_ARGUMENT;
	}

	device->i2c = i2c;
	device->address = ((uint16_t)address_7bit << 1U);
	device->timeout_ms = timeout_ms;

	return ISM330DHCX_OK;
}





ism330dhcx_status_t ism330dhcx_read_device_id(
    const ism330dhcx_t *device,
    uint8_t *device_id)
{
	if(device_id == NULL)
	{
		return ISM330DHCX_INVALID_ARGUMENT;
	}


	const ism330dhcx_status_t status =
	    ism330dhcx_read_register(
	        device,
	        ISM330DHCX_WHO_AM_I_REG,
	        device_id,
	        sizeof(*device_id));

	if (status != ISM330DHCX_OK)
	{
	    return status;
	}

	return ISM330DHCX_OK;
}


static ism330dhcx_status_t ism330dhcx_read_register(
    const ism330dhcx_t *device,
    uint8_t register_address,
    uint8_t *data,
    uint16_t length)
{
    if ((device == NULL) ||
        (device->i2c == NULL) ||
        (data == NULL) ||
        (length == 0U))
    {
        return ISM330DHCX_INVALID_ARGUMENT;
    }

	const HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Read(
			device->i2c,
			device->address,
			register_address,
			I2C_MEMADD_SIZE_8BIT,
			data,
			length,
			device->timeout_ms);

    if (hal_status != HAL_OK)
    {
        return ISM330DHCX_ERROR;
    }

    return ISM330DHCX_OK;

}


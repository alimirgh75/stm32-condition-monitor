

#include "ism330dhcx.h"


//this is only here cause it's a private detail
#define ISM330DHCX_WHO_AM_I_REG     			0x0FU
#define ISM330DHCX_CTRL3_C_REG 					0x12U
#define ISM330DHCX_CTRL3_C_RESET_MASK 			0x01U
#define ISM330DHCX_CTRL3_C_BDU_MASK 			0x40U
#define ISM330DHCX_CTRL3_C_IF_INC_MASK 			0x04U
#define ISM330DHCX_RESET_TIMEOUT_MS				0x100U


static ism330dhcx_status_t ism330dhcx_read_register(
    const ism330dhcx_t *device,
    uint8_t register_address,
    uint8_t *data,
    uint16_t length);

static ism330dhcx_status_t ism330dhcx_write_register(
		const ism330dhcx_t *device,
	    uint8_t register_address,
	    const uint8_t *data,
	    uint16_t length);

static ism330dhcx_status_t ism330dhcx_update_bits(
		const ism330dhcx_t *device,
	    uint8_t register_address,
		uint8_t mask,
	    uint8_t req_data);


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


ism330dhcx_status_t ism330dhcx_reset(const ism330dhcx_t *device)
{


	const uint8_t req_val = ISM330DHCX_CTRL3_C_RESET_MASK;

	const ism330dhcx_status_t status =  ism330dhcx_update_bits(
			device,
		    ISM330DHCX_CTRL3_C_REG,
			ISM330DHCX_CTRL3_C_RESET_MASK,
		    req_val);

	if (status != ISM330DHCX_OK)
	{
	    return status;
	}


	const uint32_t start_time_ms = HAL_GetTick();


	while((HAL_GetTick() - start_time_ms) < ISM330DHCX_RESET_TIMEOUT_MS)
	{
		uint8_t ctrl3_c = 0U;

		const ism330dhcx_status_t read_status = ism330dhcx_read_register(device, ISM330DHCX_CTRL3_C_REG, &ctrl3_c, sizeof(ctrl3_c));

		if (read_status != ISM330DHCX_OK){

			return read_status;
		}

		if((ctrl3_c & ISM330DHCX_CTRL3_C_RESET_MASK ) == 0)
		{
			return ISM330DHCX_OK;
		}
	}

	return ISM330DHCX_TIMEOUT;
}


ism330dhcx_status_t ism330dhcx_configure_interface(const ism330dhcx_t *device, const ism330dhcx_interface_config_t *config)
{
	if (config == NULL)
	{
		return ISM330DHCX_INVALID_ARGUMENT;
	}


	const uint8_t mask  = ISM330DHCX_CTRL3_C_BDU_MASK | ISM330DHCX_CTRL3_C_IF_INC_MASK;

	uint8_t req_value = 0U;

	if(config->block_data_update)
	{
		req_value |=ISM330DHCX_CTRL3_C_BDU_MASK;
	}
	if(config->auto_increment)
	{
		req_value |=ISM330DHCX_CTRL3_C_IF_INC_MASK;
	}

	return ism330dhcx_update_bits(
			device,
		    ISM330DHCX_CTRL3_C_REG,
			mask,
			req_value);

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


static ism330dhcx_status_t ism330dhcx_write_register(
    const ism330dhcx_t *device,
    uint8_t register_address,
    const uint8_t *data,
    uint16_t length)
{
    if ((device == NULL) ||
        (device->i2c == NULL) ||
        (data == NULL) ||
        (length == 0U))
    {
        return ISM330DHCX_INVALID_ARGUMENT;
    }

	const HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Write(
			device->i2c,
			device->address,
			register_address,
			I2C_MEMADD_SIZE_8BIT,
			(uint8_t *)data,
			length,
			device->timeout_ms);

    if (hal_status != HAL_OK)
    {
        return ISM330DHCX_ERROR;
    }

    return ISM330DHCX_OK;

}

static ism330dhcx_status_t ism330dhcx_update_bits(
		const ism330dhcx_t *device,
	    uint8_t register_address,
		uint8_t mask,
	    uint8_t req_data)
{
    if ((device == NULL) ||
        (device->i2c == NULL))
    {
        return ISM330DHCX_INVALID_ARGUMENT;
    }

    uint8_t old_byte = 0U;

    const ism330dhcx_status_t read_status = ism330dhcx_read_register(device, register_address, &old_byte, sizeof(old_byte));

	if (read_status != ISM330DHCX_OK)
	{
	    return read_status;
	}


    const uint8_t new_byte = (uint8_t) (old_byte & (uint8_t)~mask) | (req_data & mask) ;

    /* Avoid an unnecessary I2C transaction. */
    if (new_byte == old_byte)
    {
        return ISM330DHCX_OK;
    }


    const ism330dhcx_status_t write_status = ism330dhcx_write_register(device, register_address, &new_byte, sizeof(new_byte));

	if (write_status != ISM330DHCX_OK)
	{
	    return write_status;
	}

	return ISM330DHCX_OK;
}

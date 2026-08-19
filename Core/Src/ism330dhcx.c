

#include "ism330dhcx.h"
#include "ism330dhcx_registers.h"





#define ISM330DHCX_RESET_TIMEOUT_MS				0x100U

//Private function declarations
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


static ism330dhcx_status_t ism330dhcx_encode_lookup(
    const uint8_t *lookup_table,
    uint32_t table_length,
    uint32_t selected_value,
    uint8_t *register_value);

static ism330dhcx_status_t ism330dhcx_encode_accel_odr(
    ism330dhcx_accel_odr_t odr,
    uint8_t *register_value);


static ism330dhcx_status_t ism330dhcx_encode_accel_range(
	ism330dhcx_accel_range_t range,
    uint8_t *register_value);


static ism330dhcx_status_t ism330dhcx_encode_gyro_odr(
    ism330dhcx_gyro_odr_t odr,
    uint8_t *register_value);

static ism330dhcx_status_t ism330dhcx_encode_gyro_range(
	ism330dhcx_gyro_range_t range,
    uint8_t *register_value);

static ism330dhcx_status_t ism330dhcx_encode_performance_mode(
    ism330dhcx_performance_mode_t mode,
    uint8_t mode_mask,
    uint8_t *register_value);


static const uint8_t accel_odr_register_values
    [ISM330DHCX_ACCEL_ODR_COUNT] =
{
    [ISM330DHCX_ACCEL_ODR_PWR_DOWN]   = 0x00U,
    [ISM330DHCX_ACCEL_ODR_12_5_HZ]    = 0x10U,
    [ISM330DHCX_ACCEL_ODR_26_HZ]      = 0x20U,
    [ISM330DHCX_ACCEL_ODR_52_HZ]      = 0x30U,
    [ISM330DHCX_ACCEL_ODR_104_HZ]     = 0x40U,
    [ISM330DHCX_ACCEL_ODR_208_HZ]     = 0x50U,
    [ISM330DHCX_ACCEL_ODR_416_HZ]     = 0x60U,
    [ISM330DHCX_ACCEL_ODR_833_HZ]     = 0x70U,
    [ISM330DHCX_ACCEL_ODR_1660_HZ]    = 0x80U,
    [ISM330DHCX_ACCEL_ODR_3330_HZ]    = 0x90U,
    [ISM330DHCX_ACCEL_ODR_6660_HZ]    = 0xA0U
};

static const uint8_t accel_range_register_values
    [ISM330DHCX_ACCEL_RANGE_COUNT] =
{
    [ISM330DHCX_ACCEL_RANGE_2G]  = 0x00U,
    [ISM330DHCX_ACCEL_RANGE_4G]  = 0x08U,
    [ISM330DHCX_ACCEL_RANGE_8G]  = 0x0CU,
    [ISM330DHCX_ACCEL_RANGE_16G] = 0x04U
};

static const uint8_t gyro_odr_register_values
    [ISM330DHCX_GYRO_ODR_COUNT] =
{
    [ISM330DHCX_GYRO_ODR_PWR_DOWN] 	 = 0x00U,
    [ISM330DHCX_GYRO_ODR_12_5_HZ]    = 0x10U,
    [ISM330DHCX_GYRO_ODR_26_HZ]      = 0x20U,
    [ISM330DHCX_GYRO_ODR_52_HZ]      = 0x30U,
    [ISM330DHCX_GYRO_ODR_104_HZ]     = 0x40U,
    [ISM330DHCX_GYRO_ODR_208_HZ]     = 0x50U,
    [ISM330DHCX_GYRO_ODR_416_HZ]     = 0x60U,
    [ISM330DHCX_GYRO_ODR_833_HZ]     = 0x70U,
    [ISM330DHCX_GYRO_ODR_1660_HZ]    = 0x80U,
    [ISM330DHCX_GYRO_ODR_3330_HZ]    = 0x90U,
    [ISM330DHCX_GYRO_ODR_6660_HZ]    = 0xA0U
};

static const uint8_t gyro_range_register_values
    [ISM330DHCX_GYRO_DPS_COUNT] =
{
    [ISM330DHCX_GYRO_125_DPS]  = 0x02U,
    [ISM330DHCX_GYRO_250_DPS]  = 0x00U,
    [ISM330DHCX_GYRO_500_DPS]  = 0x04U,
    [ISM330DHCX_GYRO_1000_DPS] = 0x08U,
    [ISM330DHCX_GYRO_2000_DPS] = 0x0CU,
    [ISM330DHCX_GYRO_4000_DPS] = 0x01U
};




//Public functions
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


ism330dhcx_status_t ism330dhcx_configure_sensor(const ism330dhcx_t *device, const ism330dhcx_sensor_config_t *config)
{
    if ((device == NULL) ||
        (device->i2c == NULL) ||
        (config == NULL))
    {
        return ISM330DHCX_INVALID_ARGUMENT;
    }

	uint8_t accel_odr = 0U;
	uint8_t accel_range = 0U;
	uint8_t accel_performance = 0U;
	uint8_t gyro_odr = 0U;
	uint8_t gyro_range = 0U;
	uint8_t gyro_perfomance = 0U;

	//Accel config setting
	ism330dhcx_status_t accel_odr_status = ism330dhcx_encode_accel_odr(
			config->accel_odr,
			&accel_odr);

	if (accel_odr_status != ISM330DHCX_OK)
	{
	    return accel_odr_status;
	}

	ism330dhcx_status_t accel_range_status = ism330dhcx_encode_accel_range(
			config->accel_range,
			&accel_range);

	if (accel_range_status != ISM330DHCX_OK)
	{
	    return accel_range_status;
	}


	ism330dhcx_status_t accel_performance_mode_status = ism330dhcx_encode_performance_mode(
	    config->accel_mode,
		ISM330DHCX_CTRL6_C_XL_HM_MODE_MASK,
	    &accel_performance);

	if (accel_performance_mode_status != ISM330DHCX_OK)
	{
	    return accel_performance_mode_status;
	}


    const uint8_t accel_register_value =
        (uint8_t)(accel_odr | accel_range);


	//Gyro Config setting

	ism330dhcx_status_t gyro_odr_status = ism330dhcx_encode_gyro_odr(
			config->gyro_odr,
			&gyro_odr);

	if (gyro_odr_status != ISM330DHCX_OK)
	{
	    return gyro_odr_status;
	}

	ism330dhcx_status_t gyro_range_status = ism330dhcx_encode_gyro_range(
			config->gyro_range,
			&gyro_range);


	if (gyro_range_status != ISM330DHCX_OK)
	{
	    return gyro_range_status;
	}

    const uint8_t gyro_register_value =
        (uint8_t)(gyro_odr | gyro_range);

	ism330dhcx_status_t gyro_performance_mode_status = ism330dhcx_encode_performance_mode(
	    config->gyro_mode,
		ISM330DHCX_CTRL7_C_G_HM_MODE_MASK,
	    &gyro_perfomance);

	if (gyro_performance_mode_status != ISM330DHCX_OK)
	{
	    return gyro_performance_mode_status;
	}

	//Update bits

	const ism330dhcx_status_t accel_perf_status = ism330dhcx_update_bits(
		    device,
			ISM330DHCX_CTRL6_C_REG,
			ISM330DHCX_CTRL6_C_XL_HM_MODE_MASK,
			accel_performance);

	if (accel_perf_status != ISM330DHCX_OK)
	{
	    return accel_perf_status;
	}

	const ism330dhcx_status_t gyro_perf_status = ism330dhcx_update_bits(
		    device,
			ISM330DHCX_CTRL7_C_REG,
			ISM330DHCX_CTRL7_C_G_HM_MODE_MASK,
			gyro_perfomance);

	if (gyro_perf_status != ISM330DHCX_OK)
	{
	    return gyro_perf_status;
	}

	const ism330dhcx_status_t accel_status = ism330dhcx_update_bits(
	    device,
	    ISM330DHCX_CTRL1_XL_REG,
		ISM330DHCX_ACCEL_CONFIG_MASK,
		accel_register_value);

	if (accel_status != ISM330DHCX_OK)
	{
	    return accel_status;
	}

	return ism330dhcx_update_bits(
	    device,
		ISM330DHCX_CTRL2_G_REG,
		ISM330DHCX_GYRO_CONFIG_MASK,
		gyro_register_value);

}



//Private functions
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


static ism330dhcx_status_t ism330dhcx_encode_lookup(
    const uint8_t *lookup_table,
    uint32_t table_length,
    uint32_t selected_value,
    uint8_t *register_value)
{
    if ((lookup_table == NULL) ||
        (register_value == NULL) ||
        (selected_value >= table_length))
    {
        return ISM330DHCX_INVALID_ARGUMENT;
    }

    *register_value = lookup_table[selected_value];

    return ISM330DHCX_OK;
}


static ism330dhcx_status_t ism330dhcx_encode_accel_odr(
    ism330dhcx_accel_odr_t odr,
    uint8_t *register_value)
{
    return ism330dhcx_encode_lookup(
        accel_odr_register_values,
        (uint32_t)ISM330DHCX_ACCEL_ODR_COUNT,
        (uint32_t)odr,
        register_value);
}

static ism330dhcx_status_t ism330dhcx_encode_accel_range(
    ism330dhcx_accel_range_t range,
    uint8_t *register_value)
{
    return ism330dhcx_encode_lookup(
        accel_range_register_values,
        (uint32_t)ISM330DHCX_ACCEL_RANGE_COUNT,
        (uint32_t)range,
        register_value);
}

static ism330dhcx_status_t ism330dhcx_encode_gyro_odr(
    ism330dhcx_gyro_odr_t odr,
    uint8_t *register_value)
{
    return ism330dhcx_encode_lookup(
        gyro_odr_register_values,
        (uint32_t)ISM330DHCX_GYRO_ODR_COUNT,
        (uint32_t)odr,
        register_value);
}

static ism330dhcx_status_t ism330dhcx_encode_gyro_range(
    ism330dhcx_gyro_range_t range,
    uint8_t *register_value)
{
    return ism330dhcx_encode_lookup(
        gyro_range_register_values,
        (uint32_t)ISM330DHCX_GYRO_DPS_COUNT,
        (uint32_t)range,
        register_value);
}

static ism330dhcx_status_t ism330dhcx_encode_performance_mode(
    ism330dhcx_performance_mode_t mode,
    uint8_t mode_mask,
    uint8_t *register_value)
{
    if (register_value == NULL)
    {
        return ISM330DHCX_INVALID_ARGUMENT;
    }

    switch(mode)
    {
    	case ISM330DHCX_MODE_HIGH_PERFORMANCE:
    		*register_value = 0U;
    		break;

    	case ISM330DHCX_MODE_LOW_POWER_NORMAL:
    		*register_value = mode_mask;
    		break;

    	default :
    		return ISM330DHCX_INVALID_ARGUMENT;

    }

    return ISM330DHCX_OK;
}



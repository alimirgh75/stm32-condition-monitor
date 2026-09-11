

#include "ism330dhcx.h"
#include "ism330dhcx_registers.h"





#define ISM330DHCX_RESET_TIMEOUT_MS				100U
static const float STANDARD_GRAVITY_MPS2_PER_G = 9.80665f;


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


static int16_t ism330dhcx_decode_int16_le(
    const uint8_t *bytes);

static void ism330dhcx_convert_raw(
    const ism330dhcx_raw_sample_t *raw_sample,
    float accel_scale_g_per_lsb,
    float gyro_scale_dps_per_lsb,
    ism330dhcx_sample_t *sample);

static ism330dhcx_status_t ism330dhcx_validate_sensor_config(
    const ism330dhcx_sensor_config_t *config);

static const uint8_t accel_odr_register_values
    [ISM330DHCX_ACCEL_ODR_COUNT] =
{
    [ISM330DHCX_ACCEL_ODR_PWR_DOWN]   = 0x00U,
	[ISM330DHCX_ACCEL_ODR_1_6_HZ]     = 0xB0U,
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


/* Acceleration sensitivity in g/LSB. */
static const float accel_sensitivity_g_per_lsb
    [ISM330DHCX_ACCEL_RANGE_COUNT] =
{
    [ISM330DHCX_ACCEL_RANGE_2G]  = 0.000061f,
    [ISM330DHCX_ACCEL_RANGE_4G]  = 0.000122f,
    [ISM330DHCX_ACCEL_RANGE_8G]  = 0.000244f,
    [ISM330DHCX_ACCEL_RANGE_16G] = 0.000488f
};

/* Angular-rate sensitivity in dps/LSB. */
static const float gyro_sensitivity_dps_per_lsb
    [ISM330DHCX_GYRO_DPS_COUNT] =
{
    [ISM330DHCX_GYRO_125_DPS]  = 0.004375f,
    [ISM330DHCX_GYRO_250_DPS]  = 0.008750f,
    [ISM330DHCX_GYRO_500_DPS]  = 0.017500f,
    [ISM330DHCX_GYRO_1000_DPS] = 0.035000f,
    [ISM330DHCX_GYRO_2000_DPS] = 0.070000f,
    [ISM330DHCX_GYRO_4000_DPS] = 0.140000f
};



//Public functions
ism330dhcx_status_t ism330dhcx_init(
    ism330dhcx_t *device,
    I2C_HandleTypeDef *i2c,
    uint8_t address_7bit,
    uint32_t timeout_ms)
{
	if((device == NULL) ||
		    (i2c == NULL) ||
		    ((address_7bit != ISM330DHCX_I2C_ADDRESS_LOW_7BIT) &&
		     (address_7bit != ISM330DHCX_I2C_ADDRESS_HIGH_7BIT)) ||
		    (timeout_ms == 0U)){

		return ISM330DHCX_INVALID_ARGUMENT;
	}

	device->i2c = i2c;
	device->address = ((uint16_t)address_7bit << 1U);
	device->timeout_ms = timeout_ms;
	device->sensor_configured = false;

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

	if (*device_id != ISM330DHCX_EXPECTED_ID)
	{
	    return ISM330DHCX_WRONG_DEVICE;
	}

	return ISM330DHCX_OK;
}


ism330dhcx_status_t ism330dhcx_reset(ism330dhcx_t *device)
{

    if ((device == NULL) || (device->i2c == NULL))
    {
        return ISM330DHCX_INVALID_ARGUMENT;
    }


    device->sensor_configured = false;

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


ism330dhcx_status_t ism330dhcx_configure_sensor(ism330dhcx_t *device, const ism330dhcx_sensor_config_t *config)
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


	const ism330dhcx_status_t config_valid_status = ism330dhcx_validate_sensor_config(config);


	if (config_valid_status != ISM330DHCX_OK)
	{
	    return config_valid_status;
	}

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
	device->sensor_configured = false;

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

	const ism330dhcx_status_t gyro_status = ism330dhcx_update_bits(
		    device,
			ISM330DHCX_CTRL2_G_REG,
			ISM330DHCX_GYRO_CONFIG_MASK,
			gyro_register_value);

	if (gyro_status != ISM330DHCX_OK)
	{
	    return gyro_status;
	}



	device->sensor_config = *config;
	device->sensor_configured = true;


	return ISM330DHCX_OK;

}



ism330dhcx_status_t ism330dhcx_read_raw_sample(
    const ism330dhcx_t *device,
    ism330dhcx_raw_sample_t *sample)
{
    if ((device == NULL) || (sample == NULL))
    {
        return ISM330DHCX_INVALID_ARGUMENT;
    }

	uint8_t raw_data[ISM330DHCX_SAMPLE_BYTE_COUNT] = {0U};

	ism330dhcx_status_t read_data_status =  ism330dhcx_read_register(
	    device,
		ISM330DHCX_REG_OUTX_L_G,
		raw_data,
		(uint16_t)sizeof(raw_data));

	if(read_data_status != ISM330DHCX_OK)
	{
		 return read_data_status;
	}

	const ism330dhcx_status_t decode_status =
	    ism330dhcx_decode_raw_sample(
	        raw_data,
	        (uint16_t)sizeof(raw_data),
	        sample);

	if (decode_status != ISM330DHCX_OK)
	{
	    return decode_status;
	}

	return ISM330DHCX_OK;

}


ism330dhcx_status_t ism330dhcx_convert_raw_sample(
    const ism330dhcx_t *device,
    const ism330dhcx_raw_sample_t *raw_sample,
    ism330dhcx_sample_t *sample)
{
    if ((device == NULL) ||
        (raw_sample == NULL) ||
        (sample == NULL))
    {
        return ISM330DHCX_INVALID_ARGUMENT;
    }

    if (!device->sensor_configured)
    {
        return ISM330DHCX_NOT_CONFIGURED;
    }

    if ((device->sensor_config.accel_range >=
         ISM330DHCX_ACCEL_RANGE_COUNT) ||
        (device->sensor_config.gyro_range >=
        		ISM330DHCX_GYRO_DPS_COUNT))
    {
        return ISM330DHCX_INVALID_ARGUMENT;
    }

    const float accel_scale_g_per_lsb =
    		accel_sensitivity_g_per_lsb[
            device->sensor_config.accel_range];

    const float gyro_scale_dps_per_lsb =
    		gyro_sensitivity_dps_per_lsb[
            device->sensor_config.gyro_range];

    ism330dhcx_convert_raw(
        raw_sample,
        accel_scale_g_per_lsb,
        gyro_scale_dps_per_lsb,
        sample);

    return ISM330DHCX_OK;

}


ism330dhcx_status_t ism330dhcx_configure_int1(const ism330dhcx_t *device, const ism330dhcx_interrupt1_output_config_t *config)
{
	if ((device == NULL) || (config == NULL))
	{
	    return ISM330DHCX_INVALID_ARGUMENT;
	}


    /* Configure latched/pulsed DRDY behavior first. */
    const uint8_t pulsed_value =
        config->pulsed_drdy ? ISM330DHCX_DRDY_PULSED_MASK : 0U;

    const ism330dhcx_status_t pulsed_status =
        ism330dhcx_update_bits(
            device,
            ISM330DHCX_COUNTER_BDR_REG1_REG,
            ISM330DHCX_DRDY_PULSED_MASK,
            pulsed_value);

    if (pulsed_status != ISM330DHCX_OK)
    {
        return pulsed_status;
    }


    /* Configure which DRDY sources are routed to INT1. */
    uint8_t req_value = 0U;

    if (config->accelerometer_drdy)
    {
        req_value |= ISM330DHCX_INT1_DRDY_XL_MASK;
    }

    if (config->gyro_drdy)
    {
        req_value |= ISM330DHCX_INT1_DRDY_G_MASK;
    }

    return ism330dhcx_update_bits(
        device,
        ISM330DHCX_INT1_CTRL_REG,
        ISM330DHCX_INT1_DRDY_MASK,
        req_value);

}


ism330dhcx_status_t ism330dhcx_start_sample_read_dma(
    const ism330dhcx_t *device,
    uint8_t *buffer,
    uint16_t length)
{

    if ((device == NULL) ||
        (device->i2c == NULL) ||
        (buffer == NULL) ||
        (length != ISM330DHCX_SAMPLE_BYTE_COUNT))
    {
        return ISM330DHCX_INVALID_ARGUMENT;
    }



	HAL_StatusTypeDef dma_status = HAL_I2C_Mem_Read_DMA(device->i2c,
			device->address,
			ISM330DHCX_REG_OUTX_L_G,
			I2C_MEMADD_SIZE_8BIT,
			buffer, length);


    if (dma_status == HAL_OK)
    {
        return ISM330DHCX_OK;
    }

    return ISM330DHCX_ERROR;
}


ism330dhcx_status_t ism330dhcx_decode_raw_sample(
    const uint8_t *buffer,
    uint16_t length,
    ism330dhcx_raw_sample_t *raw_sample)
{
    if ((buffer == NULL) ||
        (raw_sample == NULL) ||
        (length != ISM330DHCX_SAMPLE_BYTE_COUNT))
    {
        return ISM330DHCX_INVALID_ARGUMENT;
    }

    raw_sample->gyro.x  = ism330dhcx_decode_int16_le(&buffer[0]);
    raw_sample->gyro.y  = ism330dhcx_decode_int16_le(&buffer[2]);
    raw_sample->gyro.z  = ism330dhcx_decode_int16_le(&buffer[4]);

    raw_sample->accel.x = ism330dhcx_decode_int16_le(&buffer[6]);
    raw_sample->accel.y = ism330dhcx_decode_int16_le(&buffer[8]);
    raw_sample->accel.z = ism330dhcx_decode_int16_le(&buffer[10]);

    return ISM330DHCX_OK;
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

	if (hal_status == HAL_TIMEOUT)
	{
	    return ISM330DHCX_TIMEOUT;
	}

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

	if (hal_status == HAL_TIMEOUT)
	{
	    return ISM330DHCX_TIMEOUT;
	}

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


    const uint8_t new_byte = (uint8_t) ((old_byte & (uint8_t)~mask) | (req_data & mask)) ;

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

static int16_t ism330dhcx_decode_int16_le(
    const uint8_t *bytes)
{
    return (int16_t)(
        (uint16_t)bytes[0] |
        ((uint16_t)bytes[1] << 8U));
}

static void ism330dhcx_convert_raw(
    const ism330dhcx_raw_sample_t *raw_sample,
    float accel_scale_g_per_lsb,
    float gyro_scale_dps_per_lsb,
    ism330dhcx_sample_t *sample)
{

	sample->acceleration_mps2.x = (float) raw_sample->accel.x * accel_scale_g_per_lsb * STANDARD_GRAVITY_MPS2_PER_G;

	sample->acceleration_mps2.y = (float) raw_sample->accel.y * accel_scale_g_per_lsb * STANDARD_GRAVITY_MPS2_PER_G;

	sample->acceleration_mps2.z = (float) raw_sample->accel.z * accel_scale_g_per_lsb * STANDARD_GRAVITY_MPS2_PER_G;

	sample->angular_rate_dps.x = (float) raw_sample->gyro.x * gyro_scale_dps_per_lsb;

	sample->angular_rate_dps.y = (float) raw_sample->gyro.y * gyro_scale_dps_per_lsb;

	sample->angular_rate_dps.z = (float) raw_sample->gyro.z * gyro_scale_dps_per_lsb;
}



static ism330dhcx_status_t ism330dhcx_validate_sensor_config(
    const ism330dhcx_sensor_config_t *config)
{
    if (config == NULL)
    {
        return ISM330DHCX_INVALID_ARGUMENT;
    }

    /* Validate the individual mode and range enum values. */
    if (((uint32_t)config->accel_mode >=
         (uint32_t)ISM330DHCX_MODE_COUNT) ||
        ((uint32_t)config->gyro_mode >=
         (uint32_t)ISM330DHCX_MODE_COUNT) ||
        ((uint32_t)config->accel_range >=
         (uint32_t)ISM330DHCX_ACCEL_RANGE_COUNT) ||
        ((uint32_t)config->gyro_range >=
         (uint32_t)ISM330DHCX_GYRO_DPS_COUNT))
    {
        return ISM330DHCX_INVALID_ARGUMENT;
    }

    /*
     * Accelerometer mode/ODR combinations.
     *
     * At 416 Hz and above, the device operates in high-performance
     * mode even if XL_HM_MODE is set. Reject that combination because
     * the requested low-power/normal mode cannot actually be provided.
     */
    switch (config->accel_odr)
    {
        case ISM330DHCX_ACCEL_ODR_PWR_DOWN:
            break;

        case ISM330DHCX_ACCEL_ODR_1_6_HZ:
            if (config->accel_mode !=
                ISM330DHCX_MODE_LOW_POWER_NORMAL)
            {
                return ISM330DHCX_INVALID_ARGUMENT;
            }
            break;

        case ISM330DHCX_ACCEL_ODR_12_5_HZ:
        case ISM330DHCX_ACCEL_ODR_26_HZ:
        case ISM330DHCX_ACCEL_ODR_52_HZ:
        case ISM330DHCX_ACCEL_ODR_104_HZ:
        case ISM330DHCX_ACCEL_ODR_208_HZ:
            break;

        case ISM330DHCX_ACCEL_ODR_416_HZ:
        case ISM330DHCX_ACCEL_ODR_833_HZ:
        case ISM330DHCX_ACCEL_ODR_1660_HZ:
        case ISM330DHCX_ACCEL_ODR_3330_HZ:
        case ISM330DHCX_ACCEL_ODR_6660_HZ:
            if (config->accel_mode !=
                ISM330DHCX_MODE_HIGH_PERFORMANCE)
            {
                return ISM330DHCX_INVALID_ARGUMENT;
            }
            break;

        default:
            return ISM330DHCX_INVALID_ARGUMENT;
    }

    /*
     * Gyroscope mode/ODR combinations.
     *
     * Low-power is available at 12.5–52 Hz.
     * Normal mode is available at 104–208 Hz.
     * Higher ODRs operate in high-performance mode.
     */
    switch (config->gyro_odr)
    {
        case ISM330DHCX_GYRO_ODR_PWR_DOWN:
        case ISM330DHCX_GYRO_ODR_12_5_HZ:
        case ISM330DHCX_GYRO_ODR_26_HZ:
        case ISM330DHCX_GYRO_ODR_52_HZ:
        case ISM330DHCX_GYRO_ODR_104_HZ:
        case ISM330DHCX_GYRO_ODR_208_HZ:
            break;

        case ISM330DHCX_GYRO_ODR_416_HZ:
        case ISM330DHCX_GYRO_ODR_833_HZ:
        case ISM330DHCX_GYRO_ODR_1660_HZ:
        case ISM330DHCX_GYRO_ODR_3330_HZ:
        case ISM330DHCX_GYRO_ODR_6660_HZ:
            if (config->gyro_mode !=
                ISM330DHCX_MODE_HIGH_PERFORMANCE)
            {
                return ISM330DHCX_INVALID_ARGUMENT;
            }
            break;

        default:
            return ISM330DHCX_INVALID_ARGUMENT;
    }

    return ISM330DHCX_OK;
}


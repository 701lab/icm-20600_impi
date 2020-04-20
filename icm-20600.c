#include "icm-20600.h"

// ******************* Function ******************* //
/*
	@brief
 */
// ************************************************ //
uint32_t icm_20600_basic_init(icm_20600_instance *icm_instance, uint32_t enable_temperature_sensor)
{
// SPI based implementation
#ifdef ICM_THROUGH_SPI

	icm_instance->cs_low();

	// Check if connection is established
	icm_instance->send_one_byte(ICM_WHO_AM_I | ICM_READ_REGISTERS);
	if ( icm_instance->send_one_byte(0xff) == 0 )
	{
		icm_instance->cs_high();
		return ICM_20600_DEVICE_IS_NOT_CONNECTED;
	}

	// Reset SPI connection
	icm_instance->cs_high();
	icm_instance->cs_low();

	icm_instance->send_one_byte(ICM_PWR_MGMT_1 | ICM_WRITE_REGISTERS);
	if ( enable_temperature_sensor )
	{
		// Wakes up. Don't disable temperature sensor
		icm_instance->send_one_byte(ICM_PWR_MGMT_1_CLKSEL_AUTO);
	}
	else
	{
		// Wakes up. Disables temperature sensor
		icm_instance->send_one_byte(ICM_PWR_MGMT_1_TEMP_DISABLE | ICM_PWR_MGMT_1_CLKSEL_AUTO);
	}

	// Reset SPI connection
	icm_instance->cs_high();
	icm_instance->cs_low();

	// Disable I2C interface
	icm_instance->send_one_byte(ICM_I2C_IF | ICM_WRITE_REGISTERS);
	icm_instance->send_one_byte(ICM_I2C_IF_DISABLE);

	icm_instance->cs_high();

	icm_instance->device_was_initialized = 1;
	return 0;

#endif /* ICM_THROUGH_SPI */

// I2C based implementation
#ifdef ICM_THROUGH_I2C

	#error I2C is not implemented yet

#endif /* ICM_THROUGH_I2C */
}

// ******************* Function ******************* //
/*
	@brief
 */
// ************************************************ //

uint32_t icm_20600_get_sensors_data(const icm_20600_instance *icm_instance, int16_t *data_storage_array, uint32_t add_temperature_data)
{
	// Code that doesn't depend on library type

	// Check if device was initialized
	if (icm_instance->device_was_initialized == 0)
	{
		return ICM_20600_INSTANCE_WAS_NOT_INITIALIZED;
	}

// SPI based implementation
#ifdef ICM_THROUGH_SPI

	icm_instance->cs_low();

	// Start sequential reading of sensor data
	icm_instance->send_one_byte(ICM_ACCEL_XOUT_H | ICM_READ_REGISTERS);

	// Get accelerometer data
	data_storage_array[icm_accelerometer_x] = icm_instance->send_one_byte(0xFF)<<8 | icm_instance->send_one_byte(0xFF);
	data_storage_array[icm_accelerometer_y] = icm_instance->send_one_byte(0xFF)<<8 | icm_instance->send_one_byte(0xFF);
	data_storage_array[icm_accelerometer_z] = icm_instance->send_one_byte(0xFF)<<8 | icm_instance->send_one_byte(0xFF);

	if (add_temperature_data)
	{
		// Add temperature sensor data to array
		data_storage_array[icm_temperature] = icm_instance->send_one_byte(0xFF)<<8 | icm_instance->send_one_byte(0xFF);
	}
	else
	{
		// Skip temperature sensor data
		icm_instance->send_one_byte(0xFF);
		icm_instance->send_one_byte(0xFF);
	}

	// Get gyroscope data
	data_storage_array[icm_gyroscope_x] = icm_instance->send_one_byte(0xFF)<<8 | icm_instance->send_one_byte(0xFF);
	data_storage_array[icm_gyroscope_y] = icm_instance->send_one_byte(0xFF)<<8 | icm_instance->send_one_byte(0xFF);
	data_storage_array[icm_gyroscope_z] = icm_instance->send_one_byte(0xFF)<<8 | icm_instance->send_one_byte(0xFF);

	icm_instance->cs_high();

	return 0;

#endif /* ICM_THROUGH_SPI */

// I2C based implementation
#ifdef ICM_THROUGH_I2C

	#error I2C is not implemented yet

#endif /* ICM_THROUGH_I2C */
}

// ******************* Function ******************* //
/*
	@brief
 */
// ************************************************ //
uint32_t icm_20600_check_if_alive(const icm_20600_instance *icm_instance)
{
//SPI implementation
#ifdef ICM_THROUGH_SPI

	icm_instance->cs_low();

	// Check if device is connected
	icm_instance->send_one_byte(ICM_WHO_AM_I | ICM_READ_REGISTERS);
	if ( icm_instance->send_one_byte(0xff) == 0 )
	{
		icm_instance->cs_high();
		return ICM_20600_INSTANCE_WAS_NOT_INITIALIZED;
	}

	icm_instance->cs_high();
	return 0;

#endif /* ICM_THROUGH_SPI */

#ifdef ICM_THROUGH_I2C

	#error I2C is not implemented yet

#endif /* ICM_THROUGH_I2C */
}

// ******************* Function ******************* //
/*
	@brief
 */
// ************************************************ //
uint32_t icm_20600_setup(const icm_20600_instance *icm_instance, const uint8_t gyro_desired_dps_scale, const uint8_t accel_desired_g_scale)
{
	// Code that doesn't depend on library type

	// Check if device was initialized
	if (icm_instance->device_was_initialized == 0)
	{
		return ICM_20600_INSTANCE_WAS_NOT_INITIALIZED;
	}

	uint8_t gyro_scale_value = gyro_desired_dps_scale;
	uint8_t accel_scale_value = accel_desired_g_scale;

	if(gyro_desired_dps_scale > icm_gyro_2000dps)
	{
		gyro_scale_value = icm_gyro_2000dps;
	}
	if(accel_desired_g_scale > icm_accel_16g)
	{
		accel_scale_value = icm_accel_16g;
	}

#ifdef ICM_THROUGH_SPI

	icm_instance->cs_low();

	icm_instance->send_one_byte(ICM_GYRO_CONFIG | ICM_WRITE_REGISTERS);
	icm_instance->send_one_byte(gyro_scale_value << ICM_GYRO_CONFIG_FS_SEL_pos);
	icm_instance->send_one_byte(accel_scale_value << ICM_ACCEL_CONFIG_1_FS_SEL_pos);

	icm_instance->cs_high();

	return 0;

#endif /* ICM_THROUGH_SPI */


#ifdef ICM_THROUGH_I2C

	#error I2C is not implemented yet

#endif /* ICM_THROUGH_I2C */
}



#include "icm-20600_impi.h"

// ******************* Function ******************* //
/*
	@brief
 */
// ************************************************ //
uint32_t icm_20600_basic_init(icm_20600_instance *icm_instance)
{
	// Interface independent code

	uint32_t mistake_code = 0;

	if(icm_instance->gyro_full_scale_setup > icm_gyro_2000dps)
	{
		icm_instance->gyro_full_scale_setup = icm_gyro_2000dps;
		mistake_code = ICM_20600_WRONG_GYROSCOPE_SCALE;
	}
	if( icm_instance->accel_full_scale_setup > icm_accel_16g)
	{
		icm_instance->accel_full_scale_setup = icm_accel_16g;
		mistake_code = ICM_20600_WRONG_ACCELEROMETER_SCALE;
	}

// SPI based implementation
#ifdef ICM_THROUGH_SPI

	// Reset SPI connection
	icm_instance->cs_high();	// Must be set high to start communication because this function should be called before any other.
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

	// Disable I2C interface
	icm_instance->send_one_byte(ICM_I2C_IF | ICM_WRITE_REGISTERS);
	icm_instance->send_one_byte(ICM_I2C_IF_DISABLE);

	// Reset SPI connection
	icm_instance->cs_high();
	icm_instance->cs_low();

	// Setup gyro and accel scales
	icm_instance->send_one_byte(ICM_GYRO_CONFIG | ICM_WRITE_REGISTERS);
	icm_instance->send_one_byte(icm_instance->gyro_full_scale_setup << ICM_GYRO_CONFIG_FS_SEL_pos);
	icm_instance->send_one_byte(icm_instance->accel_full_scale_setup << ICM_ACCEL_CONFIG_1_FS_SEL_pos);

	// Reset SPI connection
	icm_instance->cs_high();
	icm_instance->cs_low();

	icm_instance->send_one_byte(ICM_PWR_MGMT_1 | ICM_WRITE_REGISTERS);
	if ( icm_instance->enable_temperature_sensor )
	{
		// Wakes up. Don't disable temperature sensor
		icm_instance->send_one_byte(ICM_PWR_MGMT_1_CLKSEL_AUTO);
	}
	else
	{
		// Wakes up. Disables temperature sensor
		icm_instance->send_one_byte(ICM_PWR_MGMT_1_TEMP_DISABLE | ICM_PWR_MGMT_1_CLKSEL_AUTO);
	}

	icm_instance->cs_high();

	icm_instance->device_was_initialized = 1;
	return mistake_code;

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

uint32_t icm_20600_get_raw_data(icm_20600_instance *icm_instance, int16_t *data_storage_array)
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

	if (icm_instance->enable_temperature_sensor)	// If array is at least 7 int16_t fields long
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
	data_storage_array[icm_gyroscope_x] = (icm_instance->send_one_byte(0xFF)<<8 | icm_instance->send_one_byte(0xFF)) - icm_instance->gyro_calibration_coefficients[icm_x];
	data_storage_array[icm_gyroscope_y] = (icm_instance->send_one_byte(0xFF)<<8 | icm_instance->send_one_byte(0xFF)) - icm_instance->gyro_calibration_coefficients[icm_y];
	data_storage_array[icm_gyroscope_z] = (icm_instance->send_one_byte(0xFF)<<8 | icm_instance->send_one_byte(0xFF)) - icm_instance->gyro_calibration_coefficients[icm_z];

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
uint32_t icm_20600_get_proccesed_data(icm_20600_instance *icm_instance, float *data_storage_array)
{
	if (icm_instance->device_was_initialized == 0)
	{
		return ICM_20600_INSTANCE_WAS_NOT_INITIALIZED;
	}

	int16_t current_sencor_measurements[7];
	icm_20600_get_raw_data(icm_instance, current_sencor_measurements);

	float gyro_divider = 1.0f;

	switch ( icm_instance->gyro_full_scale_setup )
	{
	case icm_gyro_250dps:
		gyro_divider = 1.0f;
		break;
	case icm_gyro_500dps:
		gyro_divider = 2.0f;
		break;
	case icm_gyro_1000dps:
		gyro_divider = 4.0f;
		break;
	case icm_gyro_2000dps:
		gyro_divider = 8.0f;
		break;
	default:
		break;
	}

	float gyro_sensitivity = 131.072f/gyro_divider;
	float accel_sensitivity = (uint32_t)(16384 >> (icm_instance->accel_full_scale_setup));	// Will be the same value for the 2g setup and twice as big for every next one

	data_storage_array[icm_accelerometer_x] = (float)(current_sencor_measurements[icm_accelerometer_x]) / accel_sensitivity;
	data_storage_array[icm_accelerometer_y] = (float)(current_sencor_measurements[icm_accelerometer_y]) / accel_sensitivity;
	data_storage_array[icm_accelerometer_z] = (float)(current_sencor_measurements[icm_accelerometer_z]) / accel_sensitivity;

	data_storage_array[icm_gyroscope_x] = (float)(current_sencor_measurements[icm_gyroscope_x]) / gyro_sensitivity;
	data_storage_array[icm_gyroscope_y] = (float)(current_sencor_measurements[icm_gyroscope_y]) / gyro_sensitivity;
	data_storage_array[icm_gyroscope_z] = (float)(current_sencor_measurements[icm_gyroscope_z]) / gyro_sensitivity;

	if ( icm_instance->enable_temperature_sensor )
	{
		data_storage_array[icm_temperature] = (float)(current_sencor_measurements[icm_temperature]) / 340.0f + 36.53f;
	}

	return 0;
}



// ******************* Function ******************* //
/*
	@brief
 */
// ************************************************ //
uint32_t icm_20600_check_if_alive(icm_20600_instance *icm_instance)
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




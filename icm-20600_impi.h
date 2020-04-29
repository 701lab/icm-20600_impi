#ifndef ICM_20600_H_
#define ICM_20600_H_

#include "implementation.h"
#include "icm-20600_registers.h"
#include "icm-20600_mistakes.h"
#include "math.h"

typedef struct{

	// @brief Chip select high. Used in SPI communication to indicate the end of the transmission.
	//		Function must set MCU pin connected to the device CS pin logic low.
	const void (*cs_high)(void);

	// @brief Chip select low. Used in SPI communication to indicate start of the transmission.
	// 		Function must set MCU pin connected to the device CS pin logic high.
	const void (*cs_low)(void);

	// @brief Sends single byte through desired SPI.
	//		Function must send input byte with the SPI connected to the desired icm-20600 device in single-byte mode. Must return byte received by SPI during transmission.
	const uint8_t (*send_one_byte)(const uint8_t byte_to_be_sent);

	// Хранят предыдущее значение показаний гироскопа для трапециидального интегрирования
	float previous_gyro_x;
	float previous_gyro_y;
	float previous_gyro_z;

	float complementary_filter_coefficient;

	uint32_t gyro_full_scale_setup;

	uint32_t accel_full_scale_setup;

	// Flag that shows that device was already initialized. Needed for internal library error checking optimization. Should be set to 0 at initialization.
	uint32_t device_was_initialized;

	// Enables temperature sensor if not 0
	uint32_t enable_temperature_sensor;

	uint16_t gyro_calibration_coefficients[3];




} icm_20600;


// Будет использоваться для трапециидального интегрирования в гироскопе
//typedef struct
//{
//	float previous_gyro_x;
//	float previous_gyro_y;
//	float previous_gyro_z;
//} previous_gyro_values;

//****** ICM functions ******//

uint32_t icm_20600_basic_init(icm_20600 *icm_instance);

uint32_t icm_20600_get_raw_data(icm_20600 *icm_instance, int16_t *data_storage_array);

uint32_t icm_20600_get_proccesed_data(icm_20600 *icm_instance, float *data_storage_array);

// @brief send basic read message onto register with enable bit. If answer is 0 there is problems with connection to the device, or device didn't start.
uint32_t icm_20600_check_if_alive(icm_20600 * icm_instance);


// Должны будут в дальнейшем быть пересмотрены и по возможности улучшены
uint32_t icm_20600_calculate_all_angles(icm_20600 *icm_instance, float angles_storage[3], float integration_period);

// Вычисляет только нужный угол из трех. Может вызываться без вызова каких либо других функций, так как будет считывать значения датчика самостоятельно
uint32_t icm_20600_calculate_z_x_angle(icm_20600 *icm_instance, float *calculated_angle, float integration_period);
uint32_t icm_20600_calculate_y_z_angle(icm_20600 *icm_instance, float *calculated_angle, float integration_period);
uint32_t icm_20600_calculate_x_y_angle(icm_20600 *icm_instance, float *calculated_angle, float integration_period);



// ****** Note implemented yet
void icm_20600_registers_reset(icm_20600 *icm_instance);

void icm_20600_disable_gyro(icm_20600 *icm_instance);

void icm_20600_disable_accel(icm_20600 *icm_instance);

void icm_20600_disable_one_gyro_channel(icm_20600 *icm_instance, uint32_t number_of_channel_to_desable);

void icm_20600_disable_one_accel_channel(icm_20600 *icm_instance, uint32_t number_of_channel_to_desable);



//****** ICM enums ******//

// @brief Used for icm-20600 data pointing in arrays
enum icm_data_order
{
	icm_accelerometer_x = 0,
	icm_accelerometer_y = 1,
	icm_accelerometer_z = 2,
	icm_gyroscope_x = 3,
	icm_gyroscope_y = 4,
	icm_gyroscope_z = 5,
	icm_temperature = 6
};

enum icm_axis_order
{
	icm_x = 0,
	icm_y = 1,
	icm_z = 2
};

enum icm_angles_indexed
{
	z_x_angle = 0,
	y_z_angle = 0,
	x_y_angle = 0
};

// @brief Gyroscope configuration parameters
enum icm_gyro_config_params
{
	icm_gyro_250dps = 0,
	icm_gyro_500dps = 1,
	icm_gyro_1000dps = 2,
	icm_gyro_2000dps = 3
};

// @brief Gyroscope configuration parameters
enum icm_accel_config_params
{
	icm_accel_2g = 0,
	icm_accel_4g = 1,
	icm_accel_8g = 2,
	icm_accel_16g = 3
};


//****** Implementation checks ******//

#ifndef ICM_THROUGH_SPI
	#ifndef ICM_THROUGH_I2C
		#error	ICM preferable interface is not defined

	#endif /* ICM_THROUGH_I2C */
#endif /* ICM_THROUGH_SPI */

#ifdef ICM_THROUGH_SPI
	#ifdef ICM_THROUGH_I2C
		#error I2C and SPI can not be used at the same time

	#endif /* ICM_THROUGH_I2C */
#endif /* ICM_THROUGH_SPI */

#endif /* ICM_20600_H_ */

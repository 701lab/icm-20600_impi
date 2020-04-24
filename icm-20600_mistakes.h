#ifndef ICM_20600_MISTAKES_H_
#define ICM_20600_MISTAKES_H_

/*
	@file icm-20600_mistakes.h
	@brief Contains defines of all mistakes codes that icm-20600 library uses
 */

// @brief Mistakes code offset. Can be redefined in the source code (Should be higher than #include "nrf24l01p.h") by the user to get mistakes in any desired range.
#ifndef ICM_20600_MISTAKES_OFFSET
	#define ICM_20600_MISTAKES_OFFSET					(0U)
#endif

#define ICM_20600_DEVICE_IS_NOT_CONNECTED				(1U) + ICM_20600_MISTAKES_OFFSET
#define ICM_20600_INSTANCE_WAS_NOT_INITIALIZED			(2U) + ICM_20600_MISTAKES_OFFSET
#define ICM_20600_WRONG_GYROSCOPE_SCALE					(3U) + ICM_20600_MISTAKES_OFFSET
#define ICM_20600_WRONG_ACCELEROMETER_SCALE				(4U) + ICM_20600_MISTAKES_OFFSET


#endif /* ICM_20600_MISTAKES_H_ */

#ifndef ICM_20600_REGISTERS_H_
#define ICM_20600_REGISTERS_H_

//*** ICM-20600 registers ***//

// Registers
#define ICM_XG_OFFS_TC_H				0x04
#define ICM_XG_OFFS_TC_L				0x05
#define ICM_YG_OFFS_TC_H				0x07
#define ICM_YG_OFFS_TC_L				0x08
#define ICM_ZG_OFFS_TC_H				0x0a
#define ICM_ZG_OFFS_TC_L				0x0b

// Registers
#define ICM_SELF_TEST_X_ACCEL			0x0D
#define ICM_SELF_TEST_Y_ACCEL			0x0E
#define ICM_SELF_TEST_Z_ACCEL			0x0F

// Registers
#define ICM_XG_OFFS_USRH				0x13
#define ICM_XG_OFFS_USRL				0x14
#define ICM_YG_OFFS_USRH				0x15
#define ICM_YG_OFFS_USRL				0x16
#define ICM_ZG_OFFS_USRH				0x17
#define ICM_ZG_OFFS_USRL				0x18

// Registers
#define ICM_SMPLRT_DIV					0x19
#define ICM_CONFIG						0x1A

#define ICM_GYRO_CONFIG					0x1B
	#define	ICM_GYRO_CONFIG_FS_SEL_pos		(3U)

#define ICM_ACCEL_CONFIG_1				0x1C
	#define ICM_ACCEL_CONFIG_1_FS_SEL_pos	(3U)

#define ICM_ACCEL_CONFIG_2				0x1D
#define ICM_LP_MODE_CFG					0x1E

// Registers
#define ICM_ACCEL_WOM_X_THR				0x20
#define ICM_ACCEL_WOM_Y_THR				0x21
#define ICM_ACCEL_WOM_Z_THR				0x22
#define ICM_FIFO_EN						0x23

// Registers
#define ICM_FSYNC_INT					0x36
#define ICM_INT_PIN_CFG					0x37
#define ICM_INT_ENABLE					0x38
#define ICM_FIFO_WM_INT_STATUS			0x39
#define ICM_INT_STATUS					0x3A

// Registers
#define ICM_ACCEL_XOUT_H				0x3B
#define ICM_ACCEL_XOUT_L				0x3C
#define ICM_ACCEL_YOUT_H				0x3D
#define ICM_ACCEL_YOUT_L				0x3E
#define ICM_ACCEL_ZOUT_H				0x3F
#define ICM_ACCEL_ZOUT_L				0x40
#define ICM_TEMP_OUT_H					0x41
#define ICM_TEMP_OUT_L					0x42
#define ICM_GYRO_XOUT_H					0x43
#define ICM_GYRO_XOUT_L					0x44
#define ICM_GYRO_YOUT_H					0x45
#define ICM_GYRO_YOUT_L					0x46
#define ICM_GYRO_ZOUT_H					0x47
#define ICM_GYRO_ZOUT_L					0x48

// Registers
#define ICM_SELF_TEST_X_GYRO			0x50
#define ICM_SELF_TEST_Y_GYRO			0x51
#define ICM_SELF_TEST_Z_GYRO			0x52

// Registers
#define ICM_FIFO_WM_TH1					0x60
#define ICM_FIFO_WM_TH2					0x61

// Registers
#define ICM_SIGNAL_PATH_RESET			0x68
#define ICM_ACCEL_INTEL_CTRL			0x69
#define ICM_USER_CTRL					0x6A

// Register
#define ICM_PWR_MGMT_1					0x6B
	// Commands
	#define ICM_PWR_MGMT_1_CLKSEL_AUTO		0x01
	#define ICM_PWR_MGMT_1_CLKSEL_20MHZ		0x00
	#define ICM_PWR_MGMT_1_CLKSEL_STOP		0x07
	#define ICM_PWR_MGMT_1_TEMP_DISABLE		0x08
	#define ICM_PWR_MGMT_1_GYRO_STANDBY		0x10
	#define ICM_PWR_MGMT_1_CYCLE			0x20
	#define ICM_PWR_MGMT_1_SLEEP			0x40
	#define ICM_PWR_MGMT_1_DEVICE_RESET		0x80


#define ICM_PWR_MGMT_2					0x6C

// Register
#define ICM_I2C_IF						0x70
	// Commands
	#define ICM_I2C_IF_DISABLE				0x40

// Register
#define ICM_FIFO_COUNTH					0x72
#define ICM_FIFO_COUNTL					0x73
#define ICM_FIFO_R_W					0x74
#define ICM_WHO_AM_I					0x75

// Register
#define ICM_XA_OFFSET_H					0x77
#define ICM_XA_OFFSET_L					0x78
#define ICM_YA_OFFSET_H					0x7A
#define ICM_YA_OFFSET_L					0x7B
#define ICM_ZA_OFFSET_H					0x7D
#define ICM_ZA_OFFSET_L					0x7E

//*** ICM-20600 general commands ***//
#define ICM_READ_REGISTERS				0x80
#define ICM_WRITE_REGISTERS				0x00
//#define ICM_REGISTER_MASK				0x7F;  ??


#endif /* ICM_20600_REGISTERS_H_ */

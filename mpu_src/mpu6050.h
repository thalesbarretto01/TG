/*
 * MPU6050 device description
 * Include this in your .c file
 */

/*
 * Orientation of Sensor Axis
 *
 * Axis should be individually checked for: sequence, polarity, matching.
 *  I has been fount that:s
 * 	Accelerometer readings are inverse-polarity with respect to sensor body
 *	Gyroscope is correct with respect to the sensor body
 *
 * The correct orientation and polarity is as shown:
 *  (compare with: Product specification rev. 3.4., p. 40, 46.)
 */

#ifdef __cplusplus
	extern "C" {
#endif

#ifndef _MPU6050_H_
#define _MPU6050_H_

#define MSG_AXIS "" \
"---------------------------------------------------\n" \
"      MPU-6050 FACTORY SENSOR AXIS SETUP           \n" \
"---------------------------------------------------\n" \
"                      (-Az,+Gz CCW)                \n" \
"                          ^                        \n" \
"                          | (-Ay,+Gy CCW)          \n" \
"       sensor body        |   ^		    \n" \
"        (top view)        |  /                     \n" \
"       ____________       | /                      \n" \
"      /0          /'      |/                       \n" \
"     / MPU-60050 /.       0---------->(-Ax,+Gx CCW)\n" \
"    /           /.        Standard Axis            \n" \
"   /           /. 	   Configuration            \n" \
"  /___________/.   				    \n" \
"  '...........'    				    \n" \
"                                                   \n" \
"___________________________________________________\n"


/*
 * MPU-6050 register definitions
 *
 * Here are defined all registers documented in:
 * MPU-6000/6050 Product Specification rev. 3.4. (08/19/2013) PS-MPU-6000A-00
 * MPU-6000/6050 Register Map and Desription rev. 4.12. (08/19/2013) RM-MPU-6000A-00
 *
 * Accel offset registers
 * 1. Initial values contain factory trim values. Read value, apply bias;
 * 2. Format is in +-8g scale (1mg = 4096 LSB)
 * 3. Bit 0 on the low byte of each axis is reserved and must be unchanged.
 *
 * Gyro offset registers
 * 1. Initial values are 0;
 * 2. Format is in +-1000dps scale (1dps = 32.8 LSB)
 * 3. Bit 0 on the low byte of each axis is reserved and must be unchanged.
 */

//0 0x00
#define AUX_VDDIO		1
//2 0x02
//3 0x03
//4 0x04
//5 0x05
#define XA_OFFS_USRH		6  // 0x06
#define XA_OFFS_USRL		7  // 0x07
#define YA_OFFS_USRH		8  // 0x08
#define YA_OFFS_USRL		9  // 0x09
#define ZA_OFFS_USRH		10 // 0x0A
#define ZA_OFFS_USRL		11 // 0x0B
#define PROD_ID			12 // 0X0C // new
#define SELF_TEST_X		13
#define SELF_TEST_Y		14
#define SELF_TEST_Z		15
#define SELF_TEST_A		16
//17 0x11
//18 0x12
#define XG_OFFS_USRH		19 //0x13
#define XG_OFFS_USRL		20 //0x14
#define YG_OFFS_USRH		21 //0x15
#define YG_OFFS_USRL		22 //0x16
#define ZG_OFFS_USRH		23 //0x17
#define ZG_OFFS_USRL		24 //0x18
#define SMPLRT_DIV		25
#define CONFIG			26
#define GYRO_CONFIG		27
#define ACCEL_CONFIG		28
#define FF_THR			29
#define FF_DUR			30
#define MOT_THR			31
#define MOT_DUR			32
#define ZRMOT_THR		33
#define ZRMOT_DUR		34
#define FIFO_ENABLE		35
#define I2C_MST_CTRL		36
#define I2C_SLV0_ADDR		37
#define I2C_SLV0_REG		38
#define I2C_SLV0_CTRL		39
#define I2C_SLV1_ADDR		40
#define I2C_SLV1_REG		41
#define I2C_SLV1_CTRL		42
#define I2C_SLV2_ADDR		43
#define I2C_SLV2_REG		44
#define I2C_SLV2_CTRL		45
#define I2C_SLV3_ADDR		46
#define I2C_SLV3_REG		47
#define I2C_SLV3_CTRL		48
#define I2C_SLV4_ADDR		49
#define I2C_SLV4_REG		50
#define I2C_SLV4_DO		51
#define I2C_SLV4_CTRL		52
#define I2C_SLV4_DI		53
#define I2C_MAST_STATUS		54
#define INT_PIN_CFG		55
#define INT_ENABLE		56
#define DMP_INT_STATUS		57 // new
#define INT_STATUS		58
#define ACCEL_XOUT_H		59
#define ACCEL_XOUT_L		60
#define ACCEL_YOUT_H		61
#define ACCEL_YOUT_L		62
#define ACCEL_ZOUT_H 		63
#define ACCEL_ZOUT_L		64
#define TEMP_OUT_H		65
#define TEMP_OUT_L		66
#define GYRO_XOUT_H		67
#define GYRO_XOUT_L		68
#define GYRO_YOUT_H		69
#define GYRO_YOUT_L		70
#define GYRO_ZOUT_H		71
#define GYRO_ZOUT_L		72
#define EXT_SENS_DATA_00	73
#define EXT_SENS_DATA_01	74
#define EXT_SENS_DATA_02	75
#define EXT_SENS_DATA_03	76
#define EXT_SENS_DATA_04	77
#define EXT_SENS_DATA_05	78
#define EXT_SENS_DATA_06	79
#define EXT_SENS_DATA_07	70
#define EXT_SENS_DATA_08	81
#define EXT_SENS_DATA_09	82
#define EXT_SENS_DATA_10	83
#define EXT_SENS_DATA_11	84
#define EXT_SENS_DATA_12	85
#define EXT_SENS_DATA_13	86
#define EXT_SENS_DATA_14	87
#define EXT_SENS_DATA_15	88
#define EXT_SENS_DATA_16	89
#define EXT_SENS_DATA_17	90
#define EXT_SENS_DATA_18	91
#define EXT_SENS_DATA_19	92
#define EXT_SENS_DATA_20	93
#define EXT_SENS_DATA_21	94
#define EXT_SENS_DATA_22	95
#define EXT_SENS_DATA_23	96
#define MOT_DETECT_STATUS	97
//98 0x62
#define I2C_SLV0_DO		99
#define I2C_SLV1_DO		100
#define I2C_SLV2_DO		101
#define I2C_SLV3_DO		102
#define I2C_MST_DELAY_CTRL	103
#define SIGNAL_PATH_RESET	104
#define MOT_DETECT_CTRL		105
#define USER_CTRL		106
#define PWR_MGMT_1		107
#define PWR_MGMT_2		108
#define BANK_SEL 		109 // new
#define MEM_START_ADDR 		110 // new
#define MEM_R_W			111 // new
#define PRGM_START_H		112 // new
#define PRGM_START_L		113 // new
#define FIFO_COUNT_H		114
#define FIFO_COUNT_L		115
#define FIFO_R_W 		116
#define WHO_AM_I		117

char mpu_regnames[ 128 ][ 20 ] = {
[ AUX_VDDIO ]		= "AUX_VDDIO",
[ XA_OFFS_USRH ] 	= "XA_OFFS_USRH",
[ XA_OFFS_USRL ] 	= "XA_OFFS_USRL",
[ YA_OFFS_USRH ] 	= "YA_OFFS_USRH",
[ YA_OFFS_USRL ] 	= "YA_OFFS_USRL",
[ ZA_OFFS_USRH ] 	= "ZA_OFFS_USRH",
[ ZA_OFFS_USRL ] 	= "ZA_OFFS_USRL",
[ XG_OFFS_USRH ] 	= "XG_OFFS_USRH",
[ XG_OFFS_USRL ] 	= "XG_OFFS_USRL",
[ YG_OFFS_USRH ] 	= "YG_OFFS_USRH",
[ YG_OFFS_USRL ] 	= "YG_OFFS_USRL",
[ ZG_OFFS_USRH ] 	= "ZG_OFFS_USRH",
[ ZG_OFFS_USRL ]	= "ZG_OFFS_USRL",
[ PROD_ID ] 		= "PROD_ID",
[ SELF_TEST_X ] 	= "SELF_TEST_X",
[ SELF_TEST_Y ]		= "SELF_TEST_Y",
[ SELF_TEST_Z ]		= "SELF_TEST_Z",
[ SELF_TEST_A ]		= "SELF_TEST_A",
[ SMPLRT_DIV ]		= "SMPLRT_DIV",
[ CONFIG ]		= "CONFIG",
[ GYRO_CONFIG ]		= "GYRO_CONFIF",
[ ACCEL_CONFIG ]	= "ACCEL_CONFIG",
[ FF_THR ]		= "FF_THR",
[ FF_DUR ]		= "FF_DUR",
[ MOT_THR ]		= "MOT_THR",
[ MOT_DUR ]		= "MOT_DUR",
[ ZRMOT_THR ]		= "ZRMOT_THR",
[ ZRMOT_DUR ]		= "ZRMOT_DUR",
[ FIFO_ENABLE ]		= "FIFO_ENABLE",
[ I2C_MST_CTRL ]	= "I2C_MST_CTRL",
[ I2C_SLV0_ADDR ] 	= "I2C_SLV0_ADDR",
[ I2C_SLV0_REG ]	= "I2C_SLV0_REG",
[ I2C_SLV0_CTRL ] 	= "I2C_SLV0_CTRL",
[ I2C_SLV1_ADDR ]	= "I2C_SLV1_ADDR",
[ I2C_SLV1_REG ]	= "I2C_SLV1_REG",
[ I2C_SLV1_CTRL ]	= "I2C_SLV1_CTRL",
[ I2C_SLV2_ADDR ]	= "I2C_SLV2_ADDR",
[ I2C_SLV2_REG ]	= "I2C_SLV2_REG",
[ I2C_SLV2_CTRL ]	= "I2C_SLV2_CTRL",
[ I2C_SLV3_ADDR ]	= "I2C_SLV3_ADDR",
[ I2C_SLV3_REG ]	= "I2C_SLV3_REG",
[ I2C_SLV3_CTRL ]	= "I2C_SLV3_CTRL",
[ I2C_SLV4_ADDR ]	= "I2C_SLV4_ADDR",
[ I2C_SLV4_REG ]	= "I2C_SLV4_REG",
[ I2C_SLV4_DO ]		= "I2C_SLV4_DO",
[ I2C_SLV4_CTRL ]	= "I2C_SLV4_CTRL",
[ I2C_SLV4_DI ]		= "I2C_SLV4_DI",
[ I2C_MAST_STATUS ]	= "I2C_MST_STATUS",
[ INT_PIN_CFG ]		= "INT_PIN_CFG",
[ INT_ENABLE ]		= "INT_ENABLE",
[ DMP_INT_STATUS ] 	= "DMP_INT_STATUS",
[ INT_STATUS ]		= "INT_STATUS",
[ ACCEL_XOUT_H ] 	= "ACCEL_XOUT_H",
[ ACCEL_XOUT_L ] 	= "ACCEL_XOUT_L",
[ ACCEL_YOUT_H ] 	= "ACCEL_YOUT_H",
[ ACCEL_YOUT_L ] 	= "ACCEL_YOUT_L",
[ ACCEL_ZOUT_H ] 	= "ACCEL_ZOUT_H ",
[ ACCEL_ZOUT_L ] 	= "ACCEL_ZOUT_L",
[ TEMP_OUT_H ] 		= "TEMP_OUT_H",
[ TEMP_OUT_L ] 		= "TEMP_OUT_L",
[ GYRO_XOUT_H ] 	= "GYRO_XOUT_H",
[ GYRO_XOUT_L ] 	= "GYRO_XOUT_L",
[ GYRO_YOUT_H ] 	= "GYRO_YOUT_H",
[ GYRO_YOUT_L ] 	= "GYRO_YOUT_L",
[ GYRO_ZOUT_H ] 	= "GYRO_ZOUT_H",
[ GYRO_ZOUT_L ] 	= "GYRO_ZOUT_L",
[  EXT_SENS_DATA_00 ]	= "EXT_SENS_DATA_00",
[  EXT_SENS_DATA_01 ]	= "EXT_SENS_DATA_01",
[  EXT_SENS_DATA_02 ]	= "EXT_SENS_DATA_02",
[  EXT_SENS_DATA_03 ]	= "EXT_SENS_DATA_03",
[  EXT_SENS_DATA_04 ]	= "EXT_SENS_DATA_04",
[  EXT_SENS_DATA_05 ]	= "EXT_SENS_DATA_05",
[  EXT_SENS_DATA_06 ]	= "EXT_SENS_DATA_06",
[  EXT_SENS_DATA_07 ]	= "EXT_SENS_DATA_07",
[  EXT_SENS_DATA_08 ]	= "EXT_SENS_DATA_08",
[  EXT_SENS_DATA_09 ]	= "EXT_SENS_DATA_09",
[  EXT_SENS_DATA_10 ]	= "EXT_SENS_DATA_10",
[  EXT_SENS_DATA_11 ]	= "EXT_SENS_DATA_11",
[  EXT_SENS_DATA_12 ]	= "EXT_SENS_DATA_12",
[  EXT_SENS_DATA_13 ]	= "EXT_SENS_DATA_13",
[  EXT_SENS_DATA_14 ]	= "EXT_SENS_DATA_14",
[  EXT_SENS_DATA_15 ]	= "EXT_SENS_DATA_15",
[  EXT_SENS_DATA_16 ]	= "EXT_SENS_DATA_16",
[  EXT_SENS_DATA_17 ]	= "EXT_SENS_DATA_17",
[  EXT_SENS_DATA_18 ]	= "EXT_SENS_DATA_18",
[  EXT_SENS_DATA_19 ]	= "EXT_SENS_DATA_19",
[  EXT_SENS_DATA_20 ]	= "EXT_SENS_DATA_20",
[  EXT_SENS_DATA_21 ]	= "EXT_SENS_DATA_21",
[  EXT_SENS_DATA_22 ]	= "EXT_SENS_DATA_22",
[  EXT_SENS_DATA_23 ]	= "EXT_SENS_DATA_23",
[ MOT_DETECT_STATUS ]	= "MOT_DETECT_STATUS",
[  I2C_SLV0_DO ]	= "I2C_SLV0_DO",
[  I2C_SLV1_DO ]	= "I2C_SLV1_DO",
[  I2C_SLV2_DO ]	= "I2C_SLV2_DO",
[  I2C_SLV3_DO ]	= "I2C_SLV0_DO",
[ I2C_MST_DELAY_CTRL ]	= "I2C_MST_DELAY_CTRL",
[ SIGNAL_PATH_RESET ]	= "SIGNAL_PATH_RESET",
[ MOT_DETECT_CTRL ]	= "MOT_DETECT_CTRL",
[ USER_CTRL ]		= "USER_CTRL",
[ PWR_MGMT_1 ]		= "PWR_MGMT_1",
[ PWR_MGMT_2 ]		= "PWR_MGMT_2",
[ MEM_R_W ] 		= "MEM_R_W",
[ BANK_SEL ] 		= "BANK_SEL",
[ MEM_START_ADDR ] 	= "MEM_START_ADDR",
[ PRGM_START_H ] 	= "PRGM_START_H",
[ PRGM_START_L ] 	= "PRGM_START_L",
[ FIFO_COUNT_H ]	= "FIFO_COUNT_H",
[ FIFO_COUNT_L ]	= "FIFO_COUNT_L",
[ FIFO_R_W ]		= "FIFO_R_W",
[ WHO_AM_I ]		= "WHO_AM_I",
};

/* CONFIG - default */
uint8_t mpu6050_defcfg[10][2] = {
	{ 10,		   0},
	{ PWR_MGMT_1,  	0x01},
	{ ACCEL_CONFIG,	0x00},
	{ GYRO_CONFIG, 	0x00},
	{ SMPLRT_DIV,	0x00},
	{ CONFIG,      	0x00},
	{ INT_PIN_CFG, 	0x00},
	{ INT_ENABLE,  	0x00},
	{ USER_CTRL,   	0x60},
	{ FIFO_ENABLE, 	0x00}
};

#endif

#ifdef __cplusplus
 	}
#endif





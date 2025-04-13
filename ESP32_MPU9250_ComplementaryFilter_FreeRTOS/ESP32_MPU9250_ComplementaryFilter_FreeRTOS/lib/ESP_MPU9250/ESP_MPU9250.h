#ifndef __ESP_MPU9250_H__
#define __ESP_MPU9250_H__

#include "Arduino.h"
#include "SPI.h"

#define FreeRTOS_En								0x00
// <---- ------------ START REGISTER MAPPING ------------ ---->
#define MPU9250_I2C_ADDR						0xD0		// Default I2C address
#define MPU9250_I_AM							0x71		// Who I am register value
#define MPU9250_WHO_AM_I						0x75
#define MPU9250_CONFIG							0x1A
#define MPU9250_GYRO_CONFIG						0x1B
#define MPU9250_ACCEL_CONFIG					0x1C
#define MPU9250_ACCEL_CONFIG_2        			0x1D
#define MPU9250_ACCEL_XOUT_H					0x3B
#define MPU9250_ACCEL_XOUT_L					0x3C
#define MPU9250_ACCEL_YOUT_H					0x3D
#define MPU9250_ACCEL_YOUT_L					0x3E
#define MPU9250_ACCEL_ZOUT_H					0x3F
#define MPU9250_ACCEL_ZOUT_L					0x40
#define MPU9250_TEMP_OUT_H						0x41
#define MPU9250_TEMP_OUT_L						0x42
#define MPU9250_GYRO_XOUT_H						0x43
#define MPU9250_GYRO_XOUT_L						0x44
#define MPU9250_GYRO_YOUT_H						0x45
#define MPU9250_GYRO_YOUT_L						0x46
#define MPU9250_GYRO_ZOUT_H						0x47
#define MPU9250_GYRO_ZOUT_L						0x48
#define MPU9250_USER_CTRL             			0x6A
#define MPU9250_PWR_MGMT_1						0x6B
#define MPU9250_PWR_MGMT_2						0x6C
#define I2C_MST_CTRL							0x24

// <---- ------------ SPI Pins ------------ ---->
#define HSPI_MISO                               12
#define HSPI_MOSI                               13
#define HSPI_SCLK                               14
#define HSPI_SS                                 15

// <---- ------------ Parameters for accelerometer range ------------ ---->
typedef enum
{
	MPU9250_Accelerometer_2  = ((uint8_t)0x00), /*!< Range is +- 2G */
	MPU9250_Accelerometer_4  = ((uint8_t)0x01), /*!< Range is +- 4G */
	MPU9250_Accelerometer_8  = ((uint8_t)0x02), /*!< Range is +- 8G */
	MPU9250_Accelerometer_16 = ((uint8_t)0x03)  /*!< Range is +- 16G */
}MPU9250_Accel_TypeDef;

// <---- ------------ Parameters for gyroscope range ------------ ---->
typedef enum
{
	MPU9250_Gyroscope_250  = ((uint8_t)0x00),  /*!< Range is +- 250 degrees/s */
	MPU9250_Gyroscope_500  = ((uint8_t)0x01),  /*!< Range is +- 500 degrees/s */
	MPU9250_Gyroscope_1000 = ((uint8_t)0x02),  /*!< Range is +- 1000 degrees/s */
	MPU9250_Gyroscope_2000 = ((uint8_t)0x03)   /*!< Range is +- 2000 degrees/s */
}MPU9250_Gyro_TypeDef;

// <---- ------------ Gyroscope Digital Low Pass Filter --BW(bandwidth (Hz)) --D(delay (uS)) ------------ ---->
typedef enum
{
	BW250_D970    = ((uint8_t)0x00),
	BW184_D2900   = ((uint8_t)0x01),
	BW92_D3900    = ((uint8_t)0x02),
	BW41_D5900    = ((uint8_t)0x03),
	BW20_D9900    = ((uint8_t)0x04),
	BW10_D17850   = ((uint8_t)0x05),
	BW5_D33480    = ((uint8_t)0x06),
	BW3600_D170   = ((uint8_t)0x07)
}MPU9250_GYRO_DLPF_TypeDef;

// <---- ------------ Accelerometer Digital Low Pass Filter --BW(bandwidth (Hz)) --D(delay (uS)) ------------ ---->
typedef enum
{
	BW218_D1880   = ((uint8_t)0x00),
	BW99_D2880    = ((uint8_t)0x02),
	BW44_D4880    = ((uint8_t)0x03),
	BW21_D8870    = ((uint8_t)0x04),
	BW10_D16830   = ((uint8_t)0x05),
	BW5_D32480    = ((uint8_t)0x06),
	BW420_D1380   = ((uint8_t)0x07)
}MPU9250_ACCEL_DLPF_TypeDef;

// <---- ------------ Power Management 1 ------------ ---->
typedef enum
{
	H_RESET 		= ((uint8_t)0x01) << 7,
	SLEEP 			= ((uint8_t)0x01) << 6,
	CYCLE 			= ((uint8_t)0x01) << 5,
	GYRO_STANDBY 	= ((uint8_t)0x01) << 4,
	PD_PTAT 		= ((uint8_t)0x01) << 3,
	CLKSEL_0 		= ((uint8_t)0x00),
	CLKSEL_1 		= ((uint8_t)0x01),				// CLOCK_SEL_PLL
	CLKSEL_2 		= ((uint8_t)0x02),
	CLKSEL_3 		= ((uint8_t)0x03),
	CLKSEL_4 		= ((uint8_t)0x04),
	CLKSEL_5 		= ((uint8_t)0x05),
	CLKSEL_6 		= ((uint8_t)0x06),
	CLKSEL_7 		= ((uint8_t)0x07)
}MPU9250_PWR_MGMT1_TypeDef;

// <---- ------------ Power Management 2 ------------ ---->
typedef enum
{
	DISABLE_AX 		= ((uint8_t)0x01) << 5,
	DISABLE_AY 		= ((uint8_t)0x01) << 4,
	DISABLE_AZ 		= ((uint8_t)0x01) << 3,
	DISABLE_GX 		= ((uint8_t)0x01) << 2,
	DISABLE_GY 		= ((uint8_t)0x01) << 1,
	DISABLE_GZ 		= ((uint8_t)0x01),
	DISABLE_ALL		= ((uint8_t)0x3F),
	ENABLE_ALL 		= ((uint8_t)0x00),
}MPU9250_PWR_MGMT2_TypeDef;

// <---- ------------ MPU9250 Response ------------ ---->
typedef enum
{
	MPU9250_RESULT_OK    = ((uint8_t)0x00),
	MPU9250_RESULT_ERROR = ((uint8_t)0X01)
}MPU9250_Result;

// <---- ------------ Main MPU9250 structure ------------ ---->
typedef struct
{
	MPU9250_PWR_MGMT1_TypeDef	 PWR_MGMT1;
	MPU9250_PWR_MGMT2_TypeDef	 PWR_MGMT2;
	MPU9250_GYRO_DLPF_TypeDef    Gyro_DLPF;
	MPU9250_ACCEL_DLPF_TypeDef   Accel_DLPF;
	MPU9250_Gyro_TypeDef         Gyro_Range;
	MPU9250_Accel_TypeDef        Accel_Range;
}MPU9250TypeDef;

// <---- ------------ Main MPU9250 Functions ------------ ---->
MPU9250_Result MPU9250_Init(SPIClass &SPIx, uint8_t CS_GPIOx, MPU9250TypeDef* datastruct);
MPU9250_Result MPU9250_ReadData(SPIClass &SPIx, uint8_t* buffer, uint8_t addr, uint8_t num);
MPU9250_Result MPU9250_WriteData(SPIClass &SPIx, uint8_t addr, uint8_t data);

#endif /* __ESP_MPU9250_H__*/
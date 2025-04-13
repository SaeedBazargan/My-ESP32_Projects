#include <Arduino.h>
#include "ESP_MPU9250.h"

#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif
// <---- -------------- MPU9250 Configuration Structure -------------- ---->
MPU9250TypeDef mpu_config =
{
    .PWR_MGMT1   = CLKSEL_1,           			// Use PLL as clock source
    .PWR_MGMT2   = ENABLE_ALL,         			// Enable all sensors
    .Gyro_DLPF   = BW41_D5900,         			// Low pass filter for gyro
    .Accel_DLPF  = BW44_D4880,         			// Low pass filter for accel
    .Gyro_Range  = MPU9250_Gyroscope_250,  		// Gyroscope ±250 dps
    .Accel_Range = MPU9250_Accelerometer_2 		// Accelerometer ±2G
};
// <---- --------------  -------------- ---->

void setup()
{
  Serial.begin(300);
  Serial.println("<---- MPU9250, ComplementaryFilter, and FreeRTOS starting ---->");
  
  SPIClass hspi(HSPI);
  if(MPU9250_Init(hspi, HSPI_SS, &mpu_config) != MPU9250_RESULT_OK)
  {
	Serial.println("Error: MPU9250 initialization failed.");
  }
}
// <---- --------------  -------------- ---->

void loop()
{
//   Serial.println("eeeeeeeeerrrrrrrrrrr");
}


#include <Arduino.h>
#include <stdint.h>

#include <ESP_MPU9250.h>

#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif
// <---- --------------  -------------- ---->
MPU9250TypeDef MPU9250;
// <---- --------------  -------------- ---->

SPIClass hspi = SPIClass(HSPI);
// <---- --------------  -------------- ---->

void setup()
{
  Serial.begin(300);
  Serial.println("<---- MPU9250, ComplementaryFilter, and FreeRTOS starting ---->");

	MPU9250.PWR_MGMT1	  = CLKSEL_1;						// CLOCK_SEL_PLL
	MPU9250.PWR_MGMT2	  = ENABLE_ALL;					// Enable All Sensors
	MPU9250.Gyro_DLPF   = BW250_D970;
	MPU9250.Accel_DLPF  = BW218_D1880;
	MPU9250.Gyro_Range  = MPU9250_Gyroscope_2000;
	MPU9250.Accel_Range = MPU9250_Accelerometer_16;

	if(MPU9250_Init(hspi, HSPI_SS, &MPU9250) != MPU9250_RESULT_OK)
	{
		Serial.println("eeeeeeeeerrrrrrrrrrr");
	}  
}
// <---- --------------  -------------- ---->

void loop()
{
  Serial.println("eeeeeeeeerrrrrrrrrrr");
}




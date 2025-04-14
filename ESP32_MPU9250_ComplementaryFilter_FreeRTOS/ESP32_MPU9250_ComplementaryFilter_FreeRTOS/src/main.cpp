#include <Arduino.h>
#include "ESP_MPU9250.h"

// <---- ------------------------------------------------------------- ---->
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

#define led_pin         LED_BUILTIN

#define ZERO_MAX				20
#define ZERO_MIN				-20

// <---- ------------------------------------------------------------- ---->
static TaskHandle_t readDataTask = NULL;
static TaskHandle_t runTask = NULL;

// <---- -------------- MPU9250 Configuration Structure -------------- ---->
MPU9250TypeDef MPU9250_Config =
{
    .PWR_MGMT1   = CLKSEL_1,           			  // Use PLL as clock source
    .PWR_MGMT2   = ENABLE_ALL,         			  // Enable all sensors
    .Gyro_DLPF   = BW41_D5900,         			  // Low pass filter for gyro
    .Accel_DLPF  = BW44_D4880,         			  // Low pass filter for accel
    .Gyro_Range  = MPU9250_Gyroscope_250,  		// Gyroscope ±250 dps
    .Accel_Range = MPU9250_Accelerometer_2 		// Accelerometer ±2G
};

// <---- -------------- Variables -------------- ---->
uint8_t IMU_rawData[14] = {0};
int16_t Raw_Accel[3] = {0}, Raw_Gyro[3] = {0};

float GX = 0, GY = 0, GZ = 0, AX = 0, AY = 0, AZ = 0;
float Roll, Pitch, Yaw;

SPIClass hspi(HSPI);

// <---- ------------------------------------------------------------- ---->
void startReadDataTask(void* parameter);
void startRunTask(void* parameter);
void IMU_readRawData(void);
void IMU_UpdateAngles(void);

// <---- -------------- int main() -------------- ---->
void setup()
{
  pinMode(led_pin, OUTPUT);
  Serial.begin(300);
  Serial.println("<---- MPU9250, ComplementaryFilter, and FreeRTOS starting ---->");

  if(MPU9250_Init(hspi, HSPI_SS, &MPU9250_Config) != MPU9250_RESULT_OK)
  {
  	Serial.println("Error: MPU9250 initialization failed.");
  }

  Serial.println("MPU9250 initialization successful.");

  xTaskCreatePinnedToCore(startRunTask, "RUN_TASK", 2048, NULL, 2, &runTask, app_cpu);
  xTaskCreatePinnedToCore(startReadDataTask, "READ_DATA_TASK", 2048, NULL, 1, &readDataTask, app_cpu);
}

// <---- -------------- Loop -------------- ---->
void loop()
{}

// <---- ------------ Check Running Task ------------ ---->
void startRunTask(void* parameter)
{
  // UBaseType_t watermark = uxTaskGetStackHighWaterMark(NULL);
  // Serial.printf("RUN_TASK stack free: %u\n", watermark);

  for(;;)
  {
    digitalWrite(led_pin, HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    digitalWrite(led_pin, LOW);
    vTaskDelay(500 / portTICK_PERIOD_MS);  
  }
}

// <---- ------------ IMU Read Raw Data Task ------------ ---->
void startReadDataTask(void* parameter)
{
  // Serial.printf("Free heap: %u (min: %u)\n", 
  //            esp_get_free_heap_size(),
  //            esp_get_minimum_free_heap_size());

  for(;;)
  {
    IMU_readRawData();

    IMU_UpdateAngles();
  }
}

// <---- ------------ IMU Read Raw Data ------------ ---->
void IMU_readRawData(void)
{
  // <---- ------------ Merging Data to get real IMU data ------------ ---->
  MPU9250_ReadData(hspi, IMU_rawData, MPU9250_ACCEL_XOUT_H, 6);
	Raw_Accel[0] = (int16_t)(IMU_rawData[0] << 8 | IMU_rawData[1]);
	Raw_Accel[1] = (int16_t)(IMU_rawData[2] << 8 | IMU_rawData[3]);
	Raw_Accel[2] = (int16_t)(IMU_rawData[4] << 8 | IMU_rawData[5]);

	MPU9250_ReadData(hspi, IMU_rawData, MPU9250_GYRO_XOUT_H, 6);
	Raw_Gyro[0] = (int16_t)(IMU_rawData[0] << 8 | IMU_rawData[1]);
	Raw_Gyro[1] = (int16_t)(IMU_rawData[2] << 8 | IMU_rawData[3]);
	Raw_Gyro[2] = (int16_t)(IMU_rawData[4] << 8 | IMU_rawData[5]);

	// <---- ------------ Removing Gyro's Noise around 0 state ------------ ---->
	Raw_Gyro[0] = (Raw_Gyro[0] <= ZERO_MAX && Raw_Gyro[0] >= ZERO_MIN)? 0: Raw_Gyro[0];
	Raw_Gyro[1] = (Raw_Gyro[1] <= ZERO_MAX && Raw_Gyro[1] >= ZERO_MIN)? 0: Raw_Gyro[1];
	Raw_Gyro[2] = (Raw_Gyro[2] <= ZERO_MAX && Raw_Gyro[2] >= ZERO_MIN)? 0: Raw_Gyro[2];

	GX = (float)(Raw_Gyro[0] * (2000 / 32768.0));
	GY = (float)(Raw_Gyro[1] * (2000 / 32768.0));
	GZ = (float)(Raw_Gyro[2] * (2000 / 32768.0));

	AX = (float)(Raw_Accel[0] * (16.0 / 32768.0));
	AY = (float)(Raw_Accel[1] * (16.0 / 32768.0));
	AZ = (float)(Raw_Accel[2] * (16.0 / 32768.0));

	GX = GX / (180.0 / M_PI);
	GY = GY / (180.0 / M_PI);
	GZ = GZ / (180.0 / M_PI);

  // Serial.println(AX);
  // Serial.println(AY);
  // Serial.println(AZ);
  // Serial.println(GX);
  // Serial.println(GY);
  // Serial.println(GZ);
  // Serial.println("<---- ------------------- ---->");
}

// <---- ------------ Convert raw data to the Roll and Pitch ------------ ---->
void IMU_UpdateAngles(void)
{
  static float last_roll, last_pitch;
  float roll_acc = 0.0, pitch_acc = 0.0;

  roll_acc = atan2f(AY, sqrtf((AX * AX) + (AZ * AZ)));
  pitch_acc = atan2f(-AX, sqrtf((AY * AY) + (AZ * AZ)));
  last_roll = roll_acc;
  last_pitch = pitch_acc;
  // first_run = 0;

  Serial.print("Roll:   ");
  Serial.println(last_roll);
  Serial.print("Pitch:   ");
  Serial.println(last_pitch);
  Serial.println("zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz");

  // data.Shifted_Roll  = (Roll + 10) * 10;
  // data.Shifted_Pitch = (Pitch + 10) * 10;
  // osMessagePut(Queue_1Handle, &data, osWaitForever);
  // osDelay(100);
}
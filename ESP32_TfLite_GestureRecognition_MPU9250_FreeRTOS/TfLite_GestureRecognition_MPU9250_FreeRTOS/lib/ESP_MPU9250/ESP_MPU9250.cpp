#include "ESP_MPU9250.h"

// <---- ------------ Variables ------------ ---->
SPIClass *_spi = nullptr;
uint8_t _csPin = 0;

// <---- ------------ Main MPU9250 Functions ------------ ---->
// <---- ------------ MPU9250 Initialize ------------ ---->
MPU9250_Result MPU9250_Init(SPIClass &SPIx, uint8_t CS_GPIOx, MPU9250TypeDef* datastruct)
{
    _spi = &SPIx;
    _csPin = CS_GPIOx;
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin,HIGH);
    _spi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS);

    uint8_t WHO_AM_I = MPU9250_WHO_AM_I;
	uint8_t temp;
	uint8_t transmit_buffer[2] = {0};

	// <---- ------------ Initialize Registers ------------ ---->
	uint8_t CLOCK_SEL_PLL = 0x01;
	transmit_buffer[0] = MPU9250_PWR_MGMT_1;
	transmit_buffer[1] = CLOCK_SEL_PLL;
    MPU9250_WriteData(SPIx, transmit_buffer[0], transmit_buffer[1]);
#if FreeRTOS_En
	vTaskDelay(25 / portTICK_PERIOD_MS);
#else
	delay(25);
#endif

    uint8_t I2C_MST_EN = 0x20;
    transmit_buffer[0] = MPU9250_USER_CTRL;
    transmit_buffer[1] = I2C_MST_EN;
    MPU9250_WriteData(SPIx, transmit_buffer[0], transmit_buffer[1]);
#if FreeRTOS_En
	vTaskDelay(25 / portTICK_PERIOD_MS);
#else
	delay(25);
#endif

    uint8_t I2C_MST_CLK = 0x0D;
    transmit_buffer[0] = I2C_MST_CTRL;
    transmit_buffer[1] = I2C_MST_CLK;
    MPU9250_WriteData(SPIx, transmit_buffer[0], transmit_buffer[1]);
#if FreeRTOS_En
	vTaskDelay(25 / portTICK_PERIOD_MS);
#else
	delay(25);
#endif

    uint8_t PWR_RESET = 0x80;
    transmit_buffer[0] = MPU9250_PWR_MGMT_1;
    transmit_buffer[1] = PWR_RESET;
    MPU9250_WriteData(SPIx, transmit_buffer[0], transmit_buffer[1]);
#if FreeRTOS_En
	vTaskDelay(25 / portTICK_PERIOD_MS);
#else
	delay(25);
#endif

	// <---- ------------ Configure PowerManagement 1 ------------ ---->
	transmit_buffer[0] = MPU9250_PWR_MGMT_1;
	transmit_buffer[1] = datastruct->PWR_MGMT1;
    MPU9250_WriteData(SPIx, transmit_buffer[0], transmit_buffer[1]);
#if FreeRTOS_En
    vTaskDelay(25 / portTICK_PERIOD_MS);
#else
    delay(25);
#endif
        MPU9250_ReadData(SPIx, &temp, MPU9250_PWR_MGMT_1, 1);
        if(temp != datastruct->PWR_MGMT1)
        {
            Serial.println("Error1");
            Serial.println(temp);
            return MPU9250_RESULT_ERROR;
        }
    
    // <---- ------------ Check WHO_AM_I Register ------------ ---->
    MPU9250_ReadData(SPIx, &temp, WHO_AM_I, 1);
	if(temp != MPU9250_WHO_AM_I && temp != MPU9250_I_AM)
    {
        Serial.println("Error2:");
        Serial.println(temp);
        return MPU9250_RESULT_ERROR;
    }
#if FreeRTOS_En
    vTaskDelay(25 / portTICK_PERIOD_MS);
#else
    delay(25);
#endif
 
    // <---- ------------ Configure PowerManagement 2 ------------ ---->
	transmit_buffer[0] = MPU9250_PWR_MGMT_2;
	transmit_buffer[1] = datastruct->PWR_MGMT2;
    MPU9250_WriteData(SPIx, transmit_buffer[0], transmit_buffer[1]);
#if FreeRTOS_En
    vTaskDelay(25 / portTICK_PERIOD_MS);
#else
    delay(25);
#endif
    MPU9250_ReadData(SPIx, &temp, MPU9250_PWR_MGMT_2, 1);
    if(temp != datastruct->PWR_MGMT2)
    {
        Serial.println("Error3");
        Serial.println(temp);
        return MPU9250_RESULT_ERROR;
    }

	// <---- ------------ Configure Config Register ------------ ---->
	transmit_buffer[0] = MPU9250_CONFIG;
	transmit_buffer[1] = datastruct->Gyro_DLPF;
    MPU9250_WriteData(SPIx, transmit_buffer[0], transmit_buffer[1]);
#if FreeRTOS_En
    vTaskDelay(25 / portTICK_PERIOD_MS);
#else
    delay(25);
#endif
    MPU9250_ReadData(SPIx, &temp, MPU9250_CONFIG, 1);
    if(temp != datastruct->Gyro_DLPF)
    {
        Serial.println("Error4");
        Serial.println(temp);
        return MPU9250_RESULT_ERROR;
    }

	// <---- ------------ Configure Gyro_Config Register ------------ ---->
	transmit_buffer[0] = MPU9250_GYRO_CONFIG;
	transmit_buffer[1] = (datastruct->Gyro_Range) << 3;
    MPU9250_WriteData(SPIx, transmit_buffer[0], transmit_buffer[1]);
#if FreeRTOS_En
    vTaskDelay(25 / portTICK_PERIOD_MS);
#else
    delay(25);
#endif
    MPU9250_ReadData(SPIx, &temp, MPU9250_GYRO_CONFIG, 1);
    if(temp != ((datastruct->Gyro_Range) << 3))
    {
        Serial.println("Error5");
        Serial.println(temp);
        return MPU9250_RESULT_ERROR;
    }
    
	// <---- ------------ Configure Accel Config Register 2 ------------ ---->
	transmit_buffer[0] = MPU9250_ACCEL_CONFIG_2;
	transmit_buffer[1] = datastruct->Accel_DLPF;
    MPU9250_WriteData(SPIx, transmit_buffer[0], transmit_buffer[1]);
#if FreeRTOS_En
    vTaskDelay(25 / portTICK_PERIOD_MS);
#else
    delay(25);
#endif
    MPU9250_ReadData(SPIx, &temp, MPU9250_ACCEL_CONFIG_2, 1);
    if(temp != datastruct->Accel_DLPF)
    {
        Serial.println("Error6");
        Serial.println(temp);
        return MPU9250_RESULT_ERROR;
    }

	// <---- ------------ Configure Accel Config Register ------------ ---->
	transmit_buffer[0] = MPU9250_ACCEL_CONFIG;
	transmit_buffer[1] = (datastruct->Accel_Range) << 3;
    MPU9250_WriteData(SPIx, transmit_buffer[0], transmit_buffer[1]);
#if FreeRTOS_En
    vTaskDelay(25 / portTICK_PERIOD_MS);
#else
    delay(25);
#endif
    MPU9250_ReadData(SPIx, &temp, MPU9250_ACCEL_CONFIG, 1);
    if(temp != ((datastruct->Accel_Range) << 3))
    {
        Serial.println("Error7");
        Serial.println(temp);
        return MPU9250_RESULT_ERROR;
    }

    /* Return OK */
	return MPU9250_RESULT_OK;
}

// <---- ------------ MPU9250 Read Data ------------ ---->
MPU9250_Result MPU9250_ReadData(SPIClass &SPIx, uint8_t* buffer, uint8_t addr, uint8_t num)
{
    _spi = &SPIx;
    uint8_t reg = addr | 0x80;

    _spi->beginTransaction(SPISettings(SPI_CLOCK_DIV16, MSBFIRST, SPI_MODE3));
    digitalWrite(_csPin, LOW);
    _spi->transfer(reg);
    for(uint8_t i = 0; i < num; i++)
    {
        buffer[i] = _spi->transfer(0x00);
    }
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();

    /* Return OK */
	return MPU9250_RESULT_OK;    
}

// <---- ------------ MPU9250 Write Data ------------ ---->
MPU9250_Result MPU9250_WriteData(SPIClass &SPIx, uint8_t addr, uint8_t data)
{
    _spi = &SPIx;

    _spi->beginTransaction(SPISettings(SPI_CLOCK_DIV16, MSBFIRST, SPI_MODE3));
    digitalWrite(_csPin, LOW);
    _spi->transfer(addr);
    _spi->transfer(data);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();

    /* Return OK */
	return MPU9250_RESULT_OK;
}

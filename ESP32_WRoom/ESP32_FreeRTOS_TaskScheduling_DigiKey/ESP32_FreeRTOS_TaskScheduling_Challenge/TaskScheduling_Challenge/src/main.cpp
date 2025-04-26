#include <Arduino.h>
#include <stdint.h>

#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif
//<---- ------------------------------------------------------------------ ---->
#define lengthOfNumber              5
#define led_pin                     LED_BUILTIN
//<---- ------------------------------------------------------------------ ---->
char recNum_c[lengthOfNumber];
uint8_t counterOfRecNumber = 0;
static int realNumber = 500;
//<---- ------------------------------------------------------------------ ---->
static TaskHandle_t recTask = NULL;
static TaskHandle_t toggleTask = NULL;

void startReceiveTask(void* parameter);
void startToggleLEDTask(void* parameter);
//<---- ------------------------------------------------------------------ ---->
void setup()
{
  Serial.begin(300);
  Serial.println("<---- Multi-task LED Demo ---->");
  Serial.println("<---- Enter a number in milliseconds to change the LED delay ---->");

  pinMode(led_pin, OUTPUT);
  
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  xTaskCreatePinnedToCore(startReceiveTask, "REC_TASK", 1024, NULL, 1, &recTask, app_cpu);
  xTaskCreatePinnedToCore(startToggleLEDTask, "TOGGLE_TASK", 1024, NULL, 1, &toggleTask, app_cpu);
  vTaskDelete(NULL);
}
//<---- ------------------------------------------------------------------ ---->
void loop()
{}
//<---- ------------------------------------------------------------------ ---->
void startReceiveTask(void* parameter)
{
  while (1)
  {
    if(Serial.available())
    {
      recNum_c[++counterOfRecNumber] = Serial.read();
      Serial.print(recNum_c[counterOfRecNumber]);

      if(counterOfRecNumber > 10)
      {
        Serial.println("Wrong Number!");
      }
      else if(recNum_c[counterOfRecNumber] == '\n')
      {
        realNumber = atoi(&recNum_c[1]);
        Serial.print("your number is: ");
        Serial.println(realNumber);

        memset(recNum_c, 0, lengthOfNumber);
        counterOfRecNumber = 0;
      }
    }
  }
}
//<---- ------------------------------------------------------------------ ---->
void startToggleLEDTask(void* parameter)
{
  while (1)
  {
    digitalWrite(led_pin, HIGH);
    vTaskDelay(realNumber / portTICK_PERIOD_MS);
    digitalWrite(led_pin, LOW);
    vTaskDelay(realNumber / portTICK_PERIOD_MS);
  }  
}
//<---- ------------------------------------------------------------------ ---->

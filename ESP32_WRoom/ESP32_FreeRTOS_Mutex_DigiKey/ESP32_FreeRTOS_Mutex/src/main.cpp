#include <Arduino.h>
#include <stdint.h>

#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif
//<---- --------------------------------------------------------- ---->
static uint8_t shared_var = 0;
//<---- --------------------------------------------------------- ---->
static SemaphoreHandle_t mutex;
static TaskHandle_t inc_task = NULL;
//<---- --------------------------------------------------------- ---->
void IncrementTask(void* parameters);
//<---- --------------------------------------------------------- ---->
void setup() 
{
  Serial.begin(300);
  Serial.println("<---- FreeRTOS Race Condition Demo ---->");

  mutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(IncrementTask, "INC_TASK", 1024, NULL, 1, &inc_task, app_cpu);
  vTaskDelete(NULL);
}
//<---- --------------------------------------------------------- ---->
void loop() 
{}
//<---- --------------------------------------------------------- ---->
void IncrementTask(void* parameters)
{
  uint8_t local_var = 0;

  while (1)
  {
    if(xSemaphoreTake(mutex, 0) == pdTRUE)
    {
      local_var = shared_var;
      local_var++;
      vTaskDelay(250 / portTICK_PERIOD_MS);
      shared_var = local_var;

      xSemaphoreGive(mutex);

      Serial.println(shared_var);
    }
    else
    {

    }
  }  
}
#include <Arduino.h>
#include <stdint.h>

#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

const char msg[] = "Hello World";

static TaskHandle_t task1 = NULL;
static TaskHandle_t task2 = NULL;

void StartTask1(void* parameter)
{
  uint8_t msg_len = strlen(msg);

  while (1)
  {
    Serial.println();
    for (uint8_t i = 0; i < msg_len; i++)
    {
      Serial.print(msg[i]);
    }
    Serial.println();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void StartTask2(void* parameter)
{
  while (1)
  {
    Serial.print("*");
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  
}

void setup() 
{
  Serial.begin(300);

  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("<---- FreeRTOS Task Demo ---->");

  Serial.print("Setup and loop task running on core ");
  Serial.print(xPortGetCoreID());
  Serial.print(" with priority ");
  Serial.println(uxTaskPriorityGet(NULL));

  xTaskCreatePinnedToCore(StartTask1, "Task1", 1024, NULL, 1, &task1, app_cpu);
  xTaskCreatePinnedToCore(StartTask2, "Task2", 1024, NULL, 2, &task1, app_cpu);
}

void loop()
{
  for (uint8_t i = 0; i < 3; i++)
  {
    vTaskSuspend(task2);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    vTaskResume(task2);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }

  if (task1 != NULL)
  {
    vTaskDelete(task1);
    task1 = NULL;
  }
  
}

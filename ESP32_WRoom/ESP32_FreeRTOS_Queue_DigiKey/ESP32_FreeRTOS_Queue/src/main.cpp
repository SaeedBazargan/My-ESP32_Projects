#include <Arduino.h>
#include <stdint.h>

#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif
//<---- --------------------------------------------------------- ---->
#define msg_queue_length          5
//<---- --------------------------------------------------------- ---->
uint8_t num = 0;
//<---- --------------------------------------------------------- ---->
static QueueHandle_t msg_queue;
static TaskHandle_t msg_task;
//<---- --------------------------------------------------------- ---->
void printMessages(void* parameter);
//<---- --------------------------------------------------------- ---->
void setup()
{
  Serial.begin(300);
  Serial.println("<---- FreeRTOS Queue Demo ---->");

  msg_queue = xQueueCreate(msg_queue_length, sizeof(int));

  xTaskCreatePinnedToCore(printMessages, "PRINTMSG", 1024, NULL, 1, &msg_task, app_cpu);
}
//<---- --------------------------------------------------------- ---->
void loop()
{
  if(xQueueSend(msg_queue, &num, 10) != pdTRUE)
  {
    Serial.println("Queue is full");
  }
  num++;

  vTaskDelay(500 / portTICK_PERIOD_MS);
}
//<---- --------------------------------------------------------- ---->
void printMessages(void* parameter)
{
  uint8_t item;

  while (1)
  {
    if(xQueueReceive(msg_queue, &item, 0) == pdTRUE)
    {
      Serial.println(item);
      Serial.println("<---- ---- ---->");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

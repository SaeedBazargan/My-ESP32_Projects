#include <Arduino.h>
#include <stdint.h>

#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

#define msg_queue_length            5
#define lengthOfChar                50
#define led_pin                     BUILTIN_LED
//<---- --------------------------------------------------------- ---->
static QueueHandle_t Queue_1, Queue_2;

static TaskHandle_t Task_1 = NULL;
static TaskHandle_t Task_2 = NULL;
//<---- --------------------------------------------------------- ---->
char recChar_c[lengthOfChar];
uint8_t counterOfRecChar = 0;
//<---- --------------------------------------------------------- ---->
void startTask1(void* parameter);
void startTask2(void* parameter);
//<---- --------------------------------------------------------- ---->
void setup()
{
  Serial.begin(300);
  Serial.println("<---- FreeRTOS Queue Demo ---->");
  Serial.println("Enter the command 'delay xxx' where xxx is your desired ");

  pinMode(led_pin, OUTPUT);

  vTaskDelay(1000 / portTICK_PERIOD_MS);

<<<<<<< HEAD
  Queue_1 = xQueueCreate(msg_queue_length, sizeof(int));
  Queue_2 = xQueueCreate(msg_queue_length, sizeof(int));

  xTaskCreatePinnedToCore(startTask1, "TASK_1", 1024, NULL, 1, &Task_1, app_cpu);
  // xTaskCreatePinnedToCore(startTask2, "TASK_2", 1024, NULL, 1, &Task_2, app_cpu);
=======
  Queue_1 = xQueueCreate(msg_queue_length, sizeof(int64_t));
  Queue_2 = xQueueCreate(msg_queue_length, sizeof(int64_t));

  xTaskCreatePinnedToCore(startTask1, "TASK_1", 1024, NULL, 1, &Task_1, app_cpu);
  xTaskCreatePinnedToCore(startTask2, "TASK_2", 1024, NULL, 1, &Task_2, app_cpu);

  // Delete "setup and loop" task
  vTaskDelete(NULL);
>>>>>>> 8a043841b1eb0dea66c839e5b6ec503c11b5c212
}
//<---- --------------------------------------------------------- ---->
void loop()
{}
//<---- --------------------------------------------------------- ---->
void startTask1(void* parameter)
{
  char rec[10];
  
  while (1)
  {
    if(xQueueReceive(Queue_2, (void*)&rec, 0) == pdTRUE)
    {
      Serial.println("<---- Recieve from Queue ---->");
      Serial.println(rec);
    }

    if(Serial.available())
    {
      recChar_c[++counterOfRecChar] = Serial.read();
      Serial.print(recChar_c[counterOfRecChar]);

      if(counterOfRecChar > 49)
      {
        Serial.println(" Write Shorter than 50 words!");
        
        memset(recChar_c, 0, lengthOfChar);
        counterOfRecChar = 0;
      }
      else if(recChar_c[counterOfRecChar] == '\n')
      {
        Serial.print("your character is: ");
        Serial.println(&recChar_c[1]);

        if(memcmp(&recChar_c[1], "delay ", 6) == 0)
        {
          int tmp = atoi(&recChar_c[7]);
          
<<<<<<< HEAD
          if(xQueueSend(Queue_1, (void*)&tmp, 0) != pdTRUE)
=======
          if(xQueueSend(Queue_1, (void*)&tmp, 10) != pdTRUE)
>>>>>>> 8a043841b1eb0dea66c839e5b6ec503c11b5c212
          {
            Serial.print("you can not send delay time to Queue_1!");
          }
        }

        memset(recChar_c, 0, lengthOfChar);
        counterOfRecChar = 0;
      }
    }
  }
}
//<---- --------------------------------------------------------- ---->
void startTask2(void* parameter)
{
  int delay = 250;
  char status[10];
  uint8_t blinkCounter = 0;

  while (1)
  {
    digitalWrite(led_pin, HIGH);
    vTaskDelay(delay / portTICK_PERIOD_MS);
    digitalWrite(led_pin, LOW);
    vTaskDelay(delay / portTICK_PERIOD_MS);

    blinkCounter++;

    if(xQueueReceive(Queue_1, (void*)&delay, 0) == pdTRUE);

    if(blinkCounter == 100)
    {
      strcpy(status, "Blinked");
      if(xQueueSend(Queue_2, (void*)&status, 10) != pdTRUE)
      {
        Serial.print("you can not send delay time to Queue_2!");
      }
      blinkCounter = 0;
    }
  }
}

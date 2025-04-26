#include <Arduino.h>
#include <freertos/ringbuf.h>

#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif
//<---- ------------------------------------------------------------------ ---->
#define lengthOfNumber              5
#define led_pin                     LED_BUILTIN
#define msg_queue_length            10
//<---- ------------------------------------------------------------------ ---->
char recNum_c[lengthOfNumber];
uint8_t counterOfRecCharacter = 0;
static int realNumber = 500;

typedef struct 
{
  char message[20];  // "Average: " message
  int Number;        // The calculated average value
} AvgData_t;

static const uint16_t timer_divider = 800;   // Clock tick at 10 MHz (ESP32 Timer = 80MHz, So timer_divider = 800 it means 10 Sec)
static const uint64_t timer_max_count = 1000000;

volatile uint8_t recordCount = 0;
//<---- ------------------------------------------------------------------ ---->
static QueueHandle_t avg_msg_queue;
static TaskHandle_t recordTask_Handle = NULL;
static TaskHandle_t echoTask_Handle = NULL;
static RingbufHandle_t buffer_Handle = NULL;
static hw_timer_t *hal_timer = NULL;
//<---- ------------------------------------------------------------------ ---->
void recordSerial_Task(void* parameters);
void echoSerial_Task(void* parameters);
void IRAM_ATTR timer_isr_handler();
//<---- ------------------------------------------------------------------ ---->
//<---- ------------------------------------------------------------------ ---->

void setup() 
{
  Serial.begin(300);
  Serial.println("<---- FreeRTOS Hardware Interrupt Demo ---->");
  Serial.println("<---- Enter some numbers to calculate the average! ---->");

  pinMode(led_pin, OUTPUT);

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  buffer_Handle = xRingbufferCreate(1024, RINGBUF_TYPE_BYTEBUF);
  
  avg_msg_queue = xQueueCreate(msg_queue_length, sizeof(AvgData_t));

  xTaskCreatePinnedToCore(recordSerial_Task, "RECORD_TASK", 1024, NULL, 1, &recordTask_Handle, app_cpu);
  xTaskCreatePinnedToCore(echoSerial_Task, "ECHO_TASK", 1024, NULL, 1, &echoTask_Handle, app_cpu);
  
  vTaskDelete(NULL);
}
//<---- ------------------------------------------------------------------ ---->
void loop() 
{}
//<---- ------------------------------------------------------------------ ---->
void recordSerial_Task(void* parameters)
{
  AvgData_t send_AvgData;
  uint32_t ulNotifyValue;
  bool timer_flag = false;

  int *receivedData;
  size_t item_size;

  uint8_t counterOfNumbers = 0;

  while (1)
  {
    ulNotifyValue = ulTaskNotifyTake(pdTRUE, 0);
    if(ulNotifyValue != 0)
    {
      digitalWrite(led_pin, HIGH);
      Serial.println("<---- Hello from Timer_ISR function ---->");

      // Receive data from the ring buffer
      receivedData = (int*)xRingbufferReceive(buffer_Handle, &item_size, 0);

      for (uint8_t i = 0; i < counterOfNumbers; i++)
      {
        send_AvgData.Number += receivedData[i];
      }

      strcpy(send_AvgData.message, "Average: ");
      send_AvgData.Number /= counterOfNumbers;

      // Send the struct to the queue
      if(xQueueSend(avg_msg_queue, (void*)&send_AvgData, 0) == pdTRUE)
      {
        Serial.println("<---- The message has been sent to the queue ---->");
      }
    }

    if(Serial.available())
    {
      recNum_c[++counterOfRecCharacter] = Serial.read();
      Serial.print(recNum_c[counterOfRecCharacter]);

      if(counterOfRecCharacter > 10)
      {
        Serial.println("<---- Wrong Number! ---->");
      }
      else if(recNum_c[counterOfRecCharacter] == '\n')
      {
        counterOfNumbers++;
        realNumber = atoi(&recNum_c[1]);

        // Send the byte to the ring buffer
        xRingbufferSend(buffer_Handle, &realNumber, sizeof(realNumber), 0);

        if(!timer_flag)
        {
          hal_timer = timerBegin(0, timer_divider, true);               // Create and Start timer (timer_number, divider, countUp)
          timerAttachInterrupt(hal_timer, &timer_isr_handler, true);    // Provide ISR to timer (timer, void (*function)(), edge)
          timerAlarmWrite(hal_timer, timer_max_count, false);           // At what count should ISR trigger (timer, count, autoreload)
          timerAlarmEnable(hal_timer);                                  // Allow ISR to trigger

          timer_flag = true;
        }

        memset(recNum_c, 0, lengthOfNumber);
        counterOfRecCharacter = 0;
      }
    }
  }
}
//<---- ------------------------------------------------------------------ ---->
void echoSerial_Task(void* parameters)
{
  AvgData_t receivedAvgData;

  while (1)
  {
    if(xQueueReceive(avg_msg_queue, (void*)&receivedAvgData, 0) == pdTRUE) 
    {
      Serial.println("<---- Hello from ECHO_Serial Task ---->");
      Serial.print(receivedAvgData.message);
      Serial.println(receivedAvgData.Number);
    }
  }
}
//<---- ------------------------------------------------------------------ ---->
void IRAM_ATTR timer_isr_handler()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  vTaskNotifyGiveFromISR(recordTask_Handle, &xHigherPriorityTaskWoken);

  if(xHigherPriorityTaskWoken==pdTRUE)
  {
    portYIELD_FROM_ISR();
  }
}
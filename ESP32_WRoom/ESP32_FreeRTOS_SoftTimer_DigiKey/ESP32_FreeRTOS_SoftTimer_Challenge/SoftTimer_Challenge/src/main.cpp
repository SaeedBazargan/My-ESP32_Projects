#include <Arduino.h>

#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif
//<---- ------------------------------------------------------------------ ---->
#define led_pin                     LED_BUILTIN
#define lengthOfChar                50
//<---- ------------------------------------------------------------------ ---->
static const TickType_t checklimit = 5000 / portTICK_PERIOD_MS;

char recChar_c[lengthOfChar];
uint8_t counterOfRecChar = 0;
//<---- ------------------------------------------------------------------ ---->
static TaskHandle_t echoTask = NULL;

static TimerHandle_t checkChar_Timer = NULL;
//<---- ------------------------------------------------------------------ ---->
void myTimerCallback(TimerHandle_t xTimer);
void echoSerial_Task(void* parameters);
//<---- ------------------------------------------------------------------ ---->
void setup() 
{
  Serial.begin(300);
  Serial.println("<---- FreeRTOS Timer Demo ---->");
  Serial.println("<---- Enter the command but write shorter than 50 words! ---->");

  pinMode(led_pin, OUTPUT);

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  checkChar_Timer = xTimerCreate("CHECKING", checklimit, pdFALSE, (void*)0, myTimerCallback);

  xTaskCreatePinnedToCore(echoSerial_Task, "ECHO_TASK", 1024, NULL, 1, &echoTask, app_cpu);

  vTaskDelete(NULL);
}
//<---- ------------------------------------------------------------------ ---->
void loop() 
{}
//<---- ------------------------------------------------------------------ ---->
void echoSerial_Task(void* parameters)
{
  while(1)
  {
    if(Serial.available())
    {
      recChar_c[++counterOfRecChar] = Serial.read();
      Serial.print(recChar_c[counterOfRecChar]);

      digitalWrite(led_pin, HIGH);

      if(counterOfRecChar > 49)
      {
        Serial.println(" Please write shorter than 50 words!");
        
        memset(recChar_c, 0, lengthOfChar);
        counterOfRecChar = 0;
      }
      else if(recChar_c[counterOfRecChar] == '\n')
      {
        Serial.print("your character is: ");
        Serial.println(&recChar_c[1]);
      }

      xTimerStart(checkChar_Timer, portMAX_DELAY);
    }
  }
}
//<---- ------------------------------------------------------------------ ---->
void myTimerCallback(TimerHandle_t xTimer)
{
  memset(recChar_c, 0, lengthOfChar);
  counterOfRecChar = 0;

  Serial.println("<---- Maybe you should be fast ---->");
  digitalWrite(led_pin, LOW);
}


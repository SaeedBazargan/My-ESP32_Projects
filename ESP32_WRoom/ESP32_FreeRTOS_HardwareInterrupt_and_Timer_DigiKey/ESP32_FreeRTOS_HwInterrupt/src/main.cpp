#include <Arduino.h>

#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

#define led_pin                     LED_BUILTIN
//<---- ------------------------------------------------------------------ ---->
static const uint16_t timer_divider = 8;   // Clock tick at 10 MHz (ESP32 Timer = 80MHz, So timer_divider = 8 it means 100 milliSec)
static const uint64_t timer_max_count = 1000000;

static const TickType_t delay_time = 2000 / portTICK_PERIOD_MS;

static hw_timer_t *hal_timer = NULL;
static volatile int isr_counter;
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
//<---- ------------------------------------------------------------------ ---->
void IRAM_ATTR onTimer();
void print_values(void* parameters);
//<---- ------------------------------------------------------------------ ---->
//<---- ------------------------------------------------------------------ ---->

void setup() 
{
  Serial.begin(300);
  Serial.println("<--- FreeRTOS ISR Critical Section Demo ---->");

  // Create and Start timer (timer_number, divider, countUp)
  hal_timer = timerBegin(0, timer_divider, true);

  // Provide ISR to timer (timer, void (*function)(), edge)
  timerAttachInterrupt(hal_timer, &onTimer, true);

  // At what count should ISR trigger (timer, count, autoreload)
  timerAlarmWrite(hal_timer, timer_max_count, true);

  //Allow ISR to trigger
  timerAlarmEnable(hal_timer);

  xTaskCreatePinnedToCore(print_values, "PRINT_TASK", 1024, NULL, 1, NULL, app_cpu);

  vTaskDelete(NULL);
}
//<---- ------------------------------------------------------------------ ---->
void loop()
{}
//<---- ------------------------------------------------------------------ ---->
void IRAM_ATTR onTimer()  // This ISR function executes when timer reaches max and resets
{
  portENTER_CRITICAL_ISR(&spinlock);
  isr_counter++;
  portEXIT_CRITICAL_ISR(&spinlock);
}
//<---- ------------------------------------------------------------------ ---->
void print_values(void* parameters)   // wait for semaphore and print out value when received 
{
  while (1)
  {
    while (isr_counter > 0)
    {
      Serial.println(isr_counter);

      portENTER_CRITICAL_ISR(&spinlock);
      isr_counter--;
      portEXIT_CRITICAL_ISR(&spinlock);
    }
    
    vTaskDelay(delay_time);
  }  
}


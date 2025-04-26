#include <Arduino.h>

#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif
//<---- --------------------------------------------------------- ---->
static TimerHandle_t one_shot_timer = NULL;
static TimerHandle_t auto_reload_timer = NULL;
//<---- --------------------------------------------------------- ---->
void myTimeCallback(TimerHandle_t xTimer);
//<---- --------------------------------------------------------- ---->
void setup() 
{
    Serial.begin(300);
    Serial.println("<---- FreeRTOS Timer Demo ---->");

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    //create a one-shot timer
    one_shot_timer = xTimerCreate("ONE-SHOT", (2000 / portTICK_PERIOD_MS), pdFALSE, (void*)0, myTimeCallback);

    auto_reload_timer = xTimerCreate("AUTO-RELOAD", (1000 / portTICK_PERIOD_MS), pdTRUE, (void*)1, myTimeCallback);

    if(one_shot_timer == NULL || auto_reload_timer == NULL)
    {
        Serial.println("Could not create one of the timers");
    }
    else
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        Serial.println("Starting timer...");

        // start timer(max block time if command queue is full)
        xTimerStart(one_shot_timer, portMAX_DELAY);

        xTimerStart(auto_reload_timer, portMAX_DELAY);
    }

    vTaskDelete(NULL);
}
//<---- --------------------------------------------------------- ---->
void loop() 
{}
//<---- --------------------------------------------------------- ---->
void myTimeCallback(TimerHandle_t xTimer)
{
    if((uint32_t)pvTimerGetTimerID(xTimer) == 0)
    {
        Serial.println("one-shot timer Expired!");
    }
    else if((uint32_t)pvTimerGetTimerID(xTimer) == 1)
    {
        Serial.println("auto-reload timer Expired!");
    }
}


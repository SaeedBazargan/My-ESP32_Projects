#include <Arduino.h>
#include "esp_camera.h"
#include "soc/soc.h"            // Disable brownout detector
#include "soc/rtc_cntl_reg.h"

// <---- ------ Camera Pin Definitions ------ ---->
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y2_GPIO_NUM        5
#define Y3_GPIO_NUM       18
#define Y4_GPIO_NUM       19
#define Y5_GPIO_NUM       21
#define Y6_GPIO_NUM       36
#define Y7_GPIO_NUM       39
#define Y8_GPIO_NUM       34
#define Y9_GPIO_NUM       35
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout reset

  pinMode(4, OUTPUT);
  Serial.begin(115200);   // Open Serial communication
  Serial.setDebugOutput(false);

  // <---- ------ Camera configuration ------ ---->
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  if(psramFound())
  {
    config.frame_size = FRAMESIZE_VGA;   // 640x480
    Serial.println("HELLLLLLLLLLLLLLLLLLLLLLLLLLO");
    config.jpeg_quality = 10;
    config.fb_count = 2;
  }
  else
  {
    config.frame_size = FRAMESIZE_CIF;   // 352x288
    Serial.println("BYYYYYYYYYYYYYYYYYYYYYE");
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  if(esp_camera_init(&config) != ESP_OK)
  {
    Serial.println("Camera init failed");
    while(1);
  }

  delay(2000); // Wait for setup stabilization

  // digitalWrite(4, HIGH);
}

void loop()
{
  camera_fb_t *fb = esp_camera_fb_get(); // Capture frame

  if(!fb)
  {
    Serial.println("Camera capture failed");
    delay(1000);
    
    return;
  }

  // <---- ------ Send frame size first ------ ---->
  uint32_t img_len = fb->len;
  Serial.write((uint8_t*)&img_len, sizeof(img_len)); // Send image size (4 bytes)

  // <---- ------ Send actual image data ------ ---->
  Serial.write(fb->buf, fb->len); // Send raw JPEG bytes

  esp_camera_fb_return(fb); // Release the frame buffer

  delay(2000); // Take a photo every 2 seconds
}

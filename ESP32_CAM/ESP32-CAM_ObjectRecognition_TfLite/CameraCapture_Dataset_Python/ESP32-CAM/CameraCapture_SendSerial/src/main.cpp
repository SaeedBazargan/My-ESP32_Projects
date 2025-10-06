#include <Arduino.h>
#include "esp_camera.h"

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

int bytes_per_frame;
int bytes_per_pixel;
uint8_t camera_Width = 160;
uint8_t camera_Height = 120;

template <typename T>
inline T clamp_0_255(T x)
{
  return std::max(std::min(x, static_cast<T>(255)), static_cast<T>(0));
}

inline void ycbcr422_rgb888(int32_t Y, int32_t Cb, int32_t Cr, uint8_t* out)
{
  Cr -= 128;
  Cb -= 128;

  out[0] = clamp_0_255(Y + Cr + (Cr >> 2) + (Cr >> 3) + (Cr >> 5)); // R
  out[1] = clamp_0_255(Y - ((Cb >> 2) + (Cb >> 4) + (Cb >> 5)) - ((Cr >> 1) + (Cr >> 3) + (Cr >> 4)) + (Cr >> 5)); // G
  out[2] = clamp_0_255(Y + Cb + (Cb >> 1) + (Cb >> 2) + (Cb >> 6)); // B
}

void setup()
{
  pinMode(4, OUTPUT);
  Serial.begin(115200);

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
  config.pixel_format = PIXFORMAT_YUV422;
  config.frame_size = FRAMESIZE_QQVGA;   // 160 x 120
  config.jpeg_quality = 10;
  config.fb_count = 1;

  if(esp_camera_init(&config) != ESP_OK)
  {
    Serial.println("Camera init failed");
    while(1);
  }

  bytes_per_pixel = 2;
  bytes_per_frame = camera_Width * camera_Height * bytes_per_pixel;

  delay(2000); // Wait for setup stabilization
}

void loop()
{
  camera_fb_t *fb = esp_camera_fb_get(); // Capture frame
  if(!fb)
  {
    Serial.println("Camera capture failed");
    
    return;
  }

  // <---- ------ Send start marker "<image>" ------ ---->
  Serial.println("<image>");
  Serial.println(camera_Width);
  Serial.println(camera_Height);
  
  uint8_t rgb888[3];
  const int step_bytes = bytes_per_pixel * 2;
  
  for(int i = 0; i < bytes_per_frame; i += 4)
  {
    // Note: U and V are swapped
    const int32_t Y0 = fb->buf[i + 0];
    const int32_t Cr = fb->buf[i + 1];
    const int32_t Y1 = fb->buf[i + 2];
    const int32_t Cb = fb->buf[i + 3];

    ycbcr422_rgb888(Y0, Cb, Cr, rgb888);
    Serial.write(rgb888, 3);
    
    ycbcr422_rgb888(Y1, Cb, Cr, rgb888);
    Serial.write(rgb888, 3);
  }

  Serial.println("</image>");

  esp_camera_fb_return(fb); // Release the frame buffer
  delay(3000);
}

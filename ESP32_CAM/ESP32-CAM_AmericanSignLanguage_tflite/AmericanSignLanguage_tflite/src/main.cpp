#include <Arduino.h>
#include "esp_camera.h"

#include <TensorFlowLite_ESP32.h>

#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/c/common.h"
// #include "tensorflow/lite/micro/all_ops_resolver.h"

#include "MyAmericanSignLanguageMNIST_model.h"


#define CAMERA_MODEL_AI_THINKER // Has PSRAM

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

camera_fb_t * fb = NULL;
uint16_t *buffer;

size_t _jpg_buf_len = 0;
uint8_t * _jpg_buf = NULL;

// <---- -------- TFLite variables -------- ---->
constexpr int scratchBufSize = 0;

namespace
{
	tflite::MicroErrorReporter micro_error_reporter;
	tflite::ErrorReporter* error_reporter = &micro_error_reporter;
	const tflite::Model* model 		  	  = nullptr;
	tflite::MicroInterpreter* interpreter = nullptr;
	TfLiteTensor* model_input			  = nullptr;
	TfLiteTensor* model_output			  = nullptr;

	// <---- -------- Create an area of memory to use for input/output and other tensorflow arrays.
	// you will need to adjust this by compiling, running, and looking for errors  -------- ---->
	const int KTensorArenaSize = 81 * 1024 + scratchBufSize;
	uint8_t tensor_arena[KTensorArenaSize];
}

constexpr int kCategoryCount = 2;
const char* kCategoryLabels[24] =
{
  "A", "B", "C", "D", "E", "F", "G", "H", "I", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y"
};

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
  config.pixel_format = PIXFORMAT_RGB565;//PIXFORMAT_GRAYSCALE;//PIXFORMAT_JPEG;//PIXFORMAT_RGB565;// 
  config.frame_size = FRAMESIZE_96X96;//FRAMESIZE_QVGA;//FRAMESIZE_96X96;//
  config.jpeg_quality = 12;
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_PSRAM;

  if(esp_camera_init(&config) != ESP_OK)
  {
    Serial.println("Camera init failed");
    while(1);
  }

  delay(2000); // Wait for setup stabilization

  model = tflite::GetModel(MyAmericanSignLanguageMNIST_model);
  if(model->version() != TFLITE_SCHEMA_VERSION)
  {
	  error_reporter->Report("Model version does not match schema.");

    Serial.println("Model version does not match schema.");

	  while(1);
  }

  // static tflite::MicroMutableOpResolver<6> micro_op_resolver;
  // micro_op_resolver.AddConv2D();
  // micro_op_resolver.AddFullyConnected();
  // micro_op_resolver.AddSoftmax();
  // micro_op_resolver.AddReshape();
  // micro_op_resolver.AddMaxPool2D();
}

void loop()
{}


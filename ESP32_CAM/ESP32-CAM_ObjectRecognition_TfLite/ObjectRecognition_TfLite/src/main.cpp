#include <Arduino.h>
#include "esp_camera.h"

#include <TensorFlowLite_ESP32.h>

#include "tensorflow/lite/micro/all_ops_resolver.h"
// #include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/c/common.h"

#include "object_recognition.h"

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

// <---- ------ Labels for classification results ------ ---->
static const char *label[] = {"Ball", "Mag", "Unknown"};

uint8_t camera_Width = 160;
uint8_t camera_Height = 120;
int bytes_per_pixel = 2;
int bytes_per_frame = camera_Width * camera_Height * bytes_per_pixel;

// <---- ------ Variables for image scaling and preprocessing ------ ---->
int stride_in_y = camera_Width * bytes_per_pixel;    // Initialize resolution and Setup for image scaling and model input
static uint8_t Model_InputWidth = 48;
static uint8_t Model_InputHeight = 48;
static float scale_x = (float)camera_Height / (float)Model_InputWidth;
static float scale_y = (float)camera_Height / (float)Model_InputHeight;

// <---- -------- TFLite variables -------- ---->
namespace
{
	tflite::MicroErrorReporter micro_error_reporter;
	tflite::ErrorReporter* error_reporter = &micro_error_reporter;
	const tflite::Model* tflu_model            = nullptr;
	tflite::MicroInterpreter* interpreter = nullptr;
  TfLiteTensor* input_tensor                = nullptr;
  TfLiteTensor* output_tensor                = nullptr;

	// <---- -------- Create an area of memory to use for input/output and other tensorflow arrays.
	// you will need to adjust this by compiling, running, and looking for errors  -------- ---->
	const int KTensorArenaSize = 619840;
  static uint8_t *tensor_arena = nullptr;

  float tflu_scale     = 0.0f;
  int32_t tflu_zeropoint = 0;
}
TfLiteStatus tflite_status;


// <---- ------ Helper functions ------ ---->
// <---- ------ Helper function to limit values between 0 and 255 ------ ---->
template <typename T>
inline T clamp_0_255(T x)
{
  return std::max(std::min(x, static_cast<T>(255)), static_cast<T>(0));
}
inline void ycbcr422_rgb888(int32_t Y, int32_t Cb, int32_t Cr, uint8_t* out);
inline uint8_t bilinear_inter(uint8_t v00, uint8_t v01, uint8_t v10, uint8_t v11, float xi_f, float yi_f, int xi, int yi);
inline float rescale(float x, float scale, float offset);
inline int8_t quantize(float x, float scale, float zero_point);
void SamplingPointCoordinate(void);
void TFLite_Init(void);


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

  TFLite_Init();

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

  SamplingPointCoordinate();

  // Run inference
  tflite_status = interpreter->Invoke();
  if(tflite_status != kTfLiteOk)
  {
    Serial.println("Error invoking the TFLu interpreter");

    return;
  }

  size_t ix_max = 0;
  float  pb_max = 0;
  for(size_t ix = 0; ix < 3; ix++)
  {
    if(output_tensor->data.f[ix] > pb_max)
    {
      ix_max = ix;
      pb_max = output_tensor->data.f[ix];
    }
  }

  Serial.println(label[ix_max]);
  
  esp_camera_fb_return(fb); // Release the frame buffer
}

// <---- ------ Convert a single YCbCr422 pixel to RGB888 ------ ---->
inline void ycbcr422_rgb888(int32_t Y, int32_t Cb, int32_t Cr, uint8_t* out)
{
  Cr -= 128;
  Cb -= 128;

  out[0] = clamp_0_255(Y + Cr + (Cr >> 2) + (Cr >> 3) + (Cr >> 5)); // R
  out[1] = clamp_0_255(Y - ((Cb >> 2) + (Cb >> 4) + (Cb >> 5)) - ((Cr >> 1) + (Cr >> 3) + (Cr >> 4)) + (Cr >> 5)); // G
  out[2] = clamp_0_255(Y + Cb + (Cb >> 1) + (Cb >> 2) + (Cb >> 6)); // B
}

// <---- ------ Bilinear interpolation for downscaling pixels smoothly ------ ---->
inline uint8_t bilinear_inter(uint8_t v00, uint8_t v01, uint8_t v10, uint8_t v11, float xi_f, float yi_f, int xi, int yi)
{
    const float a  = (xi_f - xi);
    const float b  = (1.f - a);
    const float a1 = (yi_f - yi);
    const float b1 = (1.f - a1);

    // Calculate the output
    return clamp_0_255((v00 * b * b1) + (v01 * a * b1) + (v10 * b * a1) + (v11 * a * a1));
}

// <---- ------ Normalize pixel value from 0–255 to a different range ------ ---->
inline float rescale(float x, float scale, float offset)
{
  return (x * scale) - offset;
}

// <---- ------ Quantize floating-point value to int8 for TFLite model input ------ ---->
inline int8_t quantize(float x, float scale, float zero_point)
{
  return (x / scale) + zero_point;
}

// <---- ------ Calculate the corresponding sampling point position for each output coordinate ------ ---->
void SamplingPointCoordinate(void)
{
  uint8_t rgb888[3];
  int idx = 0;
  for(uint8_t yo = 0; yo < Model_InputHeight; yo++)
  {
    const float yi_f = (yo * scale_y);
    const int yi = (int)std::floor(yi_f);
    
    for(int xo = 0; xo < Model_InputWidth; xo++)
    {
      const float xi_f = (xo * scale_x);
      const int xi = (int)std::floor(xi_f);

      int x0 = xi;
      int y0 = yi;
      int x1 = std::min(xi + 1, camera_Height - 1);
      int y1 = std::min(yi + 1, camera_Height - 1);

      // Calculate the offset to access the Y component
      int ix_y00 = x0 * sizeof(int16_t) + y0 * stride_in_y;
      int ix_y01 = x1 * sizeof(int16_t) + y0 * stride_in_y;
      int ix_y10 = x0 * sizeof(int16_t) + y1 * stride_in_y;
      int ix_y11 = x1 * sizeof(int16_t) + y1 * stride_in_y;

      const int Y00 = fb->buf[ix_y00];
      const int Y01 = fb->buf[ix_y01];
      const int Y10 = fb->buf[ix_y10];
      const int Y11 = fb->buf[ix_y11];

      // Calculate the offset to access the Cr component
      const int offset_cr00 = xi % 2 == 0? 1 : -1;
      const int offset_cr01 = (xi + 1) % 2 == 0? 1 : -1;

      const int Cr00 = fb->buf[ix_y00 + offset_cr00];
      const int Cr01 = fb->buf[ix_y01 + offset_cr01];
      const int Cr10 = fb->buf[ix_y10 + offset_cr00];
      const int Cr11 = fb->buf[ix_y11 + offset_cr01];

      // Calculate the offset to access the Cb component
      const int offset_cb00 = offset_cr00 + 2;
      const int offset_cb01 = offset_cr01 + 2;

      const int Cb00 = fb->buf[ix_y00 + offset_cb00];
      const int Cb01 = fb->buf[ix_y01 + offset_cb01];
      const int Cb10 = fb->buf[ix_y10 + offset_cb00];
      const int Cb11 = fb->buf[ix_y11 + offset_cb01];

      uint8_t rgb00[3];
      uint8_t rgb01[3];
      uint8_t rgb10[3];
      uint8_t rgb11[3];

      // Convert YCbCr422 to RGB888
      ycbcr422_rgb888(Y00, Cb00, Cr00, rgb00);
      ycbcr422_rgb888(Y01, Cb01, Cr01, rgb01);
      ycbcr422_rgb888(Y10, Cb10, Cr10, rgb10);
      ycbcr422_rgb888(Y11, Cb11, Cr11, rgb11);

      // Iterate over the RGB channels
      uint8_t c_i;
      float c_f;
      int8_t c_q;
      for(uint8_t i = 0; i < 3; i++)
      {
        c_i = bilinear_inter(rgb00[i], rgb01[i], rgb10[i], rgb11[i], xi_f, yi_f, xi, yi);
        c_f = rescale((float)c_i, 1.f/255.f, -1.f);
        c_q = quantize(c_f, tflu_scale, tflu_zeropoint);
        input_tensor->data.int8[idx++] = c_q;
      }
    }
  }
}

// <---- ------ Initialize TensorFlow Lite Micro components ------ ---->
void TFLite_Init(void)
{
  Serial.println("TFLu initialization - start");

  uint8_t *tensor_arena = (uint8_t *)malloc(KTensorArenaSize);

  // Load the TFLITE model
  tflu_model = tflite::GetModel(object_recognition_model_tflite);
  if(tflu_model->version() != TFLITE_SCHEMA_VERSION)
  {
    Serial.print(tflu_model->version());
    Serial.println("");
    Serial.print(TFLITE_SCHEMA_VERSION);
    Serial.println("Model version does not match schema.");

    while(1);
  }

  tflite::AllOpsResolver tflu_ops_resolver;

  // <---- -------- build an interpreter to run the model -------- ---->
  static tflite::MicroInterpreter static_interpreter(tflu_model, tflu_ops_resolver, tensor_arena, KTensorArenaSize, error_reporter);
  interpreter = &static_interpreter;

  // <---- -------- Allocate memory from the tensor_arena for the model's tensors. -------- ---->
  tflite_status = interpreter->AllocateTensors();
  if(tflite_status != kTfLiteOk)
  {
	  error_reporter->Report("AllocateTensors failed.");

    Serial.println("AllocateTensors failed.");

	  while(1);
  }

  // <---- -------- Allocate model input/output buffers(tensors) to pointers -------- ---->
  input_tensor  = interpreter->input(0);
  output_tensor = interpreter->output(0);  

  // <---- -------- Get number of parameters in input tensor -------- ---->
  // Retrieve quantization parameters from model input tensor and Get the quantization parameters (per-tensor quantization)
  const auto* i_quantization = reinterpret_cast<TfLiteAffineQuantization*>(input_tensor->quantization.params);
  tflu_scale     = i_quantization->scale->data[0];
  tflu_zeropoint = i_quantization->zero_point->data[0];

  Serial.println("TFLu initialization - completed");
}
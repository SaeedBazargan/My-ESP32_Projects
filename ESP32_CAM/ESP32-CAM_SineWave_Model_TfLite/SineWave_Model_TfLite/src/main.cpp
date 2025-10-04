#include <Arduino.h>
#include <TensorFlowLite_ESP32.h>

#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/c/common.h"
// #include "tensorflow/lite/micro/all_ops_resolver.h"

#include "sine_model.h"

// <---- -------- TFLite variables -------- ---->
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
	const int KTensorArenaSize = 2 * 1024;
	uint8_t tensor_arena[KTensorArenaSize];
}

char buf[50];
int buf_len = 0;
TfLiteStatus tflite_status;
uint32_t num_elements;
uint32_t timestamp;
float y_val;

void setup()
{
  Serial.begin(115200);

  error_reporter->Report("ESP32-CAM TFLite Test.");

  Serial.println("ESP32-CAM TFLite Test.");

  model = tflite::GetModel(sine_model);
  if(model->version() != TFLITE_SCHEMA_VERSION)
  {
	  error_reporter->Report("Model version does not match schema.");

    Serial.println("Model version does not match schema.");

	  while(1);
  }

  // <---- -------- Pull in only needed operations (should match with your NN layers).
  // Template parameter (<n>) is number of ops to be added.
  // Available ops: tensorflow/lite/micro/kernels/micro_ops.h -------- ---->
  static tflite::MicroMutableOpResolver<1> micro_op_resolver;

  // <---- -------- Add dense NN layer operation -------- ---->
  tflite_status = micro_op_resolver.AddFullyConnected();
  if(tflite_status != kTfLiteOk)
  {
	  error_reporter->Report("Couldn't add fully connected op.");

    Serial.println("Couldn't add fully connected op.");

    while(1);
  }

  // <---- -------- build an interpreter to run the model -------- ---->
  static tflite::MicroInterpreter static_interpreter(model, micro_op_resolver, tensor_arena, KTensorArenaSize, error_reporter);
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
  model_input  = interpreter->input(0);
  model_output = interpreter->output(0);

  // <---- -------- Get number of elements in input tensor -------- ---->
  num_elements = model_input->bytes / sizeof(float);
  Serial.print("Number of input elements: ");
  Serial.println(num_elements);
}

void loop()
{
  // <---- -------- Fill input buffer (use test value) -------- ---->
  for(uint32_t i = 0; i < num_elements; i++)
  {
    model_input->data.f[i] = 2.0f;
  }

  // <---- -------- Get current timestamp -------- ---->
  unsigned long timestamp = millis();

  // <---- -------- Run inference -------- ---->
  tflite_status = interpreter->Invoke();
  if(tflite_status != kTfLiteOk)
  {
    error_reporter->Report("Invoke failed.");

    Serial.println("Invoke failed.");
  }

  // <---- -------- Read output (predict y) of neural network -------- ---->
  y_val = model_output->data.f[0];

  unsigned long duration = millis() - timestamp;
  Serial.print("Output: ");
  Serial.print(y_val, 6);   // 6 decimal precision
  Serial.print(" | Duration: ");
  Serial.print(duration);
  Serial.println(" ms");

  delay(500);
}





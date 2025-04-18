#include <Arduino.h>

#include "TensorFlowLite.h"
#include "tensorflow/lite/experimental/micro/kernels/micro_ops.h"
#include "tensorflow/lite/experimental/micro/micro_error_reporter.h"
#include "tensorflow/lite/experimental/micro/micro_interpreter.h"
#include "tensorflow/lite/experimental/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/version.h"

#include "imu_model.h"  // <-- Your model header file

#define led LED_BUILTIN

namespace {
  tflite::ErrorReporter* error_reporter = nullptr;
  const tflite::Model* model = nullptr;
  tflite::MicroInterpreter* interpreter = nullptr;
  TfLiteTensor* model_input = nullptr;
  TfLiteTensor* model_output = nullptr;

  constexpr int KTensorArenaSize = 5 * 1024;
  uint8_t tensor_arena[KTensorArenaSize];

  const char* label_names[] = { "Up", "Down", "Right", "Left" };
}

void setup() {
  Serial.begin(115200);
  pinMode(led, OUTPUT);
  Serial.println("<---- Setup starting ---->");

  // Error reporter
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;

  // Load model
  model = tflite::GetModel(imu_model_tflite);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    error_reporter->Report("Model version mismatch!");
    while (1);
  }

  // Resolver (non-template version)
  static tflite::MicroMutableOpResolver micro_mutable_op_resolver;
  micro_mutable_op_resolver.AddBuiltin(tflite::BuiltinOperator_FULLY_CONNECTED,
                                       tflite::ops::micro::Register_FULLY_CONNECTED());
  micro_mutable_op_resolver.AddBuiltin(tflite::BuiltinOperator_RELU,
                                       tflite::ops::micro::Register_RELU());
  micro_mutable_op_resolver.AddBuiltin(tflite::BuiltinOperator_SOFTMAX,
                                       tflite::ops::micro::Register_SOFTMAX());
  micro_mutable_op_resolver.AddBuiltin(tflite::BuiltinOperator_RESHAPE,
                                       tflite::ops::micro::Register_RESHAPE());

  // Interpreter
  static tflite::MicroInterpreter static_interpreter(
      model, micro_mutable_op_resolver, tensor_arena, KTensorArenaSize, error_reporter);
  interpreter = &static_interpreter;

  // Tensor allocation
  if (interpreter->AllocateTensors() != kTfLiteOk) {
    error_reporter->Report("AllocateTensors() failed");
    while (1);
  }

  model_input = interpreter->input(0);
  model_output = interpreter->output(0);

  Serial.println("Setup complete. Starting inference...");
}

void loop() {
  float roll = 110.0;
  float pitch = 98.0;

  model_input->data.f[0] = roll;
  model_input->data.f[1] = pitch;

  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    Serial.println("Invoke failed!");
    return;
  }

  int best_class = 0;
  float best_score = model_output->data.f[0];
  for (int i = 1; i < model_output->dims->data[1]; i++)
  {
    if (model_output->data.f[i] > best_score)
    {
      best_score = model_output->data.f[i];
      best_class = i;
    }
  }

  Serial.print("Roll: "); Serial.print(roll, 2);
  Serial.print(" | Pitch: "); Serial.print(pitch, 2);
  Serial.print(" => Predicted: ");
  // Serial.println(label_names[best_class]);
  Serial.print("Best_Class: ");
  Serial.println(best_class);

  digitalWrite(led, HIGH);
  delay(500);
  digitalWrite(led, LOW);
  delay(500);
}

#include <Arduino.h>

#define LED           4

void setup()
{
  pinMode(LED, OUTPUT);

  Serial.begin(9600);
}

void loop()
{
  digitalWrite(LED, HIGH);
  delay(500);
  Serial.println("<---- ----- ON ----- ---->");
  digitalWrite(LED, LOW);
  delay(500);
  Serial.println("<---- ----- OFF ----- ---->");
}


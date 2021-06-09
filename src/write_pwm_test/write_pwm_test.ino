#include <Servo.h>

const uint8_t pin = 9;

Servo motor_test;

void setup()
{
  motor_test.attach(pin, 1000, 2000);
  Serial.begin(115200);
}

void loop()
{
  uint16_t input = 0;
  if (Serial.available() > 0)
  {
    input = Serial.parseInt();

    Serial.print("Received: ");
    Serial.println(input);
  }
  motor_test.writeMicroseconds(input);
}

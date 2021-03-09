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
    if (Serial.available() > 0)
    {
        input = Serial.parseInt();

        Serial.print("Received: ");
        Serial.println(input);
    }
    motor_left.writeMicroseconds(input);
}
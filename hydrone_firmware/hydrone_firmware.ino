#include <Servo.h>
#include <SparkFunLSM6DS3.h>
#include <Wire.h>

#define PRINT_PWM 1
#define LED 13

// IMU config
LSM6DS3 imu_sensor(I2C_MODE, 0x6B);

uint16_t init_time;
const uint16_t TIME_STEP_MICROS = 10000; // 10 ms period -> 100 Hz sampling rate.
unsigned long previousTime = 0;

// Calibrated pulse width for FrSky Taranis Q X7 16CH 2.4Ghz transmitter with RX8R receiver
const uint16_t RC_min[4] = {1000, 1000, 1000, 1000};
const uint16_t RC_mid[4] = {1500, 1500, 1500, 1500};
const uint16_t RC_max[4] = {2000, 2000, 2000, 2000};

uint8_t last_channel_1_state = 0, last_channel_2_state = 0, last_channel_3_state = 0, last_channel_4_state = 0;
uint16_t receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
uint32_t timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4;

uint32_t last_rc_update = 0;
Servo motor_left, motor_right, control_toggle_left, gopro_toggle_right;

// Interrupt service routine called evey time digital input pin 8, 9, 10, or 11 changes state
// PCINT0_vect is the compiler vector for PCINT0 on PORTB of ATmega328
ISR(PCINT0_vect)
{
  //    // Channel 1==================================
  if (last_channel_1_state == 0 && PINB & B00000001)
  {
    last_channel_1_state = 1;
    timer_channel_1 = micros();
  }
  if (last_channel_1_state == 1 && !(PINB & B00000001))
  {
    last_channel_1_state = 0;
    receiver_input_channel_1 = micros() - timer_channel_1;
  }
  // Channel 2==================================
  if (last_channel_2_state == 0 && PINB & B00000010)
  {
    last_channel_2_state = 1;
    timer_channel_2 = micros();
  }
  if (last_channel_2_state == 1 && !(PINB & B00000010))
  {
    last_channel_2_state = 0;
    receiver_input_channel_2 = micros() - timer_channel_2;
  }
  //    // Channel 3==================================
  if (last_channel_3_state == 0 && PINB & B00000100)
  {
    last_channel_3_state = 1;
    timer_channel_3 = micros();
  }
  if (last_channel_3_state == 1 && !(PINB & B00000100))
  {
    last_channel_3_state = 0;
    receiver_input_channel_3 = micros() - timer_channel_3;
  }
  //    // Channel 4==================================
  if (last_channel_4_state == 0 && PINB & B00001000)
  {
    last_channel_4_state = 1;
    timer_channel_4 = micros();
  }
  if (last_channel_4_state == 1 && !(PINB & B00001000))
  {
    last_channel_4_state = 0;
    receiver_input_channel_4 = micros() - timer_channel_4;
  }
}

//Subroutine for displaying the receiver signals
//Subroutine for displaying the receiver signals
void readReceiverSignals()
{
  Serial.print("Channel 1:");
  if (receiver_input_channel_1 - 1480 < 0)
    Serial.print("<<<");
  else if (receiver_input_channel_1 - 1520 > 0)
    Serial.print(">>>");
  else
    Serial.print("-+-");
  Serial.print(receiver_input_channel_1);

  Serial.print("  Channel 2:");
  if (receiver_input_channel_2 - 1480 < 0)
    Serial.print("^^^");
  else if (receiver_input_channel_2 - 1520 > 0)
    Serial.print("vvv");
  else
    Serial.print("-+-");
  Serial.print(receiver_input_channel_2);

  Serial.print("  Channel 3:");
  if (receiver_input_channel_3 - 1480 < 0)
    Serial.print("vvv");
  else if (receiver_input_channel_3 - 1520 > 0)
    Serial.print("^^^");
  else
    Serial.print("-+-");
  Serial.print(receiver_input_channel_3);

  Serial.print("  Channel 4:");
  if (receiver_input_channel_4 - 1480 < 0)
    Serial.print("<<<");
  else if (receiver_input_channel_4 - 1520 > 0)
    Serial.print(">>>");
  else
    Serial.print("-+-");
  Serial.println(receiver_input_channel_4);
}

void inputStep()
{
  long end_time = micros() + 5000000;
  while (end_time > micros())
  {
    /// Begin Step input
    motor_left.writeMicroseconds(1550);
    motor_right.writeMicroseconds(1450);

    /// Collect IMU data
    getData();
  }
}

void getData()
{
  //millis() is not suitable for precise timing as it has a lot of jitter
  if (micros() - previousTime > TIME_STEP_MICROS)
  {
    previousTime = micros();
    Serial.print(F("Data:"));
    Serial.print(imu_sensor.readFloatAccelX(), 4);
    Serial.print(",");
    Serial.print(imu_sensor.readFloatAccelY(), 4);
    Serial.print(",");
    Serial.print(imu_sensor.readFloatAccelZ(), 4);
    Serial.print(",");
    Serial.print(imu_sensor.readFloatGyroX(), 4);
    Serial.print(",");
    Serial.print(imu_sensor.readFloatGyroY(), 4);
    Serial.print(",");
    Serial.print(imu_sensor.readFloatGyroZ(), 4);
    Serial.println("");
  }
}

void setup()
{
  // ATmega pins default to inputs, therefore there is no need to use the pinMode function
  PCICR |= (1 << PCIE0);   // set PCIE0 bit to to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0); // set PCINT0 (digital pin 8) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT1); // set PCINT0 (digital pin 9) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT2); // set PCINT0 (digital pin 10) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT3); // set PCINT0 (digital pin 11) to trigger an interrupt on state change

  Serial.begin(115200);

  // Accelerometer Config
  imu_sensor.settings.accelEnabled = 1;
  imu_sensor.settings.accelRange = 2;        //Max G force readable.  Can be: 2, 4, 8, 16
  imu_sensor.settings.accelSampleRate = 104; //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
  imu_sensor.settings.accelBandWidth = 50;   //Hz.  Can be: 50, 100, 200, 400;
  imu_sensor.settings.accelFifoEnabled = 0;

  // Gyro Config
  imu_sensor.settings.gyroEnabled = 1;
  imu_sensor.settings.gyroRange = 125;      // [dps].  Can be: 125, 245, 500, 1000, 2000
  imu_sensor.settings.gyroSampleRate = 104; // [Hz].  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  imu_sensor.settings.gyroFifoEnabled = 0;

  // Temp
  imu_sensor.settings.tempEnabled = 0;

  motor_left.attach(5, RC_min[1], RC_max[1]);
  motor_right.attach(6, RC_min[2], RC_max[2]);

  init_time = micros();
  delay(500);
}

void loop()
{
  if (millis() - last_rc_update > 25)
  {
    motor_left.writeMicroseconds(receiver_input_channel_1);
    motor_right.writeMicroseconds(receiver_input_channel_2);
    last_rc_update = millis();

    if (PRINT_PWM)
    {
      readReceiverSignals();
      delay(250);
    }

    if (receiver_input_channel_3 > 1520)
    {
      inputStep();
    }
  }
}

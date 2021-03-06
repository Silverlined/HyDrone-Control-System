// Libs for HyDrone Controll
#include <Servo.h>
#include <SparkFunLSM6DS3.h>
#include <Wire.h>
#include "PollTimer.h"

// Libs for ToF sensor
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <math.h>
#define TCAADDR 0x70
#define DISTANCE_45 0   // connected to serial bus 0
#define DISTANCE_90 1

// Libs for filtering
#include <Filters.h>
#include <AH/Timing/MillisMicrosTimer.hpp>
#include <Filters/Butterworth.hpp>

// Sampling frequency
const float f_s_gyro = 100; // [Hz]
const float f_s_tof = 10; // [Hz]
// Cut-off frequency (-3 dB)
const float f_c_gyro = 3;
const float f_c_tof = 1;
// Normalized cut-off frequency
const float f_n_gyro = 2 * f_c_gyro / f_s_gyro;
const float f_n_tof = 2 * f_c_tof / f_s_tof;

auto filterGyro = butter<6>(f_n_gyro);
auto filterToF = butter<6>(f_n_tof);

PollTimer gyroTimer(100); // [Hz]
PollTimer pidTimer(10);   // [Hz]

SFEVL53L1X distance;

void tcaselect(uint8_t i) {   // switches between different i2c busses.
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

#define UPPER_THRESHOLD 1520
#define LOWER_THRESHOLD 1480
#define DEBUG 1
#define LED 13

bool isRecording = false; 
// IMU config
LSM6DS3 imu_sensor(I2C_MODE, 0x6B);
//const float GYRO_CALIBRATION[3] = {0, 0, 0};
//const float ACCEL_CALIBRATION[3] = {0, 0, 0};
const float GYRO_CALIBRATION[3] = {0.12, -0.06, -0.14};
const float ACCEL_CALIBRATION[3] = {0.0, 0.0, 0.0};

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

float orientation = 0; // Orientation of the HyDrone

const float dt = 0.01; // 100 [Hz]
const float Kp = 2;
const int dWall = 700; // [mm]
const int wallLead = 2000; // [mm]
const int speed = 1600; // [micro seconds, Pulse]

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
  if (receiver_input_channel_1 - LOWER_THRESHOLD < 0)
    Serial.print("<<<");
  else if (receiver_input_channel_1 - UPPER_THRESHOLD > 0)
    Serial.print(">>>");
  else
    Serial.print("-+-");
  Serial.print(receiver_input_channel_1);

  Serial.print("  Channel 2:");
  if (receiver_input_channel_2 - LOWER_THRESHOLD < 0)
    Serial.print("^^^");
  else if (receiver_input_channel_2 - UPPER_THRESHOLD > 0)
    Serial.print("vvv");
  else
    Serial.print("-+-");
  Serial.print(receiver_input_channel_2);

  Serial.print("  Channel 3:");
  if (receiver_input_channel_3 - LOWER_THRESHOLD < 0)
    Serial.print("vvv");
  else if (receiver_input_channel_3 - UPPER_THRESHOLD > 0)
    Serial.print("^^^");
  else
    Serial.print("-+-");
  Serial.print(receiver_input_channel_3);

  Serial.print("  Channel 4:");
  if (receiver_input_channel_4 - LOWER_THRESHOLD < 0)
    Serial.print("<<<");
  else if (receiver_input_channel_4 - UPPER_THRESHOLD > 0)
    Serial.print(">>>");
  else
    Serial.print("-+-");
  Serial.println(receiver_input_channel_4);
}

void inputStep()
{
  long _timer_10s = micros() + 10000000;
  Serial.print(F("Begin"));
  Serial.println("");
  delay(200);
  while (_timer_10s > micros())
  {
    /// Begin Step input
    motor_left.writeMicroseconds(1750);
    motor_right.writeMicroseconds(1250);

    /// Collect IMU data, disable pin change interrupts
    if (micros() - previousTime > TIME_STEP_MICROS)
    {
      getData();
    }
  }
  Serial.print(F("Stop"));
  Serial.println("");
}

void getData()
{
  //millis() is not suitable for precise timing as it has a lot of jitter
  previousTime = micros();
  PCICR &= ~(1 << PCIE0);   /// Disable Pin Change Interrupts
  Serial.print(F("Data:"));
  Serial.print(imu_sensor.readFloatAccelX() - ACCEL_CALIBRATION[0], 4);
  Serial.print(",");
  Serial.print(imu_sensor.readFloatAccelY() - ACCEL_CALIBRATION[1], 4);
  Serial.print(",");
  Serial.print(imu_sensor.readFloatAccelZ() - ACCEL_CALIBRATION[2], 4);
  Serial.print(",");
  Serial.print(imu_sensor.readFloatGyroX() - GYRO_CALIBRATION[0], 4);
  Serial.print(",");
  Serial.print(imu_sensor.readFloatGyroY() - GYRO_CALIBRATION[1], 4);
  Serial.print(",");
  Serial.print(imu_sensor.readFloatGyroZ() - GYRO_CALIBRATION[2], 4);
  Serial.println("");
  PCICR |= (1 << PCIE0); /// Enable Pin Change Interrupts
}

void initDistanceSensors() 
{
  //Initialize ToF sensors
  tcaselect(DISTANCE_45);
  if (distance.begin() != 0)
  {
    Serial.println("Failed to initialize ToF 90. Waiting...");
    while (1);
  }
  distance.setDistanceModeLong(); 

  tcaselect(DISTANCE_90);
  if (distance.begin() != 0)
  {
    Serial.println("Failed to initialize ToF45. Waiting...");
    while (1);
  }
  distance.setDistanceModeLong(); 
}

int getDistance(int sensor_id) {  // gets ToF sensor
  tcaselect(sensor_id);

  distance.startRanging(); //Write configuration bytes to initiate measurement
  while (!distance.checkForDataReady());
  int distance_val = distance.getDistance(); //Get the result of the measurement from the sensor
  distance.clearInterrupt();
  distance.stopRanging();

  return distance_val;
}

int getOrientationError(int d90, int d45) {
  float angle = orientation * DEG_TO_RAD;
  float l = cos(angle) * d90;
  float p = l - dWall;
  float beta = atan2(p, wallLead);
  
  return (beta - angle) * RAD_TO_DEG;
}

int clamp(int pwm) {
  if (pwm < 980 || pwm >2020) {
      pwm = 1500;
    }
    return pwm;
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
  Wire.begin();
  delay(500);
  if ( imu_sensor.begin() != 0 ) {
    Serial.print(F("Failed to detect IMU\n"));
    while (1);
  }
  else Serial.print(F("IMU initialized\n"));

  // Accelerometer Config
  imu_sensor.settings.accelEnabled = 0;

  // Temp Config
  imu_sensor.settings.tempEnabled = 0;

  // Gyro Config
  imu_sensor.settings.gyroEnabled = 1;
  imu_sensor.settings.gyroRange = 125;      // [dps].  Can be: 125, 245, 500, 1000, 2000
  imu_sensor.settings.gyroSampleRate = 104; // [Hz].  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  imu_sensor.settings.gyroFifoEnabled = 0;


  motor_left.attach(5, RC_min[1], RC_max[1]);
  motor_right.attach(6, RC_min[2], RC_max[2]);

  initDistanceSensors();  // Initialize Time of Flight Sensors

  gyroTimer.start();
  pidTimer.start();
  init_time = micros();
  delay(500);
}

void loop()
{
  if (receiver_input_channel_3 > UPPER_THRESHOLD && !isRecording)
  {
    //inputStep();
    Serial.print(F("Begin"));
    Serial.println("");
    isRecording = true;
    delay(200);
  } else if (receiver_input_channel_3 < LOWER_THRESHOLD && isRecording) 
  {
    Serial.print(F("Stop"));
    Serial.println("");
    isRecording = false;
    delay(200);
  }
  
  if (receiver_input_channel_4 > UPPER_THRESHOLD) {
    if(gyroTimer.check()) 
    {
      float yaw_rate = imu_sensor.readFloatGyroZ() - GYRO_CALIBRATION[2];
      yaw_rate = filterGyro(yaw_rate);
      if (abs(yaw_rate) > 0.05) {
        orientation += yaw_rate * dt;
        Serial.print(F("Orientation:"));
        Serial.println(orientation);
      }
    }

    if (pidTimer.check())
    {
      int distance45 = getDistance(DISTANCE_45);
      int distance90 = filterToF(getDistance(DISTANCE_90));
 
      float error = getOrientationError(distance90, distance45);
      if (abs(error) < 5) error = 0;
      if (error > 30) error = 30;
      if (error < -30) error = -30;
      

      float out = Kp * error;
      Serial.println(out);
      if (out > 0) {
        motor_left.writeMicroseconds(speed - out);
        motor_right.writeMicroseconds(speed + out);
      } else if (out < 0) {
        motor_left.writeMicroseconds(speed + out);
        motor_right.writeMicroseconds(speed - out);
      } else {
        motor_left.writeMicroseconds(speed);
        motor_right.writeMicroseconds(speed);
      }
      
      if (DEBUG) {
        Serial.println("Distance 1: " + String(distance45) + "\tDistance 2: " + String(distance90));
      }
    }
    return;
  }
  
  orientation = 0;
  if (millis() - last_rc_update > 25)
  {
    receiver_input_channel_1 = clamp(receiver_input_channel_1);
    receiver_input_channel_2 = clamp(receiver_input_channel_2);
    motor_left.writeMicroseconds(receiver_input_channel_1);
    motor_right.writeMicroseconds(receiver_input_channel_2);
    last_rc_update = millis();

    if (DEBUG)
    {
      readReceiverSignals();
      delay(250);
    }
  }
}

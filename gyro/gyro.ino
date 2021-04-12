#include <SparkFunLSM6DS3.h>
#include <Wire.h>
//#include <SPI.h>

LSM6DS3 imu_sensor( I2C_MODE, 0x6B );
//LSM6DS3 imu_sensor( SPI_MODE, 10 );


uint16_t init_time;
const uint16_t TIME_STEP_MICROS = 10400; // 10 ms period -> 100 Hz sampling rate.
unsigned long previousTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(500);
  if ( imu_sensor.begin() != 0 ) {
    Serial.print("Failed to detect IMU\n");
    while (1);
  }
  else Serial.print("IMU initialized\n");

  // Accelerometer Config
  imu_sensor.settings.accelEnabled = 1;
  imu_sensor.settings.accelRange = 8;      //Max G force readable.  Can be: 2, 4, 8, 16
  imu_sensor.settings.accelSampleRate = 104;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
  imu_sensor.settings.accelBandWidth = 200;  //Hz.  Can be: 50, 100, 200, 400;
  imu_sensor.settings.accelFifoEnabled = 0; 
  
  // Gyro Config
  imu_sensor.settings.gyroEnabled = 1;
  imu_sensor.settings.gyroRange = 125;   // [dps].  Can be: 125, 245, 500, 1000, 2000
  imu_sensor.settings.gyroSampleRate = 104;   // [Hz].  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  imu_sensor.settings.gyroFifoEnabled = 0;

  // Temp
  imu_sensor.settings.tempEnabled = 0;

  init_time = micros();
  delay(500);
}

// Spikes in IMU data can be caused by the code trying to read the sensor registers while they are being written.
// The IMU has 2 interrupt pins which can be used to see when new data is available to mitigate this issue. 

void loop() {
  if (micros() - previousTime > TIME_STEP_MICROS) { //millis() is not suitable for precise timing as it has a lot of jitter
    previousTime = micros();
    Serial.print(F("Data:"));
    Serial.println(imu_sensor.readFloatAccelX(), 4); //Serial.print(",");
//    Serial.print(imu_sensor.readFloatAccelY(), 4); Serial.print(",");
//    Serial.print(imu_sensor.readFloatAccelZ(), 4); Serial.print(",");
//    Serial.print(imu_sensor.readFloatGyroX(), 4); Serial.print(",");
//    Serial.print(imu_sensor.readFloatGyroY(), 4); Serial.print(",");
//    Serial.print(imu_sensor.readFloatGyroZ(), 4); Serial.println("");
  }
}

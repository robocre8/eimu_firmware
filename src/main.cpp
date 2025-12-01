#include <Arduino.h>
#include <SPI.h>
#include "command_functions.h"
#include "serial_comm.h"
#include "i2c_comm.h"

unsigned long serialCommTime, serialCommTimeInterval = 5; // ms -> (1000/sampleTime) hz
unsigned long readImuTime, readImuTimeInterval = 5;        // ms -> (1000/sampleTime) hz

void setup()
{
  Serial.begin(115200);
  // Serial.begin(460800);
  // Serial.begin(921600);

  //---------------- INITIALIZE IMU -----------------------//
   /* Start the SPI bus */
  SPI.begin();
  /* Initialize and configure IMU */
  if (!imu.Begin()) {
    // Serial.println("Error initializing communication with IMU");
    while(1) {}
  }
  /* Set the sample rate divider */
  if (!imu.ConfigSrd(19)) {
    // Serial.println("Error configured SRD");
    while(1) {}
  }
  //--------------------------------------------------------//

  loadStoredParams();

  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin(i2cAddress);

  madgwickFilter.setAlgorithmGain(filterGain);
  madgwickFilter.setWorldFrameId(worldFrameId); // 0 - NWU,  1 - ENU,  2 - NED

  pinMode(LED_PIN, OUTPUT);

  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);

  serialCommTime = millis();
  readImuTime = millis();
}

void loop()
{
  // Serial comm loop
  recieve_and_send_data();
  // if ((millis() - serialCommTime) >= serialCommTimeInterval)
  // {
  //   recieve_and_send_data();
  //   serialCommTime = millis();
  // }

  if ((millis() - readImuTime) >= readImuTimeInterval)
  {
    if (imu.Read()) {
      float _ax, _ay, _az;
      float _gx, _gy, _gz;

      //------------READ SENSOR DATA IN ENU FRAME---------------//
      accRaw[0] = imu.accel_y_mps2();
      accRaw[1] = imu.accel_x_mps2();
      accRaw[2] = -1.00 * imu.accel_z_mps2();

      gyroRaw[0] = imu.gyro_y_radps();
      gyroRaw[1] = imu.gyro_x_radps();
      gyroRaw[2] = -1.00 * imu.gyro_z_radps();
      //--------------------------------------------------------//

      //---------------CALIBRATE SENSOR DATA IN ENU FRAME -----------------//
      // calibrate acc data
      _ax = accRaw[0] - accOff[0];
      _ay = accRaw[1] - accOff[1];
      _az = accRaw[2] - accOff[2];

      // calibrate gyro data
      _gx = gyroRaw[0] - gyroOff[0];
      _gy = gyroRaw[1] - gyroOff[1];
      _gz = gyroRaw[2] - gyroOff[2];
      //-----------------------------------------------------//

      //------------- APPLY MADWICK FILTER -----------------//
    
      // filter is updated based on the choosen world frame
      switch (worldFrameId)
      {
      case 0: // NWU
        accCal[0] = _ay;
        accCal[1] = -1.00 * _ax;
        accCal[2] = _az;

        gyroCal[0] = _gy;
        gyroCal[1] = -1.00 * _gx;
        gyroCal[2] = _gz;
        break;

      case 1: // ENU
        accCal[0] = _ax;
        accCal[1] = _ay;
        accCal[2] = _az;

        gyroCal[0] = _gx;
        gyroCal[1] = _gy;
        gyroCal[2] = _gz;
        break;

      case 2: // NED
        accCal[0] = _ay;
        accCal[1] = _ax;
        accCal[2] = -1.00 * _az;

        gyroCal[0] = _gy;
        gyroCal[1] = _gx;
        gyroCal[2] = -1.00 * _gz;
        break;
      }

      madgwickFilter.madgwickAHRSupdateIMU(
          gyroCal[0], gyroCal[1], gyroCal[2], 
          accCal[0], accCal[1], accCal[2]
      );

      madgwickFilter.getOrientationRPY(roll, pitch, yaw);
      // madgwickFilter.getOrientationQuat(qw, qx, qy, qz);

      // randomSeed(millis());
      // int randGain = random(9, 11);
      // if (randGain < 10) randGain = -10;
      // randomGainMultiplier = (float)randGain/10.0;
      randomGainMultiplier = 1.0;

      if ((int)(yawVelDriftBias*100000.0) > 0) {
        yawAccumOffset += ((yawVelDriftBias*(float)readImuTimeInterval*randomGainMultiplier)/1000.0);
        rpy[0] = roll; rpy[1] = pitch; rpy[2] = yaw - yawAccumOffset;
      } else {
        rpy[0] = roll; rpy[1] = pitch; rpy[2] = yaw;
      }
      // quat[0] = qw; quat[1] = qx; quat[2] = qy; quat[3] = qz;
      //----------------------------------------------------//
    }

    readImuTime = millis();
  }
}
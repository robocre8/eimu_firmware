#include <Arduino.h>
#include <SPI.h>
#include "command_functions.h"
#include "vectlab.h"
#include "serial_comm.h"
#include "i2c_comm.h"

float MicroTeslaToTesla(float mT)
{
  return mT * 1000000;
}

void accLPFInit()
{
  for (int i = 0; i < 3; i += 1)
  {
    accLPF[i].setCutOffFreq(cutOffFreq);
  }
}

unsigned long serialCommTime, serialCommTimeInterval = 5; // ms -> (1000/sampleTime) hz
unsigned long readImuTime, readImuTimeInterval = 5;        // ms -> (1000/sampleTime) hz

void setup()
{
  loadStoredParams();

  Serial.begin(115200);
  // Serial.begin(460800);
  // Serial.begin(921600);

  //---------------- INITIALIZE IMU -----------------------//
  // start communication with IMU 
  SPI.begin();
  status = imu.begin();
  if (status < 0) {
    // Serial.println("IMU initialization unsuccessful");
    // Serial.println("Check IMU wiring or try cycling power");
    // Serial.print("Status: ");
    // Serial.println(status);
    while(1) {}
  }
  //--------------------------------------------------------//

  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin(i2cAddress);

  madgwickFilter.setAlgorithmGain(filterGain);
  madgwickFilter.setWorldFrameId(worldFrameId); // 0 - NWU,  1 - ENU,  2 - NED

  accLPFInit();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
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
    imu.readSensor();

    float _ax, _ay, _az;
    float _gx, _gy, _gz;
    float _mx, _my, _mz;
    float r, p, y;
    float qw, qx, qy, qz;
    float g=9.8, gx, gy, gz;

    //------------READ SENSOR DATA IN ENU FRAME---------------//
    accRaw[0] = imu.getAccelY_mss();
    accRaw[1] = imu.getAccelX_mss();
    accRaw[2] = -1.00 * imu.getAccelZ_mss();

    gyroRaw[0] = imu.getGyroY_rads();
    gyroRaw[1] = imu.getGyroX_rads();
    gyroRaw[2] = -1.00 * imu.getGyroZ_rads();

    magRaw[0] = imu.getMagY_uT();
    magRaw[1] = imu.getMagX_uT();
    magRaw[2] = -1.00 * imu.getMagZ_uT();
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

    // mag_vect = magRaw - b_vect
    mag_vect[0] = magRaw[0] - magBvect[0];
    mag_vect[1] = magRaw[1] - magBvect[1];
    mag_vect[2] = magRaw[2] - magBvect[2];

    // magCal = A_mat * mag_vect
    vectOp.transform(mag_vect, magAmat, mag_vect);

    _mx = mag_vect[0];
    _my = mag_vect[1];
    _mz = mag_vect[2];
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

      magCal[0] = _my;
      magCal[1] = -1.00 * _mx;
      magCal[2] = _mz;

      break;

    case 1: // ENU
      accCal[0] = _ax;
      accCal[1] = _ay;
      accCal[2] = _az;

      gyroCal[0] = _gx;
      gyroCal[1] = _gy;
      gyroCal[2] = _gz;

      magCal[0] = _mx;
      magCal[1] = _my;
      magCal[2] = _mz;

      break;

    case 2: // NED
      accCal[0] = _ay;
      accCal[1] = _ax;
      accCal[2] = -1.00 * _az;

      gyroCal[0] = _gy;
      gyroCal[1] = _gx;
      gyroCal[2] = -1.00 * _gz;

      magCal[0] = _my;
      magCal[1] = _mx;
      magCal[2] = -1.00 * _mz;

      break;
    }

    madgwickFilter.madgwickAHRSupdate(
        gyroCal[0], gyroCal[1], gyroCal[2], 
        accCal[0], accCal[1], accCal[2],
        MicroTeslaToTesla(magCal[0]), MicroTeslaToTesla(magCal[1]), MicroTeslaToTesla(magCal[2])
    );

    madgwickFilter.getOrientationRPY(r, p, y);
    madgwickFilter.getOrientationQuat(qw, qx, qy, qz);

    rpy[0] = r; rpy[1] = p; rpy[2] = y;
    quat[0] = qw; quat[1] = qx; quat[2] = qy; quat[3] = qz;
    //----------------------------------------------------//

    //---- accelerometer precessing - remove gravity -----//
    if (worldFrameId == 0 || worldFrameId == 1){ // NWU (0) or ENU (1)
      gx = g * (2*qx*qz - 2*qy*qw);
      gy = g * (2*qy*qz + 2*qx*qw);
      gz = g * (1 - 2*qx*qx - 2*qy*qy);
    } else { //NED (2)
      gx = g * (2*qx*qz + 2*qy*qw);
      gy = g * (2*qy*qz - 2*qx*qw);
      gz = g * (1 - 2*qx*qx - 2*qy*qy);
    }
    
    linearAccRaw[0] = accCal[0] - gx;
    linearAccRaw[1] = accCal[1] - gy;
    linearAccRaw[2] = accCal[2] - gz;
    //----------------------------------------------------//

    //--------- accelerometer precessing - filter --------//
    linearAcc[0] = accLPF[0].filter(linearAccRaw[0]);
    linearAcc[1] = accLPF[1].filter(linearAccRaw[1]);
    linearAcc[2] = accLPF[2].filter(linearAccRaw[2]);
    //----------------------------------------------------//

    readImuTime = millis();
  }
}
#ifndef COMMAND_FUNCTIONS_H
#define COMMAND_FUNCTIONS_H

#include <Arduino.h>
#include <Preferences.h>
#include "madgwick_filter.h"
#include "mpu6500_spi.h"
#include <Wire.h>

//------------ Communication Command IDs --------------//
const uint8_t START_BYTE = 0xBB;
const uint8_t READ_QUAT = 0x01;
const uint8_t READ_RPY = 0x02;
const uint8_t READ_RPY_VAR = 0x03;
const uint8_t WRITE_RPY_VAR = 0x04;
const uint8_t READ_ACC = 0x05;
const uint8_t READ_ACC_RAW = 0x06;
const uint8_t READ_ACC_OFF = 0x07;
const uint8_t WRITE_ACC_OFF = 0x08;
const uint8_t READ_ACC_VAR = 0x09;
const uint8_t WRITE_ACC_VAR = 0x0A;
const uint8_t READ_GYRO = 0x0B;
const uint8_t READ_GYRO_RAW = 0x0C;
const uint8_t READ_GYRO_OFF = 0x0D;
const uint8_t WRITE_GYRO_OFF = 0x0E;
const uint8_t READ_GYRO_VAR = 0x0F;
const uint8_t WRITE_GYRO_VAR = 0x10;
const uint8_t SET_I2C_ADDR = 0x1B;
const uint8_t GET_I2C_ADDR = 0x1C;
const uint8_t SET_FILTER_GAIN = 0x1D;
const uint8_t GET_FILTER_GAIN = 0x1E;
const uint8_t SET_FRAME_ID = 0x1F;
const uint8_t GET_FRAME_ID = 0x20;
const uint8_t RESET_PARAMS = 0x21;
const uint8_t READ_QUAT_RPY = 0x22;
const uint8_t READ_ACC_GYRO = 0x23;
const uint8_t READ_YAW_WITH_DRIFT = 0x24;
const uint8_t READ_YAW_VEL_DRIFT_BIAS = 0x25;
const uint8_t WRITE_YAW_VEL_DRIFT_BIAS = 0x26;
const uint8_t CLEAR_DATA_BUFFER = 0x27;
const uint8_t READ_IMU_DATA = 0x28;
//---------------------------------------------------//

int LED_PIN = 2;

//--------------- global variables -----------------//
/* Mpu9250 object, SPI bus, CS on pin 7

ESP32-C3 I2C
SCL: 9
SDA: 8

ESP32-C3 SPI
MOSI: 6
MISO: 5
SCK: 4
CS: 7

MPU9250 SPI CONNECTION:
VCC
GND
SCL (SCK)
SCA (MOSI)
EDA
ECL
AD0 (MISO)
INT
NCS (CS)
FYNSC

*/ 

/* Mpu6500 object, SPI bus, CS on pin 7 */
Mpu6500 imu(&SPI, 7);

int status;

MadgwickFilter madgwickFilter;

float filterGain = 0.1;
int worldFrameId = 1; // 0 - NWU,  1 - ENU,  2 - NED

// initial i2cAddress
uint8_t i2cAddress = 0x68;

// for stored initialization and reset
bool firstLoad = false;
//-------------------------------------------------//Serial.print("GYR: ");

//-------------- IMU MPU6050 ---------------------//

float yawVelDriftBias = 0.0;
float yawAccumOffset = 0.0;
float randomGainMultiplier = 0.0;

float roll, pitch, yaw;
float qw, qx, qy, qz;

float accOff[3];
float accVar[3];
float accRaw[3];
float accCal[3];

float gyroOff[3];
float gyroVar[3];
float gyroRaw[3];
float gyroCal[3];

float rpy[3];
float rpyVar[3];
float quat[4];

//------------------------------------------------//



//--------------- storage variables -----------------//
Preferences storage;

const char * rpyVar_key[3] = {
  "rpyVar0",
  "rpyVar1",
  "rpyVar2",
};

const char * accOff_key[3] = {
  "accOff0",
  "accOff1",
  "accOff2",
};

const char * accVar_key[3] = {
  "accVar0",
  "accVar1",
  "accVar2",
};

const char * gyroOff_key[3] = {
  "gyroOff0",
  "gyroOff1",
  "gyroOff2",
};

const char * gyroVar_key[3] = {
  "gyroVar0",
  "gyroVar1",
  "gyroVar2",
};

const char * yawVelDriftBias_key= "yawVelDriftBias";

const char * worldFrameId_key = "worldFrameId";

const char * filterGain_key = "filterGain";

const char * i2cAddress_key = "i2cAddress";

const char * firstLoad_key = "firstLoad";

const char * params_ns = "params"; // preference namespace

void resetParamsInStorage(){
  storage.begin(params_ns, false);

  for (int i=0; i<3; i+=1){
    storage.putFloat(accOff_key[i], 0.0);
    storage.putFloat(accVar_key[i], 0.0);
    storage.putFloat(gyroOff_key[i], 0.0);
    storage.putFloat(gyroVar_key[i], 0.0);
    storage.putFloat(rpyVar_key[i], 0.0);
  }
  storage.putFloat(yawVelDriftBias_key, 0.0);
  storage.putFloat(filterGain_key, 0.1);
  storage.putInt(worldFrameId_key, 1);
  storage.putUChar(i2cAddress_key, 0x68);

  storage.end();
}

void initParams(){
  //check for firstLoad
  storage.begin(params_ns, true);
  firstLoad = storage.getBool(firstLoad_key);
  storage.end();
  // if firsLoad -> reset all params and set firstLoad to false
  if(firstLoad == true){
    resetParamsInStorage();
    firstLoad = false;
    storage.begin(params_ns, false);
    storage.putBool(firstLoad_key, firstLoad);
    storage.end();
  }

}

void loadStoredParams(){
  initParams();
  // load each parameter form the storage to the local variables
  storage.begin(params_ns, true);

  for (int i=0; i<3; i+=1){
    accOff[i] = storage.getFloat(accOff_key[i], 0.0);
    accVar[i] = storage.getFloat(accVar_key[i], 0.0);
    gyroOff[i] = storage.getFloat(gyroOff_key[i], 0.0);
    gyroVar[i] = storage.getFloat(gyroVar_key[i], 0.0);
    rpyVar[i] = storage.getFloat(rpyVar_key[i], 0.0);
  }
  yawVelDriftBias = storage.getFloat(yawVelDriftBias_key, 0.0);
  filterGain = storage.getFloat(filterGain_key, 0.1);
  worldFrameId = storage.getInt(worldFrameId_key, 1);
  i2cAddress = storage.getUChar(i2cAddress_key, 0x68);

  storage.end();
}


//-------------------------------------------------//




//--------------- global functions ----------------//

float triggerResetParams()
{
  storage.begin(params_ns, false);
  firstLoad = true;
  storage.putBool(firstLoad_key, firstLoad);
  storage.end();
  // reload to reset
  loadStoredParams();
  return 1.0;
}

float clearDataBuffer()
{
  gyroCal[0] = 0.0;
  gyroCal[1] = 0.0;
  gyroCal[2] = 0.0;

  accCal[0] = 0.0;
  accCal[1] = 0.0;
  accCal[2] = 0.0;
  
  rpy[0] = 0.0;
  rpy[1] = 0.0;
  rpy[2] = 0.0;

  quat[0] = 0.0;
  quat[0] = 0.0;
  quat[0] = 0.0;
  quat[0] = 0.0;

  yawAccumOffset = 0.0;
  randomGainMultiplier = 0.0;

  madgwickFilter.init();
  madgwickFilter.setAlgorithmGain(filterGain);
  madgwickFilter.setWorldFrameId(worldFrameId); // 0 - NWU, 1 - ENU, 2 - NED (I'm using NWU reference frame)
  
  return 1.0;
}

// #include "i2c_comm.h"
float setI2cAddress(int address)
{
  if((address <= 0) || (address > 255)){
    return 0.0;
  }
  else {
    i2cAddress = (uint8_t)address;
    storage.begin(params_ns, false);
    storage.putUChar(i2cAddress_key, i2cAddress);
    storage.end();

    Wire.begin(i2cAddress);

    return 0.0;
  }  
}
float getI2cAddress()
{
  return (float)i2cAddress;
}


float setWorldFrameId(int id)
{
  if((id < 0) || (id > 2)){
    return 0.0;
  }
  else {
    worldFrameId = id;
    storage.begin(params_ns, false);
    storage.putInt(worldFrameId_key, worldFrameId);
    storage.end();

    madgwickFilter.setWorldFrameId(worldFrameId); // 0 - NWU,  1 - ENU,  2 - NED

    return 1.0;
  }  
}
float getWorldFrameId()
{
  return (float)worldFrameId;
}


float setFilterGain(float gain)
{
  filterGain = gain;
  storage.begin(params_ns, false);
  storage.putFloat(filterGain_key, filterGain);
  storage.end();

  madgwickFilter.setAlgorithmGain(filterGain);


  return 1.0; 
}
float getFilterGain()
{
  return (float)filterGain;
}
//-----------------------------------------------------------------//



//------------------------------------------------------------------//
void readQuat(float &qw, float &qx, float &qy, float &qz)
{
  qw = quat[0];
  qx = quat[1];
  qy = quat[2];
  qz = quat[3];
}


void readRPY(float &r, float &p, float &y)
{
  r = rpy[0];
  p = rpy[1];
  y = rpy[2];
}


void readRPYVariance(float &r, float &p, float &y)
{
  r = rpyVar[0];
  p = rpyVar[1];
  y = rpyVar[2];
}
float writeRPYVariance(float r, float p, float y) {
  float rpyVal[3] = {r, p, y};
  for (int i = 0; i < 3; i += 1)
  {
    rpyVar[i] = rpyVal[i];
    storage.begin(params_ns, false);
    storage.putFloat(rpyVar_key[i], rpyVar[i]);
    storage.end();
  }

  return 1.0;
}


void readAcc(float &ax, float &ay, float &az)
{
  ax = accCal[0];
  ay = accCal[1];
  az = accCal[2];
}


void readAccRaw(float &ax, float &ay, float &az)
{
  ax = accRaw[0];
  ay = accRaw[1];
  az = accRaw[2];
}


void readAccOffset(float &ax, float &ay, float &az)
{
  ax = accOff[0];
  ay = accOff[1];
  az = accOff[2];
}
float writeAccOffset(float ax, float ay, float az) {
  float accVal[3] = {ax, ay, az};
  for (int i = 0; i < 3; i += 1)
  {
    accOff[i] = accVal[i];
    storage.begin(params_ns, false);
    storage.putFloat(accOff_key[i], accOff[i]);
    storage.end();
  }

  return 1.0;
}


void readAccVariance(float &ax, float &ay, float &az)
{
  ax = accVar[0];
  ay = accVar[1];
  az = accVar[2];
}
float writeAccVariance(float ax, float ay, float az) {
  float accVal[3] = {ax, ay, az};
  for (int i = 0; i < 3; i += 1)
  {
    accVar[i] = accVal[i];
    storage.begin(params_ns, false);
    storage.putFloat(accVar_key[i], accVar[i]);
    storage.end();
  }

  return 1.0;
}


void readGyro(float &gx, float &gy, float &gz)
{
  gx = gyroCal[0];
  gy = gyroCal[1];
  gz = gyroCal[2];
}


void readGyroRaw(float &gx, float &gy, float &gz)
{
  gx = gyroRaw[0];
  gy = gyroRaw[1];
  gz = gyroRaw[2];
}


void readGyroOffset(float &gx, float &gy, float &gz)
{
  gx = gyroOff[0];
  gy = gyroOff[1];
  gz = gyroOff[2];
}
float writeGyroOffset(float gx, float gy, float gz) {
  float gyroVal[3] = {gx, gy, gz};
  for (int i = 0; i < 3; i += 1)
  {
    gyroOff[i] = gyroVal[i];
    storage.begin(params_ns, false);
    storage.putFloat(gyroOff_key[i], gyroOff[i]);
    storage.end();
  }

  return 1.0;
}


void readGyroVariance(float &gx, float &gy, float &gz)
{
  gx = gyroVar[0];
  gy = gyroVar[1];
  gz = gyroVar[2];
}
float writeGyroVariance(float gx, float gy, float gz) {
  float gyroVal[3] = {gx, gy, gz};
  for (int i = 0; i < 3; i += 1)
  {
    gyroVar[i] = gyroVal[i];
    storage.begin(params_ns, false);
    storage.putFloat(gyroVar_key[i], gyroVar[i]);
    storage.end();
  }

  return 1.0;
}


float readYawWithDrift()
{
  return (float)yaw;
}


float readYawVelDriftBias()
{
  return (float)yawVelDriftBias;
}
float writeYawVelDriftBias(float val) {

  yawVelDriftBias = val;
  storage.begin(params_ns, false);
  storage.putFloat(yawVelDriftBias_key, yawVelDriftBias);
  storage.end();

  return 1.0;
}
//-------------------------------------------------------------------//


#endif
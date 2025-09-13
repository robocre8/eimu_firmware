#ifndef COMMAND_FUNCTIONS_H
#define COMMAND_FUNCTIONS_H

#include <Arduino.h>
#include <Preferences.h>
#include "madgwick_filter.h"
#include "mpu9250_spi.h"
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
const uint8_t READ_MAG = 0x11;
const uint8_t READ_MAG_RAW = 0x12;
const uint8_t READ_MAG_H_OFF = 0x13;
const uint8_t WRITE_MAG_H_OFF = 0x14;
const uint8_t READ_MAG_S_OFF0 = 0x15;
const uint8_t WRITE_MAG_S_OFF0 = 0x16;
const uint8_t READ_MAG_S_OFF1 = 0x17;
const uint8_t WRITE_MAG_S_OFF1 = 0x18;
const uint8_t READ_MAG_S_OFF2 = 0x19;
const uint8_t WRITE_MAG_S_OFF2 = 0x1A;
const uint8_t SET_I2C_ADDR = 0x1B;
const uint8_t GET_I2C_ADDR = 0x1C;
const uint8_t SET_FILTER_GAIN = 0x1D;
const uint8_t GET_FILTER_GAIN = 0x1E;
const uint8_t SET_FRAME_ID = 0x1F;
const uint8_t GET_FRAME_ID = 0x20;
const uint8_t RESET_PARAMS = 0x21;
const uint8_t READ_QUAT_RPY = 0x22;
const uint8_t READ_ACC_GYRO = 0x23;
//---------------------------------------------------//

//--------------- global variables -----------------//
/* Mpu9250 object, SPI bus, CS on pin 5 */
MPU9250 imu(SPI, 5);
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
float accOff[3];
float accVar[3];
float accRaw[3];
float accCal[3];

float gyroOff[3];
float gyroVar[3];
float gyroRaw[3];
float gyroCal[3];

float magRaw[3];
float magCal[3];
float magAmat[3][3];
float magBvect[3];
float mag_vect[3];

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

const char * magBvect_key[3] = {
  "magBvect0",
  "magBvect1",
  "magBvect2",
};

const char * magAmatR0_key[3] = {
  "magAmatR00",
  "magAmatR01",
  "magAmatR02",
};

const char * magAmatR1_key[3] = {
  "magAmatR10",
  "magAmatR11",
  "magAmatR12",
};

const char * magAmatR2_key[3] = {
  "magAmatR20",
  "magAmatR21",
  "magAmatR22",
};

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
    storage.putFloat(magBvect_key[i], 0.0);
    storage.putFloat(magAmatR0_key[i], 0.0);
    storage.putFloat(magAmatR1_key[i], 0.0);
    storage.putFloat(magAmatR2_key[i], 0.0);
  }
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
    magBvect[i] = storage.getFloat(magBvect_key[i], 0.0);
    magAmat[0][i] = storage.getFloat(magAmatR0_key[i], 0.0);
    magAmat[1][i] = storage.getFloat(magAmatR1_key[i], 0.0);
    magAmat[2][i] = storage.getFloat(magAmatR2_key[i], 0.0);
  }
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

void readMag(float &mx, float &my, float &mz)
{
  mx = magCal[0];
  my = magCal[1];
  mz = magCal[2];
}


void readMagRaw(float &mx, float &my, float &mz)
{
  mx = magRaw[0];
  my = magRaw[1];
  mz = magRaw[2];
}


void readMagHardOffset(float &x, float &y, float &z)
{
  x = magBvect[0];
  y = magBvect[1];
  z = magBvect[2];
}
float writeMagHardOffset(float x, float y, float z) {
  float magOffsetVal[3] = {x, y, z};
  for (int i = 0; i < 3; i += 1)
  {
    magBvect[i] = magOffsetVal[i];
    storage.begin(params_ns, false);
    storage.putFloat(magBvect_key[i], magBvect[i]);
    storage.end();
  }

  return 1.0;
}


void readMagSoftOffset0(float &x, float &y, float &z)
{
  x = magAmat[0][0];
  y = magAmat[0][1];
  z = magAmat[0][2];
}
float writeMagSoftOffset0(float x, float y, float z) {
  float magOffsetVal[3] = {x, y, z};
  for (int i = 0; i < 3; i += 1)
  {
    magAmat[0][i] = magOffsetVal[i];
    storage.begin(params_ns, false);
    storage.putFloat(magAmatR0_key[i], magAmat[0][i]);
    storage.end();
  }

  return 1.0;
}


void readMagSoftOffset1(float &x, float &y, float &z)
{
  x = magAmat[1][0];
  y = magAmat[1][1];
  z = magAmat[1][2];
}
float writeMagSoftOffset1(float x, float y, float z) {
  float magOffsetVal[3] = {x, y, z};
  for (int i = 0; i < 3; i += 1)
  {
    magAmat[1][i] = magOffsetVal[i];
    storage.begin(params_ns, false);
    storage.putFloat(magAmatR1_key[i], magAmat[1][i]);
    storage.end();
  }

  return 1.0;
}


void readMagSoftOffset2(float &x, float &y, float &z)
{
  x = magAmat[2][0];
  y = magAmat[2][1];
  z = magAmat[2][2];
}
float writeMagSoftOffset2(float x, float y, float z) {
  float magOffsetVal[3] = {x, y, z};
  for (int i = 0; i < 3; i += 1)
  {
    magAmat[2][i] = magOffsetVal[i];
    storage.begin(params_ns, false);
    storage.putFloat(magAmatR2_key[i], magAmat[2][i]);
    storage.end();
  }

  return 1.0;
}
//-------------------------------------------------------------------//


#endif
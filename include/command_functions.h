#ifndef COMMAND_FUNCTIONS_H
#define COMMAND_FUNCTIONS_H

#include <Arduino.h>
#include <Preferences.h>
#include "madgwick_filter.h"
#include "mpu9250_spi.h"
#include <Wire.h>
#include <adaptive_low_pass_filter.h>

//------------ Communication Command IDs --------------//
const uint8_t START_BYTE = 0xBB;
const uint8_t READ_QUAT = 0x01;
const uint8_t READ_RPY = 0x02;
const uint8_t READ_RPY_VAR = 0x03;
const uint8_t WRITE_RPY_VAR = 0x04;
const uint8_t READ_ACC = 0x05;
const uint8_t READ_ACC_RAW = 0x06;
// const uint8_t READ_ACC_OFF = 0x07;
// const uint8_t WRITE_ACC_OFF = 0x08;
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
// const uint8_t READ_QUAT_RPY = 0x22;
const uint8_t READ_ACC_GYRO = 0x23;
const uint8_t CLEAR_DATA_BUFFER = 0x27;
const uint8_t READ_IMU_DATA = 0x28;
const uint8_t SET_ACC_LPF_CUT_FREQ = 0x29;
const uint8_t GET_ACC_LPF_CUT_FREQ = 0x2A;
const uint8_t READ_LIN_ACC_RAW = 0x2B;
const uint8_t READ_LIN_ACC = 0x2C;

const uint8_t READ_ACC_BIAS_VECT = 0x2D;
const uint8_t WRITE_ACC_BIAS_VECT = 0x2E;
const uint8_t READ_ACC_SCALE_MAT0 = 0x2F;
const uint8_t WRITE_ACC_SCALE_MAT0 = 0x30;
const uint8_t READ_ACC_SCALE_MAT1 = 0x31;
const uint8_t WRITE_ACC_SCALE_MAT1 = 0x32;
const uint8_t READ_ACC_SCALE_MAT2 = 0x33;
const uint8_t WRITE_ACC_SCALE_MAT2 = 0x34;
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
MPU9250 imu(SPI, 7);
int status;

MadgwickFilter madgwickFilter;

float filterGain = 0.5;
int worldFrameId = 1; // 0 - NWU,  1 - ENU,  2 - NED

// initial i2cAddress
uint8_t i2cAddress = 0x68;

// for stored initialization and reset
bool firstLoad = false;
//-------------------------------------------------//Serial.print("GYR: ");

// adaptive lowpass Filter
const int filterOrder = 1;
double cutOffFreq = 1.0;

AdaptiveLowPassFilter accLPF[3] = {
  AdaptiveLowPassFilter(filterOrder, cutOffFreq), // motor 0 velocity filter
  AdaptiveLowPassFilter(filterOrder, cutOffFreq), // motor 1 velocity filter
  AdaptiveLowPassFilter(filterOrder, cutOffFreq), // motor 1 velocity filter
};

//-------------- IMU MPU6050 ---------------------//
float accRaw[3];
float accCal[3];
float accScaleMat[3][3];
float accBiasVect[3];
float acc_vect[3];
float accVar[3];

float linearAccRaw[3];
float linearAcc[3];

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

const char * accBiasVect_key[3] = {
  "accBiasVect0",
  "accBiasVect1",
  "accBiasVect2",
};

const char * accScaleMatR0_key[3] = {
  "accScaleMat00",
  "accScaleMat01",
  "accScaleMat02",
};

const char * accScaleMatR1_key[3] = {
  "accScaleMat10",
  "accScaleMat11",
  "accScaleMat12",
};

const char * accScaleMatR2_key[3] = {
  "accScaleMat20",
  "accScaleMat21",
  "accScaleMat22",
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

const char * cutOffFreq_key = "cutOffFreq";

const char * worldFrameId_key = "worldFrameId";

const char * filterGain_key = "filterGain";

const char * i2cAddress_key = "i2cAddress";

const char * firstLoad_key = "firstLoad";

const char * params_ns = "params"; // preference namespace

void resetParamsInStorage(){
  storage.begin(params_ns, false);

  for (int i=0; i<3; i+=1){
    storage.putFloat(accBiasVect_key[i], 0.0);
    storage.putFloat(accScaleMatR0_key[i], 0.0);
    storage.putFloat(accScaleMatR1_key[i], 0.0);
    storage.putFloat(accScaleMatR2_key[i], 0.0);
    storage.putFloat(accVar_key[i], 0.0);
    storage.putFloat(gyroOff_key[i], 0.0);
    storage.putFloat(gyroVar_key[i], 0.0);
    storage.putFloat(rpyVar_key[i], 0.0);
    storage.putFloat(magBvect_key[i], 0.0);
    storage.putFloat(magAmatR0_key[i], 0.0);
    storage.putFloat(magAmatR1_key[i], 0.0);
    storage.putFloat(magAmatR2_key[i], 0.0);
  }
  storage.putFloat(filterGain_key, 0.5);
  storage.putFloat(cutOffFreq_key, 1.0);
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
    accBiasVect[i] = storage.getFloat(accBiasVect_key[i], 0.0);
    accScaleMat[0][i] = storage.getFloat(accScaleMatR0_key[i], 0.0);
    accScaleMat[1][i] = storage.getFloat(accScaleMatR1_key[i], 0.0);
    accScaleMat[2][i] = storage.getFloat(accScaleMatR2_key[i], 0.0);
    accVar[i] = storage.getFloat(accVar_key[i], 0.0);
    gyroOff[i] = storage.getFloat(gyroOff_key[i], 0.0);
    gyroVar[i] = storage.getFloat(gyroVar_key[i], 0.0);
    rpyVar[i] = storage.getFloat(rpyVar_key[i], 0.0);
    magBvect[i] = storage.getFloat(magBvect_key[i], 0.0);
    magAmat[0][i] = storage.getFloat(magAmatR0_key[i], 0.0);
    magAmat[1][i] = storage.getFloat(magAmatR1_key[i], 0.0);
    magAmat[2][i] = storage.getFloat(magAmatR2_key[i], 0.0);
  }
  filterGain = storage.getFloat(filterGain_key, 0.5);
  cutOffFreq = storage.getFloat(cutOffFreq_key, 1.0);
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
  quat[1] = 0.0;
  quat[2] = 0.0;
  quat[3] = 0.0;

  madgwickFilter.init();
  madgwickFilter.setAlgorithmGain(filterGain);
  madgwickFilter.setWorldFrameId(worldFrameId); // 0 - NWU, 1 - ENU, 2 - NED (I'm using NWU reference frame)

  for (int i=0; i<3; i+=1) {
    accLPF[i].clear();
  }
  
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


float setAccFilterCF(float cf)
{
  cutOffFreq = cf;
  storage.begin(params_ns, false);
  storage.putFloat(cutOffFreq_key, cutOffFreq);
  storage.end();

  for (int i=0; i<3; i+=1) {
    accLPF[i].setCutOffFreq(cutOffFreq);
  }

  return 1.0; 
}
float getAccFilterCF()
{
  return (float)cutOffFreq;
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

void readLinearAccRaw(float &ax, float &ay, float &az)
{
  ax = linearAccRaw[0];
  ay = linearAccRaw[1];
  az = linearAccRaw[2];
}

void readLinearAcc(float &ax, float &ay, float &az)
{
  ax = linearAcc[0];
  ay = linearAcc[1];
  az = linearAcc[2];
}

void readAccRaw(float &ax, float &ay, float &az)
{
  ax = accRaw[0];
  ay = accRaw[1];
  az = accRaw[2];
}

void readAccBias(float &x, float &y, float &z)
{
  x = accBiasVect[0];
  y = accBiasVect[1];
  z = accBiasVect[2];
}
float writeAccBias(float x, float y, float z) {
  float accBiasVectVal[3] = {x, y, z};
  for (int i = 0; i < 3; i += 1)
  {
    accBiasVect[i] = accBiasVectVal[i];
    storage.begin(params_ns, false);
    storage.putFloat(accBiasVect_key[i], accBiasVect[i]);
    storage.end();
  }

  return 1.0;
}


void readAccScaleMatR0(float &x, float &y, float &z)
{
  x = accScaleMat[0][0];
  y = accScaleMat[0][1];
  z = accScaleMat[0][2];
}
float writeAccScaleMatR0(float x, float y, float z) {
  float accScaleVal[3] = {x, y, z};
  for (int i = 0; i < 3; i += 1)
  {
    accScaleMat[0][i] = accScaleVal[i];
    storage.begin(params_ns, false);
    storage.putFloat(accScaleMatR0_key[i], accScaleMat[0][i]);
    storage.end();
  }

  return 1.0;
}


void readAccScaleMatR1(float &x, float &y, float &z)
{
  x = accScaleMat[1][0];
  y = accScaleMat[1][1];
  z = accScaleMat[1][2];
}
float writeAccScaleMatR1(float x, float y, float z) {
  float accScaleVal[3] = {x, y, z};
  for (int i = 0; i < 3; i += 1)
  {
    accScaleMat[1][i] = accScaleVal[i];
    storage.begin(params_ns, false);
    storage.putFloat(accScaleMatR1_key[i], accScaleMat[1][i]);
    storage.end();
  }

  return 1.0;
}


void readAccScaleMatR2(float &x, float &y, float &z)
{
  x = accScaleMat[2][0];
  y = accScaleMat[2][1];
  z = accScaleMat[2][2];
}
float writeAccScaleMatR2(float x, float y, float z) {
  float accScaleVal[3] = {x, y, z};
  for (int i = 0; i < 3; i += 1)
  {
    accScaleMat[2][i] = accScaleVal[i];
    storage.begin(params_ns, false);
    storage.putFloat(accScaleMatR2_key[i], accScaleMat[2][i]);
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
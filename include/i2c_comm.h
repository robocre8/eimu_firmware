#ifndef I2C_COMM_H
#define I2C_COMM_H

#include <Wire.h>
#include "command_functions.h"

const uint8_t MAX_I2C_BUFFER = 32;
static uint8_t sendMsgBuffer[MAX_I2C_BUFFER];
static uint8_t sendMsgLength = 0;

// The arguments converted to integers
int i2c_cmd;
int i2c_cmd_pos;
float i2c_arg1;
float i2c_arg2;
float i2c_arg3;

/* Clear the current command parameters */
void i2c_resetCommand() {
  i2c_cmd = 0;
  i2c_cmd_pos = 0;
  i2c_arg1 = 0.0;
  i2c_arg2 = 0.0;
  i2c_arg3 = 0.0;
}

void clearSendMsgBuffer(){
  memset(sendMsgBuffer, 0, (size_t)MAX_I2C_BUFFER); 
}

// Pack float response into txBuffer
void prepareResponse3(float res0, float res1, float res2) {
  sendMsgLength = 12;
  memcpy(&sendMsgBuffer[0], &res0, sizeof(float));
  memcpy(&sendMsgBuffer[4], &res1, sizeof(float));
  memcpy(&sendMsgBuffer[8], &res2, sizeof(float));
}

// Example command handler
void i2c_runCommand() {
  gpio_set_level((gpio_num_t)LED_PIN, 1);

  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  switch (i2c_cmd) {

    case READ_RPY: {
      readRPY(x, y, z);
      prepareResponse3(x, y, z);
      break;
    }

    case READ_RPY_VAR: {
      readRPYVariance(x, y, z);
      prepareResponse3(x, y, z);
      break;
    }

    case READ_GYRO: {
      readGyro(x, y, z);
      prepareResponse3(x, y, z);
      break;
    }

    case READ_GYRO_VAR: {
      readGyroVariance(x, y, z);
      prepareResponse3(x, y, z);
      break;
    }

    // case READ_ACC: {
    //   readAcc(x, y, z);
    //   prepareResponse3(x, y, z);
    //   break;
    // }
    
    case READ_LIN_ACC: {
      readLinearAcc(x, y, z);
      prepareResponse3(x, y, z);
      break;
    }

    case READ_ACC_VAR: {
      readAccVariance(x, y, z);
      prepareResponse3(x, y, z);
      break;
    }

    case SET_FILTER_GAIN: {
      setFilterGain((double)i2c_arg2);
      gpio_set_level((gpio_num_t)LED_PIN, 0);
      break;
    }

    case GET_FILTER_GAIN: {
      x = getFilterGain();
      prepareResponse3(x, y, z);
      break;
    }

    
    case SET_FRAME_ID: {
      setWorldFrameId((int)i2c_arg2);
      gpio_set_level((gpio_num_t)LED_PIN, 0);
      break;
    }

    case GET_FRAME_ID: {
      x = getWorldFrameId();
      prepareResponse3(x, y, z);
      break;
    }

    case RESET: {
      //reset all stored parameters return 1.0 if successfull
      x = triggerResetParams();
      prepareResponse3(x, y, z);
      break;
    }

    case CLEAR: {
      // clear all inintializing variables
      x = clearDataBuffer();
      prepareResponse3(x, y, z);
      break;
    }
  }
}




// Called when master requests data
void onRequest() {
  Wire.write(sendMsgBuffer, sendMsgLength);
  clearSendMsgBuffer();
  gpio_set_level((gpio_num_t)LED_PIN, 0);
}

// Called when master sends data
void onReceive(int numBytes)
{
  // Expect exactly 4 floats = 16 bytes
  if (numBytes != 16) {
    // Drain buffer if size is wrong
    while (Wire.available()) Wire.read();
    return;
  }

  uint8_t rxBuf[16];

  for (uint8_t i = 0; i < 16; i++) {
    rxBuf[i] = Wire.read();
  }

  // Unpack floats
  float cmd_f;
  memcpy(&cmd_f,  &rxBuf[0],  4);
  memcpy(&i2c_arg1, &rxBuf[4],  4);
  memcpy(&i2c_arg2, &rxBuf[8],  4);
  memcpy(&i2c_arg3, &rxBuf[12],  4);

  // Command as integer
  i2c_cmd = (int)cmd_f;

  // Execute command
  i2c_runCommand();
}

#endif
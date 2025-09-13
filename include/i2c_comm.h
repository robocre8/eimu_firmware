#ifndef I2C_COMM_H
#define I2C_COMM_H

#include <Wire.h>
#include "command_functions.h"


const uint8_t MAX_I2C_BUFFER = 32;
static uint8_t sendMsgBuffer[MAX_I2C_BUFFER];
static uint8_t sendMsgLength = 0;

void clearSendMsgBuffer(){
  for (uint8_t i=0; i< MAX_I2C_BUFFER; i+=1){
    sendMsgBuffer[i] = 0x00;
  }
}
// Pack float response into txBuffer
void prepareResponse1(float res) {
  sendMsgLength = 4;
  memcpy(&sendMsgBuffer[0], &res, sizeof(float));
}

void prepareResponse3(float res0, float res1, float res2) {
  sendMsgLength = 12;
  memcpy(&sendMsgBuffer[0], &res0, sizeof(float));
  memcpy(&sendMsgBuffer[4], &res1, sizeof(float));
  memcpy(&sendMsgBuffer[8], &res2, sizeof(float));
}

void prepareResponse4(float res0, float res1, float res2, float res3) {
  sendMsgLength = 16;
  memcpy(&sendMsgBuffer[0], &res0, sizeof(float));
  memcpy(&sendMsgBuffer[4], &res1, sizeof(float));
  memcpy(&sendMsgBuffer[8], &res2, sizeof(float));
  memcpy(&sendMsgBuffer[12], &res3, sizeof(float));
}

void prepareResponse6(float res0, float res1, float res2, float res3, float res4, float res5) {
  sendMsgLength = 24;
  memcpy(&sendMsgBuffer[0], &res0, sizeof(float));
  memcpy(&sendMsgBuffer[4], &res1, sizeof(float));
  memcpy(&sendMsgBuffer[8], &res2, sizeof(float));
  memcpy(&sendMsgBuffer[12], &res3, sizeof(float));
  memcpy(&sendMsgBuffer[16], &res4, sizeof(float));
  memcpy(&sendMsgBuffer[20], &res5, sizeof(float));
}

void prepareResponse8(float res0, float res1, float res2, float res3, float res4, float res5, float res6, float res7) {
  sendMsgLength = 32;
  memcpy(&sendMsgBuffer[0], &res0, sizeof(float));
  memcpy(&sendMsgBuffer[4], &res1, sizeof(float));
  memcpy(&sendMsgBuffer[8], &res2, sizeof(float));
  memcpy(&sendMsgBuffer[12], &res3, sizeof(float));
  memcpy(&sendMsgBuffer[16], &res4, sizeof(float));
  memcpy(&sendMsgBuffer[20], &res5, sizeof(float));
  memcpy(&sendMsgBuffer[24], &res6, sizeof(float));
  memcpy(&sendMsgBuffer[28], &res7, sizeof(float));
}

// Example command handler
void handleCommand(uint8_t cmd, uint8_t* data, uint8_t length) {

  gpio_set_level((gpio_num_t)LED_BUILTIN, 1);

  switch (cmd) {
    case READ_QUAT: {
      float qw, qx, qy, qz;
      readQuat(qw, qx, qy, qz);
      prepareResponse4(qw, qx, qy, qz);
      break;
    }

    case READ_RPY: {
      float r, p, y;
      readRPY(r, p, y);
      prepareResponse3(r, p, y);
      break;
    }

    case READ_RPY_VAR: {
      float r, p, y;
      readRPYVariance(r, p, y);
      prepareResponse3(r, p, y);
      break;
    }

    case READ_ACC: {
      float ax, ay, az;
      readAcc(ax, ay, az);
      prepareResponse3(ax, ay, az);
      break;
    }

    case READ_ACC_VAR: {
      float ax, ay, az;
      readAccVariance(ax, ay, az);
      prepareResponse3(ax, ay, az);
      break;
    }

    case READ_GYRO: {
      float gx, gy, gz;
      readGyro(gx, gy, gz);
      prepareResponse3(gx, gy, gz);
      break;
    }

    case READ_GYRO_VAR: {
      float gx, gy, gz;
      readGyroVariance(gx, gy, gz);
      prepareResponse3(gx, gy, gz);
      break;
    }
    
    case READ_MAG: {
      float mx, my, mz;
      readMag(mx, my, mz);
      prepareResponse3(mx, my, mz);
      break;
    }

    case GET_FILTER_GAIN: {
      float res = getFilterGain();
      prepareResponse1(res);
      break;
    }

    case SET_FRAME_ID: {
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setWorldFrameId((int)value);
      prepareResponse1(res);
      break;
    }
    case GET_FRAME_ID: {
      float res = getWorldFrameId();
      prepareResponse1(res);
      break;
    }

    case READ_QUAT_RPY: {
      float qw, qx, qy, qz, r, p, y, dummy_data = 0.0;
      readQuat(qw, qx, qy, qz);
      readRPY(r, p, y);
      prepareResponse8(qw, qx, qy, qz, r, p, y, dummy_data);
      break;
    }

    case READ_ACC_GYRO: {
      float ax, ay, az, gx, gy, gz;
      readAcc(ax, ay, az);
      readGyro(gx, gy, gz);
      prepareResponse6(ax, ay, az, gx, gy, gz);
      break;
    }

    default: {
      float error = 0.0;
      prepareResponse1(error);
      break;
    }
  }
}




// Called when master requests data
void onRequest() {
  Wire.write(sendMsgBuffer, sendMsgLength);
  clearSendMsgBuffer();
  gpio_set_level((gpio_num_t)LED_BUILTIN, 0);
}

// Called when master sends data
void onReceive(int numBytes) {
  static uint8_t readState = 0;
  static uint8_t msgCmd, msgLength;
  static uint8_t msgBuffer[MAX_I2C_BUFFER];
  static uint8_t msgIndex = 0;
  static uint8_t msgChecksum = 0;

  while (Wire.available()) {
    uint8_t b = Wire.read();

    switch (readState) {
      case 0: // Wait for start
        if (b == START_BYTE) {
          readState = 1;
          msgChecksum = b;
        }
        break;

      case 1: // Command
        msgCmd = b;
        msgChecksum += b;
        readState = 2;
        break;

      case 2: // Length
        msgLength = b;
        msgChecksum += b;
        if (msgLength==0){
          readState = 4;
        }
        else{
          msgIndex = 0;
          readState = 3;
        }
        break;

      case 3: // Payload
        msgBuffer[msgIndex++] = b;
        msgChecksum += b;
        if (msgIndex >= msgLength) readState = 4;
        break;

      case 4: // Checksum
        if ((msgChecksum & 0xFF) == b) {
          handleCommand(msgCmd, msgBuffer, msgLength);
        } else {
          float error = 0.0;
          prepareResponse1(error);
        }
        readState = 0; // reset for next packet
        break;
    }
  }

}


#endif
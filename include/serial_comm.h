#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include "command_functions.h"

// The arguments converted to integers
int cmd;
int cmd_pos;
float arg1;
float arg2;
float arg3;

void send_data(float data1=0.0, float data2=0.0, float data3=0.0){
  Serial.print(data1,6);
  Serial.print(' ');
  Serial.print(data2,6);
  Serial.print(' ');
  Serial.println(data3,6);
  Serial.flush();
}

/* Clear the current command parameters */
void resetCommand() {
  cmd = 0;
  cmd_pos = 0;
  arg1 = 0.0;
  arg2 = 0.0;
  arg3 = 0.0;
}

/* Run a command.  Commands are defined in commands.h */
void runCommand() {
  gpio_set_level((gpio_num_t)LED_PIN, 1);

  switch (cmd) {
  
    case READ_RPY: {
      float x, y, z;
      readRPY(x, y, z);
      send_data(x, y, z);
      break;
    }

    case READ_RPY_VAR: {
      float x, y, z;
      readRPYVariance(x, y, z);
      send_data(x, y, z);
      break;
    }

    case WRITE_RPY_VAR: {
      writeRPYVariance(arg1, arg2, arg3);
      break;
    }

    case READ_GYRO: {
      float x, y, z;
      readGyro(x, y, z);
      send_data(x, y, z);
      break;
    }

    case READ_GYRO_RAW: {
      float x, y, z;
      readGyroRaw(x, y, z);
      send_data(x, y, z);
      break;
    }

    case READ_GYRO_OFF: {
      float x, y, z;
      readGyroOffset(x, y, z);
      send_data(x, y, z);
      break;
    }

    case WRITE_GYRO_OFF: {
      writeGyroOffset(arg1, arg2, arg3);
      break;
    }

    case READ_GYRO_VAR: {
      float x, y, z;
      readGyroVariance(x, y, z);
      send_data(x, y, z);
      break;
    }

    case WRITE_GYRO_VAR: {
      writeGyroVariance(arg1, arg2, arg3);
      break;
    }

    case READ_ACC: {
      float x, y, z;
      readAcc(x, y, z);
      send_data(x, y, z);
      break;
    }

    case READ_ACC_RAW: {
      float x, y, z;
      readAccRaw(x, y, z);
      send_data(x, y, z);
      break;
    }

    case READ_ACC_OFF: {
      float x, y, z;
      readAccOffset(x, y, z);
      send_data(x, y, z);
      break;
    }

    case WRITE_ACC_OFF: {
      writeAccOffset(arg1, arg2, arg3);
      break;
    }

    case READ_ACC_VAR: {
      float x, y, z;
      readAccVariance(x, y, z);
      send_data(x, y, z);
      break;
    }

    case WRITE_ACC_VAR: {
      writeAccVariance(arg1, arg2, arg3);
      break;
    }

    case READ_LIN_ACC: {
      float x, y, z;
      readLinearAcc(x, y, z);
      send_data(x, y, z);
      break;
    }

    case READ_LIN_ACC_RAW: {
      float x, y, z;
      readLinearAccRaw(x, y, z);
      send_data(x, y, z);
      break;
    }

    case SET_ACC_LPF_CUT_FREQ: {
      setAccFilterCF((double)arg2);
      break;
    }

    case GET_ACC_LPF_CUT_FREQ: {
      float res = getAccFilterCF();
      send_data(res);
      break;
    }

    case READ_MAG: {
      float x, y, z;
      readMag(x, y, z);
      send_data(x, y, z);
      break;
    }

    case READ_MAG_RAW: {
      float x, y, z;
      readMagRaw(x, y, z);
      send_data(x, y, z);
      break;
    }

    case READ_MAG_H_OFF: {
      float x, y, z;
      readMagHardOffset(x, y, z);
      send_data(x, y, z);
      break;
    }

    case WRITE_MAG_H_OFF: {
      writeMagHardOffset(arg1, arg2, arg3);
      break;
    }

    case READ_MAG_S_OFF0: {
      float x, y, z;
      readMagSoftOffset0(x, y, z);
      send_data(x, y, z);
      break;
    }

    case WRITE_MAG_S_OFF0: {
      writeMagSoftOffset0(arg1, arg2, arg3);
      break;
    }

    case READ_MAG_S_OFF1: {
      float x, y, z;
      readMagSoftOffset1(x, y, z);
      send_data(x, y, z);
      break;
    }

    case WRITE_MAG_S_OFF1: {
      writeMagSoftOffset1(arg1, arg2, arg3);
      break;
    }

    case READ_MAG_S_OFF2: {
      float x, y, z;
      readMagSoftOffset2(x, y, z);
      send_data(x, y, z);
      break;
    }

    case WRITE_MAG_S_OFF2: {
      writeMagSoftOffset2(arg1, arg2, arg3);
      break;
    }

    case SET_I2C_ADDR: {
      setI2cAddress((int)arg2);
      break;
    }

    case GET_I2C_ADDR: {
      float res = getI2cAddress();
      send_data(res);
      break;
    }


    case SET_FILTER_GAIN: {
      setFilterGain((double)arg2);
      break;
    }

    case GET_FILTER_GAIN: {
      float res = getFilterGain();
      send_data(res);
      break;
    }

    
    case SET_FRAME_ID: {
      setWorldFrameId((int)arg2);
      break;
    }

    case GET_FRAME_ID: {
      float res = getWorldFrameId();
      send_data(res);
      break;
    }

    case RESET: {
      //reset all stored parameters return 1.0 if successfull
      float res = triggerResetParams();
      send_data(res);
      break;
    }

    case CLEAR: {
      // clear all inintializing variables
      float res = clearDataBuffer();
      send_data(res);
      break;
    }
  }

  gpio_set_level((gpio_num_t)LED_PIN, 0);
}


void recieve_and_send_data() {
  static char rx_buffer[64];
  static uint8_t idx = 0;

  while (Serial.available()) {
    char c = Serial.read();

    // End of line → parse
    if (c == '\n' || c == '\r') {
      rx_buffer[idx] = null_char;
      idx = 0;

      char *ptr = rx_buffer;
      char *end;

      // Parse cmd
      float cmd_  = strtof(ptr, &end);
      cmd = (int)cmd_;
      ptr = end;

      // Parse arg1
      arg1 = strtof(ptr, &end);
      ptr = end;

      // Parse arg2
      arg2 = strtof(ptr, &end);
      ptr = end;

      // Parse arg3
      arg3 = strtof(ptr, &end);

      runCommand();
      return;
    }

    // Store characters safely
    if (idx < sizeof(rx_buffer) - 1) {
      rx_buffer[idx++] = c;
    }
  }
}


#endif
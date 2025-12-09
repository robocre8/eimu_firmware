#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include "command_functions.h"


void processCommand(uint8_t cmd, uint8_t* data, uint8_t length) {

  gpio_set_level((gpio_num_t)LED_PIN, 1);
  switch (cmd) {
    case READ_QUAT: {
      float qw, qx, qy, qz;
      readQuat(qw, qx, qy, qz);
      Serial.write((uint8_t*)&qw, sizeof(qw));
      Serial.write((uint8_t*)&qx, sizeof(qx));
      Serial.write((uint8_t*)&qy, sizeof(qy));
      Serial.write((uint8_t*)&qz, sizeof(qz));
      break;
    }

    case READ_RPY: {
      float r, p, y;
      readRPY(r, p, y);
      Serial.write((uint8_t*)&r, sizeof(r));
      Serial.write((uint8_t*)&p, sizeof(p));
      Serial.write((uint8_t*)&y, sizeof(y));
      break;
    }

    case READ_RPY_VAR: {
      float r, p, y;
      readRPYVariance(r, p, y);
      Serial.write((uint8_t*)&r, sizeof(r));
      Serial.write((uint8_t*)&p, sizeof(p));
      Serial.write((uint8_t*)&y, sizeof(y));
      break;
    }
    case WRITE_RPY_VAR: {
      float r, p, y;
      memcpy(&r, &data[0], sizeof(float));
      memcpy(&p, &data[4], sizeof(float));
      memcpy(&y, &data[8], sizeof(float));
      float res = writeRPYVariance(r, p, y);
      break;
    }


    case READ_ACC: {
      float ax, ay, az;
      readAcc(ax, ay, az);
      Serial.write((uint8_t*)&ax, sizeof(ax));
      Serial.write((uint8_t*)&ay, sizeof(ay));
      Serial.write((uint8_t*)&az, sizeof(az));
      break;
    }


    case READ_ACC_RAW: {
      float ax, ay, az;
      readAccRaw(ax, ay, az);
      Serial.write((uint8_t*)&ax, sizeof(ax));
      Serial.write((uint8_t*)&ay, sizeof(ay));
      Serial.write((uint8_t*)&az, sizeof(az));
      break;
    }


    case READ_LIN_ACC: {
      float ax, ay, az;
      readLinearAcc(ax, ay, az);
      Serial.write((uint8_t*)&ax, sizeof(ax));
      Serial.write((uint8_t*)&ay, sizeof(ay));
      Serial.write((uint8_t*)&az, sizeof(az));
      break;
    }


    case READ_LIN_ACC_RAW: {
      float ax, ay, az;
      readLinearAccRaw(ax, ay, az);
      Serial.write((uint8_t*)&ax, sizeof(ax));
      Serial.write((uint8_t*)&ay, sizeof(ay));
      Serial.write((uint8_t*)&az, sizeof(az));
      break;
    }


    case READ_ACC_OFF: {
      float ax, ay, az;
      readAccOffset(ax, ay, az);
      Serial.write((uint8_t*)&ax, sizeof(ax));
      Serial.write((uint8_t*)&ay, sizeof(ay));
      Serial.write((uint8_t*)&az, sizeof(az));
      break;
    }
    case WRITE_ACC_OFF: {
      float ax, ay, az;
      memcpy(&ax, &data[0], sizeof(float));
      memcpy(&ay, &data[4], sizeof(float));
      memcpy(&az, &data[8], sizeof(float));
      float res = writeAccOffset(ax, ay, az);
      break;
    }


    case READ_ACC_VAR: {
      float ax, ay, az;
      readAccVariance(ax, ay, az);
      Serial.write((uint8_t*)&ax, sizeof(ax));
      Serial.write((uint8_t*)&ay, sizeof(ay));
      Serial.write((uint8_t*)&az, sizeof(az));
      break;
    }
    case WRITE_ACC_VAR: {
      float ax, ay, az;
      memcpy(&ax, &data[0], sizeof(float));
      memcpy(&ay, &data[4], sizeof(float));
      memcpy(&az, &data[8], sizeof(float));
      float res = writeAccVariance(ax, ay, az);
      break;
    }


    case READ_GYRO: {
      float gx, gy, gz;
      readGyro(gx, gy, gz);
      Serial.write((uint8_t*)&gx, sizeof(gx));
      Serial.write((uint8_t*)&gy, sizeof(gy));
      Serial.write((uint8_t*)&gz, sizeof(gz));
      break;
    }


    case READ_GYRO_RAW: {
      float gx, gy, gz;
      readGyroRaw(gx, gy, gz);
      Serial.write((uint8_t*)&gx, sizeof(gx));
      Serial.write((uint8_t*)&gy, sizeof(gy));
      Serial.write((uint8_t*)&gz, sizeof(gz));
      break;
    }


    case READ_GYRO_OFF: {
      float gx, gy, gz;
      readGyroOffset(gx, gy, gz);
      Serial.write((uint8_t*)&gx, sizeof(gx));
      Serial.write((uint8_t*)&gy, sizeof(gy));
      Serial.write((uint8_t*)&gz, sizeof(gz));
      break;
    }
    case WRITE_GYRO_OFF: {
      float gx, gy, gz;
      memcpy(&gx, &data[0], sizeof(float));
      memcpy(&gy, &data[4], sizeof(float));
      memcpy(&gz, &data[8], sizeof(float));
      float res = writeGyroOffset(gx, gy, gz);
      break;
    }


    case READ_GYRO_VAR: {
      float gx, gy, gz;
      readGyroVariance(gx, gy, gz);
      Serial.write((uint8_t*)&gx, sizeof(gx));
      Serial.write((uint8_t*)&gy, sizeof(gy));
      Serial.write((uint8_t*)&gz, sizeof(gz));
      break;
    }
    case WRITE_GYRO_VAR: {
      float gx, gy, gz;
      memcpy(&gx, &data[0], sizeof(float));
      memcpy(&gy, &data[4], sizeof(float));
      memcpy(&gz, &data[8], sizeof(float));
      float res = writeGyroVariance(gx, gy, gz);
      break;
    }

    
    case READ_YAW_WITH_DRIFT: {
      float val = readYawWithDrift();
      Serial.write((uint8_t*)&val, sizeof(val));
      break;
    }


    case READ_YAW_VEL_DRIFT_BIAS: {
      float val = readYawVelDriftBias();
      Serial.write((uint8_t*)&val, sizeof(val));
      break;
    }
    case WRITE_YAW_VEL_DRIFT_BIAS: {
      float val;
      memcpy(&val, &data[1], sizeof(float));
      float res = writeYawVelDriftBias(val);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case SET_I2C_ADDR: {
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setI2cAddress((int)value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }
    case GET_I2C_ADDR: {
      float res = getI2cAddress();
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case SET_FILTER_GAIN: {
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setFilterGain(value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }
    case GET_FILTER_GAIN: {
      float res = getFilterGain();
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case SET_ACC_LPF_CUT_FREQ: {
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setAccFilterCF(value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }
    case GET_ACC_LPF_CUT_FREQ: {
      float res = getAccFilterCF();
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case SET_FRAME_ID: {
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setWorldFrameId((int)value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }
    case GET_FRAME_ID: {
      float res = getWorldFrameId();
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case RESET_PARAMS: {
      float res = triggerResetParams();
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case READ_ACC_GYRO: {
      float ax, ay, az, gx, gy, gz;
      readLinearAcc(ax, ay, az);
      readGyro(gx, gy, gz);
      Serial.write((uint8_t*)&ax, sizeof(ax));
      Serial.write((uint8_t*)&ay, sizeof(ay));
      Serial.write((uint8_t*)&az, sizeof(az));
      Serial.write((uint8_t*)&gx, sizeof(gx));
      Serial.write((uint8_t*)&gy, sizeof(gy));
      Serial.write((uint8_t*)&gz, sizeof(gz));
      break;
    }


    case READ_IMU_DATA: {
      float r, p, y, ax, ay, az, gx, gy, gz;
      readRPY(r, p, y);
      readLinearAcc(ax, ay, az);
      readGyro(gx, gy, gz);
      Serial.write((uint8_t*)&r, sizeof(r));
      Serial.write((uint8_t*)&p, sizeof(p));
      Serial.write((uint8_t*)&y, sizeof(y));
      Serial.write((uint8_t*)&ax, sizeof(ax));
      Serial.write((uint8_t*)&ay, sizeof(ay));
      Serial.write((uint8_t*)&az, sizeof(az));
      Serial.write((uint8_t*)&gx, sizeof(gx));
      Serial.write((uint8_t*)&gy, sizeof(gy));
      Serial.write((uint8_t*)&gz, sizeof(gz));
      break;
    }


    case CLEAR_DATA_BUFFER: {
      float res = clearDataBuffer();
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    default: {
      float error = 0.0;
      Serial.write((uint8_t*)&error, sizeof(error));
      break;
    }
  }

  gpio_set_level((gpio_num_t)LED_PIN, 0);
}








void recieve_and_send_data() {
  static uint8_t state = 0;
  static uint8_t cmd, length;
  static uint8_t buffer[32];
  static uint8_t index = 0;
  static uint8_t checksum = 0;

  while (Serial.available()) {
    uint8_t b = Serial.read();

    switch (state) {
      case 0: // Wait for start
        if (b == START_BYTE) {
          state = 1;
          checksum = b;
        }
        break;

      case 1: // Command
        cmd = b;
        checksum += b;
        state = 2;
        break;

      case 2: // Length
        length = b;
        checksum += b;
        if (length==0){
          state = 4;
        }
        else{
          index = 0;
          state = 3;
        }
        break;

      case 3: // Payload
        buffer[index++] = b;
        checksum += b;
        if (index >= length) state = 4;
        break;

      case 4: // Checksum
        if ((checksum & 0xFF) == b) {
          processCommand(cmd, buffer, length);
        } else {
          float error = 0.0;
          Serial.write((uint8_t*)&error, sizeof(error));
        }
        state = 0; // reset for next packet
        break;
    }
  }
}

#endif
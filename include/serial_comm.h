#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include "command_functions.h"


static inline void processCommand(uint8_t cmd, uint8_t* data) {

  gpio_set_level((gpio_num_t)LED_PIN, 1);

  bool needsFlush = false;

  switch (cmd) {
    case READ_QUAT: {
      float qw, qx, qy, qz;
      readQuat(qw, qx, qy, qz);

      uint8_t tx[16];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &qw, sizeof(qw)); tx_len += 4;
      memcpy(&tx[tx_len], &qx, sizeof(qx)); tx_len += 4;
      memcpy(&tx[tx_len], &qy, sizeof(qy)); tx_len += 4;
      memcpy(&tx[tx_len], &qz, sizeof(qz)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }

    case READ_RPY: {
      float r, p, y;
      readRPY(r, p, y);

      uint8_t tx[12];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &r, sizeof(r)); tx_len += 4;
      memcpy(&tx[tx_len], &p, sizeof(p)); tx_len += 4;
      memcpy(&tx[tx_len], &y, sizeof(y)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }

    case READ_RPY_VAR: {
      float r, p, y;
      readRPYVariance(r, p, y);
      
      uint8_t tx[12];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &r, sizeof(r)); tx_len += 4;
      memcpy(&tx[tx_len], &p, sizeof(p)); tx_len += 4;
      memcpy(&tx[tx_len], &y, sizeof(y)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }
    case WRITE_RPY_VAR: {
      float r = readFloat(data, 0);
      float p = readFloat(data, 4);
      float y = readFloat(data, 8);
      writeRPYVariance(r, p, y);
      break;
    }


    case READ_ACC: {
      float ax, ay, az;
      readAcc(ax, ay, az);
      
      uint8_t tx[12];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &ax, sizeof(ax)); tx_len += 4;
      memcpy(&tx[tx_len], &ay, sizeof(ay)); tx_len += 4;
      memcpy(&tx[tx_len], &az, sizeof(az)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }


    case READ_ACC_RAW: {
      float ax, ay, az;
      readAccRaw(ax, ay, az);
      
      uint8_t tx[12];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &ax, sizeof(ax)); tx_len += 4;
      memcpy(&tx[tx_len], &ay, sizeof(ay)); tx_len += 4;
      memcpy(&tx[tx_len], &az, sizeof(az)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }


    case READ_LIN_ACC: {
      float ax, ay, az;
      readLinearAcc(ax, ay, az);
      
      uint8_t tx[12];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &ax, sizeof(ax)); tx_len += 4;
      memcpy(&tx[tx_len], &ay, sizeof(ay)); tx_len += 4;
      memcpy(&tx[tx_len], &az, sizeof(az)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }


    case READ_LIN_ACC_RAW: {
      float ax, ay, az;
      readLinearAccRaw(ax, ay, az);
      
      uint8_t tx[12];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &ax, sizeof(ax)); tx_len += 4;
      memcpy(&tx[tx_len], &ay, sizeof(ay)); tx_len += 4;
      memcpy(&tx[tx_len], &az, sizeof(az)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }


    case READ_ACC_OFF: {
      float ax, ay, az;
      readAccOffset(ax, ay, az);
      
      uint8_t tx[12];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &ax, sizeof(ax)); tx_len += 4;
      memcpy(&tx[tx_len], &ay, sizeof(ay)); tx_len += 4;
      memcpy(&tx[tx_len], &az, sizeof(az)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }
    case WRITE_ACC_OFF: {
      float ax = readFloat(data, 0);
      float ay = readFloat(data, 4);
      float az = readFloat(data, 8);
      writeAccOffset(ax, ay, az);
      break;
    }


    case READ_ACC_VAR: {
      float ax, ay, az;
      readAccVariance(ax, ay, az);
      
      uint8_t tx[12];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &ax, sizeof(ax)); tx_len += 4;
      memcpy(&tx[tx_len], &ay, sizeof(ay)); tx_len += 4;
      memcpy(&tx[tx_len], &az, sizeof(az)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }
    case WRITE_ACC_VAR: {
      float ax = readFloat(data, 0);
      float ay = readFloat(data, 4);
      float az = readFloat(data, 8);
      writeAccVariance(ax, ay, az);
      break;
    }


    case READ_GYRO: {
      float gx, gy, gz;
      readGyro(gx, gy, gz);
      
      uint8_t tx[12];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &gx, sizeof(gx)); tx_len += 4;
      memcpy(&tx[tx_len], &gy, sizeof(gy)); tx_len += 4;
      memcpy(&tx[tx_len], &gz, sizeof(gz)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }


    case READ_GYRO_RAW: {
      float gx, gy, gz;
      readGyroRaw(gx, gy, gz);
      
      uint8_t tx[12];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &gx, sizeof(gx)); tx_len += 4;
      memcpy(&tx[tx_len], &gy, sizeof(gy)); tx_len += 4;
      memcpy(&tx[tx_len], &gz, sizeof(gz)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }


    case READ_GYRO_OFF: {
      float gx, gy, gz;
      readGyroOffset(gx, gy, gz);
      
      uint8_t tx[12];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &gx, sizeof(gx)); tx_len += 4;
      memcpy(&tx[tx_len], &gy, sizeof(gy)); tx_len += 4;
      memcpy(&tx[tx_len], &gz, sizeof(gz)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }
    case WRITE_GYRO_OFF: {
      float gx = readFloat(data, 0);
      float gy = readFloat(data, 4);
      float gz = readFloat(data, 8);
      writeGyroOffset(gx, gy, gz);
      break;
    }


    case READ_GYRO_VAR: {
      float gx, gy, gz;
      readGyroVariance(gx, gy, gz);
      
      uint8_t tx[12];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &gx, sizeof(gx)); tx_len += 4;
      memcpy(&tx[tx_len], &gy, sizeof(gy)); tx_len += 4;
      memcpy(&tx[tx_len], &gz, sizeof(gz)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }
    case WRITE_GYRO_VAR: {
      float gx = readFloat(data, 0);
      float gy = readFloat(data, 4);
      float gz = readFloat(data, 8);
      writeGyroVariance(gx, gy, gz);
      break;
    }

    
    case READ_MAG: {
      float mx, my, mz;
      readMag(mx, my, mz);
      
      uint8_t tx[12];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &mx, sizeof(mx)); tx_len += 4;
      memcpy(&tx[tx_len], &my, sizeof(my)); tx_len += 4;
      memcpy(&tx[tx_len], &mz, sizeof(mz)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }


    case READ_MAG_RAW: {
      float mx, my, mz;
      readMagRaw(mx, my, mz);
      
      uint8_t tx[12];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &mx, sizeof(mx)); tx_len += 4;
      memcpy(&tx[tx_len], &my, sizeof(my)); tx_len += 4;
      memcpy(&tx[tx_len], &mz, sizeof(mz)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }


    case READ_MAG_H_OFF: {
      float mx, my, mz;
      readMagHardOffset(mx, my, mz);
      
      uint8_t tx[12];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &mx, sizeof(mx)); tx_len += 4;
      memcpy(&tx[tx_len], &my, sizeof(my)); tx_len += 4;
      memcpy(&tx[tx_len], &mz, sizeof(mz)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }
    case WRITE_MAG_H_OFF: {
      float mx = readFloat(data, 0);
      float my = readFloat(data, 4);
      float mz = readFloat(data, 8);
      writeMagHardOffset(mx, my, mz);
      break;
    }


    case READ_MAG_S_OFF0: {
      float mx, my, mz;
      readMagSoftOffset0(mx, my, mz);
      
      uint8_t tx[12];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &mx, sizeof(mx)); tx_len += 4;
      memcpy(&tx[tx_len], &my, sizeof(my)); tx_len += 4;
      memcpy(&tx[tx_len], &mz, sizeof(mz)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }
    case WRITE_MAG_S_OFF0: {
      float mx = readFloat(data, 0);
      float my = readFloat(data, 4);
      float mz = readFloat(data, 8);
      writeMagSoftOffset0(mx, my, mz);
      break;
    }


    case READ_MAG_S_OFF1: {
      float mx, my, mz;
      readMagSoftOffset1(mx, my, mz);
      
      uint8_t tx[12];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &mx, sizeof(mx)); tx_len += 4;
      memcpy(&tx[tx_len], &my, sizeof(my)); tx_len += 4;
      memcpy(&tx[tx_len], &mz, sizeof(mz)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }
    case WRITE_MAG_S_OFF1: {
      float mx = readFloat(data, 0);
      float my = readFloat(data, 4);
      float mz = readFloat(data, 8);
      writeMagSoftOffset1(mx, my, mz);
      break;
    }


    case READ_MAG_S_OFF2: {
      float mx, my, mz;
      readMagSoftOffset2(mx, my, mz);
      
      uint8_t tx[12];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &mx, sizeof(mx)); tx_len += 4;
      memcpy(&tx[tx_len], &my, sizeof(my)); tx_len += 4;
      memcpy(&tx[tx_len], &mz, sizeof(mz)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }
    case WRITE_MAG_S_OFF2: {
      float mx = readFloat(data, 0);
      float my = readFloat(data, 4);
      float mz = readFloat(data, 8);
      writeMagSoftOffset2(mx, my, mz);
      break;
    }


    case SET_I2C_ADDR: {
      float value = readFloat(data, 1);
      setI2cAddress((int)value);
      break;
    }
    case GET_I2C_ADDR: {
      float res = getI2cAddress();
      Serial.write((uint8_t*)&res, sizeof(res));
      needsFlush = true;
      break;
    }


    case SET_FILTER_GAIN: {
      float value = readFloat(data, 1);
      setFilterGain(value);
      break;
    }
    case GET_FILTER_GAIN: {
      float res = getFilterGain();
      Serial.write((uint8_t*)&res, sizeof(res));
      needsFlush = true;
      break;
    }


    case SET_ACC_LPF_CUT_FREQ: {
      float value = readFloat(data, 1);
      setAccFilterCF(value);
      break;
    }
    case GET_ACC_LPF_CUT_FREQ: {
      float res = getAccFilterCF();
      Serial.write((uint8_t*)&res, sizeof(res));
      needsFlush = true;
      break;
    }


    case SET_FRAME_ID: {
      float value = readFloat(data, 1);
      setWorldFrameId((int)value);
      break;
    }
    case GET_FRAME_ID: {
      float res = getWorldFrameId();
      Serial.write((uint8_t*)&res, sizeof(res));
      needsFlush = true;
      break;
    }


    case RESET_PARAMS: {
      float res = triggerResetParams();
      Serial.write((uint8_t*)&res, sizeof(res));
      needsFlush = true;
      break;
    }


    case READ_ACC_GYRO: {
      float ax, ay, az, gx, gy, gz;
      readLinearAcc(ax, ay, az);
      readGyro(gx, gy, gz);
      
      uint8_t tx[24];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &ax, sizeof(ax)); tx_len += 4;
      memcpy(&tx[tx_len], &ay, sizeof(ay)); tx_len += 4;
      memcpy(&tx[tx_len], &az, sizeof(az)); tx_len += 4;
      memcpy(&tx[tx_len], &gx, sizeof(gx)); tx_len += 4;
      memcpy(&tx[tx_len], &gy, sizeof(gy)); tx_len += 4;
      memcpy(&tx[tx_len], &gz, sizeof(gz)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }


    case READ_IMU_DATA: {
      float r, p, y, ax, ay, az, gx, gy, gz;
      readRPY(r, p, y);
      readLinearAcc(ax, ay, az);
      readGyro(gx, gy, gz);
      
      uint8_t tx[36];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &r, sizeof(r)); tx_len += 4;
      memcpy(&tx[tx_len], &p, sizeof(p)); tx_len += 4;
      memcpy(&tx[tx_len], &y, sizeof(y)); tx_len += 4;
      memcpy(&tx[tx_len], &ax, sizeof(ax)); tx_len += 4;
      memcpy(&tx[tx_len], &ay, sizeof(ay)); tx_len += 4;
      memcpy(&tx[tx_len], &az, sizeof(az)); tx_len += 4;
      memcpy(&tx[tx_len], &gx, sizeof(gx)); tx_len += 4;
      memcpy(&tx[tx_len], &gy, sizeof(gy)); tx_len += 4;
      memcpy(&tx[tx_len], &gz, sizeof(gz)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }


    case CLEAR_DATA_BUFFER: {
      float res = clearDataBuffer();
      Serial.write((uint8_t*)&res, sizeof(res));
      needsFlush = true;
      break;
    }


    default: {
      float error = 0.0;
      Serial.write((uint8_t*)&error, sizeof(error));
      needsFlush = true;
      break;
    }
  }

  if (needsFlush) {
    Serial.flush();
  }

  gpio_set_level((gpio_num_t)LED_PIN, 0);

}








static inline void recieve_and_send_data() {
  static uint8_t state = 0;
  static uint8_t cmd, length;
  static uint8_t buffer[40];
  static uint8_t index = 0;
  static uint8_t checksum = 0;

  while (Serial.available()) {
    uint8_t b = Serial.read();

    switch (state) {
      case 0: // Wait for start
        if (b == START_BYTE) {
          state = 1;
          checksum = b;   // reset checksum correctly
        }
        break;

      case 1: // Command
        cmd = b;
        checksum += b;
        state = 2;
        break;

      case 2: // Length
        length = b;

        if (length > sizeof(buffer)) {
          state = 0;
          checksum = 0;
          break;
        }

        checksum += b;
        index = 0;
        state = (length == 0) ? 4 : 3;
        break;

      case 3: // Payload
        if (index < sizeof(buffer)) {
          buffer[index++] = b;
        }
        checksum += b;

        if (index >= length) {
          state = 4;
        }
        break;

      case 4: // Checksum
        if ((checksum & 0xFF) == b) {
          processCommand(cmd, buffer);
        } else {
          float error = 0.0f;
          Serial.write((uint8_t*)&error, sizeof(error));
          Serial.flush();
        }
        state = 0;
        break;
    }
  }
}

#endif
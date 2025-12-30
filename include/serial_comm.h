#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include "command_functions.h"


void processCommand(uint8_t cmd, uint8_t* data) {

  gpio_set_level((gpio_num_t)LED_PIN, 1);
  switch (cmd) {
    case READ_QUAT: {
      float qw, qx, qy, qz;
      readQuat(qw, qx, qy, qz);
      Serial.write((uint8_t*)&qw, sizeof(qw));
      Serial.write((uint8_t*)&qx, sizeof(qx));
      Serial.write((uint8_t*)&qy, sizeof(qy));
      Serial.write((uint8_t*)&qz, sizeof(qz));
      //Serial.flush();
      break;
    }

    case READ_RPY: {
      float r, p, y;
      readRPY(r, p, y);
      Serial.write((uint8_t*)&r, sizeof(r));
      Serial.write((uint8_t*)&p, sizeof(p));
      Serial.write((uint8_t*)&y, sizeof(y));
      //Serial.flush();
      break;
    }

    case READ_RPY_VAR: {
      float r, p, y;
      readRPYVariance(r, p, y);
      Serial.write((uint8_t*)&r, sizeof(r));
      Serial.write((uint8_t*)&p, sizeof(p));
      Serial.write((uint8_t*)&y, sizeof(y));
      //Serial.flush();
      break;
    }
    case WRITE_RPY_VAR: {
      float r, p, y;
      memcpy(&r, &data[0], sizeof(float));
      memcpy(&p, &data[4], sizeof(float));
      memcpy(&y, &data[8], sizeof(float));
      writeRPYVariance(r, p, y);
      break;
    }


    case READ_ACC: {
      float ax, ay, az;
      readAcc(ax, ay, az);
      Serial.write((uint8_t*)&ax, sizeof(ax));
      Serial.write((uint8_t*)&ay, sizeof(ay));
      Serial.write((uint8_t*)&az, sizeof(az));
      //Serial.flush();
      break;
    }


    case READ_ACC_RAW: {
      float ax, ay, az;
      readAccRaw(ax, ay, az);
      Serial.write((uint8_t*)&ax, sizeof(ax));
      Serial.write((uint8_t*)&ay, sizeof(ay));
      Serial.write((uint8_t*)&az, sizeof(az));
      //Serial.flush();
      break;
    }


    case READ_LIN_ACC: {
      float ax, ay, az;
      readLinearAcc(ax, ay, az);
      Serial.write((uint8_t*)&ax, sizeof(ax));
      Serial.write((uint8_t*)&ay, sizeof(ay));
      Serial.write((uint8_t*)&az, sizeof(az));
      //Serial.flush();
      break;
    }


    case READ_LIN_ACC_RAW: {
      float ax, ay, az;
      readLinearAccRaw(ax, ay, az);
      Serial.write((uint8_t*)&ax, sizeof(ax));
      Serial.write((uint8_t*)&ay, sizeof(ay));
      Serial.write((uint8_t*)&az, sizeof(az));
      //Serial.flush();
      break;
    }


    case READ_ACC_BIAS_VECT: {
      float ax, ay, az;
      readAccBias(ax, ay, az);
      Serial.write((uint8_t*)&ax, sizeof(ax));
      Serial.write((uint8_t*)&ay, sizeof(ay));
      Serial.write((uint8_t*)&az, sizeof(az));
      //Serial.flush();
      break;
    }
    case WRITE_ACC_BIAS_VECT: {
      float ax, ay, az;
      memcpy(&ax, &data[0], sizeof(float));
      memcpy(&ay, &data[4], sizeof(float));
      memcpy(&az, &data[8], sizeof(float));
      writeAccBias(ax, ay, az);
      break;
    }


    case READ_ACC_SCALE_MAT0: {
      float ax, ay, az;
      readAccScaleMatR0(ax, ay, az);
      Serial.write((uint8_t*)&ax, sizeof(ax));
      Serial.write((uint8_t*)&ay, sizeof(ay));
      Serial.write((uint8_t*)&az, sizeof(az));
      //Serial.flush();
      break;
    }
    case WRITE_ACC_SCALE_MAT0: {
      float ax, ay, az;
      memcpy(&ax, &data[0], sizeof(float));
      memcpy(&ay, &data[4], sizeof(float));
      memcpy(&az, &data[8], sizeof(float));
      writeAccScaleMatR0(ax, ay, az);
      break;
    }


    case READ_ACC_SCALE_MAT1: {
      float ax, ay, az;
      readAccScaleMatR1(ax, ay, az);
      Serial.write((uint8_t*)&ax, sizeof(ax));
      Serial.write((uint8_t*)&ay, sizeof(ay));
      Serial.write((uint8_t*)&az, sizeof(az));
      //Serial.flush();
      break;
    }
    case WRITE_ACC_SCALE_MAT1: {
      float ax, ay, az;
      memcpy(&ax, &data[0], sizeof(float));
      memcpy(&ay, &data[4], sizeof(float));
      memcpy(&az, &data[8], sizeof(float));
      writeAccScaleMatR1(ax, ay, az);
      break;
    }


    case READ_ACC_SCALE_MAT2: {
      float ax, ay, az;
      readAccScaleMatR2(ax, ay, az);
      Serial.write((uint8_t*)&ax, sizeof(ax));
      Serial.write((uint8_t*)&ay, sizeof(ay));
      Serial.write((uint8_t*)&az, sizeof(az));
      //Serial.flush();
      break;
    }
    case WRITE_ACC_SCALE_MAT2: {
      float ax, ay, az;
      memcpy(&ax, &data[0], sizeof(float));
      memcpy(&ay, &data[4], sizeof(float));
      memcpy(&az, &data[8], sizeof(float));
      writeAccScaleMatR2(ax, ay, az);
      break;
    }


    case READ_ACC_VAR: {
      float ax, ay, az;
      readAccVariance(ax, ay, az);
      Serial.write((uint8_t*)&ax, sizeof(ax));
      Serial.write((uint8_t*)&ay, sizeof(ay));
      Serial.write((uint8_t*)&az, sizeof(az));
      //Serial.flush();
      break;
    }
    case WRITE_ACC_VAR: {
      float ax, ay, az;
      memcpy(&ax, &data[0], sizeof(float));
      memcpy(&ay, &data[4], sizeof(float));
      memcpy(&az, &data[8], sizeof(float));
      writeAccVariance(ax, ay, az);
      break;
    }


    case READ_GYRO: {
      float gx, gy, gz;
      readGyro(gx, gy, gz);
      Serial.write((uint8_t*)&gx, sizeof(gx));
      Serial.write((uint8_t*)&gy, sizeof(gy));
      Serial.write((uint8_t*)&gz, sizeof(gz));
      //Serial.flush();
      break;
    }


    case READ_GYRO_RAW: {
      float gx, gy, gz;
      readGyroRaw(gx, gy, gz);
      Serial.write((uint8_t*)&gx, sizeof(gx));
      Serial.write((uint8_t*)&gy, sizeof(gy));
      Serial.write((uint8_t*)&gz, sizeof(gz));
      //Serial.flush();
      break;
    }


    case READ_GYRO_OFF: {
      float gx, gy, gz;
      readGyroOffset(gx, gy, gz);
      Serial.write((uint8_t*)&gx, sizeof(gx));
      Serial.write((uint8_t*)&gy, sizeof(gy));
      Serial.write((uint8_t*)&gz, sizeof(gz));
      //Serial.flush();
      break;
    }
    case WRITE_GYRO_OFF: {
      float gx, gy, gz;
      memcpy(&gx, &data[0], sizeof(float));
      memcpy(&gy, &data[4], sizeof(float));
      memcpy(&gz, &data[8], sizeof(float));
      writeGyroOffset(gx, gy, gz);
      break;
    }


    case READ_GYRO_VAR: {
      float gx, gy, gz;
      readGyroVariance(gx, gy, gz);
      Serial.write((uint8_t*)&gx, sizeof(gx));
      Serial.write((uint8_t*)&gy, sizeof(gy));
      Serial.write((uint8_t*)&gz, sizeof(gz));
      //Serial.flush();
      break;
    }
    case WRITE_GYRO_VAR: {
      float gx, gy, gz;
      memcpy(&gx, &data[0], sizeof(float));
      memcpy(&gy, &data[4], sizeof(float));
      memcpy(&gz, &data[8], sizeof(float));
      writeGyroVariance(gx, gy, gz);
      break;
    }

    
    case READ_MAG: {
      float mx, my, mz;
      readMag(mx, my, mz);
      Serial.write((uint8_t*)&mx, sizeof(mx));
      Serial.write((uint8_t*)&my, sizeof(my));
      Serial.write((uint8_t*)&mz, sizeof(mz));
      //Serial.flush();
      break;
    }


    case READ_MAG_RAW: {
      float mx, my, mz;
      readMagRaw(mx, my, mz);
      Serial.write((uint8_t*)&mx, sizeof(mx));
      Serial.write((uint8_t*)&my, sizeof(my));
      Serial.write((uint8_t*)&mz, sizeof(mz));
      //Serial.flush();
      break;
    }


    case READ_MAG_H_OFF: {
      float mx, my, mz;
      readMagHardOffset(mx, my, mz);
      Serial.write((uint8_t*)&mx, sizeof(mx));
      Serial.write((uint8_t*)&my, sizeof(my));
      Serial.write((uint8_t*)&mz, sizeof(mz));
      //Serial.flush();
      break;
    }
    case WRITE_MAG_H_OFF: {
      float mx, my, mz;
      memcpy(&mx, &data[0], sizeof(float));
      memcpy(&my, &data[4], sizeof(float));
      memcpy(&mz, &data[8], sizeof(float));
      writeMagHardOffset(mx, my, mz);
      break;
    }


    case READ_MAG_S_OFF0: {
      float mx, my, mz;
      readMagSoftOffset0(mx, my, mz);
      Serial.write((uint8_t*)&mx, sizeof(mx));
      Serial.write((uint8_t*)&my, sizeof(my));
      Serial.write((uint8_t*)&mz, sizeof(mz));
      //Serial.flush();
      break;
    }
    case WRITE_MAG_S_OFF0: {
      float mx, my, mz;
      memcpy(&mx, &data[0], sizeof(float));
      memcpy(&my, &data[4], sizeof(float));
      memcpy(&mz, &data[8], sizeof(float));
      writeMagSoftOffset0(mx, my, mz);
      break;
    }


    case READ_MAG_S_OFF1: {
      float mx, my, mz;
      readMagSoftOffset1(mx, my, mz);
      Serial.write((uint8_t*)&mx, sizeof(mx));
      Serial.write((uint8_t*)&my, sizeof(my));
      Serial.write((uint8_t*)&mz, sizeof(mz));
      //Serial.flush();
      break;
    }
    case WRITE_MAG_S_OFF1: {
      float mx, my, mz;
      memcpy(&mx, &data[0], sizeof(float));
      memcpy(&my, &data[4], sizeof(float));
      memcpy(&mz, &data[8], sizeof(float));
      writeMagSoftOffset1(mx, my, mz);
      break;
    }


    case READ_MAG_S_OFF2: {
      float mx, my, mz;
      readMagSoftOffset2(mx, my, mz);
      Serial.write((uint8_t*)&mx, sizeof(mx));
      Serial.write((uint8_t*)&my, sizeof(my));
      Serial.write((uint8_t*)&mz, sizeof(mz));
      //Serial.flush();
      break;
    }
    case WRITE_MAG_S_OFF2: {
      float mx, my, mz;
      memcpy(&mx, &data[0], sizeof(float));
      memcpy(&my, &data[4], sizeof(float));
      memcpy(&mz, &data[8], sizeof(float));
      writeMagSoftOffset2(mx, my, mz);
      break;
    }


    case SET_I2C_ADDR: {
      float value;
      memcpy(&value, &data[1], sizeof(float));
      setI2cAddress((int)value);
      break;
    }
    case GET_I2C_ADDR: {
      float res = getI2cAddress();
      Serial.write((uint8_t*)&res, sizeof(res));
      //Serial.flush();
      break;
    }


    case SET_FILTER_GAIN: {
      float value;
      memcpy(&value, &data[1], sizeof(float));
      setFilterGain(value);
      break;
    }
    case GET_FILTER_GAIN: {
      float res = getFilterGain();
      Serial.write((uint8_t*)&res, sizeof(res));
      //Serial.flush();
      break;
    }


    case SET_ACC_LPF_CUT_FREQ: {
      float value;
      memcpy(&value, &data[1], sizeof(float));
      setAccFilterCF(value);
      break;
    }
    case GET_ACC_LPF_CUT_FREQ: {
      float res = getAccFilterCF();
      Serial.write((uint8_t*)&res, sizeof(res));
      //Serial.flush();
      break;
    }


    case SET_FRAME_ID: {
      float value;
      memcpy(&value, &data[1], sizeof(float));
      setWorldFrameId((int)value);
      break;
    }
    case GET_FRAME_ID: {
      float res = getWorldFrameId();
      Serial.write((uint8_t*)&res, sizeof(res));
      //Serial.flush();
      break;
    }


    case RESET_PARAMS: {
      float res = triggerResetParams();
      Serial.write((uint8_t*)&res, sizeof(res));
      //Serial.flush();
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
      //Serial.flush();
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
      //Serial.flush();
      break;
    }


    case CLEAR_DATA_BUFFER: {
      float res = clearDataBuffer();
      Serial.write((uint8_t*)&res, sizeof(res));
      //Serial.flush();
      break;
    }


    default: {
      float error = 0.0;
      Serial.write((uint8_t*)&error, sizeof(error));
      //Serial.flush();
      break;
    }
  }

  gpio_set_level((gpio_num_t)LED_PIN, 0);
}








void recieve_and_send_data() {
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
          processCommand(cmd, buffer);
        } else {
          float error = 0.0;
          Serial.write((uint8_t*)&error, sizeof(error));
          //Serial.flush();
        }
        state = 0; // reset for next packet
        break;
    }
  }
}

#endif
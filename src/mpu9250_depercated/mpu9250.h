//
// Created by ysbf on 8/21/20.
//

#ifndef BNO055DRIVER_SRC_MPU9250_MPU9250_H
#define BNO055DRIVER_SRC_MPU9250_MPU9250_H

#include <utility>
#include <cstring>

#include "stdint.h"
#include "mpu9250_reg.h"
#include "functional"

class MPU9250 {
public:
  MPU9250(std::function<void(const uint8_t *buffer, uint16_t len)> spi_read, std::function<void(const uint8_t *buffer, uint16_t len)> spi_write, std::function<void(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t)> callback_fun)> spi_async_read,
          std::function<void(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t)> callback_fun)> spi_async_write, std::function<void(uint16_t)> delay);

  template<typename Read_Handler>
  void async_read_reg(uint8_t reg, uint8_t &value, const Read_Handler &callback_fun) {
    spi_buffer[0] = reg | 0x80u;
    spi_buffer[1] = 0xff;
    spi_async_read(spi_buffer, 2, [&](uint16_t bytes_transferred) {
      value = spi_buffer[1];
      callback_fun(bytes_transferred);
    });
  };

  template<typename Read_Handler>
  void async_read_regs(uint8_t reg, uint8_t *buffer, uint8_t len, const Read_Handler &callback_fun) {
    spi_buffer[0] = reg | 0x80u;
    //  if (len > 32) {
    //    return ;
    //  }
    //  static auto callback = callback_fun;
    memset(spi_buffer + 1, 0xff, len);
    spi_async_read(spi_buffer, len + 1, [this, buffer, len, &callback_fun](uint16_t bytes_transferred) {
      memcpy(buffer, spi_buffer + 1, len);
      //    my_printf("read_regs\r\n");
      callback_fun(bytes_transferred);
    });
  };

  template<typename Write_Handler>
  void async_write_reg(uint8_t reg, uint8_t value, const Write_Handler &callback_fun) {
    spi_buffer[0] = reg & 0x7fu;
    spi_buffer[1] = 0xff;
    spi_async_read(spi_buffer, 2, [&](uint16_t bytes_transferred) {
      value = spi_buffer[1];
      callback_fun(bytes_transferred);
    });
  };

  template<typename Write_Handler>
  void async_write_regs(uint8_t reg, uint8_t *buffer, uint8_t len, const Write_Handler &callback_fun) {
    spi_buffer[0] = (reg & 0x7fu);
    if (len > 32) {
      return;
    }
    memset(spi_buffer + 1, 0xff, len);
    spi_async_read(spi_buffer, len + 1, [&](uint16_t bytes_transferred) {
      memcpy(buffer, &spi_buffer[1], len);
      callback_fun(bytes_transferred);
    });
  };

  uint8_t read_reg(uint8_t reg);

  void write_reg(uint8_t reg, uint8_t value);

  void read_regs(uint8_t reg, const uint8_t *buffer, uint8_t len);

  void get_raw_data(int16_t *accel, int16_t *gyro, int16_t *mag);

#define DEGTORAD(x) ((x) * 0.01745329251994329576923690768489f)
  void get_9axis_data(float *accel, float *gyro, float *mag) {
    int16_t acc_raw[3], gyro_raw[3], mag_raw[3];
    get_raw_data(acc_raw, gyro_raw, mag_raw);

    gyro[0] = DEGTORAD(gyro_raw[1]) * 0.06097560975609756097560975609756f;
    gyro[1] = DEGTORAD(gyro_raw[0]) * 0.06097560975609756097560975609756f;
    gyro[2] = DEGTORAD(gyro_raw[2]) * 0.06097560975609756097560975609756f;

    accel[0] = acc_raw[0] / 16384.0f;
    accel[1] = acc_raw[1] / 16384.0f;
    accel[2] = acc_raw[2] / 16384.0f;

    mag[0] = (float) mag_raw[0];
    mag[1] = (float) mag_raw[1];
    mag[2] = (float) mag_raw[2];

  }

  template<typename Callback>
  void async_get_9axis_data(float *accel, float *gyro, float *mag, const Callback &callback_fun) {
    static int16_t acc_raw[3], gyro_raw[3], mag_raw[3];
    static float *acc_buffer, *gyro_buffer, *mag_buffer;
    acc_buffer = accel;
    gyro_buffer = gyro;
    mag_buffer = mag;
    async_get_raw_data(acc_raw, gyro_raw, mag_raw, [&callback_fun]() {

      acc_buffer[0] = acc_raw[0] / 16384.0f;
      acc_buffer[1] = acc_raw[1] / 16384.0f;
      acc_buffer[2] = acc_raw[2] / 16384.0f;
      //
      gyro_buffer[0] = gyro_raw[1] * 0.06097560975609756097560975609756f * 0.01745329251994329576923690768489f;
      gyro_buffer[1] = gyro_raw[0] * 0.06097560975609756097560975609756f * 0.01745329251994329576923690768489f;
      gyro_buffer[2] = gyro_raw[2] * 0.06097560975609756097560975609756f * 0.01745329251994329576923690768489f;

      mag_buffer[0] = (float) mag_raw[0];
      mag_buffer[1] = (float) mag_raw[1];
      mag_buffer[2] = (float) mag_raw[2];
      callback_fun();
    });

  }

  template<typename Callback>
  void async_get_raw_data(int16_t *accel, int16_t *gyro, int16_t *mag, const Callback &callback_fun) {

    static uint8_t data[22];

    //    static  auto callback=[accel, gyro, mag, &callback_fun](uint16_t bytes_transferred) {
    //      accel[0] = (data[0] << 8) | data[1];
    //      accel[1] = (data[2] << 8) | data[3];
    //      accel[2] = (data[4] << 8) | data[5];
    //
    //      gyro[0] = (data[8] << 8) | data[9];
    //      gyro[1] = (data[10] << 8) | data[11];
    //      gyro[2] = (data[12] << 8) | data[13];
    //
    //      if (!(data[14] & MPU9250_AK8963_DATA_READY) || (data[14] & MPU9250_AK8963_DATA_OVERRUN)) {
    //        //      return;
    //      }
    //      if (data[21] & MPU9250_AK8963_OVERFLOW) {
    //        //      return;
    //      }
    //      mag[0] = (data[16] << 8) | data[15];
    //      mag[1] = (data[18] << 8) | data[17];
    //      mag[2] = (data[20] << 8) | data[19];
    //      //    my_printf("world\r\n");
    //      callback_fun();
    //      //      //ned x,y,z
    //      //      mag[0] = ((long) mag[0] * MPU9250_AK8963_ASA[0]) >> 8;
    //      //      mag[1] = ((long) mag[1] * MPU9250_AK8963_ASA[1]) >> 8;
    //      //      mag[2] = ((long) mag[2] * MPU9250_AK8963_ASA[2]) >> 8;
    //
    //    } ;


    async_read_regs(MPU9250_ACCEL_XOUT_H, data, 22, [accel, gyro, mag, &callback_fun](uint16_t bytes_transferred) {
      accel[0] = (data[0] << 8) | data[1];
      accel[1] = (data[2] << 8) | data[3];
      accel[2] = (data[4] << 8) | data[5];

      gyro[0] = (data[8] << 8) | data[9];
      gyro[1] = (data[10] << 8) | data[11];
      gyro[2] = (data[12] << 8) | data[13];

      if (!(data[14] & MPU9250_AK8963_DATA_READY) || (data[14] & MPU9250_AK8963_DATA_OVERRUN)) {
        //      return;
      }
      if (data[21] & MPU9250_AK8963_OVERFLOW) {
        //      return;
      }
      mag[0] = (data[16] << 8) | data[15];
      mag[1] = (data[18] << 8) | data[17];
      mag[2] = (data[20] << 8) | data[19];
      //    my_printf("world\r\n");
      callback_fun();
      //      //ned x,y,z
      //      mag[0] = ((long) mag[0] * MPU9250_AK8963_ASA[0]) >> 8;
      //      mag[1] = ((long) mag[1] * MPU9250_AK8963_ASA[1]) >> 8;
      //      mag[2] = ((long) mag[2] * MPU9250_AK8963_ASA[2]) >> 8;

    });
  };

  //void MPU9250_DeviceInit_2(uint8_t Ascale, uint8_t Gscale, uint8_t sampleRate)
  void init();

private:
  std::function<void(const uint8_t *buffer, uint16_t len)> spi_read;
  std::function<void(const uint8_t *buffer, uint16_t len)> spi_write;

  std::function<void(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t)> callback_fun)> spi_async_read;
  std::function<void(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t)> callback_fun)> spi_async_write;

  std::function<void(uint16_t)> delay;
  uint8_t spi_buffer[33]{0};
};

#endif //BNO055DRIVER_SRC_MPU9250_MPU9250_H

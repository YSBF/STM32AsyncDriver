//
// Created by ysbf on 9/8/20.
//

#ifndef MYECL_LIB_DRIVERS_STM32_SENSORS_L3GD20_L3GD20_H
#define MYECL_LIB_DRIVERS_STM32_SENSORS_L3GD20_L3GD20_H
#include <utility>
#include <cstring>

#include "stdint.h"
#include "l3gd20_reg.h"
#include "functional"
#include "../SPI_Device.h"

class L3GD20 : SPI_Device {
public:
  using SPI_Device::SPI_Device;
  typedef enum {
    GYRO_RANGE_250DPS = 250,
    GYRO_RANGE_500DPS = 500,
    GYRO_RANGE_2000DPS = 2000
  } gyroRange_t;

  void get_gyro_data(float *gyro);
  uint8_t check() {
    uint8_t ok = read_reg(WHO_AM_I);

    if (ok == 0xd4) {
      return 1;
    }
    return 0;
  }

  bool init(gyroRange_t range);;

  void read_regs(uint8_t reg, const uint8_t *buffer, uint8_t len){
    uint8_t temp[len + 1];
    memset(temp,0,len+1);
    temp[0] = reg | 0xc0u;
    spi_write(temp, len + 1);
    memcpy((void *) buffer, temp + 1, len);

//    SPI_Device::read_regs(reg,buffer,len);
  };



private:
  gyroRange_t m_range;

};

#endif //MYECL_LIB_DRIVERS_STM32_SENSORS_L3GD20_L3GD20_H

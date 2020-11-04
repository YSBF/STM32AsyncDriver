#include "l3gd20.h"
#include "string.h"

void L3GD20::get_gyro_data(float *gyro) {
  uint8_t data[6];

  read_regs(OUT_X_L, data, 6);

  int gyro_raw[3];
  //	SPI3_DMA_ReadRegisterBuffer( MPU9250_ACCEL_XOUT_H, data, 22);
  gyro_raw[0] = (data[1] << 8u) | data[0];
  gyro_raw[1] = (data[3] << 8u) | data[2];
  gyro_raw[2] = (data[5] << 8u) | data[4];

  // Compensate values depending on the resolution
  switch (m_range) {
  case GYRO_RANGE_250DPS:gyro[0] = gyro_raw[0]*GYRO_SENSITIVITY_250DPS;
    gyro[1] =gyro_raw[1]* GYRO_SENSITIVITY_250DPS;
    gyro[2] =gyro_raw[2]* GYRO_SENSITIVITY_250DPS;
    break;
  case GYRO_RANGE_500DPS:gyro[0] =gyro_raw[0]* GYRO_SENSITIVITY_500DPS;
    gyro[1] =gyro_raw[1]* GYRO_SENSITIVITY_500DPS;
    gyro[2] =gyro_raw[2]* GYRO_SENSITIVITY_500DPS;
    break;
  case GYRO_RANGE_2000DPS:gyro[0] =gyro_raw[0]* GYRO_SENSITIVITY_2000DPS;
    gyro[1] =gyro_raw[1]* GYRO_SENSITIVITY_2000DPS;
    gyro[2] =gyro_raw[2]* GYRO_SENSITIVITY_2000DPS;
    break;
  }
}
bool L3GD20::init(L3GD20::gyroRange_t range) {
  m_range = range;
  while (check() == 0);
  if (check() == 0)
    return false;

  // Enable x, y, z and turn off power down:
  //	L3G4200D_WriteReg(CTRL_REG1, 0b11111111);
  write_reg(CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  //	L3G4200D_WriteReg(CTRL_REG2, 0b00000000);
  write_reg(CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  //	L3G4200D_WriteReg(CTRL_REG3, 0b00001000);
  //		L3G4200D_WriteReg(CTRL_REG3, 0b10000100);
  // CTRL_REG4 controls the full-scale range, among other things:
  //    fullScale &= 0x03u;
  //    write_reg(CTRL_REG4, fullScale << 4u);


  switch (m_range) {
  case GYRO_RANGE_250DPS:write_reg(CTRL_REG4, 0x00);
    break;
  case GYRO_RANGE_500DPS:write_reg(CTRL_REG4, 0x10);
    break;
  case GYRO_RANGE_2000DPS:write_reg(CTRL_REG4, 0x20);
    break;
  }


  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  write_reg(CTRL_REG5, 0b00000000);

  return true;
}

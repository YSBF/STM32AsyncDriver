//
// Created by ysbf on 8/21/20.
//

#include "stdint.h"
uint8_t check_result;

#include "mpu9250.h"
void MPU9250::init() {

  do {
    check_result = read_reg(MPU9250_WHO_AM_I);
  } while (check_result != 0x71);


  //		MPU9250_Calibrate(g_bias, a_bias);
  // wake up device

  write_reg(MPU9250_PWR_MGMT_1, MPU9250_RESET); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Wait for all registers to reset

  // get stable time source
  write_reg(MPU9250_PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200);

  write_reg(MPU9250_USER_CTRL, MPU9250_I2C_IF_DIS);          // Disable I2C Master mode

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = read_reg(MPU9250_GYRO_CONFIG); // get current GYRO_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x02u; // Clear Fchoice bits [1:0]
  c = c & ~0x18u; // Clear AFS bits [4:3]
  c = c | MPU9250_FSR_250DPS << 3u; // Set full scale range for the gyro
  // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  write_reg(MPU9250_GYRO_CONFIG, c); // Write new GYRO_CONFIG value to register

  // Set accelerometer full-scale range configuration
  c = read_reg(MPU9250_ACCEL_CONFIG); // get current ACCEL_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18u;  // Clear AFS bits [4:3]
  c = c | MPU9250_FSR_2G << 3u; // Set full scale range for the accelerometer
  write_reg(MPU9250_ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value


  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  write_reg(MPU9250_SMPLRT_DIV, 0);  // Use a 1000 Hz rate; a rate consistent with the filter update rate determined inset in CONFIG above



  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  write_reg(MPU9250_CONFIG, 0x03);


  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz

  c = read_reg(MPU9250_ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0Fu; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03u;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  write_reg(MPU9250_ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  write_reg(MPU9250_INT_PIN_CFG, 0x10);  // INT is 50 microsecond pulse and any read to clear
  write_reg(MPU9250_INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
  delay(100);



  //  write_reg(MPU9250_USER_CTRL, 0x20);          // Enable I2C Master mode


  //			delay(1);
  //			uint8_t state = read_reg(MPU9250_USER_CTRL);
  //			delay(1);
  //			write_reg(MPU9250_USER_CTRL, state | MPU9250_I2C_MST_EN);
  //			delay(1);

  //  write_reg(MPU9250_I2C_MST_CTRL, 0x1D);       // I2C configuration STOP after each transaction, master I2C bus at 400 KHz
  //  write_reg(MPU9250_I2C_MST_DELAY_CTRL, 0x81); // Use blocking data retreival and enable delay for mag sample rate mismatch
  //  write_reg(MPU9250_I2C_SLV4_CTRL, 0x01);      // Delay mag data retrieval to once every other accel/gyro data sample


  delay(100);


  //
  //    //	  static uint8_t Bias_Cleaner[6]={0,0,0,0,0,0};
  //    //	  SPI3_WriteRegisterBuffer(MPU9250_XG_OFFSET_H,Bias_Cleaner , 6);
  //    //	  SPI3_WriteRegisterBuffer(MPU9250_XA_OFFSET_H,Bias_Cleaner , 6);
  //    for (int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
  //      static uint32_t lasttimeget;
  //
  //      while (micros() < (lasttimeget + 1000));
  //      lasttimeget = micros();
  //      SPI3_ReadRegisterBuffer(MPU9250_ACCEL_XOUT_H, &rawData[0], 6);        // Read the six raw data registers into data array
  //      Acc_Bias[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
  //      Acc_Bias[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
  //      Acc_Bias[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);
  //
  //      SPI3_ReadRegisterBuffer(MPU9250_GYRO_XOUT_H, &rawData[0], 6);       // Read the six raw data registers sequentially into data array
  //      Gyro_Bias[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
  //      Gyro_Bias[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
  //      Gyro_Bias[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);
  //    }
  //
  //    for (int ii = 0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
  //      Acc_Bias[ii] /= 200;
  //      Gyro_Bias[ii] /= 200;
  //    }

  //	MPU9250_Set_Gyro_Bias(&Gyro_Bias);
  //	  	MPU9250_Set_Acc_Bias(&Acc_Bias);




  //
  //	SPI3_WriteRegisterBuffer(MPU9250_XG_OFFSET_H, Buffer_Test, 6);
  //	SPI3_WriteRegisterBuffer(MPU9250_XA_OFFSET_H, Buffer_Test, 6);
  //	SPI3_WriteRegisterBuffer(MPU9250_SELF_TEST_X_GYRO, Buffer_Test, 6);
  //	SPI3_WriteRegisterBuffer(MPU9250_SELF_TEST_X_ACCEL, Buffer_Test, 6);
  //		result_Gyro=MPU9250_Gyro_Self_Test(bias_regular,gyro_result);
  //		 result_Acc=MPU9250_Acc_Self_Test(bias_regular,accel_result);
  //		MPU9250_ReadAccCalib();
  //SPI3_ReadRegisterBuffer(MPU9250_XG_OFFSET_H, Gyro_Calibrate_Value, 6);
  //   SPI3_ReadRegisterBuffer(MPU9250_XA_OFFSET_H, Acc_Calibrate_Value, 6);
  //   SPI3_ReadRegisterBuffer(MPU9250_SELF_TEST_X_GYRO, Gyro_SelfTest_Value, 6);
  //   		   SPI3_ReadRegisterBuffer(MPU9250_SELF_TEST_X_ACCEL, Acc_SelfTest_Value, 6);

}
MPU9250::MPU9250(std::function<void(const uint8_t *, uint16_t)> spi_read, std::function<void(const uint8_t *, uint16_t)> spi_write, std::function<void(const uint8_t *, uint16_t, std::function<void(uint16_t)>)> spi_async_read, std::function<void(const uint8_t *, uint16_t, std::function<void(uint16_t)>)> spi_async_write, std::function<void(uint16_t)> delay)
    : spi_read(std::move(spi_read)), spi_write(std::move(spi_write)),
      spi_async_read(std::move(spi_async_read)), spi_async_write(std::move(spi_async_write)), delay(std::move(delay)) {

}

#include "system/debug.h"



uint8_t MPU9250::read_reg(uint8_t reg) {
  uint8_t temp[2];
  temp[0] = reg | 0x80u;
  spi_write(temp, 2);
  return temp[1];
}
void MPU9250::write_reg(uint8_t reg, uint8_t value) {
  reg &= (uint8_t) 0x7fu;
  uint8_t temp[2]{reg, value};
  spi_write(temp, 2);
}


void MPU9250::read_regs(uint8_t reg, const uint8_t *buffer, uint8_t len) {
  uint8_t temp[len + 1];
  temp[0] = reg | 0x80u;
  spi_write(temp, len + 1);
  memcpy((void *) buffer, temp + 1, len);
}
void MPU9250::get_raw_data(int16_t *accel, int16_t *gyro, int16_t *mag) {
  uint8_t data[22];

  read_regs(MPU9250_ACCEL_XOUT_H, data, 22);

  //	SPI3_DMA_ReadRegisterBuffer( MPU9250_ACCEL_XOUT_H, data, 22);
  accel[0] = (data[0] << 8) | data[1];
  accel[1] = (data[2] << 8) | data[3];
  accel[2] = (data[4] << 8) | data[5];

  gyro[0] = (data[8] << 8) | data[9];
  gyro[1] = (data[10] << 8) | data[11];
  gyro[2] = (data[12] << 8) | data[13];

  if (!(data[14] & MPU9250_AK8963_DATA_READY) || (data[14] & MPU9250_AK8963_DATA_OVERRUN)) {
    return;
  }
  if (data[21] & MPU9250_AK8963_OVERFLOW) {
    return;
  }
  mag[0] = (data[16] << 8) | data[15];
  mag[1] = (data[18] << 8) | data[17];
  mag[2] = (data[20] << 8) | data[19];
  //      //ned x,y,z
  //      mag[0] = ((long) mag[0] * MPU9250_AK8963_ASA[0]) >> 8;
  //      mag[1] = ((long) mag[1] * MPU9250_AK8963_ASA[1]) >> 8;
  //      mag[2] = ((long) mag[2] * MPU9250_AK8963_ASA[2]) >> 8;

}


//
// Created by ysbf on 9/8/20.
//

#ifndef MYECL_LIB_DRIVERS_STM32_SENSORS_SPI_DEVICE_H
#define MYECL_LIB_DRIVERS_STM32_SENSORS_SPI_DEVICE_H
#include <utility>
#include <cstring>

#include <cstdint>
#include "functional"


class SPI_Device {
public:
  SPI_Device(std::function<void(const uint8_t *buffer, uint16_t len)> spi_read, std::function<void(const uint8_t *buffer, uint16_t len)> spi_write, std::function<void(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t)> callback_fun)> spi_async_read,
          std::function<void(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t)> callback_fun)> spi_async_write, std::function<void(uint16_t)> delay)
      : spi_read(std::move(spi_read)), spi_write(std::move(spi_write)),
        spi_async_read(std::move(spi_async_read)), spi_async_write(std::move(spi_async_write)), delay(std::move(delay)) {

  };

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

   uint8_t read_reg(uint8_t reg){
    uint8_t temp[2];
    temp[0] = reg | 0x80u;
    spi_write(temp, 2);
    return temp[1];
  };

   void write_reg(uint8_t reg, uint8_t value){
    reg &= (uint8_t) 0x7fu;
    uint8_t temp[2]{reg, value};
    spi_write(temp, 2);
  };

  virtual void read_regs(uint8_t reg, const uint8_t *buffer, uint8_t len){
    uint8_t temp[len + 1];
    temp[0] = reg | 0x80u;
    spi_write(temp, len + 1);
    memcpy((void *) buffer, temp + 1, len);
  };

protected:
  std::function<void(const uint8_t *buffer, uint16_t len)> spi_read;
  std::function<void(const uint8_t *buffer, uint16_t len)> spi_write;

  std::function<void(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t)> callback_fun)> spi_async_read;
  std::function<void(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t)> callback_fun)> spi_async_write;

  std::function<void(uint16_t)> delay;
  uint8_t spi_buffer[33]{0};




};





#endif //MYECL_LIB_DRIVERS_STM32_SENSORS_SPI_DEVICE_H

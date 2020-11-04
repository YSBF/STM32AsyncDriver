//
// Created by ysbf on 8/18/20.
//

#ifndef BNO055DRIVER_CONNECTIVITY_CONNECT_H
#define BNO055DRIVER_CONNECTIVITY_CONNECT_H
#include <cstdint>
#include <utility>

#include "functional"

class connect {

  connect() {};
  virtual  bool init();

  static uint8_t read_byte(uint8_t *data);;
  static uint8_t write_byte(const uint8_t value);;

  static uint8_t read_bytes(uint8_t *buffer, const uint8_t len);;
  static uint8_t write_bytes(const uint8_t *buffer, const uint8_t len);;

#define dma_uart_maxsize 1024

  static void dma_uart_read_irq_handler();
  static void dma_uart_write_irq_handler();;

  static void dma_async_read(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t)> callback_fun);

  static uint8_t dma_read_byte(uint8_t *data);;
  static uint8_t dma_read_bytes(const uint8_t *buffer, uint16_t len);
  static uint8_t dma_write_byte(const uint8_t value);;

  static bool dma_async_write(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t)> callback_fun);
  static uint8_t dma_write_bytes(const uint8_t *buffer, uint16_t len);;

private:
  static std::function<void(uint16_t bytes_transferred)> uart_read_callback_fun;
  static std::function<void(uint16_t bytes_transferred)> uart_write_callback_fun;
  static uint16_t read_len;
  static uint16_t write_len;
  static uint8_t is_first_time_read;
  static uint8_t is_first_time_writ

};

#endif //BNO055DRIVER_CONNECTIVITY_CONNECT_H

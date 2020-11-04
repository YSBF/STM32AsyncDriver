#ifndef  BNO055DRIVER_CONNECTIVITY_UART4_H
#define BNO055DRIVER_CONNECTIVITY_UART4_H

#include <cstdint>
#include <utility>

#include "functional"

class UART4Driver {

public:
  explicit UART4Driver(bool use_dma);
   static bool init();

   static uint8_t read_byte(uint8_t *data);
   static uint8_t write_byte(uint8_t value);

   static uint8_t read_bytes(uint8_t *buffer, uint8_t len);
   static uint8_t write_bytes(const uint8_t *buffer, uint8_t len);

#define dma_uart_maxsize 1024

   void dma_uart_read_irq_handler(uint8_t remain_len);
   void dma_uart_write_irq_handler(uint8_t remain_len);

   void dma_async_read(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t)> callback_fun);

   uint8_t dma_read_byte(uint8_t *data);
   uint8_t dma_read_bytes(const uint8_t *buffer, uint16_t len);
   uint8_t dma_write_byte(uint8_t value);

   bool dma_async_write(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t)> callback_fun);
   uint8_t dma_write_bytes(const uint8_t *buffer, uint16_t len);

private:
   std::function<void(uint16_t bytes_transferred)> uart_read_callback_fun;
   std::function<void(uint16_t bytes_transferred)> uart_write_callback_fun;
   uint16_t read_len{0};
   uint16_t write_len{0};
   uint8_t is_first_time_read{1};
   uint8_t is_first_time_write{1};

};

#endif //BNO055DRIVER_CONNECTIVITY_UART4_H

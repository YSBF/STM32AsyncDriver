#ifndef  BNO055DRIVER_CONNECTIVITY_UART_H
#define BNO055DRIVER_CONNECTIVITY_UART_H

#include <cstdint>
#include <utility>
#include <stm32f4xx.h>

#include "functional"


class UARTDriver {
public:
  enum UART_MOOD{
    TX,
    RX,
    T_RX,
  };

  enum GPIO{
    A,
    B,
    C,
    D,
    E
  };

  struct UART_INFO{
    uint8_t Num;
    int BaudRate;
    bool TxUseDma;
    bool RxUseDma;
    UART_MOOD Mode;
    GPIO GpioTx;
    GPIO GpioRx;

    uint8_t GpioTxPinNum;
    uint8_t GpioRxPinNum;

  };

  struct UART_CALLBACK {
    USART_TypeDef *uart;
    DMA_Stream_TypeDef *dma_tx_stream;
    DMA_Stream_TypeDef *dma_rx_stream;
    uint32_t dma_channel;


    IRQn dma_tx_irq;
    IRQn dma_rx_irq;
    IRQn uart_irq;
    uint32_t dma_tx_flag_tc;
    uint32_t dma_rx_flag_tc;

    std::function<void()> UART_IDLE_HANDLER;
    std::function<void()> DMA_TC_HANDLER;
    std::function<void()> DMA_RC_HANDLER;
  };

public:
  explicit UARTDriver();
  explicit UARTDriver(UART_INFO &info){
    init (info);
  };
    bool init(UART_INFO &info);

    uint8_t read_byte(uint8_t *data);
    uint8_t write_byte(uint8_t value);

    uint8_t read_bytes(uint8_t *buffer, uint8_t len);
    uint8_t write_bytes(const uint8_t *buffer, uint8_t len);

#define dma_uart_maxsize 1024

   void dma_uart_read_irq_handler();
   void dma_uart_write_irq_handler();

   void dma_async_read(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t)> callback_fun);

   uint8_t dma_read_byte(uint8_t *data);
   uint8_t dma_read_bytes(const uint8_t *buffer, uint16_t len);
   uint8_t dma_write_byte(uint8_t value);

   bool dma_async_write(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t)> callback_fun);
   uint8_t dma_write_bytes(const uint8_t *buffer, uint16_t len);

  static UART_CALLBACK UART_Callback[7];
private:
   std::function<void(uint16_t bytes_transferred)> uart_read_callback_fun;
   std::function<void(uint16_t bytes_transferred)> uart_write_callback_fun;
   uint16_t read_len{0};
   uint16_t write_len{0};
   uint8_t is_first_time_read{1};
   uint8_t is_first_time_write{1};
  UART_INFO m_info;


};

#endif //BNO055DRIVER_CONNECTIVITY_UART_H

//
// Created by ysbf on 8/18/20.
//

#ifndef BNO055DRIVER_CONNECTIVITY_SPI_H
#define BNO055DRIVER_CONNECTIVITY_SPI_H

#include <functional>
#include "stm32f4xx_conf.h"

#include <cstring>

//#define CE_LOW	GPIO_ResetBits(GPIOB, GPIO_Pin_14);//reset CSN
//#define CE_HIGHT	GPIO_SetBits(GPIOB, GPIO_Pin_14);//reset CSN

//#define SPI1_DMA_NUM 2
//#define SPI1_DMA_NUM 2
//#define SPI1_RX_DMA_STREAM_NUM 0
//#define SPI1_TX_DMA_STREAM_NUM 5






class SPIDirver {

public:

  enum GPIO {
    A = 0,
    B,
    C,
    D,
    E
  };

  struct SPI_INIT_INFO {
    uint8_t Num;
    int BaudRate;
    bool TxUseDma;
    bool RxUseDma;

    GPIO GpioSCK;
    GPIO GpioMISO;
    GPIO GpioMOSI;
    GPIO GpioCS;

    uint8_t GpioSCKPinNum;
    uint8_t GpioMISOPinNum;
    uint8_t GpioMOSIPinNum;
    uint8_t GpioCSPinNum;

  };

  struct SPI_CALLBACK {
    SPI_TypeDef *spi;
    DMA_Stream_TypeDef *dma_tx_stream;
    DMA_Stream_TypeDef *dma_rx_stream;
    uint32_t dma_channel;

    IRQn dma_tx_irq;
    IRQn dma_rx_irq;
    IRQn spi_irq;
    uint32_t dma_tx_flag_tc;
    uint32_t dma_rx_flag_tc;

    std::function<void()> DMA_TC_HANDLER;
    std::function<void()> DMA_RC_HANDLER;
  };

  explicit SPIDirver() {};
  explicit SPIDirver(SPI_INIT_INFO &info) {
    init(info);
  };

  void init(SPI_INIT_INFO &info);

  void dma_spi_read_irq_handler();;
  void dma_spi_write_irq_handler();;

  void read_bytes(const uint8_t *buffer, uint16_t len);
  void write_bytes(const uint8_t *buffer, uint16_t len);

  void dma_read_bytes(const uint8_t *buffer, uint16_t len);;
  void dma_write_bytes(const uint8_t *buffer, uint16_t len);;

  void dma_async_read(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t)> callback_fun);
  void dma_async_write(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t)> callback_fun);
  uint8_t dma_read_byte();;

  void dma_write_byte(uint8_t value);;

  static SPI_CALLBACK SPI_Callback[3];
private:
  std::function<void(uint16_t bytes_transferred)> spi_read_callback_fun;
  std::function<void(uint16_t bytes_transferred)> spi_write_callback_fun;
  uint16_t read_len{0};
  uint16_t write_len{0};
  uint8_t is_first_time_dma_read_write{1};
  uint8_t SPI_Tx_DMA_Buffer[33]{0};

  [[nodiscard]] uint8_t SPI_RW(uint8_t data) const;

  uint8_t SPI_ReadReg(uint8_t reg);

  void SPI_WriteReg(uint8_t reg, uint8_t value);

  void SPI_ReadMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count);

  void SPI_WriteMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count);

  uint8_t SPI_DMA_ReadReg(uint8_t reg);

  void SPI_DMA_WriteReg(uint8_t reg, uint8_t value);

  void SPI_DMA_ReadMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count);

  void SPI_DMA_WriteMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count);

private:

  SPI_INIT_INFO m_info{};

};

#endif //BNO055DRIVER_CONNECTIVITY_SPI_H

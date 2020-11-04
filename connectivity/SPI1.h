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


#define SPI1_RX_DMA_STREAM DMA2_Stream0
#define SPI1_RX_DMA_CHANNL DMA_Channel_3
#define SPI1_TX_DMA_STREAM DMA2_Stream5
#define SPI1_TX_DMA_CHANNL DMA_Channel_3

#define SPI1_RX_DMA_IRQ DMA2_Stream0_IRQn
#define SPI1_TX_DMA_IRQ DMA2_Stream5_IRQn
#define SPI1_IRQ SPI1_IRQn



#define SPI1_RX_DMA_IRQHandler DMA2_Stream0_IRQHandler
#define SPI1_TX_DMA_IRQHandler DMA2_Stream5_IRQHandler
#define SPI1_IRQHandler SPI1_IRQHandler

#define SPI1_RX_DMA_IT_FEIF DMA_IT_FEIF0
#define SPI1_RX_DMA_IT_DMEIF DMA_IT_DMEIF0
#define SPI1_RX_DMA_IT_TEIF DMA_IT_TEIF0
#define SPI1_RX_DMA_IT_HTIF DMA_IT_HTIF0
#define SPI1_RX_DMA_IT_TCIF DMA_IT_TCIF0

#define SPI1_TX_DMA_IT_FEIF DMA_IT_FEIF5
#define SPI1_TX_DMA_IT_DMEIF DMA_IT_DMEIF5
#define SPI1_TX_DMA_IT_TEIF DMA_IT_TEIF5
#define SPI1_TX_DMA_IT_HTIF DMA_IT_HTIF5
#define SPI1_TX_DMA_IT_TCIF DMA_IT_TCIF5

#define SPI1_RX_DMA_FLAG_FEIF DMA_FLAG_FEIF0
#define SPI1_RX_DMA_FLAG_DMEIF DMA_FLAG_DMEIF0
#define SPI1_RX_DMA_FLAG_TEIF DMA_FLAG_TEIF0
#define SPI1_RX_DMA_FLAG_HTIF DMA_FLAG_HTIF0
#define SPI1_RX_DMA_FLAG_TCIF DMA_FLAG_TCIF0

#define SPI1_TX_DMA_FLAG_FEIF DMA_FLAG_FEIF5
#define SPI1_TX_DMA_FLAG_DMEIF DMA_FLAG_DMEIF5
#define SPI1_TX_DMA_FLAG_TEIF DMA_FLAG_TEIF5
#define SPI1_TX_DMA_FLAG_HTIF DMA_FLAG_HTIF5
#define SPI1_TX_DMA_FLAG_TCIF DMA_FLAG_TCIF5


#define SPI1_SCK_GPIO GPIOA
#define SPI1_MISO_GPIO GPIOA
#define SPI1_MOSI_GPIO GPIOA
#define SPI1_CS_GPIO GPIOA

#define SPI1_SCK_PIN GPIO_Pin_5
#define SPI1_MISO_PIN GPIO_Pin_6
#define SPI1_MOSI_PIN GPIO_Pin_7
#define SPI1_CS_PIN GPIO_Pin_4

#define SPI1_SCK_AF_PIN_SOURCE GPIO_PinSource5
#define SPI1_MISO_AF_PIN_SOURCE GPIO_PinSource6
#define SPI1_MOSI_AF_PIN_SOURCE  GPIO_PinSource7

#define SPI1_CLK_FUNCTION RCC_APB2PeriphClockCmd
#define SPI1_CLK RCC_APB2ENR_SPI1EN
#define SPI1_GPIO_FUNCTION RCC_AHB1PeriphClockCmd
#define SPI1_GPIO_CLK RCC_AHB1Periph_GPIOA
#define SPI1_DMA_FUNCTION RCC_AHB1PeriphClockCmd
#define SPI1_DMA_CLK RCC_AHB1Periph_DMA2


#define CSN_LOW    GPIO_ResetBits(SPI1_CS_GPIO, SPI1_CS_PIN);//reset CSN
#define CSN_HIGHT    GPIO_SetBits(SPI1_CS_GPIO, SPI1_CS_PIN);//reset CSN

class SPI1Dirver{

public:
  explicit SPI1Dirver(bool use_dma){};


  void init();


  void dma_spi_read_irq_handler(uint8_t remain_len);;
  void dma_spi_write_irq_handler(uint8_t remain_len);;

  void read_bytes(const uint8_t *buffer, uint16_t len);
  void write_bytes(const uint8_t *buffer, uint16_t len);

  void dma_read_bytes(const uint8_t *buffer, uint16_t len);;
  void dma_write_bytes(const uint8_t *buffer, uint16_t len);;

  void dma_async_read(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t)> callback_fun);
  void dma_async_write(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t)> callback_fun);
  uint8_t dma_read_byte();;

  void dma_write_byte(uint8_t value);;




private:
  std::function<void(uint16_t bytes_transferred)> spi_read_callback_fun;
  std::function<void(uint16_t bytes_transferred)> spi_write_callback_fun;
  uint16_t read_len{0};
  uint16_t write_len{0};
  uint8_t is_first_time_dma_read{1};
  uint8_t is_first_time_write{1};
  uint8_t SPI1_Tx_DMA_Buffer[33]{0};

  static uint8_t SPI1_RW(uint8_t data);

  uint8_t SPI1_ReadReg(uint8_t reg);

  void SPI1_WriteReg(uint8_t reg, uint8_t value);


  void SPI1_ReadMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count);

  void SPI1_WriteMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count);


  uint8_t SPI1_DMA_ReadReg(uint8_t reg);


  void SPI1_DMA_WriteReg(uint8_t reg, uint8_t value);


  void SPI1_DMA_ReadMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count);


  void SPI1_DMA_WriteMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count);



};







#endif //BNO055DRIVER_CONNECTIVITY_SPI_H

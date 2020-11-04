
#include "SPI.h"

SPIDirver::SPI_CALLBACK SPIDirver::SPI_Callback[3]{

    SPI_CALLBACK{
        SPI1,
        DMA2_Stream5,
        DMA2_Stream0,
        DMA_Channel_3,
        DMA2_Stream5_IRQn,
        DMA2_Stream0_IRQn,
        SPI1_IRQn,
        DMA_FLAG_TCIF5,
        DMA_FLAG_TCIF0,
        std::function<void()>{},
        std::function<void()>{},
    },
    SPI_CALLBACK{
        SPI2,
        DMA1_Stream4,
        DMA1_Stream3,
        DMA_Channel_0,
        DMA1_Stream4_IRQn,
        DMA1_Stream3_IRQn,
        SPI2_IRQn,
        DMA_FLAG_TCIF4,
        DMA_FLAG_TCIF3,
        std::function<void()>{},
        std::function<void()>{},
    },
    SPI_CALLBACK{
        SPI3,
        DMA1_Stream7,
        DMA1_Stream0,
        DMA_Channel_0,
        DMA1_Stream7_IRQn,
        DMA1_Stream0_IRQn,
        SPI3_IRQn,
        DMA_FLAG_TCIF7,
        DMA_FLAG_TCIF0,
        std::function<void()>{},
        std::function<void()>{},
    }
};;

#define    DMA2_STREAM5_TC_Handler   SPIDirver::SPI_Callback[0].DMA_TC_HANDLER
#define    DMA2_STREAM0_RC_Handler   SPIDirver::SPI_Callback[0].DMA_RC_HANDLER

#define    DMA1_STREAM4_TC_Handler   SPIDirver::SPI_Callback[1].DMA_TC_HANDLER
#define    DMA1_STREAM3_RC_Handler   SPIDirver::SPI_Callback[1].DMA_RC_HANDLER

void SPIDirver::init(SPI_INIT_INFO &info) {
  m_info = info;

  m_info.Num--;

  RCC_AHB1PeriphClockCmd(1u << m_info.GpioSCK, ENABLE);
  uint8_t gpio_af;
  switch (m_info.Num) {
  case 0:RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    gpio_af = GPIO_AF_SPI1;
    break;
  case 1:RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    gpio_af = GPIO_AF_SPI2;
    break;
  case 2:RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    gpio_af = GPIO_AF_SPI3;
    break;
  }

  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;

  GPIO_PinAFConfig((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioSCK), m_info.GpioSCKPinNum, gpio_af);
  GPIO_PinAFConfig((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioMISO), m_info.GpioMISOPinNum, gpio_af);
  GPIO_PinAFConfig((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioMOSI), m_info.GpioMOSIPinNum, gpio_af);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;

  GPIO_InitStructure.GPIO_Pin = 1u << m_info.GpioSCKPinNum;
  GPIO_Init((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioSCK), &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = 1u << m_info.GpioMISOPinNum;
  GPIO_Init((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioMISO), &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = 1u << m_info.GpioMOSIPinNum;
  GPIO_Init((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioMOSI), &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
  GPIO_InitStructure.GPIO_Pin = 1u << m_info.GpioCSPinNum;
  GPIO_Init((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), &GPIO_InitStructure);

  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //Todo complete the baudrate config
  SPI_Init(SPI_Callback[m_info.Num].spi, &SPI_InitStructure);
  SPI_Cmd(SPI_Callback[m_info.Num].spi, DISABLE);
  SPI_SSOutputCmd(SPI_Callback[m_info.Num].spi, ENABLE);
  SPI_Cmd(SPI_Callback[m_info.Num].spi, ENABLE);
  SPI_NSSInternalSoftwareConfig(SPI_Callback[m_info.Num].spi, SPI_NSSInternalSoft_Set); //!!!!
  //  SPI_Init(spi, &SPI_InitStructure);
  //  SPI_Cmd(spi, DISABLE);
  //  SPI_NSSInternalSoftwareConfig(spi, SPI_NSSInternalSoft_Set);

  SPI_I2S_ITConfig(SPI_Callback[m_info.Num].spi, SPI_I2S_IT_ERR, ENABLE);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = SPI_Callback[m_info.Num].spi_irq;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
  NVIC_Init(&NVIC_InitStructure);
  SPI_Cmd(SPI_Callback[m_info.Num].spi, ENABLE);

  //	SPI_Cmd(spi, DISABLE);
  //	SPI_SSOutputCmd(spi, ENABLE);

  SPI_Callback[m_info.Num].spi->CR2 |= 1u << 1u;//t
  SPI_Callback[m_info.Num].spi->CR2 |= 1u << 0u;//r


  DMA_InitTypeDef DMA_InitStruct;
  DMA_InitStruct.DMA_BufferSize = 1;
  DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) &SPI_Callback[m_info.Num].spi->DR;
  DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t) 0;
  DMA_InitStruct.DMA_Channel = SPI_Callback[m_info.Num].dma_channel;

  DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_Init(SPI_Callback[m_info.Num].dma_tx_stream, &DMA_InitStruct);
  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_Init(SPI_Callback[m_info.Num].dma_rx_stream, &DMA_InitStruct);

  NVIC_InitStructure.NVIC_IRQChannel = SPI_Callback[m_info.Num].dma_tx_irq;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = SPI_Callback[m_info.Num].dma_rx_irq;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_Init(&NVIC_InitStructure);

  SPI_Callback[m_info.Num].spi->CR2 |= 1u << 1u;
  SPI_Callback[m_info.Num].spi->CR2 |= 1u << 0u;

  DMA_ClearFlag(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc);
  DMA_ClearFlag(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc);

  //DMA_ITConfig(SPI_Callback[m_info.Num].dma_tx_stream, DMA_IT_TC, ENABLE);
  DMA_ITConfig(SPI_Callback[m_info.Num].dma_tx_stream, DMA_IT_TE, ENABLE);
  //DMA_ITConfig(SPI_Callback[m_info.Num].dma_rx_stream, DMA_IT_TC, ENABLE);
  DMA_ITConfig(SPI_Callback[m_info.Num].dma_rx_stream, DMA_IT_TE, ENABLE);

  SPI_Cmd(SPI_Callback[m_info.Num].spi, ENABLE);
  //   GPIO_ResetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  //
  //  SPI_Cmd(spi, ENABLE);
  //  spi_RX_DMA_STREAM->CR |= DMA_SxCR_EN;
  //  spi_TX_DMA_STREAM->CR |= DMA_SxCR_EN;
  //
  //  while (!DMA_GetFlagStatus(spi_RX_DMA_STREAM, spi_RX_DMA_FLAG_TCIF));
  //  while (!DMA_GetFlagStatus(spi_TX_DMA_STREAM, spi_TX_DMA_FLAG_TCIF));
  //   GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
}
void SPIDirver::SPI_DMA_WriteReg(uint8_t reg, uint8_t value) {

  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc));
  while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc));
  DMA_ClearFlag(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc);
  DMA_ClearFlag(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc);

  GPIO_ResetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);

  SPI_Callback[m_info.Num].dma_rx_stream->CR &= ~(uint32_t) DMA_SxCR_EN;
  SPI_Callback[m_info.Num].dma_tx_stream->CR &= ~(uint32_t) DMA_SxCR_EN;

  SPI_Tx_DMA_Buffer[0] = reg & 0x7fu;
  SPI_Tx_DMA_Buffer[1] = value;
  SPI_Callback[m_info.Num].dma_tx_stream->NDTR = 2;
  SPI_Callback[m_info.Num].dma_rx_stream->NDTR = 2;

  SPI_Callback[m_info.Num].dma_rx_stream->M0AR = (uint32_t) &SPI_Tx_DMA_Buffer;
  SPI_Callback[m_info.Num].dma_tx_stream->M0AR = (uint32_t) &SPI_Tx_DMA_Buffer;
  SPI_Callback[m_info.Num].dma_rx_stream->CR |= DMA_SxCR_EN;
  SPI_Callback[m_info.Num].dma_tx_stream->CR |= DMA_SxCR_EN;
  while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc));
  while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc));
  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
}
uint8_t SPIDirver::SPI_RW(uint8_t data) const {
  while (SPI_I2S_GetFlagStatus(SPI_Callback[m_info.Num].spi, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI_Callback[m_info.Num].spi, data);
  while (SPI_I2S_GetFlagStatus(SPI_Callback[m_info.Num].spi, SPI_I2S_FLAG_RXNE) == RESET);
  return (uint8_t) SPI_I2S_ReceiveData(SPI_Callback[m_info.Num].spi);
}
uint8_t SPIDirver::SPI_ReadReg(uint8_t reg) {
  uint8_t value;
  GPIO_ResetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);//reset CSN
  SPI_RW(reg | 0x80u);
  value = SPI_RW(0x00);
  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);//set CSN
  return value;
}
void SPIDirver::SPI_WriteReg(uint8_t reg, uint8_t value) {
  GPIO_ResetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  SPI_RW(reg & 0x7fu);
  value = SPI_RW(value);
  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);//set CSN
}
void SPIDirver::SPI_ReadMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count) {
  GPIO_ResetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  SPI_RW(reg | 0xc0u);
  while (count--) {
    *pBuf++ = SPI_RW(0x00);
  }
  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
}
void SPIDirver::SPI_WriteMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count) {
  GPIO_ResetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  SPI_RW((reg & 0x7fu) | 0x40u);
  while (count--) {
    SPI_RW(*pBuf++);
  }
  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
}
uint8_t SPIDirver::SPI_DMA_ReadReg(uint8_t reg) {
  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc));
  while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc));
  DMA_ClearFlag(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc);
  DMA_ClearFlag(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc);
  SPI_Tx_DMA_Buffer[0] = reg | 0x80u;
  SPI_Tx_DMA_Buffer[1] = 0xff;
  GPIO_ResetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);

  SPI_Callback[m_info.Num].dma_rx_stream->CR &= ~(uint32_t) DMA_SxCR_EN;
  SPI_Callback[m_info.Num].dma_tx_stream->CR &= ~(uint32_t) DMA_SxCR_EN;

  SPI_Callback[m_info.Num].dma_tx_stream->NDTR = 2;
  SPI_Callback[m_info.Num].dma_rx_stream->NDTR = 2;
  SPI_Callback[m_info.Num].dma_rx_stream->M0AR = (uint32_t) &SPI_Tx_DMA_Buffer;
  SPI_Callback[m_info.Num].dma_tx_stream->M0AR = (uint32_t) &SPI_Tx_DMA_Buffer;
  SPI_Callback[m_info.Num].dma_rx_stream->CR |= DMA_SxCR_EN;
  SPI_Callback[m_info.Num].dma_tx_stream->CR |= DMA_SxCR_EN;

  while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc));
  while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc));
  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  return SPI_Tx_DMA_Buffer[1];
}

void SPIDirver::SPI_DMA_ReadMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count) {
  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc));
  while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc));
  DMA_ClearFlag(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc);
  DMA_ClearFlag(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc);
  //	memset(SPI1_Tx_DMA_Buffer,0,count+1);
  SPI_Tx_DMA_Buffer[0] = reg | 0xc0u;
  GPIO_ResetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);

  SPI_Callback[m_info.Num].dma_rx_stream->CR &= ~(uint32_t) DMA_SxCR_EN;
  SPI_Callback[m_info.Num].dma_tx_stream->CR &= ~(uint32_t) DMA_SxCR_EN;

  SPI_Callback[m_info.Num].dma_tx_stream->NDTR = count + 1;
  SPI_Callback[m_info.Num].dma_rx_stream->NDTR = count + 1;
  SPI_Callback[m_info.Num].dma_rx_stream->M0AR = (uint32_t) &SPI_Tx_DMA_Buffer;
  SPI_Callback[m_info.Num].dma_tx_stream->M0AR = (uint32_t) &SPI_Tx_DMA_Buffer;
  SPI_Callback[m_info.Num].dma_rx_stream->CR |= DMA_SxCR_EN;
  SPI_Callback[m_info.Num].dma_tx_stream->CR |= DMA_SxCR_EN;

  while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc));
  while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc));

  memcpy(pBuf, &SPI_Tx_DMA_Buffer[1], count);
  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
}
void SPIDirver::SPI_DMA_WriteMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count) {
  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc));
  while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc));
  DMA_ClearFlag(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc);
  DMA_ClearFlag(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc);

  memcpy(&SPI_Tx_DMA_Buffer[1], pBuf, count);
  SPI_Tx_DMA_Buffer[0] = (reg & 0x7fu) | 0x40u;
  GPIO_ResetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  SPI_Callback[m_info.Num].dma_rx_stream->CR &= ~(uint32_t) DMA_SxCR_EN;
  SPI_Callback[m_info.Num].dma_tx_stream->CR &= ~(uint32_t) DMA_SxCR_EN;

  SPI_Callback[m_info.Num].dma_tx_stream->NDTR = count + 1;
  SPI_Callback[m_info.Num].dma_rx_stream->NDTR = count + 1;
  SPI_Callback[m_info.Num].dma_rx_stream->M0AR = (uint32_t) &SPI_Tx_DMA_Buffer;
  SPI_Callback[m_info.Num].dma_tx_stream->M0AR = (uint32_t) &SPI_Tx_DMA_Buffer;

  SPI_Callback[m_info.Num].dma_rx_stream->CR |= DMA_SxCR_EN;
  SPI_Callback[m_info.Num].dma_tx_stream->CR |= DMA_SxCR_EN;

  while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc));
  while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc));
  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);

}
uint8_t SPIDirver::dma_read_byte() {
  uint8_t data;
  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  if (!is_first_time_dma_read_write) {
    while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc));
    while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc));
  } else {
    is_first_time_dma_read_write = 0;
  }

  DMA_ClearFlag(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc);
  DMA_ClearFlag(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc);

  GPIO_ResetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  SPI_Callback[m_info.Num].dma_rx_stream->CR &= ~(uint32_t) DMA_SxCR_EN;
  SPI_Callback[m_info.Num].dma_tx_stream->CR &= ~(uint32_t) DMA_SxCR_EN;

  SPI_Callback[m_info.Num].dma_tx_stream->NDTR = 1;
  SPI_Callback[m_info.Num].dma_rx_stream->NDTR = 1;
  SPI_Callback[m_info.Num].dma_rx_stream->M0AR = (uint32_t) &data;
  SPI_Callback[m_info.Num].dma_tx_stream->M0AR = (uint32_t) &data;

  SPI_Callback[m_info.Num].dma_rx_stream->CR |= DMA_SxCR_EN;
  SPI_Callback[m_info.Num].dma_tx_stream->CR |= DMA_SxCR_EN;

  while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc));
  while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc));
  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  return data;
}
void SPIDirver::dma_write_byte(uint8_t value) {
  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  if (!is_first_time_dma_read_write) {
    while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc));
    while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc));
  } else {
    is_first_time_dma_read_write = 0;
  }

  DMA_ClearFlag(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc);
  DMA_ClearFlag(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc);

  GPIO_ResetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  SPI_Callback[m_info.Num].dma_tx_stream->CR &= ~(uint32_t) DMA_SxCR_EN;
  SPI_Callback[m_info.Num].dma_rx_stream->CR &= ~(uint32_t) DMA_SxCR_EN;

  SPI_Callback[m_info.Num].dma_tx_stream->NDTR = 1;
  SPI_Callback[m_info.Num].dma_rx_stream->NDTR = 1;
  SPI_Callback[m_info.Num].dma_tx_stream->M0AR = (uint32_t) &value;
  SPI_Callback[m_info.Num].dma_rx_stream->M0AR = (uint32_t) &value;

  SPI_Callback[m_info.Num].dma_tx_stream->CR |= DMA_SxCR_EN;
  SPI_Callback[m_info.Num].dma_rx_stream->CR |= DMA_SxCR_EN;

  while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc));
  while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc));
  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
}
void SPIDirver::dma_spi_read_irq_handler() {

  DMA_ITConfig(SPI_Callback[m_info.Num].dma_rx_stream, DMA_IT_TC, DISABLE);
  SPI_Callback[m_info.Num].dma_tx_stream->CR &= ~(uint32_t) DMA_SxCR_EN;
  SPI_Callback[m_info.Num].dma_rx_stream->CR &= ~(uint32_t) DMA_SxCR_EN;
  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);

  spi_read_callback_fun(read_len);
}
void SPIDirver::dma_spi_write_irq_handler() {

  DMA_ITConfig(SPI_Callback[m_info.Num].dma_tx_stream, DMA_IT_TC, DISABLE);
  SPI_Callback[m_info.Num].dma_tx_stream->CR &= ~(uint32_t) DMA_SxCR_EN;
  SPI_Callback[m_info.Num].dma_rx_stream->CR &= ~(uint32_t) DMA_SxCR_EN;
  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);

  spi_write_callback_fun(read_len);
}
void SPIDirver::read_bytes(const uint8_t *buffer, uint16_t len) {
  GPIO_ResetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  auto *pbuffer = const_cast<uint8_t *>(buffer);
  while (len--) {
    *pbuffer++ = SPI_RW(0x00);
  }
  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
}
void SPIDirver::write_bytes(const uint8_t *buffer, uint16_t len) {
  GPIO_ResetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  auto *pbuffer = const_cast<uint8_t *>(buffer);
  while (len--) {
    *pbuffer = SPI_RW(*pbuffer);
    pbuffer++;
  }
  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
}
void SPIDirver::dma_write_bytes(const uint8_t *buffer, uint16_t len) {
  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  if (!is_first_time_dma_read_write) {
    while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc));
    while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc));
  } else {
    is_first_time_dma_read_write = 0;
  }

  DMA_ClearFlag(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc);
  DMA_ClearFlag(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc);

  GPIO_ResetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  SPI_Callback[m_info.Num].dma_tx_stream->CR &= ~(uint32_t) DMA_SxCR_EN;
  SPI_Callback[m_info.Num].dma_rx_stream->CR &= ~(uint32_t) DMA_SxCR_EN;

  SPI_Callback[m_info.Num].dma_tx_stream->NDTR = len;
  SPI_Callback[m_info.Num].dma_rx_stream->NDTR = len;
  SPI_Callback[m_info.Num].dma_tx_stream->M0AR = (uint32_t) buffer;
  SPI_Callback[m_info.Num].dma_rx_stream->M0AR = (uint32_t) buffer;

  SPI_Callback[m_info.Num].dma_tx_stream->CR |= DMA_SxCR_EN;
  SPI_Callback[m_info.Num].dma_rx_stream->CR |= DMA_SxCR_EN;

  while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc));
  while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc));
  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
}
void SPIDirver::dma_read_bytes(const uint8_t *buffer, uint16_t len) {
  //   GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  if (!is_first_time_dma_read_write) {
    while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc));
    while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc));
  } else {
    is_first_time_dma_read_write = 0;
  }

  DMA_ClearFlag(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc);
  DMA_ClearFlag(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc);

  GPIO_ResetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  SPI_Callback[m_info.Num].dma_tx_stream->CR &= ~(uint32_t) DMA_SxCR_EN;
  SPI_Callback[m_info.Num].dma_rx_stream->CR &= ~(uint32_t) DMA_SxCR_EN;

  SPI_Callback[m_info.Num].dma_tx_stream->NDTR = len;
  SPI_Callback[m_info.Num].dma_rx_stream->NDTR = len;
  SPI_Callback[m_info.Num].dma_tx_stream->M0AR = (uint32_t) buffer;
  SPI_Callback[m_info.Num].dma_rx_stream->M0AR = (uint32_t) buffer;

  SPI_Callback[m_info.Num].dma_tx_stream->CR |= DMA_SxCR_EN;
  SPI_Callback[m_info.Num].dma_rx_stream->CR |= DMA_SxCR_EN;

  while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc));
  while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc));
  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
}

void SPIDirver::dma_async_read(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t bytes_transferred)> callback_fun) {

  spi_read_callback_fun = std::move(callback_fun);
  //  spi_read_callback_fun = callback_fun;

  read_len = len;

  SPI_Callback[m_info.Num].DMA_RC_HANDLER = [this]() {
    dma_spi_read_irq_handler();
  };

  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  if (!is_first_time_dma_read_write) {
    while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc));
    while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc));
  } else {
    is_first_time_dma_read_write = 0;
  }

  DMA_ClearFlag(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc);
  DMA_ClearFlag(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc);
  DMA_ITConfig(SPI_Callback[m_info.Num].dma_rx_stream, DMA_IT_TC, ENABLE);

  GPIO_ResetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  SPI_Callback[m_info.Num].dma_tx_stream->CR &= ~(uint32_t) DMA_SxCR_EN;
  SPI_Callback[m_info.Num].dma_rx_stream->CR &= ~(uint32_t) DMA_SxCR_EN;

  SPI_Callback[m_info.Num].dma_tx_stream->NDTR = len;
  SPI_Callback[m_info.Num].dma_rx_stream->NDTR = len;
  SPI_Callback[m_info.Num].dma_tx_stream->M0AR = (uint32_t) buffer;
  SPI_Callback[m_info.Num].dma_rx_stream->M0AR = (uint32_t) buffer;

  SPI_Callback[m_info.Num].dma_tx_stream->CR |= DMA_SxCR_EN;
  SPI_Callback[m_info.Num].dma_rx_stream->CR |= DMA_SxCR_EN;

}

void SPIDirver::dma_async_write(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t bytes_transferred)> callback_fun) {

  spi_write_callback_fun = std::move(callback_fun);

  write_len = len;

  SPI_Callback[m_info.Num].DMA_TC_HANDLER = [this]() {
    dma_spi_write_irq_handler();
  };

  GPIO_SetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  if (!is_first_time_dma_read_write) {
    while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc));
    while (!DMA_GetFlagStatus(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc));
  } else {
    is_first_time_dma_read_write = 0;
  }

  DMA_ClearFlag(SPI_Callback[m_info.Num].dma_tx_stream, SPI_Callback[m_info.Num].dma_tx_flag_tc);
  DMA_ClearFlag(SPI_Callback[m_info.Num].dma_rx_stream, SPI_Callback[m_info.Num].dma_rx_flag_tc);
  DMA_ITConfig(SPI_Callback[m_info.Num].dma_tx_stream, DMA_IT_TC, ENABLE);

  GPIO_ResetBits((GPIO_TypeDef *) (AHB1PERIPH_BASE + 0x0400 * m_info.GpioCS), 1u << m_info.GpioCSPinNum);
  SPI_Callback[m_info.Num].dma_tx_stream->CR &= ~(uint32_t) DMA_SxCR_EN;
  SPI_Callback[m_info.Num].dma_rx_stream->CR &= ~(uint32_t) DMA_SxCR_EN;

  SPI_Callback[m_info.Num].dma_tx_stream->NDTR = len;
  SPI_Callback[m_info.Num].dma_rx_stream->NDTR = len;
  SPI_Callback[m_info.Num].dma_tx_stream->M0AR = (uint32_t) buffer;
  SPI_Callback[m_info.Num].dma_rx_stream->M0AR = (uint32_t) buffer;

  SPI_Callback[m_info.Num].dma_tx_stream->CR |= DMA_SxCR_EN;
  SPI_Callback[m_info.Num].dma_rx_stream->CR |= DMA_SxCR_EN;

}
uint32_t SPI1_TX_DMA_Counter[5];
uint32_t SPI1_RX_DMA_Counter[5];

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

extern "C" void DMA2_Stream5_IRQHandler() {
  if (DMA_GetITStatus(DMA2_Stream5, SPI1_TX_DMA_IT_TCIF)) {
    //DMA_ClearITPendingBit(SPI_Callback[m_info.Num].dma_tx_stream, SPI1_TX_DMA_IT_HTIF);
    SPI1_TX_DMA_Counter[0]++;
    DMA2_STREAM5_TC_Handler();
  }
  if (DMA_GetITStatus(DMA2_Stream5, SPI1_TX_DMA_IT_HTIF)) {
    DMA_ClearITPendingBit(DMA2_Stream5, SPI1_TX_DMA_IT_HTIF);
    SPI1_TX_DMA_Counter[1]++;

  }
  if (DMA_GetITStatus(DMA2_Stream5, SPI1_TX_DMA_IT_TEIF)) {
    DMA_ClearITPendingBit(DMA2_Stream5, SPI1_TX_DMA_IT_TEIF);

    SPI1_TX_DMA_Counter[2]++;
  }
  if (DMA_GetITStatus(DMA2_Stream5, SPI1_TX_DMA_IT_DMEIF)) {
    DMA_ClearITPendingBit(DMA2_Stream5, SPI1_TX_DMA_IT_DMEIF);
    SPI1_TX_DMA_Counter[3]++;

  }
  if (DMA_GetITStatus(DMA2_Stream5, SPI1_TX_DMA_IT_FEIF)) {
    DMA_ClearITPendingBit(DMA2_Stream5, SPI1_TX_DMA_IT_FEIF);
    SPI1_TX_DMA_Counter[4]++;

  }

}


extern "C" void DMA2_Stream0_IRQHandler() {
  if (DMA_GetITStatus(DMA2_Stream0, SPI1_RX_DMA_IT_TCIF)) {
    //DMA_ClearITPendingBit(SPI_Callback[m_info.Num].dma_rx_stream, DMA_IT_TCIF3);

    DMA2_STREAM0_RC_Handler();
  }
  if (DMA_GetITStatus(DMA2_Stream0, SPI1_RX_DMA_IT_HTIF)) {
    DMA_ClearITPendingBit(DMA2_Stream0, SPI1_RX_DMA_IT_HTIF);
    SPI1_RX_DMA_Counter[1]++;

  }
  if (DMA_GetITStatus(DMA2_Stream0, SPI1_RX_DMA_IT_TEIF)) {
    DMA_ClearITPendingBit(DMA2_Stream0, SPI1_RX_DMA_IT_TEIF);

    SPI1_RX_DMA_Counter[2]++;
  }
  if (DMA_GetITStatus(DMA2_Stream0, SPI1_RX_DMA_IT_DMEIF)) {
    DMA_ClearITPendingBit(DMA2_Stream0, SPI1_RX_DMA_IT_DMEIF);
    SPI1_RX_DMA_Counter[3]++;

  }
  if (DMA_GetITStatus(DMA2_Stream0, SPI1_RX_DMA_IT_FEIF)) {
    DMA_ClearITPendingBit(DMA2_Stream0, SPI1_RX_DMA_IT_FEIF);
    SPI1_RX_DMA_Counter[4]++;

  }

}

uint32_t SPI2_TX_DMA_Counter[5];
uint32_t SPI2_RX_DMA_Counter[5];
#define SPI2_RX_DMA_IT_FEIF DMA_IT_FEIF3
#define SPI2_RX_DMA_IT_DMEIF DMA_IT_DMEIF3
#define SPI2_RX_DMA_IT_TEIF DMA_IT_TEIF3
#define SPI2_RX_DMA_IT_HTIF DMA_IT_HTIF3
#define SPI2_RX_DMA_IT_TCIF DMA_IT_TCIF3

#define SPI2_TX_DMA_IT_FEIF DMA_IT_FEIF4
#define SPI2_TX_DMA_IT_DMEIF DMA_IT_DMEIF4
#define SPI2_TX_DMA_IT_TEIF DMA_IT_TEIF4
#define SPI2_TX_DMA_IT_HTIF DMA_IT_HTIF4
#define SPI2_TX_DMA_IT_TCIF DMA_IT_TCIF4

extern "C" void DMA1_Stream4_IRQHandler() {
  if (DMA_GetITStatus(DMA1_Stream4, SPI2_TX_DMA_IT_TCIF)) {
    //DMA_ClearITPendingBit(SPI_Callback[m_info.Num].dma_tx_stream, SPI1_TX_DMA_IT_HTIF);
    SPI2_TX_DMA_Counter[0]++;
    DMA1_STREAM4_TC_Handler();
  }
  if (DMA_GetITStatus(DMA1_Stream4, SPI2_TX_DMA_IT_HTIF)) {
    DMA_ClearITPendingBit(DMA1_Stream4, SPI2_TX_DMA_IT_HTIF);
    SPI2_TX_DMA_Counter[1]++;

  }
  if (DMA_GetITStatus(DMA1_Stream4, SPI2_TX_DMA_IT_TEIF)) {
    DMA_ClearITPendingBit(DMA1_Stream4, SPI2_TX_DMA_IT_TEIF);

    SPI2_TX_DMA_Counter[2]++;
  }
  if (DMA_GetITStatus(DMA1_Stream4, SPI2_TX_DMA_IT_DMEIF)) {
    DMA_ClearITPendingBit(DMA1_Stream4, SPI2_TX_DMA_IT_DMEIF);
    SPI2_TX_DMA_Counter[3]++;

  }
  if (DMA_GetITStatus(DMA1_Stream4, SPI2_TX_DMA_IT_FEIF)) {
    DMA_ClearITPendingBit(DMA1_Stream4, SPI2_TX_DMA_IT_FEIF);
    SPI2_TX_DMA_Counter[4]++;

  }

}


extern "C" void DMA1_Stream3_IRQHandler() {
  if (DMA_GetITStatus(DMA1_Stream3, SPI2_RX_DMA_IT_TCIF)) {
    //DMA_ClearITPendingBit(SPI_Callback[m_info.Num].dma_rx_stream, DMA_IT_TCIF3);

    DMA1_STREAM3_RC_Handler();
  }
  if (DMA_GetITStatus(DMA1_Stream3, SPI2_RX_DMA_IT_HTIF)) {
    DMA_ClearITPendingBit(DMA1_Stream3, SPI2_RX_DMA_IT_HTIF);
    SPI2_RX_DMA_Counter[1]++;

  }
  if (DMA_GetITStatus(DMA1_Stream3, SPI2_RX_DMA_IT_TEIF)) {
    DMA_ClearITPendingBit(DMA1_Stream3, SPI2_RX_DMA_IT_TEIF);

    SPI2_RX_DMA_Counter[2]++;
  }
  if (DMA_GetITStatus(DMA1_Stream3, SPI2_RX_DMA_IT_DMEIF)) {
    DMA_ClearITPendingBit(DMA1_Stream3, SPI2_RX_DMA_IT_DMEIF);
    SPI2_RX_DMA_Counter[3]++;

  }
  if (DMA_GetITStatus(DMA1_Stream3, SPI2_RX_DMA_IT_FEIF)) {
    DMA_ClearITPendingBit(DMA1_Stream3, SPI1_RX_DMA_IT_FEIF);
    SPI2_RX_DMA_Counter[4]++;

  }

}



#include "SPI1.h"

void SPI1Dirver::init() {
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;
  SPI1_GPIO_FUNCTION(SPI1_GPIO_CLK, ENABLE);
  SPI1_CLK_FUNCTION(SPI1_CLK, ENABLE);
  SPI1_DMA_FUNCTION(SPI1_DMA_CLK, ENABLE);

  GPIO_PinAFConfig(SPI1_SCK_GPIO, SPI1_SCK_AF_PIN_SOURCE, GPIO_AF_SPI1);
  GPIO_PinAFConfig(SPI1_MISO_GPIO, SPI1_MISO_AF_PIN_SOURCE, GPIO_AF_SPI1);
  GPIO_PinAFConfig(SPI1_MOSI_GPIO, SPI1_MOSI_AF_PIN_SOURCE, GPIO_AF_SPI1);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
  
  GPIO_InitStructure.GPIO_Pin = SPI1_SCK_PIN;
  GPIO_Init(SPI1_SCK_GPIO, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin =  SPI1_MISO_PIN;
  GPIO_Init(SPI1_MISO_GPIO, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = SPI1_MOSI_PIN;
  GPIO_Init(SPI1_MOSI_GPIO, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
  GPIO_InitStructure.GPIO_Pin = SPI1_CS_PIN;
  GPIO_Init(SPI1_CS_GPIO, &GPIO_InitStructure);

  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_Init(SPI1, &SPI_InitStructure);
  SPI_Cmd(SPI1, DISABLE);
  SPI_SSOutputCmd(SPI1, ENABLE);
  SPI_Cmd(SPI1, ENABLE);
  SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set); //!!!!
//  SPI_Init(SPI1, &SPI_InitStructure);
//  SPI_Cmd(SPI1, DISABLE);
//  SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);




  SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_ERR, ENABLE);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
  NVIC_Init(&NVIC_InitStructure);
  SPI_Cmd(SPI1, ENABLE);

  //	SPI_Cmd(SPI1, DISABLE);
  //	SPI_SSOutputCmd(SPI1, ENABLE);

  SPI1->CR2 |= 1u << 1u;//t
  SPI1->CR2 |= 1u << 0u;//r


  DMA_InitTypeDef DMA_InitStruct;
  DMA_InitStruct.DMA_BufferSize = 1;
  DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) &SPI1->DR;
  DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t) 0;

  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStruct.DMA_Channel = SPI1_RX_DMA_CHANNL;
  DMA_Init(SPI1_RX_DMA_STREAM, &DMA_InitStruct);

  DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStruct.DMA_Channel = SPI1_TX_DMA_CHANNL;
  DMA_Init(SPI1_TX_DMA_STREAM, &DMA_InitStruct);

  NVIC_InitStructure.NVIC_IRQChannel = SPI1_RX_DMA_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = SPI1_TX_DMA_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_Init(&NVIC_InitStructure);

  SPI1->CR2 |= 1u << 1u;
  SPI1->CR2 |= 1u << 0u;

  DMA_ClearFlag(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_FEIF | SPI1_TX_DMA_FLAG_DMEIF | SPI1_TX_DMA_FLAG_TEIF | SPI1_TX_DMA_FLAG_HTIF | SPI1_TX_DMA_FLAG_TCIF);
  DMA_ClearFlag(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_FEIF | SPI1_RX_DMA_FLAG_DMEIF | SPI1_RX_DMA_FLAG_TEIF | SPI1_RX_DMA_FLAG_HTIF | SPI1_RX_DMA_FLAG_TCIF);

  //DMA_ITConfig(SPI1_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_ITConfig(SPI1_RX_DMA_STREAM, DMA_IT_TE, ENABLE);
  //DMA_ITConfig(SPI1_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_ITConfig(SPI1_TX_DMA_STREAM, DMA_IT_TE, ENABLE);

  SPI_Cmd(SPI1, ENABLE);
//  CSN_LOW
//
//  SPI_Cmd(SPI1, ENABLE);
//  SPI1_RX_DMA_STREAM->CR |= DMA_SxCR_EN;
//  SPI1_TX_DMA_STREAM->CR |= DMA_SxCR_EN;
//
//  while (!DMA_GetFlagStatus(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF));
//  while (!DMA_GetFlagStatus(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF));
//  CSN_HIGHT
}
void SPI1Dirver::SPI1_DMA_WriteReg(uint8_t reg, uint8_t value) {

  CSN_HIGHT
  while (!DMA_GetFlagStatus(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF));
  while (!DMA_GetFlagStatus(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF));
  DMA_ClearFlag(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF);
  DMA_ClearFlag(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF);

  CSN_LOW

  SPI1_RX_DMA_STREAM->CR &= ~(uint32_t) DMA_SxCR_EN;
  SPI1_TX_DMA_STREAM->CR &= ~(uint32_t) DMA_SxCR_EN;

  SPI1_Tx_DMA_Buffer[0] = reg & 0x7fu;
  SPI1_Tx_DMA_Buffer[1] = value;
  SPI1_TX_DMA_STREAM->NDTR = 2;
  SPI1_RX_DMA_STREAM->NDTR = 2;

  SPI1_RX_DMA_STREAM->M0AR = (uint32_t) &SPI1_Tx_DMA_Buffer;
  SPI1_TX_DMA_STREAM->M0AR = (uint32_t) &SPI1_Tx_DMA_Buffer;
  SPI1_RX_DMA_STREAM->CR |= DMA_SxCR_EN;
  SPI1_TX_DMA_STREAM->CR |= DMA_SxCR_EN;
  while (!DMA_GetFlagStatus(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF));
  while (!DMA_GetFlagStatus(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF));
  CSN_HIGHT
}
uint8_t SPI1Dirver::SPI1_RW(uint8_t data) {
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI1, data);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  return (uint8_t) SPI_I2S_ReceiveData(SPI1);
}
uint8_t SPI1Dirver::SPI1_ReadReg(uint8_t reg) {
  uint8_t value;
  CSN_LOW//reset CSN
  SPI1_RW(reg | 0x80u);
  value = SPI1_RW(0x00);
  CSN_HIGHT//set CSN
  return value;
}
void SPI1Dirver::SPI1_WriteReg(uint8_t reg, uint8_t value) {
  CSN_LOW
  SPI1_RW(reg & 0x7fu);
  value = SPI1_RW(value);
  CSN_HIGHT//set CSN
}
void SPI1Dirver::SPI1_ReadMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count) {
  CSN_LOW
  SPI1_RW(reg | 0xc0u);
  while (count--) {
    *pBuf++ = SPI1_RW(0x00);
  }
  CSN_HIGHT
}
void SPI1Dirver::SPI1_WriteMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count) {
  CSN_LOW
  SPI1_RW((reg & 0x7fu) | 0x40u);
  while (count--) {
    SPI1_RW(*pBuf++);
  }
  CSN_HIGHT
}
uint8_t SPI1Dirver::SPI1_DMA_ReadReg(uint8_t reg) {
  CSN_HIGHT
  while (!DMA_GetFlagStatus(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF));
  while (!DMA_GetFlagStatus(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF));
  DMA_ClearFlag(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF);
  DMA_ClearFlag(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF);
  SPI1_Tx_DMA_Buffer[0] = reg | 0x80u;
  SPI1_Tx_DMA_Buffer[1] = 0xff;
  CSN_LOW

  SPI1_RX_DMA_STREAM->CR &= ~(uint32_t) DMA_SxCR_EN;
  SPI1_TX_DMA_STREAM->CR &= ~(uint32_t) DMA_SxCR_EN;

  SPI1_TX_DMA_STREAM->NDTR = 2;
  SPI1_RX_DMA_STREAM->NDTR = 2;
  SPI1_RX_DMA_STREAM->M0AR = (uint32_t) &SPI1_Tx_DMA_Buffer;
  SPI1_TX_DMA_STREAM->M0AR = (uint32_t) &SPI1_Tx_DMA_Buffer;
  SPI1_RX_DMA_STREAM->CR |= DMA_SxCR_EN;
  SPI1_TX_DMA_STREAM->CR |= DMA_SxCR_EN;

  while (!DMA_GetFlagStatus(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF));
  while (!DMA_GetFlagStatus(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF));
  CSN_HIGHT
  return SPI1_Tx_DMA_Buffer[1];
}


void SPI1Dirver::SPI1_DMA_ReadMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count) {
  CSN_HIGHT
  while (!DMA_GetFlagStatus(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF));
  while (!DMA_GetFlagStatus(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF));
  DMA_ClearFlag(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF);
  DMA_ClearFlag(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF);
  //	memset(SPI1_Tx_DMA_Buffer,0,count+1);
  SPI1_Tx_DMA_Buffer[0] = reg | 0xc0u;
  CSN_LOW

  SPI1_RX_DMA_STREAM->CR &= ~(uint32_t) DMA_SxCR_EN;
  SPI1_TX_DMA_STREAM->CR &= ~(uint32_t) DMA_SxCR_EN;

  SPI1_TX_DMA_STREAM->NDTR = count + 1;
  SPI1_RX_DMA_STREAM->NDTR = count + 1;
  SPI1_RX_DMA_STREAM->M0AR = (uint32_t) &SPI1_Tx_DMA_Buffer;
  SPI1_TX_DMA_STREAM->M0AR = (uint32_t) &SPI1_Tx_DMA_Buffer;
  SPI1_RX_DMA_STREAM->CR |= DMA_SxCR_EN;
  SPI1_TX_DMA_STREAM->CR |= DMA_SxCR_EN;

  while (!DMA_GetFlagStatus(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF));
  while (!DMA_GetFlagStatus(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF));

  memcpy(pBuf, &SPI1_Tx_DMA_Buffer[1], count);
  CSN_HIGHT
}
void SPI1Dirver::SPI1_DMA_WriteMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count) {
  CSN_HIGHT
  while (!DMA_GetFlagStatus(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF));
  while (!DMA_GetFlagStatus(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF));
  DMA_ClearFlag(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF);
  DMA_ClearFlag(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF);

  memcpy(&SPI1_Tx_DMA_Buffer[1], pBuf, count);
  SPI1_Tx_DMA_Buffer[0] = (reg & 0x7f) | 0x40;
  CSN_LOW
  SPI1_RX_DMA_STREAM->CR &= ~(uint32_t) DMA_SxCR_EN;
  SPI1_TX_DMA_STREAM->CR &= ~(uint32_t) DMA_SxCR_EN;

  SPI1_TX_DMA_STREAM->NDTR = count + 1;
  SPI1_RX_DMA_STREAM->NDTR = count + 1;
  SPI1_RX_DMA_STREAM->M0AR = (uint32_t) &SPI1_Tx_DMA_Buffer;
  SPI1_TX_DMA_STREAM->M0AR = (uint32_t) &SPI1_Tx_DMA_Buffer;

  SPI1_RX_DMA_STREAM->CR |= DMA_SxCR_EN;
  SPI1_TX_DMA_STREAM->CR |= DMA_SxCR_EN;

  while (!DMA_GetFlagStatus(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF));
  while (!DMA_GetFlagStatus(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF));
  CSN_HIGHT

}
uint8_t SPI1Dirver::dma_read_byte() {
  uint8_t data;
  CSN_HIGHT
  if (!is_first_time_dma_read) {
    while (!DMA_GetFlagStatus(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF));
    while (!DMA_GetFlagStatus(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF));
  } else {
    is_first_time_dma_read = 0;
  }

  DMA_ClearFlag(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF);
  DMA_ClearFlag(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF);

  CSN_LOW
  SPI1_RX_DMA_STREAM->CR &= ~(uint32_t) DMA_SxCR_EN;
  SPI1_TX_DMA_STREAM->CR &= ~(uint32_t) DMA_SxCR_EN;

  SPI1_TX_DMA_STREAM->NDTR = 1;
  SPI1_RX_DMA_STREAM->NDTR = 1;
  SPI1_RX_DMA_STREAM->M0AR = (uint32_t) &data;
  SPI1_TX_DMA_STREAM->M0AR = (uint32_t) &data;


  SPI1_RX_DMA_STREAM->CR |= DMA_SxCR_EN;
  SPI1_TX_DMA_STREAM->CR |= DMA_SxCR_EN;

  while (!DMA_GetFlagStatus(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF));
  while (!DMA_GetFlagStatus(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF));
  CSN_HIGHT
  return data;
}
void SPI1Dirver::dma_write_byte(uint8_t value) {
  CSN_HIGHT
  if (!is_first_time_dma_read) {
    while (!DMA_GetFlagStatus(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF));
    while (!DMA_GetFlagStatus(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF));
  } else {
    is_first_time_dma_read = 0;
  }

  DMA_ClearFlag(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF);
  DMA_ClearFlag(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF);

  CSN_LOW
  SPI1_RX_DMA_STREAM->CR &= ~(uint32_t) DMA_SxCR_EN;
  SPI1_TX_DMA_STREAM->CR &= ~(uint32_t) DMA_SxCR_EN;

  SPI1_TX_DMA_STREAM->NDTR = 1;
  SPI1_RX_DMA_STREAM->NDTR = 1;
  SPI1_RX_DMA_STREAM->M0AR = (uint32_t) &value;
  SPI1_TX_DMA_STREAM->M0AR = (uint32_t) &value;

  SPI1_RX_DMA_STREAM->CR |= DMA_SxCR_EN;
  SPI1_TX_DMA_STREAM->CR |= DMA_SxCR_EN;

  while (!DMA_GetFlagStatus(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF));
  while (!DMA_GetFlagStatus(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF));
  CSN_HIGHT
}
void SPI1Dirver::dma_spi_read_irq_handler(uint8_t remain_len) {
  spi_read_callback_fun(read_len - remain_len);
}
void SPI1Dirver::dma_spi_write_irq_handler(uint8_t remain_len) {
  spi_write_callback_fun(read_len - remain_len);
}
void SPI1Dirver::read_bytes(const uint8_t *buffer, uint16_t len) {
  CSN_LOW
  uint8_t *  pbuffer= const_cast<uint8_t *>(buffer);
  while (len--) {
    *pbuffer++ = SPI1_RW(0x00);
  }
  CSN_HIGHT
}
void SPI1Dirver::write_bytes(const uint8_t *buffer, uint16_t len) {
  CSN_LOW
  uint8_t*  pbuffer= const_cast<uint8_t *>(buffer);
  while (len--) {
    *pbuffer= SPI1_RW(*pbuffer);
    pbuffer++;
  }
  CSN_HIGHT
}
void SPI1Dirver::dma_write_bytes(const uint8_t *buffer, uint16_t len) {
  CSN_HIGHT
  if (!is_first_time_dma_read) {
    while (!DMA_GetFlagStatus(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF));
    while (!DMA_GetFlagStatus(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF));
  } else {
    is_first_time_dma_read = 0;
  }

  DMA_ClearFlag(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF);
  DMA_ClearFlag(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF);

  CSN_LOW
  SPI1_RX_DMA_STREAM->CR &= ~(uint32_t) DMA_SxCR_EN;
  SPI1_TX_DMA_STREAM->CR &= ~(uint32_t) DMA_SxCR_EN;

  SPI1_TX_DMA_STREAM->NDTR = len;
  SPI1_RX_DMA_STREAM->NDTR = len;
  SPI1_RX_DMA_STREAM->M0AR = (uint32_t) buffer;
  SPI1_TX_DMA_STREAM->M0AR = (uint32_t) buffer;

  SPI1_RX_DMA_STREAM->CR |= DMA_SxCR_EN;
  SPI1_TX_DMA_STREAM->CR |= DMA_SxCR_EN;

  while (!DMA_GetFlagStatus(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF));
  while (!DMA_GetFlagStatus(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF));
  CSN_HIGHT
}
void SPI1Dirver::dma_read_bytes(const uint8_t *buffer, uint16_t len) {
//  CSN_HIGHT
  if (!is_first_time_dma_read) {
    while (!DMA_GetFlagStatus(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF));
    while (!DMA_GetFlagStatus(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF));
  } else {
    is_first_time_dma_read = 0;
  }

  DMA_ClearFlag(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF);
  DMA_ClearFlag(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF);

  CSN_LOW
  SPI1_RX_DMA_STREAM->CR &= ~(uint32_t) DMA_SxCR_EN;
  SPI1_TX_DMA_STREAM->CR &= ~(uint32_t) DMA_SxCR_EN;

  SPI1_TX_DMA_STREAM->NDTR = len;
  SPI1_RX_DMA_STREAM->NDTR = len;
  SPI1_RX_DMA_STREAM->M0AR = (uint32_t) buffer;
  SPI1_TX_DMA_STREAM->M0AR = (uint32_t) buffer;

  SPI1_RX_DMA_STREAM->CR |= DMA_SxCR_EN;
  SPI1_TX_DMA_STREAM->CR |= DMA_SxCR_EN;

  while (!DMA_GetFlagStatus(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF));
  while (!DMA_GetFlagStatus(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF));
  CSN_HIGHT
}

std::function<void(uint8_t remain_len)> SPI1_RX_DMA_TC_Handler;
std::function<void(uint8_t remain_len)> SPI1_TX_DMA_TC_Handler;


void SPI1Dirver::dma_async_read(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t bytes_transferred)> callback_fun) {


  spi_read_callback_fun = std::move(callback_fun);
//  spi_read_callback_fun = callback_fun;

  read_len = len;

  SPI1_RX_DMA_TC_Handler =[this](uint8_t remain_len){
    dma_spi_read_irq_handler (remain_len);
  };

  CSN_HIGHT
  if (!is_first_time_dma_read) {
    while (!DMA_GetFlagStatus(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF));
    while (!DMA_GetFlagStatus(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF));
  } else {
    is_first_time_dma_read = 0;
  }

  DMA_ClearFlag(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF);
  DMA_ClearFlag(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF);
  DMA_ITConfig(SPI1_RX_DMA_STREAM, DMA_IT_TC, ENABLE);

  CSN_LOW
  SPI1_RX_DMA_STREAM->CR &= ~(uint32_t) DMA_SxCR_EN;
  SPI1_TX_DMA_STREAM->CR &= ~(uint32_t) DMA_SxCR_EN;

  SPI1_TX_DMA_STREAM->NDTR = len;
  SPI1_RX_DMA_STREAM->NDTR = len;
  SPI1_RX_DMA_STREAM->M0AR = (uint32_t) buffer;
  SPI1_TX_DMA_STREAM->M0AR = (uint32_t) buffer;

  SPI1_RX_DMA_STREAM->CR |= DMA_SxCR_EN;
  SPI1_TX_DMA_STREAM->CR |= DMA_SxCR_EN;

}

void SPI1Dirver::dma_async_write(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t bytes_transferred)> callback_fun) {

  spi_write_callback_fun = std::move(callback_fun);

  write_len = len;

  SPI1_TX_DMA_TC_Handler =[this](uint8_t remain_len){
    dma_spi_write_irq_handler (remain_len);
  };


  CSN_HIGHT
  if (!is_first_time_dma_read) {
    while (!DMA_GetFlagStatus(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF));
    while (!DMA_GetFlagStatus(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF));
  } else {
    is_first_time_dma_read = 0;
  }

  DMA_ClearFlag(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF);
  DMA_ClearFlag(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF);
  DMA_ITConfig(SPI1_TX_DMA_STREAM, DMA_IT_TC, ENABLE);

  CSN_LOW
  SPI1_RX_DMA_STREAM->CR &= ~(uint32_t) DMA_SxCR_EN;
  SPI1_TX_DMA_STREAM->CR &= ~(uint32_t) DMA_SxCR_EN;

  SPI1_TX_DMA_STREAM->NDTR = len;
  SPI1_RX_DMA_STREAM->NDTR = len;
  SPI1_RX_DMA_STREAM->M0AR = (uint32_t) buffer;
  SPI1_TX_DMA_STREAM->M0AR = (uint32_t) buffer;

  SPI1_RX_DMA_STREAM->CR |= DMA_SxCR_EN;
  SPI1_TX_DMA_STREAM->CR |= DMA_SxCR_EN;

}
uint32_t SPI1_RX_DMA_Counter[5];
uint32_t SPI1_TX_DMA_Counter[5];


extern "C" void SPI1_RX_DMA_IRQHandler() {
  if (DMA_GetITStatus(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_IT_TCIF)) {
    //DMA_ClearITPendingBit(SPI1_RX_DMA_STREAM, DMA_IT_TCIF3);
    DMA_ITConfig(SPI1_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
    SPI1_RX_DMA_Counter[0]++;
    SPI1_RX_DMA_STREAM->CR &= ~(uint32_t) DMA_SxCR_EN;
    SPI1_TX_DMA_STREAM->CR &= ~(uint32_t) DMA_SxCR_EN;
    CSN_HIGHT
    SPI1_RX_DMA_TC_Handler(0);
  }
  if (DMA_GetITStatus(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_IT_HTIF)) {
    DMA_ClearITPendingBit(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_IT_HTIF);
    SPI1_RX_DMA_Counter[1]++;

  }
  if (DMA_GetITStatus(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_IT_TEIF)) {
    DMA_ClearITPendingBit(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_IT_TEIF);

    SPI1_RX_DMA_Counter[2]++;
  }
  if (DMA_GetITStatus(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_IT_DMEIF)) {
    DMA_ClearITPendingBit(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_IT_DMEIF);
    SPI1_RX_DMA_Counter[3]++;

  }
  if (DMA_GetITStatus(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_IT_FEIF)) {
    DMA_ClearITPendingBit(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_IT_FEIF);
    SPI1_RX_DMA_Counter[4]++;

  }

}

extern "C" void SPI1_TX_DMA_IRQHandler() {
  if (DMA_GetITStatus(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_IT_TCIF)) {
    //DMA_ClearITPendingBit(SPI1_RX_DMA_STREAM, SPI1_TX_DMA_IT_HTIF);
    DMA_ITConfig(SPI1_TX_DMA_STREAM, DMA_IT_TC, DISABLE);
    SPI1_TX_DMA_Counter[0]++;
    SPI1_RX_DMA_STREAM->CR &= ~(uint32_t) DMA_SxCR_EN;
    SPI1_TX_DMA_STREAM->CR &= ~(uint32_t) DMA_SxCR_EN;
    CSN_HIGHT
    SPI1_TX_DMA_TC_Handler(0);
  }
  if (DMA_GetITStatus(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_IT_HTIF)) {
    DMA_ClearITPendingBit(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_IT_HTIF);
    SPI1_TX_DMA_Counter[1]++;

  }
  if (DMA_GetITStatus(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_IT_TEIF)) {
    DMA_ClearITPendingBit(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_IT_TEIF);

    SPI1_TX_DMA_Counter[2]++;
  }
  if (DMA_GetITStatus(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_IT_DMEIF)) {
    DMA_ClearITPendingBit(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_IT_DMEIF);
    SPI1_TX_DMA_Counter[3]++;

  }
  if (DMA_GetITStatus(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_IT_FEIF)) {
    DMA_ClearITPendingBit(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_IT_FEIF);
    SPI1_TX_DMA_Counter[4]++;

  }

}

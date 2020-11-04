//
// Created by ysbf on 6/29/20.
//

#include "USART6.h"
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_conf.h"
#ifdef __cplusplus
}
#endif


#define USART6_TX_DMA_STREAM DMA1_Stream4
#define USART6_RX_DMA_STREAM DMA1_Stream2
#define USART6_TX_DMA_STREAM_IRQHandler  DMA1_Stream4_IRQHandler
#define USART6_RX_DMA_STREAM_IRQHandler  DMA1_Stream2_IRQHandler
#define USART6_IRQHandler USART6_IRQHandler

#define USART6_TX_DMA_CHANNL DMA_Channel_3
#define USART6_RX_DMA_CHANNL DMA_Channel_3



#define USART6_RX_DMA_IRQ DMA1_Stream2_IRQn
#define USART6_TX_DMA_IRQ DMA1_Stream4_IRQn
#define USART6_IRQ USART6_IRQn



std::function<void(uint8_t remain_len)> USART6_IDLE_Handler;
std::function<void(uint8_t remain_len)> USART6_RX_DMA_TC_Handler;
std::function<void(uint8_t remain_len)> USART6_TX_DMA_TC_Handler;

USART6Driver::USART6Driver(bool use_dma) {

}
bool USART6Driver::init() {
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  /* config USART1 clock */
  RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_Init(USART6, &USART_InitStructure);
  USART_DMACmd(USART6, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
  //  USART6->CR3 |= USART_DMAReq_Rx | USART_DMAReq_Tx;
  //USART6_RX_DMA_STREAM Channel4

  DMA_DeInit(USART6_RX_DMA_STREAM);
  DMA_DeInit(USART6_TX_DMA_STREAM);
  DMA_InitTypeDef DMA_InitStructure;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;

  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&(USART6->DR));
  /* Configure TX DMA */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) (0);
  DMA_Init(USART6_TX_DMA_STREAM, &DMA_InitStructure);
  /* Configure RX DMA */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) (0);
  DMA_Init(USART6_RX_DMA_STREAM, &DMA_InitStructure);
  //
  //  DMA_ITConfig(USART6_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
  //  //  DMA_ITConfig(USART6_RX_DMA_STREAM, DMA_IT_HT, ENABLE);
  DMA_ITConfig(USART6_RX_DMA_STREAM, DMA_IT_TE, ENABLE);
  DMA_ITConfig(USART6_RX_DMA_STREAM, DMA_IT_FE, ENABLE);
  //
  //  DMA_ITConfig(USART6_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
  //  //  DMA_ITConfig(USART6_RX_DMA_STREAM, DMA_IT_HT, ENABLE);
  DMA_ITConfig(USART6_TX_DMA_STREAM, DMA_IT_TE, ENABLE);
  DMA_ITConfig(USART6_TX_DMA_STREAM, DMA_IT_FE, ENABLE);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = USART6_TX_DMA_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0f;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USART6_RX_DMA_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0f;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel =USART6_IRQ ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  (void) USART6->SR;
  (void) USART6->DR;
  USART_Cmd(USART6, ENABLE);
  //  DMA_Cmd(USART6_RX_DMA_STREAM, ENABLE);
  //	DMA_Cmd(DMA1_Stream6, DISABLE);

  //USART_ITConfig(USART1, USART_IT_TC, ENABLE);
  //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  //USART_ClearFlag(USART1, USART_FLAG_TC);
  //USART_ClearITPendingBit(USART1, USART_IT_RXNE);

  //(void)USART1->SR;
  //(void)USART1->DR;
}

void USART6Driver::dma_uart_read_irq_handler(uint8_t remain_len) {

  //		GPS_Rx_Buffer[0]=100-USART6_RX_DMA_STREAM->NDTR;
  //USART_ClearFlag(USART4, USART_FLAG_TC);

  // if(!DMA_GetFlagStatus(USART6_RX_DMA_STREAM, DMA_FLAG_TCIF1)){
  // 	USART6_RX_DMA_STREAM->CR &= ~(uint32_t)DMA_SxCR_EN;
  // 	USART6_RX_DMA_STREAM->NDTR=25;
  // 	USART6_RX_DMA_STREAM->CR |= (uint32_t)DMA_SxCR_EN;
  // 	return;
  // }
    uart_read_callback_fun(read_len - remain_len);

  // 	USART4->CR1 |= USART_CR1_UE;
}

void USART6Driver::dma_uart_write_irq_handler(uint8_t remain_len) {
  uart_write_callback_fun(write_len - remain_len);
}

void USART6Driver::dma_async_read(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t)> callback_fun) {

  uart_read_callback_fun = std::move(callback_fun);

  read_len = len;

  USART6_IDLE_Handler =[this](uint8_t remain_len){
    dma_uart_read_irq_handler (remain_len);
  };
  USART6_RX_DMA_TC_Handler=USART6_IDLE_Handler;

  if (!is_first_time_read) {
    while (!DMA_GetFlagStatus(USART6_RX_DMA_STREAM, DMA_FLAG_TCIF2));
  } else {
    is_first_time_read = 0;
  }
  uint8_t clear = USART6->SR;
  clear = USART6->DR;
  DMA_ClearFlag(USART6_RX_DMA_STREAM, DMA_FLAG_TCIF2);
  USART_ClearFlag(USART6, USART_FLAG_RXNE);
  DMA_ITConfig(USART6_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
  USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);

  USART6_RX_DMA_STREAM->CR &= ~DMA_SxCR_EN;
  while ((USART6_RX_DMA_STREAM->CR & DMA_SxCR_EN) != 0);
  USART6_RX_DMA_STREAM->NDTR = read_len;
  USART6_RX_DMA_STREAM->M0AR = (uint32_t) buffer;
  USART6->CR3 &= (uint16_t) ~USART_DMAReq_Rx;
  USART6_RX_DMA_STREAM->CR |= DMA_SxCR_EN;
  USART6->CR3 |= USART_DMAReq_Rx;

}

bool USART6Driver::dma_async_write(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t)> callback_fun) {

  uart_write_callback_fun = std::move(callback_fun);
  write_len = len;

  USART6_IDLE_Handler =[this](uint8_t remain_len){
    dma_uart_write_irq_handler (remain_len);
  };
  USART6_TX_DMA_TC_Handler=USART6_IDLE_Handler;

  if (!is_first_time_write) {
    while (!DMA_GetFlagStatus(USART6_TX_DMA_STREAM, DMA_FLAG_TCIF4));
  } else {
    is_first_time_write = 0;
  }

  uint8_t clear = USART6->SR;
  clear = USART6->DR;
  DMA_ClearFlag(USART6_TX_DMA_STREAM, DMA_FLAG_TCIF4);
  //  USART_ClearFlag(USART6, USART_FLAG_TC);
  DMA_ITConfig(USART6_TX_DMA_STREAM, DMA_IT_TC, ENABLE);

  USART6_TX_DMA_STREAM->CR &= ~DMA_SxCR_EN;
  while ((USART6_TX_DMA_STREAM->CR & DMA_SxCR_EN) != 0);
  USART6_TX_DMA_STREAM->NDTR = write_len;
  USART6_TX_DMA_STREAM->M0AR = (uint32_t) buffer;
  USART6->CR3 &= (uint16_t) ~USART_DMAReq_Tx;
  USART6->CR3 |= USART_DMAReq_Tx;
  USART6_TX_DMA_STREAM->CR |= DMA_SxCR_EN;

}
uint8_t USART6Driver::read_byte(uint8_t *data) {
  while (USART_GetFlagStatus(USART6, USART_FLAG_RXNE) == RESET);
  *data = USART_ReceiveData(USART6);
  return 0;
}
uint8_t USART6Driver::write_byte(const uint8_t value) {
  USART_SendData(USART6, value);
  while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
  return 0;
}
uint8_t USART6Driver::read_bytes(uint8_t *buffer, const uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    while (USART_GetFlagStatus(USART6, USART_FLAG_RXNE) == RESET);
    buffer[i] = USART_ReceiveData(USART6);
  }
  return 0;
}
uint8_t USART6Driver::write_bytes(const uint8_t *buffer, const uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    USART_SendData(USART6, buffer[i]);
    while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
  }
  return 0;
}
uint8_t USART6Driver::dma_read_bytes(const uint8_t *buffer, uint16_t len) {
  read_len = len;
  if (!is_first_time_read) {
    while (!DMA_GetFlagStatus(USART6_RX_DMA_STREAM, DMA_FLAG_TCIF2));
  } else {
    is_first_time_read = 0;
  }
  uint8_t clear = USART6->SR;
  clear = USART6->DR;
  DMA_ClearFlag(USART6_RX_DMA_STREAM, DMA_FLAG_TCIF2);
  USART_ClearFlag(USART6, USART_FLAG_RXNE);
  DMA_ITConfig(USART6_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
  USART_ITConfig(USART6, USART_IT_IDLE, DISABLE);

  USART_Cmd(USART6, DISABLE);
  USART6_RX_DMA_STREAM->CR &= ~DMA_SxCR_EN;
  while ((USART6_RX_DMA_STREAM->CR & DMA_SxCR_EN) != 0);
  USART6_RX_DMA_STREAM->NDTR = read_len;
  USART6_RX_DMA_STREAM->M0AR = (uint32_t) buffer;
  USART6->CR3 &= (uint16_t) ~USART_DMAReq_Rx;
  USART6_RX_DMA_STREAM->CR |= DMA_SxCR_EN;
  USART6_RX_DMA_STREAM->CR |= DMA_SxCR_EN;
  USART6->CR3 |= USART_DMAReq_Rx;
  USART_Cmd(USART6, ENABLE);
  while (!DMA_GetFlagStatus(USART6_RX_DMA_STREAM, DMA_FLAG_TCIF2));

}
uint8_t USART6Driver::dma_write_bytes(const uint8_t *buffer, uint16_t len) {

  write_len = len;
  if (!is_first_time_write) {
    while (!DMA_GetFlagStatus(USART6_TX_DMA_STREAM, DMA_FLAG_TCIF4));
  } else {
    is_first_time_write = 0;
  }
  uint8_t clear = USART6->SR;
  clear = USART6->DR;
  DMA_ClearFlag(USART6_TX_DMA_STREAM, DMA_FLAG_TCIF4);
  //  USART_ClearFlag(USART6, USART_FLAG_TC);
  DMA_ITConfig(USART6_TX_DMA_STREAM, DMA_IT_TC, DISABLE);

  USART_Cmd(USART6, DISABLE);
  USART6_TX_DMA_STREAM->CR &= ~DMA_SxCR_EN;
  while ((USART6_TX_DMA_STREAM->CR & DMA_SxCR_EN) != 0);
  USART6_TX_DMA_STREAM->NDTR = write_len;
  USART6_TX_DMA_STREAM->M0AR = (uint32_t) buffer;
  USART6->CR3 &= (uint16_t) ~USART_DMAReq_Tx;
  USART6_TX_DMA_STREAM->CR |= DMA_SxCR_EN;
  USART_Cmd(USART6, ENABLE);
  USART6->CR3 |= USART_DMAReq_Tx;
  while (!DMA_GetFlagStatus(USART6_TX_DMA_STREAM, DMA_FLAG_TCIF4));

}
uint8_t USART6Driver::dma_read_byte(uint8_t *data) {
  return dma_read_bytes(data, 1);
}
uint8_t USART6Driver::dma_write_byte(const uint8_t value) {
  return dma_write_bytes(&value, 1);
}

uint32_t D12_Counter[5];
uint32_t D14_Counter[5];


/*read_byte */
extern "C" void USART6_RX_DMA_STREAM_IRQHandler(void) {
  if (DMA_GetITStatus(USART6_RX_DMA_STREAM, DMA_IT_TCIF2)) {
    DMA_Cmd(USART6_RX_DMA_STREAM, DISABLE);
    //    DMA_ClearITPendingBit(USART6_RX_DMA_STREAM, DMA_IT_TCIF2);
    USART_ITConfig(USART6, USART_IT_IDLE, DISABLE);
    D12_Counter[0]++;
    DMA_ITConfig(USART6_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
    USART6_RX_DMA_TC_Handler((uint16_t) (USART6_RX_DMA_STREAM->NDTR));

  }
  if (DMA_GetITStatus(USART6_RX_DMA_STREAM, DMA_IT_HTIF2)) {
    DMA_ClearITPendingBit(USART6_RX_DMA_STREAM, DMA_IT_HTIF2);
    D12_Counter[1]++;

  }
  if (DMA_GetITStatus(USART6_RX_DMA_STREAM, DMA_IT_TEIF2)) {
    DMA_ClearITPendingBit(USART6_RX_DMA_STREAM, DMA_IT_TEIF2);
    D12_Counter[2]++;

  }
  if (DMA_GetITStatus(USART6_RX_DMA_STREAM, DMA_IT_DMEIF2)) {
    DMA_ClearITPendingBit(USART6_RX_DMA_STREAM, DMA_IT_DMEIF2);
    D12_Counter[3]++;

  }
  if (DMA_GetITStatus(USART6_RX_DMA_STREAM, DMA_IT_FEIF2)) {
    DMA_ClearITPendingBit(USART6_RX_DMA_STREAM, DMA_IT_FEIF2);
    D12_Counter[4]++;

  }

}
//
///*write*/
extern "C" void USART6_TX_DMA_STREAM_IRQHandler(void) {
  if (DMA_GetITStatus(USART6_TX_DMA_STREAM, DMA_IT_TCIF4)) {
    DMA_Cmd(USART6_TX_DMA_STREAM, DISABLE);
    //    DMA_ClearITPendingBit(USART6_TX_DMA_STREAM, DMA_IT_TCIF4);
    USART_ITConfig(USART6, USART_IT_IDLE, DISABLE);
    D14_Counter[0]++;

    DMA_ITConfig(USART6_TX_DMA_STREAM, DMA_IT_TC, DISABLE);
    USART6_TX_DMA_TC_Handler((uint16_t) (USART6_TX_DMA_STREAM->NDTR));

  }
  if (DMA_GetITStatus(USART6_TX_DMA_STREAM, DMA_IT_HTIF4)) {
    DMA_ClearITPendingBit(USART6_TX_DMA_STREAM, DMA_IT_HTIF4);
    D14_Counter[1]++;

  }
  if (DMA_GetITStatus(USART6_TX_DMA_STREAM, DMA_IT_TEIF4)) {
    DMA_ClearITPendingBit(USART6_TX_DMA_STREAM, DMA_IT_TEIF4);

    D14_Counter[2]++;
  }
  if (DMA_GetITStatus(USART6_TX_DMA_STREAM, DMA_IT_DMEIF4)) {
    DMA_ClearITPendingBit(USART6_TX_DMA_STREAM, DMA_IT_DMEIF4);
    D14_Counter[3]++;

  }
  if (DMA_GetITStatus(USART6_TX_DMA_STREAM, DMA_IT_FEIF4)) {
    DMA_ClearITPendingBit(USART6_TX_DMA_STREAM, DMA_IT_FEIF4);
    D14_Counter[4]++;

  }

}


extern "C" void USART6_IRQHandler(void) {

  if (USART_GetITStatus(USART6, USART_IT_IDLE)) {
    USART_ClearFlag(USART6, USART_FLAG_RXNE);
    DMA_ClearITPendingBit(USART6_RX_DMA_STREAM, DMA_IT_TCIF2);
    DMA_ITConfig(USART6_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
    DMA_ITConfig(USART6_TX_DMA_STREAM, DMA_IT_TC, DISABLE);
    DMA_Cmd(USART6_RX_DMA_STREAM, DISABLE);
    USART6_IDLE_Handler((uint16_t) (USART6_RX_DMA_STREAM->NDTR));
  }

}

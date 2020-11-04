//
// Created by ysbf on 6/29/20.
//

#include "UART4.h"
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_conf.h"
#ifdef __cplusplus
}
#endif


#define UART4_TX_DMA_STREAM DMA1_Stream4
#define UART4_RX_DMA_STREAM DMA1_Stream2
#define UART4_TX_DMA_STREAM_IRQHandler  DMA1_Stream4_IRQHandler
U
#define UART4_RX_DMA_STREAM_IRQHandler  DMA1_Stream2_IRQHandler

std::function<void(uint8_t remain_len)> UART4_IDLE_Handler;
std::function<void(uint8_t remain_len)> UART4_RX_DMA_TC_Handler;
std::function<void(uint8_t remain_len)> UART4_TX_DMA_TC_Handler;

UART4Driver::UART4Driver(bool use_dma) {

}
bool UART4Driver::init() {
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  /* config USART1 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_Init(UART4, &USART_InitStructure);
  USART_DMACmd(UART4, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
  //  UART4->CR3 |= USART_DMAReq_Rx | USART_DMAReq_Tx;
  //UART4_RX_DMA_STREAM Channel4

  DMA_DeInit(UART4_RX_DMA_STREAM);
  DMA_DeInit(UART4_TX_DMA_STREAM);
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

  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&(UART4->DR));
  /* Configure TX DMA */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) (0);
  DMA_Init(UART4_TX_DMA_STREAM, &DMA_InitStructure);
  /* Configure RX DMA */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) (0);
  DMA_Init(UART4_RX_DMA_STREAM, &DMA_InitStructure);
  //
  //  DMA_ITConfig(UART4_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
  //  //  DMA_ITConfig(UART4_RX_DMA_STREAM, DMA_IT_HT, ENABLE);
  DMA_ITConfig(UART4_RX_DMA_STREAM, DMA_IT_TE, ENABLE);
  DMA_ITConfig(UART4_RX_DMA_STREAM, DMA_IT_FE, ENABLE);
  //
  //  DMA_ITConfig(UART4_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
  //  //  DMA_ITConfig(UART4_RX_DMA_STREAM, DMA_IT_HT, ENABLE);
  DMA_ITConfig(UART4_TX_DMA_STREAM, DMA_IT_TE, ENABLE);
  DMA_ITConfig(UART4_TX_DMA_STREAM, DMA_IT_FE, ENABLE);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0f;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0f;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  (void) UART4->SR;
  (void) UART4->DR;
  USART_Cmd(UART4, ENABLE);
  //  DMA_Cmd(UART4_RX_DMA_STREAM, ENABLE);
  //	DMA_Cmd(DMA1_Stream6, DISABLE);

  //USART_ITConfig(USART1, USART_IT_TC, ENABLE);
  //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  //USART_ClearFlag(USART1, USART_FLAG_TC);
  //USART_ClearITPendingBit(USART1, USART_IT_RXNE);

  //(void)USART1->SR;
  //(void)USART1->DR;
}

void UART4Driver::dma_uart_read_irq_handler(uint8_t remain_len) {

  //		GPS_Rx_Buffer[0]=100-UART4_RX_DMA_STREAM->NDTR;
  //USART_ClearFlag(USART4, USART_FLAG_TC);

  // if(!DMA_GetFlagStatus(UART4_RX_DMA_STREAM, DMA_FLAG_TCIF1)){
  // 	UART4_RX_DMA_STREAM->CR &= ~(uint32_t)DMA_SxCR_EN;
  // 	UART4_RX_DMA_STREAM->NDTR=25;
  // 	UART4_RX_DMA_STREAM->CR |= (uint32_t)DMA_SxCR_EN;
  // 	return;
  // }
    uart_read_callback_fun(read_len - remain_len);

  // 	USART4->CR1 |= USART_CR1_UE;
}

void UART4Driver::dma_uart_write_irq_handler(uint8_t remain_len) {
  uart_write_callback_fun(write_len - remain_len);
}

void UART4Driver::dma_async_read(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t)> callback_fun) {

  uart_read_callback_fun = std::move(callback_fun);

  read_len = len;

  UART4_IDLE_Handler =[this](uint8_t remain_len){
    dma_uart_read_irq_handler (remain_len);
  };
  UART4_RX_DMA_TC_Handler=UART4_IDLE_Handler;

  if (!is_first_time_read) {
    while (!DMA_GetFlagStatus(UART4_RX_DMA_STREAM, DMA_FLAG_TCIF2));
  } else {
    is_first_time_read = 0;
  }
  uint8_t clear = UART4->SR;
  clear = UART4->DR;
  DMA_ClearFlag(UART4_RX_DMA_STREAM, DMA_FLAG_TCIF2);
  USART_ClearFlag(UART4, USART_FLAG_RXNE);
  DMA_ITConfig(UART4_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
  USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);

  UART4_RX_DMA_STREAM->CR &= ~DMA_SxCR_EN;
  while ((UART4_RX_DMA_STREAM->CR & DMA_SxCR_EN) != 0);
  UART4_RX_DMA_STREAM->NDTR = read_len;
  UART4_RX_DMA_STREAM->M0AR = (uint32_t) buffer;
  UART4->CR3 &= (uint16_t) ~USART_DMAReq_Rx;
  UART4_RX_DMA_STREAM->CR |= DMA_SxCR_EN;
  UART4->CR3 |= USART_DMAReq_Rx;

}

bool UART4Driver::dma_async_write(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t)> callback_fun) {

  uart_write_callback_fun = std::move(callback_fun);
  write_len = len;

  UART4_IDLE_Handler =[this](uint8_t remain_len){
    dma_uart_write_irq_handler (remain_len);
  };
  UART4_TX_DMA_TC_Handler=UART4_IDLE_Handler;

  if (!is_first_time_write) {
    while (!DMA_GetFlagStatus(UART4_TX_DMA_STREAM, DMA_FLAG_TCIF4));
  } else {
    is_first_time_write = 0;
  }

  uint8_t clear = UART4->SR;
  clear = UART4->DR;
  DMA_ClearFlag(UART4_TX_DMA_STREAM, DMA_FLAG_TCIF4);
  //  USART_ClearFlag(UART4, USART_FLAG_TC);
  DMA_ITConfig(UART4_TX_DMA_STREAM, DMA_IT_TC, ENABLE);

  UART4_TX_DMA_STREAM->CR &= ~DMA_SxCR_EN;
  while ((UART4_TX_DMA_STREAM->CR & DMA_SxCR_EN) != 0);
  UART4_TX_DMA_STREAM->NDTR = write_len;
  UART4_TX_DMA_STREAM->M0AR = (uint32_t) buffer;
  UART4->CR3 &= (uint16_t) ~USART_DMAReq_Tx;
  UART4->CR3 |= USART_DMAReq_Tx;
  UART4_TX_DMA_STREAM->CR |= DMA_SxCR_EN;

}
uint8_t UART4Driver::read_byte(uint8_t *data) {
  while (USART_GetFlagStatus(UART4, USART_FLAG_RXNE) == RESET);
  *data = USART_ReceiveData(UART4);
  return 0;
}
uint8_t UART4Driver::write_byte(const uint8_t value) {
  USART_SendData(UART4, value);
  while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
  return 0;
}
uint8_t UART4Driver::read_bytes(uint8_t *buffer, const uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    while (USART_GetFlagStatus(UART4, USART_FLAG_RXNE) == RESET);
    buffer[i] = USART_ReceiveData(UART4);
  }
  return 0;
}
uint8_t UART4Driver::write_bytes(const uint8_t *buffer, const uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    USART_SendData(UART4, buffer[i]);
    while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
  }
  return 0;
}
uint8_t UART4Driver::dma_read_bytes(const uint8_t *buffer, uint16_t len) {
  read_len = len;
  if (!is_first_time_read) {
    while (!DMA_GetFlagStatus(UART4_RX_DMA_STREAM, DMA_FLAG_TCIF2));
  } else {
    is_first_time_read = 0;
  }
  uint8_t clear = UART4->SR;
  clear = UART4->DR;
  DMA_ClearFlag(UART4_RX_DMA_STREAM, DMA_FLAG_TCIF2);
  USART_ClearFlag(UART4, USART_FLAG_RXNE);
  DMA_ITConfig(UART4_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
  USART_ITConfig(UART4, USART_IT_IDLE, DISABLE);

  USART_Cmd(UART4, DISABLE);
  UART4_RX_DMA_STREAM->CR &= ~DMA_SxCR_EN;
  while ((UART4_RX_DMA_STREAM->CR & DMA_SxCR_EN) != 0);
  UART4_RX_DMA_STREAM->NDTR = read_len;
  UART4_RX_DMA_STREAM->M0AR = (uint32_t) buffer;
  UART4->CR3 &= (uint16_t) ~USART_DMAReq_Rx;
  UART4_RX_DMA_STREAM->CR |= DMA_SxCR_EN;
  UART4_RX_DMA_STREAM->CR |= DMA_SxCR_EN;
  UART4->CR3 |= USART_DMAReq_Rx;
  USART_Cmd(UART4, ENABLE);
  while (!DMA_GetFlagStatus(UART4_RX_DMA_STREAM, DMA_FLAG_TCIF2));

}
uint8_t UART4Driver::dma_write_bytes(const uint8_t *buffer, uint16_t len) {

  write_len = len;
  if (!is_first_time_write) {
    while (!DMA_GetFlagStatus(UART4_TX_DMA_STREAM, DMA_FLAG_TCIF4));
  } else {
    is_first_time_write = 0;
  }
  uint8_t clear = UART4->SR;
  clear = UART4->DR;
  DMA_ClearFlag(UART4_TX_DMA_STREAM, DMA_FLAG_TCIF4);
  //  USART_ClearFlag(UART4, USART_FLAG_TC);
  DMA_ITConfig(UART4_TX_DMA_STREAM, DMA_IT_TC, DISABLE);

  USART_Cmd(UART4, DISABLE);
  UART4_TX_DMA_STREAM->CR &= ~DMA_SxCR_EN;
  while ((UART4_TX_DMA_STREAM->CR & DMA_SxCR_EN) != 0);
  UART4_TX_DMA_STREAM->NDTR = write_len;
  UART4_TX_DMA_STREAM->M0AR = (uint32_t) buffer;
  UART4->CR3 &= (uint16_t) ~USART_DMAReq_Tx;
  UART4_TX_DMA_STREAM->CR |= DMA_SxCR_EN;
  USART_Cmd(UART4, ENABLE);
  UART4->CR3 |= USART_DMAReq_Tx;
  while (!DMA_GetFlagStatus(UART4_TX_DMA_STREAM, DMA_FLAG_TCIF4));

}
uint8_t UART4Driver::dma_read_byte(uint8_t *data) {
  return dma_read_bytes(data, 1);
}
uint8_t UART4Driver::dma_write_byte(const uint8_t value) {
  return dma_write_bytes(&value, 1);
}

uint32_t D12_Counter[5];
uint32_t D14_Counter[5];


/*read_byte */
extern "C" void UART4_RX_DMA_STREAM_IRQHandler(void) {
  if (DMA_GetITStatus(UART4_RX_DMA_STREAM, DMA_IT_TCIF2)) {
    DMA_Cmd(UART4_RX_DMA_STREAM, DISABLE);
    //    DMA_ClearITPendingBit(UART4_RX_DMA_STREAM, DMA_IT_TCIF2);
    USART_ITConfig(UART4, USART_IT_IDLE, DISABLE);
    D12_Counter[0]++;
    DMA_ITConfig(UART4_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
    UART4_RX_DMA_TC_Handler((uint16_t) (UART4_RX_DMA_STREAM->NDTR));

  }
  if (DMA_GetITStatus(UART4_RX_DMA_STREAM, DMA_IT_HTIF2)) {
    DMA_ClearITPendingBit(UART4_RX_DMA_STREAM, DMA_IT_HTIF2);
    D12_Counter[1]++;

  }
  if (DMA_GetITStatus(UART4_RX_DMA_STREAM, DMA_IT_TEIF2)) {
    DMA_ClearITPendingBit(UART4_RX_DMA_STREAM, DMA_IT_TEIF2);
    D12_Counter[2]++;

  }
  if (DMA_GetITStatus(UART4_RX_DMA_STREAM, DMA_IT_DMEIF2)) {
    DMA_ClearITPendingBit(UART4_RX_DMA_STREAM, DMA_IT_DMEIF2);
    D12_Counter[3]++;

  }
  if (DMA_GetITStatus(UART4_RX_DMA_STREAM, DMA_IT_FEIF2)) {
    DMA_ClearITPendingBit(UART4_RX_DMA_STREAM, DMA_IT_FEIF2);
    D12_Counter[4]++;

  }

}
//
///*write*/
extern "C" void UART4_TX_DMA_STREAM_IRQHandler(void) {
  if (DMA_GetITStatus(UART4_TX_DMA_STREAM, DMA_IT_TCIF4)) {
    DMA_Cmd(UART4_TX_DMA_STREAM, DISABLE);
    //    DMA_ClearITPendingBit(UART4_TX_DMA_STREAM, DMA_IT_TCIF4);
    USART_ITConfig(UART4, USART_IT_IDLE, DISABLE);
    D14_Counter[0]++;

    DMA_ITConfig(UART4_TX_DMA_STREAM, DMA_IT_TC, DISABLE);
    UART4_TX_DMA_TC_Handler((uint16_t) (UART4_TX_DMA_STREAM->NDTR));

  }
  if (DMA_GetITStatus(UART4_TX_DMA_STREAM, DMA_IT_HTIF4)) {
    DMA_ClearITPendingBit(UART4_TX_DMA_STREAM, DMA_IT_HTIF4);
    D14_Counter[1]++;

  }
  if (DMA_GetITStatus(UART4_TX_DMA_STREAM, DMA_IT_TEIF4)) {
    DMA_ClearITPendingBit(UART4_TX_DMA_STREAM, DMA_IT_TEIF4);

    D14_Counter[2]++;
  }
  if (DMA_GetITStatus(UART4_TX_DMA_STREAM, DMA_IT_DMEIF4)) {
    DMA_ClearITPendingBit(UART4_TX_DMA_STREAM, DMA_IT_DMEIF4);
    D14_Counter[3]++;

  }
  if (DMA_GetITStatus(UART4_TX_DMA_STREAM, DMA_IT_FEIF4)) {
    DMA_ClearITPendingBit(UART4_TX_DMA_STREAM, DMA_IT_FEIF4);
    D14_Counter[4]++;

  }

}


extern "C" void UART4_IRQHandler(void) {

  if (USART_GetITStatus(UART4, USART_IT_IDLE)) {
    USART_ClearFlag(UART4, USART_FLAG_RXNE);
    DMA_ClearITPendingBit(UART4_RX_DMA_STREAM, DMA_IT_TCIF2);
    DMA_ITConfig(UART4_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
    DMA_ITConfig(UART4_TX_DMA_STREAM, DMA_IT_TC, DISABLE);
    DMA_Cmd(UART4_RX_DMA_STREAM, DISABLE);
    UART4_IDLE_Handler((uint16_t) (UART4_RX_DMA_STREAM->NDTR));
  }

}

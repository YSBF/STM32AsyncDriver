//
// Created by ysbf on 6/29/20.
//

#include "UART.h"
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_conf.h"
#ifdef __cplusplus
}
#endif

#define USART1_IDLE_Handler UARTDriver::UART_Callback[0].UART_IDLE_HANDLER
#define DMA2_STREAM7_TC_Handler UARTDriver::UART_Callback[0].DMA_TC_HANDLER
#define    DMA2_STREAM2_RC_Handler UARTDriver::UART_Callback[0].DMA_RC_HANDLER

#define USART2_IDLE_Handler UARTDriver::UART_Callback[1].UART_IDLE_HANDLER
#define DMA1_STREAM6_TC_Handler UARTDriver::UART_Callback[1].DMA_TC_HANDLER
#define    DMA1_STREAM5_RC_Handler UARTDriver::UART_Callback[1].DMA_RC_HANDLER

#define USART3_IDLE_Handler UARTDriver::UART_Callback[2].UART_IDLE_HANDLER
#define DMA1_STREAM3_TC_Handler UARTDriver::UART_Callback[2].DMA_TC_HANDLER
#define    DMA1_STREAM1_RC_Handler UARTDriver::UART_Callback[2].DMA_RC_HANDLER

#define UART4_IDLE_Handler UARTDriver::UART_Callback[3].UART_IDLE_HANDLER
#define DMA1_STREAM4_TC_Handler UARTDriver::UART_Callback[3].DMA_TC_HANDLER
#define    DMA1_STREAM2_RC_Handler UARTDriver::UART_Callback[3].DMA_RC_HANDLER

#define UART5_IDLE_Handler UARTDriver::UART_Callback[4].UART_IDLE_HANDLER
#define DMA1_STREAM7_TC_Handler UARTDriver::UART_Callback[4].DMA_TC_HANDLER
#define    DMA1_STREAM0_RC_Handler UARTDriver::UART_Callback[4].DMA_RC_HANDLER

#define USART6_IDLE_Handler UARTDriver::UART_Callback[5].UART_IDLE_HANDLER
#define DMA2_STREAM6_TC_Handler UARTDriver::UART_Callback[5].DMA_TC_HANDLER
#define    DMA2_STREAM1_RC_Handler UARTDriver::UART_Callback[5].DMA_RC_HANDLER

UARTDriver::UART_CALLBACK UARTDriver::UART_Callback[7] = {
    UART_CALLBACK{
        USART1,
        DMA2_Stream7,
        DMA2_Stream2,
        DMA_Channel_4,
        DMA2_Stream7_IRQn,
        DMA2_Stream2_IRQn,
        USART1_IRQn,
        DMA_FLAG_TCIF7,
        DMA_FLAG_TCIF2,

        std::function<void()>{},
        std::function<void()>{},
        std::function<void()>{},
    },
    UART_CALLBACK{
        USART2,
        DMA1_Stream6,
        DMA1_Stream5,
        DMA_Channel_4,
        DMA1_Stream6_IRQn,
        DMA1_Stream5_IRQn,
        USART2_IRQn,
        DMA_FLAG_TCIF6,
        DMA_FLAG_TCIF5,
        std::function<void()>{},
        std::function<void()>{},
        std::function<void()>{},
    },
    UART_CALLBACK{
        USART3,
        DMA1_Stream3,
        DMA1_Stream1,
        DMA_Channel_4,
        DMA1_Stream3_IRQn,
        DMA1_Stream1_IRQn,
        USART3_IRQn,
        DMA_FLAG_TCIF3,
        DMA_FLAG_TCIF1,
        std::function<void()>{},
        std::function<void()>{},
        std::function<void()>{},
    },
    UART_CALLBACK{
        UART4,
        DMA1_Stream4,
        DMA1_Stream2,
        DMA_Channel_4,
        DMA1_Stream4_IRQn,
        DMA1_Stream2_IRQn,
        UART4_IRQn,
        DMA_FLAG_TCIF4,
        DMA_FLAG_TCIF2,
        std::function<void()>{},
        std::function<void()>{},
        std::function<void()>{},
    },
    UART_CALLBACK{
        UART5,
        DMA1_Stream7,
        DMA1_Stream0,
        DMA_Channel_4,
        DMA1_Stream7_IRQn,
        DMA1_Stream0_IRQn,
        UART5_IRQn,
        DMA_FLAG_TCIF7,
        DMA_FLAG_TCIF0,
        std::function<void()>{},
        std::function<void()>{},
        std::function<void()>{},
    },
    UART_CALLBACK{
        USART6,
        DMA2_Stream6,
        DMA2_Stream1,
        DMA_Channel_5,
        DMA2_Stream6_IRQn,
        DMA2_Stream1_IRQn,
        USART6_IRQn,
        DMA_FLAG_TCIF6,
        DMA_FLAG_TCIF1,
        std::function<void()>{},
        std::function<void()>{},
        std::function<void()>{},
    }

};

bool UARTDriver::init(UART_INFO &info) {
  m_info = info;
  m_info.Num--;
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  uint8_t gpio_af;

  switch (m_info.Num) {
  case 0:RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); ///USART1
    gpio_af = GPIO_AF_USART1;
    if (m_info.TxUseDma || m_info.RxUseDma)
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); ///USART2
    break;
  case 1:RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    gpio_af = GPIO_AF_USART2;
    if (m_info.TxUseDma || m_info.RxUseDma)
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    break;
  case 2:RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    gpio_af = GPIO_AF_USART3;
    if (m_info.TxUseDma || m_info.RxUseDma)
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    break;
  case 3:RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    gpio_af = GPIO_AF_UART4;
    if (m_info.TxUseDma || m_info.RxUseDma)
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    break;

  case 4:RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
    gpio_af = GPIO_AF_UART5;
    if (m_info.TxUseDma || m_info.RxUseDma)
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    break;
  case 5:RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    gpio_af = GPIO_AF_USART6;
    if (m_info.TxUseDma || m_info.RxUseDma)
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    break;
  case 6:RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7, ENABLE);
    gpio_af = GPIO_AF_UART7;
    if (m_info.TxUseDma || m_info.RxUseDma)
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    break;
  };
  UART_Callback[m_info.Num].UART_IDLE_HANDLER = std::bind(&UARTDriver::dma_uart_read_irq_handler, this);
  UART_Callback[m_info.Num].DMA_TC_HANDLER = std::bind(&UARTDriver::dma_uart_write_irq_handler, this);
  UART_Callback[m_info.Num].DMA_RC_HANDLER = std::bind(&UARTDriver::dma_uart_read_irq_handler, this);

  GPIO_TypeDef *gpio_tx, *gpio_rx;
  switch (m_info.GpioTx) {
  case GPIO::A:gpio_tx = GPIOA;
    break;
  case GPIO::B:gpio_tx = GPIOB;
    break;
  case GPIO::C:gpio_tx = GPIOC;
    break;
  case GPIO::D:gpio_tx = GPIOD;
    break;
  case GPIO::E:gpio_tx = GPIOE;
    break;
  }
  switch (m_info.GpioRx) {
  case GPIO::A:gpio_rx = GPIOA;
    break;
  case GPIO::B:gpio_rx = GPIOB;
    break;
  case GPIO::C:gpio_rx = GPIOC;
    break;
  case GPIO::D:gpio_rx = GPIOD;
    break;
  case GPIO::E:gpio_rx = GPIOE;
    break;
  }


  /* config rcc clock */
  uint32_t rcc_ahb_tx_gpio = 1 << m_info.GpioTx;
  uint32_t rcc_ahb_rx_gpio = 1 << m_info.GpioRx;
  RCC_AHB1PeriphClockCmd(rcc_ahb_tx_gpio | rcc_ahb_rx_gpio, ENABLE);


  /* config AF pin*/
  uint8_t gpio_tx_pin_source = m_info.GpioTxPinNum;
  uint8_t gpio_rx_pin_source = m_info.GpioRxPinNum;

  GPIO_PinAFConfig(gpio_tx, gpio_tx_pin_source, gpio_af);
  GPIO_PinAFConfig(gpio_rx, gpio_rx_pin_source, gpio_af);

  /* config gpio*/
  uint16_t gpio_tx_pin_num = 1u << m_info.GpioTxPinNum;
  uint16_t gpio_rx_pin_num = 1u << m_info.GpioRxPinNum;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = gpio_tx_pin_num;
  GPIO_Init(gpio_tx, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = gpio_rx_pin_num;
  GPIO_Init(gpio_rx, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = m_info.BaudRate;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_Init(UART_Callback[m_info.Num].uart, &USART_InitStructure);
  USART_DMACmd(UART_Callback[m_info.Num].uart, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
  //  UART->CR3 |= USART_DMAReq_Rx | USART_DMAReq_Tx;
  //UART_RX_DMA_STREAM Channel4

  DMA_DeInit(UART_Callback[m_info.Num].dma_tx_stream);
  DMA_DeInit(UART_Callback[m_info.Num].dma_rx_stream);
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

  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&(UART_Callback[m_info.Num].uart->DR));
  /* Configure TX DMA */
  DMA_InitStructure.DMA_Channel = UART_Callback[m_info.Num].dma_channel;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) (0);
  DMA_Init(UART_Callback[m_info.Num].dma_tx_stream, &DMA_InitStructure);
  /* Configure RX DMA */
  DMA_InitStructure.DMA_Channel = UART_Callback[m_info.Num].dma_channel;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) (0);
  DMA_Init(UART_Callback[m_info.Num].dma_rx_stream, &DMA_InitStructure);
  //
  //  DMA_ITConfig(UART_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
  //  //  DMA_ITConfig(UART_RX_DMA_STREAM, DMA_IT_HT, ENABLE);
  DMA_ITConfig(UART_Callback[m_info.Num].dma_tx_stream, DMA_IT_TE, ENABLE);
  DMA_ITConfig(UART_Callback[m_info.Num].dma_rx_stream, DMA_IT_FE, ENABLE);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = UART_Callback[m_info.Num].dma_tx_irq;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0f;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = UART_Callback[m_info.Num].dma_rx_irq;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0f;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = UART_Callback[m_info.Num].uart_irq;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  (void) UART_Callback[m_info.Num].uart->SR;
  (void) UART_Callback[m_info.Num].uart->DR;
  USART_Cmd(UART_Callback[m_info.Num].uart, ENABLE);
  //  DMA_Cmd(UART_RX_DMA_STREAM, ENABLE);
  //	DMA_Cmd(DMA1_Stream6, DISABLE);

  //USART_ITConfig(USART1, USART_IT_TC, ENABLE);
  //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  //USART_ClearFlag(USART1, USART_FLAG_TC);
  //USART_ClearITPendingBit(USART1, USART_IT_RXNE);

  //(void)USART1->SR;
  //(void)USART1->DR;
}

void UARTDriver::dma_uart_read_irq_handler() {

  DMA_Cmd(UART_Callback[m_info.Num].dma_rx_stream, DISABLE);
  DMA_ITConfig(UART_Callback[m_info.Num].dma_rx_stream, DMA_IT_TC, DISABLE);
  USART_ITConfig(UART_Callback[m_info.Num].uart, USART_IT_IDLE, DISABLE);

  uart_read_callback_fun(read_len - UART_Callback[m_info.Num].dma_rx_stream->NDTR);

  // 	USART4->CR1 |= USART_CR1_UE;
}

void UARTDriver::dma_uart_write_irq_handler() {

  DMA_Cmd(UART_Callback[m_info.Num].dma_tx_stream, DISABLE);
  DMA_ITConfig(UART_Callback[m_info.Num].dma_tx_stream, DMA_IT_TC, DISABLE);
  USART_ITConfig(UART_Callback[m_info.Num].uart, USART_IT_IDLE, DISABLE);
  uart_write_callback_fun(write_len - UART_Callback[m_info.Num].dma_tx_stream->NDTR);
}

void UARTDriver::dma_async_read(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t)> callback_fun) {

  uart_read_callback_fun = std::move(callback_fun);

  read_len = len;

  UART_Callback[m_info.Num].UART_IDLE_HANDLER = [this]() {
    dma_uart_read_irq_handler();
  };
  UART_Callback[m_info.Num].DMA_RC_HANDLER = UART_Callback[m_info.Num].UART_IDLE_HANDLER;

  if (!is_first_time_read) {
    while (!DMA_GetFlagStatus(UART_Callback[m_info.Num].dma_rx_stream, UART_Callback[m_info.Num].dma_rx_flag_tc));
  } else {
    is_first_time_read = 0;
  }
  uint8_t clear = UART_Callback[m_info.Num].uart->SR;
  clear = UART_Callback[m_info.Num].uart->DR;
  DMA_ClearFlag(UART_Callback[m_info.Num].dma_rx_stream, UART_Callback[m_info.Num].dma_rx_flag_tc);
  USART_ClearFlag(UART_Callback[m_info.Num].uart, USART_FLAG_RXNE);
  DMA_ITConfig(UART_Callback[m_info.Num].dma_rx_stream, DMA_IT_TC, ENABLE);
  USART_ITConfig(UART_Callback[m_info.Num].uart, USART_IT_IDLE, ENABLE);

  UART_Callback[m_info.Num].dma_rx_stream->CR &= ~DMA_SxCR_EN;
  while ((UART_Callback[m_info.Num].dma_rx_stream->CR & DMA_SxCR_EN) != 0);
  UART_Callback[m_info.Num].dma_rx_stream->NDTR = read_len;
  UART_Callback[m_info.Num].dma_rx_stream->M0AR = (uint32_t) buffer;
  UART_Callback[m_info.Num].uart->CR3 &= (uint16_t) ~USART_DMAReq_Rx;
  UART_Callback[m_info.Num].dma_rx_stream->CR |= DMA_SxCR_EN;
  UART_Callback[m_info.Num].uart->CR3 |= USART_DMAReq_Rx;

}

bool UARTDriver::dma_async_write(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t)> callback_fun) {

  uart_write_callback_fun = std::move(callback_fun);
  write_len = len;

  UART_Callback[m_info.Num].UART_IDLE_HANDLER = [this]() {
    dma_uart_write_irq_handler();
  };
  UART_Callback[m_info.Num].DMA_TC_HANDLER = UART_Callback[m_info.Num].UART_IDLE_HANDLER;

  if (!is_first_time_write) {
    while (!DMA_GetFlagStatus(UART_Callback[m_info.Num].dma_tx_stream, UART_Callback[m_info.Num].dma_tx_flag_tc));
  } else {
    is_first_time_write = 0;
  }

  uint8_t clear = UART_Callback[m_info.Num].uart->SR;
  clear = UART_Callback[m_info.Num].uart->DR;
  DMA_ClearFlag(UART_Callback[m_info.Num].dma_tx_stream, UART_Callback[m_info.Num].dma_tx_flag_tc);
  //  USART_ClearFlag(UART, USART_FLAG_TC);
  DMA_ITConfig(UART_Callback[m_info.Num].dma_tx_stream, DMA_IT_TC, ENABLE);

  UART_Callback[m_info.Num].dma_tx_stream->CR &= ~DMA_SxCR_EN;
  while ((UART_Callback[m_info.Num].dma_tx_stream->CR & DMA_SxCR_EN) != 0);
  UART_Callback[m_info.Num].dma_tx_stream->NDTR = write_len;
  UART_Callback[m_info.Num].dma_tx_stream->M0AR = (uint32_t) buffer;
  UART_Callback[m_info.Num].uart->CR3 &= (uint16_t) ~USART_DMAReq_Tx;
  UART_Callback[m_info.Num].uart->CR3 |= USART_DMAReq_Tx;
  UART_Callback[m_info.Num].dma_tx_stream->CR |= DMA_SxCR_EN;

}
uint8_t UARTDriver::read_byte(uint8_t *data) {
  while (USART_GetFlagStatus(UART_Callback[m_info.Num].uart, USART_FLAG_RXNE) == RESET);
  *data = USART_ReceiveData(UART_Callback[m_info.Num].uart);
  return 0;
}
uint8_t UARTDriver::write_byte(const uint8_t value) {
  USART_SendData(UART_Callback[m_info.Num].uart, value);
  while (USART_GetFlagStatus(UART_Callback[m_info.Num].uart, USART_FLAG_TXE) == RESET);
  return 0;
}
uint8_t UARTDriver::read_bytes(uint8_t *buffer, const uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    while (USART_GetFlagStatus(UART_Callback[m_info.Num].uart, USART_FLAG_RXNE) == RESET);
    buffer[i] = USART_ReceiveData(UART_Callback[m_info.Num].uart);
  }
  return 0;
}
uint8_t UARTDriver::write_bytes(const uint8_t *buffer, const uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    USART_SendData(UART_Callback[m_info.Num].uart, buffer[i]);
    while (USART_GetFlagStatus(UART_Callback[m_info.Num].uart, USART_FLAG_TXE) == RESET);
  }
  return 0;
}
uint8_t UARTDriver::dma_read_bytes(const uint8_t *buffer, uint16_t len) {
  read_len = len;
  if (!is_first_time_read) {
    while (!DMA_GetFlagStatus(UART_Callback[m_info.Num].dma_rx_stream, UART_Callback[m_info.Num].dma_rx_flag_tc));
  } else {
    is_first_time_read = 0;
  }
  uint8_t clear = UART_Callback[m_info.Num].uart->SR;
  clear = UART_Callback[m_info.Num].uart->DR;
  DMA_ClearFlag(UART_Callback[m_info.Num].dma_rx_stream, UART_Callback[m_info.Num].dma_rx_flag_tc);
  USART_ClearFlag(UART_Callback[m_info.Num].uart, USART_FLAG_RXNE);
  DMA_ITConfig(UART_Callback[m_info.Num].dma_rx_stream, DMA_IT_TC, DISABLE);
  USART_ITConfig(UART_Callback[m_info.Num].uart, USART_IT_IDLE, DISABLE);

  USART_Cmd(UART_Callback[m_info.Num].uart, DISABLE);
  UART_Callback[m_info.Num].dma_rx_stream->CR &= ~DMA_SxCR_EN;
  while ((UART_Callback[m_info.Num].dma_rx_stream->CR & DMA_SxCR_EN) != 0);
  UART_Callback[m_info.Num].dma_rx_stream->NDTR = read_len;
  UART_Callback[m_info.Num].dma_rx_stream->M0AR = (uint32_t) buffer;
  UART_Callback[m_info.Num].uart->CR3 &= (uint16_t) ~USART_DMAReq_Rx;
  UART_Callback[m_info.Num].dma_rx_stream->CR |= DMA_SxCR_EN;
  UART_Callback[m_info.Num].dma_rx_stream->CR |= DMA_SxCR_EN;
  UART_Callback[m_info.Num].uart->CR3 |= USART_DMAReq_Rx;
  USART_Cmd(UART_Callback[m_info.Num].uart, ENABLE);
  while (!DMA_GetFlagStatus(UART_Callback[m_info.Num].dma_rx_stream, UART_Callback[m_info.Num].dma_rx_flag_tc));

}
uint8_t UARTDriver::dma_write_bytes(const uint8_t *buffer, uint16_t len) {

  write_len = len;
  if (!is_first_time_write) {
    while (!DMA_GetFlagStatus(UART_Callback[m_info.Num].dma_tx_stream, UART_Callback[m_info.Num].dma_tx_flag_tc));
  } else {
    is_first_time_write = 0;
  }
  uint8_t clear = UART_Callback[m_info.Num].uart->SR;
  clear = UART_Callback[m_info.Num].uart->DR;
  DMA_ClearFlag(UART_Callback[m_info.Num].dma_tx_stream, UART_Callback[m_info.Num].dma_tx_flag_tc);
  //  USART_ClearFlag(UART, USART_FLAG_TC);
  DMA_ITConfig(UART_Callback[m_info.Num].dma_tx_stream, DMA_IT_TC, DISABLE);

  USART_Cmd(UART_Callback[m_info.Num].uart, DISABLE);
  UART_Callback[m_info.Num].dma_tx_stream->CR &= ~DMA_SxCR_EN;
  while ((UART_Callback[m_info.Num].dma_tx_stream->CR & DMA_SxCR_EN) != 0);
  UART_Callback[m_info.Num].dma_tx_stream->NDTR = write_len;
  UART_Callback[m_info.Num].dma_tx_stream->M0AR = (uint32_t) buffer;
  UART_Callback[m_info.Num].uart->CR3 &= (uint16_t) ~USART_DMAReq_Tx;
  UART_Callback[m_info.Num].dma_tx_stream->CR |= DMA_SxCR_EN;
  USART_Cmd(UART_Callback[m_info.Num].uart, ENABLE);
  UART_Callback[m_info.Num].uart->CR3 |= USART_DMAReq_Tx;
  while (!DMA_GetFlagStatus(UART_Callback[m_info.Num].dma_tx_stream, UART_Callback[m_info.Num].dma_tx_flag_tc));

}
uint8_t UARTDriver::dma_read_byte(uint8_t *data) {
  return dma_read_bytes(data, 1);
}
uint8_t UARTDriver::dma_write_byte(const uint8_t value) {
  return dma_write_bytes(&value, 1);
}
UARTDriver::UARTDriver() {

}




extern "C" void USART1_IRQHandler(void) {

  if (USART_GetITStatus(USART1, USART_IT_IDLE)) {

    USART1_IDLE_Handler();
  }

}




uint32_t D27_Counter[5];
uint32_t D22_Counter[5];

/*read_byte */
extern "C" void DMA2_Stream7_IRQHandler(void) {
  if (DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7)) {
    D27_Counter[0]++;
    DMA2_STREAM7_TC_Handler();

  }
  if (DMA_GetITStatus(DMA2_Stream7, DMA_IT_HTIF7)) {
    DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_HTIF7);
    D27_Counter[1]++;

  }
  if (DMA_GetITStatus(DMA2_Stream7, DMA_IT_TEIF7)) {
    DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TEIF7);
    D27_Counter[2]++;

  }
  if (DMA_GetITStatus(DMA2_Stream7, DMA_IT_DMEIF7)) {
    DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_DMEIF7);
    D27_Counter[3]++;

  }
  if (DMA_GetITStatus(DMA2_Stream7, DMA_IT_FEIF7)) {
    DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_FEIF7);
    D27_Counter[4]++;

  }

}
//
///*write*/
extern "C" void DMA2_Stream2_IRQHandler(void) {
  if (DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2)) {
    D22_Counter[0]++;
    DMA2_STREAM2_RC_Handler();
  }
  if (DMA_GetITStatus(DMA2_Stream2, DMA_IT_HTIF2)) {
    DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_HTIF2);
    D22_Counter[1]++;

  }
  if (DMA_GetITStatus(DMA2_Stream2, DMA_IT_TEIF2)) {
    DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TEIF2);

    D22_Counter[2]++;
  }
  if (DMA_GetITStatus(DMA2_Stream2, DMA_IT_DMEIF2)) {
    DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_DMEIF2);
    D22_Counter[3]++;

  }
  if (DMA_GetITStatus(DMA2_Stream2, DMA_IT_FEIF2)) {
    DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_FEIF2);
    D22_Counter[4]++;

  }

}



extern "C" void USART6_IRQHandler(void) {

  if (USART_GetITStatus(USART6, USART_IT_IDLE)) {

    USART6_IDLE_Handler();
  }

}


uint32_t D21_Counter[5];
uint32_t D26_Counter[5];


/*read_byte */
extern "C" void DMA2_Stream1_IRQHandler(void) {
  if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1)) {
    D21_Counter[0]++;
    DMA2_STREAM1_RC_Handler();

  }
  if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_HTIF1)) {
    DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_HTIF1);
    D21_Counter[1]++;

  }
  if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TEIF1)) {
    DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TEIF1);
    D21_Counter[2]++;

  }
  if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_DMEIF1)) {
    DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_DMEIF1);
    D21_Counter[3]++;

  }
  if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_FEIF1)) {
    DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_FEIF1);
    D21_Counter[4]++;

  }

}
//
///*write*/
extern "C" void DMA2_Stream6_IRQHandler(void) {
  if (DMA_GetITStatus(DMA2_Stream6, DMA_IT_TCIF6)) {
    D26_Counter[0]++;
    DMA2_STREAM6_TC_Handler();
  }
  if (DMA_GetITStatus(DMA2_Stream6, DMA_IT_HTIF6)) {
    DMA_ClearITPendingBit(DMA2_Stream6, DMA_IT_HTIF6);
    D26_Counter[1]++;

  }
  if (DMA_GetITStatus(DMA2_Stream6, DMA_IT_TEIF6)) {
    DMA_ClearITPendingBit(DMA2_Stream6, DMA_IT_TEIF6);

    D26_Counter[2]++;
  }
  if (DMA_GetITStatus(DMA2_Stream6, DMA_IT_DMEIF6)) {
    DMA_ClearITPendingBit(DMA2_Stream6, DMA_IT_DMEIF6);
    D26_Counter[3]++;

  }
  if (DMA_GetITStatus(DMA2_Stream6, DMA_IT_FEIF6)) {
    DMA_ClearITPendingBit(DMA2_Stream6, DMA_IT_FEIF6);
    D26_Counter[4]++;

  }

}












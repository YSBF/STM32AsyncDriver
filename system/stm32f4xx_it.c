#include "stm32f4xx_it.h"
#include "stm32f4xx.h"









void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}
extern volatile uint32_t sysTickUptime;
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	sysTickUptime++;
  //TimingDelay_Decrement();
}
//7261
uint32_t D2IRQ[5]={0,0,0,0,0};
uint32_t D17IRQ[5]={0,0,0,0,0};
uint32_t D12IRQ[5]={0,0,0,0,0};

uint32_t D11IRQ[5]={0,0,0,0,0};
uint32_t D27IRQ[5]={0,0,0,0,0};

uint32_t C17IRQ,C12IRQ,C11IRQ,C15IRQ,counter;
uint32_t C27IRQ,C22IRQ,C26IRQ;
uint32_t half_counter;


void DMA1_Stream7_IRQHandler(void){
	if(DMA_GetITStatus(DMA1_Stream7, DMA_IT_TCIF7))	{
		DMA_ClearFlag(DMA1_Stream7, DMA_FLAG_TCIF7);
		C17IRQ=DMA_GetCurrDataCounter(DMA1_Stream7);
		D17IRQ[0]++;
	}
	if(DMA_GetITStatus(DMA1_Stream7, DMA_IT_FEIF7))	{
		DMA_ClearFlag(DMA1_Stream7, DMA_FLAG_FEIF7 );
		C17IRQ=DMA_GetCurrDataCounter(DMA1_Stream7);
		D17IRQ[1]++;
	}
	if(DMA_GetITStatus(DMA1_Stream7, DMA_IT_DMEIF7 ))	{
		DMA_ClearFlag(DMA1_Stream7, DMA_FLAG_DMEIF7  );
		C17IRQ=DMA_GetCurrDataCounter(DMA1_Stream7);
		D17IRQ[2]++;
	}
	if(DMA_GetITStatus(DMA1_Stream7, DMA_IT_TEIF7  ))	{
		DMA_ClearFlag(DMA1_Stream7, DMA_FLAG_TEIF7   );
		C17IRQ=DMA_GetCurrDataCounter(DMA1_Stream7);
		D17IRQ[3]++;
	}
	if(DMA_GetITStatus(DMA1_Stream7, DMA_IT_HTIF7  ))	{
		DMA_ClearFlag(DMA1_Stream7, DMA_FLAG_HTIF7  );
		C17IRQ=DMA_GetCurrDataCounter(DMA1_Stream7);
		D17IRQ[4]++;
	}
}




void DMA1_Stream1_IRQHandler(void){
	if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1))	{
		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1);
		C11IRQ=DMA_GetCurrDataCounter(DMA1_Stream1);
		D11IRQ[0]++;
	}
	if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_FEIF1))	{
		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_FEIF1 );
		C11IRQ=DMA_GetCurrDataCounter(DMA1_Stream1);
		D11IRQ[1]++;
	}
	if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_DMEIF1 ))	{
		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_DMEIF1  );
		C11IRQ=DMA_GetCurrDataCounter(DMA1_Stream1);
		D11IRQ[2]++;
	}
	if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_TEIF1  ))	{
		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TEIF1   );
		C11IRQ=DMA_GetCurrDataCounter(DMA1_Stream1);
		D11IRQ[3]++;
	}
	if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_HTIF1  ))	{
		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_HTIF1  );
		C11IRQ=DMA_GetCurrDataCounter(DMA1_Stream1);
		D11IRQ[4]++;
	}
}


















//
//extern uint8_t ESC_CMD[18];
//void DMA1_Stream5_IRQHandler(void){
//
//	if(DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5))	{
//		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
//
//		counter=DMA_GetCurrDataCounter(DMA1_Stream5);
//		D2IRQ[0]++;
//
//
////		DMA_Cmd(DMA2_Stream7, DISABLE);
////		DMA_Cmd(DMA2_Stream7, ENABLE);
//	}
//
//	if(DMA_GetITStatus(DMA1_Stream5, DMA_IT_FEIF5))	{
//		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_FEIF5 );
//		counter=DMA_GetCurrDataCounter(DMA1_Stream5);
//		D2IRQ[1]++;
//	}
//	if(DMA_GetITStatus(DMA1_Stream5, DMA_IT_DMEIF5 ))	{
//		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_DMEIF5  );
//		counter=DMA_GetCurrDataCounter(DMA1_Stream5);
//		D2IRQ[2]++;
//	}
//
//	if(DMA_GetITStatus(DMA1_Stream5, DMA_IT_TEIF5  ))	{
//		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TEIF5   );
//		counter=DMA_GetCurrDataCounter(DMA1_Stream5);
//		D2IRQ[3]++;
//	}
//	if(DMA_GetITStatus(DMA1_Stream5, DMA_IT_HTIF5  ))	{
//		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_HTIF5  );
//		counter=DMA_GetCurrDataCounter(DMA1_Stream5);
//		D2IRQ[4]++;
//	}
//
//
//
//
////	counter++;
////	if(DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5))	{
////		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
////		DMA_Cmd(DMA1_Stream5, DISABLE);
////		 while(DMA_GetCmdStatus(DMA1_Stream5) != DISABLE);
////
//////		 while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
//////		 USART_SendData(USART1, 0x11 );
////
////		 DMA_SetCurrDataCounter(DMA1_Stream5,4);
////		 DMA_Cmd(DMA1_Stream5, ENABLE);
////		//counter=DMA_GetCurrDataCounter(DMA1_Stream5);
////
//////		static uint8_t LEDVALUE;
//////
//////
//////			LEDVALUE++;
//////
//////			ESC_CMD[0]=(LEDVALUE%2)*4000+1000;
////		//DMA_Cmd(DMA2_Stream7, ENABLE);
////		//counter=DMA_GetCurrDataCounter(DMA2_Stream7);
////
////
////	}
//
//}
void TIM3_IRQHandler(void){
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)){
//		if(counter%2==0){
//		TIM3->CCR2=2400;
//		}
//		if(counter%2==1){
//		TIM3->CCR2=200;
//		}
//		counter++;
	}
	if(TIM_GetITStatus(TIM3,TIM_IT_CC1)){
		counter=200;
	}
	if(TIM_GetITStatus(TIM3,TIM_IT_CC2)){


	}
	if(TIM_GetITStatus(TIM3,TIM_IT_Trigger)){
		counter=400;
	}
	if(TIM_GetITStatus(TIM3,TIM_IT_Break)){
		counter=500;
	}
//	TIM3->CCR2=2400;
//	DelayMs(1);
//	TIM3->CCR2=200;
//	DelayMs(1);

}



uint16_t DMA_USART_RecieveBuffer_Len;
//
//
//void USART1_IRQHandler(void){
//    uint8_t ID = 0;        //串口序号    uint16_t Size = 0;
//
//    if ( USART_GetITStatus( USART1, USART_IT_IDLE ) != RESET )           //串口空闲中断 用于DMA        {
//        USART_ClearITPendingBit( USART1, USART_IT_IDLE );
//        USART_ReceiveData( USART1);
//        USART_ClearFlag( USART1, USART_FLAG_IDLE );
//        DMA_Cmd(DMA2_Stream2, DISABLE);
//
//        uint16_t Size = DMA_USART_RecieveBuffer_Len - DMA_GetCurrDataCounter( DMA2_Stream2 );
//        if(Size > 0)
//            FIFO_PutBuff( &USART1_FIFO_Receive, DMA_USART1_Recieve_Buff, Size );        //将收到的DMA缓冲复制进串口输入FIFO  FIFO_PutBuff()函数是环型缓冲区写入Buff的用到的，USART1_FIFO_Receive 环型缓冲区结构,  DMA_USART1_Recieve_Buff DMA接收缓冲区
//       USART_DMA_Status[ID].USART_IS_IDLE = 1;  //自定义标识 串口空闲标识
//       USART_DMA_Status[ID].DMA_ReceiveOK = 1;  //自定义标识 DMA接收完成标识
//
//
//    DMA_SetCurrDataCounter( DMA2_Stream2, DMA_USART_RecieveBuffer_Len );
//    DMA_Cmd(DMA2_Stream2, ENABLE);  //开启DMA
//
//}
//}

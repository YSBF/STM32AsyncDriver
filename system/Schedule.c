#include "stm32f4xx_conf.h"
#include "stdarg.h"
#include "stdio.h"

#define TIM7_PERIOD_RESET 0

void Schedule_Init() {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);//84mhz

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
  //    uint16_t PrescalerValue =  (uint16_t) ((SystemCoreClock / 2) / 1000000) - 1;
  uint16_t PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 1000000) - 1;
  TIM_TimeBaseStruct.TIM_ClockDivision = 0;
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseStruct.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStruct.TIM_Period = 0;
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStruct);

  TIM_ARRPreloadConfig(TIM7, DISABLE);

  TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

  TIM_ClearFlag(TIM7, TIM_FLAG_Update);

  NVIC_InitTypeDef NVIC_InitStruct;

  NVIC_InitStruct.NVIC_IRQChannel = TIM7_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  //
  //GPIO_InitTypeDef GPIO_InitStruct;
  //GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
  //GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
  //GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
  //GPIO_InitStruct.GPIO_Pin=GPIO_Pin_0;
  //GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
  //GPIO_Init(GPIOE, &GPIO_InitStruct);
  //GPIO_SetBits(GPIOE, GPIO_Pin_0);




}

typedef void (*Fun)(void);
#define Queue_len 10
typedef struct {

  volatile int32_t time_period;
  volatile Fun function;
} Routine_Queue;

typedef struct {

  volatile uint8_t list[Queue_len];
  volatile int16_t list_len;
} Routine_Queue_Info;

#define ROUTINE_TIMER TIM7
//Routine_Queue_Info Queue_info;
volatile Routine_Queue_Info Queue_info;
Routine_Queue Queue[Queue_len];

void Shift_Array(uint8_t *array, uint8_t move_to, uint8_t move_from) {

  if (move_to > move_from) {

    uint8_t temp = array[move_from];
    for (uint8_t i = move_from; i < move_to; i++) {
      array[i] = array[i + 1];
    }
    array[move_to] = temp;

  } else {
    uint8_t temp = array[move_from];
    for (uint8_t i = move_from; i > move_to; i--) {
      array[i] = array[i - 1];
    }
    array[move_to] = temp;
  }
}

void SetLine(Fun fun, uint16_t period) {
  static uint8_t isfirstelement = 1;
  if (isfirstelement) {
    for (uint8_t i = 0; i < Queue_len; i++) {
      Queue_info.list[i] = i;
    }
    Queue_info.list_len = 0;
    isfirstelement = 0;
    TIM_Cmd(TIM7, ENABLE);
  }

  TIM_ITConfig(TIM7, TIM_IT_Update, DISABLE);

  if (Queue_info.list_len == Queue_len) {
    Debug("line fifo full");
    return;
  }

  Queue_info.list_len++;

  Queue[Queue_info.list[Queue_info.list_len - 1]].function = fun;

  if (Queue_info.list_len == 1) {
    Queue[Queue_info.list[0]].time_period = 0;
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    ROUTINE_TIMER->ARR = period;
    ROUTINE_TIMER->CNT = 0;

    return;
  }

  uint32_t TimerLast = ROUTINE_TIMER->ARR - ROUTINE_TIMER->CNT;

  Queue[Queue_info.list[Queue_info.list_len - 1]].time_period = period - TimerLast;

  uint8_t inserted = 0;
  for (uint8_t i = 0; i < Queue_info.list_len; i++) {

    if (Queue[Queue_info.list[i]].time_period > Queue[Queue_info.list[Queue_info.list_len - 1]].time_period && inserted == 0) {
      inserted = 1;
      Shift_Array((uint8_t *) &Queue_info.list, i, Queue_info.list_len - 1);
      if (i == 0) {//negetive
        Queue[Queue_info.list[1]].time_period = TimerLast - period;

        ROUTINE_TIMER->ARR += Queue[Queue_info.list[0]].time_period;
        Queue[Queue_info.list[0]].time_period = 0;
      } else {

      }
    } else if (inserted == 0 && i < Queue_info.list_len - 1) {
      Queue[Queue_info.list[Queue_info.list_len - 1]].time_period -= Queue[Queue_info.list[i]].time_period;
    }

    if (inserted == 1) {
      inserted++;

    } else if (inserted == 2) {
      Queue[Queue_info.list[i]].time_period -= Queue[Queue_info.list[i - 1]].time_period;
    }

  }
  TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
}
uint8_t pinout_status;

void TIM7_IRQHandler() {

  if (TIM_GetITStatus(TIM7, TIM_IT_Update)) {
    TIM_ClearITPendingBit(TIM7, TIM_FLAG_Update);

    if (Queue_info.list_len <= 0) {
      ROUTINE_TIMER->ARR = 0;
      printf("queue empty erro arr:%ld", ROUTINE_TIMER->ARR);
      return;
    }

    (Queue[Queue_info.list[0]].function)();
    Shift_Array((uint8_t *) &Queue_info.list, Queue_info.list_len - 1, 0);
    Queue_info.list_len--;

    if (Queue_info.list_len > 0) {

      for (int16_t i = 0; i < Queue_info.list_len; i++) {
        if (Queue[Queue_info.list[i]].time_period > 0) {
          ROUTINE_TIMER->ARR = Queue[Queue_info.list[i]].time_period;
          Queue[Queue_info.list[i]].time_period = 0;
          break;
        } else {
          (Queue[Queue_info.list[i]].function);
          Shift_Array((uint8_t *) &Queue_info.list, Queue_info.list_len - 1, 0);
          Queue_info.list_len--;
          i--;
        }
      }
    } else {

      ROUTINE_TIMER->ARR = 0;
    }
    //
    //	if(Queue_info.list_len<0){
    //		Debug("Queue length erro");
    //		ROUTINE_TIMER->ARR=0;
    //		Queue_info.list_len=0;
    //	}
    //
    //	if(Queue_info.list_len=0){
    //			Debug("Queue empty");
    //
    //			ROUTINE_TIMER->ARR=0;
    //		}


  }

}


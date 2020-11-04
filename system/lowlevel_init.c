
#include <inc/stm32f4xx_it.h>
#include "lowlevel_init.h"
#include "inc/stm32f4xx.h"
#include "inc/stm32f4xx_conf.h"

RCC_ClocksTypeDef RCC_Clocks;


#include "system/systemtime.h"
#include "iwdg.h"

uint8_t lowlevel_init() {
  DBGMCU_Config(DBGMCU_SLEEP | DBGMCU_STOP | DBGMCU_STANDBY, ENABLE);
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
  DelayMs(5);

  return 1;
}

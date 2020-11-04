//
// Created by ysbf on 8/26/19.
//

#ifndef F407_IWDG_H
#define F407_IWDG_H

#include "stm32f4xx_conf.h"

void IWDG_Feed(void);
void IWDG_Config(uint8_t prv ,uint16_t rlv);
void IWDG_Init();


#endif //F407_IWDG_H

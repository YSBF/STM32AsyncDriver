//
// Created by ysbf on 6/20/20.
//

#include "led.h"
#include <inc/stm32f4xx_it.h>

#include <utility>
#include "inc/stm32f4xx.h"
#include "inc/stm32f4xx_conf.h"

void LED::init(GPIO_TypeDef *GPIO, uint8_t GPIO_Pin, std::string &&color) {
  info.GPIO = GPIO;
  info.GPIO_Pin_Num = GPIO_Pin;
  info.color = std::move(color);
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(1u << ((uint32_t)info.GPIO - AHB1PERIPH_BASE) / 0x0400, ENABLE);
  GPIO_InitStructure.GPIO_Pin = 1u<<info.GPIO_Pin_Num;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(info.GPIO, &GPIO_InitStructure);

}
void LED::on() {

  info.GPIO->ODR &= ~(1u<<info.GPIO_Pin_Num);

}
void LED::off() {

  info.GPIO->ODR |= 1u<<info.GPIO_Pin_Num;
}
void LED::toggle() {

  if ( info.GPIO->ODR & 1u<<info.GPIO_Pin_Num) {
    info.GPIO->ODR &= ~(1u<<info.GPIO_Pin_Num);
  } else {
    info.GPIO->ODR |= 1u<<info.GPIO_Pin_Num;
  }

}

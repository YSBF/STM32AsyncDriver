//
// Created by ysbf on 8/26/19.
//

#include "wwdg.h"
void WWDG_Init(){

    NVIC_InitTypeDef  NVIC_InitStructure;

    //使能wwdg时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);

   // WWDG_CNT=0x7F&WWDG_CNT;   //初始化WWDG_CNT.


    //设置看门狗时钟  1281Hz = (PCLK1 (42MHz)/4096)/8 = 1281 Hz (~780 us) 
    //指定WWDG预分频器
    WWDG_SetPrescaler(WWDG_Prescaler_8);//Prescaler：计数器时钟

    //设置窗口值为80; WWDG计数器只有在计数器时才能刷新低于80（大于64），否则会产生复位
    WWDG_SetWindowValue(0x60);

    //设置重载计数值，也就是喂狗
    WWDG_Enable(0x7f);


    // 初始化中断
    NVIC_InitStructure.NVIC_IRQChannel = WWDG_IRQn; // EXTI0的中断向量号
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; // 响应优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // 使能中断
    NVIC_Init(&NVIC_InitStructure);

    // 清空提前唤醒中断的标志位
    WWDG_ClearFlag();

    // 使能提前唤醒中断
    WWDG_EnableIT();
    WWDG_Enable(0x60);

}



void WWDG_IRQHandler(void)
{
    if(WWDG_GetFlagStatus() == SET)
    {
        // 喂狗
        WWDG_SetCounter(127);
        //WWDG_SetCounter(WWDG_CNT);
        //清空提前唤醒中断的标志位
        WWDG_ClearFlag();
    }
}

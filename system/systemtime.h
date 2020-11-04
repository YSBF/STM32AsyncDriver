/*
 * systemtime.h
 *
 *  Created on: Feb 23, 2019
 *      Author: ysbf
 */

#ifndef INC_SYSTEMTIME_H_
#define INC_SYSTEMTIME_H_



#ifdef __cplusplus
extern "C"
{
#endif
#include "stdint.h"

  void Delay(volatile uint32_t nTime);
static volatile uint32_t uwTimingDelay;
uint32_t millis(void);
void TimingDelay_Decrement(void);
uint32_t DWT_Delay_Init(void);
void DelayMs(uint16_t nms);
// __STATIC_INLINE void DelayUs(uint32_t us);
uint32_t micros(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_SYSTEMTIME_H_ */

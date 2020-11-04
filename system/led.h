

#ifndef MYECL_LIB_DRIVERS_STM32_OUTPUT_LED_H
#define MYECL_LIB_DRIVERS_STM32_OUTPUT_LED_H

#include <string>
#include "stm32f4xx.h"

class LED {
public:
  struct INFO {
    GPIO_TypeDef *GPIO;
    uint16_t GPIO_Pin_Num;
    std::string color;
  };

  LED() = default;

  void init(GPIO_TypeDef *GPIO, uint8_t GPIO_Pin, std::string &&color);

  void on();
  void off();
  void toggle();

private:

  INFO info{};

};

#endif //MYECL_LIB_DRIVERS_STM32_OUTPUT_LED_H

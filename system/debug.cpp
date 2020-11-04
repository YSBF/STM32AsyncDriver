//
// Created by ysbf on 8/24/20.
//

#include "debug.h"
#include "connectivity/UART.h"
//#include "ringbuffer.h"


 UARTDriver uart6_driver;
char string[300];
void my_printf(const char *fmt, ...) // custom printf() function
{



  static bool is_first_time= true;
  if(is_first_time) {

//    UARTDriver::UART_INFO info{
//        1,
//        115200,
//        true,
//        true,
//        UARTDriver::UART_MOOD::T_RX,
//        UARTDriver::GPIO::A,
//        UARTDriver::GPIO::A,
//        9,10,
//    };


    UARTDriver::UART_INFO info{
        6,
        115200,
        true,
        true,
        UARTDriver::UART_MOOD::T_RX,
        UARTDriver::GPIO::C,
        UARTDriver::GPIO::C,
        6,7,
    };
//    UARTDriver::UART_INFO info{
//        4,
//        115200,
//        true,
//        true,
//        UARTDriver::UART_MOOD::T_RX,
//        UARTDriver::GPIO::A,
//        UARTDriver::GPIO::A,
//        0,1,
//    };
//    UART_INFO info{
//        4,
//        115200,
//        true,
//        true,
//        T_RX,
//        A,
//        A,
//        0,1,
//    };
    uart6_driver.init(info);
    is_first_time= false;
  }
  static uint32_t time_last;
  //  Debug("Tx TimeInterval:%d", micros() - time_last);
  //  time_last = micros();

  va_list argp;
  va_start(argp, fmt);


  int len = vsnprintf(string, sizeof(string), fmt, argp);


  if (len > 0) {


//    string[len] = '\r';
//    string[len + 1] = '\n';

    uart6_driver.dma_write_bytes((uint8_t *) &string, len );

  }
  va_end(argp);
}

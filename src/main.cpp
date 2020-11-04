#ifdef __cplusplus
extern "C" {
#endif
#include "system/lowlevel_init.h"
#include "system/systemtime.h"

#ifdef __cplusplus

}
#endif
#include <connectivity/UART.h>

#include <utility>
#include <src/l3gd20/l3gd20.h>

void recieve(UARTDriver &uart_driver) {
  static char a[10];

  uart_driver.dma_async_read((uint8_t *) a, 10, [&](uint16_t bytes_transferred) {
    uart_driver.dma_async_write((uint8_t *) a, bytes_transferred, [&](uint16_t bytes_transferred) {
      recieve(uart_driver);
    });
  });
}

void send(UARTDriver &uart_driver) {
  char *a = "hello\n";
  //  uart_driver.dma_write_bytes((uint8_t *) a, 6);
  uart_driver.dma_async_write((uint8_t *) a, 6, [&](uint16_t bytes_transferred) {
    send(uart_driver);

  });
}
#include "bno055/BNO055.h"
#include "functional"
#include "connectivity/SPI.h"
#include "mpu9250/mpu9250.h"
#include "system/debug.h"

extern UARTDriver uart6_driver;
#include "system/led.h"

struct UART1_Operation {

  static UARTDriver uart1_driver;
  UART1_Operation() {

    UARTDriver::UART_INFO uart1_info{
        1,
        115200,
        true,
        true,
        UARTDriver::UART_MOOD::T_RX,
        UARTDriver::GPIO::A,
        UARTDriver::GPIO::A,
        9, 10,
    };
    uart1_driver.init(uart1_info);
  }

  static uint8_t uart1_read_bytes(const uint8_t *buffer, uint16_t len) {
  }
  //  std::function<uint8_t(uint8_t *buffer, const uint8_t len)> read_bytes = std::bind(&UARTDriver::dma_read_bytes, uart1_driver, std::placeholders::_1, std::placeholders::_2);
  //  std::function<uint8_t(const uint8_t *buffer, const uint8_t len)> write_bytes = std::bind(&UARTDriver::dma_write_bytes, uart1_driver, std::placeholders::_1, std::placeholders::_2);
  //  std::function<void(uint32_t msec)> delay = DelayMs;

  uint8_t UARTDriver::*pRead(const uint8_t *buffer, uint16_t len);

};

/*struct spi1_operation {

  spi1_operation() = default;

  static void dma_read_bytes(const uint8_t *buffer, uint16_t len) {
    spi1Dirver.dma_read_bytes(buffer, len);
  }
  static void dma_write_bytes(const uint8_t *buffer, uint16_t len) {
    spi1Dirver.dma_write_bytes(buffer, len);
  }
  static void dma_async_read(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t bytes_transferred)> callback_fun){
    spi1Dirver.dma_async_read(buffer,  len, std::move(callback_fun));
  }

  static void  dma_async_write(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t bytes_transferred)> callback_fun){
    spi1Dirver.dma_async_write(buffer,  len, std::move(callback_fun)) ;
  }

private:
  static SPIDirver spi1Dirver;
  static SPIDirver::SPI_INIT_INFO spi1_info

};
SPIDirver::SPI_INIT_INFO spi1_operation::spi1_info{
    1,
    20,
    true,
    true,
    SPIDirver::GPIO::A,
    SPIDirver::GPIO::A,
    SPIDirver::GPIO::A,
    SPIDirver::GPIO::A,
    5, 6, 7, 4,
};*/
//SPIDirver spi1_operation::spi1Dirver(spi1_info);
#define INTERGER_BOARD
int main() {
  lowlevel_init();

  LED led_green, led_blue;
#ifdef INTERGER_BOARD
  led_green.init(GPIOE, 2, std::string("green"));
  led_blue.init(GPIOE, 3, std::string("blue"));
  led_green.on();
  led_blue.off();;

#else
  led_blue.init(GPIOB, 9, std::string("blue"));
#endif
  UARTDriver::UART_INFO uart1_info{
      1,
      115200,
      true,
      true,
      UARTDriver::UART_MOOD::T_RX,
      UARTDriver::GPIO::A,
      UARTDriver::GPIO::A,
      9, 10,
  };
  SPIDirver::SPI_INIT_INFO spi1_info{
      1,
      20,
      true,
      true,
      SPIDirver::GPIO::A,
      SPIDirver::GPIO::A,
      SPIDirver::GPIO::A,
      SPIDirver::GPIO::A,
      5, 6, 7, 4,
  };

  SPIDirver::SPI_INIT_INFO spi2_info{
      2,
      20,
      true,
      true,
      SPIDirver::GPIO::B,
      SPIDirver::GPIO::B,
      SPIDirver::GPIO::B,
      SPIDirver::GPIO::B,
      13, 14, 15, 12,
  };
  UARTDriver uart1_driver(uart1_info);
  SPIDirver spi1_driver(spi1_info);
  SPIDirver spi2_driver(spi2_info);
//
//  std::function<void(uint32_t msec)> delay = DelayMs;
//  std::function<uint8_t(uint8_t *buffer, const uint8_t len)> uart1_read_bytes = std::bind(&UARTDriver::dma_read_bytes, uart1_driver, std::placeholders::_1, std::placeholders::_2);
//  std::function<uint8_t(const uint8_t *buffer, const uint8_t len)> uart1_write_bytes = std::bind(&UARTDriver::dma_write_bytes, uart1_driver, std::placeholders::_1, std::placeholders::_2);
//  std::function<void(const uint8_t *buffer, uint16_t len)> spi1_read_bytes = std::bind(&SPIDirver::dma_read_bytes, spi1_driver, std::placeholders::_1, std::placeholders::_2);
//  std::function<void(const uint8_t *buffer, uint16_t len)> spi1_write_bytes = std::bind(&SPIDirver::dma_write_bytes, spi1_driver, std::placeholders::_1, std::placeholders::_2);
//  std::function<void(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t bytes_transferred)> callback_fun)> spi1_async_read_bytes = std::bind(&SPIDirver::dma_async_read, spi1_driver, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
//  std::function<void(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t bytes_transferred)> callback_fun)> spi1_async_write_bytes = std::bind(&SPIDirver::dma_async_write, spi1_driver, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
//
//  std::function<void(const uint8_t *buffer, uint16_t len)> spi2_read_bytes = std::bind(&SPIDirver::read_bytes, spi2_driver, std::placeholders::_1, std::placeholders::_2);
//  std::function<void(const uint8_t *buffer, uint16_t len)> spi2_write_bytes = std::bind(&SPIDirver::write_bytes, spi2_driver, std::placeholders::_1, std::placeholders::_2);
//  std::function<void(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t bytes_transferred)> callback_fun)> spi2_async_read_bytes = std::bind(&SPIDirver::dma_async_read, spi2_driver, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
//  std::function<void(const uint8_t *buffer, uint16_t len, std::function<void(uint16_t bytes_transferred)> callback_fun)> spi2_async_write_bytes = std::bind(&SPIDirver::dma_async_write, spi2_driver, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
//  //
//  MPU9250 mpu9250(spi1_read_bytes, spi1_write_bytes, spi1_async_read_bytes, spi1_async_write_bytes, DelayMs);
//  MPU9250 mpu9250(spi2_read_bytes, spi2_write_bytes, spi2_async_read_bytes, spi2_async_write_bytes, DelayMs);
//  mpu9250.init();

  L3GD20 l3gd20(spi2_read_bytes, spi2_write_bytes, spi2_async_read_bytes, spi2_async_write_bytes, DelayMs);
  l3gd20.init(L3GD20::gyroRange_t::GYRO_RANGE_2000DPS);


  //  BNO055 bno055(read_bytes, write_bytes, delay);
  //  bno055.begin(BNO055::bno055_opmode_t::OPERATION_MODE_NDOF);

  //  my_printf("hello\r\n");
#pragma clang diagnostic push

#pragma ide diagnostic ignored "EndlessLoop"
  //  recieve(uart6_driver);
  while (true) {

    static uint8_t buffer[2];
    buffer[0] = MPU9250_WHO_AM_I | 0x80;

    static float acc[3], gyro[3], mag[3];
    static uint8_t data[22];

    static float q[4];
//    mpu9250.async_get_9axis_data(acc, gyro, mag, [&]() {
//      my_printf("ACC:%f %f %f   GYRO:%f %f %f   MAG:%f %f %f \n", acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2]);
//    });

    float *q_tmp;
    //    q_tmp = bno055.getQuat();
    q[0] = q_tmp[0];
    q[1] = q_tmp[1];
    q[2] = q_tmp[2];
    q[3] = q_tmp[3];
    DelayMs(500);
#ifdef INTERGER_BOARD
    led_green.toggle();
#endif
    led_blue.toggle();
    l3gd20.get_gyro_data(gyro);
    my_printf("ACC:%f %f %f   GYRO:%f %f %f   MAG:%f %f %f \nQ:%f %f %f %f\n", acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2], q[0], q[1], q[2], q[3]);
    //my_printf("hello\n");
  }
#pragma clang diagnostic pop

  return 0;
}












//
//
//
//spi_read =<std::_Maybe_unary_or_binary_function<void, unsigned char const*, unsigned short>> =
//    {<std::binary_function<unsigned char const*, unsigned short, void>> = {<No data fields>}, <No data fields>},
//
//<std::_Function_base> = {
//    static _M_max_size = 8,
//    static _M_max_align = 4,
//    _M_functor = {_M_unused = {_M_object = 0x20000638,_M_const_object = 0x20000638, _M_function_pointer = 0x20000638, _M_member_pointer = (void (std::_Undefined_class::*)(std::_Undefined_class * const)) 0x20000638},
//      _M_pod_data = "8\006\000 \000\000\000"},
//
//      _M_manager = 0x8002049 <std::_Function_base::_Base_manager<std::_Bind<void (SPI1Dirver::*(SPI1Dirver, std::_Placeholder<1>, std::_Placeholder<2>))(unsigned char const*, unsigned short)> >::_M_manager(std::_Any_data&, std::_Any_data const&, std::_Manager_operation)>},
//_M_invoker = 0x8002011 <std::_Function_handler<void (unsigned char const*, unsigned short), std::_Bind<void (SPI1Dirver::*(SPI1Dirver, std::_Placeholder<1>, std::_Placeholder<2>))(unsigned char const*, unsigned short)> >::_M_invoke(std::_Any_data const&, unsigned char const*&&, unsigned short&&)>,
//
//
//
//    spi_write = <std::_Maybe_unary_or_binary_function<void, unsigned char const*, unsigned short>> = {<std::binary_function<unsigned char const*, unsigned short, void>> = {<No data fields>}, <No data fields>}, <std::_Function_base> = {static _M_max_size = 8, static _M_max_align = 4, _M_functor = {_M_unused = {_M_object = 0x20000690,
//      _M_const_object = 0x20000690, _M_function_pointer = 0x20000690, _M_member_pointer = &virtual table offset 536872592, this adjustment -570691856},
//      _M_pod_data = "\220\006\000 \341\335", <incomplete sequence \367\273>},
//      _M_manager = 0x8002049 <std::_Function_base::_Base_manager<std::_Bind<void (SPI1Dirver::*(SPI1Dirver, std::_Placeholder<1>, std::_Placeholder<2>))(unsigned char const*, unsigned short)> >::_M_manager(std::_Any_data&, std::_Any_data const&, std::_Manager_operation)>},
//_M_invoker = 0x8002011 <std::_Function_handler<void (unsigned char const*, unsigned short), std::_Bind<void (SPI1Dirver::*(SPI1Dirver, std::_Placeholder<1>, std::_Placeholder<2>))(unsigned char const*, unsigned short)> >::_M_invoke(std::_Any_data const&, unsigned char const*&&, unsigned short&&)>,
//
//
//    spi_async_read = {<std::_Maybe_unary_or_binary_function<void, unsigned char const*, short unsigned int, std::function<void(short unsigned int)> >> = {<No data fields>}, <std::_Function_base> = {static _M_max_size = 8, static _M_max_align = 4, _M_functor = {_M_unused = {_M_object = 0x200006f0, _M_const_object = 0x200006f0,
//      _M_function_pointer = 0x200006f0, _M_member_pointer = &virtual table offset 536872688, this adjustment -538444115}, _M_pod_data = "\360\006\000 [\375Ͽ"},
//      _M_manager = 0x80021b1 <std::_Function_base::_Base_manager<std::_Bind<void (SPI1Dirver::*(SPI1Dirver, std::_Placeholder<1>, std::_Placeholder<2>, std::_Placeholder<3>))(unsigned char const*, unsigned short, std::function<void (unsigned short)>)> >::_M_manager(std::_Any_data&, std::_Any_data const&, std::_Manager_operation)>},
//_M_invoker = 0x800216f <std::_Function_handler<void (unsigned char const*, unsigned short, std::function<void (unsigned short)>), std::_Bind<void (SPI1Dirver::*(SPI1Dirver, std::_Placeholder<1>, std::_Placeholder<2>, std::_Placeholder<3>))(unsigned char const*, unsigned short, std::function<void (unsigned short)>)> >::_M_invoke(std::_Any_data const&, unsigned char const*&&, unsigned short&&, std::function<void (unsigned short)>&&)>},
//
//
//
//
//    spi_async_write = {<std::_Maybe_unary_or_binary_function<void, unsigned char const*, short unsigned int, std::function<void(short unsigned int)> >> = {<No data fields>}, <std::_Function_base> = {static _M_max_size = 8, static _M_max_align = 4, _M_functor = {_M_unused = {_M_object = 0x20000748, _M_const_object = 0x20000748,
//      _M_function_pointer = 0x20000748, _M_member_pointer = (void (std::_Undefined_class::*)(std::_Undefined_class * const)) 0x20000748, this adjustment 268500964},
//      _M_pod_data = "H\a\000 \310\377\001 "},
//      _M_manager = 0x80021b1 <std::_Function_base::_Base_manager<std::_Bind<void (SPI1Dirver::*(SPI1Dirver, std::_Placeholder<1>, std::_Placeholder<2>, std::_Placeholder<3>))(unsigned char const*, unsigned short, std::function<void (unsigned short)>)> >::_M_manager(std::_Any_data&, std::_Any_data const&, std::_Manager_operation)>},
//_M_invoker = 0x800216f <std::_Function_handler<void (unsigned char const*, unsigned short, std::function<void (unsigned short)>), std::_Bind<void (SPI1Dirver::*(SPI1Dirver, std::_Placeholder<1>, std::_Placeholder<2>, std::_Placeholder<3>))(unsigned char const*, unsigned short, std::function<void (unsigned short)>)> >::_M_invoke(std::_Any_data const&, unsigned char const*&&, unsigned short&&, std::function<void (unsigned short)>&&)>},
//
//
//    delay = {<std::_Maybe_unary_or_binary_function<void, unsigned short>> = {<std::unary_function<unsigned short, void>> = {<No data fields>}, <No data fields>}, <std::_Function_base> = {static _M_max_size = 8, static _M_max_align = 4, _M_functor = {_M_unused = {_M_object = 0x8001629 <DelayMs>, _M_const_object = 0x8001629 <DelayMs>,
//      _M_function_pointer = 0x8001629 <DelayMs>, _M_member_pointer = &virtual table offset 134223401, this adjustment 67121307}, _M_pod_data = ")\026\000\b7a\000\b"},
//      _M_manager = 0x80022d5 <std::_Function_base::_Base_manager<void (*)(unsigned short)>::_M_manager(std::_Any_data&, std::_Any_data const&, std::_Manager_operation)>},
//_M_invoker = 0x80022a9 <std::_Function_handler<void (unsigned short), void (*)(unsigned short)>::_M_invoke(std::_Any_data const&, unsigned short&&)>},
//
//
//
//
//    spi_buffer = "\000\355w\377\b;R\v\360\001`\000\221\000\035", '\000' <repeats 17 times>}
//
//














































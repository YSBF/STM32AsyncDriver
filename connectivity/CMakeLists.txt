




#file(GLOB_RECURSE SOURCES *.cpp)

include_directories(${CMAKE_SOURCE_DIR}/inc)

set(SOURCES
#        UART4.cpp
        SPI.cpp
        UART.cpp
        ADC.cpp
        )

add_library(connectivity ${SOURCES} )












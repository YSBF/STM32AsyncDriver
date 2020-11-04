cmake_minimum_required(VERSION 3.16)
enable_language(ASM C CXX)

project(BNO055Driver)

include(toolchain_set.cmake)
set(CMAKE_VERBOSE_MAKEFILE ON)
SetCrossCompiler()


set(CMAKE_CXX_STANDARD 17)

file(GLOB_RECURSE SOURCES system/*.c  system/*.cpp system/*.s
        )

add_definitions(
        -DSTM32F40_41xxx
        -DUSE_STDPERIPH_DRIVER
        -DHSE_VALUE=25000000
        -DUSE_USB_OTG_FS
        -DUSE_FULL_ASSERT
        -D__VFP_FP__
        -DBNO055_API
)


include_directories(
        inc
        .
        lib/etl
        lib/STM32_USB_Device_Library/inc
        lib/STM32F4xx_StdPeriph_Driver/inc
)
add_subdirectory(connectivity)
add_subdirectory(lib/STM32F4xx_StdPeriph_Driver)
#add_subdirectory(lib/fatfs)
add_subdirectory(src/bno055)
#add_subdirectory(src/fatfs_example)
#add_subdirectory(src/sdio_fs)

add_executable(${PROJECT_NAME}.elf
        ${SOURCES}
        src/main.cpp
        src/bno055/future.cpp
        src/bno055/BNO055.cpp

        src/l3gd20/l3gd20.cpp
        src/mpu9250/mpu9250.cpp  system/debug.cpp system/debug.h)



target_link_libraries(${PROJECT_NAME}.elf
#        ${CMAKE_SOURCE_DIR}/lib/STM32F4xx_StdPeriph_Driver/libSTM32F4xx_StdPeriph_Driver.a
        connectivity
        std_periph_driver
#        fatfs
#        fatfs_example
        bno055
        )



add_custom_target (${PROJECT_NAME}.bin
        DEPENDS ${PROJECT_NAME}.elf
        COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
        )








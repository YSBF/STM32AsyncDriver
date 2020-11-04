




file(GLOB_RECURSE SOURCES src/*.c)
include_directories(${CMAKE_SOURCE_DIR}/inc)
add_library(std_periph_driver ${SOURCES})












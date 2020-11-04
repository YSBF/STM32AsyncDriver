

file(GLOB SOURCES *.cpp *.c)
add_library(fatfs
        ${SOURCES}
        )


file(GLOB SOURCES *.cpp *.c)
add_library(sdio_fs
        ${SOURCES}
        )
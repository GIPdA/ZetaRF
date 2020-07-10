cmake_minimum_required(VERSION 3.6)
project(zetarf)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)

include(CheckIncludeFiles)
CHECK_INCLUDE_FILES("wiringPi.h;wiringPiSPI.h" WIRINGPI LANGUAGE CXX)
if(${WIRINGPI})
    add_compile_definitions(WIRINGPI)
endif()

add_subdirectory(src)
add_subdirectory(examples/rpi/SimpleRx)
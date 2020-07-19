cmake_minimum_required(VERSION 3.6)
project(zetarf)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

# Check that dependencies are available

include(CheckIncludeFiles)
CHECK_INCLUDE_FILES("wiringPi.h;wiringPiSPI.h" WIRINGPI LANGUAGE CXX)
if(${WIRINGPI})
    add_compile_definitions(WIRINGPI)
else()
    message(FATAL_ERROR "WiringPi library could not be found.")
endif()

# Build ZetaRF library

file(GLOB_RECURSE CONFIG_SOURCES "src/configs/*.cpp")

add_library(${PROJECT_NAME} STATIC src/ZetaRf.cpp ${CONFIG_SOURCES})
target_link_libraries(${PROJECT_NAME} wiringPi)

# Build Raspberry Pi examples

add_subdirectory(examples_raspberrypi)
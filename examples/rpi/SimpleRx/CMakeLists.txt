cmake_minimum_required(VERSION 3.6)
project(simplerx)

include_directories(../../../src)

add_executable(${PROJECT_NAME} SimpleRx.cpp)
target_link_libraries(${PROJECT_NAME} wiringPi)
target_link_libraries(${PROJECT_NAME} zetarf)
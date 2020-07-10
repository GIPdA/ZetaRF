cmake_minimum_required(VERSION 3.6)
project(zetarf)

file(GLOB_RECURSE CONFIG_SOURCES "configs/*.h" "configs/*.cpp")

add_library(${PROJECT_NAME} STATIC ZetaRF.cpp ${CONFIG_SOURCES})
target_link_libraries(${PROJECT_NAME} wiringPi)
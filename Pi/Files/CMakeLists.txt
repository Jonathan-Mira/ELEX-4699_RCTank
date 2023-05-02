cmake_minimum_required(VERSION 3.18.4)
project(RCTank_S)

find_path(PIGPIO_INCLUDE_DIR pigpio.h)
include_directories(${PIGPIO_INCLUDE_DIR})

find_package (OpenCV 4.0.0 REQUIRED)
include_directories ("/usr/lib/opencv4/")

find_library(PIGPIO_LIBRARY pigpio)

set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")

set(CMAKE_CXX_STANDARD 11)

list(APPEND CMAKE_CXX_FLAGS "-pthread -std=c++11 ${CMAKE_CXX_FLAGS}")

add_executable(RCTank_S main.cpp server.cpp)
target_link_libraries(RCTank_S ${PIGPIO_LIBRARY} ${OpenCV_LIBS})

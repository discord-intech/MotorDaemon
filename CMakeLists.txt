cmake_minimum_required(VERSION 2.4)
project(MotorDaemon)

SET(CMAKE_SYSTEM_NAME Linux)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -I/usr/include/glib-2.0 -I/usr/lib/glib-2.0/include -std=c++11 -lblacklib -lglib-2.0")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m64 -I/usr/include/glib-2.0 -I/usr/lib/glib-2.0/include -lblacklib -lglib-2.0")

set(SOURCE_FILES main.cpp Motor.cpp Motor.hpp MotionController.cpp MotionController.hpp Odometry.cpp Odometry.hpp)
add_executable(MotorDaemon ${SOURCE_FILES})
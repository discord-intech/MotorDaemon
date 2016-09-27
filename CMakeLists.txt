cmake_minimum_required(VERSION 2.4)
project(MotorDaemon)

#-I/usr/include/glib-2.0 -I/usr/lib/glib-2.0/include

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -lblacklib -lglib-2.0")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -lblacklib -lglib-2.0")

set(SOURCE_FILES main.cpp Motor.cpp Motor.hpp MotionController.cpp MotionController.hpp Odometry.cpp Odometry.hpp)
add_executable(MotorDaemon ${SOURCE_FILES})
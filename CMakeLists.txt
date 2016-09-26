cmake_minimum_required(VERSION 3.6)
project(MotorDaemon)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/usr/include/glib-2.0 -std=c++11 -lglib-2.0")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}" -I/usr/include/glib-2.0 -lglib-2.0)

set(SOURCE_FILES main.cpp Motor.cpp Motor.hpp MotionController.cpp MotionController.hpp Odometry.cpp Odometry.hpp)
add_executable(MotorDaemon ${SOURCE_FILES})
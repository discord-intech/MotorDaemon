cmake_minimum_required(VERSION 2.4)
project(MotorDaemon)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -lblacklib -lpthread")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -lblacklib -lpthread")

set(SOURCE_FILES main.cpp Motor.cpp Motor.hpp MotionController.cpp MotionController.hpp Odometry.cpp Odometry.hpp)
add_executable(MotorDaemon ${SOURCE_FILES})
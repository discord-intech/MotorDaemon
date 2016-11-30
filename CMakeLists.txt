cmake_minimum_required(VERSION 2.4)
project(MotorDaemon)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -lblacklib -lpthread")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -lblacklib -lpthread")

include_directories(include)

file(GLOB SOURCES "src/*.cpp")

add_executable(MotorDaemon ${SOURCES})
cmake_minimum_required(VERSION 4.2)
project(pathsgrid)

set(CMAKE_CXX_STANDARD 20)

add_executable(pathsgrid main.cpp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

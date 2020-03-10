cmake_minimum_required(VERSION 3.15)
project(.)

set(CMAKE_CXX_STANDARD 14)

add_executable(. main.cpp geometry.h model.h tgaimage.h model.cpp tgaimage.cpp)
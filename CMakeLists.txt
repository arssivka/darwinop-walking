cmake_minimum_required(VERSION 3.9)
project(darwinop_gait)

set(CMAKE_CXX_STANDARD 98)

add_library(darwinop_gait Walking.cpp Walking.h)
target_include_directories()
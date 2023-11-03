# CMakeLists.txt

cmake_minimum_required(VERSION 3.0)
project(hightorque_biped_sdk_core)
SET(CMAKE_CXX_FLAGS "-std=c++14")

add_library(hightorque_biped_sdk_core STATIC ${CMAKE_CURRENT_LIST_DIR}/../../../src/Livelybot_Driver.cpp)
target_include_directories(hightorque_biped_sdk_core PUBLIC ${CMAKE_CURRENT_LIST_DIR}/../../../include)

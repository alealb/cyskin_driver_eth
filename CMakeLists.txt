cmake_minimum_required(VERSION 3.8)
project(cyskin_driver_eth)

set(CMAKE_CXX_STANDARD 17)

find_package(skin REQUIRED)

if(NOT skin_FOUND)
  message(FATAL_ERROR "Error: Skin middleware not found")
else()
  message(STATUS "Skin middleware found")
endif()


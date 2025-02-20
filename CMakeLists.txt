cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME cyskin_driver_eth)
project(cyskin_driver_eth)

set(CMAKE_CXX_STANDARD 17)

find_package(skin REQUIRED)

if(NOT skin_FOUND)
  message(FATAL_ERROR "Error: Skin middleware not found")
else()
  message(STATUS "Skin middleware found")
endif()

include_directories(include)

# Include SOEM headers
include_directories(/home/SOEM/install/include/soem)

# Add SOEM library path
link_directories(/home/SOEM/install/lib)

# Suppress the -Wpmf-conversions warning
add_compile_options(-Wno-pmf-conversions)

add_executable(
  ${PROJECT_NAME} src/main.cpp
  src/ecatHandler.cpp
  )

target_link_libraries(${example} ${PROJECT_NAME} -lpthread -lrt skin::skin -lsoem)


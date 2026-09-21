cmake_minimum_required(VERSION 3.22)
project(drone_sil LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../firmware/main/flight
                 ${CMAKE_CURRENT_BINARY_DIR}/flight_build)

add_executable(sil_runner
  main_sil.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/main/hal/sil_wire.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/main/hal/hal_sil.cpp)

target_include_directories(sil_runner PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/main)
target_compile_options(sil_runner PRIVATE -Wall -Wextra -Werror -fno-exceptions -fno-rtti)
target_link_libraries(sil_runner PRIVATE flight m)

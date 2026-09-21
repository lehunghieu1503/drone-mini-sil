cmake_minimum_required(VERSION 3.22)

# Host build: `cmake -S firmware -B build/host -DDRONE_HOST_LIB=ON`.
# Must work on a machine with no ESP-IDF in the environment.
if(DRONE_HOST_LIB)
  project(drone_host LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)

  add_subdirectory(main/flight)

  add_library(drone_flight SHARED
    main/hal/sil_wire.cpp
    main/hal/hal_mock.cpp
    host/host_bindings.cpp)
  target_include_directories(drone_flight PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/main)
  target_compile_options(drone_flight PRIVATE
    -Wall -Wextra -Werror -fno-exceptions -fno-rtti)
  target_link_libraries(drone_flight PRIVATE flight m)
else()
  # Chip build: ESP-IDF is the top-level project (P9). Do not nest it.
  include($ENV{IDF_PATH}/tools/cmake/project.cmake)
  project(drone_fw)
endif()

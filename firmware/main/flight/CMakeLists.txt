# Plain static library shared by the host build and the ESP-IDF component.
# IDF appends its own -std=gnu++26; we pin gnu++20 explicitly (see phase-01).
add_library(flight STATIC
  control_loop.cpp
  mixer.cpp
  pid.cpp
  estimator.cpp
  failsafe.cpp
  led.cpp
  rc_parse.cpp
  output.cpp)

target_include_directories(flight
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_compile_features(flight PUBLIC cxx_std_20)
target_compile_options(flight PRIVATE
  -Wall -Wextra -Werror -fno-exceptions -fno-rtti)
target_link_libraries(flight PUBLIC m)

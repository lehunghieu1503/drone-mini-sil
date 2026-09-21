idf_component_register(SRCS "app_main.cpp"
                       INCLUDE_DIRS ".")

# gnu++20 must be pinned at component level: IDF appends -std=gnu++26 after
# other flags, and GCC honors the last -std= on the command line (phase-01).
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++20)

# Link glue: without this the flight symbols are undefined in app_main.
add_subdirectory(flight)
target_link_libraries(${COMPONENT_LIB} INTERFACE flight)

# CONFIG_DRONE_HIL is dropped silently if Kconfig.projbuild is missing, which
# would compile out the whole HIL transport. Fail loudly instead (RT#11).
if(CONFIG_DRONE_HIL)
  target_compile_definitions(${COMPONENT_LIB} PUBLIC CONFIG_DRONE_HIL=1)
endif()

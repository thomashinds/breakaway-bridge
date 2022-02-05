idf_component_register(SRCS "event-handler.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES ble-trainer)

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++17)
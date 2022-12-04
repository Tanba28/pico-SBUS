add_library(pico-SBUS sbus.cpp)

target_link_libraries(pico-SBUS pico_stdlib hardware_irq hardware_dma pico-FreeRTOS)

target_include_directories(pico-SBUS
    PUBLIC ${PROJECT_SOURCE_DIR}/src
    )


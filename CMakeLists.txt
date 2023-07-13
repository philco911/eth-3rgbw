cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(eth_3rgbw)

pico_sdk_init()

add_executable(eth_3rgbw)
pico_generate_pio_header(eth_3rgbw ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)
target_sources(eth_3rgbw PRIVATE main.cpp Ws2812.cpp Status.cpp Eeprom.cpp Ch9120Eth.cpp Gamma.cpp)

target_link_libraries(eth_3rgbw PRIVATE pico_stdlib pico_multicore hardware_pio hardware_pwm hardware_sync hardware_flash hardware_watchdog)
pico_add_extra_outputs(eth_3rgbw)

add_custom_target(eth_3rgbw_datasheet DEPENDS ${CMAKE_CURRENT_LIST_DIR}/generated/ws2812.py)
add_custom_command(OUTPUT ${CMAKE_CURRENT_LIST_DIR}/generated/ws2812.py
        DEPENDS ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio
        COMMAND Pioasm -o python ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio ${CMAKE_CURRENT_LIST_DIR}/generated/ws2812.py
        )
add_dependencies(eth_3rgbw eth_3rgbw_datasheet)

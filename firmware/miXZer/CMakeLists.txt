cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == VS Code Pico SDK boilerplate ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ==================================

set(PICO_BOARD waveshare_rp2040_zero CACHE STRING "Board type")
include(pico_sdk_import.cmake)

project(miXZer C CXX ASM)
pico_sdk_init()

add_executable(miXZer
    miXZer.c         # <- file is at project root
    lib/i2s.c
)

pico_set_program_name(miXZer "miXZer")
pico_set_program_version(miXZer "0.1")

# Generate header from PIO program
pico_generate_pio_header(miXZer ${CMAKE_CURRENT_LIST_DIR}/lib/i2s.pio)

# (Optional) mark PIO source as header-only so IDEs don’t try to compile it
set_source_files_properties(${CMAKE_CURRENT_LIST_DIR}/lib/i2s.pio
    PROPERTIES HEADER_FILE_ONLY TRUE)

# Console I/O
pico_enable_stdio_uart(miXZer 0)
pico_enable_stdio_usb(miXZer 1)

# Includes (add binary dir so i2s.pio.h is found)
target_include_directories(miXZer PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/lib
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Libraries needed
target_link_libraries(miXZer
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_irq
    hardware_clocks
)

pico_add_extra_outputs(miXZer)

# == Channel identification tool (separate firmware, same I2S hardware) ==
add_executable(channel_id
    channel_id.c
    lib/i2s.c
)

pico_set_program_name(channel_id "channel_id")
pico_set_program_version(channel_id "0.1")

pico_generate_pio_header(channel_id ${CMAKE_CURRENT_LIST_DIR}/lib/i2s.pio)
set_source_files_properties(${CMAKE_CURRENT_LIST_DIR}/lib/i2s.pio
    PROPERTIES HEADER_FILE_ONLY TRUE)

pico_enable_stdio_uart(channel_id 0)
pico_enable_stdio_usb(channel_id 1)

target_include_directories(channel_id PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/lib
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(channel_id
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_irq
    hardware_clocks
)

pico_add_extra_outputs(channel_id)

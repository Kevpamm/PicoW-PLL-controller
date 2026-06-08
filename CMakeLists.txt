# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()

set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)

set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK
include(pico_sdk_import.cmake)

project(PLL_BLE C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable
add_executable(PLL_BLE
    PLL_PICOW.c
)

pico_set_program_name(PLL_BLE "PLL_BLE")
pico_set_program_version(PLL_BLE "0.1")

# Enable USB serial, disable UART serial
pico_enable_stdio_uart(PLL_BLE 0)
pico_enable_stdio_usb(PLL_BLE 1)

# Generate BLE_service.h from BLE_service.gatt
pico_btstack_make_gatt_header(
    PLL_BLE
    PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/GATT_service/BLE_service.gatt"
)

# Include folders
target_include_directories(PLL_BLE PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/GATT_service
)

# Link libraries
target_link_libraries(PLL_BLE
    pico_stdlib
    pico_btstack_ble
    pico_btstack_cyw43
    pico_cyw43_arch_none
    hardware_adc
    hardware_base
    pico_bootrom
)

pico_add_extra_outputs(PLL_BLE)
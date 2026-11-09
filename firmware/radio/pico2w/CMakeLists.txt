# Leumpang Radio untuk Raspberry Pi Pico 2 W (atau Pico W: -DPICO_BOARD=pico_w)
#   export PICO_SDK_PATH=/jalur/pico-sdk   (2.1.1+, dengan submodule)
#   cmake -B build -DPICO_BOARD=pico2_w && cmake --build build -j
#   -> build/leumpang_radio.uf2 (tahan BOOTSEL, colok USB, salin .uf2)
cmake_minimum_required(VERSION 3.13)

if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD "pico2_w")
endif()

set(LA_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../..)
set(BLUEPAD32_ROOT ${LA_ROOT}/firmware/radio/third_party/bluepad32)
# BTstack bawaan Pico SDK (lisensi BTstack untuk RP2040/RP2350 dari Raspberry Pi)
set(BTSTACK_ROOT ${PICO_SDK_PATH}/lib/btstack)

include(pico_sdk_import.cmake)
project(leumpang_radio C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(leumpang_radio
    src/main.cpp
    ../common/radio.cpp
    ${LA_ROOT}/core/src/lap.cpp
)
target_include_directories(leumpang_radio PRIVATE
    src
    ../common
    ${LA_ROOT}/core/include
    ${BLUEPAD32_ROOT}/src/components/bluepad32/include)
target_compile_options(leumpang_radio PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions -fno-rtti>)

# btstack_config.h / sdkconfig.h dibutuhkan libbluepad32
include_directories(src)
include_directories(${BTSTACK_ROOT}/3rd-party/bluedroid/encoder/include)
include_directories(${BTSTACK_ROOT}/3rd-party/bluedroid/decoder/include)
include_directories(${BTSTACK_ROOT}/src)

target_link_libraries(leumpang_radio PUBLIC
    pico_stdlib
    hardware_uart
    pico_cyw43_arch_none
    pico_btstack_classic
    pico_btstack_cyw43
    bluepad32
)
add_subdirectory(${BLUEPAD32_ROOT}/src/components/bluepad32 libbluepad32)

pico_enable_stdio_usb(leumpang_radio 1)
pico_enable_stdio_uart(leumpang_radio 0)
pico_add_extra_outputs(leumpang_radio)

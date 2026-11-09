# Leumpang Radio untuk ESP32 klasik (DevKit WROOM-32, ESP32-PICO-D4/PICO-KIT) - Bluepad32 4.2.0
#   idf.py set-target esp32 && idf.py build flash monitor      (ESP-IDF v5.3 / v5.4)
# Hanya ESP32 klasik yang punya Bluetooth Classic (wajib untuk DualShock 4).
cmake_minimum_required(VERSION 3.16)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(LA_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../..)
set(BLUEPAD32_ROOT ${CMAKE_CURRENT_LIST_DIR}/../third_party/bluepad32)
set(BTSTACK_ROOT ${BLUEPAD32_ROOT}/external/btstack)

# BTstack sebagai komponen ESP-IDF: setara port/esp32/integrate_btstack.py, tetapi disalin
# ke folder build (submodule tidak diubah, IDF tidak ditulisi).
set(LA_BTSTACK_COMP ${CMAKE_BINARY_DIR}/la_btstack/btstack)
if(NOT EXISTS ${LA_BTSTACK_COMP}/CMakeLists.txt)
    if(NOT EXISTS ${BTSTACK_ROOT}/src/btstack.h)
        message(FATAL_ERROR "BTstack tidak ada. Jalankan: git submodule update --init --recursive firmware/radio/third_party/bluepad32")
    endif()
    message(STATUS "Menyiapkan komponen BTstack di ${LA_BTSTACK_COMP}")
    file(COPY ${BTSTACK_ROOT}/port/esp32/components/btstack/ DESTINATION ${LA_BTSTACK_COMP})
    foreach(d src 3rd-party/bluedroid 3rd-party/hxcmod-player 3rd-party/lwip/dhcp-server 3rd-party/lc3-google
              3rd-party/md5 3rd-party/micro-ecc 3rd-party/yxml platform/freertos platform/lwip tool)
        file(COPY ${BTSTACK_ROOT}/${d}/ DESTINATION ${LA_BTSTACK_COMP}/${d})
    endforeach()
    foreach(f hal_time_ms.h hal_uart_dma.h hci_dump_embedded_stdout.h hci_dump_embedded_stdout.c)
        file(COPY ${BTSTACK_ROOT}/platform/embedded/${f} DESTINATION ${LA_BTSTACK_COMP}/platform/embedded)
    endforeach()
endif()

set(EXTRA_COMPONENT_DIRS ${BLUEPAD32_ROOT}/src/components ${CMAKE_BINARY_DIR}/la_btstack)   # bluepad32 + cmd_nvs/cmd_system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(leumpang_radio)

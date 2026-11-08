# =============================================================================
#  Leumpang Ajeg - firmware STM32H562 (WeAct STM32H562RGT6 core board)
#    cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
#    cmake --build build -j        ->  build/leumpang_h562.{elf,bin,hex}
#  Dependensi diunduh otomatis (versi dipin) lewat FetchContent.
# =============================================================================
cmake_minimum_required(VERSION 3.20)
if(NOT CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/cmake/arm-none-eabi.cmake)
endif()
project(leumpang_h562 LANGUAGES C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_C_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)
FetchContent_Declare(st_hal    GIT_REPOSITORY https://github.com/STMicroelectronics/stm32h5xx-hal-driver.git GIT_TAG v1.7.0 GIT_SHALLOW TRUE SOURCE_SUBDIR _download_only_)
FetchContent_Declare(st_device GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis-device-h5.git      GIT_TAG v1.7.0 GIT_SHALLOW TRUE SOURCE_SUBDIR _download_only_)
FetchContent_Declare(st_cmsis  GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis-core.git           GIT_TAG v5.9.0_20250520 GIT_SHALLOW TRUE SOURCE_SUBDIR _download_only_)
FetchContent_Declare(st_fatfs  GIT_REPOSITORY https://github.com/STMicroelectronics/stm32_mw_fatfs.git       GIT_TAG r0.16_stm32cube_20260904 GIT_SHALLOW TRUE SOURCE_SUBDIR _download_only_)
FetchContent_Declare(tinyusb   GIT_REPOSITORY https://github.com/hathach/tinyusb.git                         GIT_TAG 0.21.0 GIT_SHALLOW TRUE SOURCE_SUBDIR _download_only_)
# SOURCE_SUBDIR fiktif: hanya unduh sumber, jangan add_subdirectory (CMake >= 3.20, aman di 4.x)
foreach(dep st_hal st_device st_cmsis st_fatfs tinyusb)
  FetchContent_GetProperties(${dep})
endforeach()
FetchContent_MakeAvailable(st_hal st_device st_cmsis st_fatfs tinyusb)

set(HAL ${st_hal_SOURCE_DIR}/Src)
set(HAL_SRC
  ${HAL}/stm32h5xx_hal.c ${HAL}/stm32h5xx_hal_cortex.c ${HAL}/stm32h5xx_hal_rcc.c ${HAL}/stm32h5xx_hal_rcc_ex.c
  ${HAL}/stm32h5xx_hal_gpio.c ${HAL}/stm32h5xx_hal_pwr.c ${HAL}/stm32h5xx_hal_pwr_ex.c ${HAL}/stm32h5xx_hal_flash.c
  ${HAL}/stm32h5xx_hal_flash_ex.c ${HAL}/stm32h5xx_hal_dma.c ${HAL}/stm32h5xx_hal_dma_ex.c ${HAL}/stm32h5xx_hal_exti.c
  ${HAL}/stm32h5xx_hal_i2c.c ${HAL}/stm32h5xx_hal_i2c_ex.c ${HAL}/stm32h5xx_hal_icache.c ${HAL}/stm32h5xx_hal_uart.c
  ${HAL}/stm32h5xx_hal_uart_ex.c ${HAL}/stm32h5xx_hal_sd.c ${HAL}/stm32h5xx_hal_sd_ex.c ${HAL}/stm32h5xx_ll_sdmmc.c
  ${HAL}/stm32h5xx_hal_tim.c ${HAL}/stm32h5xx_hal_tim_ex.c)
set(DEVICE_SRC
  ${st_device_SOURCE_DIR}/Source/Templates/system_stm32h5xx.c
  ${st_device_SOURCE_DIR}/Source/Templates/gcc/startup_stm32h562xx.s)
set(TUSB ${tinyusb_SOURCE_DIR}/src)
set(TUSB_SRC
  ${TUSB}/tusb.c ${TUSB}/common/tusb_fifo.c ${TUSB}/device/usbd.c ${TUSB}/device/usbd_control.c
  ${TUSB}/class/cdc/cdc_device.c ${TUSB}/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
  ${TUSB}/portable/st/stm32_fsdev/fsdev_common.c)
set(FATFS_SRC ${st_fatfs_SOURCE_DIR}/source/ff.c)

set(CORE ${CMAKE_CURRENT_LIST_DIR}/../../core)
set(CORE_SRC ${CORE}/src/controller.cpp ${CORE}/src/lap.cpp ${CORE}/src/lap_endpoint.cpp
             ${CORE}/src/lap_serial.cpp ${CORE}/src/motion.cpp ${CORE}/src/icm20948.cpp)

set(APP_SRC src/main.cpp src/clock.cpp src/ics_bus.cpp src/imu.cpp src/storage.cpp
            src/sd_diskio.c src/usb_descriptors.c src/stm32_it.c)

add_executable(leumpang_h562 ${APP_SRC} ${CORE_SRC} ${HAL_SRC} ${DEVICE_SRC} ${TUSB_SRC} ${FATFS_SRC})
set_target_properties(leumpang_h562 PROPERTIES SUFFIX ".elf")
target_include_directories(leumpang_h562 PRIVATE
  include ${CORE}/include
  ${st_hal_SOURCE_DIR}/Inc ${st_device_SOURCE_DIR}/Include ${st_cmsis_SOURCE_DIR}/Core/Include
  ${TUSB} ${st_fatfs_SOURCE_DIR}/source)
target_compile_definitions(leumpang_h562 PRIVATE STM32H562xx USE_HAL_DRIVER CFG_TUSB_MCU=OPT_MCU_STM32H5)
target_compile_options(leumpang_h562 PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra>)
set_source_files_properties(${HAL_SRC} ${TUSB_SRC} ${FATFS_SRC} ${DEVICE_SRC} PROPERTIES COMPILE_OPTIONS "-w")
target_link_options(leumpang_h562 PRIVATE -T${CMAKE_CURRENT_LIST_DIR}/STM32H562RGTX_FLASH.ld
                    -Wl,-Map=${CMAKE_BINARY_DIR}/leumpang_h562.map -Wl,--print-memory-usage)
add_custom_command(TARGET leumpang_h562 POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:leumpang_h562> ${CMAKE_BINARY_DIR}/leumpang_h562.bin
  COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:leumpang_h562> ${CMAKE_BINARY_DIR}/leumpang_h562.hex
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:leumpang_h562>)

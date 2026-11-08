# =============================================================================
#  Leumpang Ajeg - build HOST (Linux/macOS): core, simulator, tes, cek firmware
#  Firmware dibangun terpisah dengan ESP-IDF (firmware/esp32s3).
#
#    cmake -B build && cmake --build build -j && ctest --test-dir build --output-on-failure
# =============================================================================
cmake_minimum_required(VERSION 3.16)
project(leumpang_ajeg LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

option(LA_SANITIZE "AddressSanitizer + UBSan untuk core/sim/tes" ON)
option(LA_FIRMWARE_CHECK "Cek compile+link firmware ESP32-S3 terhadap stub IDF" ON)

set(LA_WARN -Wall -Wextra -Wshadow)
set(LA_SAN "")
if(LA_SANITIZE AND NOT MSVC)
  set(LA_SAN -fsanitize=address,undefined -fno-omit-frame-pointer)
endif()

set(LA_CORE_SRC core/src/controller.cpp core/src/lap.cpp core/src/lap_endpoint.cpp
                core/src/lap_serial.cpp core/src/motion.cpp core/src/icm20948.cpp)

# ---- core ----------------------------------------------------------------
add_library(leumpang_core STATIC ${LA_CORE_SRC})
target_include_directories(leumpang_core PUBLIC core/include)
target_compile_options(leumpang_core PRIVATE ${LA_WARN} ${LA_SAN})
target_link_options(leumpang_core PUBLIC ${LA_SAN})

# ---- simulator ------------------------------------------------------------
if(UNIX)
  add_executable(leumpang_sim sim/leumpang_sim.cpp)
  target_link_libraries(leumpang_sim PRIVATE leumpang_core)
  target_compile_options(leumpang_sim PRIVATE ${LA_WARN} ${LA_SAN})
endif()

# ---- tes ------------------------------------------------------------------
enable_testing()
foreach(t test_lap test_controller test_imu)
  add_executable(${t} tests/${t}.cpp)
  target_include_directories(${t} PRIVATE tests)
  target_link_libraries(${t} PRIVATE leumpang_core)
  target_compile_options(${t} PRIVATE ${LA_WARN} ${LA_SAN})
  add_test(NAME ${t} COMMAND ${t})
endforeach()

# Paritas: firmware V4 lama (tests/legacy) vs core, input identik per siklus
add_executable(test_parity tests/test_parity.cpp tests/legacy/control_legacy.cpp)
target_include_directories(test_parity PRIVATE tests/idf_stubs tests/legacy)
target_compile_definitions(test_parity PRIVATE HOST_TEST)
target_compile_options(test_parity PRIVATE -w ${LA_SAN})
target_link_libraries(test_parity PRIVATE leumpang_core)
add_test(NAME test_parity COMMAND test_parity)

find_package(Python3 COMPONENTS Interpreter)
if(Python3_FOUND AND TARGET leumpang_sim)
  add_test(NAME test_lap_python COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/test_lap_py.py $<TARGET_FILE:leumpang_sim>)
endif()

# ---- cek firmware ESP (S3 & P4) di host: syntax + link semua modul ------------
if(LA_FIRMWARE_CHECK AND UNIX AND NOT APPLE AND Python3_FOUND)
  set(WEAK ${CMAKE_BINARY_DIR}/idf_weak_stubs.cpp)
  add_custom_command(OUTPUT ${WEAK}
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/gen_weak_stubs.py ${CMAKE_SOURCE_DIR}/tests/idf_stubs/idf_stub.h ${WEAK}
    DEPENDS tests/gen_weak_stubs.py tests/idf_stubs/idf_stub.h)
  set_source_files_properties(${WEAK} PROPERTIES COMPILE_OPTIONS "-w")
  file(GLOB FW_ESP_SRC firmware/esp/main/*.cpp)
  foreach(v s3 p4)
    add_executable(fw_${v}_linkcheck ${FW_ESP_SRC} ${LA_CORE_SRC} ${WEAK})
    target_include_directories(fw_${v}_linkcheck PRIVATE tests/idf_stubs core/include firmware/esp/main)
    target_compile_options(fw_${v}_linkcheck PRIVATE -Wall -Wextra -Wno-missing-field-initializers)
    target_link_options(fw_${v}_linkcheck PRIVATE -nostartfiles -Wl,-e,app_main)
  endforeach()
  target_compile_definitions(fw_p4_linkcheck PRIVATE LA_STUB_P4)
endif()

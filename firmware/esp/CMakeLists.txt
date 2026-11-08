# Leumpang Ajeg - firmware ESP-IDF (>= 5.3) untuk ESP32-S3 dan ESP32-P4
#   idf.py set-target esp32s3   # atau esp32p4 (FireBeetle 2)
#   idf.py build flash monitor
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(leumpang_ajeg)

# Komponen ESP-IDF yang membungkus leumpang-core (dipakai firmware esp32s3 & esp32p4).
set(LA_CORE ${CMAKE_CURRENT_LIST_DIR}/../../../core)
idf_component_register(SRCS "${LA_CORE}/src/controller.cpp" "${LA_CORE}/src/lap.cpp" "${LA_CORE}/src/lap_endpoint.cpp"
                            "${LA_CORE}/src/lap_serial.cpp" "${LA_CORE}/src/motion.cpp" "${LA_CORE}/src/icm20948.cpp"
                       INCLUDE_DIRS "${LA_CORE}/include")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra)

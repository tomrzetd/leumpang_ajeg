# Komponen main otomatis bergantung ke semua komponen (termasuk leumpang_core di ../components).
idf_component_register(SRCS "main.cpp" "control.cpp" "imu_icm20948.cpp" "web.cpp" "local_ui.cpp" "lap_tcp.cpp"
                       INCLUDE_DIRS ".")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Wno-missing-field-initializers)

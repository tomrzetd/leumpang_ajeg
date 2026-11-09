set(LA_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../../..)
idf_component_register(SRCS "main.cpp" "../../common/radio.cpp" "${LA_ROOT}/core/src/lap.cpp"
                       INCLUDE_DIRS "." "../../common" "${LA_ROOT}/core/include"
                       REQUIRES bluepad32 btstack driver esp_timer)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Wno-missing-field-initializers)

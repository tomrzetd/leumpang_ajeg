# Leumpang Studio - aplikasi desktop Qt 6 (Windows / Linux / macOS x86_64).
#   cmake -B build -DCMAKE_PREFIX_PATH=<Qt>/6.8.3/<kit>      (Qt >= 6.5)
#   cmake --build build --config Release
# Deteksi pose kamera memakai ONNX Runtime (diunduh otomatis, -DLA_STUDIO_POSE=OFF untuk mematikan).
cmake_minimum_required(VERSION 3.21)
project(leumpang_studio VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Gui Qml Quick Quick3D QuickControls2 Network SerialPort Multimedia)
qt_standard_project_setup(REQUIRES 6.5)

set(LA_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
option(LA_STUDIO_POSE "Deteksi pose kamera dengan ONNX Runtime" ON)
set(LA_ORT_VERSION 1.22.0 CACHE STRING "Versi ONNX Runtime")

# ---- leumpang-core + pustaka studio (sama persis dengan firmware & tes host) ----
add_library(la_core STATIC
  ${LA_ROOT}/core/src/controller.cpp ${LA_ROOT}/core/src/lap.cpp ${LA_ROOT}/core/src/lap_endpoint.cpp
  ${LA_ROOT}/core/src/lap_serial.cpp ${LA_ROOT}/core/src/motion.cpp ${LA_ROOT}/core/src/icm20948.cpp)
target_include_directories(la_core PUBLIC ${LA_ROOT}/core/include)
if(MSVC)
  target_compile_definitions(la_core PUBLIC _CRT_SECURE_NO_WARNINGS NOMINMAX _USE_MATH_DEFINES)
  target_compile_options(la_core PUBLIC /utf-8)
endif()

add_library(la_studio_lib STATIC
  lib/kinematics.cpp lib/retarget.cpp lib/motion_doc.cpp lib/lap_client.cpp lib/sim_robot.cpp)
target_include_directories(la_studio_lib PUBLIC lib)
target_link_libraries(la_studio_lib PUBLIC la_core)

# ---- ONNX Runtime (prebuilt resmi dari GitHub Microsoft) ----
if(LA_STUDIO_POSE)
  if(WIN32)
    set(_ort_pkg onnxruntime-win-x64-${LA_ORT_VERSION})
    set(_ort_url https://github.com/microsoft/onnxruntime/releases/download/v${LA_ORT_VERSION}/${_ort_pkg}.zip)
  elseif(APPLE)
    set(_ort_pkg onnxruntime-osx-universal2-${LA_ORT_VERSION})
    set(_ort_url https://github.com/microsoft/onnxruntime/releases/download/v${LA_ORT_VERSION}/${_ort_pkg}.tgz)
  else()
    set(_ort_pkg onnxruntime-linux-x64-${LA_ORT_VERSION})
    set(_ort_url https://github.com/microsoft/onnxruntime/releases/download/v${LA_ORT_VERSION}/${_ort_pkg}.tgz)
  endif()
  include(FetchContent)
  FetchContent_Declare(onnxruntime URL ${_ort_url} DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  FetchContent_MakeAvailable(onnxruntime)
  set(ORT_DIR ${onnxruntime_SOURCE_DIR})
  add_library(onnxruntime SHARED IMPORTED)
  if(WIN32)
    set_target_properties(onnxruntime PROPERTIES IMPORTED_LOCATION ${ORT_DIR}/lib/onnxruntime.dll
                          IMPORTED_IMPLIB ${ORT_DIR}/lib/onnxruntime.lib)
  elseif(APPLE)
    set_target_properties(onnxruntime PROPERTIES IMPORTED_LOCATION ${ORT_DIR}/lib/libonnxruntime.dylib)
  else()
    set_target_properties(onnxruntime PROPERTIES IMPORTED_LOCATION ${ORT_DIR}/lib/libonnxruntime.so)
  endif()
  target_include_directories(onnxruntime INTERFACE ${ORT_DIR}/include)
  target_sources(la_studio_lib PRIVATE lib/pose_onnx.cpp)
  target_link_libraries(la_studio_lib PUBLIC onnxruntime)
  target_compile_definitions(la_studio_lib PUBLIC LA_HAS_ORT=1)
endif()

# ---- aplikasi ----
qt_add_executable(leumpang_studio WIN32 MACOSX_BUNDLE app/main.cpp)
qt_add_qml_module(leumpang_studio
  URI Leumpang
  VERSION 1.0
  QML_FILES qml/Main.qml qml/Twin3D.qml qml/ServoPage.qml qml/MotionPage.qml qml/CameraPage.qml
            qml/GaitPage.qml qml/ParamPage.qml qml/LogPage.qml qml/Pill.qml
  SOURCES app/robot_link.h app/robot_link.cpp app/twin_model.h app/twin_model.cpp
          app/stl_mesh.h app/stl_mesh.cpp app/motion_editor.h app/motion_editor.cpp
          app/pose_capture.h app/pose_capture.cpp)

file(GLOB LA_MESHES RELATIVE ${LA_ROOT}/model/meshes ${LA_ROOT}/model/meshes/*.stl)
set(_mesh_abs)
foreach(m ${LA_MESHES})
  list(APPEND _mesh_abs ${LA_ROOT}/model/meshes/${m})
endforeach()
qt_add_resources(leumpang_studio "meshes" PREFIX "/meshes" BASE ${LA_ROOT}/model/meshes FILES ${_mesh_abs})
qt_add_resources(leumpang_studio "motions" PREFIX "/motions" BASE ${LA_ROOT}/motions FILES ${LA_ROOT}/motions/lambai.json)

target_link_libraries(leumpang_studio PRIVATE la_studio_lib
  Qt6::Core Qt6::Gui Qt6::Qml Qt6::Quick Qt6::Quick3D Qt6::QuickControls2 Qt6::Network Qt6::SerialPort Qt6::Multimedia)
target_include_directories(leumpang_studio PRIVATE app)

if(LA_STUDIO_POSE)
  if(WIN32)
    add_custom_command(TARGET leumpang_studio POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${ORT_DIR}/lib/onnxruntime.dll $<TARGET_FILE_DIR:leumpang_studio>)
  elseif(NOT APPLE)
    set_target_properties(leumpang_studio PROPERTIES BUILD_RPATH "${ORT_DIR}/lib" INSTALL_RPATH "$ORIGIN/../lib")
  endif()
endif()

install(TARGETS leumpang_studio BUNDLE DESTINATION . RUNTIME DESTINATION bin)

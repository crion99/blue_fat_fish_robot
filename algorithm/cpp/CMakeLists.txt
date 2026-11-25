# 蓝胖鱼机器人 · 算法方向（C++17 + OpenCV 4）
#
# Ubuntu 依赖：sudo apt install -y build-essential cmake libopencv-dev
# 构建：cmake -S . -B build && cmake --build build -j

cmake_minimum_required(VERSION 3.16)
project(blue_fat_fish_algo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(MSVC)
  add_compile_options(/utf-8 /W3)
else()
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio imgcodecs)

# ---- 算法核心：不依赖 OpenCV，可在没有相机/没有图形界面的机器上跑自检 ----
add_library(bff_core STATIC
  src/image.cpp
  src/color.cpp
  src/blobs.cpp
  src/annotate.cpp
  src/synthetic_rune.cpp
  src/rune_detector.cpp
  src/pnp.cpp
  src/aim.cpp
  src/pipeline.cpp
)
target_include_directories(bff_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ---- OpenCV 桥接层：配置读取 + 相机采集 ----
add_library(bff_cv STATIC
  src/config.cpp
  src/camera_source.cpp
)
target_link_libraries(bff_cv PUBLIC bff_core ${OpenCV_LIBS})
target_include_directories(bff_cv PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)
target_compile_definitions(bff_cv PUBLIC BFF_WITH_OPENCV)

# ---- 可执行程序 ----
add_executable(live_view apps/live_view.cpp)
target_link_libraries(live_view PRIVATE bff_cv)

add_executable(pose_selftest apps/pose_selftest.cpp)
target_link_libraries(pose_selftest PRIVATE bff_core)

add_executable(rune_selftest apps/rune_selftest.cpp)
target_link_libraries(rune_selftest PRIVATE bff_core)

# ---- 冒烟测试：ctest 里就能跑通 1、3 两个部分的算法 ----
enable_testing()
add_test(NAME pose_selftest COMMAND pose_selftest ${CMAKE_BINARY_DIR}/out 60)
add_test(NAME rune_selftest COMMAND rune_selftest ${CMAKE_BINARY_DIR}/out 6)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/out)

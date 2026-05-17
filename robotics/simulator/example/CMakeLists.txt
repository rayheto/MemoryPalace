# MuJoCo 仿真示例顶层 CMakeLists
# - 统一查找 MuJoCo 安装路径
# - 各 example 通过 add_subdirectory() 接入，子项目复用 $MUJOCO_TARGET 即可
# 用法：
#   cd example && mkdir build && cd build
#   cmake .. && cmake --build . -j
#   ./hopper_1d/hopper_sim     （在该子项目目录下生成 trajectory.csv）

cmake_minimum_required(VERSION 3.13)
project(mujoco_examples LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── 1. 解析 MuJoCo 路径 ─────────────────────────────────────────────
# 优先级：CMake 缓存变量 > 环境变量 > 默认路径
if(NOT DEFINED MUJOCO_DIR)
    if(DEFINED ENV{MUJOCO_DIR})
        set(MUJOCO_DIR $ENV{MUJOCO_DIR})
    else()
        set(MUJOCO_DIR "/home/hlei/robotic/OmniBotSeries-Tinker/OmniBotHub/Linux/mujoco210")
    endif()
endif()
message(STATUS "MUJOCO_DIR = ${MUJOCO_DIR}")

if(NOT EXISTS "${MUJOCO_DIR}/include/mujoco.h")
    message(FATAL_ERROR "MuJoCo not found at ${MUJOCO_DIR}. "
            "Pass -DMUJOCO_DIR=/path/to/mujoco210 to cmake.")
endif()

# ── 2. 公共 IMPORTED 目标 ──────────────────────────────────────────
# mujoco::mujoco     → libmujoco210nogl.so （headless，无 GL 依赖）
# mujoco::mujoco_gl  → libmujoco210.so    （带渲染，UI 用）
# mujoco::glfw       → libglfw.so.3        （MuJoCo 自带的 GLFW）
add_library(mujoco::mujoco SHARED IMPORTED GLOBAL)
set_target_properties(mujoco::mujoco PROPERTIES
    IMPORTED_LOCATION             "${MUJOCO_DIR}/bin/libmujoco210nogl.so"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_DIR}/include"
)

add_library(mujoco::mujoco_gl SHARED IMPORTED GLOBAL)
set_target_properties(mujoco::mujoco_gl PROPERTIES
    IMPORTED_LOCATION             "${MUJOCO_DIR}/bin/libmujoco210.so"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_DIR}/include"
)

add_library(mujoco::glfw SHARED IMPORTED GLOBAL)
set_target_properties(mujoco::glfw PROPERTIES
    IMPORTED_LOCATION             "${MUJOCO_DIR}/bin/libglfw.so.3"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_DIR}/include"
)

# MuJoCo 自带的 GLEW（libmujoco210.so 依赖它）
add_library(mujoco::glew SHARED IMPORTED GLOBAL)
set_target_properties(mujoco::glew PROPERTIES
    IMPORTED_LOCATION "${MUJOCO_DIR}/bin/libglew.so"
)

find_package(OpenGL REQUIRED)

# Eigen3：纯头文件库，子项目链接 Eigen3::Eigen 即可
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
message(STATUS "Eigen3 = ${EIGEN3_INCLUDE_DIR} (version ${EIGEN3_VERSION_STRING})")

# 让链接产物自动带上 rpath，免去 LD_LIBRARY_PATH
set(CMAKE_INSTALL_RPATH "${MUJOCO_DIR}/bin")
set(CMAKE_BUILD_RPATH   "${MUJOCO_DIR}/bin")

# ── 3. ImGui + ImPlot（FetchContent，首跑会从 github 拉源码） ─────
include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.91.5
    GIT_SHALLOW    TRUE
)
FetchContent_Declare(implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG        v0.16
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(imgui implot)

# NLopt（slip_1d_trajopt 用，SLSQP 求解直接配点 NLP）
FetchContent_Declare(nlopt
    GIT_REPOSITORY https://github.com/stevengj/nlopt.git
    GIT_TAG        v2.7.1
    GIT_SHALLOW    TRUE
)
set(NLOPT_PYTHON  OFF CACHE BOOL "" FORCE)
set(NLOPT_OCTAVE  OFF CACHE BOOL "" FORCE)
set(NLOPT_MATLAB  OFF CACHE BOOL "" FORCE)
set(NLOPT_GUILE   OFF CACHE BOOL "" FORCE)
set(NLOPT_SWIG    OFF CACHE BOOL "" FORCE)
set(NLOPT_TESTS   OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(nlopt)

# MuJoCo 自带的 glfw3.h 在 include 根目录下；imgui_impl_glfw 期待 <GLFW/glfw3.h>，做个 shim
set(GLFW_SHIM_DIR ${CMAKE_BINARY_DIR}/_glfw_shim)
file(WRITE ${GLFW_SHIM_DIR}/GLFW/glfw3.h "#pragma once\n#include <glfw3.h>\n")

add_library(imgui_backend STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
)
target_include_directories(imgui_backend PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${implot_SOURCE_DIR}
    ${GLFW_SHIM_DIR}
    ${MUJOCO_DIR}/include
)
target_link_libraries(imgui_backend PUBLIC mujoco::glfw OpenGL::GL)

# ── 4. 通用面板外壳 ────────────────────────────────────────────────
add_subdirectory(simulation_dashboard)

# ── 5. 接入各子项目 ────────────────────────────────────────────────
add_subdirectory(slip_1d)
add_subdirectory(slip_1d_trajopt)
# 新例子在此 add_subdirectory(<name>) 即可

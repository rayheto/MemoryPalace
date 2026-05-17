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

# ── 3. 接入各子项目 ────────────────────────────────────────────────
add_subdirectory(slip_1d)
# 新例子在此 add_subdirectory(<name>) 即可

cmake_minimum_required(VERSION 3.16)
project(cpp_examples LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 启用编译警告
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 添加 CRTP 示例
add_subdirectory(crtp_motor)

# 添加 C 语言 OOP 示例
add_subdirectory(c_oop)

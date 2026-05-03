# CMake 构建系统 —— 从传递规则到交叉编译

## 一、`target_link_libraries` 的 PUBLIC / PRIVATE / INTERFACE

这是 CMake 中最核心也最容易被误解的概念。三个关键字控制的是**属性的传递方向**。

假设有这样的依赖链：`App → Lib → Util`

```cmake
add_library(Util STATIC util.cpp)
target_include_directories(Util PRIVATE include/)       # 只有 Util 自己需要

add_library(Lib STATIC lib.cpp)
target_link_libraries(Lib PUBLIC Util)                   # Lib 使用 Util

add_executable(App main.cpp)
target_link_libraries(App PRIVATE Lib)                   # App 使用 Lib
```

### 1.1 传递规则表

```
                  Util 的 INTERFACE 属性          Util 的 PRIVATE 属性
                  (如 INTERFACE_INCLUDE_DIRS)     (如 INCLUDE_DIRECTORIES)
                  
Lib 用 PUBLIC 链接: 传递给 App ✓                   不传递 ✗
Lib 用 PRIVATE 链接: 不传递 ✗                     不传递 ✗
Lib 用 INTERFACE 链接: 部分传递给 App(取决于值)     不传递 ✗
```

### 1.2 具体示例

```cmake
# Util 库
target_include_directories(Util
    PUBLIC  util_public   # → 进入 INTERFACE_INCLUDE_DIRECTORIES
    PRIVATE util_private  # → 进入 INCLUDE_DIRECTORIES
)
target_compile_definitions(Util PRIVATE UTIL_INTERNAL=1)

# Lib 库
target_link_libraries(Lib PUBLIC Util)
# Lib 编译时: include 路径 = util_public + util_private, 宏有 UTIL_INTERNAL
# Lib 的 INTERFACE_INCLUDE_DIRECTORIES 继承 Util 的 util_public

# App
target_link_libraries(App PRIVATE Lib)
# App 编译时: include 路径 = util_public (继承自 Lib)
# App 看不到 util_private, 也看不到 UTIL_INTERNAL 宏
```

**底层原理：** CMake 在生成阶段维护两个列表：
- **BUILD 属性**（`INCLUDE_DIRECTORIES` / `COMPILE_DEFINITIONS`）：仅当前 target 编译时使用
- **INTERFACE 属性**（`INTERFACE_INCLUDE_DIRECTORIES` / `INTERFACE_COMPILE_DEFINITIONS`）：传递给依赖者

`target_link_libraries(A PUBLIC B)` 等价于：
```
A 的 BUILD 属性 += B 的 INTERFACE 属性
A 的 INTERFACE 属性 += B 的 INTERFACE 属性
```

`target_link_libraries(A PRIVATE B)` 等价于：
```
A 的 BUILD 属性 += B 的 INTERFACE 属性
// 不继承到 A 的 INTERFACE → B 对 A 的依赖者不可见
```

### 1.3 典型用途

| 关键字 | 含义 | 典型场景 |
|--------|------|---------|
| `PRIVATE` | 我用你，但不暴露给依赖我的人 | `App` 用 `Lib`，只 App 知道 |
| `PUBLIC` | 我用你，且依赖我的人也需要你 | `Lib` 用 `Util`，因为 Lib 的头文件里 `#include "util.h"` |
| `INTERFACE` | 我不用你，但依赖我的人需要你 | header-only 库之间的依赖 |

**判断规则：** 如果 `A.h` 里 `#include "B.h"`，则 A 对 B 必须为 PUBLIC（或至少 INTERFACE）。如果只在 `A.cpp` 里 include，PRIVATE 就够。

---

## 二、交叉编译：Toolchain File

### 2.1 最简单的工作链

```cmake
# arm-none-eabi.cmake
set(CMAKE_SYSTEM_NAME       Generic)       # 告诉 CMake 这是裸机环境
set(CMAKE_SYSTEM_PROCESSOR  arm)

set(CMAKE_C_COMPILER    arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER  arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
```

使用: `cmake -B build -DCMAKE_TOOLCHAIN_FILE=arm-none-eabi.cmake`

### 2.2 `CMAKE_FIND_ROOT_PATH_MODE` 的含义

- **`NEVER`**：在主机系统路径中查找（如 `find_program`，编译工具链本身在主机上）
- **`ONLY`**：只在 toolchain 的 sysroot 中查找（如头文件和库，不能被主机 `/usr/include` 污染）
- **`BOTH`**：两边都找

### 2.3 ESP32 交叉编译的完整示例

```cmake
# 实际工作链 (简化自 ESP-IDF)
set(CMAKE_SYSTEM_NAME        Generic)
set(CMAKE_SYSTEM_PROCESSOR   xtensa)

set(CMAKE_C_COMPILER    ${tools}/xtensa-esp32-elf-gcc)
set(CMAKE_CXX_COMPILER  ${tools}/xtensa-esp32-elf-g++)
set(CMAKE_ASM_COMPILER  ${tools}/xtensa-esp32-elf-gcc)

set(CMAKE_C_FLAGS_INIT      "-mlongcalls -Wno-frame-address")
set(CMAKE_CXX_STANDARD      17)

# 跳过 CMake 编译器的"尝试编译测试"
set(CMAKE_C_COMPILER_WORKS   1)
set(CMAKE_CXX_COMPILER_WORKS 1)
```

`CMAKE_C_COMPILER_WORKS` 设为 1 是为了跳过 CMake 的 `try_compile` 探测——在无 OS 的裸机系统上，链接器找不到 `_start` 符号，try_compile 会失败。

---

## 三、`find_package` vs `FetchContent`

### 3.1 find_package：系统已安装的库

```cmake
find_package(OpenSSL REQUIRED)
target_link_libraries(my_app PRIVATE OpenSSL::SSL)
```

CMake 搜索 `FindOpenSSL.cmake` 或 `OpenSSLConfig.cmake`，读取目标定义。问题：嵌入式交叉编译时，主机上的 `/usr/lib/libssl.so` 是 x86_64 的，不可用。

### 3.2 FetchContent：从源码拉取并原地构建

```cmake
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
FetchContent_MakeAvailable(googletest)
target_link_libraries(my_test PRIVATE gtest_main)
```

FetchContent 在 configure 阶段下载源码、`add_subdirectory` 加入构建。优势：无需预装、版本锁定、交叉编译自然支持（因为用同一个 toolchain）。

### 3.3 嵌入式项目的推荐策略

```cmake
# 用 FetchContent 管理所有依赖（版本锁定, CI 可重现）
FetchContent_Declare(freertos ...)
FetchContent_Declare(littlefs ...)

# 对硬件 SDK 用 find_package(仅搜索 toolchain sysroot)
find_package(STM32CubeF4 REQUIRED PATHS ${SDK_PATH})
```

---

## 四、生成器表达式（Generator Expressions）

生成器表达式 `${<:>}` 是在**生成阶段**才求值的。它不等价于 `${}`（配置阶段求值）。

```cmake
# 多配置生成器 (如 Xcode/Visual Studio) 中不同配置用不同 flag
target_compile_options(lib PRIVATE
    $<$<CONFIG:Debug>:-DDEBUG>
    $<$<CONFIG:Release>:-DNDEBUG>
)

# 基于编译器 ID 的条件
target_compile_options(lib PRIVATE
    $<$<CXX_COMPILER_ID:GNU>:-Wall>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

# 排除特定源文件的编译选项
set_source_files_properties(third_party.c PROPERTIES
    COMPILE_FLAGS $<$<COMPILE_LANGUAGE:C>:-Wno-error>
)

# 仅在构建目标(非 install/export)时添加
target_link_libraries(lib PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
```

---

## 五、常用嵌入式 CMake 模式

### 5.1 链接脚本指定

```cmake
target_link_options(firmware.elf PRIVATE
    -T ${CMAKE_CURRENT_SOURCE_DIR}/STM32F407VGTx_FLASH.ld
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
)
```

### 5.2 编译数据库 + 静态分析

```cmake
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # 生成 compile_commands.json
# IDE (clangd / vscode) 和 clang-tidy 可直接使用
```

### 5.3 构建后的 bin/hex 生成

```cmake
add_custom_command(TARGET firmware.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:firmware.elf>
            ${CMAKE_CURRENT_BINARY_DIR}/firmware.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:firmware.elf>
            ${CMAKE_CURRENT_BINARY_DIR}/firmware.hex
)
```

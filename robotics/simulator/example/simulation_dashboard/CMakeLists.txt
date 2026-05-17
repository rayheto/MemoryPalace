# 通用 MuJoCo + ImGui + ImPlot 仿真面板外壳
# 父 CMakeLists 已准备好 imgui_backend / mujoco::* / OpenGL::GL 目标

add_library(simulation_dashboard STATIC dashboard.cpp)
target_include_directories(simulation_dashboard PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(simulation_dashboard PUBLIC
    imgui_backend
    mujoco::mujoco_gl
    mujoco::glfw
    mujoco::glew
    OpenGL::GL
)

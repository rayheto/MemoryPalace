# 1D 垂直跳跃器示例（cartPoleDynamics.m → MuJoCo C++ 移植）
# 父 CMakeLists 已定义 simulation_dashboard / mujoco::* / Eigen3::Eigen 目标

# ── 动力学模型（cartPoleDynamics 的 Eigen 实现） ──
add_library(slip_1d_model STATIC slip_1d_model.cpp)
target_include_directories(slip_1d_model PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(slip_1d_model PUBLIC Eigen3::Eigen)

# ── UI 版（仅业务逻辑，UI 外壳由 simulation_dashboard 提供） ──
add_executable(hopper_sim_ui hopper_sim_ui.cpp)
target_link_libraries(hopper_sim_ui PRIVATE
    slip_1d_model
    simulation_dashboard
)

# 把 model.xml 拷到可执行文件目录，方便直接运行
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/model.xml
               ${CMAKE_CURRENT_BINARY_DIR}/model.xml
               COPYONLY)

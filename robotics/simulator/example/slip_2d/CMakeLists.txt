# 2D SLIP（事件触发 RK4，MATLAB ode45 参考的 C++ 移植）
# 父 CMakeLists 已定义 simulation_dashboard / Eigen3::Eigen 目标

add_library(slip_2d_model STATIC slip_2d_model.cpp slip_2d_sim.cpp)
target_include_directories(slip_2d_model PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(slip_2d_model PUBLIC Eigen3::Eigen)
target_compile_features(slip_2d_model PUBLIC cxx_std_14)   # aggregate init + 默认成员初始化

add_executable(slip_2d_sim_ui slip_2d_sim_ui.cpp)
target_link_libraries(slip_2d_sim_ui PRIVATE slip_2d_model simulation_dashboard)
target_compile_features(slip_2d_sim_ui PRIVATE cxx_std_14)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/model.xml
               ${CMAKE_CURRENT_BINARY_DIR}/model.xml COPYONLY)

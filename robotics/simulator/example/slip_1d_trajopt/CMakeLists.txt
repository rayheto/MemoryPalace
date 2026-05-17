# 1D SLIP 三段式轨迹优化（cartPoleDynamics + NLopt SLSQP）
# 父 CMakeLists 已定义 slip_1d_model / simulation_dashboard / nlopt 目标

add_library(slip_trajopt STATIC trajopt.cpp)
target_include_directories(slip_trajopt PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(slip_trajopt PUBLIC slip_1d_model Eigen3::Eigen nlopt)

add_executable(slip_trajopt_ui slip_trajopt_ui.cpp)
target_link_libraries(slip_trajopt_ui PRIVATE slip_trajopt simulation_dashboard)

# model.xml 直接复用 slip_1d 的，便于 build 后从本目录启动
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../slip_1d/model.xml
               ${CMAKE_CURRENT_BINARY_DIR}/model.xml COPYONLY)

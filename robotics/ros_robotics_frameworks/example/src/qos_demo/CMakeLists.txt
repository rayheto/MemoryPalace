cmake_minimum_required(VERSION 3.8)
project(qos_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(history_depth_demo src/history_depth_demo.cpp)
ament_target_dependencies(history_depth_demo rclcpp std_msgs)

add_executable(transient_local_demo src/transient_local_demo.cpp)
ament_target_dependencies(transient_local_demo rclcpp std_msgs)


install(TARGETS history_depth_demo transient_local_demo 
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

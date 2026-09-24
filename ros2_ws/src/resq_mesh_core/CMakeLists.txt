cmake_minimum_required(VERSION 3.8)
project(resq_mesh_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(mission_node src/mission_node.cpp)

ament_target_dependencies(mission_node
  rclcpp
  std_msgs
)

add_executable(mission_service src/mission_service.cpp)

ament_target_dependencies(mission_service
  rclcpp
  std_srvs
)

add_executable(mission_action src/mission_action.cpp)

ament_target_dependencies(mission_action
  rclcpp
  rclcpp_action
  example_interfaces
)

install(TARGETS
  mission_node
  mission_service
  mission_action
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

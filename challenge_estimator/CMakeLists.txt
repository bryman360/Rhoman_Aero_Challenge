cmake_minimum_required(VERSION 3.8)
project(challenge_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(challenge_interfaces REQUIRED)

add_executable(estimator_node src/estimator_node.cpp)
ament_target_dependencies(estimator_node rclcpp challenge_interfaces)

install(
  TARGETS estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(ROV_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(controller_msgs REQUIRED)

add_executable(ROV_controller src/main.cpp)

target_include_directories(ROV_controller PRIVATE
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(ROV_controller
  rclcpp
  std_msgs
  Eigen3
  controller_msgs
)

target_include_directories(ROV_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  ROV_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()

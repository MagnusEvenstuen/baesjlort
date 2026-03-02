cmake_minimum_required(VERSION 3.14)
project(log_to_file)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(imu_logger src/imu/main.cpp src/imu/imu_logger.cpp)
target_include_directories(imu_logger PRIVATE include)
target_link_libraries(imu_logger 
	rclcpp::rclcpp
	${sensor_msgs_TARGETS}
)

add_executable(image_logger src/image/main.cpp src/image/image_logger.cpp)
target_include_directories(image_logger PRIVATE include)
target_link_libraries(image_logger 
	rclcpp::rclcpp
	${sensor_msgs_TARGETS}
    cv_bridge::cv_bridge
    ${OpenCV_LIBS}
)

install(TARGETS
  imu_logger
  image_logger
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.8)
project(rvr_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rvr_msgs REQUIRED)
find_package(jsoncpp REQUIRED)

find_library(SERIALPORT_LIB NAMES serialport)
if(NOT SERIALPORT_LIB)
  message(FATAL_ERROR "libserialport not found. Make sure libserialport-dev is installed.")
endif()

# Add bridge_node executable
add_executable(bridge_node src/bridge_node.cpp)
ament_target_dependencies(bridge_node rclcpp rvr_msgs)
target_link_libraries(bridge_node jsoncpp_lib ${SERIALPORT_LIB})
install(TARGETS bridge_node DESTINATION lib/${PROJECT_NAME})

# Add camera_node executable
add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node rclcpp image_transport cv_bridge)
target_link_libraries(camera_node ${OpenCV_LIBS})
install(TARGETS camera_node DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

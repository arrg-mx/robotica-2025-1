cmake_minimum_required(VERSION 3.8)
project(examen_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED) #for python scripts
#find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED) #for python scripts
#find_package(geometry_msgs REQUIRED)
#find_package(nav_msgs REQUIRED)
#find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED) #for ros control
#find_package(rclcpp_action REQUIRED) #for ros control actions


install(
  DIRECTORY launch rviz world src world
  DESTINATION share/${PROJECT_NAME}/
)


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables #They must start with the shebag tag
install(PROGRAMS
  src/trajectory_test.py
  src/tray_test.py
  DESTINATION lib/${PROJECT_NAME} )

ament_package()


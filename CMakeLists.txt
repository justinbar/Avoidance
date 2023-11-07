cmake_minimum_required(VERSION 3.5)
project(control_robot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(control_robot src/control_service.cpp)
ament_target_dependencies(control_robot
  "rclcpp"
  "std_srvs"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs")


find_library(PIGPIO_LIBRARY NAMES pigpio REQUIRED)





install(TARGETS
  control_robot
  DESTINATION lib/${PROJECT_NAME})

  
add_executable(control_robot2 src/control_service2.cpp)
ament_target_dependencies(control_robot2
  "rclcpp"
  "std_srvs"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs")

install(TARGETS
  control_robot2
  DESTINATION lib/${PROJECT_NAME})


add_executable(control_robot3 src/robotcontrol.cpp)
ament_target_dependencies(control_robot3
  "rclcpp"
  "std_srvs"
  "geometry_msgs")

install(TARGETS
  control_robot3
  DESTINATION lib/${PROJECT_NAME})


ament_package()

cmake_minimum_required(VERSION 3.5)
project(dynpick_driver)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
)

set(CMAKE_CXX_STANDARD 14)

## Declare a cpp executable
add_executable(dynpick_driver_node src/main.cpp src/kbhit.c)
ament_target_dependencies(dynpick_driver_node rclcpp geometry_msgs std_srvs)

add_executable(dynpick_com_tester src/test-com.c)

#############
## Install ##
#############

install(TARGETS dynpick_driver_node dynpick_com_tester
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf DESTINATION share/${PROJECT_NAME})

##########
## Test ##
##########

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

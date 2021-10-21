cmake_minimum_required(VERSION 3.5)

project(offboard_ibqr)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(EnableC++XX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# Check if any sanitizers set
include(EnableSanitizers)

# Check if ROS_DISTRO is greater or equal to Dashing
# Required since
#   - "create_subscription()" and "create_publisher()" APIs changed
#   - eigen3_cmake_module is only available in these distros
list(APPEND ROS_DISTROS "dashing" "eloquent" "foxy" "galactic" "rolling")
set(ROS_DISTRO)
if(DEFINED ENV{ROS2_DISTRO})
  set(ROS_DISTRO $ENV{ROS2_DISTRO})
  if(${ROS_DISTRO} IN_LIST ROS_DISTROS)
    add_definitions(-DROS_DEFAULT_API=1)
  endif()
elseif(DEFINED ENV{ROS_DISTRO})
  set(ROS_DISTRO $ENV{ROS_DISTRO})
  if(${ROS_DISTRO} IN_LIST ROS_DISTROS)
    add_definitions(-DROS_DEFAULT_API=1)
  endif()
else()
  message(FATAL_ERROR "No ROS Distro set")
endif()

set(USES_DEPRECATED_EXPORT_API "ardent" "bouncy" "crystal" "dashing" "eloquent")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(multi_rtd_interfaces REQUIRED)
get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)

find_package(px4_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PythonInterp 3 REQUIRED)
if(ROS_DISTRO IN_LIST ROS_DISTROS)
  find_package(eigen3_cmake_module REQUIRED)
endif()
find_package(Eigen3 REQUIRED NO_MODULE)

#################
# Setup targets #
#################

include_directories(include)

function(custom_executable subfolder target)
  add_executable(${target} src/${subfolder}/${target}.cpp)
  ament_target_dependencies(${target}
    rclcpp
    px4_msgs
    trajectory_msgs
    multi_rtd_interfaces
    std_msgs
  )
  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()


## Add executables
custom_executable(offboard offboard_control)
custom_executable(offboard offboard_tester)
#custom_executable(offboard offboard_waypoint)
custom_executable(myTools trajectory_logger)

############
# Install ##
############

# Export information to downstream packages
if(ROS_DISTRO IN_LIST ROS_DISTROS)
  ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs)
else()
  ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime Eigen3 px4_msgs)
endif()

ament_export_include_directories(include)

# Install artifacts
install(DIRECTORY cmake
        DESTINATION share/${PROJECT_NAME}
)


ament_package()

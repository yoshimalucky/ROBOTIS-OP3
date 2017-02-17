cmake_minimum_required(VERSION 2.8.3)
project(op3_online_walking_module)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cmake_modules
  eigen_conversions
  dynamixel_sdk
  robotis_device
  robotis_math
  robotis_controller_msgs
  robotis_framework_common
  #scilab_optimization
  op3_balance_control
  op3_kinematics_dynamics
  op3_online_walking_module_msgs
)

find_package(Eigen REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES op3_online_walking_module
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(op3_online_walking_module
  src/op3_online_walking.cpp
  src/online_walking_module.cpp
) 

add_dependencies(op3_online_walking_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(op3_online_walking_module
  #/usr/lib/scilab/libscilab.so
  ${catkin_LIBRARIES}
)

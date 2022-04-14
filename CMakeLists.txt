cmake_minimum_required(VERSION 2.8.3)
project(controller_common)

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  rtt_rosparam
  cartesian_trajectory_msgs
  trapezoid_trajectory_msgs
  eigen_conversions
  kdl_conversions
  trajectory_msgs
  rtt_actionlib
  kdl_parser
  fabric_logger
  kin_dyn_model
  )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdfdom_headers REQUIRED)

link_directories(${orocos_kdl_LIBRARY_DIRS})

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${urdfdom_headers_INCLUDE_DIRS})

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file

include_directories(${USE_OROCOS_INCLUDE_DIRS} include)
ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)
### Orocos Targets ###

orocos_component(controller_common
  src/controller_common.cpp
  src/pose_transform.cpp
  src/tf_publisher.cpp
  src/velocityprofile_spline.cpp
  src/velocityprofile_trapezoid.cpp
  src/scalar_dummy.cpp
  src/elmo_servo_state.cpp
)
target_link_libraries(controller_common ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${orocos_kdl_LIBRARIES})

### Orocos Package Exports and Install Targets ###

# Export package information (replaces catkin_package() macro) 
orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros rtt_rosparam geometry_msgs std_msgs message_runtime rtt_roscomm cartesian_trajectory_msgs trapezoid_trajectory_msgs trajectory_msgs rtt_actionlib urdfdom_headers kdl_parser fabric_logger
)

orocos_install_headers(
  DIRECTORY include/${PROJECT_NAME}/
)

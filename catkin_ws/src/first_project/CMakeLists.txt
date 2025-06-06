cmake_minimum_required(VERSION 2.8.3)
project(first_project)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  tf
  sensor_msgs
  dynamic_reconfigure
  message_filters
  message_generation
  std_msgs
)

add_message_files(
  FILES
  Custom.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
# Declare parameter configuration files
generate_dynamic_reconfigure_options(
  config/gps_odometer_params.cfg
  config/odometer_params.cfg
)

# Declare the catkin package
catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES first_project
  #CATKIN_DEPENDS message_runtime 
)

# Include directories
include_directories(include ${catkin_INCLUDE_DIRS}
)

#Add executable for each node
add_executable(odometer src/odom_pub_sub.cpp src/Kalman_filter.cpp)
add_dependencies(odometer ${PROJECT_NAME}_gencfg)

#Link the executables with the required libraries
target_link_libraries(odometer
 ${catkin_LIBRARIES}
)

add_executable(gps_odometer src/gps_pub_sub.cpp)
add_dependencies(gps_odometer ${PROJECT_NAME}_gencfg)

target_link_libraries(gps_odometer
  ${catkin_LIBRARIES}
)

add_executable(sector_time src/sector_time_pub_sub.cpp)
add_dependencies(sector_time custom_messages_generate_messages_cpp)
target_link_libraries(sector_time 
  ${catkin_LIBRARIES}
)

add_executable(EKF src/EKF_pub_sub.cpp src/Kalman_filter.cpp)

#Link the executables with the required libraries
target_link_libraries(EKF
 ${catkin_LIBRARIES}
)
# Install the executables
#install(TARGETS
#  odometer
#  gps_odometer
#  sector_time
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
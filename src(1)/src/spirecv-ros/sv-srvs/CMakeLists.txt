cmake_minimum_required(VERSION 2.8.3)
project(spirecv_srvs)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_srvs
)

add_service_files(
  DIRECTORY srvs
  FILES
  CameraService.srv
  GimbalSetHome.srv
  TrackService.srv
)

generate_messages(
  DEPENDENCIES
  #sensor_msgs
  # std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


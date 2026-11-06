cmake_minimum_required(VERSION 2.8.3)
project(spirecv_ros)


find_package(catkin REQUIRED COMPONENTS
  message_generation  
  roscpp
  geometry_msgs
  actionlib_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_eigen
  spirecv_msgs
  image_transport
  cv_bridge
)


catkin_package(
  CATKIN_DEPENDS
  message_runtime
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  std_msgs
)



find_package(SpireCV REQUIRED)
find_package(OpenCV 4 REQUIRED)

message(STATUS "SpireCV library status:")
message(STATUS "    include path: ${SV_INCLUDE_DIRS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SV_INCLUDE_DIRS}
  #/usr/local/include

)
link_directories(/usr/local/lib
		/usr/local/lib/gstreamer-1.0)


add_executable(common_object_detection
  samples/mipi/common_object_detection.cpp
)
target_link_libraries(common_object_detection ${catkin_LIBRARIES} ${SV_LIBRARIES})

add_executable(ellipse_detection
  samples/mipi/ellipse_detection.cpp
)
target_link_libraries(ellipse_detection ${catkin_LIBRARIES} ${SV_LIBRARIES})

add_executable(landing_marker_detection
  samples/mipi/landing_marker_detection.cpp
)
target_link_libraries(landing_marker_detection ${catkin_LIBRARIES} ${SV_LIBRARIES})

add_executable(single_object_tracking
  samples/mipi/single_object_tracking.cpp
)
target_link_libraries(single_object_tracking ${catkin_LIBRARIES} ${SV_LIBRARIES})

add_executable(aruco_detection
  samples/mipi/aruco_detection.cpp
)
target_link_libraries(aruco_detection ${catkin_LIBRARIES} ${SV_LIBRARIES})

add_executable(common_detection_tracking
  samples/mipi/common_object_detection_tracking.cpp
)
target_link_libraries(common_detection_tracking ${catkin_LIBRARIES} ${SV_LIBRARIES})


add_executable(gimbal_server
  samples/gimbal_server.cpp
)
target_link_libraries(gimbal_server ${catkin_LIBRARIES} ${SV_LIBRARIES})

add_executable(video_streaming
  samples/video_streaming.cpp
)
target_link_libraries(video_streaming ${catkin_LIBRARIES} ${SV_LIBRARIES})

add_executable(gimbal_detection_with_tracking
  samples/g1/gimbal_detection_with_tracking.cpp
)
target_link_libraries(gimbal_detection_with_tracking ${catkin_LIBRARIES} ${SV_LIBRARIES})

add_executable(gimbal_aruco_detection_with_tracking
  samples/g1/gimbal_aruco_detection_with_tracking.cpp
)
target_link_libraries(gimbal_aruco_detection_with_tracking ${catkin_LIBRARIES} ${SV_LIBRARIES})

add_executable(car_detection_with_d435i
  samples/gazebo/car_detection_tracking_with_d435i.cpp
)
target_link_libraries(car_detection_with_d435i ${catkin_LIBRARIES} ${SV_LIBRARIES})

add_executable(aruco_detection_with_d435i
  samples/gazebo/aruco_detection_with_d435i.cpp
)
target_link_libraries(aruco_detection_with_d435i ${catkin_LIBRARIES} ${SV_LIBRARIES})



add_executable(gx40_aruco_clicked_tracking samples/gx40/gx40_aruco_clicked_tracking.cpp)
target_link_libraries(gx40_aruco_clicked_tracking ${catkin_LIBRARIES} ${SV_LIBRARIES})

add_executable(gimbal_aruco_clicked_tracking samples/g1/gimbal_aruco_clicked_tracking.cpp)
target_link_libraries(gimbal_aruco_clicked_tracking ${catkin_LIBRARIES} ${SV_LIBRARIES})


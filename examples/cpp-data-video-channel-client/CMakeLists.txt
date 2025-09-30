cmake_minimum_required(VERSION 3.14)
project(CppDataVideoChannelClient)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)
find_package(pcl_conversions REQUIRED)
find_package(rosbag2_cpp REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${webrtc_native_INCLUDE}
  ../../opentera-webrtc-native-client/3rdParty/json/include
  ../../opentera-webrtc-native-client/3rdParty/IXWebSocket
  ../../opentera-webrtc-native-client/3rdParty/cpp-httplib
  ../../opentera-webrtc-native-client/OpenteraWebrtcNativeClient/include
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  pcl_conversions
   rosbag2_cpp
)

target_link_libraries(${PROJECT_NAME}
  OpenteraWebrtcNativeClient
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

if(NOT WIN32)
  target_link_libraries(${PROJECT_NAME} pthread)
endif()

if (NOT OPENTERA_WEBRTC_USE_SYSTEM_OPENCV)
  add_dependencies(${PROJECT_NAME} cv_bridge)
endif()

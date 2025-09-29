cmake_minimum_required(VERSION 3.14.0)
project(CppDataChannelClient)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(BEFORE SYSTEM ${webrtc_native_INCLUDE})
include_directories(../../opentera-webrtc-native-client/3rdParty/json/include)
include_directories(../../opentera-webrtc-native-client/3rdParty/IXWebSocket)
include_directories(../../opentera-webrtc-native-client/3rdParty/cpp-httplib)
include_directories(../../opentera-webrtc-native-client/OpenteraWebrtcNativeClient/include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(CppDataChannelClient main.cpp)

target_link_libraries(CppDataChannelClient
  OpenteraWebrtcNativeClient
)

ament_target_dependencies(CppDataChannelClient
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
)

if (NOT WIN32)
  target_link_libraries(CppDataChannelClient pthread)
endif()

set_property(TARGET CppDataChannelClient PROPERTY CXX_STANDARD 17)

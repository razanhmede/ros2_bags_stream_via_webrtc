cmake_minimum_required(VERSION 3.14.0)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

project(CppVideoStreamClient)

set(LIBRARY_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE})
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)
find_package(pcl_conversions REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(${rcl_interfaces_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${rclcpp_INCLUDE_DIRS})
include_directories(${cv_bridge_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})
include_directories(${rosbag2_cpp_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${nav_msgs_INCLUDE_DIRS})
include_directories(BEFORE SYSTEM ${webrtc_native_INCLUDE})
include_directories(../../opentera-webrtc-native-client/3rdParty/json/include)
include_directories(../../opentera-webrtc-native-client/3rdParty/IXWebSocket)
include_directories(../../opentera-webrtc-native-client/3rdParty/cpp-httplib)
include_directories(../../opentera-webrtc-native-client/OpenteraWebrtcNativeClient/include)

add_executable(CppVideoStreamClient main.cpp)
ament_target_dependencies(CppVideoStreamClient
    rclcpp
    sensor_msgs
    cv_bridge
    OpenCV
    rosbag2_cpp
    rcl_interfaces
    pcl_conversions
    nav_msgs
  
)

target_link_libraries(CppVideoStreamClient
    OpenteraWebrtcNativeClient
    opencv_videoio
    opencv_highgui
    ${PCL_LIBRARIES}
)

if (NOT WIN32)
    target_link_libraries(CppVideoStreamClient
        pthread
    )
endif()

if (NOT OPENTERA_WEBRTC_USE_SYSTEM_OPENCV)
    add_dependencies(CppVideoStreamClient opencv_highgui opencv_videoio cv_bridge ${PCL_LIBRARIES})
endif()

set_property(TARGET CppVideoStreamClient PROPERTY CXX_STANDARD 17)

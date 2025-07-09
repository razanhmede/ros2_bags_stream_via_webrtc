# ros2_bags_stream_via_webrtc

A C++ ROS2 application that reads sensor data from a ROS2 bag file and streams combined video frames (RGB, depth colormap, LiDAR scans, and map) over WebRTC using the Opentera WebRTC Native Client.

## Features

- Lays back ROS2 rosbag2 recordings in real time, respecting original timestamps.

- Decodes and displays:

1. **RGB camera frames**

2. **Depth camera frames (colorized with a jet colormap)**

3. **3D LiDAR point clouds (front and rear) projected into top‑down images**

4. **2D occupancy grid maps**

- Composes a tiled video frame combining all streams, with labels and frame numbering.

- Streams video over WebRTC with configurable ICE servers and signaling.

- Clean shutdown and resource management.

## Steps to run this example:

- Clone the repo
- Follow the build instructions to build opentera-webrtc
- Run the cpp-video-stream client using the following command in the terminal:
 ```bash 
./CppVideoStreamClient /PATH_TO_ROS_BAG
```
- Run in another terminal the web-stream-client using 

 ```bash 
./start_server_bash
``` 

- Open port 8080 , connect to the signaling server using the connect button and then call to rosbag client to visualize the data incoming from the bag.
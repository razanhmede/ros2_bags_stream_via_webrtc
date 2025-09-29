#include <OpenteraWebrtcNativeClient/StreamClient.h>

#include <cv_bridge/cv_bridge.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

using namespace opentera;
using namespace std::chrono_literals;

class TopicVideoSource : public VideoSource, public rclcpp::Node {
  std::atomic_bool m_stopped;
  std::thread m_thread;

  // Buffers
  cv::Mat color_image, depth_colormap, map_image;
  cv::Mat lidar_front_img, lidar_rear_img;
  std::atomic_bool got_front = false, got_rear = false;

  // Subs
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_front_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_rear_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

public:
  TopicVideoSource()
      : VideoSource(VideoSourceConfiguration::create(false, false)),
        Node("topic_video_source"),
        m_stopped(false),
        lidar_front_img(500, 500, CV_8UC3, cv::Scalar(0, 0, 0)),
        lidar_rear_img(500, 500, CV_8UC3, cv::Scalar(0, 0, 0)) {
    using std::placeholders::_1;

    {
      rclcpp::QoS qos(rclcpp::KeepLast(5));
      qos.best_effort();
      color_sub_ = create_subscription<sensor_msgs::msg::Image>(
          "/robot_interface/front_camera/color/image_raw",
          qos, std::bind(&TopicVideoSource::colorCallback, this, _1));
    }
    {
      rclcpp::QoS qos(rclcpp::KeepLast(5));
      qos.best_effort();
      depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
          "/robot_interface/front_camera/depth/image_rect_raw",
          qos, std::bind(&TopicVideoSource::depthCallback, this, _1));
    }
    {
      rclcpp::QoS qos(rclcpp::KeepLast(5));
      qos.best_effort();
      lidar_front_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          "/robot_interface/scan_front_cloud", qos,
          std::bind(&TopicVideoSource::lidarFrontCallback, this, _1));
    }
    {
      rclcpp::QoS qos(rclcpp::KeepLast(5));
      qos.best_effort();
      lidar_rear_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          "/robot_interface/scan_rear_cloud", qos,
          std::bind(&TopicVideoSource::lidarRearCallback, this, _1));
    }
    {
      rclcpp::QoS qos(rclcpp::KeepLast(1));
      qos.reliable();
      qos.transient_local();
      map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
          "/map", qos, std::bind(&TopicVideoSource::mapCallback, this, _1));
    }

    // Processing loop
    m_thread = std::thread(&TopicVideoSource::run, this);
  }

  ~TopicVideoSource() override {
    m_stopped.store(true);
    if (m_thread.joinable())
      m_thread.join();
  }

  // transforms the point clouds into top view images to be displayed
  enum class LiDARType { Front, Rear, LaserScan3D };

  // Draw point cloud into a shared image (top view)
  void drawPointCloudOnSharedImage(
      const sensor_msgs::msg::PointCloud2::SharedPtr& msg, cv::Mat& img,
      const cv::Scalar& color) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    int img_size = img.rows;

    for (const auto& pt : cloud->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y))
        continue;

      int x = static_cast<int>(pt.x * 20 + img_size / 2);
      int y = static_cast<int>(pt.y * 20 + img_size / 2);

      if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
        cv::circle(img, cv::Point(x, img_size - y - 1), 1, color, -1);
      }
    }
  }

  // Transforms occupancy grid into 3-channel BGR image
  cv::Mat occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr& map_msg) {
    int width  = map_msg->info.width;
    int height = map_msg->info.height;

    if (width == 0 || height == 0)
      return cv::Mat(); // empty image

    cv::Mat gray_img(height, width, CV_8UC1);
    for (size_t i = 0; i < map_msg->data.size(); ++i) {
      int8_t val = map_msg->data[i];
      uint8_t pixel = 0;

      if (val == -1)      pixel = 127; // unknown
      else if (val == 0)  pixel = 255; // free
      else                pixel = 0;   // occupied

      gray_img.data[i] = pixel;
    }

    std::vector<cv::Mat> channels(3, gray_img);
    cv::Mat map_img_bgr;
    cv::merge(channels, map_img_bgr);
    return map_img_bgr;
  }

  // ---- Callbacks ----
  void colorCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    color_image = std::move(img);
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat raw = cv_bridge::toCvCopy(msg)->image;  // keep source encoding

    cv::Mat normalized;
    if (raw.type() == CV_32FC1) {
      cv::Mat no_nan = raw.clone();
      cv::patchNaNs(no_nan, 0.f);
      cv::normalize(no_nan, normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    } else if (raw.type() == CV_16UC1) {
      const double max_mm = 5000.0;
      cv::Mat clipped = raw.clone();
      clipped.setTo(max_mm, clipped == 0); // treat 0 as "no return"
      clipped.convertTo(normalized, CV_8UC1, 255.0 / max_mm);
      cv::bitwise_not(normalized, normalized); // nearer = brighter (optional)
    } else {
      return; // unsupported depth type
    }

    cv::Mat cm;
    cv::applyColorMap(normalized, cm, cv::COLORMAP_JET);
    depth_colormap = std::move(cm);
  }

  void lidarFrontCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    cv::Mat img(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
    drawPointCloudOnSharedImage(msg, img, cv::Scalar(0, 255, 0));
    lidar_front_img = std::move(img);
    got_front = true;
  }

  void lidarRearCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    cv::Mat img(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
    drawPointCloudOnSharedImage(msg, img, cv::Scalar(0, 0, 255));
    lidar_rear_img = std::move(img);
    got_rear = true;
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    cv::Mat img = occupancyGridToImage(msg);
    map_image = std::move(img);
  }

private:
  void run() {
    int frame_count = 0;

    while (rclcpp::ok() && !m_stopped) {
      // Need at least RGB + both lidars to build the mosaic
      if (color_image.empty() || !got_front.load() || !got_rear.load()) {
        std::this_thread::sleep_for(5ms);
        continue;
      }

      // RGB (labelled)
      cv::Mat rgb_labeled = color_image.clone();
      cv::putText(rgb_labeled, "RGB", {10, rgb_labeled.rows - 10},
                  cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);

      const cv::Size tile_sz = rgb_labeled.size();
      const int pad = 10;

      // Depth (optional)
      cv::Mat padded_depth;
      if (!depth_colormap.empty()) {
        cv::Mat depth_resized;
        cv::resize(depth_colormap, depth_resized, tile_sz);
        cv::putText(depth_resized, "Depth", {10, 30},
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::copyMakeBorder(depth_resized, padded_depth, 0, 0, 0, pad, cv::BORDER_CONSTANT);
      } else {
        padded_depth = cv::Mat(tile_sz.height, tile_sz.width + pad, CV_8UC3, cv::Scalar(50, 50, 50));
        cv::putText(padded_depth, "No Depth", {10, 30},
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 200), 2);
      }
      cv::Mat depth_snapshot = depth_colormap.clone();
      // LiDARs (both required for bottom row)
      cv::Mat lidar_f_resized, lidar_r_resized;
      cv::resize(lidar_front_img, lidar_f_resized, tile_sz);
      cv::resize(lidar_rear_img,  lidar_r_resized, tile_sz);
      cv::putText(lidar_f_resized, "LiDAR Front", {10, 30},
                  cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
      cv::putText(lidar_r_resized, "LiDAR Rear", {10, 30},
                  cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);

      cv::Mat padded_color, padded_lidar_front, padded_lidar_rear;
      cv::copyMakeBorder(rgb_labeled,     padded_color,       0, 0, 0, pad, cv::BORDER_CONSTANT);
      cv::copyMakeBorder(lidar_f_resized, padded_lidar_front, 0, 0, 0, pad, cv::BORDER_CONSTANT);
      cv::copyMakeBorder(lidar_r_resized, padded_lidar_rear,  0, 0, 0, pad, cv::BORDER_CONSTANT);

      // Map (optional)
      cv::Mat padded_map;
      if (!map_image.empty()) {
        cv::Mat map_resized;
        cv::resize(map_image, map_resized, tile_sz);
        cv::putText(map_resized, "Map", {10, 30},
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::copyMakeBorder(map_resized, padded_map, 0, 0, 0, pad, cv::BORDER_CONSTANT);
      } else {
        padded_map = cv::Mat::zeros(padded_depth.size(), padded_depth.type());
        cv::putText(padded_map, "No Map", {10, 30},
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(100, 100, 100), 2);
      }

      // Compose rows
      cv::Mat top_row, bottom_row;
      cv::hconcat(std::vector<cv::Mat>{padded_color, padded_depth, padded_map}, top_row);
      cv::Mat empty_pad = cv::Mat::zeros(padded_map.size(), padded_map.type());
      cv::hconcat(std::vector<cv::Mat>{padded_lidar_front, padded_lidar_rear, empty_pad}, bottom_row);

      // Final frame
      cv::Mat final_combined;
      cv::vconcat(top_row, bottom_row, final_combined);

      cv::putText(final_combined, "Frame: " + std::to_string(frame_count++),
                  {20, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);

      // Send
      int64_t timestamp_us =
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now().time_since_epoch()).count();

      sendFrame(final_combined, timestamp_us);
      std::cout << "[DEBUG] Sending frame size: "
                << final_combined.cols << "x" << final_combined.rows
                << " channels=" << final_combined.channels() << std::endl;
    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
 
  std::vector<IceServer> iceServers;
  if (!IceServer::fetchFromServer("http://localhost:3001/iceservers", "abc", iceServers)) {
    std::cout << "IceServer::fetchFromServer failed" << std::endl;
    iceServers.clear();
  }

  auto signalingServerConfiguration = SignalingServerConfiguration::create(
      "ws://localhost:3001/signaling", "CppClient", "chat", "abc");

  auto webrtcConfiguration = WebrtcConfiguration::create(iceServers);
  auto videoStreamConfiguration = VideoStreamConfiguration::create();
  auto videoSource = std::make_shared<TopicVideoSource>();

  StreamClient client(signalingServerConfiguration, webrtcConfiguration,
                      videoStreamConfiguration, videoSource, nullptr); // no audio source

  client.setLogger([](const std::string& message) {
    std::cout << "[LOG] " << message << std::endl;
  });

  client.setOnSignalingConnectionOpened(
      []() { std::cout << "Signaling connection opened." << std::endl; });

  client.setOnSignalingConnectionClosed(
      []() { std::cout << "Signaling connection closed." << std::endl; });

  client.setOnSignalingConnectionError([](const std::string& error) {
    std::cout << "Signaling connection error: " << error << std::endl;
  });

  client.setOnRoomClientsChanged(
      [](const std::vector<RoomClient>& roomClients) {
        std::cout << "Room clients changed:" << std::endl;
        for (const auto& c : roomClients) {
          std::cout << "\tID=" << c.id() << ", Name=" << c.name()
                    << ", Connected=" << (c.isConnected() ? "yes" : "no")
                    << std::endl;
        }
      });

  client.setOnClientConnected([](const Client& client) {
    std::cout << "Client connected: ID=" << client.id()
              << ", Name=" << client.name() << std::endl;
  });

  client.setOnClientDisconnected([](const Client& client) {
    std::cout << "Client disconnected: ID=" << client.id()
              << ", Name=" << client.name() << std::endl;
  });

  client.setOnClientConnectionFailed([](const Client& client) {
    std::cout << "Client connection failed: ID=" << client.id()
              << ", Name=" << client.name() << std::endl;
  });

  client.setOnAddRemoteStream([](const Client& client) {
    std::cout << "Remote stream added: ID=" << client.id()
              << ", Name=" << client.name() << std::endl;
  });

  client.setOnRemoveRemoteStream([](const Client& client) {
    std::cout << "Remote stream removed: ID=" << client.id()
              << ", Name=" << client.name() << std::endl;
  });

  client.setOnVideoFrameReceived(
      [](const Client& client, const cv::Mat& bgrImg, uint64_t) {
        std::cout << "Video frame received from: " << client.id() << std::endl;
      });

  client.connect();

  while (rclcpp::ok()) {
    rclcpp::spin_some(videoSource);
  }
  std::cin.get();
  rclcpp::shutdown();
  return 0;
}

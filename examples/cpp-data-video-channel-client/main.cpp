#include <OpenteraWebrtcNativeClient/DataChannelClient.h>
#include <OpenteraWebrtcNativeClient/StreamClient.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>

using nlohmann::json;
using namespace opentera;
using namespace std;
using namespace std::chrono_literals;

namespace {
inline constexpr char TOPIC_CMDVEL[] = "/dev_web_ui/cmd_vel";
inline constexpr char TOPIC_ACTIVE[] = "/navigation/active_sender";
inline constexpr char TOPIC_CONNECT[] = "/dev_web_ui/connect";
} // namespace

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
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      lidar_front_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      lidar_rear_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

public:
  TopicVideoSource()
      : VideoSource(VideoSourceConfiguration::create(false, false)),
        Node("topic_video_source"), m_stopped(false),
        lidar_front_img(500, 500, CV_8UC3, cv::Scalar(0, 0, 0)),
        lidar_rear_img(500, 500, CV_8UC3, cv::Scalar(0, 0, 0)) {
    using std::placeholders::_1;

    {
      rclcpp::QoS qos(rclcpp::KeepLast(5));
      qos.best_effort();
      color_sub_ = create_subscription<sensor_msgs::msg::Image>(
          "/robot_interface/front_camera/color/image_raw", qos,
          std::bind(&TopicVideoSource::colorCallback, this, _1));
    }
    {
      rclcpp::QoS qos(rclcpp::KeepLast(5));
      qos.best_effort();
      depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
          "/robot_interface/front_camera/depth/image_rect_raw", qos,
          std::bind(&TopicVideoSource::depthCallback, this, _1));
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
          "/lts_ng/map", qos,
          std::bind(&TopicVideoSource::mapCallback, this, _1));
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
      const sensor_msgs::msg::PointCloud2::SharedPtr &msg, cv::Mat &img,
      const cv::Scalar &color) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    int img_size = img.rows;

    for (const auto &pt : cloud->points) {
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
  cv::Mat
  occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr &map_msg) {
    int width = map_msg->info.width;
    int height = map_msg->info.height;

    if (width == 0 || height == 0)
      return cv::Mat(); // empty image

    cv::Mat gray_img(height, width, CV_8UC1);
    for (size_t i = 0; i < map_msg->data.size(); ++i) {
      int8_t val = map_msg->data[i];
      uint8_t pixel = 0;

      if (val == -1)
        pixel = 127; // unknown
      else if (val == 0)
        pixel = 255; // free
      else
        pixel = 0; // occupied

      gray_img.data[i] = pixel;
    }

    std::vector<cv::Mat> channels(3, gray_img);
    cv::Mat map_img_bgr;
    cv::merge(channels, map_img_bgr);
    return map_img_bgr;
  }

  // ---- Callbacks ----
  void colorCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat img =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    color_image = std::move(img);
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat raw = cv_bridge::toCvCopy(msg)->image; // keep source encoding

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr rear(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *rear);
    Eigen::Matrix4f T_rear_to_front;
    T_rear_to_front << -1.f, 0.f, 0.f, -1.365f, -0.f, -1.f, 0.f, 0.245f, 0.f,
        0.f, 1.f, 0.000f, 0.f, 0.f, 0.f, 1.000f;
    pcl::PointCloud<pcl::PointXYZ> rear_tf;
    pcl::transformPointCloud(*rear, rear_tf, T_rear_to_front);
    cv::Mat img(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
    sensor_msgs::msg::PointCloud2 msg_tf;
    pcl::toROSMsg(rear_tf, msg_tf);
    drawPointCloudOnSharedImage(
        std::make_shared<sensor_msgs::msg::PointCloud2>(msg_tf), img,
        cv::Scalar(0, 0, 255));

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
    auto safe_resize = [&](const cv::Mat &src, cv::Mat &dst,
                           const cv::Size &sz) {
      if (sz.width > 0 && sz.height > 0 && !src.empty()) {
        cv::resize(src, dst, sz);
      } else {
        dst = src.clone();
      }
    };
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
        safe_resize(depth_colormap, depth_resized, tile_sz);
        cv::putText(depth_resized, "Depth", {10, 30}, cv::FONT_HERSHEY_SIMPLEX,
                    1, cv::Scalar(255, 255, 255), 2);
        cv::copyMakeBorder(depth_resized, padded_depth, 0, 0, 0, pad,
                           cv::BORDER_CONSTANT);
      } else {
        padded_depth = cv::Mat(tile_sz.height, tile_sz.width + pad, CV_8UC3,
                               cv::Scalar(50, 50, 50));
        cv::putText(padded_depth, "No Depth", {10, 30},
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 200), 2);
      }

      // LiDARs (both required for bottom row)
      auto colorize_mask = [](const cv::Mat &src, const cv::Scalar &color,
                              const cv::Size &sz) {
        if (src.empty())
          return cv::Mat();
        cv::Mat img = (src.channels() == 1) ? cv::Mat() : src.clone();
        if (src.channels() == 1)
          cv::cvtColor(src, img, cv::COLOR_GRAY2BGR);
        if (sz.width > 0 && sz.height > 0)
          cv::resize(img, img, sz, 0, 0, cv::INTER_NEAREST);
        cv::Mat gray, mask, out(img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, mask, 1, 255, cv::THRESH_BINARY);
        out.setTo(color, mask);
        return out;
      };

      cv::Mat lidar_front_col =
          colorize_mask(lidar_front_img, cv::Scalar(0, 255, 0), tile_sz);
      cv::Mat lidar_rear_col =
          colorize_mask(lidar_rear_img, cv::Scalar(0, 0, 255), tile_sz);
      cv::Mat lidar_combined;
      if (!lidar_front_col.empty() && !lidar_rear_col.empty()) {
        cv::max(lidar_front_col, lidar_rear_col, lidar_combined);
      } else if (!lidar_front_col.empty()) {
        lidar_combined = lidar_front_col;
      } else if (!lidar_rear_col.empty()) {
        lidar_combined = lidar_rear_col;
      } else {
        lidar_combined = cv::Mat(tile_sz.height, tile_sz.width, CV_8UC3,
                                 cv::Scalar(50, 50, 50));
        cv::putText(lidar_combined, "No LiDAR", {10, 30},
                    cv::FONT_HERSHEY_SIMPLEX, 1, {200, 200, 200}, 2);
      }
      cv::circle(lidar_combined, {tile_sz.width / 2, tile_sz.height / 2}, 6,
                 {255, 255, 255}, -1, cv::LINE_AA);
      cv::putText(lidar_combined, "LiDAR Front", {10, 30},
                  cv::FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0}, 2);
      cv::putText(lidar_combined, "LiDAR Rear", {10, 60},
                  cv::FONT_HERSHEY_SIMPLEX, 1, {0, 0, 255}, 2);

      cv::Mat padded_color, padded_lidar;
      cv::copyMakeBorder(rgb_labeled, padded_color, 0, 0, 0, pad,
                         cv::BORDER_CONSTANT);
      cv::copyMakeBorder(lidar_combined, padded_lidar, 0, 0, 0, pad,
                         cv::BORDER_CONSTANT);

      // Map (optional)
      cv::Mat padded_map;
      if (!map_image.empty() && map_image.cols > 0 && map_image.rows > 0) {

        cv::Mat map_resized;
        cv::resize(map_image, map_resized, tile_sz, 0, 0, cv::INTER_NEAREST);
        cv::copyMakeBorder(map_resized, padded_map, 0, 0, 0, pad,
                           cv::BORDER_CONSTANT);
      } else {
        padded_map =
            cv::Mat::zeros(tile_sz.height, tile_sz.width + pad, CV_8UC3);
      }

      // Compose rows
      cv::Mat top_row, bottom_row;
      cv::hconcat(std::vector<cv::Mat>{padded_color, padded_depth}, top_row);
      cv::hconcat(std::vector<cv::Mat>{padded_lidar, padded_map}, bottom_row);
      // Final frame
      cv::Mat final_combined;
      cv::vconcat(top_row, bottom_row, final_combined);

      cv::putText(final_combined, "Frame: " + std::to_string(frame_count++),
                  {20, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                  cv::Scalar(255, 255, 255), 2);

      // Send
      int64_t timestamp_us =
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now().time_since_epoch())
              .count();

      sendFrame(final_combined, timestamp_us);
      std::cout << "[DEBUG] Sending frame size: " << final_combined.cols << "x"
                << final_combined.rows
                << " channels=" << final_combined.channels() << std::endl;
    }
  }
};

class TopicDataChannel : public rclcpp::Node {
public:
  TopicDataChannel() : rclcpp::Node("webrtc_ros_bridgeless") {
    auto qos_latched =
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();

    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>(
        "/dev_web_ui/cmd_vel", qos_latched);
    pub_connect_ =
        create_publisher<std_msgs::msg::Bool>(TOPIC_CONNECT, qos_latched);
    sub_active_ = create_subscription<std_msgs::msg::String>(
        TOPIC_ACTIVE, rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr) {});
    RCLCPP_INFO(get_logger(), "ROS pubs ready: %s, %s, %s", TOPIC_CMDVEL,
                TOPIC_ACTIVE, TOPIC_CONNECT);
  }

  void handleJsonMessage(const std::string &s) {
    json j;
    try {
      j = json::parse(s);
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "Bad JSON: %s", e.what());
      return;
    }

    if (!j.contains("op") || j["op"] != "publish" || !j.contains("topic"))
      return;

    const std::string topic = j["topic"].get<std::string>();

    if (topic == TOPIC_CMDVEL) {
      try {
        const auto &msgj = j.at("msg");
        geometry_msgs::msg::Twist m;
        m.linear.x = msgj.at("linear").value("x", 0.0);
        m.linear.y = msgj.at("linear").value("y", 0.0);
        m.linear.z = msgj.at("linear").value("z", 0.0);
        m.angular.x = msgj.at("angular").value("x", 0.0);
        m.angular.y = msgj.at("angular").value("y", 0.0);
        m.angular.z = msgj.at("angular").value("z", 0.0);
        pub_cmd_->publish(m);
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "Twist parse error: %s", e.what());
      }
    } else if (topic == TOPIC_CONNECT) {
      try {
        const auto &msgj = j.at("msg");
        std_msgs::msg::Bool out;
        if (msgj.is_boolean())
          out.data = msgj.get<bool>();
        else if (msgj.is_number())
          out.data = (msgj.get<int>() != 0);
        else if (msgj.is_string())
          out.data = (msgj.get<std::string>() == "true");
        else
          out.data = false;
        pub_connect_->publish(out);
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "Connect parse error: %s", e.what());
      }
    }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_connect_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_active_;
};
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  const char *SIGNALING_URL = "ws://localhost:8081/signaling";
  std::vector<IceServer> iceServers = {IceServer(
      "turn:424-iwhub.idealworks.com:3478?transport=udp", "webrtc", "abc123")};
  auto webrtcCfg = WebrtcConfiguration::create(iceServers);
  auto videoCfg = VideoStreamConfiguration::create();

  auto videoNode = std::make_shared<TopicVideoSource>();
  auto dataNode = std::make_shared<TopicDataChannel>();

  auto sigVideo = SignalingServerConfiguration::create(
      SIGNALING_URL, "CppClient-Video", "chat", "abc");
  auto sigData = SignalingServerConfiguration::create(
      SIGNALING_URL, "CppClient-Data", "chat", "abc");

  StreamClient streamClient(sigVideo, webrtcCfg, videoCfg, videoNode, nullptr);
  auto dcCfg = DataChannelConfiguration::create();
  DataChannelClient dataClient(sigData, webrtcCfg, dcCfg);

  streamClient.setLogger(
      [](const std::string &m) { std::cout << "[STREAM] " << m << "\n"; });
  streamClient.setOnSignalingConnectionOpened(
      []() { std::cout << "Video signaling opened.\n"; });
  streamClient.setOnSignalingConnectionClosed(
      []() { std::cout << "Video signaling closed.\n"; });
  streamClient.setOnSignalingConnectionError([](const std::string &e) {
    std::cout << "Video signaling error: " << e << "\n";
  });
  streamClient.setOnRoomClientsChanged([](const std::vector<RoomClient> &cs) {
    std::cout << "[STREAM] Room clients changed (" << cs.size() << ")\n";
  });
  streamClient.setOnClientConnected([](const Client &c) {
    std::cout << "[STREAM] connected: " << c.name() << "\n";
  });
  streamClient.setOnClientDisconnected([](const Client &c) {
    std::cout << "[STREAM] disconnected: " << c.name() << "\n";
  });
  streamClient.setOnClientConnectionFailed([](const Client &c) {
    std::cout << "[STREAM] failed: " << c.name() << "\n";
  });
  streamClient.setOnAddRemoteStream([](const Client &c) {
    std::cout << "[STREAM] remote added: " << c.name() << "\n";
  });
  streamClient.setOnRemoveRemoteStream([](const Client &c) {
    std::cout << "[STREAM] remote removed: " << c.name() << "\n";
  });
  streamClient.setOnVideoFrameReceived(
      [](const Client &c, const cv::Mat &, uint64_t) {
        std::cout << "[STREAM] frame from: " << c.name() << "\n";
      });

  dataClient.setLogger(
      [](const std::string &m) { std::cout << "[DATA] " << m << "\n"; });
  dataClient.setOnSignalingConnectionOpened(
      []() { std::cout << "Data signaling opened.\n"; });
  dataClient.setOnSignalingConnectionClosed(
      []() { std::cout << "Data signaling closed.\n"; });
  dataClient.setOnSignalingConnectionError([](const std::string &e) {
    std::cout << "Data signaling error: " << e << "\n";
  });
  dataClient.setOnRoomClientsChanged([](const std::vector<RoomClient> &cs) {
    std::cout << "[DATA] Room clients changed (" << cs.size() << ")\n";
  });
  dataClient.setOnDataChannelOpened([](const Client &c) {
    std::cout << "[DATA] channel opened: " << c.name() << "\n";
  });
  dataClient.setOnDataChannelClosed([](const Client &c) {
    std::cout << "[DATA] channel closed: " << c.name() << "\n";
  });
  dataClient.setOnDataChannelError([](const Client &c, const std::string &e) {
    std::cout << "[DATA] channel error from " << c.name() << ": " << e << "\n";
  });
  dataClient.setOnDataChannelMessageString(
      [&](const Client &c, const std::string &msg) {
        std::cout << "[DATA] from " << c.name() << ": " << msg << "\n";
        dataNode->handleJsonMessage(msg);
      });

  streamClient.connect();
  dataClient.connect();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(videoNode);
  exec.add_node(dataNode);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}

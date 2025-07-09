#include <OpenteraWebrtcNativeClient/StreamClient.h>


#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <nav_msgs/msg/occupancy_grid.hpp>


#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <atomic>
#include <string>

using namespace opentera;
using namespace std::chrono_literals;

class BagVideoSource : public VideoSource
{
    std::atomic_bool m_stopped;
    std::thread m_thread;
    std::string m_bag_path;

public:
    BagVideoSource(std::string bag_path)
        : VideoSource(VideoSourceConfiguration::create(false, true)),
          m_stopped(false),
          m_thread(&BagVideoSource::run, this),
          m_bag_path(std::move(bag_path))
    {
    }

    ~BagVideoSource() override
    {
        m_stopped.store(true);
        m_thread.join();
    }

    // transforms the point clouds into top view images to be displayed
    enum class LiDARType
    {
        Front,
        Rear,
        LaserScan3D
    };

    // Modified function
    void drawPointCloudOnSharedImage(
        const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
        cv::Mat& img,
        const cv::Scalar& color)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        int img_size = img.rows;

        for (const auto& pt : cloud->points)
        {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y))
                continue;

            int x = static_cast<int>(pt.x * 20 + img_size / 2);
            int y = static_cast<int>(pt.y * 20 + img_size / 2);


            if (x >= 0 && x < img_size && y >= 0 && y < img_size)
            {
                cv::circle(img, cv::Point(x, img_size - y - 1), 1, color, -1);
            }
        }
    }


    // Transforms occupancy grid into grayscale opencv image
    cv::Mat occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr& map_msg)
    {
        int width = map_msg->info.width;
        int height = map_msg->info.height;

        cv::Mat map_img(height, width, CV_8UC1);

        for (int i = 0; i < height * width; ++i)
        {
            int8_t val = map_msg->data[i];
            uint8_t pixel;
            if (val == -1)
                pixel = 127;  // Unknown -> gray
            else if (val == 0)
                pixel = 255;  // Free -> white
            else
                pixel = 0;  // Occupied -> black

            map_img.at<uint8_t>(i / width, i % width) = pixel;
        }

        return map_img;
    }

private:
    void run()
    {
        rclcpp::init(0, nullptr);

        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = m_bag_path;
        storage_options.storage_id = "sqlite3";

        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";

        int frame_count = 0;

        while (!m_stopped)
        {
            rosbag2_cpp::Reader reader;
            try
            {
                reader.open(storage_options, converter_options);
            }
            catch (const std::exception& e)
            {
                std::cerr << "Failed to open bag file: " << e.what() << std::endl;
                break;
            }

            int frame_count = 0;
            cv::Mat color_image, depth_colormap, map_image;
            cv::Mat lidar_front_img(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
            cv::Mat lidar_rear_img(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));

            bool got_front = false, got_rear = false, got_laser = false;

            rclcpp::Time last_msg_time;
            bool first_msg = true;

            while (!m_stopped && reader.has_next())
            {
                auto bag_msg = reader.read_next();

                rclcpp::Time current_msg_time(bag_msg->time_stamp);

                if (!first_msg)
                {
                    auto time_diff = current_msg_time - last_msg_time;
                    int64_t sleep_us = time_diff.nanoseconds() / 1000;
                    if (sleep_us > 0 && sleep_us < 10'000'000)
                        std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
                }
                last_msg_time = current_msg_time;
                first_msg = false;

                if (bag_msg->topic_name == "/robot_interface/front_camera/color/resize/image_raw/compressed")
                {
                    auto comp_img_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
                    rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
                    rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serializer;
                    serializer.deserialize_message(&serialized_msg, comp_img_msg.get());
                    color_image = cv::imdecode(cv::Mat(comp_img_msg->data), cv::IMREAD_COLOR);
                }
                else if (
                    bag_msg->topic_name == "/robot_interface/front_camera/depth/resize/image_rect_raw/compressedDepth")
                {
                    auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
                    rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
                    rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
                    serializer.deserialize_message(&serialized_msg, img_msg.get());
                    cv::Mat raw_image = cv_bridge::toCvCopy(img_msg)->image;
                    if (raw_image.type() == CV_32FC1)
                    {
                        cv::Mat normalized;
                        cv::normalize(raw_image, normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
                        cv::applyColorMap(normalized, depth_colormap, cv::COLORMAP_JET);
                    }
                }
                else if (bag_msg->topic_name == "/robot_interface/scan_front_cloud")
                {
                    auto pc_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                    rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
                    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
                    serializer.deserialize_message(&serialized_msg, pc_msg.get());
                    drawPointCloudOnSharedImage(pc_msg, lidar_front_img, cv::Scalar(0, 255, 0));
                    got_front = true;
                }
                else if (bag_msg->topic_name == "/robot_interface/scan_rear_cloud")
                {
                    auto pc_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                    rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
                    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
                    serializer.deserialize_message(&serialized_msg, pc_msg.get());
                    drawPointCloudOnSharedImage(pc_msg, lidar_rear_img, cv::Scalar(0, 0, 255));
                    got_rear = true;
                }
                else if (bag_msg->topic_name == "/map")
                {
                    auto map_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
                    rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
                    rclcpp::Serialization<nav_msgs::msg::OccupancyGrid> serializer;
                    serializer.deserialize_message(&serialized_msg, map_msg.get());
                    map_image = occupancyGridToImage(map_msg);
                }

                // if (!color_image.empty() && !depth_colormap.empty() && got_front && got_rear)
                if (!color_image.empty() && got_front && got_rear)
                {
                    // Labels
                    cv::putText(
                        color_image,
                        "RGB",
                        {10, color_image.rows - 10},
                        cv::FONT_HERSHEY_SIMPLEX,
                        1,
                        cv::Scalar(255, 255, 255),
                        2);
                    // cv::putText(depth_colormap, "Depth", {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1,
                    // cv::Scalar(255,255,255), 2);
                    cv::putText(
                        lidar_front_img,
                        "LiDAR Front",
                        {10, 30},
                        cv::FONT_HERSHEY_SIMPLEX,
                        1,
                        cv::Scalar(0, 255, 0),
                        2);
                    cv::putText(
                        lidar_rear_img,
                        "LiDAR Rear",
                        {10, 30},
                        cv::FONT_HERSHEY_SIMPLEX,
                        1,
                        cv::Scalar(0, 0, 255),
                        2);

                    // Padding
                    int pad = 10;
                    cv::Mat padded_color, padded_depth, padded_map;
                    cv::Mat padded_lidar_front, padded_lidar_rear;

                    // Resize
                    if (!depth_colormap.empty())
                    {
                        cv::resize(depth_colormap, depth_colormap, color_image.size());
                        cv::putText(
                            depth_colormap,
                            "Depth",
                            {10, 30},
                            cv::FONT_HERSHEY_SIMPLEX,
                            1,
                            cv::Scalar(255, 255, 255),
                            2);
                        cv::copyMakeBorder(depth_colormap, padded_depth, 0, 0, 0, pad, cv::BORDER_CONSTANT);
                    }
                    else
                    {
                        // Create a placeholder image with "No Depth"
                        padded_depth =
                            cv::Mat(color_image.rows, color_image.cols + pad, CV_8UC3, cv::Scalar(50, 50, 50));
                        cv::putText(
                            padded_depth,
                            "No Depth",
                            {10, 30},
                            cv::FONT_HERSHEY_SIMPLEX,
                            1,
                            cv::Scalar(200, 200, 200),
                            2);
                    }

                    cv::resize(lidar_front_img, lidar_front_img, color_image.size());
                    cv::resize(lidar_rear_img, lidar_rear_img, color_image.size());

                    cv::copyMakeBorder(color_image, padded_color, 0, 0, 0, pad, cv::BORDER_CONSTANT);
                    // cv::copyMakeBorder(depth_colormap, padded_depth, 0, 0, 0, pad, cv::BORDER_CONSTANT);
                    cv::copyMakeBorder(lidar_front_img, padded_lidar_front, 0, 0, 0, pad, cv::BORDER_CONSTANT);
                    cv::copyMakeBorder(lidar_rear_img, padded_lidar_rear, 0, 0, 0, pad, cv::BORDER_CONSTANT);

                    if (!map_image.empty())
                    {
                        cv::resize(map_image, map_image, color_image.size());

                        if (map_image.channels() == 1)
                        {
                            cv::cvtColor(map_image, map_image, cv::COLOR_GRAY2BGR);
                        }

                        cv::putText(
                            map_image,
                            "Map",
                            {10, 30},
                            cv::FONT_HERSHEY_SIMPLEX,
                            1,
                            cv::Scalar(255, 255, 255),
                            2);
                        cv::copyMakeBorder(map_image, padded_map, 0, 0, 0, pad, cv::BORDER_CONSTANT);
                    }

                    else
                    {
                        padded_map = cv::Mat::zeros(padded_depth.size(), padded_depth.type());
                        cv::putText(
                            padded_map,
                            "No Map",
                            {10, 30},
                            cv::FONT_HERSHEY_SIMPLEX,
                            1,
                            cv::Scalar(100, 100, 100),
                            2);
                    }

                    // Rows
                    cv::Mat top_row, bottom_row;
                    cv::hconcat(std::vector<cv::Mat>{padded_color, padded_depth, padded_map}, top_row);
                    cv::Mat empty_pad = cv::Mat::zeros(padded_map.size(), padded_map.type());
                    cv::hconcat(std::vector<cv::Mat>{padded_lidar_front, padded_lidar_rear, empty_pad}, bottom_row);


                    // Final frame
                    cv::Mat final_combined;
                    cv::vconcat(top_row, bottom_row, final_combined);

                    cv::putText(
                        final_combined,
                        "Frame: " + std::to_string(frame_count++),
                        {20, 30},
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.8,
                        cv::Scalar(255, 255, 255),
                        2);

                    // Send
                    int64_t timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                               std::chrono::steady_clock::now().time_since_epoch())
                                               .count();
                    sendFrame(final_combined, timestamp_us);

                    // Reset
                    color_image.release();
                    depth_colormap.release();
                    lidar_front_img = cv::Mat(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
                    lidar_rear_img = cv::Mat(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
                    got_front = got_rear = false;

                    //std::this_thread::sleep_for(233ms);
                }
            }
        }

        rclcpp::shutdown();
    }
};

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cout << "Usage: CppStreamClient <bag_file_path>" << std::endl;
        return EXIT_FAILURE;
    }

    std::string bag_path = argv[1];

    std::vector<IceServer> iceServers;
    if (!IceServer::fetchFromServer("http://localhost:8080/iceservers", "abc", iceServers))
    {
        std::cout << "IceServer::fetchFromServer failed" << std::endl;
        iceServers.clear();
    }

    auto signalingServerConfiguration =
        SignalingServerConfiguration::create("ws://localhost:8080/signaling", "CppBagClient", "chat", "abc");

    auto webrtcConfiguration = WebrtcConfiguration::create(iceServers);
    auto videoStreamConfiguration = VideoStreamConfiguration::create();
    auto videoSource = std::make_shared<BagVideoSource>(bag_path);

    StreamClient client(
        signalingServerConfiguration,
        webrtcConfiguration,
        videoStreamConfiguration,
        videoSource,
        nullptr);  // no audio source

    client.setLogger([](const std::string& message) { std::cout << "[LOG] " << message << std::endl; });

    client.setOnSignalingConnectionOpened([]() { std::cout << "Signaling connection opened." << std::endl; });

    client.setOnSignalingConnectionClosed([]() { std::cout << "Signaling connection closed." << std::endl; });

    client.setOnSignalingConnectionError([](const std::string& error)
                                         { std::cout << "Signaling connection error: " << error << std::endl; });

    client.setOnRoomClientsChanged(
        [](const std::vector<RoomClient>& roomClients)
        {
            std::cout << "Room clients changed:" << std::endl;
            for (const auto& c : roomClients)
            {
                std::cout << "\tID=" << c.id() << ", Name=" << c.name()
                          << ", Connected=" << (c.isConnected() ? "yes" : "no") << std::endl;
            }
        });

    client.setOnClientConnected(
        [](const Client& client)
        {
            std::cout << "Client connected: ID=" << client.id() << ", Name=" << client.name() << std::endl;
            cv::namedWindow(client.id(), cv::WINDOW_AUTOSIZE);
        });

    client.setOnClientDisconnected(
        [](const Client& client)
        {
            std::cout << "Client disconnected: ID=" << client.id() << ", Name=" << client.name() << std::endl;
            cv::destroyWindow(client.id());
        });

    client.setOnClientConnectionFailed(
        [](const Client& client)
        { std::cout << "Client connection failed: ID=" << client.id() << ", Name=" << client.name() << std::endl; });

    client.setOnAddRemoteStream(
        [](const Client& client)
        { std::cout << "Remote stream added: ID=" << client.id() << ", Name=" << client.name() << std::endl; });

    client.setOnRemoveRemoteStream(
        [](const Client& client)
        { std::cout << "Remote stream removed: ID=" << client.id() << ", Name=" << client.name() << std::endl; });

    client.setOnVideoFrameReceived(
        [](const Client& client, const cv::Mat& bgrImg, uint64_t)
        {
            std::cout << "Video frame received from: " << client.id() << std::endl;
            cv::imshow(client.id(), bgrImg);
            cv::waitKey(1);
        });

    client.connect();

    std::cin.get();
    return 0;
}

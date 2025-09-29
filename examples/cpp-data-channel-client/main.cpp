#include <OpenteraWebrtcNativeClient/DataChannelClient.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <nlohmann/json.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <thread>

using nlohmann::json;
using namespace opentera;
using namespace std;

static constexpr const char* TOPIC_CMDVEL  = "/cockpit/cmd_vel";
static constexpr const char* TOPIC_ACTIVE  = "/navigation/active_sender";
static constexpr const char* TOPIC_CONNECT = "/cockpit/connect";

class RosIo : public rclcpp::Node
{
public:
    RosIo() : rclcpp::Node("webrtc_ros_bridgeless")
    {

        auto qos_stream  = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
        auto qos_latched = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();

        pub_cmd_     = create_publisher<geometry_msgs::msg::Twist>(TOPIC_CMDVEL,  qos_stream);
        pub_active_  = create_publisher<std_msgs::msg::String>(TOPIC_ACTIVE,      qos_latched);
        pub_connect_ = create_publisher<std_msgs::msg::Bool>(TOPIC_CONNECT,       qos_latched);

        RCLCPP_INFO(get_logger(), "ROS pubs ready: %s, %s, %s",
                    TOPIC_CMDVEL, TOPIC_ACTIVE, TOPIC_CONNECT);
    }

    void handleJsonMessage(const std::string& s)
    {
        json j;
        try { j = json::parse(s); }
        catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Bad JSON: %s", e.what());
            return;
        }

        if (!j.contains("op") || j["op"] != "publish" || !j.contains("topic"))
            return;

        const std::string topic = j["topic"].get<std::string>();

        if (topic == TOPIC_CMDVEL) {
            try {
                const auto& msgj = j.at("msg");
                geometry_msgs::msg::Twist m;
                m.linear.x  = msgj.at("linear").value("x", 0.0);
                m.linear.y  = msgj.at("linear").value("y", 0.0);
                m.linear.z  = msgj.at("linear").value("z", 0.0);
                m.angular.x = msgj.at("angular").value("x", 0.0);
                m.angular.y = msgj.at("angular").value("y", 0.0);
                m.angular.z = msgj.at("angular").value("z", 0.0);
                pub_cmd_->publish(m);
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "Twist parse error: %s", e.what());
            }
        }
        else if (topic == TOPIC_ACTIVE) {
            try {
                const auto& msgj = j.at("msg");
                std_msgs::msg::String out;
                out.data = msgj.dump();
                pub_active_->publish(out);
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "Active parse error: %s", e.what());
            }
        }
        else if (topic == TOPIC_CONNECT) {
            try {
                const auto& msgj = j.at("msg");
                std_msgs::msg::Bool out;
                if (msgj.is_boolean())      out.data = msgj.get<bool>();
                else if (msgj.is_number())  out.data = (msgj.get<int>() != 0);
                else if (msgj.is_string())  out.data = (msgj.get<std::string>() == "true");
                else                        out.data = false;
                pub_connect_->publish(out);
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "Connect parse error: %s", e.what());
            }
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr      pub_active_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr        pub_connect_;
};

int main(int argc, char* argv[])
{
    const char* SIGNALING_URL = "ws://localhost:8081/signaling";
    std::vector<IceServer> iceServers = {
        IceServer("turn:424-iwhub.idealworks.com:3478?transport=udp",
                  "webrtc", "abc123")
    };

    auto signalingServerConfiguration =
        SignalingServerConfiguration::create(SIGNALING_URL, "C++", "chat", "abc");

    auto webrtcConfiguration = WebrtcConfiguration::create(iceServers);

    
     auto dataChannelConfiguration = DataChannelConfiguration::create();
    DataChannelClient client(signalingServerConfiguration, webrtcConfiguration, dataChannelConfiguration);


    rclcpp::init(argc, argv);
    auto ros = std::make_shared<RosIo>();
    std::thread rosThread([&](){
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(ros);
        exec.spin();
        exec.remove_node(ros);
    });

    client.setOnSignalingConnectionOpened([](){ std::cout << "OnSignalingConnectionOpened\n"; });
    client.setOnSignalingConnectionClosed([](){ std::cout << "OnSignalingConnectionClosed\n"; });
    client.setOnSignalingConnectionError([](const std::string& error){
        std::cout << "OnSignalingConnectionError:\n\t" << error << std::endl;
    });
    client.setOnRoomClientsChanged([](const std::vector<RoomClient>& roomClients){
        std::cout << "OnRoomClientsChanged:\n";
        for (const auto& c : roomClients) {
            std::cout << "\tid=" << c.id() << ", name=" << c.name()
                      << ", isConnected=" << c.isConnected() << "\n";
        }
    });
    client.setOnClientConnected([](const Client& c){
        std::cout << "OnClientConnected:\n\tid=" << c.id() << ", name=" << c.name() << "\n";
    });
    client.setOnClientDisconnected([](const Client& c){
        std::cout << "OnClientDisconnected:\n\tid=" << c.id() << ", name=" << c.name() << "\n";
    });
    client.setOnClientConnectionFailed([](const Client& c){
        std::cout << "OnClientConnectionFailed:\n\tid=" << c.id() << ", name=" << c.name() << "\n";
    });
    client.setOnError([](const std::string& error){ std::cout << "error:\n\t" << error << "\n"; });
    client.setLogger([](const std::string& message){ std::cout << "log:\n\t" << message << "\n"; });

    client.setOnDataChannelOpened([](const Client& c){
        std::cout << "OnDataChannelOpened:\n\tid=" << c.id() << ", name=" << c.name() << "\n";
    });
    client.setOnDataChannelClosed([](const Client& c){
        std::cout << "OnDataChannelClosed:\n\tid=" << c.id() << ", name=" << c.name() << "\n";
    });
    client.setOnDataChannelError([](const Client& c, const std::string& error){
        std::cout << "OnDataChannelError:\n\tid=" << c.id() << ", name=" << c.name()
                  << "\n\t" << error << "\n";
    });
    client.setOnDataChannelMessageString([&](const Client& c, const std::string& message){
        std::cout << "setOnDataChannelMessageString:\n\tfrom=" << c.name()
                  << "\n\t" << message << "\n";
        ros->handleJsonMessage(message);
    });

    client.connect();
    std::cin.get();

    rclcpp::shutdown();
    if (rosThread.joinable()) rosThread.join();
    return 0;
}

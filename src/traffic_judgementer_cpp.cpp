// SPDX-FileCopyrightText: 2024 Shuma Suzuki
// SPDX-LIcense-Identifier: BSD-3-Clause
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <thread>
#include <string>
#include <fstream>

class TrafficJudgementerCpp : public rclcpp::Node {
public:
    TrafficJudgementerCpp() : Node("traffic_judgmenter"), traffic_request_(false) {
        // サブスクリプション
        current_waypoint_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "waypoint_manager2/current_waypoint", 1,
            std::bind(&TrafficJudgementerCpp::current_waypoint_callback, this, std::placeholders::_1));

        traffic_yolov8_sub_ = this->create_subscription<std_msgs::msg::String>(
            "current_traffic_output", 1,
            std::bind(&TrafficJudgementerCpp::current_traffic_output_callback, this, std::placeholders::_1));

        // クライアント
        client_ = this->create_client<std_srvs::srv::Trigger>("waypoint_manager2/next_wp");

        // Crossing points データの読み込み
        std::ifstream file(CROSSPOINT_PATH);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open crossing points file: %s", CROSSPOINT_PATH);
            rclcpp::shutdown();
            return;
        }

        YAML::Node config = YAML::Load(file);
        crossing_points_numbers_ = config["crossing_point_numbers"].as<std::vector<int>>();

        RCLCPP_INFO(this->get_logger(), "TrafficJudgementerCpp node initialized.");
    }

private:
    // 定数
    //static constexpr const char* CROSSPOINT_PATH = "/root/turtlebot3_ws/src/traffic_topic/config/crossing_points/test.yaml";
    static constexpr const char* CROSSPOINT_PATH = "/home/orne-box/orne_ws/src/cross_traffic_manager_cpp/config/crossing_points/tukuba2024.yaml";

    // メンバ変数
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr current_waypoint_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr traffic_yolov8_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    std::vector<int> crossing_points_numbers_;
    int current_waypoint_msg_ = -1;
    std::string traffic_msg_;
    bool traffic_request_;

    // コールバック関数
    void current_waypoint_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        current_waypoint_msg_ = msg->data;
        traffic_judgment();
    }

    void current_traffic_output_callback(const std_msgs::msg::String::SharedPtr msg) {
        traffic_msg_ = msg->data;
        traffic_judgment();
    }

    // 判定ロジック
    void traffic_judgment() {
        if (current_waypoint_msg_ != -1 && !traffic_msg_.empty()) {
            if (std::find(crossing_points_numbers_.begin(), crossing_points_numbers_.end(), current_waypoint_msg_) != crossing_points_numbers_.end() &&
                traffic_msg_.find("Blue") != std::string::npos && !traffic_request_) {
                
                traffic_request_ = true;
                RCLCPP_INFO(this->get_logger(), "Crossing condition met. Sending request...");
                std::this_thread::sleep_for(std::chrono::seconds(3));
                send_request();
                std::this_thread::sleep_for(std::chrono::seconds(2));
            } else {
                RCLCPP_INFO(this->get_logger(), "Waiting for crossing chance...");
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Waiting for start publisher...");
        }
    }

    // サービスリクエストの送信
    void send_request() {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

        auto future = client_->async_send_request(request, std::bind(&TrafficJudgementerCpp::callback_response, this, std::placeholders::_1));

        traffic_request_ = false;
    }

    // サービス応答の処理
    void callback_response(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Response: success=%d, message=\"%s\"",
                        response->success, response->message.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrafficJudgementerCpp>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


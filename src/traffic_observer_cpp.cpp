// SPDX-FileCopyrightText: 2024 Shuma Suzuki
// SPDX-LIcense-Identifier: BSD-3-Clause
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cstdio>
#include <cstring>
#include <thread>
#include <iostream>

class TrafficObserverCpp : public rclcpp::Node {
public:
    TrafficObserverCpp()
        : Node("traffic_observer"), stop_requested_(false) {
        // ROS2 Publisherの初期化
        publisher_ = this->create_publisher<std_msgs::msg::String>("current_traffic_output", 1);

        // popenでスクリプトを実行
        script_thread_ = std::thread(&TrafficObserverCpp::execute_script, this);
    }

    ~TrafficObserverCpp() {
        stop_requested_ = true;  // スレッド停止をリクエスト
        if (script_thread_.joinable()) {
            script_thread_.join();  // スレッドの終了を待機
        }
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::thread script_thread_;
    bool stop_requested_;

    void execute_script() {
        // 実行するスクリプトのパス
        const char *command = "/home/orne-box/traffic_shell/connect_yolov8.sh";
        
        // popenを使用してスクリプトを実行
        FILE *pipe = popen(command, "r");
        if (!pipe) {
            RCLCPP_ERROR(this->get_logger(), "Failed to run script: %s", command);
            return;
        }

        char buffer[128];
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            if (stop_requested_) {
                break;  // 停止リクエストがあれば終了
            }

            // 出力をROS2メッセージとして送信
            auto message = std_msgs::msg::String();
            message.data = std::string(buffer);
            publisher_->publish(message);

            // ログとしても出力
            RCLCPP_INFO(this->get_logger(), "Published: %s", message.data.c_str());
        }

        // popenの終了処理
        int return_code = pclose(pipe);
        RCLCPP_INFO(this->get_logger(), "Script exited with code: %d", return_code);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TrafficObserverCpp>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}










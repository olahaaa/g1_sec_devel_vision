#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <g1/g1_loco_client.hpp>
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

class LocoClientNode : public rclcpp::Node{
public:
    LocoClientNode(): Node("loco_client_node"), client_(this){
        this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 10,
            [this](tf2_msgs::msg::TFMessage::SharedPtr msg){
                this->callback(msg);
            });
        thread_ = std::thread([this]{
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            process(distance_, orientation_);
        });
        client_.SetFsmId(1);
    }

private:
    unitree::robot::g1::LocoClient client_;
    std::thread thread_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;

    void process(double distance, double orientation){
        //调用g1_client控制运动
    };
    void callback(const tf2_msgs::msg::TFMessage::SharedPtr msg){
        for (const auto &transform : msg->transforms) {
            if (transform.child_frame_id == "tag36h11:1") {
                distance_ = transform.transform.translation.z;
                //orientation_ = transform.transform.rotation.z; 这是四元数，需要转换
            }
        }
    };

    double distance_;
    double orientation_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocoClientNode>());
    rclcpp::shutdown();
    return 0;
}
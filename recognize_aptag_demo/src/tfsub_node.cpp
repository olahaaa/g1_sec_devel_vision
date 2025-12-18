#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

class TFSubNode: public rclcpp::Node {
public:
    TFSubNode(): Node("tfsub_node") {
        subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 10,
            std::bind(&TFSubNode::topic_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "TFSubNode started...");
        }

private:
  void topic_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)

  {

    RCLCPP_INFO(this->get_logger(), "Received %zu transform(s)", msg->transforms.size());
    // 遍历所有变换并打印详细信息
    for (const auto& transform : msg->transforms) {
      RCLCPP_INFO(this->get_logger(), 
        "\nTransform from '%s' to '%s':\n"
        "  Timestamp: %d.%09d\n"
        "  Translation: (%f, %f, %f)\n"
        "  Rotation (quaternion): (%f, %f, %f, %f)",
        transform.header.frame_id.c_str(),
        transform.child_frame_id.c_str(),
        transform.header.stamp.sec,
        transform.header.stamp.nanosec,
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z,
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
    }

  }
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFSubNode>());
    rclcpp::shutdown();
    return 0;
}
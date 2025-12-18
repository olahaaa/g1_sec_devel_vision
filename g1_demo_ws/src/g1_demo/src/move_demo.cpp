#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <cmath>
#include <g1/g1_loco_client.hpp> // 确保包含 G1 SDK 头文件
#include "tf2_msgs/msg/tf_message.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class LocoClientNode : public rclcpp::Node{
public:
    LocoClientNode(): Node("loco_client_node"), client_(this){
        // 创建订阅 /tf 话题的订阅者
        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 10,
            [this](tf2_msgs::msg::TFMessage::SharedPtr msg){
                this->callback(msg);
            });

        // 创建一个独立的线程用于周期性地处理运动控制逻辑
        control_thread_ = std::thread([this](){
            while(rclcpp::ok() && !stop_thread_flag_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 每100ms处理一次
                std::lock_guard<std::mutex> lock(data_mutex_); // 锁定数据访问
                process(last_distance_, last_heading_deviation_);
            }
        });


        RCLCPP_INFO(this->get_logger(), "LocoClientNode initialized.");
    }

    // 析构函数，确保线程正确结束
    ~LocoClientNode() {
        stop_thread_flag_ = true;
        if (control_thread_.joinable()) {
            control_thread_.join();
        }
    }

private:
    unitree::robot::g1::LocoClient client_; // Unitree G1 运动控制客户端
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    
    std::thread control_thread_;           // 控制线程
    std::atomic<bool> stop_thread_flag_{false}; // 线程停止标志
    std::mutex data_mutex_;               // 保护共享数据的互斥锁

    // 存储从 /tf 回调中获取的最新数据
    double last_distance_ = -1.0;          // 初始化为无效值
    double last_heading_deviation_ = 0.0;  // 初始化为无偏差

    /**
     * @brief 根据距离和航向偏差控制机器人运动
     * 
     * @param distance 到 AprilTag 的 Z 轴距离 (米)
     * @param heading_deviation AprilTag 相对于相机 Z 轴的航向偏差角 (弧度)，由 pitch 角获得
     */
    void process(double distance, double heading_deviation){
        
        // 检查是否检测到了有效的 AprilTag 数据
        if (distance <= 0.0) {  
             RCLCPP_INFO(this->get_logger(), "No valid AprilTag data available or distance invalid.");
             // 停止机器人
             client_.StopMove(); // 使用 G1 SDK 的停止命令
             return;
        }

        RCLCPP_INFO(this->get_logger(), "Processing - Distance: %.3fm, Heading Deviation (Pitch): %.3frad", distance, heading_deviation);

        constexpr double target_distance = 0.5;       // 目标距离 (米)
        constexpr double distance_threshold = 0.05;   // 距离精度阈值 (米)
        constexpr double heading_threshold = 0.05;    // 航向偏差阈值 (弧度)

        constexpr double k_linear_p = 0.5;            // 线速度 P 增益
        constexpr double k_angular_p = 1.0;           // 角速度 P 增益
        
        constexpr double max_linear_speed = 0.3;      // 最大线速度 (m/s)
        constexpr double max_angular_speed = 0.8;     // 最大角速度 (rad/s)

        // --- 1. 计算线速度 (前进/后退) ---
        double dist_error = distance - target_distance;
        double linear_vel_x = 0.0;
        if (std::abs(dist_error) > distance_threshold) {
            linear_vel_x = k_linear_p * dist_error;
            // 限制线速度范围
            linear_vel_x = std::clamp(linear_vel_x, -max_linear_speed, max_linear_speed);
        }

        // --- 2. 计算角速度 (偏转/转向) ---
        double angular_vel_yaw = 0.0;
        if (std::abs(heading_deviation) > heading_threshold) {
            // 注意：可能需要根据实际效果调整符号，以确保负偏差产生负角速度（左转）
            angular_vel_yaw = k_angular_p * (-heading_deviation); 
            // 限制角速度范围
            angular_vel_yaw = std::clamp(angular_vel_yaw, -max_angular_speed, max_angular_speed);
        }

        // --- 3. 发送控制指令 ---

        if (std::abs(linear_vel_x) > 0.01 || std::abs(angular_vel_yaw) > 0.01) {
             client_.SetVelocity(linear_vel_x, 0.0, angular_vel_yaw); // 发送速度指令
             RCLCPP_INFO(this->get_logger(), "Sending Velocity - Vx: %.3f m/s, Wz: %.3f rad/s", linear_vel_x, angular_vel_yaw);
        } else {
             // 在目标位置和朝向附近，停止
             client_.StopMove();
             RCLCPP_INFO(this->get_logger(), "Target reached/stationary. Stopping robot.");
        }
    }


    void callback(const tf2_msgs::msg::TFMessage::SharedPtr msg){
        for (const auto &transform : msg->transforms) {
            // 查找目标 AprilTag 的变换
            if (transform.child_frame_id == "tag36h11:1") {
                
                double new_distance = transform.transform.translation.z;
                
                // --- 四元数转欧拉角 ---
                tf2::Quaternion quat(
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                );
                quat.normalize(); // 确保四元数归一化
                
                tf2::Matrix3x3 matrix(quat);
                double roll, pitch, yaw;
                matrix.getRPY(roll, pitch, yaw); // Z-Y-X 顺序

                //使用 Pitch 角作为航向偏差 ---
                double new_heading_deviation = pitch; 

                // 更新共享数据，加锁保护
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    last_distance_ = new_distance;
                    last_heading_deviation_ = new_heading_deviation;
                }

                RCLCPP_INFO(this->get_logger(), 
                    "AprilTag 'tag36h11:1' updated - Distance: %.3fm, Heading Deviation (Pitch): %.3frad", 
                    new_distance, new_heading_deviation);
                
                return; // 找到目标后退出循环
            }
        }
    }

};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<LocoClientNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
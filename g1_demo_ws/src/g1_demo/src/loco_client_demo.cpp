#include <chrono>
#include <g1/g1_loco_client.hpp>
#include <iostream>
#include <map>
#include <rclcpp/utilities.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "common/ut_errror.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class UnitreeG1ControlNode : public rclcpp::Node {
 public:
  explicit UnitreeG1ControlNode(): Node("unitree_g1_control_node"), client_(this) {
    // Create subscription for commands
    command_sub_ = this->create_subscription<std_msgs::msg::String>(
        "g1_loco_command", 10,
        std::bind(&UnitreeG1ControlNode::CommandCallback, this,std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Unitree G1 Control Node started. Listening on topic 'g1_loco_command'");
  }

  bool handleActionError(int32_t error_code) {
    if (error_code == 0) {
      return true;
    }
    RCLCPP_ERROR(this->get_logger(), "Execute action failed, error code: %d",
                 error_code);
    UT_PRINT_ERR(error_code,
                 unitree::robot::g1::UT_ROBOT_LOCO_ERR_LOCOSTATE_NOT_AVAILABLE);
    UT_PRINT_ERR(error_code,
                 unitree::robot::g1::UT_ROBOT_LOCO_ERR_INVALID_FSM_ID);
    UT_PRINT_ERR(error_code,
                 unitree::robot::g1::UT_ROBOT_LOCO_ERR_INVALID_TASK_ID);
    UT_PRINT_ERR(error_code, UT_ROBOT_TASK_TIMEOUT);
    return false;
  }

 private:
  void CommandCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::string command_str = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received command: %s", command_str.c_str());

    // Parse command: format is "command=value" or "command"
    std::string command_name;
    std::string command_value;

    size_t pos = command_str.find('=');
    if (pos != std::string::npos) {
      command_name = command_str.substr(0, pos);
      command_value = command_str.substr(pos + 1);

      // Remove quotes if present
      if (command_value.length() >= 2 && 
          command_value.front() == '"' && command_value.back() == '"') {
        command_value = command_value.substr(1, command_value.length() - 2);
      }
    } else {
      command_name = command_str;
      command_value = "";
    }

    // Process the single command
    ProcessSingleCommand(command_name, command_value);
  }

  void ProcessSingleCommand(const std::string &command_name,
                           const std::string &command_value) {
    RCLCPP_INFO(this->get_logger(), "Processing command: [%s] with param: [%s]",
                command_name.c_str(), command_value.c_str());

    if (command_name == "get_fsm_id") {
      int fsm_id = 0;
      auto ret = client_.GetFsmId(fsm_id);
      if (!handleActionError(ret)) {
        RCLCPP_ERROR(this->get_logger(), "GetFsmId failed");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "current fsm_id: %d", fsm_id);
    }

    else if (command_name == "get_fsm_mode") {
      int fsm_mode = 0;
      auto ret = client_.GetFsmMode(fsm_mode);
      if (!handleActionError(ret)) {
        RCLCPP_ERROR(this->get_logger(), "GetFsmMode failed");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "current fsm_mode: %d", fsm_mode);
    }

    else if (command_name == "get_balance_mode") {
      int balance_mode = 0;
      auto ret = client_.GetBalanceMode(balance_mode);
      if (!handleActionError(ret)) {
        RCLCPP_ERROR(this->get_logger(), "GetBalanceMode failed");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "current balance_mode: %d", balance_mode);
    }

    else if (command_name == "get_swing_height") {
      float swing_height = NAN;
      auto ret = client_.GetSwingHeight(swing_height);
      if (!handleActionError(ret)) {
        RCLCPP_ERROR(this->get_logger(), "GetSwingHeight failed");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "current swing_height: %f", swing_height);
    }

    else if (command_name == "get_stand_height") {
      float stand_height = NAN;
      auto ret = client_.GetStandHeight(stand_height);
      if (!handleActionError(ret)) {
        RCLCPP_ERROR(this->get_logger(), "GetStandHeight failed");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "current stand_height: %f", stand_height);
    }

    else if (command_name == "get_phase") {
      std::vector<float> phase;
      auto ret = client_.GetPhase(phase);
      if (!handleActionError(ret)) {
        RCLCPP_ERROR(this->get_logger(), "GetPhase failed");
        return;
      }
      std::stringstream ss;
      ss << "current phase: (";
      for (const auto &p : phase) {
        ss << p << ", ";
      }
      ss << ")";
      RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    else if (command_name == "set_fsm_id") {
      int fsm_id = std::stoi(command_value);
      auto ret = client_.SetFsmId(fsm_id);
      if (!handleActionError(ret)) {
        RCLCPP_ERROR(this->get_logger(), "SetFsmId failed");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "set fsm_id to %d", fsm_id);
    }

    else if (command_name == "set_balance_mode") {
      int balance_mode = std::stoi(command_value);
      auto ret = client_.SetBalanceMode(balance_mode);
      if (!handleActionError(ret)) {
        RCLCPP_ERROR(this->get_logger(), "SetBalanceMode failed");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "set balance_mode to %d", balance_mode);
    }

    else if (command_name == "set_swing_height") {
      float swing_height = std::stof(command_value);
      auto ret = client_.SetSwingHeight(swing_height);
      if (!handleActionError(ret)) {
        RCLCPP_ERROR(this->get_logger(), "SetSwingHeight failed");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "set swing_height to %f", swing_height);
    }

    else if (command_name == "set_stand_height") {
      float stand_height = std::stof(command_value);
      auto ret = client_.SetStandHeight(stand_height);
      if (!handleActionError(ret)) {
        RCLCPP_ERROR(this->get_logger(), "SetStandHeight failed");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "set stand_height to %f", stand_height);
    }

    else if (command_name == "set_velocity") {
      std::vector<float> param = stringToFloatVector(command_value);
      auto param_size = param.size();
      float vx = NAN;
      float vy = NAN;
      float omega = NAN;
      float duration = NAN;
      if (param_size == 3) {
        vx = param.at(0);
        vy = param.at(1);
        omega = param.at(2);
        duration = 1.F;
      } else if (param_size == 4) {
        vx = param.at(0);
        vy = param.at(1);
        omega = param.at(2);
        duration = param.at(3);
      } else {
        RCLCPP_ERROR(this->get_logger(),
                     "Invalid param size for method SetVelocity: %zu",
                     param_size);
        return;
      }

      auto ret = client_.SetVelocity(vx, vy, omega, duration);
      if (!handleActionError(ret)) {
        RCLCPP_ERROR(this->get_logger(), "SetVelocity failed");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "set velocity to %s",
                  command_value.c_str());
    }

    else if (command_name == "damp") {
      auto ret = client_.Damp();
      if (!handleActionError(ret)) {
        RCLCPP_ERROR(this->get_logger(), "Damp failed");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Damp command sent");
    }

    else if (command_name == "start") {
      auto ret = client_.Start();
      if (!handleActionError(ret)) {
        RCLCPP_ERROR(this->get_logger(), "Start failed");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Start command sent");
    }

    else if (command_name == "squat") {
      auto ret = client_.Squat();
      if (!handleActionError(ret)) {
        RCLCPP_ERROR(this->get_logger(), "Squat failed");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Squat command sent");
    }

    else if (command_name == "sit") {
      auto ret = client_.Sit();
      if (!handleActionError(ret)) {
        RCLCPP_ERROR(this->get_logger(), "Sit failed");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Sit command sent");
    }

    else if (command_name == "stand_up") {
      auto ret = client_.StandUp();
      if (!handleActionError(ret)) {
        RCLCPP_ERROR(this->get_logger(), "StandUp failed");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "StandUp command sent");
    }

    else if (command_name == "zero_torque") {
      auto ret = client_.ZeroTorque();
      if (!handleActionError(ret)) {
        RCLCPP_ERROR(this->get_logger(), "ZeroTorque failed");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "ZeroTorque command sent");
    }

    else if (command_name == "stop_move") {
      auto ret = client_.StopMove();
      if (!handleActionError(ret)) {
        RCLCPP_ERROR(this->get_logger(), "StopMove failed");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "StopMove command sent");
    }

    else if (command_name == "balance_stand") {
      auto ret = client_.BalanceStand();
      if (!handleActionError(ret)) {
        RCLCPP_ERROR(this->get_logger(), "BalanceStand failed");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "BalanceStand command sent");
    }

    else if (command_name == "move") {
      std::vector<float> param = stringToFloatVector(command_value);
      auto param_size = param.size();
      float vx = NAN;
      float vy = NAN;
      float omega = NAN;
      if (param_size == 3) {
        vx = param.at(0);
        vy = param.at(1);
        omega = param.at(2);
      } else {
        RCLCPP_ERROR(this->get_logger(),
                     "Invalid param size for method Move: %zu", param_size);
        return;
      }
      auto ret = client_.Move(vx, vy, omega);
      if (!handleActionError(ret)) {
        RCLCPP_ERROR(this->get_logger(), "Move failed");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Move command sent with params: %s",
                  command_value.c_str());
    }

    else if (command_name == "set_speed_mode") {
      auto ret = client_.SetSpeedMode(std::stoi(command_value));
      if (!handleActionError(ret)) {
        RCLCPP_ERROR(this->get_logger(), "SetSpeedMode failed");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "set speed mode to %s",
                  command_value.c_str());
    }

    else {
      RCLCPP_WARN(this->get_logger(), "Unknown command: %s",
                  command_name.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Done processing command: %s",
                command_name.c_str());
  }

  static std::vector<float> stringToFloatVector(const std::string &str) {
    std::vector<float> result;
    std::stringstream ss(str);
    float num = NAN;
    while (ss >> num) {
      result.push_back(num);
      // ignore any trailing whitespace
      ss.ignore();
    }
    return result;
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  unitree::robot::g1::LocoClient client_;
};

int main(int argc, char const *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<UnitreeG1ControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}


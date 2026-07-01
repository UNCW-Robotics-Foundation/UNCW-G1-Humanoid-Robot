#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg//wireless_controller.hpp"

#include "base_client.hpp"
#include "common/time_tools.hpp"
#include "common/ut_errror.hpp"
#include "nlohmann/json.hpp"
#include "patch.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "unitree_go/msg/motor_cmds.hpp"
#include "unitree_go/msg/motor_cmd.hpp"

class HandsTest : public rclcpp::Node {
 public:
  HandsTest() : Node("hands_test_node") {
    pub_ = this->create_publisher<unitree_go::msg::MotorCmds>("/brainco/right/cmd", 10);
    thread_ = std::thread([this]() { ControlLoop(); });

  }


 private:
 rclcpp::Publisher<unitree_go::msg::MotorCmds>::SharedPtr pub_;
 std::thread thread_;
 std::array<float, 6> positions;

  void ControlLoop() {
    while ( true)
    {
        RCLCPP_INFO(this->get_logger(), "Opening hand...");
        std::vector<unitree_go::msg::MotorCmd> tmpCommands;
        unitree_go::msg::MotorCmds handCmds;
        positions = {0, 0, 0, 0, 0, 0};
        for (int i = 0; i < positions.size(); i++) {
            unitree_go::msg::MotorCmd handCmd;
            handCmd.q = positions[i];
            handCmd.dq = 1.0F;
            handCmd.tau = 0.0F;
            handCmd.kp = 0.0F;
            handCmd.kd = 0.0F;
            handCmd.mode = 0;
            tmpCommands.push_back(handCmd);
        }
        handCmds.cmds = tmpCommands;
        pub_->publish(handCmds);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        RCLCPP_INFO(this->get_logger(), "Closing fingers...");
        positions = {0, 1, 0, 1, 0, 1};
        for (int i = 0; i < positions.size(); i++) {
            unitree_go::msg::MotorCmd handCmd;
            handCmd.q = positions[i];
            handCmd.dq = 1.0F;
            handCmd.tau = 0.0F;
            handCmd.kp = 0.0F;
            handCmd.kd = 0.0F;
            handCmd.mode = 0;
            tmpCommands[i] = handCmd;
        }
        handCmds.cmds = tmpCommands;
        pub_->publish(handCmds);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        RCLCPP_INFO(this->get_logger(), "Closing thumb...");
        positions = {1, 1, 0, 1, 0.5, 1};
        for (int i = 0; i < positions.size(); i++) {
            unitree_go::msg::MotorCmd handCmd;
            handCmd.q = positions[i];
            handCmd.dq = 1.0F;
            handCmd.tau = 0.0F;
            handCmd.kp = 0.0F;
            handCmd.kd = 0.0F;
            handCmd.mode = 0;
            tmpCommands[i] = handCmd;
        }
        handCmds.cmds = tmpCommands;
        pub_->publish(handCmds);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }

};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);  // Initialize rclcpp
  // Run ROS2 node which is make share with wireless_controller_suber class
  rclcpp::spin(std::make_shared<HandsTest>());
  rclcpp::shutdown();
  return 0;
}
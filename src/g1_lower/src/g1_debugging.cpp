#include "common/motor_crc_hg.h"
#include "gamepad.hpp"
#include "motor_crc_hg.h"
#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "g1_msgs/msg/g1_debug.hpp"
#include "g1_msgs/msg/g1_joints_debug.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class G1GuiJointsCmd : public rclcpp::Node {
 public:
  G1GuiJointsCmd() : Node("g1_gui_joints_cmd") {
    //  bind to g1_ankle_swing_sender::LowStateHandler for subscribe "lowstate"
    //  topic
    lowcmd_subscriber_ = this->create_subscription<unitree_hg::msg::LowCmd>(
        "lowcmd", 10,
        [this](unitree_hg::msg::LowCmd::SharedPtr message) {
          CmdCallback(message);
        });
    lowstate_subscriber_ = this->create_subscription<unitree_hg::msg::LowState>(
        "lowstate", 10,
        [this](unitree_hg::msg::LowState::SharedPtr message) {
          StateCallback(message);
        });

    debug_pub_ =
        this->create_publisher<g1_msgs::msg::G1JointsDebug>("g1_dbg", 10);

    sleep_time_ = std::chrono::milliseconds(static_cast<int>(control_dt_ * 1000));
    // init_thread = std::thread([this]() { InitPosition(); });

    timer_ = this->create_wall_timer(std::chrono::milliseconds(2),
                                      [this] { ControlLoop(); });
  }

 private:

  rclcpp::Subscription<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_subscriber_; 
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_subscriber_;
  rclcpp::Publisher<g1_msgs::msg::G1JointsDebug>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  // std::thread init_thread;

  double control_dt_{0.02F};
  double duration_{3.0F};      // [3 s]
  std::chrono::milliseconds sleep_time_{};

  bool cmd_flag = false;
  bool state_flag = false;
  bool init_flag = false;
  unitree_hg::msg::LowCmd cmd;
  unitree_hg::msg::LowState current_state;

  void CmdCallback(const unitree_hg::msg::LowCmd::SharedPtr msg) {
    cmd = *msg;
    cmd_flag = true;
  }

  void StateCallback(const unitree_hg::msg::LowState::SharedPtr msg) {
    current_state = *msg;
    state_flag = true;
  }
  
  void ControlLoop() {
    if (cmd_flag && state_flag) {
      g1_msgs::msg::G1JointsDebug joints_dbg;
      for (int i = 0; i < 29; i++) {
        g1_msgs::msg::G1Debug single_joint_dbg;
        single_joint_dbg.target_q = cmd.motor_cmd[i].q;
        single_joint_dbg.actual_q = current_state.motor_state[i].q;
        single_joint_dbg.delta_q = DeltaHelper(single_joint_dbg.target_q, single_joint_dbg.actual_q);
        joints_dbg.joints_debug[i] = single_joint_dbg;
      }
      debug_pub_->publish(joints_dbg);
    }
  }

  float DeltaHelper(float x, float y) {
    if (x > y) {
      return x - y;
    } else {
      return y - x;
    }
  }

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);  // Initialize rclcpp
  auto node =
      std::make_shared<G1GuiJointsCmd>();  // Create a ROS2 node and make
                                               // share with
                                               // g1_ankle_swing_sender class
  rclcpp::spin(node);                          // Run ROS2 node
  rclcpp::shutdown();                          // Exit
  return 0;
}
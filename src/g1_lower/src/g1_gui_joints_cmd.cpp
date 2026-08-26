#include "common/motor_crc_hg.h"
#include "gamepad.hpp"
#include "motor_crc_hg.h"
#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include <chrono>
#include <thread>
using namespace std::chrono_literals;

class G1GuiJointsCmd : public rclcpp::Node {
 public:
  G1GuiJointsCmd() : Node("g1_gui_joints_cmd") {
    //  bind to g1_ankle_swing_sender::LowStateHandler for subscribe "lowstate"
    //  topic
    gui_subscriber_ = this->create_subscription<unitree_hg::msg::LowCmd>(
        "gui_lowcmd", 10,
        [this](unitree_hg::msg::LowCmd::SharedPtr message) {
          GuiCallback(message);
        });
    lowstate_subscriber_ = this->create_subscription<unitree_hg::msg::LowState>(
        "lowstate", 10,
        [this](unitree_hg::msg::LowState::SharedPtr message) {
          StateCallback(message);
        });

    lowcmd_publisher_ =
        this->create_publisher<unitree_hg::msg::LowCmd>("lowcmd", 10);

    sleep_time_ = std::chrono::milliseconds(static_cast<int>(control_dt_ * 1000));
    init_thread = std::thread([this]() { InitPosition(); });

    timer_ = this->create_wall_timer(std::chrono::milliseconds(2),
                                      [this] { ControlLoop(); });
  }

 private:

  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_publisher_;
  rclcpp::Subscription<unitree_hg::msg::LowCmd>::SharedPtr gui_subscriber_; 
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread init_thread;

  double control_dt_{0.02F};
  double duration_{3.0F};      // [3 s]
  std::chrono::milliseconds sleep_time_{};

  bool gui_flag = false;
  bool state_flag = false;
  bool init_flag = false;
  unitree_hg::msg::LowCmd gui_cmd;
  unitree_hg::msg::LowState current_state;

  void GuiCallback(const unitree_hg::msg::LowCmd::SharedPtr msg) {
    gui_cmd = *msg;
    gui_flag = true;
  }

  void StateCallback(const unitree_hg::msg::LowState::SharedPtr msg) {
    current_state = *msg;
    state_flag = true;
  }
  
  void ControlLoop() {
    if (gui_flag && init_flag) {
      get_crc(gui_cmd);
      lowcmd_publisher_->publish(gui_cmd);
    }
  }

  void InitPosition() {
    std::array<float, 29> current_pos;
    std::array<float, 29> target_pos;
    while (!state_flag) {
      std::this_thread::sleep_for(100ms);
    }

    for (int i = 0; i < 29; i++) {
      current_pos[i] = current_state.motor_state[i].q;
      target_pos[i] = 0.0;
    }
    MoveToInitial(target_pos, current_pos, duration_);
    init_flag = true;

  }

  void MoveToInitial(const std::array<float, 29>& target,
              std::array<float, 29>& current, float duration) {
    const int steps = static_cast<int>(duration / control_dt_);
    const std::array<float, 29> initial = current;

    for (int i = 0; i < steps; ++i) {
      for (size_t j = 0; j < 29; ++j) {
          // linear interpolation
        current[j] = ((i * (target[j] - initial[j])) / steps) + initial[j];
      }

      SendPositionCommand(current);
      std::this_thread::sleep_for(sleep_time_);
    }
  }

  /*
  Helper function for MoveTo function. Very similar to control function.
  */
  void SendPositionCommand(const std::array<float, 29>& positions) {
    unitree_hg::msg::LowCmd cmd;
    cmd.mode_machine = 5;

    for (size_t i = 0; i < 29; ++i) {
      //int idx = static_cast<int>(arm_joints_[i]);
      cmd.motor_cmd[i].q = positions[i];
      cmd.motor_cmd[i].dq = 0.0F;
      cmd.motor_cmd[i].tau = 0.0F;
      cmd.motor_cmd[i].mode = 1;
      cmd.motor_cmd[i].kp = 60.0;
      cmd.motor_cmd[i].kd = 1.5;

    }
    get_crc(cmd);

    lowcmd_publisher_->publish(cmd);
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
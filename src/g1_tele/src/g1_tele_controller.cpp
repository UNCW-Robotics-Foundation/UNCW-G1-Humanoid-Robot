#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include <chrono>
#include <thread>
using namespace std::chrono_literals;

class G1RuckigCmd : public rclcpp::Node {
 public:
  G1RuckigCmd() : Node("g1_ruckig_cmd") {
    //  bind to g1_ankle_swing_sender::LowStateHandler for subscribe "lowstate"
    //  topic
    ruckig_subscriber_ = this->create_subscription<unitree_hg::msg::LowCmd>(
        "joint/cmd", 10,
        [this](unitree_hg::msg::LowCmd::SharedPtr message) {
          RuckigCallback(message);
        });
    lowstate_subscriber_ = this->create_subscription<unitree_hg::msg::LowState>(
        "lowstate", 10,
        [this](unitree_hg::msg::LowState::SharedPtr message) {
          StateCallback(message);
        });

    lowcmd_publisher_ =
        this->create_publisher<unitree_hg::msg::LowCmd>("lowcmd", 10);

    sleep_time_ = std::chrono::milliseconds(static_cast<int>(control_dt_ * 1000));
    // init_thread = std::thread([this]() { InitPosition(); });

    timer_ = this->create_wall_timer(std::chrono::milliseconds(2),
                                      [this] { ControlLoop(); });
  }

 private:

  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_publisher_;
  rclcpp::Subscription<unitree_hg::msg::LowCmd>::SharedPtr ruckig_subscriber_; 
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread init_thread;

  double control_dt_{0.02F};
  double duration_{3.0F};      // [3 s]
  std::chrono::milliseconds sleep_time_{};

  bool ruckig_flag = false;
  bool state_flag = false;
  bool init_flag = false;
  unitree_hg::msg::LowCmd ruckig_cmd;
  unitree_hg::msg::LowCmd final_cmd;
  unitree_hg::msg::LowState current_state;

  void RuckigCallback(const unitree_hg::msg::LowCmd::SharedPtr msg) {
    if (init_flag){
      final_cmd = *msg;
    }
    // ruckig_cmd = *msg;
    // ruckig_flag = true;
  }

  void StateCallback(const unitree_hg::msg::LowState::SharedPtr msg) {
    current_state = *msg;
    state_flag = true;
  }
  
  void ControlLoop() {
    if (init_flag) {
      return;
      // get_crc(final_cmd);
      // lowcmd_publisher_->publish(final_cmd);
    }
  }

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);  // Initialize rclcpp
  auto node =
      std::make_shared<G1RuckigCmd>();  // Create a ROS2 node and make
                                               // share with
                                               // g1_ankle_swing_sender class
  rclcpp::spin(node);                          // Run ROS2 node
  rclcpp::shutdown();                          // Exit
  return 0;
}
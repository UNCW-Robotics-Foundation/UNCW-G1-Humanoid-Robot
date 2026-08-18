#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg//wireless_controller.hpp"

#include "thread"
#include "chrono"
#include "base_client.hpp"
#include "common/time_tools.hpp"
#include "common/ut_errror.hpp"
#include "nlohmann/json.hpp"
#include "patch.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "unitree_go/msg/motor_cmds.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include "unitree_go/msg/motor_states.hpp"
#include "unitree_go/msg/motor_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

std::vector<std::string> FingerJointNames = {
  "thumb",
  "thumb_aux",
  "index",
  "middle",
  "ring",
  "pinky"
};

class HandsTest : public rclcpp::Node {
 public:
  HandsTest() : Node("hands_test_node") {
    pub_ = this->create_publisher<unitree_go::msg::MotorCmds>("/brainco/right/cmd", 10);
    pub2_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_commands_right", 10);
    sub_ = this->create_subscription<unitree_go::msg::MotorStates>(
        "/brainco/right/state", 10,
        [this](const unitree_go::msg::MotorStates::SharedPtr msg) { StateCallback(msg); });
    touch_sub_ = this->create_subscription<unitree_go::msg::MotorStates>(
        "/brainco/right/touch", 10,
        [this](const unitree_go::msg::MotorStates::SharedPtr msg) { TouchCallback(msg); });
    thread_ = std::thread([this]() { ControlLoop(); });
    sleep_time_ =
        std::chrono::milliseconds(static_cast<int>(control_dt_ * 1000));
    timer1_ = this->create_wall_timer(std::chrono::milliseconds(2),
                                      [this] { ContinuousCheck(); });

  }


  private:
  rclcpp::Publisher<unitree_go::msg::MotorCmds>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub2_;
  rclcpp::Subscription<unitree_go::msg::MotorStates>::SharedPtr sub_;
  rclcpp::Subscription<unitree_go::msg::MotorStates>::SharedPtr touch_sub_;
  rclcpp::TimerBase::SharedPtr timer1_;
  std::thread thread_;
  unitree_go::msg::MotorStates current_state;
  unitree_go::msg::MotorStates touch_state;
  std::array<float, 6> positions;
  std::vector<double> positions2;
  std::vector<double> speeds;
  std::array<float,6> current_pos{}; 
  std::array<float,5> current_touch{}; 

  float control_dt_{0.02F};
  float move_duration_ = 3.0F;
  std::chrono::milliseconds sleep_time_{};
  bool state_received_ = false;
  bool touch_state_received_ = false;
  bool touch_flag = false;
  int test = 0;

  void StateCallback(const unitree_go::msg::MotorStates::SharedPtr msg) {
    current_state = *msg;

    for (size_t i = 0; i < 6; ++i) {
      current_pos[i] =
          current_state.states[i].q;
    }

    state_received_ = true;
  }

  void TouchCallback(const unitree_go::msg::MotorStates::SharedPtr msg) {
    touch_state = *msg;

    for (size_t i = 0; i < 5; ++i) {
      current_touch[i] =
          touch_state.states[i].q;
    }

    touch_state_received_ = true;
  }

  void ContinuousCheck() {
    if (current_touch[0] > 20.0) {
      //RCLCPP_INFO(this->get_logger(), "thumb touch data exceeds 40");
      touch_flag = true;
    } else {
      touch_flag = false;
    }
    //RCLCPP_INFO(this->get_logger(), "thumb touch data: %f", current_touch[0]);
  }

  void ControlLoop() {
    std::vector<unitree_go::msg::MotorCmd> tmpCommands;
    unitree_go::msg::MotorCmds handCmds;
    sensor_msgs::msg::JointState handCmds2;
    handCmds2.name = FingerJointNames;
    std::array<float, 6> target {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    std::array<float, 6> current = current_pos;
    const std::array<float, 6> initial = current;
    float duration = 4.0;
    const int steps = static_cast<int>(duration / control_dt_);

    switch (test) {
      // Basic commands test. Good for testing specific fingers and speeds.
      case 0:
        while ( true )
        {
          tmpCommands.clear();
          RCLCPP_INFO(this->get_logger(), "Opening hand...");
          positions = {0, 0, 0, 0, 0, 0};
          positions2 = {0, 0, 0, 0, 0, 0};
          speeds = {10, 10, 10, 10, 10, 10};
          for (int i = 0; i < 6; i++) {
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

          handCmds2.header.stamp = this->get_clock()->now();
          handCmds2.position = positions2;
          handCmds2.velocity = speeds;
          pub2_->publish(handCmds2);
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));

          RCLCPP_INFO(this->get_logger(), "Closing fingers...");
          positions = {0, 1, 1, 1, 1, 1};
          positions2 = {0, 10, 10, 10, 10, 10};
          for (int i = 0; i < 6; i++) {
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

          handCmds2.header.stamp = this->get_clock()->now();
          handCmds2.position = positions2;
          handCmds2.velocity = speeds;
          pub2_->publish(handCmds2);
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));

          RCLCPP_INFO(this->get_logger(), "Closing thumb...");
          positions = {1, 1, 1, 1, 1, 1};
          positions2 = {10, 10, 10, 10, 10, 10};
          for (int i = 0; i < 6; i++) {
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

          handCmds2.header.stamp = this->get_clock()->now();
          handCmds2.position = positions2;
          handCmds2.velocity = speeds;
          pub2_->publish(handCmds2);
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        break;

      case 1:
        while (!(state_received_)){
          RCLCPP_INFO(this->get_logger(), "Waiting for initial hand pose...");
          std::this_thread::sleep_for(100ms);
        }

        for (int i = 0; i < steps; ++i) {
          for (size_t j = 0; j < 6; ++j) {
            current[j] = ((i * (target[j] - initial[j])) / steps) + initial[j];
          }

          for (int i = 0; i < 6; i++) {
            //RCLCPP_INFO(this->get_logger(), "setting cmd...");
              unitree_go::msg::MotorCmd handCmd;
              handCmd.q = current[i];
              handCmd.dq = 1.0F;
              handCmd.tau = 0.0F;
              handCmd.kp = 0.0F;
              handCmd.kd = 0.0F;
              handCmd.mode = 0;
              tmpCommands.push_back(handCmd);
            }
          //RCLCPP_INFO(this->get_logger(), "publishing cmds...");
          handCmds.cmds = tmpCommands;
          pub_->publish(handCmds);
          tmpCommands.clear();
          std::this_thread::sleep_for(sleep_time_);
        }
        break;

      case 2:
        while (!(state_received_) || !(touch_state_received_)){
          RCLCPP_INFO(this->get_logger(), "Waiting for initial hand pose and touch data...");
          std::this_thread::sleep_for(100ms);
        }
        RCLCPP_INFO(this->get_logger(), "Opening hand...");
        positions = {0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 6; i++) {
            unitree_go::msg::MotorCmd handCmd;
            handCmd.q = positions[i];
            handCmd.dq = 0.5F;
            handCmd.tau = 0.0F;
            handCmd.kp = 0.0F;
            handCmd.kd = 0.0F;
            handCmd.mode = 0;
            tmpCommands.push_back(handCmd);
        }
        handCmds.cmds = tmpCommands;
        pub_->publish(handCmds);
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));

        RCLCPP_INFO(this->get_logger(), "Closing fingers...");
        positions = {0, 1, 1, 1, 1, 1};
        for (int i = 0; i < 6; i++) {
          unitree_go::msg::MotorCmd handCmd;
          handCmd.q = positions[i];
          handCmd.dq = 0.5F;
          handCmd.tau = 0.0F;
          handCmd.kp = 0.0F;
          handCmd.kd = 0.0F;
          handCmd.mode = 0;
          tmpCommands[i] = handCmd;
        }
        handCmds.cmds = tmpCommands;
        pub_->publish(handCmds);

        int count = 0;
        RCLCPP_INFO(this->get_logger(), "Waiting for touch...");
        while (count < steps*2) {
          if (touch_flag) {
            RCLCPP_INFO(this->get_logger(), "Touch detected. Sending stop comand...");
            std::array<float, 6> current = current_pos;
            for (int i = 0; i < 6; i++) {
              unitree_go::msg::MotorCmd handCmd;
              handCmd.q = current[i];
              handCmd.dq = 1.0F;
              handCmd.tau = 0.0F;
              handCmd.kp = 0.0F;
              handCmd.kd = 0.0F;
              handCmd.mode = 0;
              tmpCommands[i] = handCmd;
            }
            handCmds.cmds = tmpCommands;
            pub_->publish(handCmds);
            break;
          }

          count++;
          std::this_thread::sleep_for(sleep_time_);
        }
        RCLCPP_INFO(this->get_logger(), "Stopped waiting...");
        break;
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
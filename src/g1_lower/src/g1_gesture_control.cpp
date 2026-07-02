#include <algorithm>
#include <array>
#include <chrono>
#include <mutex>
#include <iostream>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <unitree_hg/msg/low_cmd.hpp>
#include <unitree_hg/msg/low_state.hpp>
#include <string>
#include "sensor_msgs/msg/joy.hpp"
#include "unitree_go/msg//wireless_controller.hpp"

#include "g1/g1.hpp"

using namespace std::chrono_literals;
using LowCmd = unitree_hg::msg::LowCmd;
using LowState = unitree_hg::msg::LowState;

class CustomGestureController : public rclcpp::Node {
static constexpr int NUM_ARM_JOINTS = 17;
static constexpr auto NOT_USED_JOINT = G1Arm7JointIndex::NOT_USED_JOINT;
std::array<G1Arm7JointIndex, NUM_ARM_JOINTS> arm_joints_ = {
    G1Arm7JointIndex::LEFT_SHOULDER_PITCH,
    G1Arm7JointIndex::LEFT_SHOULDER_ROLL,
    G1Arm7JointIndex::LEFT_SHOULDER_YAW,
    G1Arm7JointIndex::LEFT_ELBOW,
    G1Arm7JointIndex::LEFT_WRIST_ROLL,
    G1Arm7JointIndex::LEFT_WRIST_PITCH,
    G1Arm7JointIndex::LEFT_WRIST_YAW,
    G1Arm7JointIndex::RIGHT_SHOULDER_PITCH,
    G1Arm7JointIndex::RIGHT_SHOULDER_ROLL,
    G1Arm7JointIndex::RIGHT_SHOULDER_YAW,
    G1Arm7JointIndex::RIGHT_ELBOW,
    G1Arm7JointIndex::RIGHT_WRIST_ROLL,
    G1Arm7JointIndex::RIGHT_WRIST_PITCH,
    G1Arm7JointIndex::RIGHT_WRIST_YAW,
    G1Arm7JointIndex::WAIST_YAW,
    G1Arm7JointIndex::WAIST_ROLL,
    G1Arm7JointIndex::WAIST_PITCH};
std::array<float, NUM_ARM_JOINTS> target_pos_ = {
    0.0F, PI_2,  0.0F, PI_2, 0.0F, 0.0F, 0.0F,  // left
    0.0F, -PI_2, 0.0F, PI_2, 0.0F, 0.0F, 0.0F,  // right
    0.0F, 0.F,   0.F};

 public:
  CustomGestureController() : Node("custom_gesture_controller") {
    // ROS2接口初始化
    //pub_ = this->create_publisher<LowCmd>("/lowcmd", 10);
    pub_ = this->create_publisher<LowCmd>("/arm_sdk", 10);
    sub_ = this->create_subscription<LowState>(
        "/lowstate", 10,
        [this](const LowState::SharedPtr msg) { StateCallback(msg); });
    // joy_suber_ = this->create_subscription<sensor_msgs::msg::Joy>(
    //             "joy", 10,
    //             [this](const sensor_msgs::msg::Joy::SharedPtr data) {
    //             JoyHandler(data);
    //             });
    suber_ = this->create_subscription<unitree_go::msg::WirelessController>(
        "/wirelesscontroller", 10,
        [this](const unitree_go::msg::WirelessController::SharedPtr data) {
          wireless_callback(data);
        });

    thread_txt_ = std::thread([this]() { ReadFileLoop(); });
    //txt_read_ = this->create_wall_timer(std::chrono::milliseconds(2), [this] { ReadFileLoop(); });
    sleep_time_ =
        std::chrono::milliseconds(static_cast<int>(control_dt_ * 1000));
    
  }

 private:
  rclcpp::Publisher<LowCmd>::SharedPtr pub_;
  rclcpp::Subscription<LowState>::SharedPtr sub_;
  //rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_suber_;
  rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr suber_;
  std::thread thread_txt_;
  rclcpp::TimerBase::SharedPtr txt_read_;
  std::string gesture_data;

  LowState last_state_;
  LowState init_state_;
  std::mutex state_mutex_;

  int count = 0;
  std::string file_name = "gestures/wave.txt";
  bool state_received_ = false;
  bool initial_move_ = false;
  bool btn_flag = false;
  bool busy_flag = false;
  bool e_stop = false;

  float kp_{60.0F}, kd_{1.5F};
  float control_dt_{0.02F};
  float max_joint_velocity_{0.5F};
  float move_duration_ = 3.0F;
  std::chrono::milliseconds sleep_time_{};

  std::array<float, NUM_ARM_JOINTS> current_jpos_{};
  std::array<float, NUM_ARM_JOINTS> init_pos_{};

  void ReadFileLoop() {
    while (true) {
      RCLCPP_INFO(this->get_logger(), "Waiting for initial robot pose and button input...");
      while (!((state_received_) && (busy_flag))){
        std::this_thread::sleep_for(100ms);
        
      } 
      auto start_pos = init_pos_;
      RCLCPP_INFO(this->get_logger(), "Initial pose recorded...");
      RCLCPP_INFO(this->get_logger(), "Performing gesture...");
      unitree_hg::msg::LowState low_state_data;
      int i = 0;

      std::ifstream GestureTxtFile(file_name);
      while (getline (GestureTxtFile, gesture_data)) {
        low_state_data.motor_state.at(i).q = std::stof(gesture_data);
        if (e_stop) {
          RCLCPP_INFO(this->get_logger(), "Stopping gesture early...");
          move_duration_ = 5.0F;
          break;
        }

        if (!(i + 1 < 29)) {
          Control(low_state_data);
          i = 0;
          std::this_thread::sleep_for(1ms);
        } else{
          i++;
        }
        
      }
      GestureTxtFile.close();
      RCLCPP_INFO(this->get_logger(), "Moving to init");
      MoveTo(start_pos, current_jpos_, move_duration_, true);
      RCLCPP_INFO(this->get_logger(), "Gesture complete.");
      LowCmd cmd;
      cmd.motor_cmd[static_cast<int>(NOT_USED_JOINT)].q = 0.0F;
      pub_->publish(cmd);
      RCLCPP_INFO(this->get_logger(), "Control stopped.");
      busy_flag = false;
      e_stop = false;
      move_duration_ = 3.0F;
    }
  }

  void Control(const LowState joint_data) {
    //std::lock_guard<std::mutex> lock(state_mutex_);
    last_state_ = joint_data;

    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      current_jpos_[i] =
          last_state_.motor_state[static_cast<int>(arm_joints_[i])].q;
    }

    LowCmd cmd;

    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      int idx = static_cast<int>(arm_joints_[i]);
      cmd.motor_cmd[idx].q = current_jpos_[i];
      cmd.motor_cmd[idx].dq = 0.0F;
      cmd.motor_cmd[idx].tau = 0.0F;
      if (i >= arm_joints_.size() - 3) {
        cmd.motor_cmd[idx].kp = kp_ * 4.0F;
        cmd.motor_cmd[idx].kd = kd_ * 4.0F;
      } else {
        cmd.motor_cmd[idx].kp = kp_;
        cmd.motor_cmd[idx].kd = kd_;
      }
    }

    cmd.motor_cmd[static_cast<int>(NOT_USED_JOINT)].q = 1.0F;

    if (initial_move_) {
      pub_->publish(cmd);
    } else {
      RCLCPP_INFO(this->get_logger(), "Moving to init");
      MoveTo(current_jpos_, init_pos_, move_duration_, true);
      RCLCPP_INFO(this->get_logger(), "Completed init");
      initial_move_ = true;
    }
  }

  void StateCallback(const LowState::SharedPtr msg) {
    //std::lock_guard<std::mutex> lock(state_mutex_);
    // if (state_received_) {
    //   return;
    // }
    init_state_ = *msg;

    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      init_pos_[i] =
          init_state_.motor_state[static_cast<int>(arm_joints_[i])].q;
      //RCLCPP_INFO(this->get_logger(), "Init joint %d = %f", i, init_pos_[i]);
    }

    state_received_ = true;
  }

  void MoveTo(const std::array<float, NUM_ARM_JOINTS>& target,
              std::array<float, NUM_ARM_JOINTS>& current, float duration,
              bool smooth) {
    const int steps = static_cast<int>(duration / control_dt_);
    const float max_delta = max_joint_velocity_ * control_dt_;

    for (int i = 0; i < steps; ++i) {
      std::string dbg_;
      float phase = static_cast<float>(i) / static_cast<float>(steps);

      for (size_t j = 0; j < arm_joints_.size(); ++j) {
        if (smooth) {
          // smooth mode: linear interpolation
          current[j] = current[j] * (1 - phase) + target[j] * phase;
        } else {
          // non-smooth mode: move with max velocity
          float diff = target[j] - current[j];
          current[j] += std::clamp(diff, -max_delta, max_delta);
        }
        //dbg_ += std::to_string(current[j]) + "; ";
        //RCLCPP_INFO(this->get_logger(), "current target: %f", current[j]);
      }
      //RCLCPP_INFO(this->get_logger(), "Current targets: %f; %f; %f", current[0], current[1], current[2]);
      SendPositionCommand(current);
      std::this_thread::sleep_for(sleep_time_);
    }
  }

  void SendPositionCommand(const std::array<float, NUM_ARM_JOINTS>& positions) {
    LowCmd cmd;

    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      int idx = static_cast<int>(arm_joints_[i]);
      cmd.motor_cmd[idx].q = positions[i];
      cmd.motor_cmd[idx].dq = 0.0F;
      cmd.motor_cmd[idx].tau = 0.0F;
      if (i >= arm_joints_.size() - 3) {
        cmd.motor_cmd[idx].kp = kp_ * 4.0F;
        cmd.motor_cmd[idx].kd = kd_ * 4.0F;
      } else {
        cmd.motor_cmd[idx].kp = kp_;
        cmd.motor_cmd[idx].kd = kd_;
      }
    }

    cmd.motor_cmd[static_cast<int>(NOT_USED_JOINT)].q = 1.0F;

    pub_->publish(cmd);
  }

  void JoyHandler(sensor_msgs::msg::Joy::SharedPtr message) {

    if (btn_flag) {
        int press_counter = 0;
        for (int i = 0; i < message->buttons.size(); i++) {
            if (message->buttons[i] == 1) {
                break;
            }
            press_counter++;
        }
        if (press_counter == message->buttons.size()) {
            RCLCPP_INFO(this->get_logger(), "CONTROLLER HANDLER; Button released...");
            btn_flag = false;
        }
    } 
    else if (message->buttons[0] == 1) {
      if (!(busy_flag)) {
        RCLCPP_INFO(this->get_logger(), "CONTROLLER HANDLER; Button A pressed. Performing wave...");
        file_name = "gestures/wave.txt";
        btn_flag = true;
        busy_flag = true;
      } else {
        RCLCPP_INFO(this->get_logger(), "Button pressed while busy");
        e_stop = true;
      }
    }
    else if (message->buttons[1] == 1) {
        if (!(busy_flag)) {
        RCLCPP_INFO(this->get_logger(), "CONTROLLER HANDLER; Button B pressed. Performing punch...");
        file_name = "gestures/punch.txt";
        btn_flag = true;
        busy_flag = true;
      } else {
        RCLCPP_INFO(this->get_logger(), "Button pressed while busy");
        e_stop = true;
      }
    }
    else if (message->buttons[2] == 1) {
        if (!(busy_flag)) {
        RCLCPP_INFO(this->get_logger(), "CONTROLLER HANDLER; Button X pressed. Performing headrub...");
        file_name = "gestures/head_rub.txt";
        btn_flag = true;
        busy_flag = true;
      } else {
        RCLCPP_INFO(this->get_logger(), "Button pressed while busy");
        e_stop = true;
      }
    }
    else if (message->buttons[3] == 1) {
        RCLCPP_INFO(this->get_logger(), "CONTROLLER HANDLER; Button Y pressed. Nothing configured...");
        btn_flag = true;
    }
    else {
        //RCLCPP_INFO(this->get_logger(), "CONTROLLER HANDLER; Listening...");
    }
  }

  void wireless_callback(const unitree_go::msg::WirelessController::SharedPtr& data) {
    if ((data->keys == 320) && (btn_flag)) {
      if (!(busy_flag)) {
        RCLCPP_INFO(this->get_logger(), "CONTROLLER HANDLER; Button A pressed. Performing wave...");
        file_name = "gestures/wave.txt";
        btn_flag = true;
        busy_flag = true;
        initial_move_ = false;
      } else {
        RCLCPP_INFO(this->get_logger(), "Button pressed while busy");
        e_stop = true;
      }
      btn_flag = false;
    } else if ((data->keys == 576) && (btn_flag)) {
      if (!(busy_flag)) {
        RCLCPP_INFO(this->get_logger(), "CONTROLLER HANDLER; Button B pressed. Performing punch...");
        file_name = "gestures/punch.txt";
        btn_flag = true;
        busy_flag = true;
        initial_move_ = false;
      } else {
        RCLCPP_INFO(this->get_logger(), "Button pressed while busy");
        e_stop = true;
      }
      btn_flag = false;
    }else if ((data->keys == 1088) && (btn_flag)) {
      if (!(busy_flag)) {
        RCLCPP_INFO(this->get_logger(), "CONTROLLER HANDLER; Button X pressed. Performing headrub...");
        file_name = "gestures/head_rub.txt";
        btn_flag = true;
        busy_flag = true;
        initial_move_ = false;
      } else {
        RCLCPP_INFO(this->get_logger(), "Button pressed while busy");
        e_stop = true;
      }
      btn_flag = false;
    }else if ((data->keys == 0) && !(btn_flag)) {
      btn_flag = true;
    }

  }
  

};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CustomGestureController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
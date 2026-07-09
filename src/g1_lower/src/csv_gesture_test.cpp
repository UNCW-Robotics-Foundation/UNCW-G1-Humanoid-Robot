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

/*

This program sends commands to the upper arm control of the G1 robot. The commands are read from
text files that are located in a gesture folder. There are is a script for creating these txt files.
They come from rosbags of the lowstate topic. The robot will enter a "busy" state when performing
a gesture, and if another button for a custom gesture is pressed during that state, the robot
will go back to its initial state.
Should work with either the provided controller or an xbox one depending on which topic is subscribed.
Can work with the simulator if it publishes to low_cmd. DO NOT run with real robot when it is publishing
to low_cmd.

Subscribers - lowstate; wirelesscontroller or joy
Publishers - arm_sdk or low_cmd

*/

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
    pub_ = this->create_publisher<LowCmd>("/lowcmd", 10); // uncomment for Mujoco
    //pub_ = this->create_publisher<LowCmd>("/arm_sdk", 10);  // uncomment for real robot
    sub_ = this->create_subscription<LowState>(
        "/lowstate", 10,
        [this](const LowState::SharedPtr msg) { StateCallback(msg); });
    joy_suber_ = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy", 10,
                [this](const sensor_msgs::msg::Joy::SharedPtr data) {
                JoyHandler(data);
                }); // uncomment for Xbox
    suber_ = this->create_subscription<unitree_go::msg::WirelessController>(
        "/wirelesscontroller", 10,
        [this](const unitree_go::msg::WirelessController::SharedPtr data) {
          WirelessCallback(data);
        });             // uncomment for provided controller

    // A seperate thread is created for managing the control loop
    thread_txt_ = std::thread([this]() { ReadFileLoop(); });
    sleep_time_ =
        std::chrono::milliseconds(static_cast<int>(control_dt_ * 1000));
    
  }

 private:
  rclcpp::Publisher<LowCmd>::SharedPtr pub_;
  rclcpp::Subscription<LowState>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_suber_;          // uncomment for Xbox
  rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr suber_;  // uncomment for provided controller
  std::thread thread_txt_;
  std::string gesture_data;

  LowState last_state_;
  LowState init_state_;

  int count = 0;
  std::string file_name; 
  bool state_received_ = false;
  bool initial_move_ = false;
  bool btn_flag = false;
  bool busy_flag = false;
  bool e_stop = false;
  bool last_move = false;
  bool user_flag = false;
  bool waiting_for_user = false;
  bool looping_flag = false;

  float kp_{60.0F}, kd_{1.5F};
  float control_dt_{0.02F};
  float max_joint_velocity_{0.5F};
  float move_duration_ = 3.0F;
  std::chrono::milliseconds sleep_time_{};

  std::array<float, NUM_ARM_JOINTS> current_jpos_{};  // The robot's current target
  std::array<float, NUM_ARM_JOINTS> init_pos_{};      // The robot's current pos from lowstate
  std::vector<std::array<float, NUM_ARM_JOINTS>> looped_rec;

  /*
  In a constant loop to cleanly handle file operations. Waits for a reading from lowstate (state_received_)
  and for a "busy" state swap (busy_flag) before leaving loop. Once out, the robot's current pose is 
  recorded for moving out of and back into. The file_name variable should have been updated when an
  appropriate button was pressed. The file reading is in a loop that will exit early if one of the 
  preset gesture buttons is pressed again while it is busy.
  */
  void ReadFileLoop() {
    while (true) {
      //RCLCPP_INFO(this->get_logger(), "Waiting for initial robot pose and button input...");
      while (!((state_received_) && (busy_flag))){
        std::this_thread::sleep_for(100ms);
      } 
      auto start_pos = init_pos_;
      //RCLCPP_INFO(this->get_logger(), "Initial pose recorded...");
      //RCLCPP_INFO(this->get_logger(), "Performing gesture...");
      unitree_hg::msg::LowState low_state_data;
      int i = 0;

      std::ifstream GestureTxtFile(file_name);
      getline (GestureTxtFile, gesture_data);
      while (getline (GestureTxtFile, gesture_data)) {
        std::vector<std::string> row;
        std::stringstream ss(gesture_data);
        std::string cell;

        while (std::getline(ss, cell, ',')) {
            row.push_back(cell);
        }
        for (int i = 0; i < 29; i++) {
          low_state_data.motor_state.at(i).q = std::stof(row[i]);
        }

        //low_state_data.motor_state.at(i).q = std::stof(gesture_data);
        if (e_stop) {
          last_move = true;
          //RCLCPP_INFO(this->get_logger(), "Stopping gesture early...");
          move_duration_ = 4.0F;  // when the robot's arms are significantly further than the starting position, a higher duration smoothens its return
          break;
        }

        Control(low_state_data, std::stoi(row[29]));

        // if (!(i + 1 < 29)) {  // each line is a joint, so every 29 is a set
        //   Control(low_state_data);
        //   i = 0;
        //   std::this_thread::sleep_for(1ms); // if this sleep was not here, this thread would make the robot move dangerously fast
        // } else{
        //   i++;
        // }
        
      }

      // after file is read, the robot gets moved back to initial position
      GestureTxtFile.close();
      //RCLCPP_INFO(this->get_logger(), "Moving to init");
      MoveTo(start_pos, init_pos_, move_duration_, true);
      //RCLCPP_INFO(this->get_logger(), "Gesture complete.");
      LowCmd cmd;
      cmd.motor_cmd[static_cast<int>(NOT_USED_JOINT)].q = 0.0F;
      pub_->publish(cmd);
      //RCLCPP_INFO(this->get_logger(), "Control stopped.");

      // flags get reset
      busy_flag = false;
      e_stop = false;
      move_duration_ = 3.0F;
    }
  }

  /*
  The main control Function. Sets all the appropriate joint positions and gains for the robot's 
  arms and waist. If the gesture just started, it will call another method that handles moving from 
  one position to another.
  */
  void Control(const LowState joint_data, const int data_flag) {
    last_state_ = joint_data;

    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      current_jpos_[i] =
          last_state_.motor_state[static_cast<int>(arm_joints_[i])].q;
    }

    if (looping_flag) {
      if (data_flag != 3) {
        waiting_for_user = true;
        RCLCPP_INFO(this->get_logger(), "Beginning looped recording...");
        while (!(user_flag) && !(e_stop)) {
          MoveTo(looped_rec[0], init_pos_, 1.0F, true);
          for (int i = 1; i < looped_rec.size(); i++) {
            if (e_stop || user_flag) {
              break;
            }
            LowCmd cmd;

            for (size_t j = 0; j < arm_joints_.size(); ++j) {
              if (e_stop) {
                break;
              }
              int idx = static_cast<int>(arm_joints_[j]);
              cmd.motor_cmd[idx].q = looped_rec[i][j];
              cmd.motor_cmd[idx].dq = 0.0F;
              cmd.motor_cmd[idx].tau = 0.0F;
              if (j >= arm_joints_.size() - 3) {
                cmd.motor_cmd[idx].kp = kp_ * 4.0F;
                cmd.motor_cmd[idx].kd = kd_ * 4.0F;
              } else {
                cmd.motor_cmd[idx].kp = kp_;
                cmd.motor_cmd[idx].kd = kd_;
              }
            }

            cmd.motor_cmd[static_cast<int>(NOT_USED_JOINT)].q = 1.0F;
            pub_->publish(cmd);
            std::this_thread::sleep_for(1ms);
          }
        }
        RCLCPP_INFO(this->get_logger(), "looped recording ended.");
        looped_rec.clear();
        waiting_for_user = false;
        user_flag = false;
      }
    }
    LowCmd cmd;


    switch (data_flag) {
      case 0:   // regular point
        MoveTo(current_jpos_, init_pos_, move_duration_, true);
        break;

      case 1:   // Continuous Recording
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
        pub_->publish(cmd);
        std::this_thread::sleep_for(1ms);
        break;

      case 2:   // User input required
        MoveTo(current_jpos_, init_pos_, move_duration_, true);
        waiting_for_user = true;
        RCLCPP_INFO(this->get_logger(), "Waiting for user input...");
        while (!((user_flag) || (e_stop))){
          std::this_thread::sleep_for(100ms);
        }
        RCLCPP_INFO(this->get_logger(), "User input received.");
        waiting_for_user = false;
        user_flag = false;
        break;

      case 3:   // Looped continuous recording
        looping_flag = true;
        looped_rec.push_back(current_jpos_);
        break;
    }

    // if (initial_move_) {
    //   pub_->publish(cmd);
    // } else {
    //   //RCLCPP_INFO(this->get_logger(), "Moving to init");
    //   MoveTo(current_jpos_, init_pos_, move_duration_, true);
    //   //RCLCPP_INFO(this->get_logger(), "Completed init");
    //   initial_move_ = true;
    // }
    //MoveTo(current_jpos_, init_pos_, move_duration_, true);
  }

  // Function for lowstate subscriber
  void StateCallback(const LowState::SharedPtr msg) {
    init_state_ = *msg;

    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      init_pos_[i] =
          init_state_.motor_state[static_cast<int>(arm_joints_[i])].q;
    }

    state_received_ = true;
  }

  /*
  Function for moving from current to target position. Intended for moving from initial robot
  running mode position to start of gesture position and back.
  */
  void MoveTo(const std::array<float, NUM_ARM_JOINTS>& target,
              std::array<float, NUM_ARM_JOINTS>& current, float duration,
              bool smooth) {
    const int steps = static_cast<int>(duration / control_dt_);
    const float max_delta = max_joint_velocity_ * control_dt_;

    const std::array<float, NUM_ARM_JOINTS> initial = current;

    for (int i = 0; i < steps; ++i) {
      float phase = static_cast<float>(i) / static_cast<float>(steps);

      for (size_t j = 0; j < arm_joints_.size(); ++j) {
        if (smooth) {
          // smooth mode: linear interpolation
          //current[j] = current[j] * (1 - phase) + target[j] * phase;
          current[j] = ((i * (target[j] - initial[j])) / steps) + initial[j];
        } else {
          // non-smooth mode: move with max velocity
          float diff = target[j] - current[j];
          current[j] += std::clamp(diff, -max_delta, max_delta);
        }
      }

      if ((e_stop) && !(last_move)) {
        last_move = true;
        return;
      }

      SendPositionCommand(current);
      std::this_thread::sleep_for(sleep_time_);
    }
    // Q1 flag
    if (last_move) {
      last_move = false;
    }
  }

  /*
  Helper function for MoveTo function. Very similar to control function.
  */
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

  /*
  Function for joy subscriber. Any modifications to button inputs and/or gestures need to be made here.
  */
  void JoyHandler(sensor_msgs::msg::Joy::SharedPtr message) {
    // Method for knowing when a butten has been released so presses don't repeat. Joy msg is different from wireless controller msg
    if (btn_flag) {
        int press_counter = 0;
        for (int i = 0; i < message->buttons.size(); i++) {
            if (message->buttons[i] == 1) {
                break;
            }
            press_counter++;
        }
        if (press_counter == message->buttons.size()) {
            //RCLCPP_INFO(this->get_logger(), "CONTROLLER HANDLER; Button released...");
            btn_flag = false;
        }
    } 
    // 9 - lb; 10 - rb; 11 = up(d-pad)
    // Buttons flip the btn_flag (flaps back when released), the busy_flag (flips back when gesture completes), and e_stop flag (flips back when gesture completes)
    else if (message->buttons[0] == 1) {
      ButtonsHelper("gestures/csv_test3.csv");
      btn_flag = true;
    }
    else if (message->buttons[1] == 1) {
      //ButtonsHelper("gestures/punch.csv");
      btn_flag = true;
    }
    else if (message->buttons[2] == 1) {
      //ButtonsHelper("gestures/head_rub.csv");
      btn_flag = true;
    }
    else if (message->buttons[3] == 1) {
      ButtonsHelper("gestures/ymca.csv");
      btn_flag = true;
    }
    else if (message->buttons[9] == 1) {
      ButtonsHelper("gestures/csv_test2.csv");
      btn_flag = true;
    }
    else if (message->buttons[10] == 1) {
      ButtonsHelper("gestures/csv_test1.csv");
      btn_flag = true;
    }
    else if (message->buttons[11] == 1) {
      btn_flag = true;
      if (waiting_for_user) {
        user_flag = true;
      }
    }
    else {
        //RCLCPP_INFO(this->get_logger(), "CONTROLLER HANDLER; Listening...");
    }
  }

  /*
  Function for wireless_controller subscriber. Any modifications to button inputs and/or 
  gestures need to be made here. I recommend using F1 + <button>. The controller sends an integer
  value when a button is pressed. I think they are unique powers of 2, so each button press SHOULD be 
  unique, but there are still some bugs because pressing X and A at the same time sometimes performs a
  clap even though it is assigned to double tap A. Double check an input is free by testing in
  robot running mode.
  */
  void WirelessCallback(const unitree_go::msg::WirelessController::SharedPtr& data) {
    // Buttons flip the btn_flag (flaps back when released), the busy_flag (flips back when gesture completes), and e_stop flag (flips back when gesture completes)
    if ((data->keys == 320) && (btn_flag)) {
      ButtonsHelper("gestures/wave.txt");
      btn_flag = false;
    } else if ((data->keys == 576) && (btn_flag)) {
      ButtonsHelper("gestures/punch.txt");
      btn_flag = false;
    }else if ((data->keys == 1088) && (btn_flag)) {
      ButtonsHelper("gestures/head_rub.txt");
      btn_flag = false;

    // Method for avoiding held presses. While btn_flag is false, nothing happens.
    }else if ((data->keys == 0) && !(btn_flag)) {
      btn_flag = true;
    }

  }

  // Helper function for controllers
  void ButtonsHelper(std::string gesture_name) {
    if (!(busy_flag)) {
      //RCLCPP_INFO(this->get_logger(), "CONTROLLER HANDLER; Button B pressed. Performing punch...");
      file_name = gesture_name;
      btn_flag = true;
      busy_flag = true;
      initial_move_ = false;
    } else {
      //RCLCPP_INFO(this->get_logger(), "Button pressed while busy");
      e_stop = true;
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
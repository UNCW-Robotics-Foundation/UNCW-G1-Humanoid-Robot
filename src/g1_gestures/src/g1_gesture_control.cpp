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
#include "g1/gestures.hpp"
#include "ruckig/ruckig.hpp"

/*

This program sends commands to the upper arm control of the G1 robot. The commands are read from
csv files that are located in a gesture folder. There is a script for creating these csv files.
The robot will enter a "busy" state when performing a gesture, and if another button for a custom 
gesture is pressed during that state, the robot will go back to its initial state. Should work with 
either the provided controller or an xbox one depending on which topic is subscribed. Can work with 
the simulator if it publishes to low_cmd. DO NOT run with real robot when it is publishing to low_cmd.

Subscribers - lowstate; wirelesscontroller or joy
Publishers - arm_sdk or low_cmd

*/

using namespace std::chrono_literals;
using LowCmd = unitree_hg::msg::LowCmd;
using LowState = unitree_hg::msg::LowState;
using ruckig::Ruckig;
using ruckig::InputParameter;
using ruckig::OutputParameter;
using ruckig::Result;

class CustomGestureController : public rclcpp::Node {
static constexpr int NUM_ARM_JOINTS = 17;
static constexpr auto NOT_USED_JOINT = G1Arm7JointIndex::NOT_USED_JOINT;
std::array<G1Arm7JointIndex, NUM_ARM_JOINTS> arm_joints_ = {
    G1Arm7JointIndex::WAIST_YAW,
    G1Arm7JointIndex::WAIST_ROLL,
    G1Arm7JointIndex::WAIST_PITCH,
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
    G1Arm7JointIndex::RIGHT_WRIST_YAW};

enum Gesture_Type {
  Point,
  CSV,
  Nothing
};

enum Robot_Status {
  Running,
  Custom_
};

enum Csv_Flags {
  Regular,
  Continuous,
  MoveThenWait,
  looped,
  Wait
};

static constexpr int DOF = 17;
static constexpr double CONTROL_DT = 1.0 / 250.0;     // 250 Hz fast loop
static constexpr double RESYNC_POS_TOLERANCE = 0.02;  // rad, per-joint
static constexpr double max_v = 0.5;   // plan:  0.05    pen:  0.5
static constexpr double max_a = 1.0;    //        0.1          1.0
static constexpr double max_j = 2.5;    //        1.0          2.0

 public:
  CustomGestureController() : Node("custom_gesture_controller"), otg_(CONTROL_DT) {

    for (int i = 0; i < DOF; ++i) {
      input_.max_velocity[i]         = max_v;
      input_.max_acceleration[i]     = max_a;
      input_.max_jerk[i]             = max_j;
      input_.current_position[i]     = 0.0;
      input_.current_velocity[i]     = 0.0;
      input_.current_acceleration[i] = 0.0;
      input_.target_position[i]      = 0.0;
      input_.target_velocity[i]      = 0.0;
      input_.target_acceleration[i]  = 0.0;
      measured_position_[i] = 0.0;
      measured_velocity_[i] = 0.0;
    }

    //pub_ = this->create_publisher<LowCmd>("/lowcmd", 10); // uncomment for Mujoco
    pub_ = this->create_publisher<LowCmd>("/arm_sdk", 10);  // uncomment for real robot

    sub_ = this->create_subscription<LowState>(
        "/lowstate", 10,
        [this](const LowState::SharedPtr msg) { StateCallback(msg); });

    suber_ = this->create_subscription<unitree_go::msg::WirelessController>(
        "/wirelesscontroller", 10,
        [this](const unitree_go::msg::WirelessController::SharedPtr data) {
          WirelessCallback(data);
        }); 

    // A seperate thread is created for managing the control loop
    timer_ = create_wall_timer(std::chrono::duration<double>(CONTROL_DT), std::bind(&CustomGestureController::Main_Control, this));
    
  }

 private:
  rclcpp::Publisher<LowCmd>::SharedPtr pub_;
  rclcpp::Subscription<LowState>::SharedPtr sub_;
  rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr suber_;
  rclcpp::TimerBase::SharedPtr timer_;

  LowState current_low_state_;

  int csv_tracker = 0;
  int csv_size = 0;
  double saved_time;
  std::string file_name; 
  bool state_received_ = false;
  bool btn_flag = false;
  bool busy_flag = false;
  bool e_stop = false;
  bool user_flag = false;
  bool waiting_for_user = false;
  bool record_init_arms = true;
  bool record_time = true;
  bool set_ruckig_flag = true;

  float kp_{60.0F}, kd_{1.5F};
  float control_dt_{0.02F};
  float move_duration_ = 3.0F;

  std::array<float, NUM_ARM_JOINTS> current_jpos_{};  // The robot's current target
  std::array<float, NUM_ARM_JOINTS> curent_arm_pos_{};      // The robot's current pos from lowstate
  std::array<float, NUM_ARM_JOINTS> init_arm_pos_{};
  std::vector<std::array<float, NUM_ARM_JOINTS>> looped_rec;  // Storage for looping gestures
  std::vector<std::vector<std::string>> csv_data;

  Gesture_Type gesture_type = Gesture_Type::Nothing;
  G1Gestures static_gestures;
  std::vector<std::array<float, NUM_ARM_JOINTS>> target_points;
  int last_point_gesture = 999;

  Ruckig<DOF> otg_;
  InputParameter<DOF> input_;
  OutputParameter<DOF> output_;
  std::array<double, DOF> measured_position_{};
  std::array<double, DOF> measured_velocity_{};

  std::mutex state_mutex_;
  std::mutex target_mutex_;
  std::array<double, DOF> pending_target_{};

  void Main_Control() {
    // TODO: Add third flag for fsm state. Only want to perform gestures when in running mode.
    while (!((state_received_) && (busy_flag))){
      return;
    }

    std::lock_guard<std::mutex> lock(target_mutex_);
    if (record_init_arms) {
      init_arm_pos_ = curent_arm_pos_;
      record_init_arms = false;
      for (int i = 0; i < 17; i++) {
        RCLCPP_INFO(this->get_logger(), "[INITIAL] Saving initial %i to %f", i, init_arm_pos_[i]);
      }
    }

    if (e_stop) {
      ExitCustom();
      return;
    }

    switch (gesture_type)
    {
    case Point: {
      Result res = otg_.update(input_, output_);
      if (res != Result::Working && res != Result::Finished) {
        RCLCPP_ERROR(get_logger(), "Ruckig update failed (code %d)", static_cast<int>(res));
        return;
      } else if (res == Result::Finished) {
        if (record_time) {
          RCLCPP_INFO(this->get_logger(), "Starting e_stop timer...");
          saved_time = this->get_clock()->now().seconds();
          record_time = false;
        } else {
          if (this->get_clock()->now().seconds() - saved_time > 5) {
            e_stop = true;
          }
        }
      }

      output_.pass_to_input(input_);

      SendPositionCommandRuckig();
      break;
    }
    case CSV:
      if (csv_tracker >= csv_size) {
        e_stop = true;
        return;
      }
      switch (std::stoi(csv_data[csv_tracker][29])) {
        case Regular:
          if (HandleRegular()) {
            return;
          }
          break;
        
        case Continuous:
          HandleContinuous();
          break;
        
        case MoveThenWait:
          break;

        case looped:
          break;

        case Wait:
          break;

        default:
          e_stop = true;
          break;
      }
      csv_tracker ++;

      break;

    case Nothing:
      RCLCPP_INFO(this->get_logger(), "PLACEHOLDER");
      break;
    
    default:
      e_stop = true;
      break;
    }

  }

  bool HandleRegular() {
    if (set_ruckig_flag) {
      for (int i = 0; i < 17; i++) {
        input_.current_position[i] = curent_arm_pos_[i];

        input_.target_position[i] = std::stof(csv_data[csv_tracker][i + 12]);
        input_.target_velocity[i]     = 0.0;
        input_.target_acceleration[i] = 0.0;
      }

      set_ruckig_flag = false;
    }

    Result res = otg_.update(input_, output_);
    if (res != Result::Working && res != Result::Finished) {
      RCLCPP_ERROR(get_logger(), "Ruckig update failed (code %d); Emergency Stopping...", static_cast<int>(res));
      e_stop = true;
      return false;
    } else if (res == Result::Finished) {
      set_ruckig_flag = true;
      return false;
    }

    output_.pass_to_input(input_);
    SendPositionCommandRuckig();
    return true;
  }

  void HandleContinuous() {
    LowCmd cmd;

    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      int idx = static_cast<int>(arm_joints_[i]);
      cmd.motor_cmd[idx].q = std::stof(csv_data[csv_tracker][i + 12]);
      cmd.motor_cmd[idx].dq = 0.0F;
      // cmd.motor_cmd[idx].dq = 0.0F;
      cmd.motor_cmd[idx].tau = 0.0F;
      if (i < 3) {
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

  // Function for lowstate subscriber
  void StateCallback(const LowState::SharedPtr msg) {
    current_low_state_ = *msg;

    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      curent_arm_pos_[i] =
          current_low_state_.motor_state[static_cast<int>(arm_joints_[i])].q;
    }

    state_received_ = true;
  }

  void SendPositionCommandRuckig() {
    LowCmd cmd;

    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      int idx = static_cast<int>(arm_joints_[i]);
      cmd.motor_cmd[idx].q = output_.new_position[i];
      cmd.motor_cmd[idx].dq = 0.0F;
      // cmd.motor_cmd[idx].dq = 0.0F;
      cmd.motor_cmd[idx].tau = 0.0F;
      if (i < 3) {
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

  void ExitCustom() {
    RCLCPP_INFO(this->get_logger(), "Returning to initial position...");
    for (int i = 0; i < 17; i++) {
      input_.current_position[i] = curent_arm_pos_[i];

      input_.target_position[i] = init_arm_pos_[i];
      input_.target_velocity[i]     = 0.0;
      input_.target_acceleration[i] = 0.0;
    }

    Result res = otg_.update(input_, output_);
    while (res == Result::Working) {
      output_.pass_to_input(input_);
      SendPositionCommandRuckig();

      rclcpp::sleep_for(std::chrono::milliseconds(4)); // 250 Hz
      res = otg_.update(input_, output_);
    }
    busy_flag = false;
    e_stop = false;
    record_init_arms = true;
    gesture_type = Gesture_Type::Nothing;
    last_point_gesture = 999;
    csv_data.clear();
    set_ruckig_flag = true;

    LowCmd cmd;
    cmd.motor_cmd[static_cast<int>(NOT_USED_JOINT)].q = 0.0F;
    pub_->publish(cmd);

    RCLCPP_INFO(this->get_logger(), "Custom Gestures Exited");

  }

  /*
  Function for wireless_controller subscriber. Any modifications to button inputs and/or 
  gestures need to be made here. I recommend using <F1 or L1> + <button>. The controller sends an integer
  value when a button is pressed. I think they are unique powers of 2, so each button press SHOULD be 
  unique, but there are still some bugs because pressing X and A at the same time sometimes performs a
  clap even though it is assigned to double tap A. Double check an input is free by testing in
  robot running mode.
  */
  void WirelessCallback(const unitree_go::msg::WirelessController::SharedPtr& data) {
    // Buttons flip the btn_flag (flaps back when released), the busy_flag (flips back when gesture completes), and e_stop flag (flips back when gesture completes)
    if ((data->keys == 258) && (btn_flag)) {        // L1 + A
      CsvButton("gestures/ymca.csv");
      btn_flag = false;
    } else if ((data->keys == 514) && (btn_flag)) { // L1 + B
      CsvButton("gestures/raise.csv");
      btn_flag = false;
    } else if ((data->keys == 1026) && (btn_flag)) { // L1 + X
      CsvButton("gestures/rodeo.csv");
      btn_flag = false;
    } else if ((data->keys == 2050) && (btn_flag)) { // L1 + Y
      CsvButton("gestures/wave.csv");
      btn_flag = false;
    } else if ((data->keys == 4098) && (btn_flag)) { // L1 + UP
      CsvButton("gestures/wings.csv");
      btn_flag = false;
    } else if ((data->keys == 20) && (btn_flag)) { // R2 + START
      PointButton(0, static_gestures.Stand_By);
      btn_flag = false;
    } else if ((data->keys == 32784) && (btn_flag)) { // R2 + LEFT
      PointButton(1, static_gestures.Point_Left);
      btn_flag = false;
    } else if ((data->keys == 8208) && (btn_flag)) { // R2 + RIGHT
      PointButton(2, static_gestures.Point_Right);
      btn_flag = false;
    } else if ((data->keys == 6) && (btn_flag)) { // L1 + START
      if (waiting_for_user) {
        user_flag = true;
      }
      btn_flag = false;
    }
    // Method for avoiding held presses. While btn_flag is false, nothing happens. 2 is th value for LB,
    // so users can hold lb and press a button without needing to let go of everything everytime
      else if ((data->keys <= 2) && !(btn_flag)) {
      btn_flag = true;
    }

  }

  // Helper function for controllers
  void CsvButton(std::string gesture_name) {
    if (!(busy_flag)) {
      std::lock_guard<std::mutex> lock(target_mutex_);
      std::ifstream GestureFile(gesture_name);
      std::string raw_csv;
      getline (GestureFile, raw_csv);
      while (getline (GestureFile, raw_csv)) {
        std::stringstream ss(raw_csv);
        std::string cell;
        std::vector<std::string> row;

        while (std::getline(ss, cell, ',')) {
          row.push_back(cell);
        }
        csv_data.push_back(row);
      }

      csv_size = csv_data.size();
      csv_tracker = 0;
      btn_flag = true;
      busy_flag = true;
      gesture_type = Gesture_Type::CSV;
    } else {
      RCLCPP_INFO(this->get_logger(), "EMERGENCY STOP!!!");
      e_stop = true;
    }
  }

  void PointButton(int point_gesture_id, std::array<float, 17> point_gesture) {
    if ((last_point_gesture == point_gesture_id) || ((gesture_type != Gesture_Type::Nothing) && (gesture_type != Gesture_Type::Point))) {
      e_stop = true;
      RCLCPP_INFO(this->get_logger(), "EMERGENCY STOP!!!");
      return;
    } else if (!e_stop) {  // TODO: make this check fsm id for flag
      std::lock_guard<std::mutex> lock(target_mutex_);
      last_point_gesture = point_gesture_id;
      record_time = true;

      for (int i = 0; i < 17; i++) {
        //pending_target_[i] = point_gesture[i];
        input_.target_position[i] = point_gesture[i];
        input_.target_velocity[i]     = 0.0;
        input_.target_acceleration[i] = 0.0;
        RCLCPP_INFO(this->get_logger(), "Setting %i to target %f", i, point_gesture[i]);
      }

      if (gesture_type != Gesture_Type::Point) {
        for (int i = 0; i < 17; i++) {
          input_.current_position[i] = curent_arm_pos_[i];
        }
        RCLCPP_INFO(this->get_logger(), "Current_position set");
        gesture_type = Gesture_Type::Point;
      }
      if (!busy_flag) {
        busy_flag = true;
      }
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
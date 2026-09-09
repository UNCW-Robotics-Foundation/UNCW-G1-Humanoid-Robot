#include <algorithm>
#include <array>
#include <chrono>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <unitree_hg/msg/low_cmd.hpp>
#include <unitree_hg/msg/low_state.hpp>
#include <g1_msgs/msg/arm_states.hpp>
#include <g1_msgs/msg/motor_state.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "motor_crc_hg.h"

#include "g1/g1.hpp"

using namespace std::chrono_literals;
//using LowCmd = unitree_hg::msg::LowCmd;
using LowState = unitree_hg::msg::LowState;

class ArmLowLevelController : public rclcpp::Node {
static constexpr int NUM_ARM_JOINTS = 14;
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
    G1Arm7JointIndex::RIGHT_WRIST_YAW};

// Stiffness for all G1 Joints
const std::array<float, 29> Kp{
    60, 60, 60, 100, 40, 40,      // legs
    60, 60, 60, 100, 40, 40,      // legs
    60, 40, 40,                   // waist
    40, 40, 40, 40,  40, 40, 40,  // arms
    40, 40, 40, 40,  40, 40, 40   // arms
};

// Damping for all G1 Joints
const std::array<float, 29> Kd{
    1, 1, 1, 2, 1, 1,     // legs
    1, 1, 1, 2, 1, 1,     // legs
    1, 1, 1,              // waist
    1, 1, 1, 1, 1, 1, 1,  // arms
    1, 1, 1, 1, 1, 1, 1   // arms
};

 public:
  ArmLowLevelController() : Node("arm_lowlevel_controller") {
    // ROS2接口初始化
    //cmd_pub_ = this->create_publisher<unitree_hg::msg::LowCmd>("/arm_sdk", 10);
    cmd_pub_ = this->create_publisher<unitree_hg::msg::LowCmd>("/lowcmd", 10);
    arm_joints_pub_ = this->create_publisher<g1_msgs::msg::ArmStates>("/arm_joints", 10);
    lowstate_sub_ = this->create_subscription<LowState>(
        "/lowstate", 10,
        [this](const LowState::SharedPtr msg) { StateCallback(msg); });
    ik_sub_ = this->create_subscription<g1_msgs::msg::ArmStates>(
        "/ik_sol", 10,
        [this](const g1_msgs::msg::ArmStates::SharedPtr msg) { IkCallback(msg); });

    sleep_time_ =
        std::chrono::milliseconds(static_cast<int>(control_dt_ * 1000));

    thread_ = std::thread([this]() { InitRobot(); });

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    joy_suber_ = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy", 10,
                [this](const sensor_msgs::msg::Joy::SharedPtr data) {
                JoyHandler(data);
                });
    timer_ = this->create_wall_timer(std::chrono::milliseconds(30),
                                      [this] { IkLoop(); });
  }

 private:
  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr cmd_pub_;
  rclcpp::Publisher<g1_msgs::msg::ArmStates>::SharedPtr arm_joints_pub_;
  rclcpp::Subscription<LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<g1_msgs::msg::ArmStates>::SharedPtr ik_sub_;
  std::thread thread_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_suber_; 

  LowState last_state_;
  g1_msgs::msg::ArmStates ik_sol;
  std::mutex state_mutex_;
  bool state_received_ = false;

  int count = 0;

  float control_dt_{0.02F};
  float kp_{60.0F}, kd_{1.5F};
  float max_joint_velocity_{0.5F};
  std::chrono::milliseconds sleep_time_{};

  std::array<g1_msgs::msg::MotorState, NUM_ARM_JOINTS> current_arm_pos_;
  g1_msgs::msg::ArmStates current_arms;

  // const float kp_high = 300.0;
  // const float kd_high = 3.0;
  const float kp_low = 80.0;
  const float kd_low = 3.0;
  const float kp_wrist = 40.0;
  const float kd_wrist = 1.5;

  bool btn_flag = false;
  bool init_flag = false;
  bool state_flag = false;
  bool stop_flag = false;
  bool ik_pub_flag = false;
  bool first_ik_flag = false;
  bool e_stop = false;

  float move_duration_ = 3.0F;

  unitree_hg::msg::LowCmd final_cmd;
  unitree_hg::msg::LowCmd zero_cmd;
  std::array<float, 14> ik_init_error;

  void StateCallback(const LowState::SharedPtr msg) {
    last_state_ = *msg;
    state_flag = true;

    g1_msgs::msg::ArmStates current_arms;
    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      g1_msgs::msg::MotorState tmp_joint;
      tmp_joint.q = last_state_.motor_state[static_cast<int>(arm_joints_[i])].q;
      tmp_joint.dq = last_state_.motor_state[static_cast<int>(arm_joints_[i])].dq;
      current_arm_pos_[i] = tmp_joint;
    }
    current_arms.motor_states = current_arm_pos_;

    arm_joints_pub_->publish(current_arms);

  }

  void IkCallback(const g1_msgs::msg::ArmStates::SharedPtr msg) {
    ik_sol = *msg;

    if (!first_ik_flag) {
      for (int i = 0; i < 14; i++) {
        // ik_init_error[i] = ik_sol.motor_states[i].q;
        ik_init_error[i] = 0.0;
        RCLCPP_INFO(this->get_logger(), "Joint %i int error: %f", i, ik_init_error[i]);
        first_ik_flag = true;
      }
    }


    for (int i = 15; i < 29; ++i) {
      final_cmd.motor_cmd[i].q = ik_sol.motor_states[i-15].q - ik_init_error[i-15];
      final_cmd.motor_cmd[i].dq = 0.0F;
      final_cmd.motor_cmd[i].tau = ik_sol.motor_states[i-15].dq;
      //final_cmd.motor_cmd[i].tau = 0.0F;
      //cmd.motor_cmd[i].mode = 1;
      if (((i >= 19) && (i <= 21)) || (i >= 26)) {
        final_cmd.motor_cmd[i].kp = kp_wrist;
        final_cmd.motor_cmd[i].kd = kd_wrist;
      } else {
        final_cmd.motor_cmd[i].kp = kp_low;
        final_cmd.motor_cmd[i].kd = kd_low;
      }

    }

  }

  void JoyHandler(sensor_msgs::msg::Joy::SharedPtr message) {
    // Method for knowing when a butten has been released so presses don't repeat. Joy msg is different from wireless controller msg
    if (btn_flag) {
        int press_counter = 0;
        int msg_btn_size = message->buttons.size();
        for (int i = 0; i < msg_btn_size; i++) {
            if (message->buttons[i] == 1) {
                break;
            }
            press_counter++;
        }
        if (press_counter == msg_btn_size) {
            //RCLCPP_INFO(this->get_logger(), "CONTROLLER HANDLER; Button released...");
            btn_flag = false;
        }
    } 

    else if (message->buttons[0] == 1) {   // A
      e_stop = true;
      btn_flag = true;
      RCLCPP_INFO(this->get_logger(), "!!!ROBOT EMERGENCY STOP!!!");
    }
    else if (message->buttons[3] == 1) {   // Y
      e_stop = true;
      btn_flag = true;
      RCLCPP_INFO(this->get_logger(), "!!!ROBOT EMERGENCY STOP!!!");
    }
    else if (message->buttons[1] == 1) {   // B
      if ((ik_pub_flag) || ((!ik_pub_flag) && (!stop_flag))){
        init_flag = true;
        ik_pub_flag = false;
      }

      btn_flag = true;
    }
    else if (message->buttons[2] == 1) {   // X
      if (ik_pub_flag){
        stop_flag = true;
        ik_pub_flag = false;
      }

      btn_flag = true;
    }

  }

  void IkLoop() {
    if ((!first_ik_flag) && (ik_pub_flag) && (!e_stop)) {
      get_crc(zero_cmd);
      cmd_pub_->publish(zero_cmd);
    }
    else if ((first_ik_flag) && (ik_pub_flag) && (!e_stop)) {
      get_crc(final_cmd);
      cmd_pub_->publish(final_cmd);
    }
  }

  void InitRobot() {
    bool once_flag = false;
    while (!state_flag) {
      std::this_thread::sleep_for(sleep_time_);
    }
    std::this_thread::sleep_for(sleep_time_);

    std::array<float, 29> initial;
    std::array<float, 29> zero_initial;

    std::array<float, 29> current_lowstate;
    final_cmd.mode_machine = 5;
    zero_cmd.mode_machine = 5;
    for (int i = 0; i < 29; i++) {
      unitree_hg::msg::MotorCmd tmp_cmd;
      current_lowstate[i] = last_state_.motor_state[i].q;
      initial[i] = last_state_.motor_state[i].q;
      zero_initial[i] = 0.0;

      tmp_cmd.q = 0.0;
      tmp_cmd.dq = 0.0F;
      tmp_cmd.tau = 0.0F;
      tmp_cmd.mode = 1;
      if ((i >= 15) || (i <= 11)) {
        tmp_cmd.kp = Kp[i];
        tmp_cmd.kd = Kd[i];
      } else {
        tmp_cmd.kp = Kp[i] * 4.0f;
        tmp_cmd.kd = Kd[i] * 4.0f;
      }

      final_cmd.motor_cmd[i] = tmp_cmd;
      zero_cmd.motor_cmd[i] = tmp_cmd;
    }
    MoveToInitial(zero_initial, current_lowstate, 3.0F);
    RCLCPP_INFO(this->get_logger(), "Robot Initialized");
    ik_pub_flag = true;

    while (!e_stop) {
      if (init_flag) {
        for (int i = 0; i < 29; i++) {
          current_lowstate[i] = last_state_.motor_state[i].q;
        }
        MoveToInitial(zero_initial, current_lowstate, 3.0F);
        init_flag = false;
        RCLCPP_INFO(this->get_logger(), "Robot Initialized");
        ik_pub_flag = true;
      }

      if (stop_flag) {
        ik_pub_flag = false;
        for (int i = 0; i < 29; i++) {
          current_lowstate[i] = last_state_.motor_state[i].q;
        }
        MoveToInitial(initial, current_lowstate, 3.0F);
        stop_flag = false;
        RCLCPP_INFO(this->get_logger(), "Robot Stopped");
      }

      if ((first_ik_flag) && (once_flag)) {
        ik_pub_flag = false;
        once_flag = false;

        std::array<float, 29> first_ik_pos;
        for (int i = 0; i < 29; i++) {
          current_lowstate[i] = last_state_.motor_state[i].q;
          first_ik_pos[i] = final_cmd.motor_cmd[i].q;
        }

        MoveToInitial(first_ik_pos, current_lowstate, 1.0F);
        ik_pub_flag = true;
      }

    }
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

      if (e_stop) {
        return;
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
    //cmd.mode_pr = 0;
    cmd.mode_machine = 5;

    for (size_t i = 0; i < 29; ++i) {
      //int idx = static_cast<int>(arm_joints_[i]);
      cmd.motor_cmd[i].q = positions[i];
      cmd.motor_cmd[i].dq = 0.0F;
      cmd.motor_cmd[i].tau = 0.0F;
      cmd.motor_cmd[i].mode = 1;
      if ((i >= 15) || (i <= 11)) {
        cmd.motor_cmd[i].kp = Kp[i];
        cmd.motor_cmd[i].kd = Kd[i];
      } else {
        cmd.motor_cmd[i].kp = Kp[i] * 4.0f;
        cmd.motor_cmd[i].kd = Kd[i] * 4.0f;
      }

    }

    //cmd.motor_cmd[static_cast<int>(NOT_USED_JOINT)].q = 1.0F;
    get_crc(cmd);

    if (e_stop) {
      return;
    }
    cmd_pub_->publish(cmd);
  }

};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmLowLevelController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
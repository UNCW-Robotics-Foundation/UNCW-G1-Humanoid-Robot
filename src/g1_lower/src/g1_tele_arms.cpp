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

#include "g1/g1.hpp"

using namespace std::chrono_literals;
using LowCmd = unitree_hg::msg::LowCmd;
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
std::array<float, NUM_ARM_JOINTS> target_pos_ = {
    0.0F, PI_2,  0.0F, PI_2, 0.0F, 0.0F, 0.0F,  // left
    0.0F, -PI_2, 0.0F, PI_2, 0.0F, 0.0F, 0.0F   // right
};

 public:
  ArmLowLevelController() : Node("arm_lowlevel_controller") {
    // ROS2接口初始化
    cmd_pub_ = this->create_publisher<LowCmd>("/arm_sdk", 10);
    arm_joints_pub_ = this->create_publisher<g1_msgs::msg::ArmStates>("/arm_joints", 10);
    sub_ = this->create_subscription<LowState>(
        "/lowstate", 10,
        [this](const LowState::SharedPtr msg) { StateCallback(msg); });

    sleep_time_ =
        std::chrono::milliseconds(static_cast<int>(control_dt_ * 1000));

    thread_ = std::thread([this]() { ControlLoop(); });
  }

 private:
  rclcpp::Publisher<LowCmd>::SharedPtr cmd_pub_;
  rclcpp::Publisher<g1_msgs::msg::ArmStates>::SharedPtr arm_joints_pub_;
  rclcpp::Subscription<LowState>::SharedPtr sub_;
  std::thread thread_;

  rclcpp::TimerBase::SharedPtr timer1_;

  LowState last_state_;
  std::mutex state_mutex_;
  bool state_received_ = false;

  int count = 0;

  float control_dt_{0.02F};
  float kp_{60.0F}, kd_{1.5F};
  float max_joint_velocity_{0.5F};
  std::chrono::milliseconds sleep_time_{};

  std::array<g1_msgs::msg::MotorState, NUM_ARM_JOINTS> current_arm_pos_{};

  void StateCallback(const LowState::SharedPtr msg) {
    last_state_ = *msg;

    // if (state_received_) {
    //   return;
    // }
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

  void ControlLoop() {
    int x = 1;
  }

};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmLowLevelController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
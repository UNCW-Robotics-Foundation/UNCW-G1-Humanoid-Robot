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
    //cmd_pub_ = this->create_publisher<LowCmd>("/arm_sdk", 10);
    cmd_pub_ = this->create_publisher<LowCmd>("/lowcmd", 10);
    rviz_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    arm_joints_pub_ = this->create_publisher<g1_msgs::msg::ArmStates>("/arm_joints", 10);
    lowstate_sub_ = this->create_subscription<LowState>(
        "/lowstate", 10,
        [this](const LowState::SharedPtr msg) { StateCallback(msg); });
    ik_sub_ = this->create_subscription<g1_msgs::msg::ArmStates>(
        "/ik_sol", 10,
        [this](const g1_msgs::msg::ArmStates::SharedPtr msg) { IkCallback(msg); });
    js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { JSCallback(msg); });

    sleep_time_ =
        std::chrono::milliseconds(static_cast<int>(control_dt_ * 1000));

    thread_ = std::thread([this]() { InitRobot(); });

    timer1_ = this->create_wall_timer(std::chrono::milliseconds(2),
                                      [this] { FrameBroadcast(); });

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    joy_suber_ = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy", 10,
                [this](const sensor_msgs::msg::Joy::SharedPtr data) {
                JoyHandler(data);
                });
  }

 private:
  rclcpp::Publisher<LowCmd>::SharedPtr cmd_pub_;
  rclcpp::Publisher<g1_msgs::msg::ArmStates>::SharedPtr arm_joints_pub_;
  rclcpp::Subscription<LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<g1_msgs::msg::ArmStates>::SharedPtr ik_sub_;
  std::thread thread_;
  rclcpp::TimerBase::SharedPtr timer1_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_suber_; 
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr rviz_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_; 

  LowState last_state_;
  g1_msgs::msg::ArmStates ik_sol;
  std::mutex state_mutex_;
  bool state_received_ = false;

  int count = 0;

  float control_dt_{0.02F};
  float kp_{60.0F}, kd_{1.5F};
  float max_joint_velocity_{0.5F};
  std::chrono::milliseconds sleep_time_{};

  std::array<g1_msgs::msg::MotorState, NUM_ARM_JOINTS> current_arm_pos_{};
  g1_msgs::msg::ArmStates current_arms;
  sensor_msgs::msg::JointState initial_js;
  sensor_msgs::msg::JointState new_js;

  const float kp_high = 300.0;
  const float kd_high = 3.0;
  const float kp_low = 80.0;
  const float kd_low = 3.0;
  const float kp_wrist = 40.0;
  const float kd_wrist = 1.5;

  float t_x = 0.0;
  float t_y = 0.0;
  float t_z = 0.0;
  bool btn_flag = false;
  bool once_flag = true;
  bool js_once_flag = true;

  float move_duration_ = 3.0F;

  void StateCallback(const LowState::SharedPtr msg) {
    last_state_ = *msg;

    // if (state_received_) {
    //   return;
    // }
    // if (once_flag) {
    //   for (size_t i = 0; i < arm_joints_.size(); ++i) {
    //     g1_msgs::msg::MotorState tmp_joint;
    //     tmp_joint.q = last_state_.motor_state[static_cast<int>(arm_joints_[i])].q;
    //     tmp_joint.dq = last_state_.motor_state[static_cast<int>(arm_joints_[i])].dq;
    //     current_arm_pos_[i] = tmp_joint;
    //   }
    //   current_arms.motor_states = current_arm_pos_;
    //   once_flag = false;
    // } else {
    //   arm_joints_pub_->publish(current_arms);
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

  void IkCallback(const g1_msgs::msg::ArmStates::SharedPtr msg) {
    ik_sol = *msg;

    LowCmd cmd;
    //cmd.mode_pr = 0;
    //cmd.mode_machine = 5;
    // new_js.position = initial_js.position;
    // new_js.name = initial_js.name;

    // if (state_received_) {
    //   return;
    // }
    for (int i = 15; i < 29; ++i) {
      cmd.motor_cmd[i].q = ik_sol.motor_states[i-15].q;
      cmd.motor_cmd[i].dq = 0.0F;
      cmd.motor_cmd[i].tau = 0.0F;
      //cmd.motor_cmd[i].mode = 1;
      cmd.motor_cmd[i].kp = Kp[i];
      cmd.motor_cmd[i].kd = Kd[i];
      // if (i - 15 < 7) {
      //   new_js.position[i] = ik_sol.motor_states[i-15].q;
      // } else {
      //   new_js.position[i + 16] = ik_sol.motor_states[i-15].q;
      // }
    }
    cmd.motor_cmd[29].q = 1.0F;
    //new_js.header.stamp = this->get_clock()->now();

    cmd_pub_->publish(cmd);
    //rviz_pub_->publish(new_js);

  }

  void JSCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (js_once_flag) {
      initial_js = *msg;
      new_js = initial_js;
      js_once_flag = false;
      RCLCPP_INFO(this->get_logger(), "Initial joint_state saved");
    } 
    

  }

  void FrameBroadcast() {
    // geometry_msgs::msg::TransformStamped t;

    // t.header.stamp = this->get_clock()->now();
    // t.header.frame_id = "world";
    // t.child_frame_id = "target";
    // t.transform.translation.x = 0.0 + t_x;
    // t.transform.translation.y = 0.0 + t_y;
    // t.transform.translation.z = 0.0 + t_z;
    // t.transform.rotation.x = 0.0;
    // t.transform.rotation.y = 0.0;
    // t.transform.rotation.z = 0.0;
    // t.transform.rotation.w = 1.0;

    // tf_broadcaster_->sendTransform(t);

    if (!js_once_flag) {
      new_js.header.stamp = this->get_clock()->now();
      // rviz_pub_->publish(new_js);
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

    // Buttons flip the btn_flag (flips back when released), 
    // the busy_flag (flips back when gesture completes), 
    // and e_stop flag (flips back when gesture completes)
    else if (message->buttons[11] == 1) {   // d-pad up
      t_x = t_x + 0.01;
      btn_flag = true;
    }
    else if (message->buttons[12] == 1) {   // d-pad down
      t_x = t_x - 0.01;
      btn_flag = true;
    }
    else if (message->buttons[13] == 1) {   // d-pad left
      t_z = t_z + 0.01;
      btn_flag = true;
    }
    else if (message->buttons[14] == 1) {   // d-pad right
      t_z = t_z - 0.01;
      btn_flag = true;
    }
    else if (message->buttons[0] == 1) {   // A
      t_y = t_y + 0.01;
      btn_flag = true;
    }
    else if (message->buttons[3] == 1) {   // Y
      t_y = t_y - 0.01;
      btn_flag = true;
    }
    else if (message->buttons[1] == 1) {   // B
      // LowCmd cmd;
      // for (int i = 0; i < 29; ++i) {
      //   cmd.motor_cmd[i].q = 0.0F;
      //   cmd.motor_cmd[i].dq = 0.0F;
      //   cmd.motor_cmd[i].tau = 0.0F;
      //   cmd.motor_cmd[i].kp = Kp[i];
      //   cmd.motor_cmd[i].kd = Kd[i];
      //   cmd.motor_cmd[i].mode = 1;
      // }
      // cmd.motor_cmd[29].q = 1.0F;

      // cmd_pub_->publish(cmd);
      std::array<float, NUM_ARM_JOINTS> current_lowstate_arms;
      for (int i = 15; i < 29; i++) {
        current_lowstate_arms[i-15] = last_state_.motor_state[i].q;
      }
      MoveToFinal({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, current_lowstate_arms, 3.0F);
      RCLCPP_INFO(this->get_logger(), "Robot Initialized");

      //sensor_msgs::msg::JointState new_js;
      new_js.position = initial_js.position;
      new_js.name = initial_js.name;
      //new_js.header.stamp = this->get_clock()->now();
      //rviz_pub_->publish(new_js);

      btn_flag = true;
    }
    else {
        //RCLCPP_INFO(this->get_logger(), "CONTROLLER HANDLER; Listening...");
    }
  }

  void InitRobot() {
    std::this_thread::sleep_for(sleep_time_);
    // while (true) {
    //   LowCmd cmd;
    //   for (int i = 0; i < 29; ++i) {
    //     cmd.motor_cmd[i].q = 0.0F;
    //     cmd.motor_cmd[i].dq = 0.0F;
    //     cmd.motor_cmd[i].tau = 0.0F;
    //     cmd.motor_cmd[i].kp = Kp[i];
    //     cmd.motor_cmd[i].kd = Kd[i];
    //     cmd.motor_cmd[i].mode = 1;
    //   }
    //   cmd.motor_cmd[29].q = 1.0F;

    //   cmd_pub_->publish(cmd);
    //   //RCLCPP_INFO(this->get_logger(), "Robot Initialized");
    // }
    // LowCmd cmd;
    // for (int i = 0; i < 29; ++i) {
    //   cmd.motor_cmd[i].q = 0.0F;
    //   cmd.motor_cmd[i].dq = 0.0F;
    //   cmd.motor_cmd[i].tau = 0.0F;
    //   cmd.motor_cmd[i].kp = Kp[i];
    //   cmd.motor_cmd[i].kd = Kd[i];
    //   cmd.motor_cmd[i].mode = 1;
    // }
    // cmd.motor_cmd[29].q = 1.0F;

    // cmd_pub_->publish(cmd);

    std::array<float, NUM_ARM_JOINTS> current_lowstate_arms;
    for (int i = 15; i < 29; i++) {
      current_lowstate_arms[i-15] = last_state_.motor_state[i].q;
    }
    MoveToFinal({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, current_lowstate_arms, 3.0F);

    RCLCPP_INFO(this->get_logger(), "Robot Initialized");
  }

  // Same as function above without e_stop or last_move flags
  void MoveToFinal(const std::array<float, NUM_ARM_JOINTS>& target,
              std::array<float, NUM_ARM_JOINTS>& current, float duration) {
    const int steps = static_cast<int>(duration / control_dt_);
    const std::array<float, NUM_ARM_JOINTS> initial = current;

    for (int i = 0; i < steps; ++i) {
      for (size_t j = 0; j < arm_joints_.size(); ++j) {
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
  void SendPositionCommand(const std::array<float, NUM_ARM_JOINTS>& positions) {
    LowCmd cmd;

    for (size_t i = 15; i < 29; ++i) {
      //int idx = static_cast<int>(arm_joints_[i]);
      cmd.motor_cmd[i].q = positions[i-15];
      cmd.motor_cmd[i].dq = 0.0F;
      cmd.motor_cmd[i].tau = 0.0F;
      //cmd.motor_cmd[i].mode = 1;
      cmd.motor_cmd[i].kp = Kp[i];
      cmd.motor_cmd[i].kd = Kd[i];
    }

    cmd.motor_cmd[static_cast<int>(NOT_USED_JOINT)].q = 1.0F;

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
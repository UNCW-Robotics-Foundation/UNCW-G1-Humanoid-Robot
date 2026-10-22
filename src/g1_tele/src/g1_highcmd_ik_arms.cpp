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
#include <g1_msgs/msg/g1_arm_debug.hpp>
#include <g1_msgs/msg/g1_debug.hpp>
#include <g1_msgs/msg/debug_data.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ruckig/ruckig.hpp"
#include <cmath>

#include "g1/g1.hpp"

using namespace std::chrono_literals;
//using LowCmd = unitree_hg::msg::LowCmd;
using LowState = unitree_hg::msg::LowState;
using ruckig::Ruckig;
using ruckig::InputParameter;
using ruckig::OutputParameter;
using ruckig::Result;

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
// const std::array<float, 29> Kp{
//     60, 60, 60, 100, 40, 40,      // legs
//     60, 60, 60, 100, 40, 40,      // legs
//     60, 40, 40,                   // waist
//     40, 40, 40, 40,  40, 40, 40,  // arms
//     40, 40, 40, 40,  40, 40, 40   // arms
// };

// // Damping for all G1 Joints
// const std::array<float, 29> Kd{
//     1, 1, 1, 2, 1, 1,     // legs
//     1, 1, 1, 2, 1, 1,     // legs
//     1, 1, 1,              // waist
//     1, 1, 1, 1, 1, 1, 1,  // arms
//     1, 1, 1, 1, 1, 1, 1   // arms
// };

// Stiffness for all G1 Joints
const std::array<float, 29> Kp{
    60, 60, 60, 100, 40, 40,      // legs
    60, 60, 60, 100, 40, 40,      // legs
    60, 40, 40,                   // waist
    80.0, 80.0, 80.0, 80.0, 40.0, 40.0, 40.0,  // arms
    80.0, 80.0, 80.0, 80.0, 40.0, 40.0, 40.0   // arms
};

// Damping for all G1 Joints
const std::array<float, 29> Kd{
    1, 1, 1, 2, 1, 1,     // legs
    1, 1, 1, 2, 1, 1,     // legs
    1, 1, 1,              // waist
    3.0, 3.0, 3.0, 3.0, 1.5, 1.5, 1.5,  // arms
    3.0, 3.0, 3.0, 3.0, 1.5, 1.5, 1.5   // arms
};

static constexpr int DOF = 7;
static constexpr double CONTROL_DT = 1.0 / 250.0;     // 250 Hz fast loop
static constexpr double RESYNC_POS_TOLERANCE = 0.02;  // rad, per-joint
static constexpr double max_v = 0.1;   // plan:  0.05    pen:  0.5
static constexpr double max_a = 0.2;    //        0.1          1.0
static constexpr double max_j = 1.25;    //        1.0          2.0

 public:
  ArmLowLevelController() : Node("arm_lowlevel_controller"), otg_(CONTROL_DT) {
    declare_parameter<std::vector<double>>("max_velocity",     {max_v, max_v, max_v, max_v, max_v, max_v, max_v});
    declare_parameter<std::vector<double>>("max_acceleration", {max_a, max_a, max_a, max_a, max_a, max_a, max_a});
    declare_parameter<std::vector<double>>("max_jerk",         {max_j, max_j, max_j, max_j, max_j, max_j, max_j});

    auto max_vel = get_parameter("max_velocity").as_double_array();
    auto max_acc = get_parameter("max_acceleration").as_double_array();
    auto max_jrk = get_parameter("max_jerk").as_double_array();

    for (int i = 0; i < DOF; ++i) {
      input_.max_velocity[i]         = max_vel[i];
      input_.max_acceleration[i]     = max_acc[i];
      input_.max_jerk[i]             = max_jrk[i];
      input_.current_position[i]     = 0.0;
      input_.current_velocity[i]     = 0.0;
      input_.current_acceleration[i] = 0.0;
      input_.target_position[i]      = 0.0;
      input_.target_velocity[i]      = 0.0;
      input_.target_acceleration[i]  = 0.0;
      measured_position_[i] = 0.0;
      measured_velocity_[i] = 0.0;
    }


    is_sim = false;
    // ROS2接口初始化
    if (is_sim) {
      cmd_pub_ = this->create_publisher<unitree_hg::msg::LowCmd>("/lowcmd", 10);
    } else {
      cmd_pub_ = this->create_publisher<unitree_hg::msg::LowCmd>("/arm_sdk", 10);
    }
    arm_joints_pub_ = this->create_publisher<g1_msgs::msg::ArmStates>("/arm_joints", 10);
    ruckig_joints_pub_ = this->create_publisher<g1_msgs::msg::ArmStates>("/joints/ruckig", 10);
    dbg_pub_ = this->create_publisher<g1_msgs::msg::G1Debug>("/main_dbg/lil", 10);
    dbg_pub2_ = this->create_publisher<g1_msgs::msg::DebugData>("/main_dbg/big", 10);
    dbg_pub3_ = this->create_publisher<g1_msgs::msg::G1ArmDebug>("/main_dbg/arm", 10);
    //ruckig_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/ruckig_state", 10);
    lowstate_sub_ = this->create_subscription<LowState>(
        "/lowstate", 10,
        [this](const LowState::SharedPtr msg) { StateCallback(msg); });
    ik_sub_ = this->create_subscription<g1_msgs::msg::ArmStates>(
        "/ik_sol", 10,
        [this](const g1_msgs::msg::ArmStates::SharedPtr msg) { IkCallback(msg); });
    tau_sub_ = this->create_subscription<g1_msgs::msg::ArmStates>(
        "/joints/tau", 10,
        [this](const g1_msgs::msg::ArmStates::SharedPtr msg) { TauCallback(msg); });

    joy_suber_ = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy", 10,
                [this](const sensor_msgs::msg::Joy::SharedPtr data) {
                JoyHandler(data);
                });

    timer_ = create_wall_timer(std::chrono::duration<double>(CONTROL_DT), std::bind(&ArmLowLevelController::ControlLoop, this));
  }

 private:
  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr cmd_pub_;
  rclcpp::Publisher<g1_msgs::msg::ArmStates>::SharedPtr arm_joints_pub_;
  rclcpp::Publisher<g1_msgs::msg::ArmStates>::SharedPtr ruckig_joints_pub_;
  rclcpp::Publisher<g1_msgs::msg::G1Debug>::SharedPtr dbg_pub_;
  rclcpp::Publisher<g1_msgs::msg::DebugData>::SharedPtr dbg_pub2_;
  rclcpp::Publisher<g1_msgs::msg::G1ArmDebug>::SharedPtr dbg_pub3_;
  //rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ruckig_state_pub_;
  rclcpp::Subscription<LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<g1_msgs::msg::ArmStates>::SharedPtr ik_sub_;
  rclcpp::Subscription<g1_msgs::msg::ArmStates>::SharedPtr tau_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_suber_; 

  rclcpp::TimerBase::SharedPtr timer_;

  LowState last_state_;
  LowState initial_lowstate_;
  g1_msgs::msg::ArmStates ik_sol;
  g1_msgs::msg::ArmStates sol_tau;
  bool state_received_ = false;

  float control_dt_{0.02F};
  float kp_{60.0F}, kd_{1.5F};
  float max_joint_velocity_{0.5F};

  std::array<g1_msgs::msg::MotorState, NUM_ARM_JOINTS> current_arm_pos_;
  g1_msgs::msg::ArmStates current_arms;

  // const float kp_high = 300.0;
  // const float kd_high = 3.0;
  const float kp_low = 80.0;
  const float kd_low = 3.0;
  const float kp_wrist = 40.0;
  const float kd_wrist = 1.5;
  const std::array<float, 7> kp_arm = {30.0, 30.0, 30.0, 20.0, 3.0, 3.0, 3.0};
  const std::array<float, 7> kd_arm = {1.25, 1.25, 1.25, 0.75, 1.0, 1.0, 1.0};
  // const std::array<float, 7> kp_arm = {35.0, 35.0, 35.0, 25.0, 4.0, 4.0, 4.0}; // max
  // const std::array<float, 7> kd_arm = {1.5, 1.5, 1.5, 1.0, 0.2, 0.2, 0.2};     // max
  // const std::array<float, 7> kp_arm = {20.0, 20.0, 20.0, 15.0, 1.5, 1.5, 1.5}; // min
  // const std::array<float, 7> kd_arm = {0.8, 0.8, 0.8, 0.5, 0.05, 0.05, 0.05};     // min

  bool btn_flag = false;
  bool init_flag = false;
  bool state_flag = false;
  bool stop_flag = false;
  bool ik_pub_flag = false;
  bool first_ik_flag = false;
  bool e_stop = false;

  float move_duration_ = 3.0F;
  float pd_scalar = 1.0;
  int node_state = 0;
  int last_button = 0;

  unitree_hg::msg::LowCmd final_cmd;

  Ruckig<DOF> otg_;
  InputParameter<DOF> input_;
  OutputParameter<DOF> output_;
  std::array<double, DOF> measured_position_{};
  std::array<double, DOF> measured_velocity_{};

  std::mutex state_mutex_;
  std::mutex target_mutex_;
  std::array<double, DOF> pending_target_{};
  //std::array<double, DOF> initial_target_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // original
  std::array<double, DOF> initial_target_ = {0.138933,-0.900519,-0.252580,0.049375,-0.520139,-0.539578,0.179715};    // modified
  std::array<double, DOF> exit_target_{};
  bool new_target_ = false;
  bool initialized_ = false;
  bool have_measured_state_ = false;

  bool is_sim = false;

  // std_msgs::msg::Bool ruckig_status;

  void StateCallback(const LowState::SharedPtr msg) {
    last_state_ = *msg;
    state_flag = true;
    std::lock_guard<std::mutex> lock(state_mutex_);

    // g1_msgs::msg::ArmStates current_arms;
    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      g1_msgs::msg::MotorState tmp_joint;
      tmp_joint.q = last_state_.motor_state[static_cast<int>(arm_joints_[i])].q;
      tmp_joint.dq = last_state_.motor_state[static_cast<int>(arm_joints_[i])].dq;
      current_arm_pos_[i] = tmp_joint;
      if (i >= 7) {
        measured_position_[i-7] = last_state_.motor_state[static_cast<int>(arm_joints_[i])].q;
        measured_velocity_[i-7] = last_state_.motor_state[static_cast<int>(arm_joints_[i])].dq;
      }
    }
    have_measured_state_ = true;
    current_arms.motor_states = current_arm_pos_;
    arm_joints_pub_->publish(current_arms);

  }

  void IkCallback(const g1_msgs::msg::ArmStates::SharedPtr msg) {
    ik_sol = *msg;
    if (ik_sol.motor_states.size() != 14) {
      RCLCPP_INFO(this->get_logger(), "IK solver returned unknown object");
      return;
    }

    std::lock_guard<std::mutex> lock(target_mutex_);
    for (int i = 7; i < 14; i++) {
      pending_target_[i-7] = ik_sol.motor_states[i].q;
    }
    new_target_ = true;

    if (!first_ik_flag) {
      for (int i = 7; i < 14; i++) {
        input_.current_position[i-7] = ik_sol.motor_states[i].q;
        //input_.current_position[i] = last_state_.motor_state[15 + i].q;
      }
      first_ik_flag = true;
    }
  }

  void TauCallback(const g1_msgs::msg::ArmStates::SharedPtr msg) {
    sol_tau = *msg;

    // for (int i = 22; i < 29; ++i) {
    //   final_cmd.motor_cmd[i].tau = sol_tau.motor_states[i-15].dq;
      //final_cmd.motor_cmd[i].tau = 0.0;
    // }
  }

  void JoyHandler(sensor_msgs::msg::Joy::SharedPtr message) {
    // Method for knowing when a butten has been released so presses don't repeat. Joy msg is different from wireless controller msg
    if (btn_flag) {
        if (message->buttons[last_button] == 0) {
          btn_flag = false;
        }
    } 

    else if (message->buttons[0] == 1) {   // A
      if ((node_state == 0) || (node_state == 2)) {
        if (node_state == 0) {
          for (int i = 0; i < 7; i++) {
            input_.current_position[i] = current_arms.motor_states[7 + i].q;
          }
          // input_.current_position[current_arms.motor_states[7].q, current_arms.motor_states[8].q, current_arms.motor_states[9].q, 
          //   current_arms.motor_states[10].q, current_arms.motor_states[11].q, current_arms.motor_states[12].q, 
          //   current_arms.motor_states[13].q];
          exit_target_ = {current_arms.motor_states[7].q, current_arms.motor_states[8].q, current_arms.motor_states[9].q, 
            current_arms.motor_states[10].q, current_arms.motor_states[11].q, current_arms.motor_states[12].q, 
            current_arms.motor_states[13].q};
          initial_lowstate_ = last_state_;
        }
        new_target_ = true;
        node_state ++;
        pd_scalar = 3.0;

        RCLCPP_INFO(this->get_logger(), "Moving to initial");
      }

      btn_flag = true;
      last_button = 0;
    }
    else if (message->buttons[3] == 1) {   // Y
      // btn_flag = true;
      // RCLCPP_INFO(this->get_logger(), "!!!ROBOT EMERGENCY STOP!!!");
    }
    else if (message->buttons[1] == 1) {   // B
      node_state = 99;
      new_target_ = true;

      btn_flag = true;
      last_button = 1;
      pd_scalar = 3.0;
      RCLCPP_INFO(this->get_logger(), "Exiting control");
    }
    else if (message->buttons[2] == 1) {   // X

      // btn_flag = true;
    }

  }

  void ControlLoop() {
    if (node_state < 1) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      if ((new_target_) && ((node_state == 1) || (node_state == 3))) {    // Initial position
        for (int i = 0; i < DOF; i++) {
          input_.target_position[i]     = initial_target_[i];
          input_.target_velocity[i]     = 0.0;
          input_.target_acceleration[i] = 0.0;

        }
        //  if (node_state == 1) {
        //   input_.current_position[current_arms.motor_states[7].q, current_arms.motor_states[8].q, current_arms.motor_states[9].q, 
        //     current_arms.motor_states[10].q, current_arms.motor_states[11].q, current_arms.motor_states[12].q, 
        //     current_arms.motor_states[13].q];
        //  }
        new_target_ = false;

      } else if ((new_target_) && (node_state == 2)) {                     // Listen to ik node
        for (int i = 0; i < DOF; i++) {
          input_.target_position[i]     = pending_target_[i];
          input_.target_velocity[i]     = 0.0;
          input_.target_acceleration[i] = 0.0;

        }
        new_target_ = false;

      } else if ((new_target_) && (node_state == 99)) {                     // Exit control
        for (int i = 0; i < DOF; i++) {
          input_.target_position[i]     = exit_target_[i];
          input_.target_velocity[i]     = 0.0;
          input_.target_acceleration[i] = 0.0;

        }
        new_target_ = false;
      }
    }

    //MaybeResync();

    Result res = otg_.update(input_, output_);
    if (res != Result::Working && res != Result::Finished) {
      RCLCPP_ERROR(get_logger(), "Ruckig update failed (code %d)", static_cast<int>(res));
      return;
    } else if (res == Result::Finished) {
      //RCLCPP_INFO(this->get_logger(), "Robot main trajectory finished");
      // ruckig_status.data = false;
      if (node_state == 1) {
        // pd_scalar = 10.0;
        if (is_sim) {
          pd_scalar = 4.0;
        } else {
          pd_scalar = 10.0;
        }
        node_state = 2;
      } else if (node_state == 99) {
        PublishCommand(0.0);
        node_state = 100;
      }
      return;
    }

    output_.pass_to_input(input_);

    PublishCommand(1.0);
    // ruckig_state_pub_->publish(ruckig_status);

  }

  void MaybeResync() {
    std::array<double, DOF> meas_pos{}, meas_vel{};
    bool have_state;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      have_state = have_measured_state_;
      meas_pos = measured_position_;
      meas_vel = measured_velocity_;
    }
    if (!have_state) return;
 
    double max_err = 0.0;
    for (int i = 0; i < DOF; ++i) {
      max_err = std::max(max_err, std::abs(meas_pos[i] - input_.current_position[i]));
    }
 
    if (max_err > RESYNC_POS_TOLERANCE) {
      RCLCPP_INFO(this->get_logger(), "Resyncing Ruckig");
      // Resync position AND velocity together -- never just one.
      // current_acceleration is deliberately left alone (Ruckig's own
      // last estimate), since we don't have a clean measured acceleration.
      for (int i = 0; i < DOF; ++i) {
        input_.current_position[i] = meas_pos[i];
        input_.current_velocity[i] = meas_vel[i];
      }
    }
  }

  void PublishCommand(float control) {
    unitree_hg::msg::LowCmd cmd;
    for (int i = 12; i < 29; ++i) {
      // cmd.motor_cmd[i].q = output_.new_position[i-22];
      // cmd.motor_cmd[i].dq = output_.new_velocity[i-22];
      // cmd.motor_cmd[i].tau = sol_tau.motor_states[i-15].dq;
      // cmd.motor_cmd[i].kp = kp_arm[(i - 22) % 7] * 10.0;
      // cmd.motor_cmd[i].kd = kd_arm[(i - 22) % 7] * 10.0;
      if ((i >= 22) && (i < 29)) {
        cmd.motor_cmd[i].q = output_.new_position[i-22];
        cmd.motor_cmd[i].dq = output_.new_velocity[i-22];
        //cmd.motor_cmd[i].dq = 0.0F;
        cmd.motor_cmd[i].tau = sol_tau.motor_states[i-15].dq;
        cmd.motor_cmd[i].kp = kp_arm[(i - 22) % 7] * pd_scalar;
        cmd.motor_cmd[i].kd = kd_arm[(i - 22) % 7] * pd_scalar;
        // cmd.motor_cmd[i].kp = Kp[i];
        // cmd.motor_cmd[i].kd = Kd[i];
      } else {
        cmd.motor_cmd[i].q = initial_lowstate_.motor_state[i].q;
        cmd.motor_cmd[i].dq = 0.0;
        cmd.motor_cmd[i].tau = 0.0;
        if ((i >= 12) && (i < 15)) {
          cmd.motor_cmd[i].kp = Kp[i] * 4.0;
          cmd.motor_cmd[i].kd = Kd[i] * 4.0;
        } else {
          cmd.motor_cmd[i].kp = Kp[i];
          cmd.motor_cmd[i].kd = Kd[i];
        }
      }
    }

    cmd.motor_cmd[static_cast<int>(NOT_USED_JOINT)].q = control;

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
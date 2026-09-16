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
#include "motor_crc_hg.h"
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
static constexpr double max_v = 0.1;
static constexpr double max_a = 0.2;
static constexpr double max_j = 1.5;

 public:
  ArmLowLevelController() : Node("arm_lowlevel_controller"), otg_(CONTROL_DT) {
    // declare_parameter<std::vector<double>>("max_velocity",     {3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0});
    // declare_parameter<std::vector<double>>("max_acceleration", {8.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0});
    // declare_parameter<std::vector<double>>("max_jerk",         {40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0});
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



    // ROS2接口初始化
    //cmd_pub_ = this->create_publisher<unitree_hg::msg::LowCmd>("/arm_sdk", 10);
    cmd_pub_ = this->create_publisher<unitree_hg::msg::LowCmd>("/lowcmd", 10);
    arm_joints_pub_ = this->create_publisher<g1_msgs::msg::ArmStates>("/arm_joints", 10);
    dbg_pub_ = this->create_publisher<g1_msgs::msg::G1Debug>("/main_dbg/lil", 10);
    dbg_pub2_ = this->create_publisher<g1_msgs::msg::DebugData>("/main_dbg/big", 10);
    dbg_pub3_ = this->create_publisher<g1_msgs::msg::G1ArmDebug>("/main_dbg/arm", 10);
    ruckig_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/ruckig_state", 10);
    lowstate_sub_ = this->create_subscription<LowState>(
        "/lowstate", 10,
        [this](const LowState::SharedPtr msg) { StateCallback(msg); });
    ik_sub_ = this->create_subscription<g1_msgs::msg::ArmStates>(
        "/ik_sol", 10,
        [this](const g1_msgs::msg::ArmStates::SharedPtr msg) { IkCallback(msg); });

    sleep_time_ =
        std::chrono::milliseconds(static_cast<int>(control_dt_ * 1000));

    thread_ = std::thread([this]() { InitRobot(); });

    joy_suber_ = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy", 10,
                [this](const sensor_msgs::msg::Joy::SharedPtr data) {
                JoyHandler(data);
                });
    // timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
    //                                   [this] { ControlLoop(); });
    timer_ = create_wall_timer(std::chrono::duration<double>(CONTROL_DT), std::bind(&ArmLowLevelController::ControlLoop, this));
  }

 private:
  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr cmd_pub_;
  rclcpp::Publisher<g1_msgs::msg::ArmStates>::SharedPtr arm_joints_pub_;
  rclcpp::Publisher<g1_msgs::msg::G1Debug>::SharedPtr dbg_pub_;
  rclcpp::Publisher<g1_msgs::msg::DebugData>::SharedPtr dbg_pub2_;
  rclcpp::Publisher<g1_msgs::msg::G1ArmDebug>::SharedPtr dbg_pub3_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ruckig_state_pub_;
  rclcpp::Subscription<LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<g1_msgs::msg::ArmStates>::SharedPtr ik_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_suber_; 
  std::thread thread_;
  rclcpp::TimerBase::SharedPtr timer_;

  LowState last_state_;
  g1_msgs::msg::ArmStates ik_sol;
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

  unitree_hg::msg::LowCmd final_cmd;
  unitree_hg::msg::LowCmd zero_cmd;

  Ruckig<DOF> otg_;
  InputParameter<DOF> input_;
  OutputParameter<DOF> output_;
  std::array<double, DOF> measured_position_{};
  std::array<double, DOF> measured_velocity_{};

  std::mutex state_mutex_;
  std::mutex target_mutex_;
  std::array<double, DOF> pending_target_{};
  bool new_target_ = false;
  bool initialized_ = false;
  bool have_measured_state_ = false;

  std_msgs::msg::Bool ruckig_status;

  void StateCallback(const LowState::SharedPtr msg) {
    last_state_ = *msg;
    state_flag = true;
    std::lock_guard<std::mutex> lock(state_mutex_);

    g1_msgs::msg::ArmStates current_arms;
    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      g1_msgs::msg::MotorState tmp_joint;
      tmp_joint.q = last_state_.motor_state[static_cast<int>(arm_joints_[i])].q;
      tmp_joint.dq = last_state_.motor_state[static_cast<int>(arm_joints_[i])].dq;
      current_arm_pos_[i] = tmp_joint;
      if (i < 7) {
        measured_position_[i] = last_state_.motor_state[static_cast<int>(arm_joints_[i])].q;
        measured_velocity_[i] = last_state_.motor_state[static_cast<int>(arm_joints_[i])].dq;
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
    for (int i = 0; i < 7; i++) {
      pending_target_[i] = ik_sol.motor_states[i].q;
    }
    new_target_ = true;

    if (!first_ik_flag) {
      for (int i = 0; i < 7; i++) {
        input_.current_position[i] = ik_sol.motor_states[i].q;
        //input_.current_position[i] = last_state_.motor_state[15 + i].q;
      }
      first_ik_flag = true;
    }

    // if (!first_ik_flag) {
    //   first_ik_flag = true;
    // }


    for (int i = 15; i < 22; ++i) {
      // final_cmd.motor_cmd[i].q = ik_sol.motor_states[i-15].q;
      // final_cmd.motor_cmd[i].dq = 0.0F;
      final_cmd.motor_cmd[i].tau = ik_sol.motor_states[i-15].dq;
      //final_cmd.motor_cmd[i].tau = 0.0F;
      // final_cmd.motor_cmd[i].kp = kp_arm[(i - 15) % 7];
      // final_cmd.motor_cmd[i].kd = kd_arm[(i - 15) % 7];
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

  void ControlLoop() {
    // if (!first_ik_flag) {
    //   return;
    // }

    if (!ik_pub_flag) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      if (new_target_) {
        for (int i = 0; i < DOF; i++) {
          input_.target_position[i]     = pending_target_[i];
          input_.target_velocity[i]     = 0.0;
          input_.target_acceleration[i] = 0.0;

        }
        new_target_ = false;
        ruckig_status.data = true;
      }
    }

    //MaybeResync();

    Result res = otg_.update(input_, output_);
    if (res != Result::Working && res != Result::Finished) {
      RCLCPP_ERROR(get_logger(), "Ruckig update failed (code %d)", static_cast<int>(res));
      return;
    } else if (res == Result::Finished) {
      RCLCPP_INFO(this->get_logger(), "Robot main trajectory finished");
      ruckig_status.data = false;
    }

    output_.pass_to_input(input_);

    PublishCommand();
    ruckig_state_pub_->publish(ruckig_status);
    
    
    // if ((!first_ik_flag) && (ik_pub_flag) && (!e_stop)) {
    //   get_crc(zero_cmd);
    //   cmd_pub_->publish(zero_cmd);
    // }
    // else if ((first_ik_flag) && (ik_pub_flag) && (!e_stop)) {
    //   get_crc(final_cmd);
    //   cmd_pub_->publish(final_cmd);
    // }
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

  void PublishCommand() {
    if (!first_ik_flag){ 
      get_crc(final_cmd);
      cmd_pub_->publish(final_cmd);
      return;
    }
    for (int i = 15; i < 22; ++i) {
      final_cmd.motor_cmd[i].q = output_.new_position[i-15];
      final_cmd.motor_cmd[i].dq = output_.new_velocity[i-15];
      //final_cmd.motor_cmd[i].dq = 0.0F;
      //final_cmd.motor_cmd[i].tau = ik_sol.motor_states[i-15].dq;
      final_cmd.motor_cmd[i].kp = kp_arm[(i - 15) % 7];
      final_cmd.motor_cmd[i].kd = kd_arm[(i - 15) % 7];
    }
    // g1_msgs::msg::G1Debug dbg;
    // dbg.target_q = output_.new_velocity[0];
    // dbg_pub_->publish(dbg);
    // g1_msgs::msg::DebugData dbg2;
    // dbg2.target_tx = last_state_.motor_state[18].q;
    // dbg2.target_ty = abs(final_cmd.motor_cmd[18].q - last_state_.motor_state[18].q);
    // dbg2.target_tz = ik_sol.motor_states[3].dq;
    // dbg2.delta_tx = last_state_.motor_state[15].q;
    // dbg_pub2_->publish(dbg2);
    g1_msgs::msg::G1ArmDebug dbg3;
    dbg3.elbow_angle = last_state_.motor_state[18].q;
    dbg3.elbow_error = abs(final_cmd.motor_cmd[18].q - last_state_.motor_state[18].q);
    dbg3.elbow_tau = ik_sol.motor_states[3].dq;
    dbg3.wrist_roll_angle = last_state_.motor_state[19].q;
    dbg3.wrist_roll_error = abs(final_cmd.motor_cmd[19].q - last_state_.motor_state[19].q);
    dbg3.wrist_roll_tau = ik_sol.motor_states[4].dq;
    dbg3.wrist_pitch_angle = last_state_.motor_state[20].q;
    dbg3.wrist_pitch_error = abs(final_cmd.motor_cmd[20].q - last_state_.motor_state[20].q);
    dbg3.wrist_pitch_tau = ik_sol.motor_states[5].dq;
    dbg3.wrist_yaw_angle = last_state_.motor_state[21].q;
    dbg3.wrist_yaw_error = abs(final_cmd.motor_cmd[21].q - last_state_.motor_state[21].q);
    dbg3.wrist_yaw_tau = ik_sol.motor_states[6].dq;
    dbg3.shoulder_pitch = last_state_.motor_state[15].q;
    dbg_pub3_->publish(dbg3);

    get_crc(final_cmd);
    cmd_pub_->publish(final_cmd);
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
      // -0.5/0.5 - shoulder_pitch; 0.75/-0.25 - elbow
      // if (i == 15) {
      //   zero_initial[i] = 0.5;
      //   tmp_cmd.q = 0.5;
      // } else if (i == 18) {
      //   zero_initial[i] = -0.25;
      //   tmp_cmd.q = -0.25;
      // } else {
      //   zero_initial[i] = 0.0;
      //   tmp_cmd.q = 0.0;
      // }
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
/**
 * This example demonstrates how to use ROS2 to control ankle commands of
 *unitree g1 robot
 **/
#include <iomanip>

#include "common/motor_crc_hg.h"
#include "gamepad.hpp"
#include "motor_crc_hg.h"
#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/imu_state.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>
using namespace std::chrono_literals;

const auto HG_CMD_TOPIC = "lowcmd";
const auto HG_STATE_TOPIC = "bag/lowstate";
constexpr float PI = 3.14159265358979323846F;
template <typename T>
class DataBuffer {
 public:
  void SetData(const T &new_data) {
    std::lock_guard<std::mutex> const lock(mutex_);
    data_ = std::make_shared<T>(new_data);
  }

  std::shared_ptr<const T> GetData() {
    std::lock_guard<std::mutex> const lock(mutex_);
    return data_ ? data_ : nullptr;
  }

  void Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    data_ = nullptr;
  }

 private:
  std::shared_ptr<T> data_;
  std::mutex mutex_;
};

const int G1_NUM_MOTOR = 29;
struct ImuState {
  std::array<float, 3> rpy = {};
  std::array<float, 3> omega = {};
};
struct MotorCommand {
  std::array<float, G1_NUM_MOTOR> q_target = {};
  std::array<float, G1_NUM_MOTOR> dq_target = {};
  std::array<float, G1_NUM_MOTOR> kp = {};
  std::array<float, G1_NUM_MOTOR> kd = {};
  std::array<float, G1_NUM_MOTOR> tau_ff = {};
};
struct MotorState {
  std::array<float, G1_NUM_MOTOR> q = {};
  std::array<float, G1_NUM_MOTOR> dq = {};
};

// Stiffness for all G1 Joints
const std::array<float, G1_NUM_MOTOR> Kp{
    60, 60, 60, 100, 40, 40,      // legs
    60, 60, 60, 100, 40, 40,      // legs
    60, 40, 40,                   // waist
    40, 40, 40, 40,  40, 40, 40,  // arms
    40, 40, 40, 40,  40, 40, 40   // arms
};

// Damping for all G1 Joints
const std::array<float, G1_NUM_MOTOR> Kd{
    1, 1, 1, 2, 1, 1,     // legs
    1, 1, 1, 2, 1, 1,     // legs
    1, 1, 1,              // waist
    1, 1, 1, 1, 1, 1, 1,  // arms
    1, 1, 1, 1, 1, 1, 1   // arms
};

enum class Mode {
  PR = 0,  // Series Control for Ptich/Roll Joints
  AB = 1   // Parallel Control for A/B Joints
};

enum G1JointIndex {
  LEFT_HIP_PITCH = 0,
  LEFT_HIP_ROLL = 1,
  LEFT_HIP_YAW = 2,
  LEFT_KNEE = 3,
  LEFT_ANKLE_PITCH = 4,
  LEFT_ANKLE_B = 4,
  LEFT_ANKLE_ROLL = 5,
  LEFT_ANKLE_A = 5,
  RIGHT_HIP_PITCH = 6,
  RIGHT_HIP_ROLL = 7,
  RIGHT_HIP_YAW = 8,
  RIGHT_KNEE = 9,
  RIGHT_ANKLE_PITCH = 10,
  RIGHT_ANKLE_B = 10,
  RIGHT_ANKLE_ROLL = 11,
  RIGHT_ANKLE_A = 11,
  WAIST_YAW = 12,
  WAIST_ROLL = 13,   // NOTE INVALID for g1 23dof/29dof with waist locked
  WAIST_A = 13,      // NOTE INVALID for g1 23dof/29dof with waist locked
  WAIST_PITCH = 14,  // NOTE INVALID for g1 23dof/29dof with waist locked
  WAIST_B = 14,      // NOTE INVALID for g1 23dof/29dof with waist locked
  LEFT_SHOULDER_PITCH = 15,
  LEFT_SHOULDER_ROLL = 16,
  LEFT_SHOULDER_YAW = 17,
  LEFT_ELBOW = 18,
  LEFT_WRIST_ROLL = 19,
  LEFT_WRIST_PITCH = 20,  // NOTE INVALID for g1 23dof
  LEFT_WRIST_YAW = 21,    // NOTE INVALID for g1 23dof
  RIGHT_SHOULDER_PITCH = 22,
  RIGHT_SHOULDER_ROLL = 23,
  RIGHT_SHOULDER_YAW = 24,
  RIGHT_ELBOW = 25,
  RIGHT_WRIST_ROLL = 26,
  RIGHT_WRIST_PITCH = 27,  // NOTE INVALID for g1 23dof
  RIGHT_WRIST_YAW = 28     // NOTE INVALID for g1 23dof
};

class GestureTxtTest : public rclcpp::Node {
 public:
  GestureTxtTest() : Node("Gesture_txt_test"), mode_machine_(0) {
    // the mLowcmdPublisher is set to publish "/lowcmd" topic
    lowcmd_publisher_ =
        this->create_publisher<unitree_hg::msg::LowCmd>(HG_CMD_TOPIC, 10);

    thread_ = std::thread([this]() { ControlLoop(); });
  }

 private:
  std::thread thread_;
  int count = 0;
  std::string gesture_data;

  void ControlLoop() {
    RCLCPP_INFO(this->get_logger(), "Performing gesture...");
    unitree_hg::msg::LowCmd low_command;
    low_command.mode_pr = static_cast<uint8_t>(mode_pr_);
    low_command.mode_machine = 5;
    int i = 0;

    std::ifstream GestureTxtFile("converted_test.txt");
    while (getline (GestureTxtFile, gesture_data)) {
      low_command.motor_cmd.at(i).mode = 1;  // 1:Enable, 0:Disable
      low_command.motor_cmd.at(i).tau = 0.0;
      low_command.motor_cmd.at(i).q = std::stof(gesture_data);
      low_command.motor_cmd.at(i).dq = 0.0;
      low_command.motor_cmd.at(i).kp = Kp[i];
      low_command.motor_cmd.at(i).kd = Kd[i];

      if (!(i + 1 < 29)) {
        get_crc(low_command);
        lowcmd_publisher_->publish(low_command);
        i = 0;
        std::this_thread::sleep_for(1ms);
      } else{
        i++;
      }
      
    }
    GestureTxtFile.close();
    RCLCPP_INFO(this->get_logger(), "Gesture complete.");
  }

  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr
      lowcmd_publisher_;  // ROS2 Publisher
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr
      bag_lowstate_subscriber_;  // ROS2 Subscriber

  double time_{0.0};
  double control_dt_{0.002};  // [2ms]
  double duration_{3.0};      // [3 s]
  int32_t counter_{0};
  Mode mode_pr_{Mode::PR};
  std::atomic<uint8_t> mode_machine_;

  int32_t cmd_motor_index_{0};
  int32_t degree_{0};
  int32_t cmd_reference_{0};
  bool listen_flag_{false};
  bool new_cmd_flag_{false};

  DataBuffer<MotorState> motor_state_buffer_;
  DataBuffer<MotorCommand> motor_command_buffer_;
  DataBuffer<ImuState> imu_state_buffer_;

  std::array<float, 29> g1JointStates;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);  // Initialize rclcpp
  auto node =
      std::make_shared<GestureTxtTest>();  // Create a ROS2 node and make
                                               // share with
                                               // g1_bag_test class
  rclcpp::spin(node);                          // Run ROS2 node
  rclcpp::shutdown();                          // Exit
  return 0;
}
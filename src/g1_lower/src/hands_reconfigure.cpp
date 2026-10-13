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

class HandsReconfigure : public rclcpp::Node {
 public:
  HandsReconfigure() : Node("hands_reconfigure_node") {
    rcl_interfaces::msg::FloatingPointRange float_range;
    float_range.from_value = 0.0;
    float_range.to_value = 1.0;
    float_range.step = 0.01;

    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.floating_point_range.push_back(float_range);

    this->declare_parameter("left_thumb1", 0.0, desc);
    this->declare_parameter("left_thumb2", 0.0, desc);
    this->declare_parameter("left_pointer", 0.0, desc);
    this->declare_parameter("left_middle", 0.0, desc);
    this->declare_parameter("left_ring", 0.0, desc);
    this->declare_parameter("left_pinky", 0.0, desc);
    this->declare_parameter("right_thumb1", 0.0, desc);
    this->declare_parameter("right_thumb2", 0.0, desc);
    this->declare_parameter("right_pointer", 0.0, desc);
    this->declare_parameter("right_middle", 0.0, desc);
    this->declare_parameter("right_ring", 0.0, desc);
    this->declare_parameter("right_pinky", 0.0, desc);


    right_pub_ = this->create_publisher<unitree_go::msg::MotorCmds>("/brainco/right/cmd", 10);
    left_pub_ = this->create_publisher<unitree_go::msg::MotorCmds>("/brainco/left/cmd", 10);
    sub_ = this->create_subscription<unitree_go::msg::MotorStates>(
        "/brainco/right/state", 10,
        [this](const unitree_go::msg::MotorStates::SharedPtr msg) { StateCallback(msg); });
    touch_sub_ = this->create_subscription<unitree_go::msg::MotorStates>(
        "/brainco/right/touch", 10,
        [this](const unitree_go::msg::MotorStates::SharedPtr msg) { TouchCallback(msg); });

    timer1_ = this->create_wall_timer(std::chrono::milliseconds(2), [this] { Control(); });
    param_callback_ = this->add_on_set_parameters_callback(
      std::bind(&HandsReconfigure::on_parameters_change, this, std::placeholders::_1)
    );
  }


  private:
  rclcpp::Publisher<unitree_go::msg::MotorCmds>::SharedPtr right_pub_;
  rclcpp::Publisher<unitree_go::msg::MotorCmds>::SharedPtr left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub2_;
  rclcpp::Subscription<unitree_go::msg::MotorStates>::SharedPtr sub_;
  rclcpp::Subscription<unitree_go::msg::MotorStates>::SharedPtr touch_sub_;
  rclcpp::TimerBase::SharedPtr timer1_;
  unitree_go::msg::MotorStates current_state;
  unitree_go::msg::MotorStates touch_state;
  std::array<float, 6> positions;
  std::vector<double> positions2;
  std::vector<double> speeds;
  std::array<float,6> current_pos{}; 
  std::array<float,5> current_touch{}; 

  bool state_received_ = false;
  bool touch_state_received_ = false;
  bool touch_flag = false;

  bool init_flag = true;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_;
  unitree_go::msg::MotorCmds leftHandCmds;
  unitree_go::msg::MotorCmds rightHandCmds;

  rcl_interfaces::msg::SetParametersResult on_parameters_change(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Finger update applied successfully";
    RCLCPP_INFO(this->get_logger(), "Received Parameter Request");

    for (const auto &param : parameters) {
      const std::string &name = param.get_name();

      if (name == "left_thumb1") {
        leftHandCmds.cmds[0].q = param.as_double();
        left_pub_->publish(leftHandCmds);
      } else if (name == "left_thumb2") {
        leftHandCmds.cmds[1].q = param.as_double();
        left_pub_->publish(leftHandCmds);
      } else if (name == "left_pointer") {
        leftHandCmds.cmds[2].q = param.as_double();
        left_pub_->publish(leftHandCmds);
      } else if (name == "left_middle") {
        leftHandCmds.cmds[3].q = param.as_double();
        left_pub_->publish(leftHandCmds);
      } else if (name == "left_ring") {
        leftHandCmds.cmds[4].q = param.as_double();
        left_pub_->publish(leftHandCmds);
      } else if (name == "left_pinky") {
        leftHandCmds.cmds[5].q = param.as_double();
        left_pub_->publish(leftHandCmds);
      } else if (name == "right_thumb1") {
        rightHandCmds.cmds[0].q = param.as_double();
        right_pub_->publish(rightHandCmds);
      } else if (name == "right_thumb2") {
        rightHandCmds.cmds[1].q = param.as_double();
        right_pub_->publish(rightHandCmds);
      } else if (name == "right_pointer") {
        rightHandCmds.cmds[2].q = param.as_double();
        right_pub_->publish(rightHandCmds);
      } else if (name == "right_middle") {
        rightHandCmds.cmds[3].q = param.as_double();
        right_pub_->publish(rightHandCmds);
      } else if (name == "right_ring") {
        rightHandCmds.cmds[4].q = param.as_double();
        right_pub_->publish(rightHandCmds);
      } else if (name == "right_pinky") {
        rightHandCmds.cmds[5].q = param.as_double();
        right_pub_->publish(rightHandCmds);
      }
    }

    return result;
  }

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

  void Control() {
    if ((!(state_received_) || !(touch_state_received_))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for initial hand pose and touch data...");
      return;
    }
    if (init_flag) {
      init_flag = false;
      InitHands();
      return;
    }

  }

  void InitHands() {
    // while (!(state_received_) || !(touch_state_received_)){
    //   RCLCPP_INFO(this->get_logger(), "Waiting for initial hand pose and touch data...");
    //   std::this_thread::sleep_for(100ms);
    // }
    RCLCPP_INFO(this->get_logger(), "Opening hand...");
    std::vector<unitree_go::msg::MotorCmd> tmpCommands;
    positions = {0, 0, 0, 0, 0, 0};
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
    rightHandCmds.cmds = tmpCommands;
    leftHandCmds.cmds = tmpCommands;
    right_pub_->publish(rightHandCmds);
    left_pub_->publish(leftHandCmds);
  }

};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);  // Initialize rclcpp
  // Run ROS2 node which is make share with wireless_controller_suber class
  rclcpp::spin(std::make_shared<HandsReconfigure>());
  rclcpp::shutdown();
  return 0;
}
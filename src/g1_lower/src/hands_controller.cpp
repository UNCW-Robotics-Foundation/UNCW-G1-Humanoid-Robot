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

using namespace std::chrono_literals;

class HandsController : public rclcpp::Node {
 public:
  HandsController() : Node("hands_controller_node") {
    l_pub_ = this->create_publisher<unitree_go::msg::MotorCmds>("/brainco/left/cmd", 10);
    r_pub_ = this->create_publisher<unitree_go::msg::MotorCmds>("/brainco/right/cmd", 10);
    suber_ = this->create_subscription<unitree_go::msg::WirelessController>(
        "/wirelesscontroller", 10,
        [this](const unitree_go::msg::WirelessController::SharedPtr data) {
          WirelessCallback(data);
        }); 
    //thread_ = std::thread([this]() { ControlLoop(); });

  }


  private:
  rclcpp::Publisher<unitree_go::msg::MotorCmds>::SharedPtr l_pub_;
  rclcpp::Publisher<unitree_go::msg::MotorCmds>::SharedPtr r_pub_;
  rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr suber_;
  //std::thread thread_;
  //std::array<float, 6> positions; 

  const std::array<float, 6> open_hand {0, 0, 0, 0, 0, 0}; 
  const std::array<float, 6> fist {1, 1, 1, 1, 1, 1}; 
  const std::array<float, 6> thumb_out {0, 0, 1, 1, 1, 1}; 
  const std::array<float, 6> point {1, 1, 0, 1, 1, 1}; 

  bool btn_flag = false;
  bool left = true;
  bool right = false;

  void MoveHands(std::array<float, 6> positions, bool pub_flag) {
    std::vector<unitree_go::msg::MotorCmd> tmpCommands;
    unitree_go::msg::MotorCmds handCmds;

    //positions = {0, 0, 0, 0, 0, 0};
    for (int i = 0; i < positions.size(); i++) {
        unitree_go::msg::MotorCmd handCmd;
        handCmd.q = positions[i];
        handCmd.dq = 1.0F;
        handCmd.tau = 0.0F;
        handCmd.kp = 0.0F;
        handCmd.kd = 0.0F;
        handCmd.mode = 0;
        tmpCommands.push_back(handCmd);
    }
    handCmds.cmds = tmpCommands;
    if (pub_flag) {
      l_pub_->publish(handCmds);
    } else {
      r_pub_->publish(handCmds);
    }
  }

  /*
  Function for wireless_controller subscriber. Any modifications to button inputs and/or 
  gestures need to be made here. I recommend using F1 + <button> for left hand, F1 + <button> for 
  right hand, and F1 + F2 + <button> for both hands. The controller sends an integer value when a 
  button is pressed. I think they are unique powers of 2, so each button press SHOULD be unique,
  but there are still some bugs because pressing X and A at the same time sometimes performs a
  clap even though it is assigned to double tap A. Double check an input is free by testing in
  robot running mode.
  */
  void WirelessCallback(const unitree_go::msg::WirelessController::SharedPtr& data) {
    // Buttons flip the btn_flag (flaps back when released), the busy_flag (flips back when gesture completes), and e_stop flag (flips back when gesture completes)
    if ((data->keys == 65) && (btn_flag)) {         // f1 + r1
      MoveHands(open_hand, left);
      btn_flag = false;
    } else if ((data->keys == 129) && (btn_flag)) { // f3 + r1
      MoveHands(open_hand, right);
      btn_flag = false;
    }else if ((data->keys == 193) && (btn_flag)) {  // f1 + f3 + r1
      MoveHands(open_hand, left);
      MoveHands(open_hand, right);
      btn_flag = false;
    }else if ((data->keys == 80) && (btn_flag)) { // f1 + r2
      MoveHands(fist, left);
      btn_flag = false;
    }else if ((data->keys == 144) && (btn_flag)) { // f3 + r2
      MoveHands(fist, right);
      btn_flag = false;
    }else if ((data->keys == 208) && (btn_flag)) { // f1 + f3 + r2
      MoveHands(fist, left);
      MoveHands(fist, right);
      btn_flag = false;
    }else if ((data->keys == 66) && (btn_flag)) { // f1 + l1
      MoveHands(thumb_out, left);
      btn_flag = false;
    }else if ((data->keys == 130) && (btn_flag)) { // f3 + l1
      MoveHands(thumb_out, right);
      btn_flag = false;
    }else if ((data->keys == 194) && (btn_flag)) { // f1 + f3 + l1
      MoveHands(thumb_out, left);
      MoveHands(thumb_out, right);
      btn_flag = false;
    }else if ((data->keys == 96) && (btn_flag)) { // f1 + l2
      MoveHands(point, left);
      btn_flag = false;
    }else if ((data->keys == 160) && (btn_flag)) { // f3 + l2
      MoveHands(point, right);
      btn_flag = false;
    }else if ((data->keys == 224) && (btn_flag)) { // f1 + f3 + l2
      MoveHands(point, left);
      MoveHands(point, right);
      btn_flag = false;
  
    // Method for avoiding held presses. While btn_flag is false, nothing happens.
    }else if ((data->keys == 0) && !(btn_flag)) {
      btn_flag = true;
    }

  }

};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);  // Initialize rclcpp
  // Run ROS2 node which is make share with wireless_controller_suber class
  rclcpp::spin(std::make_shared<HandsController>());
  rclcpp::shutdown();
  return 0;
}
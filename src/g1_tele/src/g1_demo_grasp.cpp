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
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

class HandsController : public rclcpp::Node {
 public:
  HandsController() : Node("hands_controller_node") {
    l_pub_ = this->create_publisher<unitree_go::msg::MotorCmds>("/brainco/left/cmd", 10);
    r_pub_ = this->create_publisher<unitree_go::msg::MotorCmds>("/brainco/right/cmd", 10);
    point_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/demo/hand", 10);
    suber_ = this->create_subscription<unitree_go::msg::WirelessController>(
      "/wirelesscontroller", 10,
      [this](const unitree_go::msg::WirelessController::SharedPtr data) {
        WirelessCallback(data);
      }); 
    l_touch_sub_ = this->create_subscription<unitree_go::msg::MotorStates>(
      "/brainco/left/touch", 10,
      [this](const unitree_go::msg::MotorStates::SharedPtr data) {
        TouchCallback(data);
      }); 
    l_state_sub_ = this->create_subscription<unitree_go::msg::MotorStates>(
      "/brainco/left/state", 10,
      [this](const unitree_go::msg::MotorStates::SharedPtr data) {
        StateCallback(data);
      }); 
    point_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/demo/point", 10,
      [this](const geometry_msgs::msg::Point::SharedPtr data) {
        PointCallback(data);
      }); 

    thread_ = std::thread([this]() { ContinuousMove(); });
    timer_ = this->create_wall_timer(std::chrono::milliseconds(2), [this] { Control(); });

  }


  private:
  rclcpp::Publisher<unitree_go::msg::MotorCmds>::SharedPtr l_pub_;
  rclcpp::Publisher<unitree_go::msg::MotorCmds>::SharedPtr r_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr point_pub_;
  rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr suber_; 
  rclcpp::Subscription<unitree_go::msg::MotorStates>::SharedPtr l_touch_sub_; 
  rclcpp::Subscription<unitree_go::msg::MotorStates>::SharedPtr l_state_sub_; 
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point_sub_;
  std::thread thread_;
  rclcpp::TimerBase::SharedPtr timer_;

  const std::array<float, 6> open_hand {0, 0, 0, 0, 0, 0}; 
  const std::array<float, 6> fist {1, 1, 1, 1, 1, 1}; 
  const std::array<float, 6> pre_fist {0, 1, 1, 1, 1, 1}; 
  const std::array<float, 6> thumb_out {0, 0, 1, 1, 1, 1}; 
  const std::array<float, 6> point {1, 1, 0, 1, 1, 1}; 
  const std::array<float, 6> pre_point {0, 1, 0, 1, 1, 1}; 
  const std::array<float, 6> wings {0, 0, 0.3, 0.3, 0.3, 0.3}; 
  const std::array<float, 6> peace {1, 1, 0.0, 0.0, 1, 1}; 
  const std::array<float, 6> pre_peace {0, 1, 0.0, 0.0, 1, 1};

  std::array<float, 6> saved_state {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<float, 6> pre_grasp {0, 1, 0, 0, 0, 0}; 

  enum class HandFlag : int {
    LEFT = 0,
    RIGHT = 1,
    BOTH = 2
  };

  bool btn_flag = false;
  bool cont_flag = false;
  bool cont_num = 0;
  bool do_once = true;

  geometry_msgs::msg::Point hand_signal;

  void Control() {

  }

  void StateCallback(const unitree_go::msg::MotorStates::SharedPtr& data) {
    std::array<float, 6> tmp_state = {
      data->states[0].q,
      data->states[1].q,
      data->states[2].q,
      data->states[3].q,
      data->states[4].q,
      data->states[5].q
    };
    saved_state = tmp_state;
  }

  void TouchCallback(const unitree_go::msg::MotorStates::SharedPtr& data) {
    if (data->states[1].q > 60) {
      MoveHands(saved_state, static_cast<int>(HandFlag::LEFT));
      hand_signal.x = 1.0;
      point_pub_->publish(hand_signal);
    }
  }

  void PointCallback(const geometry_msgs::msg::Point::SharedPtr& data) {
    if (data->x == 1.0) {
      StartGraspingLeft();
    }
  }

  void StartGraspingLeft() {
    std::vector<unitree_go::msg::MotorCmd> tmpCommands;
    unitree_go::msg::MotorCmds handCmds;

    for (int i = 0; i < 6; i++) {
        unitree_go::msg::MotorCmd handCmd;
        handCmd.q = fist[i];
        handCmd.dq = 0.2F;
        handCmd.tau = 0.0F;
        handCmd.kp = 0.0F;
        handCmd.kd = 0.0F;
        handCmd.mode = 0;
        tmpCommands.push_back(handCmd);
    }
    handCmds.cmds = tmpCommands;
    l_pub_->publish(handCmds);
  }

  void MoveHands(std::array<float, 6> positions, int hand_flag) {
    if (cont_flag) {
      cont_flag = false;
    }
    std::vector<unitree_go::msg::MotorCmd> tmpCommands;
    unitree_go::msg::MotorCmds handCmds;

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
    handCmds.cmds = tmpCommands;

    switch (hand_flag) {
      case 0:
        l_pub_->publish(handCmds);
        break;
      
      case 1:
        r_pub_->publish(handCmds);
        break;

      case 2:
        l_pub_->publish(handCmds);
        r_pub_->publish(handCmds);
        break;
    }
  }

  void ContinuousMove() {
    while (true) {  
      while (!(cont_flag)) {
        std::this_thread::sleep_for(500ms);
      }
      if (cont_flag && cont_num == 0) {
        std::vector<unitree_go::msg::MotorCmd> tmpCommands;
        unitree_go::msg::MotorCmds handCmds;

        for (int i = 0; i < 6; i++) {
            unitree_go::msg::MotorCmd handCmd;
            handCmd.q = wings[i];
            handCmd.dq = 1.0F;
            handCmd.tau = 0.0F;
            handCmd.kp = 0.0F;
            handCmd.kd = 0.0F;
            handCmd.mode = 0;
            tmpCommands.push_back(handCmd);
        }
        handCmds.cmds = tmpCommands;
        l_pub_->publish(handCmds);
        r_pub_->publish(handCmds);
        std::this_thread::sleep_for(400ms);
      }

      if (cont_flag && cont_num == 0) {
        std::vector<unitree_go::msg::MotorCmd> tmpCommands;
        unitree_go::msg::MotorCmds handCmds;

        for (int i = 0; i < 6; i++) {
            unitree_go::msg::MotorCmd handCmd;
            handCmd.q = open_hand[i];
            handCmd.dq = 1.0F;
            handCmd.tau = 0.0F;
            handCmd.kp = 0.0F;
            handCmd.kd = 0.0F;
            handCmd.mode = 0;
            tmpCommands.push_back(handCmd);
        }
        handCmds.cmds = tmpCommands;
        l_pub_->publish(handCmds);
        r_pub_->publish(handCmds);
        std::this_thread::sleep_for(400ms);
      }

      if (cont_flag && cont_num == 1) {
        unitree_go::msg::MotorCmds handCmds;
        std::vector<float> flutter {1, 0, 0, 0}; 
        while (cont_flag) {
          std::vector<unitree_go::msg::MotorCmd> tmpCommands;
          unitree_go::msg::MotorCmd handCmd;
          handCmd.q = 0;
          handCmd.dq = 1.0F;
          handCmd.tau = 0.0F;
          handCmd.kp = 0.0F;
          handCmd.kd = 0.0F;
          handCmd.mode = 0;
          tmpCommands.push_back(handCmd);
          tmpCommands.push_back(handCmd);
          for (int i = 0; i < 4; i++) {
              unitree_go::msg::MotorCmd handCmd;
              handCmd.q = flutter[i];
              handCmd.dq = 1.0F;
              handCmd.tau = 0.0F;
              handCmd.kp = 0.0F;
              handCmd.kd = 0.0F;
              handCmd.mode = 0;
              tmpCommands.push_back(handCmd);
          }
          handCmds.cmds = tmpCommands;
          l_pub_->publish(handCmds);
          r_pub_->publish(handCmds);
          flutter.insert(flutter.begin(), flutter[3]);
          flutter.pop_back();
          std::this_thread::sleep_for(300ms);
        }
      }
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
      MoveHands(pre_grasp, static_cast<int>(HandFlag::LEFT));
      std::this_thread::sleep_for(300ms);
      MoveHands(open_hand, static_cast<int>(HandFlag::LEFT));
      btn_flag = false;
    } else if ((data->keys == 129) && (btn_flag)) { // f3 + r1
      MoveHands(pre_grasp, static_cast<int>(HandFlag::RIGHT));
      std::this_thread::sleep_for(300ms);
      MoveHands(open_hand, static_cast<int>(HandFlag::RIGHT));
      btn_flag = false;
    }else if ((data->keys == 193) && (btn_flag)) {  // f1 + f3 + r1
      MoveHands(pre_grasp, static_cast<int>(HandFlag::BOTH));
      std::this_thread::sleep_for(300ms);
      MoveHands(open_hand, static_cast<int>(HandFlag::BOTH));
      btn_flag = false;
    }else if ((data->keys == 80) && (btn_flag)) { // f1 + r2
      MoveHands(pre_fist, static_cast<int>(HandFlag::LEFT));
      std::this_thread::sleep_for(500ms);
      MoveHands(fist, static_cast<int>(HandFlag::LEFT));
      btn_flag = false;
    }else if ((data->keys == 144) && (btn_flag)) { // f3 + r2
      MoveHands(pre_fist, static_cast<int>(HandFlag::RIGHT));
      std::this_thread::sleep_for(500ms);
      MoveHands(fist, static_cast<int>(HandFlag::RIGHT));
      btn_flag = false;
    }else if ((data->keys == 208) && (btn_flag)) { // f1 + f3 + r2
      MoveHands(pre_fist, static_cast<int>(HandFlag::BOTH));
      std::this_thread::sleep_for(500ms);
      MoveHands(fist, static_cast<int>(HandFlag::BOTH));
      btn_flag = false;
    }else if ((data->keys == 66) && (btn_flag)) { // f1 + l1
      MoveHands(thumb_out, static_cast<int>(HandFlag::LEFT));
      btn_flag = false;
    }else if ((data->keys == 130) && (btn_flag)) { // f3 + l1
      MoveHands(thumb_out, static_cast<int>(HandFlag::RIGHT));
      btn_flag = false;
    }else if ((data->keys == 194) && (btn_flag)) { // f1 + f3 + l1
      MoveHands(thumb_out, static_cast<int>(HandFlag::BOTH));
      btn_flag = false;
    }else if ((data->keys == 96) && (btn_flag)) { // f1 + l2
      MoveHands(pre_point, static_cast<int>(HandFlag::LEFT));
      std::this_thread::sleep_for(400ms);
      MoveHands(point, static_cast<int>(HandFlag::LEFT));
      btn_flag = false;
    }else if ((data->keys == 160) && (btn_flag)) { // f3 + l2
      MoveHands(pre_point, static_cast<int>(HandFlag::RIGHT));
      std::this_thread::sleep_for(400ms);
      MoveHands(point, static_cast<int>(HandFlag::RIGHT));
      btn_flag = false;
    }else if ((data->keys == 224) && (btn_flag)) { // f1 + f3 + l2
      MoveHands(pre_point, static_cast<int>(HandFlag::BOTH));
      std::this_thread::sleep_for(400ms);
      MoveHands(point, static_cast<int>(HandFlag::BOTH));
      btn_flag = false;
    }else if ((data->keys == 320) && (btn_flag)) { // f1 + A
      if (cont_flag) {
        cont_flag = false;
      } else {
        cont_num = 0;
        cont_flag = true;
      }
      btn_flag = false;
    }else if ((data->keys == 576) && (btn_flag)) { // f1 + B
      if (cont_flag) {
        cont_flag = false;
      } else {
        cont_num = 1;
        cont_flag = true;
      }
      btn_flag = false;
  
    // Method for avoiding held presses. While btn_flag is false, nothing happens.
    }else if ((data->keys == 1088) && (btn_flag)) { // f1 + X
      MoveHands(pre_peace, static_cast<int>(HandFlag::BOTH));
      std::this_thread::sleep_for(400ms);
      MoveHands(peace, static_cast<int>(HandFlag::BOTH));
      btn_flag = false;
    }else if ((data->keys == 8) && (btn_flag)) { // SELECT
      MoveHands(pre_grasp, static_cast<int>(HandFlag::LEFT));
      std::this_thread::sleep_for(500ms);
      StartGraspingLeft();
      btn_flag = false;
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
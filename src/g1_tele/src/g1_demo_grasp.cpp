#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/wireless_controller.hpp"

#include "thread"
#include "chrono"
#include "ros2_stark_interfaces/msg/touch_status.hpp"
#include "ros2_stark_interfaces/msg/motor_status.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

class HandsController : public rclcpp::Node {
 public:
  HandsController() : Node("hands_controller_node") {
    l_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/brainco/cmd_left", 10);
    r_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/brainco/cmd_right", 10);
    point_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/demo/hand", 10);
    suber_ = this->create_subscription<unitree_go::msg::WirelessController>(
      "/wirelesscontroller", 10,
      [this](const unitree_go::msg::WirelessController::SharedPtr data) {
        WirelessCallback(data);
      }); 
    r_touch_sub_ = this->create_subscription<ros2_stark_interfaces::msg::TouchStatus>(
      "/brainco/touch_status_r", 10,
      [this](const ros2_stark_interfaces::msg::TouchStatus::SharedPtr data) {
        TouchCallback(data);
      });
    r_state_sub_ = this->create_subscription<ros2_stark_interfaces::msg::MotorStatus>(
      "/brainco/motor_status_r", 10,
      [this](const ros2_stark_interfaces::msg::MotorStatus::SharedPtr data) {
        StateCallback(data);
      }); 
    point_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/demo/plan", 10,
      [this](const geometry_msgs::msg::Point::SharedPtr data) {
        PointCallback(data);
      }); 

    // thread_ = std::thread([this]() { ContinuousMove(); });
    timer_ = this->create_wall_timer(std::chrono::milliseconds(2), [this] { Control(); });

  }


  private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr l_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr r_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr point_pub_;
  rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr suber_; 
  rclcpp::Subscription<ros2_stark_interfaces::msg::TouchStatus>::SharedPtr r_touch_sub_; 
  rclcpp::Subscription<ros2_stark_interfaces::msg::MotorStatus>::SharedPtr r_state_sub_; 
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point_sub_;
  // std::thread thread_;
  rclcpp::TimerBase::SharedPtr timer_;

  const std::vector<double> open_hand {0, 0, 0, 0, 0, 0}; 
  const std::vector<double> fist {1, 1, 1, 1, 1, 1}; 
  const std::vector<double> pre_fist {0, 1, 1, 1, 1, 1}; 

  const std::vector<double> max_speed {1, 1, 1, 1, 1, 1};
  const std::vector<double> grasp_speed {0.2, 0.2, 0.2, 0.2, 0.2, 0.2};

  std::vector<double> saved_state {0, 0, 0, 0, 0, 0};
  const std::vector<double> pre_grasp {0, 1, 0, 0, 0, 0}; 
  const std::vector<double> grasp {0.4, 1.0, 0.5, 0.5, 0.5, 0.6}; 

  enum class HandFlag : int {
    LEFT = 0,
    RIGHT = 1,
    BOTH = 2
  };

  bool btn_flag = false;
  bool cont_flag = false;
  bool cont_num = 0;
  bool grasp_once = false;

  geometry_msgs::msg::Point hand_signal;

  void Control() {

  }

  void StateCallback(const ros2_stark_interfaces::msg::MotorStatus::SharedPtr& data) {
    for (int i = 0; i < 6; i ++) {
      saved_state[i] = data->positions[i];
    }
  }

  void TouchCallback(const ros2_stark_interfaces::msg::TouchStatus::SharedPtr& data) {
    if ((data->data[0].normal_force1 > 60) && (grasp_once)) {
      RCLCPP_INFO(this->get_logger(), "Index finger force flag triggered");
      MoveHands(saved_state, static_cast<int>(HandFlag::RIGHT));
      hand_signal.x = 1.0;
      point_pub_->publish(hand_signal);
      grasp_once = false;
    }
  }

  void PointCallback(const geometry_msgs::msg::Point::SharedPtr& data) {
    if (data->x == 1.0) {
      MoveHands(pre_grasp, static_cast<int>(HandFlag::RIGHT));
      std::this_thread::sleep_for(300ms);
      StartGrasping(false);
    } else if (data->x == 2.0) {
      MoveHands(pre_grasp, static_cast<int>(HandFlag::RIGHT));
      std::this_thread::sleep_for(300ms);
      MoveHands(open_hand, static_cast<int>(HandFlag::RIGHT));
      std::this_thread::sleep_for(300ms);
      hand_signal.x = 1.0;
      point_pub_->publish(hand_signal);
    }
  }

  void StartGrasping(bool left) {
    grasp_once = true;
    sensor_msgs::msg::JointState hand_cmd;
    hand_cmd.position = grasp;
    hand_cmd.velocity = grasp_speed;
    if (left) {
      l_pub_->publish(hand_cmd);
    } else {
      r_pub_->publish(hand_cmd);
    }
  }

  void MoveHands(const std::vector<double> positions, int hand_flag) {
    if (cont_flag) {
      cont_flag = false;
    }
    sensor_msgs::msg::JointState hand_cmd;
    hand_cmd.position = positions;
    hand_cmd.velocity = max_speed;

    switch (hand_flag) {
      case 0:
        l_pub_->publish(hand_cmd);
        break;
      
      case 1:
        r_pub_->publish(hand_cmd);
        break;

      case 2:
        l_pub_->publish(hand_cmd);
        r_pub_->publish(hand_cmd);
        break;
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
    }else if ((data->keys == 8) && (btn_flag)) { // SELECT
      MoveHands(pre_grasp, static_cast<int>(HandFlag::RIGHT));
      std::this_thread::sleep_for(500ms);
      StartGrasping(false);
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
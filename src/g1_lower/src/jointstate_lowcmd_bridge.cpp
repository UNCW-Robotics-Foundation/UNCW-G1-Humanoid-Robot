#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/imu_state.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

const int G1_NUM_MOTOR = 45;
std::vector<std::string> g1JointNames = {
  "left_hip_pitch_joint",
  "left_hip_roll_joint",
  "left_hip_yaw_joint",
  "left_knee_joint",
  "left_ankle_pitch_joint",
  "left_ankle_roll_joint",
  "right_hip_pitch_joint",
  "right_hip_roll_joint",
  "right_hip_yaw_joint",
  "right_knee_joint",
  "right_ankle_pitch_joint",
  "right_ankle_roll_joint",
  "waist_yaw_joint",
  "waist_roll_joint", 
  "waist_pitch_joint", 
  "left_shoulder_pitch_joint",
  "left_shoulder_roll_joint",
  "left_shoulder_yaw_joint",
  "left_elbow_joint",
  "left_wrist_roll_joint",
  "left_wrist_pitch_joint", 
  "left_wrist_yaw_joint",
  "right_shoulder_pitch_joint",
  "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint",
  "right_elbow_joint",
  "right_wrist_roll_joint",
  "right_wrist_pitch_joint",
  "right_wrist_yaw_joint"
};

class LowStateSuber : public rclcpp::Node {
    public:
        LowStateSuber() : Node("low_state_suber") {
            // suber is set to subscribe "/lowcmd" or  "lf/lowstate" (low frequencies)
            // topic
            const auto *topic_name = "joint_states";
            // The suber  callback function is bind to low_state_suber::topic_callback
            js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                topic_name, 10,
                [this](const sensor_msgs::msg::JointState::SharedPtr data) {
                JointStateHandler(data);
                });
            
            lf_js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("lf/joint_states", 10);
        }

    private:
        void helperFunc(std::vector<_Float64> vec) {
            for (int i = 0; i < 29; i++) {
                g1JointPosOld[i] = vec[i];
            }
        }
        void JointStateHandler(sensor_msgs::msg::JointState::SharedPtr message) {
            std::vector<_Float64> g1JointPos;
            sensor_msgs::msg::JointState js;
            // get motor state
            for (int i = 0; i < G1_NUM_MOTOR; i++) {
                if (first_flag_) {
                    if ((i < 22) || (i > 37)) {
                        if (g1JointPosOld[i] - message->position[i] > 0.2) {
                            RCLCPP_INFO(this->get_logger(), "Detected change greater than 0.2 in a joint. Terminating node...");
                            rclcpp::shutdown();
                            break;
                        }
                        g1JointPos.push_back(message->position[i]);
                    }
                } else {
                    if ((i < 22) || (i > 37)) {
                        g1JointPos.push_back(message->position[i]);
                        first_flag_ = true;
                    }
                }
            }
            helperFunc(g1JointPos);

            js.header.stamp = this->get_clock()->now();
            js.name = g1JointNames;
            js.position = g1JointPos;

            lf_js_pub_->publish(js);
        }
    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr lf_js_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
    bool first_flag_ = false;
    std::array<float, 29> g1JointPosOld;
    float test;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);  // Initialize rclcpp
  rclcpp::spin(
      std::make_shared<LowStateSuber>());  // Run ROS2 node which is make
                                           // share with low_state_suber class
  rclcpp::shutdown();
  return 0;
}
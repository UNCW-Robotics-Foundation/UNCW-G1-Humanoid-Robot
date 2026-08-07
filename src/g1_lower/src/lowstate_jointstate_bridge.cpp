#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/imu_state.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "unitree_go/msg/sport_mode_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

const int G1_NUM_MOTOR = 29;
const int BRAINCO_NUM_MOTOR = 12;
const int BRAINCO_NUM_STILL = 20;
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
  "right_wrist_yaw_joint",
  "left_thumb_metacarpal_joint", //Brainco hand joints begin. First 12 have motors
  "left_thumb_proximal_joint",
  "left_index_proximal_joint",
  "left_middle_proximal_joint",
  "left_ring_proximal_joint",
  "left_pinky_proximal_joint",
  "right_thumb_metacarpal_joint",
  "right_thumb_proximal_joint",
  "right_index_proximal_joint",
  "right_middle_proximal_joint",
  "right_ring_proximal_joint",
  "right_pinky_proximal_joint",
  "left_thumb_distal_joint",    // Brainco motorless joints begin
  "left_thumb_tip_joint",
  "left_index_distal_joint",
  "left_index_tip_joint",
  "left_middle_distal_joint",
  "left_middle_tip_joint",
  "left_ring_distal_joint",
  "left_ring_tip_joint",
  "left_pinky_distal_joint",
  "left_pinky_tip_joint",
  "right_thumb_distal_joint",
  "right_thumb_tip_joint",
  "right_index_distal_joint",
  "right_index_tip_joint",
  "right_middle_distal_joint",
  "right_middle_tip_joint",
  "right_ring_distal_joint",
  "right_ring_tip_joint",
  "right_pinky_distal_joint",
  "right_pinky_tip_joint"
};

class LowStateSuber : public rclcpp::Node {
    public:
        LowStateSuber() : Node("low_state_suber") {
            // suber is set to subscribe "/lowcmd" or  "lf/lowstate" (low frequencies)
            // topic
            const auto *topic_name = "lowstate";
            // The suber  callback function is bind to low_state_suber::topic_callback
            suber_ = this->create_subscription<unitree_hg::msg::LowState>(
                topic_name, 10,
                [this](const unitree_hg::msg::LowState::SharedPtr data) {
                LowStateHandler(data);
                });

            imu_suber_ = this->create_subscription<unitree_go::msg::SportModeState>(
                "/lf/odommodestate", 10,
                [this](const unitree_go::msg::SportModeState::SharedPtr data) {
                ImuStateHandler(data);
                });
            
            js_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

    private:
        void LowStateHandler(unitree_hg::msg::LowState::SharedPtr message) {
            std::vector<_Float64> g1JointPos;
            sensor_msgs::msg::JointState js;
            
            // get motor state
            for (int i = 0; i < G1_NUM_MOTOR; ++i) {
                g1JointPos.push_back(message->motor_state[i].q);
            }
            // temporary placeholder for fingers. Need to learn how to read
            // finger data from robot and place here
            for (int i = 0; i < BRAINCO_NUM_MOTOR; ++i) {
                g1JointPos.push_back(0.0);
            }
            // Still need pos for parts of fingers without motors
            for (int i = 0; i < BRAINCO_NUM_STILL; ++i) {
                g1JointPos.push_back(0.0);
            }

            js.header.stamp = this->get_clock()->now();
            js.name = g1JointNames;
            js.position = g1JointPos;

            js_pub->publish(js);
        }

        void ImuStateHandler(unitree_go::msg::SportModeState::SharedPtr msg) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "world";
            t.child_frame_id = "pelvis";

            t.transform.translation.x = msg->position[0];
            t.transform.translation.y = msg->position[1];
            t.transform.translation.z = msg->position[2];

            tf2::Quaternion q;
            q.setRPY(msg->imu_state.rpy[0], msg->imu_state.rpy[1], msg->imu_state.rpy[2]);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();

            // t.transform.rotation.x = msg->imu_state.quaternion[0];
            // t.transform.rotation.y = msg->imu_state.quaternion[1];
            // t.transform.rotation.z = msg->imu_state.quaternion[2];
            // t.transform.rotation.w = msg->imu_state.quaternion[3];

            tf_broadcaster_->sendTransform(t);
            
        }
    
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr suber_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr imu_suber_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string robot_name = "pelvis";
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);  // Initialize rclcpp
  rclcpp::spin(
      std::make_shared<LowStateSuber>());  // Run ROS2 node which is make
                                           // share with low_state_suber class
  rclcpp::shutdown();
  return 0;
}

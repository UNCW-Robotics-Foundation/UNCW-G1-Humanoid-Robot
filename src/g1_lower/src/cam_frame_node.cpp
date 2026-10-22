#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "chrono"

using namespace std::chrono_literals;

class CamFrameNode : public rclcpp::Node {
 public:
  CamFrameNode() : Node("cam_frame_node") {
    rcl_interfaces::msg::FloatingPointRange float_range_t;
    float_range_t.from_value = -0.04;
    float_range_t.to_value = 0.04;
    float_range_t.step = 0.00001;

    rcl_interfaces::msg::FloatingPointRange float_range_r;
    float_range_r.from_value = -0.3;
    float_range_r.to_value = 0.3;
    float_range_r.step = 0.000001;

    rcl_interfaces::msg::ParameterDescriptor desc_t;
    rcl_interfaces::msg::ParameterDescriptor desc_r;
    desc_t.floating_point_range.push_back(float_range_t);
    desc_r.floating_point_range.push_back(float_range_r);

    cam_x = this->declare_parameter("cam_x", -0.03254, desc_t);
    cam_y = this->declare_parameter("cam_y", 0.00949, desc_t);
    cam_z = this->declare_parameter("cam_z", 0.00065, desc_t);
    cam_roll = this->declare_parameter("cam_roll", 0.004529, desc_r);
    cam_pitch = this->declare_parameter("cam_pitch", -0.119438, desc_r);
    cam_yaw = this->declare_parameter("cam_yaw", -0.003737, desc_r);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer1_ = this->create_wall_timer(std::chrono::milliseconds(2), [this] { Control(); });
    param_callback_ = this->add_on_set_parameters_callback(
      std::bind(&CamFrameNode::on_parameters_change, this, std::placeholders::_1)
    );
  }


  private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_;

  // float cam_x = this->get_parameter("cam_x").as_double();
  // float cam_y = this->get_parameter("cam_y").as_double();
  // float cam_z = this->get_parameter("cam_z").as_double();
  // float cam_roll = this->get_parameter("cam_roll").as_double();
  // float cam_pitch = this->get_parameter("cam_pitch").as_double();
  // float cam_yaw = this->get_parameter("cam_yaw").as_double();

  float cam_x;
  float cam_y;
  float cam_z;
  float cam_roll;
  float cam_pitch;
  float cam_yaw;

  rcl_interfaces::msg::SetParametersResult on_parameters_change(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Finger update applied successfully";
    RCLCPP_INFO(this->get_logger(), "Received Parameter Request");

    for (const auto &param : parameters) {
      const std::string &name = param.get_name();

      if (name == "cam_x") {
        cam_x = param.as_double();
      } else if (name == "cam_y") {
        cam_y = param.as_double();
      } else if (name == "cam_z") {
        cam_z = param.as_double();
      } else if (name == "cam_roll") {
        cam_roll = param.as_double();
      } else if (name == "cam_pitch") {
        cam_pitch = param.as_double();
      } else if (name == "cam_yaw") {
        cam_yaw = param.as_double();
      }
    }

    return result;
  }

  void Control() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "d435_link";
    t.child_frame_id = "camera_link";

    t.transform.translation.x = cam_x;
    t.transform.translation.y = cam_y;
    t.transform.translation.z = cam_z;

    tf2::Quaternion q;
    q.setRPY(cam_roll, cam_pitch, cam_yaw);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
  }

};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);  // Initialize rclcpp
  // Run ROS2 node which is make share with wireless_controller_suber class
  rclcpp::spin(std::make_shared<CamFrameNode>());
  rclcpp::shutdown();
  return 0;
}
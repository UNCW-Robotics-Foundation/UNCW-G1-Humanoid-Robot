#include "rclcpp/rclcpp.hpp"
#include "chrono"
#include "geometry_msgs/msg/point.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include <pcl_ros/point_cloud.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std::chrono_literals;

class crop_3D_image : public rclcpp::Node {
 public:
  crop_3D_image() : Node("crop_3D_image") {
    // l_pub_ = this->create_publisher<unitree_go::msg::MotorCmds>("/brainco/left/cmd", 10);
    // suber_ = this->create_subscription<unitree_go::msg::WirelessController>(
    //   "/wirelesscontroller", 10,
    //   [this](const unitree_go::msg::WirelessController::SharedPtr data) {
    //     WirelessCallback(data);
    //   }); 

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this] { Control(); });

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, 
      std::bind(&crop_3D_image::image_callback, this, std::placeholders::_1));

  }


  private:
  // rclcpp::Publisher<unitree_go::msg::MotorCmds>::SharedPtr l_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Point hand_signal;

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        // Convert ROS image to cv::Mat format securely
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::imshow("C++ Camera Viewer", cv_ptr->image);
    cv::waitKey(1);
  }

  // void StateCallback(const unitree_go::msg::MotorStates::SharedPtr& data) {
    
  // }

  void Control() {

  }

};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);  // Initialize rclcpp
  // Run ROS2 node which is make share with wireless_controller_suber class
  rclcpp::spin(std::make_shared<crop_3D_image>());
  rclcpp::shutdown();
  return 0;
}
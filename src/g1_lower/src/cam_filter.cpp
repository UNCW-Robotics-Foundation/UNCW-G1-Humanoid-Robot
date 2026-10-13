#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/common.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include <pcl_ros/transforms.hpp>
#include <pcl/filters/voxel_grid.h>

class CamFilterNode : public rclcpp::Node {
 public:
  CamFilterNode() : Node("cam_filter_node") {
    rcl_interfaces::msg::FloatingPointRange float_range_voxel;
    float_range_voxel.from_value = 0.0;
    float_range_voxel.to_value = 0.1;
    float_range_voxel.step = 0.0001;

    rcl_interfaces::msg::FloatingPointRange float_range_box;
    float_range_box.from_value = -5.0;
    float_range_box.to_value = 5.0;
    float_range_box.step = 0.01;

    rcl_interfaces::msg::FloatingPointRange float_range_dev;
    float_range_dev.from_value = 0.0;
    float_range_dev.to_value = 1.0;
    float_range_dev.step = 0.001;

    rcl_interfaces::msg::IntegerRange int_range_mean;
    int_range_mean.from_value = 0;
    int_range_mean.to_value = 100;
    int_range_mean.step = 1;

    rcl_interfaces::msg::ParameterDescriptor desc_voxel;
    rcl_interfaces::msg::ParameterDescriptor desc_box;
    rcl_interfaces::msg::ParameterDescriptor desc_dev;
    rcl_interfaces::msg::ParameterDescriptor desc_mean;

    desc_voxel.floating_point_range.push_back(float_range_voxel);
    desc_box.floating_point_range.push_back(float_range_box);
    desc_dev.floating_point_range.push_back(float_range_dev);
    desc_mean.integer_range.push_back(int_range_mean);

    voxel_leaf_size = this->declare_parameter("voxel_leaf_size", 0.05, desc_voxel);

    x_box_min = this->declare_parameter("x_box_min", -1.0, desc_box);
    x_box_max = this->declare_parameter("x_box_max", 1.0, desc_box);
    y_box_min = this->declare_parameter("y_box_min", -1.0, desc_box);
    y_box_max = this->declare_parameter("y_box_max", 1.0, desc_box);
    z_box_min = this->declare_parameter("z_box_min", -1.0, desc_box);
    z_box_max = this->declare_parameter("z_box_max", 1.0, desc_box);

    sor_mean = this->declare_parameter("sor_mean", 50, desc_mean);
    sor_std_dev = this->declare_parameter("sor_std_dev", 0.5, desc_dev);
    use_sor = this->declare_parameter("use_sor", true);
    use_voxel = this->declare_parameter("use_voxel", true);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cloud_subscriber_ =
          this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points", 1, std::bind(&CamFilterNode::cloud_callback, this, std::placeholders::_1));

    crop_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("crop_cloud", 1);
    voxel_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_cloud", 1);

    param_callback_ = this->add_on_set_parameters_callback(
      std::bind(&CamFilterNode::on_parameters_change, this, std::placeholders::_1)
    );
  }


  private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr crop_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_grid_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;

  float voxel_leaf_size;
  float x_box_min;
  float x_box_max;
  float y_box_min;
  float y_box_max;
  float z_box_min;
  float z_box_max;
  float sor_std_dev;
  int sor_mean;
  bool use_sor;
  bool use_voxel;

  rcl_interfaces::msg::SetParametersResult on_parameters_change(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Filter update applied successfully";
    RCLCPP_INFO(this->get_logger(), "Received Parameter Request");

    for (const auto &param : parameters) {
      const std::string &name = param.get_name();

      if (name == "voxel_leaf_size") {
        voxel_leaf_size = param.as_double();
      } else if (name == "x_box_min") {
        x_box_min = param.as_double();
      } else if (name == "x_box_max") {
        x_box_max = param.as_double();
      } else if (name == "y_box_min") {
        y_box_min = param.as_double();
      } else if (name == "y_box_max") {
        y_box_max = param.as_double();
      } else if (name == "z_box_min") {
        z_box_min = param.as_double();
      } else if (name == "z_box_max") {
        z_box_max = param.as_double();
      } else if (name == "sor_std_dev") {
        sor_std_dev = param.as_double();
      } else if (name == "sor_mean") {
        sor_mean = param.as_int();
      } else if (name == "use_sor") {
        use_sor = param.as_bool();
      } else if (name == "use_voxel") {
        use_voxel = param.as_bool();
      }
    }

    return result;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr recent_cloud) {
    geometry_msgs::msg::TransformStamped stransform;
    try {
      stransform = tf_buffer_->lookupTransform("pelvis", recent_cloud->header.frame_id,
          tf2::TimePointZero, tf2::durationFromSec(3));
    }
    catch(const tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      return;
    }
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    pcl_ros::transformPointCloud("pelvis", stransform, *recent_cloud, transformed_cloud);

    // Voxel Grid Filter
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(transformed_cloud, cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(cloud));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cropbox_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sor_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());

    //this->publishPointCloud(voxel_grid_pub_, *cloud_voxel_filtered);

    // Cropbox filter
    //pcl::PointCloud<pcl::PointXYZRGB> xyz_filtered_cloud;
    pcl::CropBox<pcl::PointXYZRGB> crop;
    //crop.setInputCloud(cloud_voxel_filtered);
    crop.setInputCloud(cloud_ptr);
    Eigen::Vector4f min_point = Eigen::Vector4f(x_box_min, y_box_min, z_box_min, 0);
    Eigen::Vector4f max_point = Eigen::Vector4f(x_box_max, y_box_max, z_box_max, 0);
    crop.setMin(min_point);
    crop.setMax(max_point);
    crop.filter(*cloud_cropbox_filtered);

    if ((use_sor) && (use_voxel)) {
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
      voxel_filter.setInputCloud(cloud_cropbox_filtered);
      voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
      voxel_filter.filter(*cloud_voxel_filtered);

      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
      sor.setInputCloud(cloud_voxel_filtered);
      sor.setMeanK(sor_mean);
      sor.setStddevMulThresh(sor_std_dev);
      sor.filter(*cloud_sor_filtered);

      this->publishPointCloud(crop_pub_, *cloud_sor_filtered);
    } else {
      if (use_voxel) {
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
        voxel_filter.setInputCloud(cloud_cropbox_filtered);
        voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
        voxel_filter.filter(*cloud_voxel_filtered);

        this->publishPointCloud(crop_pub_, *cloud_voxel_filtered);

      } else if (use_sor) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud_cropbox_filtered);
        sor.setMeanK(sor_mean);
        sor.setStddevMulThresh(sor_std_dev);
        sor.filter(*cloud_sor_filtered);

        this->publishPointCloud(crop_pub_, *cloud_sor_filtered);

      } else {
        this->publishPointCloud(crop_pub_, *cloud_cropbox_filtered);
      }
    }
  }

  void publishPointCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher, pcl::PointCloud<pcl::PointXYZRGB> point_cloud) {
    sensor_msgs::msg::PointCloud2::SharedPtr pc2_cloud(new sensor_msgs::msg::PointCloud2);

    pcl::toROSMsg(point_cloud, *pc2_cloud);
    pc2_cloud->header.frame_id = "pelvis";
    pc2_cloud->header.stamp    = this->get_clock()->now();
    publisher->publish(*pc2_cloud);
  }

};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);  // Initialize rclcpp
  // Run ROS2 node which is make share with wireless_controller_suber class
  rclcpp::spin(std::make_shared<CamFilterNode>());
  rclcpp::shutdown();
  return 0;
}
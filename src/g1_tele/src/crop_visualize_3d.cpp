#include "rclcpp/rclcpp.hpp"
#include "chrono"
#include "cmath"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include <g1_msgs/msg/sphere_params.hpp>
#include <g1_msgs/msg/xy_zarray.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std::chrono_literals;
using namespace cv;
using namespace std;
using namespace cv_bridge;

class crop_3D_image : public rclcpp::Node {
 public:
  crop_3D_image() : Node("crop_3D_image") {
    // l_pub_ = this->create_publisher<unitree_go::msg::MotorCmds>("/brainco/left/cmd", 10);
    // suber_ = this->create_subscription<unitree_go::msg::WirelessController>(
    //   "/wirelesscontroller", 10,
    //   [this](const unitree_go::msg::WirelessController::SharedPtr data) {
    //     WirelessCallback(data);
    //   }); 

    cam_sub = this->create_subscription<sensor_msgs::msg::Image>(
      "/ball_2d", 10,
      [this](const sensor_msgs::msg::Image::SharedPtr data) {
        get_img(data);
      }); 
    
    pcl_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/camera/depth/color/points", 10,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr data) {
        get_pcl(data);
      }); 
    
    sphere_sub = this->create_subscription<g1_msgs::msg::SphereParams>(
      "/sphere_params", 10,
      [this](const g1_msgs::msg::SphereParams::SharedPtr data) {
        get_sphere_params(data);
      }); 

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this] { Control(); });

    // subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    //   "/ball_2D", 10, 
    //   std::bind(&crop_3D_image::get_img, this, std::placeholders::_1));

  }


  private:
  // rclcpp::Publisher<unitree_go::msg::MotorCmds>::SharedPtr l_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub;
  rclcpp::Subscription<g1_msgs::msg::SphereParams>::SharedPtr sphere_sub;
  rclcpp::TimerBase::SharedPtr timer_;

  Mat img;
  CvImagePtr cv_ptr;

  pcl::PointCloud<pcl::PointXYZ> raw_pcl;
  pcl::PointCloud<pcl::PointXYZ> cropped_pcl;	
  pcl::PointCloud<pcl::PointXYZ> ball_fit_pcl;
  pcl::PointCloud<pcl::PointXYZ> ball_center_pcl;	

  g1_msgs::msg::SphereParams sphere_params;
  g1_msgs::msg::XYZarray xyz;

  bool image_received = false;
  bool pcl_received = false;

  int width, height;

  void get_img(const sensor_msgs::msg::Image::SharedPtr&  _data){
    // read the rectified rgb input image from the camera
    cv_ptr = toCvCopy(_data,"mono8");
    // take the image part and put it in a mat variable in opencv format
    img = cv_ptr->image;
    // get the width and height
    width = _data->width;
    height = _data->height;
    
    image_received = true;
  }

  void get_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr& _cloud){
    // convert the ROS message to PCL
    pcl::fromROSMsg(*_cloud, raw_pcl);
    // raise the flag
    pcl_received = true;
  }

  void get_sphere_params(const g1_msgs::msg::SphereParams::SharedPtr& _params){
    //read the parameters and save them
    sphere_params = *_params;
    std::cout << sphere_params.xc << " " << sphere_params.yc << " " << sphere_params.zc << " "<< sphere_params.radius << std::endl;
  }

  pcl::PointCloud<pcl::PointXYZ> crop_pcl(pcl::PointCloud<pcl::PointXYZ>& _raw_pcl, cv::Mat& filtered_2D_img){
    // define an empty point cloud
    pcl::PointCloud<pcl::PointXYZ> tmp_pcl;
    // a 3D point for filtering the ball image
    pcl::PointXYZ point_buffer_usedInAtMethod; 
    // loop through the rows and columns
    for (int row = 0; row < filtered_2D_img.rows; row++){
      for (int col = 0; col < filtered_2D_img.cols; col++) {
        // if there is a pixel on the ball (from the ball_2D topic add the corresponding point to the point cloud)
        if (filtered_2D_img.at<uchar>(row,col) == 255){
          point_buffer_usedInAtMethod = _raw_pcl.at(col , row);
          tmp_pcl.push_back(point_buffer_usedInAtMethod);
        }
      
      }
    }

    return tmp_pcl;
  }

  g1_msgs::msg::XYZarray convert_pcl(pcl::PointCloud<pcl::PointXYZ>& _pcl){
    g1_msgs::msg::XYZarray xyz;
    // define a pointer to loop throught the pcl data
    pcl::PointCloud<pcl::PointXYZ>::iterator pi = _pcl.begin();

        xyz.points.clear();
        geometry_msgs::msg::Point point; 
        // convert the pcl data to point and add them to the array
    for( ; pi != _pcl.end(); pi++ ) {
    
      if( 0 < fabs( pi->x ) && 0 < fabs( pi->y ) && 0 < fabs( pi->z ) ){
        point.x = pi->x;
        point.y = pi->y;
        point.z = pi->z;
        
        xyz.points.push_back(point); 
      }
    }
    
    return xyz;
  }

  pcl::PointCloud<pcl::PointXYZ> visualize_sphere(const g1_msgs::msg::SphereParams& _params){
    // define an empty point cloud
    pcl::PointCloud<pcl::PointXYZ> tmp_pcl;
    pcl::PointXYZ tmp_point;
    // using the spherical coordinates to produce xyz using the coordinates of the center and the radius
    // example of math: https://tutorial.math.lamar.edu/classes/calciii/SphericalCoords.aspx
    // break the theta and phi values to 20 chunks between 0 and 2*Pi
    int theta_segments = 20;
    int phi_segments = 20;
    for (int theta_ind = 0; theta_ind < theta_segments; theta_ind++){
      for (int phi_ind = 0; phi_ind < phi_segments; phi_ind++){
        // calculate current theta and phi on the grid
        float theta = (float) theta_ind/theta_segments*2*M_PI;
        float phi = (float) phi_ind/phi_segments*2*M_PI;
        // use the param values and update the points
        tmp_point.x = _params.xc + _params.radius*sin(phi)*cos(theta);
        tmp_point.y = _params.yc + _params.radius*sin(phi)*sin(theta);
        tmp_point.z = _params.zc + _params.radius*cos(phi); 
        tmp_pcl.push_back(tmp_point);
      }
      
    }
    return tmp_pcl;

  }

  pcl::PointCloud<pcl::PointXYZ> visualize_center(const g1_msgs::msg::SphereParams& _params){
    // define an empty point cloud
    pcl::PointCloud<pcl::PointXYZ> tmp_pcl;
    pcl::PointXYZ tmp_point;
    // use the param values and update the points
    tmp_point.x = _params.xc;
    tmp_point.y = _params.yc;
    tmp_point.z = _params.zc; 
    tmp_pcl.push_back(tmp_point);

    return tmp_pcl;

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
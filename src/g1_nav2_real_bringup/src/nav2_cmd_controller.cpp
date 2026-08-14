#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "g1/g1_loco_client.hpp"
#include "unitree_api/msg/request.hpp"
#include "nlohmann/json.hpp"

class Nav2VelController : public rclcpp::Node {
    public:
        Nav2VelController() : Node("nav2_vel_controller") {
            cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10,
                [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                cmdVelCallback(msg);
                });

            api_pub_ = this->create_publisher<unitree_api::msg::Request>("api/sport/request", 10);
        }

    private:
        void cmdVelCallback(geometry_msgs::msg::Twist::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Navigating to Goal...");
            unitree_api::msg::Request req;
            req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_VELOCITY;
            nlohmann::json js;
            std::vector<float> velocity = {(float)msg->linear.x, (float)msg->linear.y, (float)msg->angular.z};
            js["velocity"] = velocity;
            // Dead-man's switch: Nav2 republishes cmd_vel at 20 Hz, so a
            // short duration just means the robot stops ~1 s after Nav2
            // goes quiet (instead of walking for 10 days).
            js["duration"] = 1.0;
            req.parameter = js.dump();
            api_pub_->publish(req);
        }

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr api_pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nav2VelController>());
    rclcpp::shutdown();
    return 0;
}

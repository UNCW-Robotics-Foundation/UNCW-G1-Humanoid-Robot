#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/imu_state.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "g1/g1_loco_client.hpp"
#include "unitree_api/msg/request.hpp"
#include "nlohmann/json.hpp"
#include "sensor_msgs/msg/joy.hpp"

class LocoClientNode : public rclcpp::Node {
    public:
        LocoClientNode() : Node("loco_client_node") {
            suber_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10,
                [this](const geometry_msgs::msg::Twist::SharedPtr data) {
                LowStateHandler(data);
                });
            joy_suber_ = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy", 10,
                [this](const sensor_msgs::msg::Joy::SharedPtr data) {
                JoyHandler(data);
                });
            
            api_pub = this->create_publisher<unitree_api::msg::Request>("api/sport/request", 10);
        }

    private:
        std::thread thread_;
        void LowStateHandler(geometry_msgs::msg::Twist::SharedPtr message) {
            if (message->linear.x > 0) {
                RCLCPP_INFO(this->get_logger(), "KEYBOARD HANDLER; Moving forwards...");
                unitree_api::msg::Request req;
                req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_VELOCITY;
                nlohmann::json js;
                std::vector<float> velocity = {0.2, 0.0, 0.0};
                js["velocity"] = velocity;
                js["duration"] = 5.0;
                req.parameter = js.dump();
                api_pub->publish(req);

            } else if (message->linear.x < 0) {
                RCLCPP_INFO(this->get_logger(), "KEYBOARD HANDLER; Moving backwards...");
                unitree_api::msg::Request req;
                req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_VELOCITY;
                nlohmann::json js;
                std::vector<float> velocity = {-0.2, 0.0, 0.0};
                js["velocity"] = velocity;
                js["duration"] = 5.0;
                req.parameter = js.dump();
                api_pub->publish(req);

            } else if (message->angular.z > 0) {
                RCLCPP_INFO(this->get_logger(), "KEYBOARD HANDLER; Moving right...");
                unitree_api::msg::Request req;
                req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_VELOCITY;
                nlohmann::json js;
                std::vector<float> velocity = {0.0, 0.0, 0.3};
                js["velocity"] = velocity;
                js["duration"] = 4.0;
                req.parameter = js.dump();
                api_pub->publish(req);

            } else if (message->angular.z < 0) {
                RCLCPP_INFO(this->get_logger(), "KEYBOARD HANDLER; Moving left...");
                unitree_api::msg::Request req;
                req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_VELOCITY;
                nlohmann::json js;
                std::vector<float> velocity = {0.0, 0.0, -0.3};
                js["velocity"] = velocity;
                js["duration"] = 4.0;
                req.parameter = js.dump();
                api_pub->publish(req);

            }else {
                RCLCPP_INFO(this->get_logger(), "KEYBOARD HANDLER; Stopping...");
                unitree_api::msg::Request req;
                req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_VELOCITY;
                nlohmann::json js;
                std::vector<float> velocity = {0.0, 0.0, 0.0};
                js["velocity"] = velocity;
                js["duration"] = 0.0;
                req.parameter = js.dump();
                api_pub->publish(req);
            }
            //RCLCPP_INFO(this->get_logger(), "KEYBOARD HANDLER; Listening...");
        }

        void JoyHandler(sensor_msgs::msg::Joy::SharedPtr message) {

            if (btn_flag) {
                int counter = 0;
                for (int i = 0; i < message->buttons.size(); i++) {
                    if (message->buttons[i] == 1) {
                        break;
                    }
                    counter++;
                }
                if (counter == message->buttons.size()) {
                    RCLCPP_INFO(this->get_logger(), "CONTROLLER HANDLER; Button released...");
                    btn_flag = false;
                }
            } 
            else if (message->buttons[0] == 1) {
                RCLCPP_INFO(this->get_logger(), "CONTROLLER HANDLER; Button A pressed. Entering zero torque mode...");
                btn_flag = true;
                unitree_api::msg::Request req;
                req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_FSM_ID;
                nlohmann::json js;
                js["data"] = 0;
                req.parameter = js.dump();
                api_pub->publish(req);
            }
            else if (message->buttons[1] == 1) {
                RCLCPP_INFO(this->get_logger(), "CONTROLLER HANDLER; Button B pressed. Entering damp mode...");
                btn_flag = true;
                unitree_api::msg::Request req;
                req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_FSM_ID;
                nlohmann::json js;
                js["data"] = 1;
                req.parameter = js.dump();
                api_pub->publish(req);
            }
            else if (message->buttons[2] == 1) {
                RCLCPP_INFO(this->get_logger(), "CONTROLLER HANDLER; Button X pressed. Entering standing mode...");
                btn_flag = true;
                unitree_api::msg::Request req;
                req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_FSM_ID;
                nlohmann::json js;
                js["data"] = 4;
                req.parameter = js.dump();
                api_pub->publish(req);
            }
            else if (message->buttons[3] == 1) {
                RCLCPP_INFO(this->get_logger(), "CONTROLLER HANDLER; Button Y pressed. Entering start mode...");
                btn_flag = true;
                unitree_api::msg::Request req;
                req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_FSM_ID;
                nlohmann::json js;
                js["data"] = 801;
                req.parameter = js.dump();
                api_pub->publish(req);
            }
            else {
                //RCLCPP_INFO(this->get_logger(), "CONTROLLER HANDLER; Listening...");
            }
        }
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr suber_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_suber_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr api_pub;
    bool btn_flag = false;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);  // Initialize rclcpp
  rclcpp::spin(
      std::make_shared<LocoClientNode>());  // Run ROS2 node which is make
                                           // share with low_state_suber class
  rclcpp::shutdown();
  return 0;
}
#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_state.hpp"

#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>

using namespace std::chrono_literals;

class BagToTxtNode : public rclcpp::Node {
    public:
        BagToTxtNode() : Node("bag_to_txt_node") {
            suber_ = this->create_subscription<unitree_hg::msg::LowState>(
                "bag/lowstate", 10,
                [this](const unitree_hg::msg::LowState::SharedPtr data) {
                LowStateHandler(data);
                });

            thread_ = std::thread([this]() { ControlLoop(); });
        }

    private:
        std::thread thread_;
        bool bag_received = false;
        int count = 0;
        std::vector<_Float64> g1JointPos;

        rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr suber_;
        rclcpp::TimerBase::SharedPtr timer1_;

        void LowStateHandler(unitree_hg::msg::LowState::SharedPtr message) {
            for (int i = 0; i < 29; i++) {
                g1JointPos.push_back(message->motor_state[i].q);
            }
            bag_received = true;
            count = 0;
        }

        void ControlLoop() {
            while (!bag_received) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Waiting for Bag/LowState...");
            std::this_thread::sleep_for(100ms);
            }
            RCLCPP_INFO(this->get_logger(), "Bag/LowState received. Storing data...");
            timer1_ = this->create_wall_timer(std::chrono::milliseconds(2),
                                      [this] { FileWriter(); });
        }

        void FileWriter() {
            if (count > 2) {
            // StopControl();
                RCLCPP_INFO(this->get_logger(), "Bag file ended. Writing data...");
                std::ofstream BagFile("converted_test.txt");
                for (int i = 0; i < g1JointPos.size(); i++) {
                    BagFile << g1JointPos[i];
                    if (i + 1 < g1JointPos.size()) {
                        BagFile << std::endl;
                    }
                }
                BagFile.close();
                RCLCPP_INFO(this->get_logger(), "Writing complete.");
                rclcpp::shutdown();
                } else {
                count++;
                }
            }

};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);  // Initialize rclcpp
  rclcpp::spin(
      std::make_shared<BagToTxtNode>());  // Run ROS2 node which is make
                                           // share with low_state_suber class
  rclcpp::shutdown();
  return 0;
}
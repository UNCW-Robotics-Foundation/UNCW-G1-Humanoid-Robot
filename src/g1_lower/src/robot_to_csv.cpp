#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_state.hpp"

#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>

using namespace std::chrono_literals;

class RobotToCSV : public rclcpp::Node {
    public:
        RobotToCSV() : Node("robot_to_csv_node") {
            suber_ = this->create_subscription<unitree_hg::msg::LowState>(
                "lowstate", 10,
                [this](const unitree_hg::msg::LowState::SharedPtr data) {
                LowStateHandler(data);
                });

            thread_ = std::thread([this]() { ControlLoop(); });
        }

    private:
        std::thread thread_;
        bool state_received = false;
        bool stop_flag = false;
        std::array<float, 29> g1JointPos{};
        std::vector<std::string> storedJoints;

        rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr suber_;
        rclcpp::TimerBase::SharedPtr timer1_;

        void LowStateHandler(unitree_hg::msg::LowState::SharedPtr message) {
            for (int i = 0; i < 29; i++) {
                g1JointPos[i] = message->motor_state[i].q;
            }
            state_received = true;
        }

        void ControlLoop() {
            while (!state_received) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Waiting for LowState...");
            std::this_thread::sleep_for(100ms);
            }
            RCLCPP_INFO(this->get_logger(), "LowState subscribed.");
            timer1_ = this->create_wall_timer(std::chrono::milliseconds(2),
                                      [this] { FileWriter(); });
        }

        void FileWriter() {
            if (!(stop_flag)) {
                std::string line;
                line += std::to_string(g1JointPos[0]);
                for (int i = 1; i < g1JointPos.size(); i++) {
                    line += "," + std::to_string(g1JointPos[0]);
                }
                line += "\n";
                storedJoints.push_back(line);

            } else {
            std::ofstream CSVFile("csv_test.csv");
            RCLCPP_INFO(this->get_logger(), "Writing to csv...");
            for (int i = 0; i < storedJoints.size(); i++) {
                CSVFile << storedJoints[i];
            }
            CSVFile.close();
            RCLCPP_INFO(this->get_logger(), "Writing complete.");
            rclcpp::shutdown();
            }
            }

};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);  // Initialize rclcpp
  rclcpp::spin(
      std::make_shared<RobotToCSV>());  // Run ROS2 node which is make
                                           // share with low_state_suber class
  rclcpp::shutdown();
  return 0;
}
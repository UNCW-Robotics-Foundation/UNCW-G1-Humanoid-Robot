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
        std::vector<std::array<float, 29>> storedJoints;

        rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr suber_;
        rclcpp::TimerBase::SharedPtr timer1_;

        void LowStateHandler(unitree_hg::msg::LowState::SharedPtr message) {
            for (int i = 0; i < 29; i++) {
                g1JointPos[i] = message->motor_state[i].q;
            }
            state_received = true;
        }

        void ControlLoop() {
            std::string str;
            int counter = 0;
            while (!state_received) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Waiting for LowState...");
            std::this_thread::sleep_for(100ms);
            }
            RCLCPP_INFO(this->get_logger(), "LowState subscribed. Recording starting...");

            std::cout << std::endl;
            std::cout << "Press enter to record. Type anything before hitting enter to stop: ";
            std::getline(std::cin, str);
            while (str == "") {
                storedJoints.push_back(g1JointPos);
                counter++;
                if (counter < 10) {
                    std::cout << counter << "  point recorded. (enter: record; anything+enter: exit): ";
                } else {
                    std::cout << counter << " point recorded. (enter: record; anything+enter: exit): ";
                }
                std::getline(std::cin, str);
            }
            std::cout << std::endl;
            RCLCPP_INFO(this->get_logger(), "Recording stopped");
            if (counter > 0) {
                FileWriter();
            }
            rclcpp::shutdown();
        }

        void FileWriter() {
            std::string tmp_line;
            std::ofstream CSVFile("csv_test.csv");
            RCLCPP_INFO(this->get_logger(), "Writing to csv...");
            CSVFile << "LEFT_HIP_PITCH,LEFT_HIP_ROLL,LEFT_HIP_YAW,LEFT_KNEE,LEFT_ANKLE_PITCH,LEFT_ANKLE_ROLL,RIGHT_HIP_PITCH,RIGHT_HIP_ROLL,RIGHT_HIP_YAW,RIGHT_KNEE,RIGHT_ANKLE_PITCH,RIGHT_ANKLE_ROLL,WAIST_YAW,WAIST_ROLL,WAIST_PITCH,LEFT_SHOULDER_PITCH,LEFT_SHOULDER_ROLL,LEFT_SHOULDER_YAW,LEFT_ELBOW,LEFT_WRIST_ROLL,LEFT_WRIST_PITCH,LEFT_WRIST_YAW,RIGHT_SHOULDER_PITCH,RIGHT_SHOULDER_ROLL,RIGHT_SHOULDER_YAW,RIGHT_ELBOW,RIGHT_WRIST_ROLL,RIGHT_WRIST_PITCH,RIGHT_WRIST_YAW\n";
            for (int i = 0; i < storedJoints.size(); i++) {
                tmp_line = std::to_string(storedJoints[i][0]);
                for (int j = 1; j < 29; j++) {
                    tmp_line += "," + std::to_string(storedJoints[i][j]);
                }
                if (i < storedJoints.size() - 1) {
                   tmp_line += "\n"; 
                }
                CSVFile << tmp_line;
            }
            CSVFile.close();
            RCLCPP_INFO(this->get_logger(), "Writing complete.");
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
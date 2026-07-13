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

            timer1_ = this->create_wall_timer(std::chrono::milliseconds(2),
                                      [this] { ContinuousRec(); });
        }

    private:
        std::thread thread_;
        bool state_received = false;
        bool c_rec_flag = false;
        bool first_c_rec_flag = true;
        bool l_flag = false;
        bool last_l_flag = false;
        std::array<float, 29> g1JointPos{};
        std::vector<std::array<float, 29>> storedJoints;
        std::vector<int> storedFlags;

        rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr suber_;
        rclcpp::TimerBase::SharedPtr timer1_;

        int counter = 0;
        int max = 10;
        std::string spacing = "         ";

        void LowStateHandler(unitree_hg::msg::LowState::SharedPtr message) {
            for (int i = 0; i < 29; i++) {
                g1JointPos[i] = message->motor_state[i].q;
            }
            state_received = true;
        }

        void ControlLoop() {
            std::string str;
            while (!state_received) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Waiting for LowState...");
            std::this_thread::sleep_for(100ms);
            }
            RCLCPP_INFO(this->get_logger(), "LowState subscribed. Recording starting...");

            std::cout << std::endl;
            std::cout << "Empty input for single point, input u for user point, input c for continuous recording, input l for looped recording, input anything else to stop: ";
            std::getline(std::cin, str);
            while ((str == "") || (str == "c") || (str == "u") || (str == "l")) {
                if (str == "") {
                    storedFlags.push_back(0);
                } else if (str == "u") {
                    storedFlags.push_back(2);
                }

                if ((str == "c") || (str == "l")) {
                    c_rec_flag = true;
                    first_c_rec_flag = true;
                    if (str == "l") {
                        l_flag = true;
                        last_l_flag = true;
                        std::cout << "Continuous looped recording started. Press enter to stop...";
                    } else {
                        std::cout << "Continuous recording started. Press enter to stop...";
                    }
                    std::getline(std::cin, str);
                    c_rec_flag = false;
                    l_flag = false;
                } else{
                    storedJoints.push_back(g1JointPos);
                    counter++;
                    if (counter >= max) {
                        max = max * 10;
                        spacing.pop_back();
                }
                }
                std::cout << counter << spacing << "point(s) recorded. (empty: regular point; u: user point; c: continuous recording; l: looped recording; anything else: exit): ";
                std::getline(std::cin, str);

            }
            std::cout << std::endl;
            RCLCPP_INFO(this->get_logger(), "Recording ended");
            if (counter > 0) {
                FileWriter();
            }
            rclcpp::shutdown();
        }

        void FileWriter() {
            std::string tmp_line;
            std::ofstream CSVFile("csv_test.csv");
            RCLCPP_INFO(this->get_logger(), "Writing to csv...");
            CSVFile << "LEFT_HIP_PITCH,LEFT_HIP_ROLL,LEFT_HIP_YAW,LEFT_KNEE,LEFT_ANKLE_PITCH,LEFT_ANKLE_ROLL,RIGHT_HIP_PITCH,RIGHT_HIP_ROLL,RIGHT_HIP_YAW,RIGHT_KNEE,RIGHT_ANKLE_PITCH,RIGHT_ANKLE_ROLL,WAIST_YAW,WAIST_ROLL,WAIST_PITCH,LEFT_SHOULDER_PITCH,LEFT_SHOULDER_ROLL,LEFT_SHOULDER_YAW,LEFT_ELBOW,LEFT_WRIST_ROLL,LEFT_WRIST_PITCH,LEFT_WRIST_YAW,RIGHT_SHOULDER_PITCH,RIGHT_SHOULDER_ROLL,RIGHT_SHOULDER_YAW,RIGHT_ELBOW,RIGHT_WRIST_ROLL,RIGHT_WRIST_PITCH,RIGHT_WRIST_YAW,FLAG\n";
            for (int i = 0; i < storedJoints.size(); i++) {
                tmp_line = std::to_string(storedJoints[i][0]);
                for (int j = 1; j < 29; j++) {
                    tmp_line += "," + std::to_string(storedJoints[i][j]);
                }
                tmp_line += "," + std::to_string(storedFlags[i]);
                if (i < storedJoints.size() - 1) {
                   tmp_line += "\n"; 
                }
                CSVFile << tmp_line;
            }
            CSVFile.close();
            RCLCPP_INFO(this->get_logger(), "Writing complete.");
        }

        void ContinuousRec() {
            if (c_rec_flag) {
                storedJoints.push_back(g1JointPos);
                if (l_flag) {
                    storedFlags.push_back(3);
                } else if (!(first_c_rec_flag)) {
                    storedFlags.push_back(1);
                } else{
                    storedFlags.push_back(0);
                    first_c_rec_flag = false;
                }
                counter ++;
                if (counter >= max) {
                    max = max * 10;
                    spacing.pop_back();
                }
            } else if (last_l_flag) {
                storedJoints.push_back(g1JointPos);
                storedFlags.push_back(1);
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
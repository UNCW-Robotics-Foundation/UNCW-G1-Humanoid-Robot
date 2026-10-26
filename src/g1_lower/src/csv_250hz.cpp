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

            thread_ = std::thread([this]() { UserInputLoop(); });
            // thread_ = std::thread([this]() { ModifyFile(); });

            timer1_ = this->create_wall_timer(std::chrono::milliseconds(4),
                                      [this] { RosControl(); });
        }

    private:
        std::thread thread_;
        std::array<float, 29> g1JointPos{};
        std::vector<std::array<float, 29>> storedJoints;
        std::vector<int> storedFlags;

        rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr suber_;
        rclcpp::TimerBase::SharedPtr timer1_;

        int recording_flag = 99;
        int counter = 0;
        std::chrono::milliseconds wait_rec_time = 30000ms;
        std::chrono::milliseconds wait_rec_time2 = 10000ms;

        bool state_received = false;
        bool initial_continuous_flag = true;

        void LowStateHandler(unitree_hg::msg::LowState::SharedPtr message) {
            for (int i = 0; i < 29; i++) {
                g1JointPos[i] = message->motor_state[i].q;
            }
            state_received = true;
        }

        void UserInputLoop() {
            std::string str;
            int tmp_int;
            while (!state_received) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "Waiting for LowState...");
                std::this_thread::sleep_for(100ms);
            }
            RCLCPP_INFO(this->get_logger(), "LowState subscribed. Recording starting...");

            DisplayInstructions();
            std::getline(std::cin, str);
            while (str != "") {
                std::cout << std::endl;
                try {
                    tmp_int = std::stoi(str);
                    if ((tmp_int != 0) && (tmp_int != 1) && (tmp_int != 4) && (tmp_int != 5) && (tmp_int != 6)) {
                        std::cout << "Incorrect value. Try again: " << std::endl;

                    } else if (tmp_int == 1) {
                        initial_continuous_flag = true;
                        recording_flag = tmp_int;
                        std::cout << "Input anything to end continuous recording: " ;
                        std::getline(std::cin, str);
                        recording_flag = 99;
                        std::cout << counter << " points recorded" << std::endl;

                    } else if (tmp_int == 5) {
                        std::cout << "20s timer started... " << std::endl;
                        std::this_thread::sleep_for(wait_rec_time);
                        recording_flag = 0;
                        counter ++;
                        std::cout << counter << " points recorded" << std::endl;

                    } else if (tmp_int == 6) {
                        std::cout << "20s timer started... " << std::endl;
                        std::this_thread::sleep_for(wait_rec_time);
                        std::cout << "Recording started... " << std::endl;
                        initial_continuous_flag = true;
                        recording_flag = 1;
                        std::this_thread::sleep_for(wait_rec_time2);
                        recording_flag = 99;
                        std::cout << counter << " points recorded" << std::endl;

                    } else {
                        recording_flag = tmp_int;
                        if (recording_flag == 4) {
                            counter ++;
                        }
                        counter ++;
                        std::cout << counter << " points recorded" << std::endl;
                    }

                } catch(const std::exception& ex) {
                    std::cout << "Error: Incorrect input. Try again: " << std::endl;
                }

                DisplayInstructions();
                std::getline(std::cin, str);
            }

            if (counter == 0) {
                std::cout << "Program stopped. No points recorded" << std::endl;
            } else {
                std::cout << "Program stopped." << std::endl;
                FileWriter();
            }
            rclcpp::shutdown();
        }

        void DisplayInstructions() {
            std::cout << std::endl;
            std::cout << "Empty input to end recording" << std::endl;
            std::cout << "0 = Single point" << std::endl;
            std::cout << "1 = Continuous points" << std::endl;
            std::cout << "4 = Single point with wait flag at end" << std::endl;
            std::cout << "5 = Timer start then single point" << std::endl;
            std::cout << "6 = Timer start then continuous for 10s" << std::endl;
        }

        void RosControl() {
            switch (recording_flag) {
                case 0:         // Single point
                    storedJoints.push_back(g1JointPos);
                    storedFlags.push_back(0);
                    recording_flag = 99;
                    break;

                case 1:         // Single point into continuous
                    if (initial_continuous_flag) {
                        storedJoints.push_back(g1JointPos);
                        storedFlags.push_back(0);
                        counter ++;
                        initial_continuous_flag = false;
                    }

                    storedJoints.push_back(g1JointPos);
                    storedFlags.push_back(1);
                    counter ++;
                    break;

                case 4:         // Single point into wait
                    storedJoints.push_back(g1JointPos);
                    storedFlags.push_back(0);
                    storedJoints.push_back(g1JointPos);
                    storedFlags.push_back(4);
                    recording_flag = 99;
                    break;

                default:
                    break;
            }
        }

        void FileWriter() {
            int stored_joints_size = storedJoints.size();
            std::string tmp_line;
            std::ofstream CSVFile("csv_test.csv");
            RCLCPP_INFO(this->get_logger(), "Writing to csv...");
            CSVFile << "LEFT_HIP_PITCH,LEFT_HIP_ROLL,LEFT_HIP_YAW,LEFT_KNEE,LEFT_ANKLE_PITCH,LEFT_ANKLE_ROLL,RIGHT_HIP_PITCH,RIGHT_HIP_ROLL,RIGHT_HIP_YAW,RIGHT_KNEE,RIGHT_ANKLE_PITCH,RIGHT_ANKLE_ROLL,WAIST_YAW,WAIST_ROLL,WAIST_PITCH,LEFT_SHOULDER_PITCH,LEFT_SHOULDER_ROLL,LEFT_SHOULDER_YAW,LEFT_ELBOW,LEFT_WRIST_ROLL,LEFT_WRIST_PITCH,LEFT_WRIST_YAW,RIGHT_SHOULDER_PITCH,RIGHT_SHOULDER_ROLL,RIGHT_SHOULDER_YAW,RIGHT_ELBOW,RIGHT_WRIST_ROLL,RIGHT_WRIST_PITCH,RIGHT_WRIST_YAW,FLAG\n";
            for (int i = 0; i < stored_joints_size; i++) {
                tmp_line = std::to_string(storedJoints[i][0]);
                for (int j = 1; j < 29; j++) {
                    tmp_line += "," + std::to_string(storedJoints[i][j]);
                }
                tmp_line += "," + std::to_string(storedFlags[i]);
                if (i < stored_joints_size - 1) {
                   tmp_line += "\n"; 
                }
                CSVFile << tmp_line;
            }
            CSVFile.close();
            RCLCPP_INFO(this->get_logger(), "Writing complete.");
        }

        void ModifyFile() {
            std::array<float, 29> poms = {-0.151660,-0.016294,0.028471,0.295607,-0.142253,0.003667,-0.117997,0.002625,0.012213,0.318608,-0.215959,-0.008948,0.000830,-0.000455,-0.000184,0.256199,0.279068,-0.080510,0.828170,0.002936,0.008341,0.004674,0.257062,-0.288149,0.091044,0.791426,-0.006867,0.007538,0.004123};
            int csv_size = 0;
            std::vector<std::vector<std::string>> csv_data;
            std::ifstream GestureFile("gestures/pom_poms.csv");
            std::string raw_csv;
            getline (GestureFile, raw_csv);
            while (getline (GestureFile, raw_csv)) {
                std::stringstream ss(raw_csv);
                std::string cell;
                std::vector<std::string> row;

                while (std::getline(ss, cell, ',')) {
                row.push_back(cell);
                }
                csv_data.push_back(row);
            }

            csv_size = csv_data.size();
            for (int i = 0; i < csv_size; i++) {
                for (int j = 12; j < 22; j ++) {
                    if (j > 14) {
                        csv_data[i][j + 7] = std::to_string(poms[j + 7]);
                    } else {
                        csv_data[i][j] = std::to_string(poms[j]);
                    }
                }
            }

            std::string tmp_line;
            std::ofstream CSVFile("csv_modified.csv");
            RCLCPP_INFO(this->get_logger(), "Writing to csv...");
            CSVFile << "LEFT_HIP_PITCH,LEFT_HIP_ROLL,LEFT_HIP_YAW,LEFT_KNEE,LEFT_ANKLE_PITCH,LEFT_ANKLE_ROLL,RIGHT_HIP_PITCH,RIGHT_HIP_ROLL,RIGHT_HIP_YAW,RIGHT_KNEE,RIGHT_ANKLE_PITCH,RIGHT_ANKLE_ROLL,WAIST_YAW,WAIST_ROLL,WAIST_PITCH,LEFT_SHOULDER_PITCH,LEFT_SHOULDER_ROLL,LEFT_SHOULDER_YAW,LEFT_ELBOW,LEFT_WRIST_ROLL,LEFT_WRIST_PITCH,LEFT_WRIST_YAW,RIGHT_SHOULDER_PITCH,RIGHT_SHOULDER_ROLL,RIGHT_SHOULDER_YAW,RIGHT_ELBOW,RIGHT_WRIST_ROLL,RIGHT_WRIST_PITCH,RIGHT_WRIST_YAW,FLAG\n";
            for (int i = 0; i < csv_size; i++) {
                tmp_line = csv_data[i][0];
                for (int j = 1; j < 30; j++) {
                    tmp_line += "," + csv_data[i][j];
                }
                if (i < csv_size - 1) {
                   tmp_line += "\n"; 
                }
                CSVFile << tmp_line;
            }
            CSVFile.close();
            RCLCPP_INFO(this->get_logger(), "Writing complete.");
            rclcpp::shutdown();
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
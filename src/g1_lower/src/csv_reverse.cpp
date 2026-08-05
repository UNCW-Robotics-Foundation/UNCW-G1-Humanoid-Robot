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

            thread_ = std::thread([this]() { ControlLoop(); });
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
        std::list<std::string> storedStrings;

        int counter = 0;
        int max = 10;
        std::string spacing = "         ";

        void ControlLoop() {
            std::ifstream file("wings.csv");
            std::string str;
            while (std::getline(file, str)) {
                storedStrings.push_front(str);
                str.clear();
            }
            RCLCPP_INFO(this->get_logger(), "conversion ended");
            FileWriter();
            rclcpp::shutdown();
        }

        void FileWriter() {
            std::ofstream CSVFile("csv_reversed.csv");
            RCLCPP_INFO(this->get_logger(), "Writing to csv...");
            CSVFile << "LEFT_HIP_PITCH,LEFT_HIP_ROLL,LEFT_HIP_YAW,LEFT_KNEE,LEFT_ANKLE_PITCH,LEFT_ANKLE_ROLL,RIGHT_HIP_PITCH,RIGHT_HIP_ROLL,RIGHT_HIP_YAW,RIGHT_KNEE,RIGHT_ANKLE_PITCH,RIGHT_ANKLE_ROLL,WAIST_YAW,WAIST_ROLL,WAIST_PITCH,LEFT_SHOULDER_PITCH,LEFT_SHOULDER_ROLL,LEFT_SHOULDER_YAW,LEFT_ELBOW,LEFT_WRIST_ROLL,LEFT_WRIST_PITCH,LEFT_WRIST_YAW,RIGHT_SHOULDER_PITCH,RIGHT_SHOULDER_ROLL,RIGHT_SHOULDER_YAW,RIGHT_ELBOW,RIGHT_WRIST_ROLL,RIGHT_WRIST_PITCH,RIGHT_WRIST_YAW,FLAG\n";
            for (std::string line : storedStrings) {
                CSVFile << line + "\n";
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
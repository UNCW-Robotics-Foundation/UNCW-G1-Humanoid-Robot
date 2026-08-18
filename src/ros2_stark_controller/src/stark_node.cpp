/**
 * stark_node_v2.cpp — Fixed dual-hand ROS2 controller for Brainco Revo2 Touch hands
 *
 * Requires SDK v2.0.2+ (libbc_stark_sdk.so from brainco-hand-sdk download-lib.sh).
 *
 * Key fixes over original stark_node.cpp:
 *  - Calls stark_enable_touch_sensor() for both hands during init — the SDK requires
 *    explicit enable before any touch read will succeed (root cause of skipped readings)
 *  - Sets hardware type via stark_set_hardware_type() + stark_get_device_info() first
 *  - Fixed slave_id assignment bug in joint callbacks (slave_id_=0x7e → slave_id_l_)
 *  - Right-hand touch status now published on /touch_status_r
 *  - publish_motor no longer redundantly queries both hands in a single function
 *
 * Published topics:
 *   /motor_status    (ros2_stark_interfaces/MotorStatus)  left hand
 *   /motor_status_r  (ros2_stark_interfaces/MotorStatus)  right hand
 *   /touch_status    (ros2_stark_interfaces/TouchStatus)  left hand
 *   /touch_status_r  (ros2_stark_interfaces/TouchStatus)  right hand
 *   /turbo_mode      (std_msgs/Bool)
 *   /voltage         (std_msgs/UInt16)
 *
 * Subscribed topics:
 *   /joint_commands_left   (sensor_msgs/JointState)
 *   /joint_commands_right  (sensor_msgs/JointState)
 */

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int16.hpp"

#include "ros2_stark_interfaces/msg/motor_status.hpp"
#include "ros2_stark_interfaces/msg/touch_status.hpp"

#include "ros2_stark_controller/stark-sdk.h"

class StarkNode : public rclcpp::Node {
public:
    StarkNode() : Node("stark_node") {
        // ── Parameters ────────────────────────────────────────────────────────
        port_l_      = declare_parameter<std::string>("port_l",      "/dev/ttyUSB0");
        port_r_      = declare_parameter<std::string>("port_r",      "/dev/ttyUSB1");
        baudrate_    = declare_parameter<int>("baudrate",            115200);
        slave_id_l_  = static_cast<uint8_t>(declare_parameter<int>("slave_id_l", 1));
        slave_id_r_  = static_cast<uint8_t>(declare_parameter<int>("slave_id_r", 1));
        log_screen_  = declare_parameter<bool>("log_screen",        true);

        RCLCPP_INFO(get_logger(), "Left  hand: %s  baud=%d  id=%d",
                    port_l_.c_str(), baudrate_, slave_id_l_);
        RCLCPP_INFO(get_logger(), "Right hand: %s  baud=%d  id=%d",
                    port_r_.c_str(), baudrate_, slave_id_r_);

        init_logging(LOG_LEVEL_INFO);

        if (!open_and_init_hand(port_l_, baudrate_, slave_id_l_, handle_l_, "LEFT"))
            throw std::runtime_error("Left hand init failed");
        if (!open_and_init_hand(port_r_, baudrate_, slave_id_r_, handle_r_, "RIGHT"))
            throw std::runtime_error("Right hand init failed");

        // ── Publishers ────────────────────────────────────────────────────────
        motor_pub_l_ = create_publisher<ros2_stark_interfaces::msg::MotorStatus>("motor_status",   10);
        motor_pub_r_ = create_publisher<ros2_stark_interfaces::msg::MotorStatus>("motor_status_r", 10);
        touch_pub_l_ = create_publisher<ros2_stark_interfaces::msg::TouchStatus>("touch_status",   10);
        touch_pub_r_ = create_publisher<ros2_stark_interfaces::msg::TouchStatus>("touch_status_r", 10);
        turbo_pub_   = create_publisher<std_msgs::msg::Bool>("turbo_mode", 10);
        voltage_pub_ = create_publisher<std_msgs::msg::UInt16>("voltage",   10);

        // ── Subscriptions ─────────────────────────────────────────────────────
        joint_sub_l_ = create_subscription<sensor_msgs::msg::JointState>(
            "joint_commands_left", 10,
            std::bind(&StarkNode::joint_cmd_left, this, std::placeholders::_1));
        joint_sub_r_ = create_subscription<sensor_msgs::msg::JointState>(
            "joint_commands_right", 10,
            std::bind(&StarkNode::joint_cmd_right, this, std::placeholders::_1));

        // ── Timers ────────────────────────────────────────────────────────────
        poll_timer_ = create_wall_timer(std::chrono::milliseconds(50),
                          std::bind(&StarkNode::on_poll, this));
        info_timer_ = create_wall_timer(std::chrono::seconds(30),
                          std::bind(&StarkNode::on_info_poll, this));

        RCLCPP_INFO(get_logger(), "Waiting for joint commands…");
    }

    ~StarkNode() {
        if (handle_l_) close_device_handler(handle_l_, STARK_PROTOCOL_TYPE_MODBUS);
        if (handle_r_) close_device_handler(handle_r_, STARK_PROTOCOL_TYPE_MODBUS);
    }

private:
    // ── Init helper ───────────────────────────────────────────────────────────

    bool open_and_init_hand(const std::string& port, int baud,
                             uint8_t slave_id, DeviceHandler*& out_handle,
                             const char* label) {
        out_handle = modbus_open(port.c_str(), static_cast<uint32_t>(baud));
        if (!out_handle) {
            RCLCPP_ERROR(get_logger(), "[%s] Failed to open %s at %d baud",
                         label, port.c_str(), baud);
            return false;
        }

        // Set a safe default so get_device_info parses correctly, then update
        // to the real hardware type the device reports.
        stark_set_hardware_type(out_handle, slave_id, STARK_HARDWARE_TYPE_REVO2_BASIC);
        CDeviceInfo* info = stark_get_device_info(out_handle, slave_id);
        if (info) {
            stark_set_hardware_type(out_handle, slave_id,
                static_cast<StarkHardwareType>(info->hardware_type));
            RCLCPP_INFO(get_logger(), "[%s] SN=%s  FW=%s  HW=%u",
                        label, info->serial_number, info->firmware_version,
                        info->hardware_type);
            free_device_info(info);
        } else {
            RCLCPP_WARN(get_logger(),
                "[%s] Could not read device info — using preset HW type", label);
        }

        // Enable all 5 touch sensors (bits 0-4 = fingers 0-4).
        // Without this the SDK returns null for every touch read.
        stark_enable_touch_sensor(out_handle, slave_id, 0x1F);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Reset + zero-calibrate the touch sensors while the hand is unloaded
        // (nothing pressing on it yet at this point in startup). Without this,
        // per-sensor baseline drift can leave idle-state force readings
        // elevated enough to look like contact at rest. stark-sdk.h documents
        // stark_calibrate_touch_sensor for exactly this case ("use when 3D
        // force values are non-zero in idle state").
        stark_reset_touch_sensor(out_handle, slave_id, 0x1F);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        stark_calibrate_touch_sensor(out_handle, slave_id, 0x1F);

        RCLCPP_INFO(get_logger(), "[%s] Touch sensors enabled, reset, and calibrated — waiting 1 s…", label);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        return true;
    }

    // ── Timer callbacks ───────────────────────────────────────────────────────

    void on_poll() {
        publish_motor(handle_l_, slave_id_l_, motor_pub_l_, "LEFT");
        publish_motor(handle_r_, slave_id_r_, motor_pub_r_, "RIGHT");
        publish_touch(handle_l_, slave_id_l_, touch_pub_l_, "LEFT");
        publish_touch(handle_r_, slave_id_r_, touch_pub_r_, "RIGHT");
    }

    void on_info_poll() {
        auto v = stark_get_voltage(handle_l_, slave_id_l_);
        auto msg = std_msgs::msg::UInt16();
        msg.data = static_cast<uint16_t>(v);
        voltage_pub_->publish(msg);
    }

    // ── Publish helpers ───────────────────────────────────────────────────────

    void publish_motor(DeviceHandler* handle, uint8_t slave_id,
                       rclcpp::Publisher<ros2_stark_interfaces::msg::MotorStatus>::SharedPtr& pub,
                       const char* label) {
        auto* data = stark_get_motor_status(handle, slave_id);
        if (!data) {
            if (log_screen_)
                RCLCPP_WARN(get_logger(), "Failed to get [%s] motor status", label);
            return;
        }
        auto msg = ros2_stark_interfaces::msg::MotorStatus();
        std::copy(data->positions, data->positions + 6, msg.positions.begin());
        std::copy(data->speeds,    data->speeds    + 6, msg.speeds.begin());
        std::copy(data->currents,  data->currents  + 6, msg.currents.begin());
        std::copy(data->states,    data->states    + 6, msg.states.begin());
        pub->publish(msg);
        free_motor_status_data(data);
    }

    void publish_touch(DeviceHandler* handle, uint8_t slave_id,
                       rclcpp::Publisher<ros2_stark_interfaces::msg::TouchStatus>::SharedPtr& pub,
                       const char* label) {
        auto* data = stark_get_touch_status(handle, slave_id);
        if (!data) {
            if (log_screen_)
                RCLCPP_WARN(get_logger(), "Failed to get [%s] touch status", label);
            return;
        }
        auto msg = ros2_stark_interfaces::msg::TouchStatus();
        for (int i = 0; i < 5; ++i) {
            const auto& item = data->items[i];
            msg.data[i].normal_force1         = item.normal_force1;
            msg.data[i].normal_force2         = item.normal_force2;
            msg.data[i].normal_force3         = item.normal_force3;
            msg.data[i].tangential_force1     = item.tangential_force1;
            msg.data[i].tangential_force2     = item.tangential_force2;
            msg.data[i].tangential_force3     = item.tangential_force3;
            msg.data[i].tangential_direction1 = item.tangential_direction1;
            msg.data[i].tangential_direction2 = item.tangential_direction2;
            msg.data[i].tangential_direction3 = item.tangential_direction3;
            msg.data[i].self_proximity1       = item.self_proximity1;
            msg.data[i].self_proximity2       = item.self_proximity2;
            msg.data[i].mutual_proximity      = item.mutual_proximity;
            msg.data[i].status                = item.status;
        }
        pub->publish(msg);
        free_touch_finger_data(data);
    }

    // ── Joint command callbacks ───────────────────────────────────────────────

    void joint_cmd_left(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (log_screen_) RCLCPP_INFO(get_logger(), "Received joint commands left");
        if (msg->position.size() < 6) return;
        if (msg->velocity.size() < 6) return;
        uint16_t pos[6];
        uint16_t speed[6];
        for (int i = 0; i < 6; ++i) {
            pos[i] = static_cast<uint16_t>(msg->position[i] * 100);
            speed[i] = static_cast<uint16_t>(msg->velocity[i] * 100);
        }
        if (log_screen_) {
            RCLCPP_INFO(get_logger(), "Left positions: %d %d %d %d %d %d",
                        pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
            RCLCPP_INFO(get_logger(), "Left speeds: %d %d %d %d %d %d",
                        speed[0], speed[1], speed[2], speed[3], speed[4], speed[5]);
        }
        stark_set_finger_positions_and_speeds(handle_l_, slave_id_l_, pos, speed, 6);
    }

    void joint_cmd_right(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (log_screen_) RCLCPP_INFO(get_logger(), "Received joint commands right");
        if (msg->position.size() < 6) return;
        if (msg->velocity.size() < 6) return;
        uint16_t pos[6];
        uint16_t speed[6];
        for (int i = 0; i < 6; ++i) {
            pos[i] = static_cast<uint16_t>(msg->position[i] * 100);
            speed[i] = static_cast<uint16_t>(msg->velocity[i] * 100);
        }
        if (log_screen_) {
            RCLCPP_INFO(get_logger(), "Right positions: %d %d %d %d %d %d",
                        pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
            RCLCPP_INFO(get_logger(), "Right speeds: %d %d %d %d %d %d",
                        speed[0], speed[1], speed[2], speed[3], speed[4], speed[5]);
        }
        stark_set_finger_positions_and_speeds(handle_r_, slave_id_r_, pos, speed, 6);
    }

    // ── Member variables ──────────────────────────────────────────────────────

    std::string    port_l_, port_r_;
    int            baudrate_;
    uint8_t        slave_id_l_, slave_id_r_;
    bool           log_screen_;

    DeviceHandler* handle_l_ = nullptr;
    DeviceHandler* handle_r_ = nullptr;

    rclcpp::Publisher<ros2_stark_interfaces::msg::MotorStatus>::SharedPtr motor_pub_l_, motor_pub_r_;
    rclcpp::Publisher<ros2_stark_interfaces::msg::TouchStatus>::SharedPtr touch_pub_l_, touch_pub_r_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    turbo_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr  voltage_pub_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_l_, joint_sub_r_;
    rclcpp::TimerBase::SharedPtr poll_timer_, info_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StarkNode>());
    rclcpp::shutdown();
    return 0;
}

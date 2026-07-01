/**
 * This example demonstrates how to use ROS2 to receive wireless controller
 *states of unitree go2 robot
 **/
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg//wireless_controller.hpp"

#include "base_client.hpp"
#include "common/time_tools.hpp"
#include "common/ut_errror.hpp"
#include "nlohmann/json.hpp"
#include "patch.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"

constexpr int32_t ROBOT_API_ID_AUDIO_TTS = 1001;
constexpr int32_t ROBOT_API_ID_AUDIO_ASR = 1002;
constexpr int32_t ROBOT_API_ID_AUDIO_START_PLAY = 1003;
constexpr int32_t ROBOT_API_ID_AUDIO_STOP_PLAY = 1004;
constexpr int32_t ROBOT_API_ID_AUDIO_GET_VOLUME = 1005;
constexpr int32_t ROBOT_API_ID_AUDIO_SET_VOLUME = 1006;
constexpr int32_t ROBOT_API_ID_AUDIO_SET_RGB_LED = 1010;

class WirelessControllerSuber : public rclcpp::Node {
 public:
  WirelessControllerSuber() : Node("wireless_controller_suber") {
    // the cmd_puber is set to subscribe "/wirelesscontroller" topic
    suber_ = this->create_subscription<unitree_go::msg::WirelessController>(
        "/wirelesscontroller", 10,
        [this](const unitree_go::msg::WirelessController::SharedPtr data) {
          topic_callback(data);
        });

    pub_ = this->create_publisher<unitree_api::msg::Request>("/api/voice/request", 10);
  }

 private:
 rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr suber_;
 rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr pub_;
 bool once_flag = true;
 uint32_t tts_index_ = 0;

  void topic_callback(const unitree_go::msg::WirelessController::SharedPtr& data) {
    if ((data->keys == 1280) && (once_flag)) {
      nlohmann::json js;
      unitree_api::msg::Request req;
      req.header.identity.api_id = ROBOT_API_ID_AUDIO_TTS;
      js["index"] = tts_index_++;
      js["text"] = "Destroy all humans! With kindness.";
      js["speaker_id"] = 1;
      req.parameter = js.dump();
      pub_->publish(req);
      once_flag = false;
    } else if ((data->keys == 0) && !(once_flag)) {
      once_flag = true;
    }

  }

  int32_t PlayStream(const std::string &app_name, const std::string &stream_id,
                     const std::vector<uint8_t> &pcm_data) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_AUDIO_START_PLAY;
    nlohmann::json js;
    js["app_name"] = app_name;
    js["stream_id"] = stream_id;
    req.parameter = js.dump();
    req.binary = pcm_data;
    pub_->publish(req);
  }

};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);  // Initialize rclcpp
  // Run ROS2 node which is make share with wireless_controller_suber class
  rclcpp::spin(std::make_shared<WirelessControllerSuber>());
  rclcpp::shutdown();
  return 0;
}
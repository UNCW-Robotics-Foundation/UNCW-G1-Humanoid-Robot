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
#include <fstream>
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "g1/g1_loco_client.hpp"
#include "g1/g1_audio_client.hpp"
#include "g1_msgs/msg/status_request.hpp"

constexpr int32_t ROBOT_API_ID_AUDIO_TTS = 1001;
constexpr int32_t ROBOT_API_ID_AUDIO_ASR = 1002;
constexpr int32_t ROBOT_API_ID_AUDIO_START_PLAY = 1003;
constexpr int32_t ROBOT_API_ID_AUDIO_STOP_PLAY = 1004;
constexpr int32_t ROBOT_API_ID_AUDIO_GET_VOLUME = 1005;
constexpr int32_t ROBOT_API_ID_AUDIO_SET_VOLUME = 1006;
constexpr int32_t ROBOT_API_ID_AUDIO_SET_RGB_LED = 1010;

// wave reader start
struct WaveHeader {
  void SeekToDataChunk(std::istream &is) {
    while (is && subchunk2_id != 0x61746164) {
      is.seekg(subchunk2_size, std::istream::cur);
      is.read(reinterpret_cast<char *>(&subchunk2_id), sizeof(int32_t));
      is.read(reinterpret_cast<char *>(&subchunk2_size), sizeof(int32_t));
    }
  }

  int32_t chunk_id;
  int32_t chunk_size;
  int32_t format;
  int32_t subchunk1_id;
  int32_t subchunk1_size;
  int16_t audio_format;
  int16_t num_channels;
  int32_t sample_rate;
  int32_t byte_rate;
  int16_t block_align;
  int16_t bits_per_sample;
  int32_t subchunk2_id;    // a tag of this chunk
  int32_t subchunk2_size;  // size of subchunk2
};

class WirelessControllerSuber : public rclcpp::Node {
 public:
  WirelessControllerSuber() : Node("wireless_controller_suber"), loco_client_(this), audio_client_() {
    // the cmd_puber is set to subscribe "/wirelesscontroller" topic
    // suber_ = this->create_subscription<unitree_go::msg::WirelessController>(
    //   "/wirelesscontroller", 10,
    //   [this](const unitree_go::msg::WirelessController::SharedPtr data) {
    //     topic_callback(data);
    //   });

    fsm_suber_ = this->create_subscription<unitree_api::msg::Response>(
      "/api/sport/response", 10,
      [this](const unitree_api::msg::Response::SharedPtr data) {
        fsm_callback(data);
      });

    status_suber_ = this->create_subscription<g1_msgs::msg::StatusRequest>(
      "/gesture_status", 10,
      [this](const g1_msgs::msg::StatusRequest::SharedPtr data) {
        status_callback(data);
      });

    pub_ = this->create_publisher<unitree_api::msg::Request>("/api/voice/request", 10);
    // pub_loco_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [this] { Control(); });
  }

 private:
 rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr suber_;
 rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr fsm_suber_;
 rclcpp::Subscription<g1_msgs::msg::StatusRequest>::SharedPtr status_suber_;
 rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr pub_;
 // rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr pub_loco_;

 rclcpp::TimerBase::SharedPtr timer_;

 unitree::robot::g1::LocoClient loco_client_;
 unitree::ros2::g1::AudioClient audio_client_;

 bool once_flag = true;
 bool playing_flag = false;
 bool led_flag = true;
 bool save_time_flag = true;

 uint32_t tts_index_ = 0;
 std::string audio_file_path;

 int fsm_id = 0;
 double saved_time;
 int rgb_setting = 0;

 void Control() {
  if ((led_flag) && (fsm_id == 801)) {
    std::string txt = "Sending command. led_flag = " + std::to_string(led_flag);
    if (save_time_flag) {
      RCLCPP_INFO(this->get_logger(), txt.c_str());
      save_time_flag = false;
      saved_time = this->get_clock()->now().seconds();

      unitree_api::msg::Request req;
      req.header.identity.api_id = ROBOT_API_ID_AUDIO_SET_RGB_LED;
      nlohmann::json js;
      switch (rgb_setting)
      {
      case 0:
        js["R"] = 0;
        js["G"] = 51;
        js["B"] = 102;
        break;

      case 1:
        // js["R"] = 249;
        // js["G"] = 227;
        // js["B"] = 127;
        js["R"] = 255;
        js["G"] = 255;
        js["B"] = 0;
        break;
      
      case 2:
        js["R"] = 0;
        js["G"] = 118;
        js["B"] = 128;
        break;
      
      default:
        break;
      }

      req.parameter = js.dump();
      pub_->publish(req);
    } else if (this->get_clock()->now().seconds() - saved_time > 2.01) {
      save_time_flag = true;
      if (rgb_setting < 2) {
        rgb_setting ++;
      } else {
        rgb_setting = 0;
      }
      RCLCPP_INFO(this->get_logger(), "Switching color to %i", rgb_setting);
    }

    // RCLCPP_INFO(this->get_logger(), "time elapsed: %f; led_control: %s", this->get_clock()->now().seconds() - saved_time, txt.c_str());
  }
 }

 void status_callback(const g1_msgs::msg::StatusRequest::SharedPtr& data) {
  int code = data->code;
  nlohmann::json js;
  unitree_api::msg::Request req;
  req.header.identity.api_id = ROBOT_API_ID_AUDIO_TTS;
  js["index"] = tts_index_++;
  js["speaker_id"] = 1;

  switch (code)
  {
  case 0:     // Gesture Controller; reads flag data
    if (data->flag) {
      js["text"] = "Custom gesture control activated.";
      req.parameter = js.dump();
      pub_->publish(req);
      led_flag = true;
    } else {
      js["text"] = "Custom gesture control deactivated.";
      req.parameter = js.dump();
      pub_->publish(req);
      led_flag = false;
    }
    break;
  
  default:
    js["text"] = "Unknown status request";
    req.parameter = js.dump();
    pub_->publish(req);
    break;
  }
 }

 void fsm_callback(const unitree_api::msg::Response::SharedPtr& data) {
  nlohmann::json js = nlohmann::json::parse(data->data);
  js["data"].get_to(fsm_id);

  // RCLCPP_INFO(this->get_logger(), "Current fsm id: %i", fsm_id);
 }

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
    } else if ((data->keys == 2112) && (once_flag)) {
      once_flag = false;
      if (playing_flag) {
        PlayStop("pump");
        playing_flag = false;
      } else{
        playing_flag = true;
        int32_t sample_rate = -1;
        int8_t num_channels = 0;
        bool filestate = false;
        std::vector<uint8_t> pcm = ReadWave("pump_it_up.wav", &sample_rate, &num_channels, &filestate);
        PlayStream(
          "pump",
          std::to_string(unitree::common::GetCurrentTimeMilliseconds()), pcm);
      }
    } else if ((data->keys == 128) && (once_flag)) {  // F3
      // unitree_api::msg::Request req;
      // req.header.identity.api_id = ROBOT_API_ID_LOCO_GET_FSM_ID;
      // nlohmann::json js;
      // pub_loco_->publish(req);

      std::string tts_text = "Currently in fsm mode " + std::to_string(fsm_id);
      audio_client_.TtsMaker(tts_text, 1);
      once_flag = false;
    } else if ((data->keys == 64) && (once_flag)) {  // F1
      if (led_flag) {
        led_flag = false;
      } else {
        led_flag = true;
      }
      RCLCPP_INFO(this->get_logger(), "led_flag switching to %s", std::to_string(led_flag).c_str());
      once_flag = false;
    } else if ((data->keys == 0) && !(once_flag)) {
      once_flag = true;
    }

  }

  std::vector<uint8_t> ReadWave(const std::string &filename,
                              int32_t *sampling_rate, int8_t *channel_count,
                              bool *is_ok) {
    std::ifstream is(filename, std::ifstream::binary);
    auto samples = ReadWaveImpl(is, sampling_rate, channel_count, is_ok);
    return samples;
    }

    std::vector<uint8_t> ReadWaveImpl(std::istream &is, int32_t *sampling_rate,
                                    int8_t *channel_count, bool *is_ok) {
    WaveHeader header{};
    is.read(reinterpret_cast<char *>(&header.chunk_id), sizeof(header.chunk_id));

    //                        F F I R
    if (header.chunk_id != 0x46464952) {
      printf("Expected chunk_id RIFF. Given: 0x%08x\n", header.chunk_id);
      *is_ok = false;
      return {};
    }

    is.read(reinterpret_cast<char *>(&header.chunk_size),
            sizeof(header.chunk_size));

    is.read(reinterpret_cast<char *>(&header.format), sizeof(header.format));

    //                      E V A W
    if (header.format != 0x45564157) {
      printf("Expected format WAVE. Given: 0x%08x\n", header.format);
      *is_ok = false;
      return {};
    }

    is.read(reinterpret_cast<char *>(&header.subchunk1_id),
            sizeof(header.subchunk1_id));

    is.read(reinterpret_cast<char *>(&header.subchunk1_size),
            sizeof(header.subchunk1_size));

    if (header.subchunk1_id == 0x4b4e554a) {
      // skip junk padding
      is.seekg(header.subchunk1_size, std::istream::cur);

      is.read(reinterpret_cast<char *>(&header.subchunk1_id),
              sizeof(header.subchunk1_id));

      is.read(reinterpret_cast<char *>(&header.subchunk1_size),
              sizeof(header.subchunk1_size));
    }

    if (header.subchunk1_id != 0x20746d66) {
      printf("Expected subchunk1_id 0x20746d66. Given: 0x%08x\n",
            header.subchunk1_id);
      *is_ok = false;
      return {};
    }

    if (header.subchunk1_size != 16 &&
        header.subchunk1_size != 18) {  // 16 for PCM
      printf("Expected subchunk1_size 16. Given: %d\n", header.subchunk1_size);
      *is_ok = false;
      return {};
    }

    is.read(reinterpret_cast<char *>(&header.audio_format),
            sizeof(header.audio_format));

    if (header.audio_format != 1) {  // 1 for PCM
      printf("Expected audio_format 1. Given: %d\n", header.audio_format);
      *is_ok = false;
      return {};
    }

    is.read(reinterpret_cast<char *>(&header.num_channels),
            sizeof(header.num_channels));

    *channel_count = static_cast<int8_t>(header.num_channels);

    is.read(reinterpret_cast<char *>(&header.sample_rate),
            sizeof(header.sample_rate));

    is.read(reinterpret_cast<char *>(&header.byte_rate),
            sizeof(header.byte_rate));

    is.read(reinterpret_cast<char *>(&header.block_align),
            sizeof(header.block_align));

    is.read(reinterpret_cast<char *>(&header.bits_per_sample),
            sizeof(header.bits_per_sample));

    if (header.byte_rate !=
        (header.sample_rate * header.num_channels * header.bits_per_sample / 8)) {
      printf("Incorrect byte rate: %d. Expected: %d", header.byte_rate,
            (header.sample_rate * header.num_channels * header.bits_per_sample /
              8));
      *is_ok = false;
      return {};
    }

    if (header.block_align !=
        (header.num_channels * header.bits_per_sample / 8)) {
      printf("Incorrect block align: %d. Expected: %d\n", header.block_align,
            (header.num_channels * header.bits_per_sample / 8));
      *is_ok = false;
      return {};
    }

    if (header.bits_per_sample != 16) {  // we support only 16 bits per sample
      printf("Expected bits_per_sample 16. Given: %d\n", header.bits_per_sample);
      *is_ok = false;
      return {};
    }

    if (header.subchunk1_size == 18) {
      int16_t extra_size = -1;
      is.read(reinterpret_cast<char *>(&extra_size), sizeof(int16_t));
      if (extra_size != 0) {
        printf(
            "Extra size should be 0 for wave from NAudio. Current extra size "
            "%d\n",
            extra_size);
        *is_ok = false;
        return {};
      }
    }

    is.read(reinterpret_cast<char *>(&header.subchunk2_id),
            sizeof(header.subchunk2_id));

    is.read(reinterpret_cast<char *>(&header.subchunk2_size),
            sizeof(header.subchunk2_size));

    header.SeekToDataChunk(is);
    if (!is) {
      *is_ok = false;
      return {};
    }

    *sampling_rate = header.sample_rate;

    // header.subchunk2_size contains the number of bytes in the data.
    // As we assume each sample contains two bytes, so it is divided by 2 here
    std::vector<int16_t> samples(header.subchunk2_size / 2);

    is.read(reinterpret_cast<char *>(samples.data()), header.subchunk2_size);
    if (!is) {
      *is_ok = false;
      return {};
    }

    std::vector<uint8_t> ans(samples.size() * 2);
    for (int32_t i = 0; i != static_cast<int32_t>(samples.size()); ++i) {
      ans[i * 2] = samples[i] & 0xFF;
      ans[i * 2 + 1] = (samples[i] >> 8) & 0xFF;
    }

    *is_ok = true;
    return ans;
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

  int32_t PlayStop(const std::string &app_name) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_AUDIO_STOP_PLAY;
    nlohmann::json js;
    js["app_name"] = app_name;
    req.parameter = js.dump();
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
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
 bool playing_flag = false;

 uint32_t tts_index_ = 0;
 std::string audio_file_path;

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
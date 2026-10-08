#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/microphone/microphone.h"
#include "esphome/components/microphone/microphone_source.h"

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "lwip/sockets.h"

namespace esphome {
namespace rtsp_audio {

enum class AudioCodec : uint8_t {
  L16 = 0,
  PCMU = 1,
  PCMA = 2,
};

class RTSPAudioComponent : public Component {
 public:
  void set_microphone(microphone::Microphone *mic) { this->mic_ = mic; }
  void set_port(int port) { this->port_ = port; }
  void set_channel(uint8_t channel) { this->channel_ = channel; }
  void set_gain_factor(int32_t gain_factor) { this->gain_factor_ = gain_factor; }
  void set_codec(const std::string &codec) {
    if (codec == "pcmu") {
      this->codec_ = AudioCodec::PCMU;
    } else if (codec == "pcma") {
      this->codec_ = AudioCodec::PCMA;
    } else {
      this->codec_ = AudioCodec::L16;
    }
  }
  void set_output_sample_rate(int sample_rate) { this->output_sample_rate_config_ = sample_rate; }
  void set_rtp_config(int payload_type, int packet_ms) {
    if (payload_type >= 0) {
      this->rtp_payload_type_ = payload_type;
      this->rtp_payload_type_user_set_ = true;
    }
    this->packet_ms_ = packet_ms;
  }
  void set_debug(bool debug) { this->debug_ = debug; }
  void set_buffer_ms(int buffer_ms) { this->buffer_ms_ = buffer_ms; }
  void set_max_clients(int max_clients) { this->max_clients_ = max_clients < 1 ? 1 : (max_clients > 6 ? 6 : max_clients); }
  void set_status_interval(uint32_t interval_ms) { this->status_interval_ms_ = interval_ms; }
  void set_auth(const std::string &username, const std::string &password, const std::string &realm) {
    this->auth_username_ = username;
    this->auth_password_ = password;
    this->auth_realm_ = realm;
    this->auth_enabled_ = !username.empty();
  }

  void setup() override;
  void loop() override;
  void dump_config() override;
  void on_shutdown() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

 protected:
  bool start_server_task_();
  void stop_server_();
  void log_status_(const char *reason);
  std::string local_ip_() const;

  static void server_task_trampoline_(void *arg);
  static void rtp_task_trampoline_(void *arg);
  static void client_task_trampoline_(void *arg);
  void server_task_();
  void rtp_task_();

  int create_tcp_server_();
  int allocate_client_session_(int fd);
  void release_client_session_(int index);
  int active_client_count_() const;
  int active_stream_count_() const;
  void update_streaming_state_();
  void handle_rtsp_client_(int client_fd, int session_index);
  bool read_rtsp_request_(int fd, std::string &request);
  void send_rtsp_response_(int fd, int code, const char *reason, int cseq, const std::string &headers, const std::string &body);
  int parse_cseq_(const std::string &request) const;
  bool parse_client_ports_(const std::string &request, int *rtp_port, int *rtcp_port) const;
  std::string make_sdp_() const;
  void configure_codec_();
  const char *codec_name_() const;
  const char *rtpmap_name_() const;
  static uint8_t linear_to_ulaw_(int16_t sample);
  static uint8_t linear_to_alaw_(int16_t sample);
  bool request_authorized_(const std::string &request) const;
  void send_auth_required_(int fd, int cseq);
  static std::string base64_encode_(const std::string &input);
  void close_rtp_sockets_(int index);
  void close_all_client_sessions_();

  microphone::Microphone *mic_{nullptr};
  microphone::MicrophoneSource *mic_source_{nullptr};
  StreamBufferHandle_t audio_buffer_{nullptr};
  uint8_t channel_{0};
  int32_t gain_factor_{4};

  int port_{8554};
  int sample_rate_{16000};  // populated from the microphone at setup() time
  int output_sample_rate_{16000};
  int output_sample_rate_config_{0};
  AudioCodec codec_{AudioCodec::L16};
  int rtp_payload_type_{96};
  bool rtp_payload_type_user_set_{false};
  int packet_ms_{20};
  int buffer_ms_{200};
  bool debug_{false};
  uint32_t status_interval_ms_{10000};
  bool auth_enabled_{false};
  std::string auth_username_;
  std::string auth_password_;
  std::string auth_realm_{"ESPHome RTSP Audio"};
  std::string auth_token_;

  std::atomic<bool> running_{false};
  std::atomic<bool> started_{false};
  std::atomic<bool> client_connected_{false};
  std::atomic<bool> streaming_{false};

  TaskHandle_t server_task_handle_{nullptr};
  TaskHandle_t rtp_task_handle_{nullptr};
  int server_fd_{-1};
  struct ClientSession {
    bool allocated{false};
    bool playing{false};
    int control_fd{-1};
    int rtp_fd{-1};
    int rtcp_fd{-1};
    int server_rtp_port{0};
    int server_rtcp_port{0};
    uint32_t ssrc{0};
    uint16_t rtp_sequence{0};
    uint32_t rtp_timestamp{0};
    std::string session_id;
    ::sockaddr_in client_rtp_addr{};
    ::sockaddr_in client_rtcp_addr{};
  };

  struct ClientTaskArg {
    RTSPAudioComponent *self;
    int fd;
    int session_index;
  };

  int max_clients_{2};
  std::vector<ClientSession> sessions_;
  SemaphoreHandle_t sessions_mutex_{nullptr};

  uint32_t setup_attempts_{0};
  std::atomic<uint32_t> rtp_packets_{0};
  std::atomic<uint32_t> rtp_send_errors_{0};
  std::atomic<uint32_t> clipped_samples_{0};
  std::atomic<uint32_t> i2s_reads_{0};
  std::atomic<uint32_t> i2s_empty_reads_{0};
  std::atomic<uint32_t> dropped_bytes_{0};
  std::atomic<int32_t> last_peak_{0};
  std::atomic<int32_t> last_min_{0};
  std::atomic<int32_t> last_max_{0};
  std::atomic<uint32_t> last_bytes_read_{0};
  uint32_t last_status_ms_{0};
  uint32_t last_send_error_log_ms_{0};
  std::string last_error_{"not started yet"};
};

}  // namespace rtsp_audio
}  // namespace esphome

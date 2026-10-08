#include "rtsp_audio.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cinttypes>
#include <vector>

#include "esp_err.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/tcp.h"

namespace esphome {
namespace rtsp_audio {

static const char *const TAG = "rtsp_audio";
// ESP-IDF allocates task stacks from heap. This component previously reserved
// 8 KB for each of its server, client, and RTP tasks, consuming 24 KB for one
// stream before audio and lwIP buffers. The tasks keep packet data on the heap
// and use shallow call frames, so 4 KB matches ESPHome's I2S microphone task.
static constexpr uint32_t RTSP_TASK_STACK_BYTES = 4096;

static uint32_t now_ms() { return (uint32_t) (esp_timer_get_time() / 1000ULL); }

static std::string header_value(const std::string &request, const char *name) {
  std::string needle = std::string(name) + ":";
  std::string lower_req = request;
  std::string lower_needle = needle;
  std::transform(lower_req.begin(), lower_req.end(), lower_req.begin(), ::tolower);
  std::transform(lower_needle.begin(), lower_needle.end(), lower_needle.begin(), ::tolower);
  size_t pos = lower_req.find(lower_needle);
  if (pos == std::string::npos) return "";
  pos += needle.size();
  while (pos < request.size() && (request[pos] == ' ' || request[pos] == '\t')) pos++;
  size_t end = request.find("\r\n", pos);
  if (end == std::string::npos) end = request.find('\n', pos);
  if (end == std::string::npos) end = request.size();
  return request.substr(pos, end - pos);
}

void RTSPAudioComponent::setup() {
  this->setup_attempts_++;
  this->running_ = true;
  this->sessions_mutex_ = xSemaphoreCreateMutex();
  this->sessions_.resize(this->max_clients_);
  ESP_LOGI(TAG, "Native RTSP audio (microphone-source, multiclient v6) setup attempt %" PRIu32, this->setup_attempts_);

  if (this->auth_enabled_) {
    this->auth_token_ = base64_encode_(this->auth_username_ + ":" + this->auth_password_);
    ESP_LOGI(TAG, "RTSP Basic authentication enabled (realm: %s)", this->auth_realm_.c_str());
  }

  if (this->mic_ == nullptr) {
    this->last_error_ = "no microphone: entity configured";
    ESP_LOGE(TAG, "%s", this->last_error_.c_str());
    return;
  }

  // Always request 16-bit from MicrophoneSource: that's exactly what RTP L16
  // needs, and it means MicrophoneSource does the 24/32-bit -> 16-bit
  // conversion for us instead of us hand-rolling a bit-shift (which is what
  // the old direct-I2S version of this component had to do itself).
  this->mic_source_ = new microphone::MicrophoneSource(this->mic_, /*bits_per_sample=*/16, this->gain_factor_,
                                                         /*passive=*/false);
  this->mic_source_->add_channel(this->channel_);

  audio::AudioStreamInfo info = this->mic_source_->get_audio_stream_info();
  this->sample_rate_ = info.get_sample_rate();
  this->configure_codec_();

  // Small buffer between the microphone callback and RTP sender. Keeping this
  // low avoids building up noticeable latency; the trigger level is one RTP
  // packet so the sender wakes for whole packets instead of tiny partial chunks.
  const size_t packet_bytes = std::max<size_t>(160, ((size_t) this->sample_rate_ * (size_t) this->packet_ms_ / 1000U) * 2U);
  size_t buffer_bytes = ((size_t) this->sample_rate_ * (size_t) this->buffer_ms_ / 1000U) * 2U;
  if (buffer_bytes < packet_bytes * 3U) buffer_bytes = packet_bytes * 3U;
  if (buffer_bytes < 4096) buffer_bytes = 4096;
  this->audio_buffer_ = xStreamBufferCreate(buffer_bytes, packet_bytes);
  if (this->audio_buffer_ == nullptr) {
    this->last_error_ = "failed to allocate audio stream buffer";
    ESP_LOGE(TAG, "%s", this->last_error_.c_str());
    return;
  }

  this->mic_source_->add_data_callback([this](const std::vector<uint8_t> &data) {
    if (this->audio_buffer_ != nullptr) {
      // Zero timeout: never block the microphone's own task. If the RTP
      // sender has fallen behind and the buffer is full, this just sends as
      // many bytes as currently fit and silently drops the rest.
      size_t written = xStreamBufferSend(this->audio_buffer_, data.data(), data.size(), 0);
      if (written < data.size()) this->dropped_bytes_ += (uint32_t) (data.size() - written);
    }
  });

  if (!this->start_server_task_()) {
    ESP_LOGE(TAG, "RTSP server startup failed: %s", this->last_error_.c_str());
    return;
  }
  this->started_ = true;
  this->last_error_ = "none";
  ESP_LOGI(TAG, "Native RTSP audio server task started on port %d (microphone: %u Hz, codec: %s/%d)", this->port_,
           (unsigned) this->sample_rate_, this->rtpmap_name_(), this->output_sample_rate_);
  ESP_LOGI(TAG, "RTSP URL: rtsp://%s:%d/", this->local_ip_().c_str(), this->port_);
}

bool RTSPAudioComponent::start_server_task_() {
  if (this->server_task_handle_ != nullptr) return true;
  BaseType_t ok = xTaskCreatePinnedToCore(&RTSPAudioComponent::server_task_trampoline_, "rtsp_audio_srv", RTSP_TASK_STACK_BYTES, this, 5,
                                          &this->server_task_handle_, 0);
  if (ok != pdPASS) {
    this->last_error_ = "xTaskCreate server failed";
    return false;
  }
  return true;
}

void RTSPAudioComponent::server_task_trampoline_(void *arg) {
  auto *self = static_cast<RTSPAudioComponent *>(arg);
  self->server_task_();
  self->server_task_handle_ = nullptr;
  vTaskDelete(nullptr);
}

void RTSPAudioComponent::rtp_task_trampoline_(void *arg) {
  auto *self = static_cast<RTSPAudioComponent *>(arg);
  self->rtp_task_();
  self->rtp_task_handle_ = nullptr;
  vTaskDelete(nullptr);
}

void RTSPAudioComponent::client_task_trampoline_(void *arg) {
  auto *task_arg = static_cast<RTSPAudioComponent::ClientTaskArg *>(arg);
  auto *self = task_arg->self;
  int fd = task_arg->fd;
  int session_index = task_arg->session_index;
  delete task_arg;

  self->handle_rtsp_client_(fd, session_index);
  self->release_client_session_(session_index);
  close(fd);
  ESP_LOGI(TAG, "RTSP client session %d disconnected", session_index);
  vTaskDelete(nullptr);
}

int RTSPAudioComponent::create_tcp_server_() {
  int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (fd < 0) {
    this->last_error_ = std::string("socket failed: ") + strerror(errno);
    return -1;
  }
  int yes = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  ::sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(this->port_);
  if (bind(fd, (::sockaddr *) &addr, sizeof(addr)) < 0) {
    this->last_error_ = std::string("bind failed: ") + strerror(errno);
    close(fd);
    return -1;
  }
  if (listen(fd, this->max_clients_) < 0) {
    this->last_error_ = std::string("listen failed: ") + strerror(errno);
    close(fd);
    return -1;
  }
  return fd;
}

void RTSPAudioComponent::server_task_() {
  while (this->running_) {
    this->server_fd_ = this->create_tcp_server_();
    if (this->server_fd_ < 0) {
      ESP_LOGE(TAG, "RTSP listen setup failed: %s; retrying", this->last_error_.c_str());
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }
    ESP_LOGI(TAG, "RTSP TCP control listening on 0.0.0.0:%d, max_clients=%d", this->port_, this->max_clients_);

    while (this->running_) {
      this->server_stack_free_ = uxTaskGetStackHighWaterMark(nullptr);
      ::sockaddr_in peer = {};
      socklen_t len = sizeof(peer);
      int cfd = accept(this->server_fd_, (::sockaddr *) &peer, &len);
      if (cfd < 0) {
        if (this->running_) ESP_LOGW(TAG, "accept failed: %s", strerror(errno));
        continue;
      }
      char peer_ip[INET_ADDRSTRLEN] = {0};
      inet_ntop(AF_INET, &peer.sin_addr, peer_ip, sizeof(peer_ip));
      int nodelay = 1;
      setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

      int session_index = this->allocate_client_session_(cfd);
      if (session_index < 0) {
        ESP_LOGW(TAG, "Rejecting RTSP client from %s:%u: max_clients=%d reached", peer_ip, ntohs(peer.sin_port), this->max_clients_);
        this->send_rtsp_response_(cfd, 503, "Service Unavailable", 1, "Retry-After: 10\r\n", "");
        close(cfd);
        continue;
      }

      ESP_LOGI(TAG, "RTSP client session %d connected from %s:%u", session_index, peer_ip, ntohs(peer.sin_port));
      auto *arg = new ClientTaskArg{this, cfd, session_index};
      char task_name[20];
      snprintf(task_name, sizeof(task_name), "rtsp_audio_c%d", session_index);
      BaseType_t ok = xTaskCreatePinnedToCore(&RTSPAudioComponent::client_task_trampoline_, task_name, RTSP_TASK_STACK_BYTES, arg, 5, nullptr, 0);
      if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create RTSP client task for session %d", session_index);
        delete arg;
        this->release_client_session_(session_index);
        close(cfd);
      }
    }

    close(this->server_fd_);
    this->server_fd_ = -1;
  }
}

int RTSPAudioComponent::allocate_client_session_(int fd) {
  xSemaphoreTake(this->sessions_mutex_, portMAX_DELAY);
  for (int i = 0; i < (int) this->sessions_.size(); i++) {
    auto &s = this->sessions_[i];
    if (!s.allocated) {
      s = ClientSession{};
      s.allocated = true;
      s.control_fd = fd;
      s.ssrc = 0x45535048u ^ ((uint32_t) esp_random()) ^ ((uint32_t) (i + 1) << 24);
      s.rtp_sequence = (uint16_t) esp_random();
      s.rtp_timestamp = esp_random();
      char session_id[17];
      snprintf(session_id, sizeof(session_id), "%08" PRIx32 "%02x", esp_random(), i & 0xFF);
      s.session_id = session_id;
      this->client_connected_ = true;
      xSemaphoreGive(this->sessions_mutex_);
      return i;
    }
  }
  xSemaphoreGive(this->sessions_mutex_);
  return -1;
}

void RTSPAudioComponent::release_client_session_(int index) {
  if (index < 0 || index >= (int) this->sessions_.size()) return;
  this->close_rtp_sockets_(index);
  xSemaphoreTake(this->sessions_mutex_, portMAX_DELAY);
  this->sessions_[index] = ClientSession{};
  xSemaphoreGive(this->sessions_mutex_);
  this->update_streaming_state_();
}

int RTSPAudioComponent::active_client_count_() const {
  int count = 0;
  for (const auto &s : this->sessions_) if (s.allocated) count++;
  return count;
}

int RTSPAudioComponent::active_stream_count_() const {
  int count = 0;
  for (const auto &s : this->sessions_) if (s.allocated && s.playing && s.rtp_fd >= 0) count++;
  return count;
}

void RTSPAudioComponent::update_streaming_state_() {
  this->client_connected_ = this->active_client_count_() > 0;
  this->streaming_ = this->active_stream_count_() > 0;
}

bool RTSPAudioComponent::read_rtsp_request_(int fd, std::string &request) {
  request.clear();
  char buf[512];
  while (this->running_) {
    int n = recv(fd, buf, sizeof(buf), 0);
    if (n <= 0) return false;
    request.append(buf, n);
    if (request.find("\r\n\r\n") != std::string::npos || request.find("\n\n") != std::string::npos) return true;
    if (request.size() > 4096) return false;
  }
  return false;
}

int RTSPAudioComponent::parse_cseq_(const std::string &request) const {
  std::string v = header_value(request, "CSeq");
  if (v.empty()) return 1;
  return atoi(v.c_str());
}

bool RTSPAudioComponent::parse_client_ports_(const std::string &request, int *rtp_port, int *rtcp_port) const {
  std::string transport = header_value(request, "Transport");
  size_t p = transport.find("client_port=");
  if (p == std::string::npos) return false;
  p += strlen("client_port=");
  int a = atoi(transport.c_str() + p);
  size_t dash = transport.find('-', p);
  int b = (dash == std::string::npos) ? (a + 1) : atoi(transport.c_str() + dash + 1);
  if (a <= 0 || b <= 0) return false;
  *rtp_port = a;
  *rtcp_port = b;
  return true;
}


std::string RTSPAudioComponent::base64_encode_(const std::string &input) {
  static const char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve(((input.size() + 2) / 3) * 4);

  uint32_t val = 0;
  int valb = -6;
  for (uint8_t c : input) {
    val = (val << 8) | c;
    valb += 8;
    while (valb >= 0) {
      out.push_back(table[(val >> valb) & 0x3F]);
      valb -= 6;
    }
  }
  if (valb > -6) out.push_back(table[((val << 8) >> (valb + 8)) & 0x3F]);
  while (out.size() % 4) out.push_back('=');
  return out;
}

bool RTSPAudioComponent::request_authorized_(const std::string &request) const {
  if (!this->auth_enabled_) return true;
  std::string auth = header_value(request, "Authorization");
  if (auth.empty()) return false;

  const std::string prefix = "Basic ";
  if (auth.size() < prefix.size()) return false;
  if (auth.compare(0, prefix.size(), prefix) != 0) return false;

  size_t pos = prefix.size();
  while (pos < auth.size() && (auth[pos] == ' ' || auth[pos] == '\t')) pos++;
  size_t end = auth.size();
  while (end > pos && (auth[end - 1] == ' ' || auth[end - 1] == '\t' || auth[end - 1] == '\r' || auth[end - 1] == '\n')) end--;
  return auth.substr(pos, end - pos) == this->auth_token_;
}

void RTSPAudioComponent::send_auth_required_(int fd, int cseq) {
  std::string h = "WWW-Authenticate: Basic realm=\"" + this->auth_realm_ + "\"\r\n";
  this->send_rtsp_response_(fd, 401, "Unauthorized", cseq, h, "");
}


const char *RTSPAudioComponent::codec_name_() const {
  switch (this->codec_) {
    case AudioCodec::PCMU:
      return "pcmu";
    case AudioCodec::PCMA:
      return "pcma";
    case AudioCodec::L16:
    default:
      return "l16";
  }
}

const char *RTSPAudioComponent::rtpmap_name_() const {
  switch (this->codec_) {
    case AudioCodec::PCMU:
      return "PCMU";
    case AudioCodec::PCMA:
      return "PCMA";
    case AudioCodec::L16:
    default:
      return "L16";
  }
}

void RTSPAudioComponent::configure_codec_() {
  if (this->codec_ == AudioCodec::L16) {
    this->output_sample_rate_ = this->sample_rate_;
    if (!this->rtp_payload_type_user_set_) this->rtp_payload_type_ = 96;
    return;
  }

  this->output_sample_rate_ = this->output_sample_rate_config_ > 0 ? this->output_sample_rate_config_ : 8000;
  if (!this->rtp_payload_type_user_set_) {
    this->rtp_payload_type_ = (this->codec_ == AudioCodec::PCMA) ? 8 : 0;
  }

  if (this->sample_rate_ < this->output_sample_rate_) {
    ESP_LOGW(TAG, "Codec %s requested output_sample_rate=%d but microphone sample_rate=%d; output will run at microphone rate",
             this->codec_name_(), this->output_sample_rate_, this->sample_rate_);
    this->output_sample_rate_ = this->sample_rate_;
  } else if (this->sample_rate_ % this->output_sample_rate_ != 0) {
    ESP_LOGW(TAG, "Codec %s uses simple nearest-sample resampling from %d Hz to %d Hz; prefer integer ratios such as 16000->8000",
             this->codec_name_(), this->sample_rate_, this->output_sample_rate_);
  }
}

uint8_t RTSPAudioComponent::linear_to_ulaw_(int16_t sample) {
  static constexpr int BIAS = 0x84;
  static constexpr int CLIP = 32635;
  int sign = 0;
  int pcm = sample;
  if (pcm < 0) {
    pcm = -pcm;
    sign = 0x80;
  }
  if (pcm > CLIP) pcm = CLIP;
  pcm += BIAS;

  int exponent = 7;
  for (int exp_mask = 0x4000; (pcm & exp_mask) == 0 && exponent > 0; exponent--, exp_mask >>= 1) {}
  int mantissa = (pcm >> (exponent + 3)) & 0x0F;
  return static_cast<uint8_t>(~(sign | (exponent << 4) | mantissa));
}

uint8_t RTSPAudioComponent::linear_to_alaw_(int16_t sample) {
  static const int16_t SEG_END[8] = {0x00FF, 0x01FF, 0x03FF, 0x07FF, 0x0FFF, 0x1FFF, 0x3FFF, 0x7FFF};
  int pcm = sample;
  int mask;
  if (pcm >= 0) {
    mask = 0xD5;
  } else {
    mask = 0x55;
    pcm = -pcm - 1;
  }
  if (pcm > 32635) pcm = 32635;

  int seg = 0;
  while (seg < 8 && pcm > SEG_END[seg]) seg++;

  uint8_t aval;
  if (seg >= 8) {
    aval = 0x7F;
  } else if (seg == 0) {
    aval = static_cast<uint8_t>(pcm >> 4);
  } else {
    aval = static_cast<uint8_t>((seg << 4) | ((pcm >> (seg + 3)) & 0x0F));
  }
  return static_cast<uint8_t>(aval ^ mask);
}

std::string RTSPAudioComponent::make_sdp_() const {
  char sdp[512];
  snprintf(sdp, sizeof(sdp),
           "v=0\r\n"
           "o=- 0 0 IN IP4 %s\r\n"
           "s=ESPHome RTSP Audio\r\n"
           "c=IN IP4 %s\r\n"
           "t=0 0\r\n"
           "m=audio 0 RTP/AVP %d\r\n"
           "a=rtpmap:%d %s/%d/1\r\n"
           "a=control:trackID=0\r\n",
           this->local_ip_().c_str(), this->local_ip_().c_str(), this->rtp_payload_type_, this->rtp_payload_type_, this->rtpmap_name_(), this->output_sample_rate_);
  return std::string(sdp);
}

void RTSPAudioComponent::send_rtsp_response_(int fd, int code, const char *reason, int cseq, const std::string &headers, const std::string &body) {
  char head[768];
  int len = snprintf(head, sizeof(head),
                     "RTSP/1.0 %d %s\r\n"
                     "CSeq: %d\r\n"
                     "%s"
                     "Content-Length: %u\r\n"
                     "\r\n",
                     code, reason, cseq, headers.c_str(), (unsigned) body.size());
  send(fd, head, len, 0);
  if (!body.empty()) send(fd, body.data(), body.size(), 0);
}

void RTSPAudioComponent::handle_rtsp_client_(int client_fd, int session_index) {
  while (this->running_) {
    std::string req;
    if (!this->read_rtsp_request_(client_fd, req)) break;

    int cseq = this->parse_cseq_(req);
    std::string first = req.substr(0, req.find('\n'));
    if (!first.empty() && first.back() == '\r') first.pop_back();
    if (this->debug_) ESP_LOGI(TAG, "RTSP[%d] request: %s", session_index, first.c_str());

    if (this->auth_enabled_ && req.rfind("OPTIONS", 0) != 0 && !this->request_authorized_(req)) {
      if (this->debug_) ESP_LOGI(TAG, "RTSP[%d] request rejected: missing or invalid Basic auth", session_index);
      this->send_auth_required_(client_fd, cseq);
      continue;
    }

    if (req.rfind("OPTIONS", 0) == 0) {
      this->send_rtsp_response_(client_fd, 200, "OK", cseq, "Public: OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN\r\n", "");
    } else if (req.rfind("DESCRIBE", 0) == 0) {
      std::string sdp = this->make_sdp_();
      this->send_rtsp_response_(client_fd, 200, "OK", cseq, "Content-Type: application/sdp\r\n", sdp);
    } else if (req.rfind("SETUP", 0) == 0) {
      if (req.find("RTP/AVP/TCP") != std::string::npos || req.find("interleaved") != std::string::npos) {
        this->send_rtsp_response_(client_fd, 461, "Unsupported Transport", cseq, "", "");
        continue;
      }
      int client_rtp = 0, client_rtcp = 0;
      if (!this->parse_client_ports_(req, &client_rtp, &client_rtcp)) {
        this->send_rtsp_response_(client_fd, 400, "Bad Request", cseq, "", "");
        continue;
      }

      ::sockaddr_in peer = {};
      socklen_t peer_len = sizeof(peer);
      getpeername(client_fd, (::sockaddr *) &peer, &peer_len);
      this->close_rtp_sockets_(session_index);

      int rtp_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
      int rtcp_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
      if (rtp_fd < 0 || rtcp_fd < 0) {
        if (rtp_fd >= 0) close(rtp_fd);
        if (rtcp_fd >= 0) close(rtcp_fd);
        this->send_rtsp_response_(client_fd, 500, "Internal Server Error", cseq, "", "");
        continue;
      }

      int sndbuf = 16 * 1024;
      setsockopt(rtp_fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

      ::sockaddr_in local = {};
      local.sin_family = AF_INET;
      local.sin_addr.s_addr = htonl(INADDR_ANY);
      local.sin_port = 0;
      bind(rtp_fd, (::sockaddr *) &local, sizeof(local));
      bind(rtcp_fd, (::sockaddr *) &local, sizeof(local));
      socklen_t llen = sizeof(local);
      getsockname(rtp_fd, (::sockaddr *) &local, &llen);
      int server_rtp_port = ntohs(local.sin_port);
      getsockname(rtcp_fd, (::sockaddr *) &local, &llen);
      int server_rtcp_port = ntohs(local.sin_port);

      xSemaphoreTake(this->sessions_mutex_, portMAX_DELAY);
      auto &sess = this->sessions_[session_index];
      sess.rtp_fd = rtp_fd;
      sess.rtcp_fd = rtcp_fd;
      sess.server_rtp_port = server_rtp_port;
      sess.server_rtcp_port = server_rtcp_port;
      sess.client_rtp_addr = peer;
      sess.client_rtcp_addr = peer;
      sess.client_rtp_addr.sin_port = htons(client_rtp);
      sess.client_rtcp_addr.sin_port = htons(client_rtcp);
      std::string session = sess.session_id;
      xSemaphoreGive(this->sessions_mutex_);

      char h[256];
      snprintf(h, sizeof(h),
               "Transport: RTP/AVP;unicast;client_port=%d-%d;server_port=%d-%d\r\n"
               "Session: %s\r\n",
               client_rtp, client_rtcp, server_rtp_port, server_rtcp_port, session.c_str());
      this->send_rtsp_response_(client_fd, 200, "OK", cseq, h, "");
      ESP_LOGI(TAG, "RTSP[%d] SETUP complete: client RTP/RTCP %d/%d, server RTP/RTCP %d/%d", session_index,
               client_rtp, client_rtcp, server_rtp_port, server_rtcp_port);
    } else if (req.rfind("PLAY", 0) == 0) {
      bool ok = false;
      uint16_t seq = 0;
      uint32_t ts = 0;
      std::string session;
      bool was_any_streaming = this->active_stream_count_() > 0;
      xSemaphoreTake(this->sessions_mutex_, portMAX_DELAY);
      if (session_index >= 0 && session_index < (int) this->sessions_.size()) {
        auto &sess = this->sessions_[session_index];
        ok = sess.allocated && sess.rtp_fd >= 0;
        if (ok) {
          sess.playing = true;
          seq = sess.rtp_sequence;
          ts = sess.rtp_timestamp;
          session = sess.session_id;
        }
      }
      xSemaphoreGive(this->sessions_mutex_);
      if (!ok) {
        this->send_rtsp_response_(client_fd, 454, "Session Not Found", cseq, "", "");
        continue;
      }
      if (!was_any_streaming && this->audio_buffer_ != nullptr) xStreamBufferReset(this->audio_buffer_);
      this->update_streaming_state_();
      if (this->rtp_task_handle_ == nullptr) {
        xTaskCreatePinnedToCore(&RTSPAudioComponent::rtp_task_trampoline_, "rtsp_audio_rtp", RTSP_TASK_STACK_BYTES, this, 6,
                                &this->rtp_task_handle_, 1);
      }
      char h[128];
      snprintf(h, sizeof(h), "Session: %s\r\nRTP-Info: url=trackID=0;seq=%u;rtptime=%" PRIu32 "\r\n", session.c_str(), seq, ts);
      this->send_rtsp_response_(client_fd, 200, "OK", cseq, h, "");
      ESP_LOGI(TAG, "RTSP[%d] PLAY; active streams=%d", session_index, this->active_stream_count_());
    } else if (req.rfind("TEARDOWN", 0) == 0) {
      this->send_rtsp_response_(client_fd, 200, "OK", cseq, "", "");
      break;
    } else {
      this->send_rtsp_response_(client_fd, 405, "Method Not Allowed", cseq, "", "");
    }
  }
}

void RTSPAudioComponent::rtp_task_() {
  const int input_samples_per_packet = std::max(80, (this->sample_rate_ * this->packet_ms_) / 1000);
  const int output_samples_per_packet = std::max(80, (this->output_sample_rate_ * this->packet_ms_) / 1000);
  const size_t bytes_per_output_sample = (this->codec_ == AudioCodec::L16) ? 2U : 1U;

  struct RtpTarget {
    int index;
    int fd;
    ::sockaddr_in addr;
    uint32_t ssrc;
    uint16_t sequence;
    uint32_t timestamp;
  };

  std::vector<uint8_t> input(input_samples_per_packet * 2);
  std::vector<uint8_t> packet(12 + output_samples_per_packet * bytes_per_output_sample);
  std::vector<RtpTarget> targets;
  targets.reserve(6);

  ESP_LOGI(TAG, "RTP task started: codec=%s input=%d Hz output=%d Hz, %d ms packets, max_clients=%d",
           this->rtpmap_name_(), this->sample_rate_, this->output_sample_rate_, this->packet_ms_, this->max_clients_);

  while (this->running_) {
    this->rtp_stack_free_ = uxTaskGetStackHighWaterMark(nullptr);
    targets.clear();
    xSemaphoreTake(this->sessions_mutex_, portMAX_DELAY);
    for (int i = 0; i < (int) this->sessions_.size(); i++) {
      const auto &sess = this->sessions_[i];
      if (sess.allocated && sess.playing && sess.rtp_fd >= 0) {
        targets.push_back(RtpTarget{i, sess.rtp_fd, sess.client_rtp_addr, sess.ssrc, sess.rtp_sequence, sess.rtp_timestamp});
      }
    }
    xSemaphoreGive(this->sessions_mutex_);

    if (targets.empty()) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    size_t bytes_read = 0;
    if (this->audio_buffer_ != nullptr) {
      const uint32_t deadline = now_ms() + (uint32_t) std::max(50, this->packet_ms_ * 4);
      while (bytes_read < input.size() && this->running_ && this->streaming_) {
        // The INMP441 source commonly supplies 256-byte chunks every ~8 ms.
        // A 2 ms follow-up wait turns each chunk into its own undersized RTP
        // datagram, vastly increasing packet rate and exhausting Wi-Fi pbufs.
        // Wait long enough to accumulate the configured packet duration while
        // the outer deadline still bounds a stopped microphone.
        TickType_t wait_ticks = bytes_read == 0 ? pdMS_TO_TICKS(std::max(50, this->packet_ms_ * 4)) : pdMS_TO_TICKS(10);
        size_t n = xStreamBufferReceive(this->audio_buffer_, input.data() + bytes_read, input.size() - bytes_read, wait_ticks);
        bytes_read += n;
        if (n == 0 || now_ms() >= deadline) break;
      }
    }
    this->last_bytes_read_ = (uint32_t) bytes_read;
    if (bytes_read == 0) {
      this->i2s_empty_reads_++;
      continue;
    }
    this->i2s_reads_++;

    int input_samples = bytes_read / 2;
    if (input_samples <= 0) continue;
    if (input_samples > input_samples_per_packet) input_samples = input_samples_per_packet;

    int output_samples = (int) (((int64_t) input_samples * this->output_sample_rate_) / this->sample_rate_);
    if (output_samples <= 0) output_samples = 1;
    if (output_samples > output_samples_per_packet) output_samples = output_samples_per_packet;

    int32_t min_sample = 32767;
    int32_t max_sample = -32768;
    int32_t peak_sample = 0;

    packet[0] = 0x80;
    packet[1] = (uint8_t) (this->rtp_payload_type_ & 0x7F);

    size_t payload_len = 0;
    for (int out_i = 0; out_i < output_samples; out_i++) {
      int in_i = (int) (((int64_t) out_i * this->sample_rate_) / this->output_sample_rate_);
      if (in_i >= input_samples) in_i = input_samples - 1;

      int16_t v;
      memcpy(&v, &input[in_i * 2], 2);
      if (v < min_sample) min_sample = v;
      if (v > max_sample) max_sample = v;
      int32_t abs_sample = v < 0 ? -(int32_t) v : (int32_t) v;
      if (abs_sample > peak_sample) peak_sample = abs_sample;
      if (abs_sample >= 32767) this->clipped_samples_++;

      if (this->codec_ == AudioCodec::PCMU) {
        packet[12 + payload_len++] = linear_to_ulaw_(v);
      } else if (this->codec_ == AudioCodec::PCMA) {
        packet[12 + payload_len++] = linear_to_alaw_(v);
      } else {
        uint16_t be = htons((uint16_t) v);
        memcpy(&packet[12 + payload_len], &be, 2);
        payload_len += 2;
      }
    }

    this->last_min_ = min_sample;
    this->last_max_ = max_sample;
    this->last_peak_ = peak_sample;

    for (const auto &target : targets) {
      packet[2] = (uint8_t) (target.sequence >> 8);
      packet[3] = (uint8_t) (target.sequence & 0xFF);
      packet[4] = (uint8_t) (target.timestamp >> 24);
      packet[5] = (uint8_t) (target.timestamp >> 16);
      packet[6] = (uint8_t) (target.timestamp >> 8);
      packet[7] = (uint8_t) (target.timestamp & 0xFF);
      packet[8] = (uint8_t) (target.ssrc >> 24);
      packet[9] = (uint8_t) (target.ssrc >> 16);
      packet[10] = (uint8_t) (target.ssrc >> 8);
      packet[11] = (uint8_t) (target.ssrc & 0xFF);

      int sent = sendto(target.fd, packet.data(), 12 + payload_len, 0, (::sockaddr *) &target.addr, sizeof(::sockaddr_in));
      if (sent < 0) {
        int err = errno;

        // A client may TEARDOWN/disconnect between taking the target snapshot
        // above and this sendto(). In that case the RTP socket has already
        // been closed by the RTSP control task and lwIP returns EBADF. This is
        // expected during normal disconnects, so don't count or log it as a
        // streaming problem.
        if (err == EBADF || err == ENOTSOCK) {
          ESP_LOGD(TAG, "RTP session %d closed while sending; ignoring final packet", target.index);
          continue;
        }

        this->rtp_send_errors_++;
        uint32_t t = now_ms();
        if (t - this->last_send_error_log_ms_ > 2000) {
          this->last_send_error_log_ms_ = t;
          ESP_LOGW(TAG, "RTP packet send failed for session %d: %s; dropping packet", target.index, strerror(err));
        }
        // xStreamBufferReceive() already consumed this packet. Advancing the
        // RTP clock makes loss an explicit gap; otherwise the next, different
        // audio would reuse this packet's sequence number and timestamp.
        xSemaphoreTake(this->sessions_mutex_, portMAX_DELAY);
        if (target.index >= 0 && target.index < (int) this->sessions_.size()) {
          auto &sess = this->sessions_[target.index];
          if (sess.allocated && sess.playing) {
            sess.rtp_sequence++;
            sess.rtp_timestamp += output_samples;
          }
        }
        xSemaphoreGive(this->sessions_mutex_);
        continue;
      }

      xSemaphoreTake(this->sessions_mutex_, portMAX_DELAY);
      if (target.index >= 0 && target.index < (int) this->sessions_.size()) {
        auto &sess = this->sessions_[target.index];
        if (sess.allocated && sess.playing) {
          sess.rtp_sequence++;
          sess.rtp_timestamp += output_samples;
        }
      }
      xSemaphoreGive(this->sessions_mutex_);
    }

    this->rtp_packets_++;
    vTaskDelay(1);
  }
  ESP_LOGI(TAG, "RTP task stopped");
}

void RTSPAudioComponent::close_rtp_sockets_(int index) {
  if (index < 0 || index >= (int) this->sessions_.size()) return;

  int rtp_fd = -1;
  int rtcp_fd = -1;
  xSemaphoreTake(this->sessions_mutex_, portMAX_DELAY);
  auto &sess = this->sessions_[index];
  sess.playing = false;
  rtp_fd = sess.rtp_fd;
  rtcp_fd = sess.rtcp_fd;
  sess.rtp_fd = -1;
  sess.rtcp_fd = -1;
  sess.server_rtp_port = 0;
  sess.server_rtcp_port = 0;
  xSemaphoreGive(this->sessions_mutex_);

  if (rtp_fd >= 0) close(rtp_fd);
  if (rtcp_fd >= 0) close(rtcp_fd);
  this->update_streaming_state_();
}

void RTSPAudioComponent::close_all_client_sessions_() {
  for (int i = 0; i < (int) this->sessions_.size(); i++) {
    if (this->sessions_[i].allocated) {
      int fd = -1;
      xSemaphoreTake(this->sessions_mutex_, portMAX_DELAY);
      fd = this->sessions_[i].control_fd;
      xSemaphoreGive(this->sessions_mutex_);
      this->close_rtp_sockets_(i);
      if (fd >= 0) {
        shutdown(fd, SHUT_RDWR);
        close(fd);
      }
    }
  }
}

std::string RTSPAudioComponent::local_ip_() const {
  esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (netif == nullptr) return "0.0.0.0";
  esp_netif_ip_info_t ip = {};
  if (esp_netif_get_ip_info(netif, &ip) != ESP_OK) return "0.0.0.0";
  char buf[16];
  snprintf(buf, sizeof(buf), IPSTR, IP2STR(&ip.ip));
  return std::string(buf);
}

void RTSPAudioComponent::loop() {
  uint32_t now = now_ms();

  // Only run the microphone while an RTSP client is actually in PLAY state,
  // so an idle server doesn't keep the mic (and whatever it owns) running for
  // nothing. mic_source_->start()/stop() are only ever called from here (the
  // main ESPHome loop task), never from the RTSP/RTP tasks directly -- the
  // RTSP task just flips the streaming_ atomic and this polls it.
  if (this->mic_source_ != nullptr) {
    bool want_running = this->streaming_.load();
    bool mic_running = this->mic_source_->is_running();
    if (want_running && !mic_running) {
      ESP_LOGD(TAG, "RTSP playback active, starting microphone capture.");
      this->mic_source_->start();
    } else if (!want_running && mic_running) {
      ESP_LOGD(TAG, "RTSP playback stopped, stopping microphone capture.");
      this->mic_source_->stop();
    }
  }

  if (this->debug_ && this->status_interval_ms_ > 0 && now - this->last_status_ms_ >= this->status_interval_ms_) {
    this->last_status_ms_ = now;
    this->log_status_("periodic");
  }
}

void RTSPAudioComponent::log_status_(const char *reason) {
  ESP_LOGI(TAG,
           "RTSP native status [%s]: started=%s mic_running=%s clients=%d/%d streams=%d ip=%s port=%d free_heap=%u "
           "codec=%s/%d packets=%u send_err=%u clip=%u reads=%u empty=%u drop=%u bytes=%u stack_free=%u/%u peak=%d min=%d max=%d last_error=%s",
           reason, YESNO(this->started_.load()),
           YESNO(this->mic_source_ != nullptr && this->mic_source_->is_running()), this->active_client_count_(), this->max_clients_,
           this->active_stream_count_(), this->local_ip_().c_str(), this->port_, (unsigned) esp_get_free_heap_size(),
           this->rtpmap_name_(), this->output_sample_rate_, (unsigned) this->rtp_packets_.load(), (unsigned) this->rtp_send_errors_.load(),
           (unsigned) this->clipped_samples_.load(), (unsigned) this->i2s_reads_.load(), (unsigned) this->i2s_empty_reads_.load(), (unsigned) this->dropped_bytes_.load(), (unsigned) this->last_bytes_read_.load(),
           (unsigned) this->server_stack_free_.load(), (unsigned) this->rtp_stack_free_.load(),
           (int) this->last_peak_.load(), (int) this->last_min_.load(), (int) this->last_max_.load(), this->last_error_.c_str());
}

void RTSPAudioComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Native RTSP Audio (microphone-source) v5-multiclient");
  ESP_LOGCONFIG(TAG, "  Port: %d", this->port_);
  ESP_LOGCONFIG(TAG, "  Microphone sample rate: %d Hz", this->sample_rate_);
  ESP_LOGCONFIG(TAG, "  Encoder codec: %s", this->codec_name_());
  ESP_LOGCONFIG(TAG, "  RTP output: %s/%d/1 payload_type=%d", this->rtpmap_name_(), this->output_sample_rate_, this->rtp_payload_type_);
  ESP_LOGCONFIG(TAG, "  Microphone channel index: %u", (unsigned) this->channel_);
  ESP_LOGCONFIG(TAG, "  Gain factor: %d", (int) this->gain_factor_);
  ESP_LOGCONFIG(TAG, "  Packet duration: %d ms", this->packet_ms_);
  ESP_LOGCONFIG(TAG, "  Audio buffer: %d ms", this->buffer_ms_);
  ESP_LOGCONFIG(TAG, "  Max clients: %d", this->max_clients_);
  ESP_LOGCONFIG(TAG, "  Authentication: %s", YESNO(this->auth_enabled_));
  if (this->auth_enabled_) {
    ESP_LOGCONFIG(TAG, "  Auth realm: %s", this->auth_realm_.c_str());
    ESP_LOGCONFIG(TAG, "  Auth username: %s", this->auth_username_.c_str());
  }
  ESP_LOGCONFIG(TAG, "  Debug: %s", YESNO(this->debug_));
  ESP_LOGCONFIG(TAG, "  Status interval: %" PRIu32 " ms", this->status_interval_ms_);
  ESP_LOGCONFIG(TAG, "  Runtime started=%s client=%s streaming=%s ip=%s last_error=%s", YESNO(this->started_.load()),
                YESNO(this->client_connected_.load()), YESNO(this->streaming_.load()), this->local_ip_().c_str(),
                this->last_error_.c_str());
}

void RTSPAudioComponent::stop_server_() {
  this->running_ = false;
  this->streaming_ = false;
  this->close_all_client_sessions_();
  if (this->server_fd_ >= 0) {
    shutdown(this->server_fd_, SHUT_RDWR);
    close(this->server_fd_);
    this->server_fd_ = -1;
  }
}

void RTSPAudioComponent::on_shutdown() {
  ESP_LOGI(TAG, "Shutting down native RTSP audio");
  this->stop_server_();
  if (this->mic_source_ != nullptr && this->mic_source_->is_running()) {
    this->mic_source_->stop();
  }
  if (this->audio_buffer_ != nullptr) {
    vStreamBufferDelete(this->audio_buffer_);
    this->audio_buffer_ = nullptr;
  }
}

}  // namespace rtsp_audio
}  // namespace esphome

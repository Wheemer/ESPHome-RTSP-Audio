#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace esphome::rtsp_audio::internal {

// ESPHome supplies standard I2S samples in host-endian, 32-bit PCM words.
// INMP441 data is left-aligned in that word, so retaining the high 16 bits
// produces the signed PCM16 representation used by RTP L16.
inline size_t pcm_s32_to_s16(const uint8_t *input, size_t input_bytes, uint8_t channels, uint8_t channel_index,
                             int16_t *output) {
  if (channels == 0 || channel_index >= channels)
    return 0;
  const size_t frames = input_bytes / (sizeof(int32_t) * channels);
  for (size_t i = 0; i < frames; ++i) {
    int32_t sample;
    std::memcpy(&sample, input + (i * channels + channel_index) * sizeof(sample), sizeof(sample));
    output[i] = static_cast<int16_t>(sample >> 16);
  }
  return frames;
}

}  // namespace esphome::rtsp_audio::internal

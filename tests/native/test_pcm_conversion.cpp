#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstring>

#include "pcm_conversion.h"

namespace esphome::rtsp_audio::internal {
namespace {

TEST(PcmConversionTest, KeepsTheMostSignificantSigned16Bits) {
  constexpr std::array<int32_t, 5> input_samples = {0x00000000, 0x00010000, -0x00010000, 0x7FFF0000,
                                                    static_cast<int32_t>(0x80000000)};
  std::array<uint8_t, sizeof(input_samples)> input_bytes{};
  std::array<int16_t, input_samples.size()> output{};
  std::memcpy(input_bytes.data(), input_samples.data(), input_bytes.size());

  EXPECT_EQ(input_samples.size(), pcm_s32_to_s16(input_bytes.data(), input_bytes.size(), output.data()));
  EXPECT_EQ(0, output[0]);
  EXPECT_EQ(1, output[1]);
  EXPECT_EQ(-1, output[2]);
  EXPECT_EQ(INT16_MAX, output[3]);
  EXPECT_EQ(INT16_MIN, output[4]);
}

TEST(PcmConversionTest, IgnoresAnIncompleteTrailingWord) {
  constexpr int32_t input_sample = 0x12340000;
  std::array<uint8_t, sizeof(input_sample) + 1> input{};
  int16_t output = 0;
  std::memcpy(input.data(), &input_sample, sizeof(input_sample));

  EXPECT_EQ(1U, pcm_s32_to_s16(input.data(), input.size(), &output));
  EXPECT_EQ(0x1234, output);
}

}  // namespace
}  // namespace esphome::rtsp_audio::internal

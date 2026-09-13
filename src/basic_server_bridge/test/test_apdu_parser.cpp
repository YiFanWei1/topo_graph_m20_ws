#include "basic_server_bridge/apdu_parser.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

using basic_server_bridge::ApduParser;

TEST(ApduParser, ParsesLittleEndianHeaderAndJsonPayload)
{
  const std::string payload = "{\"Type\":1002,\"Command\":6}";
  std::vector<uint8_t> frame(ApduParser::kHeaderSize + payload.size(), 0);
  frame[0] = 0xeb;
  frame[1] = 0x91;
  frame[2] = 0xeb;
  frame[3] = 0x90;
  frame[4] = static_cast<uint8_t>(payload.size());
  frame[5] = 0;
  frame[6] = 0x34;
  frame[7] = 0x12;
  frame[8] = ApduParser::kJsonFormat;
  std::copy(payload.begin(), payload.end(), frame.begin() + ApduParser::kHeaderSize);

  std::string error;
  const auto parsed = ApduParser::parse(frame.data(), frame.size(), &error);
  ASSERT_TRUE(parsed.has_value()) << error;
  EXPECT_EQ(parsed->message_id, 0x1234);
  EXPECT_EQ(parsed->asdu_format, ApduParser::kJsonFormat);
  EXPECT_EQ(parsed->asdu, payload);
}

TEST(ApduParser, ExtractsOneFrameFromTcpBuffer)
{
  const std::string payload = "{}";
  std::vector<uint8_t> buffer(ApduParser::kHeaderSize + payload.size(), 0);
  buffer[0] = 0xeb;
  buffer[1] = 0x91;
  buffer[2] = 0xeb;
  buffer[3] = 0x90;
  buffer[4] = static_cast<uint8_t>(payload.size());
  buffer[8] = ApduParser::kJsonFormat;
  std::copy(payload.begin(), payload.end(), buffer.begin() + ApduParser::kHeaderSize);
  buffer.push_back(0xff);

  const auto parsed = ApduParser::extract(buffer);
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->asdu, payload);
  ASSERT_EQ(buffer.size(), 1U);
  EXPECT_EQ(buffer.front(), 0xff);
}

TEST(ApduParser, RejectsInvalidSyncBytes)
{
  std::vector<uint8_t> frame(ApduParser::kHeaderSize, 0);
  std::string error;
  EXPECT_FALSE(ApduParser::parse(frame.data(), frame.size(), &error).has_value());
  EXPECT_NE(error.find("synchronization"), std::string::npos);
}

#include "basic_server_bridge/apdu_parser.hpp"

#include <algorithm>
#include <array>

namespace basic_server_bridge
{

namespace
{
constexpr uint8_t kSync0 = 0xeb;
constexpr uint8_t kSync1 = 0x91;
constexpr uint8_t kSync2 = 0xeb;
constexpr uint8_t kSync3 = 0x90;
constexpr std::size_t kLengthOffset = 4;
constexpr std::size_t kMessageIdOffset = 6;
constexpr std::size_t kFormatOffset = 8;
constexpr std::size_t kMaxAsduSize = 65535;
constexpr std::array<uint8_t, 4> kSyncBytes{kSync0, kSync1, kSync2, kSync3};
}

void ApduParser::setError(std::string *error, const std::string &message)
{
  if (error != nullptr) {
    *error = message;
  }
}

bool ApduParser::hasSyncBytes(const uint8_t *data, std::size_t size)
{
  return size >= 4 && data[0] == kSync0 && data[1] == kSync1 &&
         data[2] == kSync2 && data[3] == kSync3;
}

std::optional<ApduFrame> ApduParser::parse(
  const uint8_t *data, std::size_t size, std::string *error)
{
  if (data == nullptr) {
    setError(error, "APDU data is null");
    return std::nullopt;
  }
  if (size < kHeaderSize) {
    setError(error, "APDU is shorter than the 16-byte header");
    return std::nullopt;
  }
  if (!hasSyncBytes(data, size)) {
    setError(error, "invalid APDU synchronization bytes");
    return std::nullopt;
  }

  const std::size_t asdu_size =
    static_cast<std::size_t>(data[kLengthOffset]) |
    (static_cast<std::size_t>(data[kLengthOffset + 1]) << 8);
  if (asdu_size > kMaxAsduSize || size != kHeaderSize + asdu_size) {
    setError(error, "APDU length does not match the received datagram");
    return std::nullopt;
  }

  ApduFrame frame;
  frame.message_id = static_cast<uint16_t>(data[kMessageIdOffset]) |
                     (static_cast<uint16_t>(data[kMessageIdOffset + 1]) << 8);
  frame.asdu_format = data[kFormatOffset];
  frame.asdu.assign(
    reinterpret_cast<const char *>(data + kHeaderSize), asdu_size);
  return frame;
}

std::optional<ApduFrame> ApduParser::extract(
  std::vector<uint8_t> &buffer, std::string *error)
{
  if (buffer.size() < kHeaderSize) {
    return std::nullopt;
  }

  auto sync = std::search(
    buffer.begin(), buffer.end(), kSyncBytes.begin(), kSyncBytes.end());
  if (sync == buffer.end()) {
    buffer.clear();
    setError(error, "APDU synchronization bytes not found in TCP stream");
    return std::nullopt;
  }
  if (sync != buffer.begin()) {
    buffer.erase(buffer.begin(), sync);
  }
  if (buffer.size() < kHeaderSize) {
    return std::nullopt;
  }

  const std::size_t asdu_size =
    static_cast<std::size_t>(buffer[kLengthOffset]) |
    (static_cast<std::size_t>(buffer[kLengthOffset + 1]) << 8);
  const std::size_t frame_size = kHeaderSize + asdu_size;
  if (buffer.size() < frame_size) {
    return std::nullopt;
  }

  auto frame = parse(buffer.data(), frame_size, error);
  buffer.erase(buffer.begin(), buffer.begin() + frame_size);
  return frame;
}

}  // namespace basic_server_bridge

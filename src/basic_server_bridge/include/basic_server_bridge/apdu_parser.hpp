#ifndef BASIC_SERVER_BRIDGE__APDU_PARSER_HPP_
#define BASIC_SERVER_BRIDGE__APDU_PARSER_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace basic_server_bridge
{

struct ApduFrame
{
  uint16_t message_id{0};
  uint8_t asdu_format{0};
  std::string asdu;
};

class ApduParser
{
public:
  static constexpr std::size_t kHeaderSize = 16;
  static constexpr uint8_t kJsonFormat = 0x01;

  static std::optional<ApduFrame> parse(
    const uint8_t *data, std::size_t size, std::string *error = nullptr);

  // Extract one complete frame from a TCP receive buffer.
  static std::optional<ApduFrame> extract(std::vector<uint8_t> &buffer, std::string *error = nullptr);

private:
  static bool hasSyncBytes(const uint8_t *data, std::size_t size);
  static void setError(std::string *error, const std::string &message);
};

}  // namespace basic_server_bridge

#endif  // BASIC_SERVER_BRIDGE__APDU_PARSER_HPP_

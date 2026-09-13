#ifndef BASIC_SERVER_BRIDGE__BASIC_SERVER_CLIENT_HPP_
#define BASIC_SERVER_BRIDGE__BASIC_SERVER_CLIENT_HPP_

#include "basic_server_bridge/apdu_parser.hpp"

#include <chrono>
#include <cstdint>
#include <netinet/in.h>
#include <string>
#include <vector>

namespace basic_server_bridge
{

class BasicServerClient
{
public:
  enum class Transport
  {
    Udp,
    Tcp
  };

  BasicServerClient(
    std::string server_ip, uint16_t port, Transport transport, uint16_t local_udp_port = 0);
  ~BasicServerClient();

  BasicServerClient(const BasicServerClient &) = delete;
  BasicServerClient &operator=(const BasicServerClient &) = delete;

  bool open(std::string *error = nullptr);
  void close();
  bool isOpen() const;
  uint16_t localPort() const;

  bool sendHeartbeat(std::string *error = nullptr);
  // Sends a JSON ASDU and returns the APDU message ID used for request/response matching.
  bool sendJson(const std::string &asdu, uint16_t *message_id, std::string *error = nullptr);
  static std::string currentLocalTime();
  std::optional<ApduFrame> receive(
    std::chrono::milliseconds timeout, std::string *error = nullptr);

private:
  bool sendBytes(const std::vector<uint8_t> &bytes, std::string *error);
  bool waitReadable(std::chrono::milliseconds timeout, std::string *error);
  std::optional<ApduFrame> receiveUdp(std::string *error);
  std::optional<ApduFrame> receiveTcp(std::string *error);
  std::vector<uint8_t> makeApdu(const std::string &asdu, uint16_t *message_id = nullptr);
  static void setError(std::string *error, const std::string &message);

  std::string server_ip_;
  uint16_t port_;
  Transport transport_;
  uint16_t local_udp_port_;
  int socket_fd_{-1};
  sockaddr_in server_addr_{};
  uint16_t next_message_id_{0};
  std::vector<uint8_t> tcp_buffer_;
};

}  // namespace basic_server_bridge

#endif  // BASIC_SERVER_BRIDGE__BASIC_SERVER_CLIENT_HPP_

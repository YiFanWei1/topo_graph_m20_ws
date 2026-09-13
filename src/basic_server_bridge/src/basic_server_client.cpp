#include "basic_server_bridge/basic_server_client.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <utility>

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
constexpr uint16_t kDefaultLocalPort = 0;
}

BasicServerClient::BasicServerClient(
  std::string server_ip, uint16_t port, Transport transport, uint16_t local_udp_port)
: server_ip_(std::move(server_ip)),
  port_(port),
  transport_(transport),
  local_udp_port_(local_udp_port == 0 ? kDefaultLocalPort : local_udp_port)
{
}

BasicServerClient::~BasicServerClient()
{
  close();
}

void BasicServerClient::setError(std::string *error, const std::string &message)
{
  if (error != nullptr) {
    *error = message;
  }
}

bool BasicServerClient::open(std::string *error)
{
  close();

  socket_fd_ = ::socket(AF_INET, transport_ == Transport::Udp ? SOCK_DGRAM : SOCK_STREAM, 0);
  if (socket_fd_ < 0) {
    setError(error, std::string("socket: ") + std::strerror(errno));
    return false;
  }

  sockaddr_in server_addr{};
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port_);
  if (::inet_pton(AF_INET, server_ip_.c_str(), &server_addr.sin_addr) != 1) {
    setError(error, "invalid server IPv4 address: " + server_ip_);
    close();
    return false;
  }
  server_addr_ = server_addr;

  if (transport_ == Transport::Udp) {
    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(local_udp_port_);
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(socket_fd_, reinterpret_cast<sockaddr *>(&local_addr), sizeof(local_addr)) < 0) {
      setError(error, std::string("bind: ") + std::strerror(errno));
      close();
      return false;
    }
  }

  if (transport_ == Transport::Tcp) {
    if (::connect(socket_fd_, reinterpret_cast<sockaddr *>(&server_addr), sizeof(server_addr)) < 0) {
      setError(error, std::string("connect: ") + std::strerror(errno));
      close();
      return false;
    }
  }

  tcp_buffer_.clear();
  return true;
}

void BasicServerClient::close()
{
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
  tcp_buffer_.clear();
}

bool BasicServerClient::isOpen() const
{
  return socket_fd_ >= 0;
}

uint16_t BasicServerClient::localPort() const
{
  if (!isOpen()) {
    return 0;
  }
  sockaddr_in local_addr{};
  socklen_t address_size = sizeof(local_addr);
  if (::getsockname(socket_fd_, reinterpret_cast<sockaddr *>(&local_addr), &address_size) < 0) {
    return 0;
  }
  return ntohs(local_addr.sin_port);
}

std::string BasicServerClient::currentLocalTime()
{
  const std::time_t now = std::time(nullptr);
  std::tm local_time{};
  localtime_r(&now, &local_time);
  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S");
  return stream.str();
}

std::vector<uint8_t> BasicServerClient::makeApdu(const std::string &asdu, uint16_t *message_id)
{
  if (asdu.size() > 65535) {
    return {};
  }
  const auto asdu_size = static_cast<uint16_t>(asdu.size());
  const auto frame_message_id = next_message_id_++;
  if (message_id != nullptr) {
    *message_id = frame_message_id;
  }
  std::vector<uint8_t> apdu(ApduParser::kHeaderSize + asdu.size(), 0);
  apdu[0] = kSync0;
  apdu[1] = kSync1;
  apdu[2] = kSync2;
  apdu[3] = kSync3;
  apdu[kLengthOffset] = static_cast<uint8_t>(asdu_size & 0xff);
  apdu[kLengthOffset + 1] = static_cast<uint8_t>((asdu_size >> 8) & 0xff);
  apdu[kMessageIdOffset] = static_cast<uint8_t>(frame_message_id & 0xff);
  apdu[kMessageIdOffset + 1] = static_cast<uint8_t>((frame_message_id >> 8) & 0xff);
  apdu[kFormatOffset] = ApduParser::kJsonFormat;
  std::copy(asdu.begin(), asdu.end(), apdu.begin() + ApduParser::kHeaderSize);
  return apdu;
}

bool BasicServerClient::sendBytes(const std::vector<uint8_t> &bytes, std::string *error)
{
  if (!isOpen()) {
    setError(error, "socket is not open");
    return false;
  }

  std::size_t sent = 0;
  while (sent < bytes.size()) {
    const ssize_t result = transport_ == Transport::Udp ?
      ::sendto(
      socket_fd_, bytes.data() + sent, bytes.size() - sent, 0,
      reinterpret_cast<const sockaddr *>(&server_addr_), sizeof(server_addr_)) :
      ::send(socket_fd_, bytes.data() + sent, bytes.size() - sent, MSG_NOSIGNAL);
    if (result < 0 && errno == EINTR) {
      continue;
    }
    if (result <= 0) {
      setError(error, std::string("send: ") + std::strerror(errno));
      return false;
    }
    sent += static_cast<std::size_t>(result);
  }
  return true;
}

bool BasicServerClient::sendHeartbeat(std::string *error)
{
  const std::string asdu =
    "{\"PatrolDevice\":{"
    "\"Type\":100,\"Command\":100,\"Time\":\"" +
    currentLocalTime() + "\",\"Items\":{}}}";
  const auto apdu = makeApdu(asdu);
  if (apdu.empty()) {
    setError(error, "heartbeat ASDU is too large");
    return false;
  }
  return sendBytes(apdu, error);
}

bool BasicServerClient::sendJson(const std::string &asdu, uint16_t *message_id, std::string *error)
{
  const auto apdu = makeApdu(asdu, message_id);
  if (apdu.empty()) {
    setError(error, "JSON ASDU is too large");
    return false;
  }
  return sendBytes(apdu, error);
}

bool BasicServerClient::waitReadable(std::chrono::milliseconds timeout, std::string *error)
{
  pollfd descriptor{socket_fd_, POLLIN, 0};
  const int timeout_ms = static_cast<int>(timeout.count());
  int result;
  do {
    result = ::poll(&descriptor, 1, timeout_ms);
  } while (result < 0 && errno == EINTR);

  if (result < 0) {
    setError(error, std::string("poll: ") + std::strerror(errno));
    return false;
  }
  if (result == 0) {
    return false;
  }
  if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    setError(error, "socket became unavailable");
    return false;
  }
  return (descriptor.revents & POLLIN) != 0;
}

std::optional<ApduFrame> BasicServerClient::receiveUdp(std::string *error)
{
  std::vector<uint8_t> buffer(65535 + ApduParser::kHeaderSize);
  sockaddr_in source_addr{};
  socklen_t source_size = sizeof(source_addr);
  const ssize_t received = ::recvfrom(
    socket_fd_, buffer.data(), buffer.size(), 0,
    reinterpret_cast<sockaddr *>(&source_addr), &source_size);
  if (received < 0) {
    if (errno == EINTR) {
      return std::nullopt;
    }
    setError(error, std::string("recv UDP: ") + std::strerror(errno));
    return std::nullopt;
  }
  return ApduParser::parse(buffer.data(), static_cast<std::size_t>(received), error);
}

std::optional<ApduFrame> BasicServerClient::receiveTcp(std::string *error)
{
  std::array<uint8_t, 4096> chunk{};
  const ssize_t received = ::recv(socket_fd_, chunk.data(), chunk.size(), 0);
  if (received == 0) {
    setError(error, "TCP peer closed the connection");
    return std::nullopt;
  }
  if (received < 0) {
    if (errno == EINTR) {
      return std::nullopt;
    }
    setError(error, std::string("recv TCP: ") + std::strerror(errno));
    return std::nullopt;
  }
  tcp_buffer_.insert(tcp_buffer_.end(), chunk.begin(), chunk.begin() + received);
  return ApduParser::extract(tcp_buffer_, error);
}

std::optional<ApduFrame> BasicServerClient::receive(
  std::chrono::milliseconds timeout, std::string *error)
{
  if (!isOpen()) {
    setError(error, "socket is not open");
    return std::nullopt;
  }
  if (transport_ == Transport::Tcp) {
    auto buffered_frame = ApduParser::extract(tcp_buffer_, error);
    if (buffered_frame.has_value()) {
      return buffered_frame;
    }
  }
  if (!waitReadable(timeout, error)) {
    return std::nullopt;
  }
  return transport_ == Transport::Udp ? receiveUdp(error) : receiveTcp(error);
}

}  // namespace basic_server_bridge

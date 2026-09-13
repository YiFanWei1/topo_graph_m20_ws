#include "basic_server_bridge/basic_server_client.hpp"

#include <glog/logging.h>
#include <json/json.h>

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>

#include "drdds/msg/gait.hpp"
#include "drdds/msg/motion_info.hpp"
#include "drdds/msg/motion_state.hpp"
#include "core/module_base.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"

namespace basic_server_bridge
{

class BasicServerBridgeNode : public rclcpp::Node, public ModuleBase
{
public:
  explicit BasicServerBridgeNode(const std::string &config_path)
  : Node("basic_server_bridge"),
    ModuleBase(config_path, "basic_server_bridge", "basic_server_bridge")
  {
    loadParameters();
    print_table();

    const auto transport = transport_ == "tcp" ? BasicServerClient::Transport::Tcp :
      BasicServerClient::Transport::Udp;
    const uint16_t port = static_cast<uint16_t>(
      transport == BasicServerClient::Transport::Tcp ? tcp_port_ : udp_port_);
    const auto local_port = static_cast<uint16_t>(local_udp_port_);
    client_ = std::make_unique<BasicServerClient>(server_ip_, port, transport, local_port);

    motion_state_publisher_ = this->create_publisher<drdds::msg::MotionState>(
      motion_state_status_topic_, 10);
    gait_publisher_ = this->create_publisher<drdds::msg::Gait>(gait_status_topic_, 10);
    motion_info_publisher_ = this->create_publisher<drdds::msg::MotionInfo>(motion_info_status_topic_, 10);
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 10);
    control_result_publisher_ = this->create_publisher<std_msgs::msg::String>(control_result_topic_, 10);

    motion_state_subscription_ = this->create_subscription<drdds::msg::MotionState>(
      motion_state_control_topic_, 10,
      [this](const drdds::msg::MotionState::SharedPtr message) {
        if (!isValidMotionState(message->data.state)) {
          publishControlResult("rejected motion_state: supported values are 0, 1, 2, 3, 4, 17");
          return;
        }
        enqueueCommand(2, 22, "motion_state", [state = message->data.state](Json::Value &items) {
          items["MotionParam"] = state;
        });
      });
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        setAxisCommand(message->linear.x, message->linear.y, message->angular.z, "cmd_vel_smoothed");
      });
    mode_subscription_ = this->create_subscription<std_msgs::msg::UInt32>(
      mode_control_topic_, 10,
      [this](const std_msgs::msg::UInt32::SharedPtr message) {
        if (message->data > 2U) {
          publishControlResult("rejected usage_mode: supported values are 0, 1, 2");
          return;
        }
        enqueueCommand(1101, 5, "usage_mode", [mode = message->data](Json::Value &items) {
          items["Mode"] = mode;
        });
      });

    if (heartbeat_hz_ <= 0.0) {
      heartbeat_hz_ = 1.0;
    }
    heartbeat_period_ = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / heartbeat_hz_));
    running_.store(true);
    worker_ = std::thread(&BasicServerBridgeNode::run, this);

    LOG(INFO) << "basic_server bridge started: " << server_ip_ << ":" << port << " over " <<
      (transport == BasicServerClient::Transport::Tcp ? "TCP" : "UDP");
  }

  ~BasicServerBridgeNode() override
  {
    running_.store(false);
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void loadParameters()
  {
    readParam("server_ip", server_ip_, std::string("10.21.31.103"));
    readParam("udp_port", udp_port_, 30000);
    readParam("tcp_port", tcp_port_, 30001);
    readParam("local_udp_port", local_udp_port_, 0);
    readParam("transport", transport_, std::string("udp"));
    readParam("heartbeat_hz", heartbeat_hz_, 1.0);
    readParam("motion_state_control_topic", motion_state_control_topic_, std::string("/m20/control/motion_state"));
    readParam("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel_smoothed"));
    readParam("mode_control_topic", mode_control_topic_, std::string("/m20/control/usage_mode"));
    readParam("motion_state_status_topic", motion_state_status_topic_, std::string("/m20/state/motion_state"));
    readParam("gait_status_topic", gait_status_topic_, std::string("/m20/state/gait"));
    readParam("motion_info_status_topic", motion_info_status_topic_, std::string("/m20/state/motion_info"));
    readParam("odometry_topic", odometry_topic_, std::string("/m20/state/odometry"));
    readParam("odom_frame_id", odom_frame_id_, std::string("odom"));
    readParam("base_frame_id", base_frame_id_, std::string("base_link"));
    readParam("odom_max_integration_dt_sec", odom_max_integration_dt_sec_, 0.2);
    readParam("control_result_topic", control_result_topic_, std::string("/m20/state/control_result"));
    readParam("axis_command_timeout_ms", axis_command_timeout_ms_, 300);
    readParam("network_disconnect_timeout_ms", network_disconnect_timeout_ms_, 3000);
    readParam("maximum_vx_mps", maximum_vx_mps_, 0.8);
    readParam("maximum_vy_mps", maximum_vy_mps_, 0.3);
    readParam("maximum_wz_radps", maximum_wz_radps_, 0.5);

    if (udp_port_ < 1 || udp_port_ > 65535) {
      LOG(WARNING) << "udp_port is out of range, using default 30000";
      udp_port_ = 30000;
    }
    if (tcp_port_ < 1 || tcp_port_ > 65535) {
      LOG(WARNING) << "tcp_port is out of range, using default 30001";
      tcp_port_ = 30001;
    }
    if (local_udp_port_ < 0 || local_udp_port_ > 65535) {
      LOG(WARNING) << "local_udp_port is out of range, using an ephemeral port";
      local_udp_port_ = 0;
    }
    if (transport_ != "udp" && transport_ != "tcp") {
      LOG(WARNING) << "transport must be 'udp' or 'tcp', using udp";
      transport_ = "udp";
    }
    if (axis_command_timeout_ms_ < 50) {
      axis_command_timeout_ms_ = 300;
    }
    if (network_disconnect_timeout_ms_ < 1000) {
      LOG(WARNING) << "network_disconnect_timeout_ms must be at least 1000; using 3000";
      network_disconnect_timeout_ms_ = 3000;
    }
    if (odom_max_integration_dt_sec_ <= 0.0) {
      LOG(WARNING) << "odom_max_integration_dt_sec must be positive; using 0.2";
      odom_max_integration_dt_sec_ = 0.2;
    }
    if (maximum_vx_mps_ <= 0.0 || maximum_vy_mps_ < 0.0 || maximum_wz_radps_ <= 0.0) {
      throw std::invalid_argument("M20 physical velocity limits are invalid");
    }
    LOG(INFO) << "parameters: server_ip=" << server_ip_ << " udp_port=" << udp_port_ <<
      " tcp_port=" << tcp_port_ << " local_udp_port=" << local_udp_port_ <<
      " transport=" << transport_ << " heartbeat_hz=" << heartbeat_hz_;
  }

  static bool getInteger(const Json::Value &object, const char *name, int &value)
  {
    if (!object.isObject() || !object.isMember(name) || !object[name].isInt()) {
      return false;
    }
    value = object[name].asInt();
    return true;
  }

  static bool getNumber(const Json::Value &object, const char *name, float &value)
  {
    if (!object.isObject() || !object.isMember(name) || !object[name].isNumeric()) {
      return false;
    }
    value = object[name].asFloat();
    return true;
  }

  static bool isValidMotionState(const int32_t state)
  {
    return state == 0 || state == 1 || state == 2 || state == 3 || state == 4 || state == 17;
  }

  struct QueuedCommand
  {
    int type;
    int command;
    std::string name;
    Json::Value items;
  };

  struct PendingCommand
  {
    int type;
    int command;
    std::string name;
  };

  struct AxisCommand
  {
    float x{0.0F};
    float y{0.0F};
    float yaw{0.0F};
  };

  struct AxisLimits
  {
    float x;
    float y;
    float yaw;
  };

  static AxisLimits axisLimitsForGait(const uint32_t gait)
  {
    switch (gait) {
      case 0x1001U:  // Standard-basic
      case 0x1003U:  // Standard-stairs
      case 0x3003U:  // Agile-stairs
        return AxisLimits{2.0F, 1.0F, 2.0F};
      case 0x3002U:  // Agile-flat
        return AxisLimits{2.0F, 1.0F, 1.5F};
      default:
        // Safe fallback matching the M20 RL runtime velocity profile (0x3003).
        return AxisLimits{2.0F, 1.0F, 2.0F};
    }
  }

  void enqueueCommand(
    int type, int command, const std::string &name,
    const std::function<void(Json::Value &)> &fill_items)
  {
    QueuedCommand queued{type, command, name, Json::Value(Json::objectValue)};
    fill_items(queued.items);
    std::lock_guard<std::mutex> lock(control_mutex_);
    command_queue_.push_back(std::move(queued));
  }

  void setAxisCommand(const double x_vel, const double y_vel, const double yaw_vel, const char *source)
  {
    if (!std::isfinite(x_vel) || !std::isfinite(y_vel) || !std::isfinite(yaw_vel))
    {
      publishControlResult(std::string("rejected ") + source + ": all velocity fields must be finite");
      return;
    }
    const auto limits = axisLimitsForGait(active_gait_.load());
    AxisCommand command;
    const double safe_x = std::clamp(x_vel, -maximum_vx_mps_, maximum_vx_mps_);
    const double safe_y = std::clamp(y_vel, -maximum_vy_mps_, maximum_vy_mps_);
    const double safe_yaw = std::clamp(yaw_vel, -maximum_wz_radps_, maximum_wz_radps_);
    command.x = std::clamp(static_cast<float>(safe_x / limits.x), -1.0F, 1.0F);
    command.y = std::clamp(static_cast<float>(safe_y / limits.y), -1.0F, 1.0F);
    command.yaw = std::clamp(static_cast<float>(safe_yaw / limits.yaw), -1.0F, 1.0F);
    QueuedCommand queued{2, 21, "axis", Json::Value(Json::objectValue)};
    queued.items["X"] = command.x;
    queued.items["Y"] = command.y;
    queued.items["Z"] = 0.0;
    queued.items["Roll"] = 0.0;
    queued.items["Pitch"] = 0.0;
    queued.items["Yaw"] = command.yaw;
    std::lock_guard<std::mutex> lock(control_mutex_);
    command_queue_.push_back(std::move(queued));
    last_axis_command_time_ = std::chrono::steady_clock::now();
    has_axis_command_ = true;
    axis_stop_sent_ = false;
  }

  void publishControlResult(const std::string &text, const bool write_log = true)
  {
    std_msgs::msg::String result;
    result.data = text;
    control_result_publisher_->publish(result);
    if (write_log) {LOG(INFO) << text;}
  }

  bool sendCommand(const QueuedCommand &command)
  {
    Json::Value device(Json::objectValue);
    device["Type"] = command.type;
    device["Command"] = command.command;
    device["Time"] = BasicServerClient::currentLocalTime();
    device["Items"] = command.items;
    Json::Value root(Json::objectValue);
    root["PatrolDevice"] = device;
    Json::StreamWriterBuilder writer;
    writer["indentation"] = "";
    uint16_t message_id = 0;
    std::string error;
    if (!client_->sendJson(Json::writeString(writer, root), &message_id, &error)) {
      publishControlResult("send failed: " + command.name + ": " + error);
      return false;
    }
    pending_commands_[message_id] = PendingCommand{command.type, command.command, command.name};
    std::ostringstream stream;
    stream << "sent " << command.name << " (type=" << command.type << ", command=" <<
      command.command << ", id=" << message_id << ")";
    publishControlResult(stream.str(), command.name != "axis");
    return true;
  }

  bool sendZeroAxisCommand(const char *reason)
  {
    if (!client_->isOpen()) {
      LOG(WARNING) << "cannot send zero-speed stop for " << reason << "; socket is closed";
      return false;
    }
    QueuedCommand command{2, 21, "axis_stop", Json::Value(Json::objectValue)};
    command.items["X"] = 0.0;
    command.items["Y"] = 0.0;
    command.items["Z"] = 0.0;
    command.items["Roll"] = 0.0;
    command.items["Pitch"] = 0.0;
    command.items["Yaw"] = 0.0;
    LOG(WARNING) << "sending zero-speed stop before " << reason;
    return sendCommand(command);
  }

  void processQueuedCommands()
  {
    std::deque<QueuedCommand> commands;
    {
      std::lock_guard<std::mutex> lock(control_mutex_);
      commands.swap(command_queue_);
    }
    for (const auto &command : commands) {
      sendCommand(command);
    }
  }

  void processAxisTimeout(const std::chrono::steady_clock::time_point now)
  {
    bool should_send = false;
    {
      std::lock_guard<std::mutex> lock(control_mutex_);
      if (has_axis_command_) {
        const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - last_axis_command_time_).count();
        if (age > axis_command_timeout_ms_ && !axis_stop_sent_) {
          axis_stop_sent_ = true;
          should_send = true;
        }
      }
    }
    if (should_send) {
      sendZeroAxisCommand("velocity command timeout");
    }
  }

  drdds::msg::MetaType makeHeader()
  {
    drdds::msg::MetaType header;
    header.frame_id = frame_id_++;
    const int64_t nanoseconds = this->now().nanoseconds();
    header.timestamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    header.timestamp.nsec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
    return header;
  }

  bool parseJson(const ApduFrame &frame, Json::Value &root)
  {
    if (frame.asdu_format != ApduParser::kJsonFormat) {
      LOG_EVERY_N(WARNING, 100) << "ignoring ASDU with unsupported format 0x" << std::hex <<
        static_cast<int>(frame.asdu_format) << std::dec <<
        "; send JSON heartbeats to request JSON status";
      return false;
    }

    Json::CharReaderBuilder builder;
    builder["collectComments"] = false;
    std::string errors;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    if (!reader->parse(frame.asdu.data(), frame.asdu.data() + frame.asdu.size(), &root, &errors)) {
      LOG_EVERY_N(WARNING, 100) << "invalid basic_server JSON: " << errors;
      return false;
    }
    return root.isObject();
  }

  void publishBasicStatus(const Json::Value &basic_status)
  {
    int motion_state = 0;
    int gait = 0;
    bool published_motion_state = false;
    bool published_gait = false;
    if (getInteger(basic_status, "MotionState", motion_state)) {
      cached_motion_state_ = motion_state;
      drdds::msg::MotionState message;
      message.header = makeHeader();
      message.data.state = motion_state;
      motion_state_publisher_->publish(message);
      published_motion_state = true;
    }
    if (getInteger(basic_status, "Gait", gait)) {
      cached_gait_ = gait;
      active_gait_.store(static_cast<uint32_t>(gait));
      drdds::msg::Gait message;
      message.header = makeHeader();
      message.data.gait = static_cast<uint32_t>(gait);
      gait_publisher_->publish(message);
      published_gait = true;
    }
    if (published_motion_state || published_gait) {
      LOG_EVERY_N(INFO, 50) << "published basic status: MotionState=" << cached_motion_state_ <<
        " Gait=" << cached_gait_;
    }
  }

  void publishMotionStatus(const Json::Value &motion_status)
  {
    drdds::msg::MotionInfo message;
    message.header = makeHeader();
    message.data.state.state = cached_motion_state_;
    message.data.gait.gait = static_cast<uint32_t>(cached_gait_);
    getNumber(motion_status, "LinearX", message.data.vel_x);
    getNumber(motion_status, "LinearY", message.data.vel_y);
    getNumber(motion_status, "OmegaZ", message.data.vel_yaw);
    getNumber(motion_status, "Height", message.data.height);
    getNumber(motion_status, "Payload", message.data.payload);
    getNumber(motion_status, "RemainMile", message.data.remain_mile);
    motion_info_publisher_->publish(message);
    publishOdometry(message);
    LOG_EVERY_N(INFO, 50) << "published motion info: vel_x=" << message.data.vel_x <<
      " vel_y=" << message.data.vel_y << " vel_yaw=" << message.data.vel_yaw;
  }

  void publishOdometry(const drdds::msg::MotionInfo &motion_info)
  {
    const auto now_steady = std::chrono::steady_clock::now();
    if (has_odom_timestamp_) {
      const double dt = std::chrono::duration<double>(now_steady - last_odom_timestamp_).count();
      if (dt > 0.0 && dt <= odom_max_integration_dt_sec_) {
        const double cos_yaw = std::cos(odom_yaw_);
        const double sin_yaw = std::sin(odom_yaw_);
        const double vel_x = motion_info.data.vel_x;
        const double vel_y = motion_info.data.vel_y;
        odom_x_ += (vel_x * cos_yaw - vel_y * sin_yaw) * dt;
        odom_y_ += (vel_x * sin_yaw + vel_y * cos_yaw) * dt;
        odom_yaw_ = std::atan2(
          std::sin(odom_yaw_ + motion_info.data.vel_yaw * dt),
          std::cos(odom_yaw_ + motion_info.data.vel_yaw * dt));
      } else if (dt > odom_max_integration_dt_sec_) {
        LOG(WARNING) << "odometry integration gap " << dt << " s exceeds limit; pose not advanced";
      }
    }
    last_odom_timestamp_ = now_steady;
    has_odom_timestamp_ = true;

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = this->now();
    odometry.header.frame_id = odom_frame_id_;
    odometry.child_frame_id = base_frame_id_;
    odometry.pose.pose.position.x = odom_x_;
    odometry.pose.pose.position.y = odom_y_;
    odometry.pose.pose.orientation.z = std::sin(odom_yaw_ * 0.5);
    odometry.pose.pose.orientation.w = std::cos(odom_yaw_ * 0.5);
    odometry.twist.twist.linear.x = motion_info.data.vel_x;
    odometry.twist.twist.linear.y = motion_info.data.vel_y;
    odometry.twist.twist.angular.z = motion_info.data.vel_yaw;
    odometry_publisher_->publish(odometry);
  }

  void handleFrame(const ApduFrame &frame)
  {
    Json::Value root;
    if (!parseJson(frame, root)) {
      return;
    }
    const Json::Value &device = root["PatrolDevice"];
    const Json::Value &items = device["Items"];
    int type = 0;
    int command = 0;
    if (!getInteger(device, "Type", type) || !getInteger(device, "Command", command)) {
      LOG_EVERY_N(WARNING, 100) << "ignoring basic_server ASDU without Type/Command";
      return;
    }
    if (!items.isObject()) {
      return;
    }
    if (type == 1002 && command == 6 && items.isMember("BasicStatus")) {
      publishBasicStatus(items["BasicStatus"]);
    } else if (type == 1002 && command == 4 && items.isMember("MotionStatus")) {
      publishMotionStatus(items["MotionStatus"]);
    } else {
      const auto pending = pending_commands_.find(frame.message_id);
      if (pending == pending_commands_.end()) {
        return;
      }
      const auto &request = pending->second;
      std::ostringstream stream;
      stream << "result " << request.name << " (id=" << frame.message_id << "): ";
      if (type != request.type || command != request.command) {
        stream << "mismatched response type=" << type << " command=" << command;
      } else {
        if (items.isMember("ErrorCode")) {
          stream << "error_code=" << items["ErrorCode"].asInt() << " ";
        }
        stream << (items.isMember("ErrorMessage") ? items["ErrorMessage"].asString() : "received");
      }
      pending_commands_.erase(pending);
      publishControlResult(stream.str());
    }
  }

  void run()
  {
    auto next_heartbeat = std::chrono::steady_clock::now();
    auto last_received = next_heartbeat;
    while (running_.load()) {
      if (!client_->isOpen()) {
      std::string error;
      if (!client_->open(&error)) {
          LOG_EVERY_N(WARNING, 5) << "basic_server connection failed: " << error;
          std::this_thread::sleep_for(std::chrono::seconds(1));
          continue;
        }
        LOG(INFO) << "basic_server socket ready; local transport port: " << client_->localPort();
        next_heartbeat = std::chrono::steady_clock::now();
        last_received = next_heartbeat;
      }

      const auto now = std::chrono::steady_clock::now();
      if (now >= next_heartbeat) {
        std::string error;
        if (!client_->sendHeartbeat(&error)) {
          LOG_EVERY_N(WARNING, 5) << "basic_server heartbeat failed: " << error;
          sendZeroAxisCommand("heartbeat failure");
          client_->close();
          continue;
        }
        VLOG(1) << "basic_server heartbeat sent";
        next_heartbeat = now + heartbeat_period_;
      }
      processQueuedCommands();
      processAxisTimeout(now);
      const auto receive_age = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_received).count();
      if (receive_age > network_disconnect_timeout_ms_) {
        LOG(WARNING) << "no basic_server response for " << receive_age << " ms; stopping and reconnecting";
        sendZeroAxisCommand("network response timeout");
        client_->close();
        continue;
      }

      std::string error;
      auto frame = client_->receive(std::chrono::milliseconds(20), &error);
      if (!frame.has_value()) {
        if (!error.empty() && error != "APDU synchronization bytes not found in TCP stream") {
          LOG_EVERY_N(WARNING, 5) << "basic_server receive failed: " << error;
          if (error.find("closed") != std::string::npos || error.find("unavailable") != std::string::npos) {
            sendZeroAxisCommand("network socket failure");
            client_->close();
          }
        }
        continue;
      }
      VLOG(1) << "received APDU: id=" << frame->message_id << " format=0x" << std::hex <<
        static_cast<int>(frame->asdu_format) << std::dec << " asdu_bytes=" << frame->asdu.size();
      last_received = std::chrono::steady_clock::now();
      handleFrame(*frame);
    }
    sendZeroAxisCommand("node shutdown");
    client_->close();
  }

  std::unique_ptr<BasicServerClient> client_;
  std::thread worker_;
  std::atomic<bool> running_{false};
  std::chrono::milliseconds heartbeat_period_{1000};
  std::string server_ip_;
  int udp_port_{30000};
  int tcp_port_{30001};
  int local_udp_port_{0};
  std::string transport_{"udp"};
  double heartbeat_hz_{1.0};
  std::string motion_state_control_topic_{"/m20/control/motion_state"};
  std::string cmd_vel_topic_{"/cmd_vel_smoothed"};
  std::string mode_control_topic_{"/m20/control/usage_mode"};
  std::string motion_state_status_topic_{"/m20/state/motion_state"};
  std::string gait_status_topic_{"/m20/state/gait"};
  std::string motion_info_status_topic_{"/m20/state/motion_info"};
  std::string odometry_topic_{"/m20/state/odometry"};
  std::string odom_frame_id_{"odom"};
  std::string base_frame_id_{"base_link"};
  std::string control_result_topic_{"/m20/state/control_result"};
  int axis_command_timeout_ms_{300};
  int network_disconnect_timeout_ms_{3000};
  double odom_max_integration_dt_sec_{0.2};
  double maximum_vx_mps_{0.8};
  double maximum_vy_mps_{0.3};
  double maximum_wz_radps_{0.5};
  uint64_t frame_id_{0};
  int cached_motion_state_{0};
  int cached_gait_{0};
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};
  std::chrono::steady_clock::time_point last_odom_timestamp_{};
  bool has_odom_timestamp_{false};
  std::atomic<uint32_t> active_gait_{0x3003U};
  std::mutex control_mutex_;
  std::deque<QueuedCommand> command_queue_;
  std::chrono::steady_clock::time_point last_axis_command_time_{};
  bool has_axis_command_{false};
  bool axis_stop_sent_{false};
  std::unordered_map<uint16_t, PendingCommand> pending_commands_;

  rclcpp::Publisher<drdds::msg::MotionState>::SharedPtr motion_state_publisher_;
  rclcpp::Publisher<drdds::msg::Gait>::SharedPtr gait_publisher_;
  rclcpp::Publisher<drdds::msg::MotionInfo>::SharedPtr motion_info_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_result_publisher_;
  rclcpp::Subscription<drdds::msg::MotionState>::SharedPtr motion_state_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr mode_subscription_;
};

}  // namespace basic_server_bridge

int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;

  int result = 0;
  std::string config_path;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--ros-args") {
      break;
    }
    if (!argument.empty() && argument.front() != '-') {
      config_path = argument;
      break;
    }
  }
  if (config_path.empty()) {
    LOG(INFO) << "no config file provided; using readParam defaults";
  } else {
    LOG(INFO) << "loading config file: " << config_path;
  }

  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<basic_server_bridge::BasicServerBridgeNode>(config_path));
  } catch (const std::exception &exception) {
    LOG(ERROR) << "basic_server_bridge stopped with exception: " << exception.what();
    result = 1;
  }
  rclcpp::shutdown();
  google::ShutdownGoogleLogging();
  return result;
}

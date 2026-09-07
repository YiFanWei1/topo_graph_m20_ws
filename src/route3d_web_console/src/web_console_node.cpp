#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "route3d_web_console/point_cloud_codec.hpp"
#include "route3d_web_console/process_manager.hpp"
#include "route3d_web_console/topology_store.hpp"

namespace fs = std::filesystem;
namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
using tcp = asio::ip::tcp;
using json = nlohmann::json;
using namespace std::chrono_literals;

namespace route3d_web_console
{
namespace
{

std::string expandUser(const std::string & raw)
{
  if (raw.rfind("~/", 0) != 0) {return raw;}
  const char * home = std::getenv("HOME");
  return home == nullptr ? raw : std::string(home) + raw.substr(1);
}

std::string replaceAll(std::string value, const std::string & key, const std::string & replacement)
{
  std::size_t offset = 0;
  while ((offset = value.find(key, offset)) != std::string::npos) {
    value.replace(offset, key.size(), replacement);
    offset += replacement.size();
  }
  return value;
}

std::string shellQuote(const std::string & value)
{
  std::string output = "'";
  for (const char c : value) {
    output += c == '\'' ? "'\\''" : std::string(1, c);
  }
  return output + "'";
}

std::string mimeType(const fs::path & path)
{
  const auto ext = path.extension().string();
  if (ext == ".html") {return "text/html; charset=utf-8";}
  if (ext == ".js") {return "text/javascript; charset=utf-8";}
  if (ext == ".css") {return "text/css; charset=utf-8";}
  if (ext == ".json") {return "application/json";}
  if (ext == ".svg") {return "image/svg+xml";}
  if (ext == ".png") {return "image/png";}
  return "application/octet-stream";
}

std::string readText(const fs::path & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {throw std::runtime_error("cannot read " + path.string());}
  return std::string(std::istreambuf_iterator<char>(input), {});
}

bool pathInside(const fs::path & path, const std::vector<fs::path> & roots)
{
  std::error_code error;
  const auto canonical = fs::weakly_canonical(path, error);
  if (error) {return false;}
  for (const auto & root : roots) {
    const auto allowed = fs::weakly_canonical(root, error);
    if (error) {continue;}
    auto p = canonical.begin();
    auto r = allowed.begin();
    for (; r != allowed.end() && p != canonical.end() && *r == *p; ++r, ++p) {}
    if (r == allowed.end()) {return true;}
  }
  return false;
}

}  // namespace

class WebSession : public std::enable_shared_from_this<WebSession>
{
public:
  using CommandCallback = std::function<void(const std::string &, const json &)>;
  using CloseCallback = std::function<void(const std::string &)>;

  WebSession(tcp::socket socket, std::string id, CommandCallback command, CloseCallback close)
  : ws_(std::in_place, std::move(socket)), id_(std::move(id)),
    command_(std::move(command)), close_(std::move(close)) {}

  const std::string & id() const {return id_;}
  void start(http::request<http::string_body> request)
  {
    thread_ = std::thread([self = shared_from_this(), request = std::move(request)]() mutable {
      self->run(std::move(request));
    });
    thread_.detach();
  }
  void enqueueText(std::string text)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (texts_.size() >= 128U) {texts_.pop_front();}
    texts_.push_back(std::move(text));
  }
  void enqueueBinary(std::vector<std::uint8_t> data, const bool replace_live)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (replace_live) {
      latest_live_ = std::move(data);
      return;
    }
    binary_bytes_ += data.size();
    while (!binaries_.empty() && binary_bytes_ > 12U * 1024U * 1024U) {
      binary_bytes_ -= binaries_.front().size();
      binaries_.pop_front();
    }
    binaries_.push_back(std::move(data));
  }
  void requestStop() {stopping_.store(true);}

private:
  void run(http::request<http::string_body> request)
  {
    beast::error_code error;
    ws_->set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
    ws_->accept(request, error);
    if (error) {ws_.reset(); close_(id_); return;}
    while (!stopping_.load()) {
      flush();
      const auto available = ws_->next_layer().available(error);
      if (error) {break;}
      if (available == 0U) {
        std::this_thread::sleep_for(10ms);
        continue;
      }
      beast::flat_buffer buffer;
      ws_->read(buffer, error);
      if (!error) {
        if (ws_->got_text()) {
          try {command_(id_, json::parse(beast::buffers_to_string(buffer.data())));}
          catch (const std::exception & exception) {
            enqueueText(json{{"type", "error"}, {"message", exception.what()}}.dump());
          }
        }
      } else {
        break;
      }
    }
    beast::error_code ignored;
    ws_->close(websocket::close_code::normal, ignored);
    ws_.reset();
    close_(id_);
  }

  void flush()
  {
    for (;;) {
      std::string text;
      std::vector<std::uint8_t> binary;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!texts_.empty()) {
          text = std::move(texts_.front());
          texts_.pop_front();
        } else if (!binaries_.empty()) {
          binary = std::move(binaries_.front());
          binary_bytes_ -= binary.size();
          binaries_.pop_front();
        } else if (latest_live_) {
          binary = std::move(*latest_live_);
          latest_live_.reset();
        } else {
          return;
        }
      }
      beast::error_code error;
      if (!text.empty()) {
        ws_->text(true);
        ws_->write(asio::buffer(text), error);
      } else {
        ws_->binary(true);
        ws_->write(asio::buffer(binary), error);
      }
      if (error) {stopping_.store(true); return;}
    }
  }

  std::optional<websocket::stream<tcp::socket>> ws_;
  std::string id_;
  CommandCallback command_;
  CloseCallback close_;
  std::atomic<bool> stopping_{false};
  std::mutex mutex_;
  std::deque<std::string> texts_;
  std::deque<std::vector<std::uint8_t>> binaries_;
  std::optional<std::vector<std::uint8_t>> latest_live_;
  std::size_t binary_bytes_{0};
  std::thread thread_;
};

class HttpWsServer
{
public:
  using CommandCallback = WebSession::CommandCallback;
  using ConnectCallback = std::function<void(const std::shared_ptr<WebSession> &)>;

  HttpWsServer(std::string address, const unsigned short port, fs::path static_root,
    CommandCallback command, ConnectCallback connected)
  : address_(std::move(address)), port_(port), static_root_(std::move(static_root)),
    command_(std::move(command)), connected_(std::move(connected)) {}

  ~HttpWsServer() {stop();}
  void start() {thread_ = std::thread(&HttpWsServer::acceptLoop, this);}
  void stop()
  {
    stopping_.store(true);
    std::vector<std::shared_ptr<WebSession>> sessions;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & entry : sessions_) {sessions.push_back(entry.second);}
    }
    for (const auto & session : sessions) {session->requestStop();}
    {
      std::unique_lock<std::mutex> lock(mutex_);
      sessions_closed_.wait_for(lock, 2s, [this]() {return sessions_.empty();});
    }
    // Beast's websocket service belongs to the accept thread's io_context.
    // Destroy every stream while that context is still alive.
    sessions.clear();
    if (thread_.joinable()) {thread_.join();}
  }
  void broadcastText(const json & message)
  {
    const auto text = message.dump();
    each([&text](const auto & session) {session->enqueueText(text);});
  }
  void sendText(const std::string & id, const json & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = sessions_.find(id);
    if (found != sessions_.end()) {found->second->enqueueText(message.dump());}
  }
  void broadcastBinary(const std::vector<std::uint8_t> & packet, const bool replace_live)
  {
    each([&packet, replace_live](const auto & session) {
      session->enqueueBinary(packet, replace_live);
    });
  }

private:
  template<typename Function>
  void each(Function function)
  {
    std::vector<std::shared_ptr<WebSession>> sessions;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & entry : sessions_) {sessions.push_back(entry.second);}
    }
    for (const auto & session : sessions) {function(session);}
  }
  void remove(const std::string & id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sessions_.erase(id);
    sessions_closed_.notify_all();
  }
  void acceptLoop()
  {
    try {
      const auto address = asio::ip::make_address(address_);
      acceptor_ = std::make_unique<tcp::acceptor>(context_, tcp::endpoint(address, port_));
      beast::error_code mode_error;
      acceptor_->non_blocking(true, mode_error);
      while (!stopping_.load()) {
        beast::error_code error;
        tcp::socket socket(context_);
        acceptor_->accept(socket, error);
        if (error) {
          if (!stopping_.load()) {std::this_thread::sleep_for(20ms);}
          continue;
        }
        std::thread([this, socket = std::move(socket)]() mutable {serve(std::move(socket));}).detach();
      }
      beast::error_code ignored;
      acceptor_->close(ignored);
      acceptor_.reset();
    } catch (...) {}
  }
  void serve(tcp::socket socket)
  {
    beast::flat_buffer buffer;
    http::request<http::string_body> request;
    beast::error_code error;
    http::read(socket, buffer, request, error);
    if (error) {return;}
    if (websocket::is_upgrade(request) && request.target() == "/ws") {
      const auto id = std::to_string(next_id_.fetch_add(1));
      auto session = std::make_shared<WebSession>(std::move(socket), id, command_,
        [this](const std::string & closed) {remove(closed);});
      {
        std::lock_guard<std::mutex> lock(mutex_);
        sessions_[id] = session;
      }
      session->start(std::move(request));
      connected_(session);
      return;
    }
    std::string target(request.target());
    if (target == "/health") {
      sendHttp(socket, request, 200, "application/json", "{\"ok\":true}\n");
      return;
    }
    if (target == "/") {target = "/index.html";}
    if (target.find("..") != std::string::npos) {
      sendHttp(socket, request, 400, "text/plain", "invalid path\n");
      return;
    }
    const fs::path path = static_root_ / target.substr(1);
    try {sendHttp(socket, request, 200, mimeType(path), readText(path));}
    catch (...) {sendHttp(socket, request, 404, "text/plain", "not found\n");}
  }
  static void sendHttp(tcp::socket & socket, const http::request<http::string_body> & request,
    const unsigned status, const std::string & content_type, const std::string & body)
  {
    http::response<http::string_body> response{static_cast<http::status>(status), request.version()};
    response.set(http::field::server, "route3d-web-console");
    response.set(http::field::content_type, content_type);
    response.set(http::field::cache_control, "no-store");
    response.keep_alive(false);
    response.body() = body;
    response.prepare_payload();
    beast::error_code ignored;
    http::write(socket, response, ignored);
  }

  std::string address_;
  unsigned short port_;
  fs::path static_root_;
  CommandCallback command_;
  ConnectCallback connected_;
  std::atomic<bool> stopping_{false};
  std::atomic<std::uint64_t> next_id_{1};
  std::thread thread_;
  // The io_context must outlive every accepted socket and Beast websocket service.
  asio::io_context context_;
  std::unique_ptr<tcp::acceptor> acceptor_;
  std::mutex mutex_;
  std::condition_variable sessions_closed_;
  std::unordered_map<std::string, std::shared_ptr<WebSession>> sessions_;
};

class WebConsoleNode : public rclcpp::Node
{
public:
  WebConsoleNode()
  : Node("route3d_web_console"), process_manager_(
      static_cast<std::size_t>(declare_parameter<int>("runtime.log_lines", 300)),
      [this](const std::string & name, const std::string & line) {
        if (server_) {server_->broadcastText({{"type", "process.log"}, {"process", name}, {"line", line}});}
        if (name == "topology_recording") {
          std::lock_guard<std::mutex> lock(recording_mutex_);
          if (line.find("保存/处理失败") != std::string::npos) {recording_state_ = "error";}
        }
      })
  {
    configure();
    createRosInterfaces();
    server_ = std::make_unique<HttpWsServer>(bind_address_, port_, static_root_,
      [this](const std::string & id, const json & request) {handleCommand(id, request);},
      [this](const std::shared_ptr<WebSession> & session) {sendSnapshot(session);});
    server_->start();
    status_timer_ = create_wall_timer(500ms, std::bind(&WebConsoleNode::publishSnapshot, this));
    lease_timer_ = create_wall_timer(250ms, std::bind(&WebConsoleNode::checkLease, this));
    recording_timer_ = create_wall_timer(500ms, std::bind(&WebConsoleNode::pollTopologyRecording, this));
    RCLCPP_INFO(get_logger(), "Route3D web console: http://%s:%u", bind_address_.c_str(), port_);
  }

  ~WebConsoleNode() override
  {
    if (server_) {server_->stop();}
    process_manager_.stopAll(stop_timeout_);
  }

private:
  void configure()
  {
    bind_address_ = declare_parameter<std::string>("server.bind_address", "0.0.0.0");
    port_ = static_cast<unsigned short>(declare_parameter<int>("server.port", 8080));
    lease_timeout_ = std::chrono::duration<double>(
      declare_parameter<double>("server.control_lease_timeout_s", 3.0));
    static_root_ = expandUser(declare_parameter<std::string>("server.static_root", ""));
    catalog_seed_ = expandUser(declare_parameter<std::string>("catalog_seed_file", ""));
    catalog_file_ = expandUser(declare_parameter<std::string>(
      "runtime.catalog_file", "~/.ros/route3d_web_console/map_catalog.json"));
    cache_directory_ = expandUser(declare_parameter<std::string>(
      "runtime.cache_directory", "~/.ros/route3d_web_console/cache"));
    for (const auto & root : declare_parameter<std::vector<std::string>>(
      "maps.allowed_roots", std::vector<std::string>{})) {allowed_roots_.push_back(expandUser(root));}
    for (const auto & root : declare_parameter<std::vector<std::string>>(
      "maps.allowed_localization_roots", std::vector<std::string>{"/opt/mapping_ws"})) {
      allowed_localization_roots_.push_back(expandUser(root));
    }
    raw_cloud_topic_ = declare_parameter<std::string>("topics.raw_cloud", "/livox/lidar");
    registered_cloud_topic_ = declare_parameter<std::string>(
      "topics.registered_cloud", "/cloud_registered");
    global_map_topic_ = declare_parameter<std::string>("topics.global_map", "/global_map");
    odometry_topic_ = declare_parameter<std::string>("topics.odometry", "/lio_odom_hf");
    initial_pose_topic_ = declare_parameter<std::string>("topics.initial_pose", "/initialpose");
    plan_topic_ = declare_parameter<std::string>(
      "topics.plan_request", "/route3d_dijkstra/plan_request");
    pause_service_ = declare_parameter<std::string>(
      "services.pause", "/route3d_pid_controller/pause");
    resume_service_ = declare_parameter<std::string>(
      "services.resume", "/route3d_pid_controller/resume");
    cancel_service_ = declare_parameter<std::string>(
      "services.cancel", "/route3d_pid_controller/cancel");
    cloud_voxel_ = static_cast<float>(declare_parameter<double>("cloud.voxel_size_m", 0.12));
    cloud_range_ = static_cast<float>(declare_parameter<double>("cloud.maximum_range_m", 20.0));
    cloud_max_points_ = static_cast<std::size_t>(declare_parameter<int>(
      "cloud.maximum_points", 20000));
    cloud_period_ = std::chrono::duration<double>(1.0 / declare_parameter<double>(
      "cloud.maximum_rate_hz", 5.0));
    registered_cloud_fallback_timeout_ = std::chrono::duration<double>(
      declare_parameter<double>("cloud.registered_fallback_timeout_s", 1.0));
    const auto lods = declare_parameter<std::vector<double>>(
      "cloud.map_lod_voxels_m", std::vector<double>{0.4, 0.2, 0.1});
    for (const auto value : lods) {map_lods_.push_back(static_cast<float>(value));}
    chunk_points_ = static_cast<std::size_t>(declare_parameter<int>(
      "cloud.map_chunk_points", 32768));
    odom_freshness_s_ = declare_parameter<double>("localization.odometry_freshness_s", 0.75);
    initial_pose_frame_id_ = declare_parameter<std::string>(
      "localization.initial_pose_frame_id", "map");
    initial_pose_z_ = declare_parameter<double>("localization.initial_pose_z", 0.0);
    xy_variance_ = declare_parameter<double>("localization.xy_variance", 0.25);
    yaw_variance_ = declare_parameter<double>("localization.yaw_variance", 0.0685);
    ros_setup_ = declare_parameter<std::string>("process.ros_setup", "/opt/ros/jazzy/setup.bash");
    workspace_setup_ = declare_parameter<std::string>("process.workspace_setup", "");
    radar_command_ = declare_parameter<std::string>("process.radar_command", "");
    localization_command_ = declare_parameter<std::string>("process.localization_command", "");
    planner_command_ = declare_parameter<std::string>("process.planner_command", "");
    topology_record_command_ = declare_parameter<std::string>(
      "process.topology_record_command",
      "script -q -e -c 'ros2 launch route3d_product_demo live_route_product.launch.py "
      "config_file:={record_config}' /dev/null");
    topology_record_config_template_ = expandUser(declare_parameter<std::string>(
      "topology.record_config_template", ""));
    topology_record_sync_slop_s_ = declare_parameter<double>(
      "topology.record_sync_slop_s", 0.15);
    topology_results_root_ = expandUser(declare_parameter<std::string>(
      "topology.record_results_root", "data"));
    network_interface_ = declare_parameter<std::string>("process.network_interface", "enp2s0");
    domain_id_ = declare_parameter<int>("process.ros_domain_id", 42);
    stop_timeout_ = std::chrono::milliseconds(static_cast<int>(1000.0 *
      declare_parameter<double>("process.stop_timeout_s", 5.0)));
    initializeCatalog();
  }

  void initializeCatalog()
  {
    fs::create_directories(catalog_file_.parent_path());
    fs::create_directories(cache_directory_);
    if (!fs::exists(catalog_file_) && !catalog_seed_.empty() && fs::exists(catalog_seed_)) {
      fs::copy_file(catalog_seed_, catalog_file_);
    }
    if (!fs::exists(catalog_file_)) {
      std::ofstream(catalog_file_) << json{{"version", 1}, {"maps", json::array()}}.dump(2) << '\n';
    }
    catalog_ = json::parse(readText(catalog_file_));
    bool migrated = false;
    for (auto & map : catalog_["maps"]) {
      if (!map.contains("topologies") || !map["topologies"].is_array()) {
        map["topologies"] = json::array();
        const std::string old_path = map.value("topology_path", "");
        if (!old_path.empty()) {
          const fs::path path = expandUser(old_path);
          const std::string id = path.parent_path().filename().string().empty() ?
            path.stem().string() : path.parent_path().filename().string();
          map["topologies"].push_back({{"id", id}, {"label", id}, {"path", old_path},
            {"enabled", true}});
          map["default_topology_id"] = id;
        }
        migrated = true;
      }
      if (!map.contains("default_topology_id") && !map["topologies"].empty()) {
        map["default_topology_id"] = map["topologies"][0].value("id", "");
        migrated = true;
      }
    }
    if (migrated) {writeCatalog();}
  }

  void writeCatalog()
  {
    const fs::path temporary = catalog_file_.string() + ".tmp";
    {
      std::ofstream output(temporary, std::ios::trunc);
      if (!output) {throw std::runtime_error("cannot write map catalog temporary file");}
      output << catalog_.dump(2) << '\n';
      output.flush();
      if (!output) {throw std::runtime_error("cannot flush map catalog temporary file");}
    }
    fs::rename(temporary, catalog_file_);
  }

  void createRosInterfaces()
  {
    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    raw_cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(raw_cloud_topic_,
      sensor_qos, [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        cloudCallback(message, CloudStream::kRawLidar);
      });
    registered_cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      registered_cloud_topic_, sensor_qos,
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        cloudCallback(message, CloudStream::kLiveRegistered);
      });
    // The GLIO localization executable waits until /global_map has at least one subscriber
    // before it finishes initialization.  The original 510.sh starts RViz, which supplied
    // that subscriber.  The web console loads the PCD directly, so it still needs this
    // lightweight subscription when RViz is intentionally not launched.
    global_map_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      global_map_topic_, rclcpp::QoS(1).reliable(),
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        topic_health_["global_map"] = {
          {"points", static_cast<std::uint64_t>(message->width) * message->height},
          {"frame_id", message->header.frame_id}, {"last_seen_ms", 0}};
      });
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(odometry_topic_,
      rclcpp::QoS(20).best_effort(), std::bind(&WebConsoleNode::odometryCallback, this,
      std::placeholders::_1));
    initial_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic_, rclcpp::QoS(10).reliable());
    plan_publisher_ = create_publisher<std_msgs::msg::Int32MultiArray>(
      plan_topic_, rclcpp::QoS(10).reliable());
    pause_client_ = create_client<std_srvs::srv::Trigger>(pause_service_);
    resume_client_ = create_client<std_srvs::srv::Trigger>(resume_service_);
    cancel_client_ = create_client<std_srvs::srv::Trigger>(cancel_service_);
    subscribeStatus("dijkstra", declare_parameter<std::string>(
      "topics.dijkstra_status", "/route3d_dijkstra/status"));
    subscribeStatus("slicer", declare_parameter<std::string>(
      "topics.slicer_status", "/route3d_route_slicer/status"));
    subscribeStatus("pid", declare_parameter<std::string>(
      "topics.pid_status", "/route3d_pid_controller/status"));
    subscribeStatus("active_controller", declare_parameter<std::string>(
      "topics.active_controller", "/route3d_controller/active_source"));
    subscribeStatus("adapter", declare_parameter<std::string>(
      "topics.adapter_status", "/route3d_go2_adapter/status"));
    selected_command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      declare_parameter<std::string>("topics.selected_command",
      "/route3d_go2_adapter/selected_command"), rclcpp::QoS(10).reliable(),
      [this](const geometry_msgs::msg::Twist::ConstSharedPtr message) {
        if (server_) {server_->broadcastText({{"type", "velocity"},
          {"vx", message->linear.x}, {"vy", message->linear.y}, {"wz", message->angular.z}});}
      });
  }

  void subscribeStatus(const std::string & key, const std::string & topic)
  {
    status_subscriptions_.push_back(create_subscription<std_msgs::msg::String>(topic,
      rclcpp::QoS(10).reliable().transient_local(),
      [this, key](const std_msgs::msg::String::ConstSharedPtr message) {
        json value = message->data;
        try {value = json::parse(message->data);} catch (...) {}
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          ros_status_[key] = value;
        }
        if (server_) {server_->broadcastText({{"type", "ros.status"}, {"source", key}, {"data", value}});}
      }));
  }

  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    const auto & p = message->pose.pose.position;
    const auto & q = message->pose.pose.orientation;
    json pose{{"type", "pose"}, {"frame_id", message->header.frame_id},
      {"stamp", {message->header.stamp.sec, message->header.stamp.nanosec}},
      {"position", {p.x, p.y, p.z}}, {"orientation", {q.x, q.y, q.z, q.w}}};
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_pose_ = pose;
      last_odometry_steady_ = std::chrono::steady_clock::now();
      trajectory_.push_back({p.x, p.y, p.z});
      if (trajectory_.size() > 2000U) {
        std::vector<std::array<double, 3>> compact;
        compact.reserve(1001U);
        for (std::size_t i = 0; i < trajectory_.size(); i += 2U) {compact.push_back(trajectory_[i]);}
        trajectory_ = std::move(compact);
      }
    }
    if (server_) {server_->broadcastText(pose);}
  }

  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr message, const CloudStream stream)
  {
    const auto now = std::chrono::steady_clock::now();
    // /livox/lidar is expressed in the sensor/body frame while /cloud_registered is
    // already in the localization frame.  Sending both into the same browser layer
    // makes the cloud alternate between the origin and the localized pose.  Raw lidar
    // is therefore only a fallback while registered output is unavailable or stale.
    if (stream == CloudStream::kRawLidar &&
      last_registered_cloud_.time_since_epoch().count() != 0 &&
      now - last_registered_cloud_ < registered_cloud_fallback_timeout_)
    {
      return;
    }
    auto & last = stream == CloudStream::kLiveRegistered ? last_registered_cloud_ : last_raw_cloud_;
    if (now - last < cloud_period_) {return;}
    last = now;
    try {
      auto cloud = PointCloudCodec::fromRos(*message, cloud_voxel_, cloud_range_, cloud_max_points_);
      const auto packets = PointCloudCodec::encode(cloud, stream, ++cloud_sequence_, cloud_max_points_);
      if (server_ && !packets.empty()) {server_->broadcastBinary(packets.front(), true);}
      const std::string key = stream == CloudStream::kLiveRegistered ? "registered_cloud" : "raw_cloud";
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        topic_health_[key] = {{"points", cloud.points.size()}, {"frame_id", cloud.frame_id},
          {"last_seen_ms", 0}};
      }
    } catch (const std::exception & exception) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cloud conversion: %s", exception.what());
    }
  }

  void handleCommand(const std::string & client, const json & request)
  {
    const std::string type = request.value("type", "");
    const std::string request_id = request.value("request_id", "");
    try {
      if (type == "map.list") {sendMaps(client, request_id);}
      else if (type == "map.select") {selectMap(client, request, request_id);}
      else if (type == "map.upsert") {upsertMap(client, request, request_id);}
      else if (type == "map.cloud_lod") {
        if (selected_map_id_.empty()) {throw std::runtime_error("select a map first");}
        const float voxel = request.value("voxel_m", map_lods_.empty() ? 0.4F : map_lods_.front());
        if (std::find(map_lods_.begin(), map_lods_.end(), voxel) == map_lods_.end()) {
          throw std::runtime_error("unsupported map LOD");
        }
        streamStaticMap(findMap(selected_map_id_), voxel);
        reply(client, request_id, true, "map LOD requested");
      }
      else if (type == "topology.list") {sendTopologies(client, request_id);}
      else if (type == "topology.select") {selectTopology(client, request, request_id);}
      else if (type == "topology.load") {sendTopology(client, request_id);}
      else if (type == "topology.save") {saveTopology(client, request, request_id);}
      else if (type == "control.claim") {claimControl(client, request_id);}
      else if (type == "control.heartbeat") {heartbeat(client, request_id);}
      else if (type == "control.release") {releaseControl(client, request_id);}
      else {
        requireControl(client);
        if (type == "process.start") {startProcess(client, request, request_id);}
        else if (type == "process.stop") {stopProcess(client, request, request_id);}
        else if (type == "localization.set_initial_pose") {setInitialPose(client, request, request_id);}
        else if (type == "navigation.plan") {plan(client, request, request_id);}
        else if (type == "navigation.pause") {callTrigger(client, pause_client_, "pause", request_id);}
        else if (type == "navigation.resume") {callTrigger(client, resume_client_, "resume", request_id);}
        else if (type == "navigation.cancel") {callTrigger(client, cancel_client_, "cancel", request_id);}
        else if (type == "topology.record.start") {startTopologyRecording(client, request_id);}
        else if (type == "topology.record.stop") {stopTopologyRecording(client, request, request_id);}
        else if (type == "topology.record.abort") {abortTopologyRecording(client, request_id);}
        else {throw std::runtime_error("unknown command: " + type);}
      }
    } catch (const std::exception & exception) {
      reply(client, request_id, false, exception.what());
    }
  }

  void sendSnapshot(const std::shared_ptr<WebSession> & session)
  {
    session->enqueueText(json{{"type", "hello"}, {"client_id", session->id()},
      {"protocol", 1}, {"binary_magic", "R3PC"}}.dump());
    session->enqueueText(snapshot().dump());
    session->enqueueText(json{{"type", "map.catalog"}, {"maps", catalog_.value("maps", json::array())},
      {"selected", selected_map_id_}, {"selected_topology", selected_topology_id_}}.dump());
    if (current_topology_) {
      session->enqueueText(json{{"type", "topology.snapshot"}, {"data", current_topology_->data},
        {"revision", current_topology_->revision}, {"id", selected_topology_id_}}.dump());
    }
  }

  json snapshot()
  {
    json processes = json::object();
    for (const auto & name : {"radar", "localization", "planner", "topology_recording"}) {
      const auto value = process_manager_.snapshot(name);
      processes[name] = {{"running", value.running}, {"pid", value.pid},
        {"exit_code", value.exit_code}};
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::string recording_state;
    {
      std::lock_guard<std::mutex> record_lock(recording_mutex_);
      recording_state = recording_state_;
    }
    return {{"type", "snapshot"}, {"processes", processes}, {"selected_map", selected_map_id_},
      {"selected_topology", selected_topology_id_}, {"topology_recording_state", recording_state},
      {"control_owner", control_owner_}, {"ros", ros_status_}, {"topics", topic_health_},
      {"pose", last_pose_}, {"trajectory", trajectory_}};
  }

  void publishSnapshot() {if (server_) {server_->broadcastText(snapshot());}}

  void sendMaps(const std::string & client, const std::string & request_id)
  {
    server_->sendText(client, {{"type", "map.catalog"}, {"request_id", request_id},
      {"maps", catalog_.value("maps", json::array())}, {"selected", selected_map_id_},
      {"selected_topology", selected_topology_id_},
      {"discovered_pcds", discoverPcds()}});
  }

  json discoverPcds()
  {
    json output = json::array();
    for (const auto & root : allowed_roots_) {
      std::error_code error;
      if (!fs::is_directory(root, error)) {continue;}
      for (fs::recursive_directory_iterator it(root, fs::directory_options::skip_permission_denied, error), end;
        it != end && output.size() < 500U; it.increment(error))
      {
        if (error) {error.clear(); continue;}
        if (it.depth() > 5) {it.disable_recursion_pending();}
        if (it->is_regular_file(error) && it->path().extension() == ".pcd") {
          output.push_back(it->path().string());
        }
      }
    }
    return output;
  }

  void selectMap(const std::string & client, const json & request, const std::string & request_id)
  {
    requireControl(client);
    if (process_manager_.snapshot("planner").running ||
      process_manager_.snapshot("localization").running ||
      process_manager_.snapshot("topology_recording").running) {
      throw std::runtime_error("stop planner, localization and topology recording before switching maps");
    }
    const std::string id = request.value("id", "");
    const auto map = findMap(id);
    requireMapPaths(map);
    selected_map_id_ = id;
    selected_topology_id_.clear();
    current_topology_.reset();
    const std::string default_id = map.value("default_topology_id", "");
    if (!default_id.empty()) {
      const auto topology = findTopology(map, default_id);
      selected_topology_id_ = default_id;
      current_topology_ = TopologyStore::load(topology.at("path").get<std::string>());
    }
    reply(client, request_id, true, "map selected");
    server_->broadcastText({{"type", "map.catalog"},
      {"maps", catalog_.value("maps", json::array())}, {"selected", selected_map_id_},
      {"selected_topology", selected_topology_id_}});
    if (current_topology_) {
      server_->broadcastText({{"type", "topology.snapshot"}, {"data", current_topology_->data},
        {"revision", current_topology_->revision}, {"id", selected_topology_id_}});
    }
    streamStaticMap(map, map_lods_.empty() ? 0.4F : map_lods_.front());
  }

  void upsertMap(const std::string & client, const json & request, const std::string & request_id)
  {
    requireControl(client);
    const auto entry = request.at("map");
    for (const auto & key : {"id", "label", "pcd_path", "localization_config_template",
      "frame_id"}) {
      if (!entry.contains(key) || !entry[key].is_string() || entry[key].get<std::string>().empty()) {
        throw std::runtime_error(std::string("map entry missing ") + key);
      }
    }
    requireMapPaths(entry, false);
    auto & maps = catalog_["maps"];
    bool replaced = false;
    for (auto & item : maps) {
      if (item.value("id", "") == entry["id"].get<std::string>()) {item = entry; replaced = true; break;}
    }
    if (!replaced) {maps.push_back(entry);}
    if (!entry.contains("topologies") || !entry["topologies"].is_array()) {
      throw std::runtime_error("map entry missing topologies array");
    }
    writeCatalog();
    reply(client, request_id, true, "map catalog saved");
    server_->broadcastText({{"type", "map.catalog"}, {"maps", maps}, {"selected", selected_map_id_},
      {"selected_topology", selected_topology_id_}});
  }

  json findMap(const std::string & id) const
  {
    for (const auto & item : catalog_.value("maps", json::array())) {
      if (item.value("id", "") == id && item.value("enabled", true)) {return item;}
    }
    throw std::runtime_error("unknown or disabled map: " + id);
  }

  json findTopology(const json & map, const std::string & id) const
  {
    for (const auto & item : map.value("topologies", json::array())) {
      if (item.value("id", "") == id && item.value("enabled", true)) {return item;}
    }
    throw std::runtime_error("unknown or disabled topology: " + id);
  }

  void sendTopologies(const std::string & client, const std::string & request_id)
  {
    if (selected_map_id_.empty()) {throw std::runtime_error("select a map first");}
    const auto map = findMap(selected_map_id_);
    server_->sendText(client, {{"type", "topology.catalog"}, {"request_id", request_id},
      {"map_id", selected_map_id_}, {"topologies", map.value("topologies", json::array())},
      {"selected", selected_topology_id_}});
  }

  void selectTopology(
    const std::string & client, const json & request, const std::string & request_id)
  {
    requireControl(client);
    if (selected_map_id_.empty()) {throw std::runtime_error("select a map first");}
    if (process_manager_.snapshot("planner").running ||
      process_manager_.snapshot("topology_recording").running) {
      throw std::runtime_error("stop planner and topology recording before switching topology");
    }
    const auto map = findMap(selected_map_id_);
    const std::string id = request.value("id", "");
    const auto topology = findTopology(map, id);
    const fs::path path = expandUser(topology.at("path").get<std::string>());
    if (!pathInside(path, allowed_roots_) || !fs::is_regular_file(path)) {
      throw std::runtime_error("topology path is invalid or outside allowed roots");
    }
    current_topology_ = TopologyStore::load(path);
    selected_topology_id_ = id;
    reply(client, request_id, true, "topology selected");
    server_->broadcastText({{"type", "topology.snapshot"}, {"data", current_topology_->data},
      {"revision", current_topology_->revision}, {"id", selected_topology_id_}});
    server_->broadcastText({{"type", "topology.catalog"}, {"map_id", selected_map_id_},
      {"topologies", map.value("topologies", json::array())}, {"selected", selected_topology_id_}});
  }

  void requireMapPaths(const json & map, const bool require_exists = true) const
  {
    const fs::path pcd = expandUser(map.at("pcd_path").get<std::string>());
    if (!pathInside(pcd, allowed_roots_)) {throw std::runtime_error("pcd_path outside allowed roots");}
    if (require_exists && !fs::is_regular_file(pcd)) {throw std::runtime_error("pcd_path does not exist");}
    for (const auto & topology : map.value("topologies", json::array())) {
      const fs::path path = expandUser(topology.at("path").get<std::string>());
      if (!pathInside(path, allowed_roots_)) {throw std::runtime_error("topology path outside allowed roots");}
    }
    const fs::path localization = expandUser(
      map.at("localization_config_template").get<std::string>());
    if (!pathInside(localization, allowed_localization_roots_)) {
      throw std::runtime_error("localization_config_template outside allowed roots");
    }
    if (require_exists && !fs::is_regular_file(localization)) {
      throw std::runtime_error("localization_config_template does not exist");
    }
  }

  void streamStaticMap(const json & map, const float voxel)
  {
    const auto path = fs::path(map.at("pcd_path").get<std::string>());
    const auto frame = map.value("frame_id", "camera_init");
    std::thread([this, path, frame, voxel]() {
      try {
        const auto cloud = PointCloudCodec::loadPcd(path, frame, voxel, 1500000U);
        const auto packets = PointCloudCodec::encode(
          cloud, CloudStream::kStaticMap, ++cloud_sequence_, chunk_points_);
        for (const auto & packet : packets) {
          if (server_) {server_->broadcastBinary(packet, false);}
        }
        if (server_) {server_->broadcastText({{"type", "map.cloud_ready"},
          {"voxel_m", voxel}, {"points", cloud.points.size()}});}
      } catch (const std::exception & exception) {
        if (server_) {server_->broadcastText({{"type", "error"}, {"message", exception.what()}});}
      }
    }).detach();
  }

  void sendTopology(const std::string & client, const std::string & request_id)
  {
    if (!current_topology_) {throw std::runtime_error("no map selected");}
    server_->sendText(client, {{"type", "topology.snapshot"}, {"request_id", request_id},
      {"data", current_topology_->data}, {"revision", current_topology_->revision},
      {"id", selected_topology_id_}});
  }

  void saveTopology(const std::string & client, const json & request, const std::string & request_id)
  {
    requireControl(client);
    if (process_manager_.snapshot("planner").running) {
      throw std::runtime_error("stop planner before saving topology");
    }
    const auto map = findMap(selected_map_id_);
    const auto topology = findTopology(map, selected_topology_id_);
    current_topology_ = TopologyStore::save(topology.at("path").get<std::string>(),
      request.at("data"), request.value("revision", ""));
    reply(client, request_id, true, "topology saved");
    server_->broadcastText({{"type", "topology.saved"}, {"data", current_topology_->data},
      {"revision", current_topology_->revision}});
  }

  std::string commandPrefix() const
  {
    std::string command = "source " + shellQuote(ros_setup_);
    if (!workspace_setup_.empty()) {command += " && source " + shellQuote(workspace_setup_);}
    return command + " && ";
  }

  void startProcess(const std::string & client, const json & request, const std::string & request_id)
  {
    const std::string target = request.value("target", "");
    std::string command;
    if (target == "radar") {
      command = radar_command_;
    } else if (target == "localization") {
      if (selected_map_id_.empty()) {throw std::runtime_error("select a map first");}
      if (!process_manager_.snapshot("radar").running) {
        throw std::runtime_error("radar must be running before localization");
      }
      command = replaceAll(localization_command_, "{localization_config}",
        shellQuote(makeLocalizationConfig(findMap(selected_map_id_))));
    } else if (target == "planner") {
      if (process_manager_.snapshot("topology_recording").running) {
        throw std::runtime_error("finish topology recording before starting planner");
      }
      if (!localizationReady()) {throw std::runtime_error("fresh localization odometry is required");}
      if (!current_topology_ || selected_topology_id_.empty()) {
        throw std::runtime_error("select and load a topology first");
      }
      const auto map = findMap(selected_map_id_);
      const auto topology = findTopology(map, selected_topology_id_);
      command = replaceAll(planner_command_, "{graph_file}",
        shellQuote(topology.at("path").get<std::string>()));
      command = replaceAll(command, "{network_interface}", shellQuote(network_interface_));
      command = replaceAll(command, "{enable_motion}", request.value("enable_motion", false) ? "true" : "false");
    } else {
      throw std::runtime_error("invalid process target");
    }
    std::string error;
    if (!process_manager_.start(target, commandPrefix() + command,
      {{"ROS_DOMAIN_ID", std::to_string(domain_id_)}}, error)) {throw std::runtime_error(error);}
    reply(client, request_id, true, target + " started");
  }

  void stopProcess(const std::string & client, const json & request, const std::string & request_id)
  {
    const std::string target = request.value("target", "");
    if (target != "radar" && target != "localization" && target != "planner") {
      throw std::runtime_error("invalid process target");
    }
    if ((target == "radar" || target == "localization") &&
      process_manager_.snapshot("planner").running) {
      throw std::runtime_error("stop planner before stopping localization or radar");
    }
    if ((target == "radar" || target == "localization") &&
      process_manager_.snapshot("topology_recording").running) {
      throw std::runtime_error("finish or abort topology recording before stopping localization or radar");
    }
    if (target == "planner") {requestCancel(); std::this_thread::sleep_for(150ms);}
    // A graceful ROS shutdown can legitimately take longer than the three-second browser
    // lease.  Keep the lease alive while this synchronous operation is in progress; otherwise
    // the same client can lose ownership before receiving the stop acknowledgement.
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (control_owner_ == client) {
        control_heartbeat_ = std::chrono::steady_clock::now() + stop_timeout_ + 2s;
      }
    }
    std::string error;
    const bool stopped = process_manager_.stop(target, stop_timeout_, error);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (control_owner_ == client) {control_heartbeat_ = std::chrono::steady_clock::now();}
    }
    if (!stopped) {throw std::runtime_error(error);}
    reply(client, request_id, true, target + " stopped");
  }

  void startTopologyRecording(const std::string & client, const std::string & request_id)
  {
    if (selected_map_id_.empty()) {throw std::runtime_error("select a localization map first");}
    if (!localizationReady()) {throw std::runtime_error("fresh localization odometry is required");}
    if (process_manager_.snapshot("planner").running) {
      throw std::runtime_error("stop planner before topology recording");
    }
    if (process_manager_.snapshot("topology_recording").running) {
      throw std::runtime_error("topology recording is already running");
    }
    std::string command = topology_record_command_;
    command = replaceAll(command, "{record_config}", shellQuote(makeTopologyRecordingConfig()));
    std::string error;
    if (!process_manager_.start("topology_recording", commandPrefix() + command,
      {{"ROS_DOMAIN_ID", std::to_string(domain_id_)}}, error)) {
      throw std::runtime_error(error);
    }
    {
      std::lock_guard<std::mutex> lock(recording_mutex_);
      recording_state_ = "starting";
      recording_session_.clear();
      recording_save_name_.clear();
      recording_started_ = std::chrono::system_clock::now();
      preview_mtime_ = fs::file_time_type::min();
      recording_preexisting_sessions_.clear();
      std::error_code scan_error;
      for (const auto & entry : fs::directory_iterator("/tmp", scan_error)) {
        if (!scan_error && entry.is_directory(scan_error) &&
          entry.path().filename().string().rfind("route3d_live_", 0) == 0) {
          recording_preexisting_sessions_.insert(entry.path());
        }
      }
    }
    if (server_) {server_->broadcastText({{"type", "topology.preview.clear"}});}
    // ros2 launch needs time to create route3d_live_keyboard and its /dev/tty. Sending `1`
    // before the menu is printed lets the launcher consume the byte. Wait for the exact menu
    // marker so the following input is equivalent to the operator pressing 1 at the prompt.
    bool ready = false;
    const auto ready_deadline = std::chrono::steady_clock::now() + 12s;
    while (std::chrono::steady_clock::now() < ready_deadline) {
      const auto process = process_manager_.snapshot("topology_recording");
      for (const auto & line : process.log_lines) {
        if (line.find("Route3D 实时产品流程") != std::string::npos) {ready = true; break;}
      }
      if (ready || !process.running) {break;}
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (control_owner_ == client) {control_heartbeat_ = std::chrono::steady_clock::now();}
      }
      std::this_thread::sleep_for(100ms);
    }
    if (!ready) {
      process_manager_.stop("topology_recording", stop_timeout_, error);
      {
        std::lock_guard<std::mutex> lock(recording_mutex_);
        recording_state_ = "error";
      }
      throw std::runtime_error("topology keyboard workflow did not become ready within 12 seconds");
    }
    if (!process_manager_.writeInput("topology_recording", "1", error)) {
      process_manager_.stop("topology_recording", stop_timeout_, error);
      {
        std::lock_guard<std::mutex> lock(recording_mutex_);
        recording_state_ = "error";
      }
      throw std::runtime_error(error);
    }
    bool started = false;
    const auto start_deadline = std::chrono::steady_clock::now() + 8s;
    while (std::chrono::steady_clock::now() < start_deadline) {
      const auto process = process_manager_.snapshot("topology_recording");
      for (const auto & line : process.log_lines) {
        if (line.find("[1] 已开始") != std::string::npos) {started = true; break;}
        if (line.find("在线骨架节点启动失败") != std::string::npos ||
          line.find("数据记录器启动失败") != std::string::npos) {
          error = line;
          break;
        }
      }
      if (started || !error.empty() || !process.running) {break;}
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (control_owner_ == client) {control_heartbeat_ = std::chrono::steady_clock::now();}
      }
      std::this_thread::sleep_for(100ms);
    }
    if (!started) {
      process_manager_.stop("topology_recording", stop_timeout_, error);
      {
        std::lock_guard<std::mutex> lock(recording_mutex_);
        recording_state_ = "error";
      }
      throw std::runtime_error(error.empty() ?
        "topology recorder did not confirm start within 8 seconds" : error);
    }
    {
      std::lock_guard<std::mutex> lock(recording_mutex_);
      recording_state_ = "recording";
    }
    reply(client, request_id, true, "topology recording started; drive the robot with the remote");
  }

  std::string makeTopologyRecordingConfig()
  {
    if (topology_record_config_template_.empty() ||
      !fs::is_regular_file(topology_record_config_template_)) {
      throw std::runtime_error("topology recording config template does not exist");
    }
    std::istringstream input(readText(topology_record_config_template_));
    std::ostringstream output;
    std::string line;
    bool changed_slop = false;
    bool changed_results = false;
    bool in_rviz = false;
    while (std::getline(input, line)) {
      const auto first = line.find_first_not_of(" \t");
      const std::string trimmed = first == std::string::npos ? "" : line.substr(first);
      const std::string indent = first == std::string::npos ? "" : line.substr(0, first);
      if (trimmed.rfind("results_root:", 0) == 0) {
        line = indent + "results_root: " + topology_results_root_.string();
        changed_results = true;
      } else if (trimmed.rfind("sync_slop:", 0) == 0) {
        line = indent + "sync_slop: " + std::to_string(topology_record_sync_slop_s_);
        changed_slop = true;
      } else if (trimmed == "rviz:") {
        in_rviz = true;
      } else if (in_rviz && trimmed.rfind("enabled:", 0) == 0) {
        line = indent + "enabled: false";
        in_rviz = false;
      }
      output << line << '\n';
    }
    if (!changed_slop || !changed_results) {
      throw std::runtime_error("recording template lacks sync_slop or results_root");
    }
    const fs::path directory = cache_directory_ / "runtime";
    fs::create_directories(directory);
    const fs::path path = directory / "live_route_product_web.yaml";
    std::ofstream(path, std::ios::trunc) << output.str();
    return path.string();
  }

  static std::string validateSaveName(const std::string & raw)
  {
    if (raw.empty() || raw == "." || raw == ".." || raw.size() > 80U ||
      !std::regex_match(raw, std::regex("[A-Za-z0-9._-]+"))) {
      throw std::runtime_error("save name may only contain letters, digits, dot, underscore and hyphen");
    }
    return raw;
  }

  void stopTopologyRecording(
    const std::string & client, const json & request, const std::string & request_id)
  {
    if (!process_manager_.snapshot("topology_recording").running) {
      throw std::runtime_error("topology recording is not running");
    }
    const std::string name = validateSaveName(request.value("name", ""));
    const fs::path destination = topology_results_root_ / name;
    if (!pathInside(destination, allowed_roots_)) {
      throw std::runtime_error("topology result directory is outside allowed roots");
    }
    if (fs::exists(destination)) {throw std::runtime_error("result name already exists");}
    {
      std::lock_guard<std::mutex> lock(recording_mutex_);
      recording_save_name_ = name;
      recording_state_ = "processing";
    }
    std::string error;
    // No newline is placed between 2 and the name: the legacy loop consumes the first
    // character in cbreak mode, then readline consumes the remaining name after processing.
    if (!process_manager_.writeInput("topology_recording", "2" + name + "\n", error)) {
      throw std::runtime_error(error);
    }
    reply(client, request_id, true, "recording stopped; final topology is being processed");
  }

  void abortTopologyRecording(const std::string & client, const std::string & request_id)
  {
    std::string error;
    if (process_manager_.snapshot("topology_recording").running) {
      if (!process_manager_.writeInput("topology_recording", "3", error)) {
        throw std::runtime_error(error);
      }
    }
    {
      std::lock_guard<std::mutex> lock(recording_mutex_);
      recording_state_ = "aborted";
      recording_save_name_.clear();
    }
    reply(client, request_id, true, "topology recording aborted without saving");
  }

  void pollTopologyRecording()
  {
    std::string state;
    std::string save_name;
    {
      std::lock_guard<std::mutex> lock(recording_mutex_);
      state = recording_state_;
      save_name = recording_save_name_;
    }
    if (state == "recording" || state == "starting") {
      fs::path newest;
      fs::file_time_type newest_time = fs::file_time_type::min();
      std::error_code error;
      for (const auto & entry : fs::directory_iterator("/tmp", error)) {
        if (error) {break;}
        if (!entry.is_directory(error) ||
          entry.path().filename().string().rfind("route3d_live_", 0) != 0) {continue;}
        {
          std::lock_guard<std::mutex> lock(recording_mutex_);
          if (recording_preexisting_sessions_.count(entry.path()) != 0U) {continue;}
        }
        const auto graph = entry.path() / "live" / "topoGraph_live.json";
        if (!fs::is_regular_file(graph, error)) {continue;}
        const auto time = fs::last_write_time(graph, error);
        if (!error && time > newest_time) {newest = graph; newest_time = time;}
      }
      if (!newest.empty() && newest_time != preview_mtime_) {
        try {
          const auto preview = TopologyStore::load(newest);
          preview_mtime_ = newest_time;
          {
            std::lock_guard<std::mutex> lock(recording_mutex_);
            recording_session_ = newest.parent_path().parent_path();
          }
          if (server_) {server_->broadcastText({{"type", "topology.preview"},
            {"data", preview.data}, {"source", newest.string()}});}
        } catch (const std::exception &) {
          // The online writer uses atomic replacement, but the first poll may still precede
          // schema completion. The next timer tick retries without interrupting recording.
        }
      }
    }
    if (state != "processing" || save_name.empty()) {return;}
    const fs::path output = topology_results_root_ / save_name / "topoGraph_data.json";
    if (!fs::is_regular_file(output)) {
      if (!process_manager_.snapshot("topology_recording").running) {
        {
          std::lock_guard<std::mutex> lock(recording_mutex_);
          recording_state_ = "error";
        }
        if (server_) {server_->broadcastText({{"type", "error"},
          {"message", "自动打点处理失败：未生成 topoGraph_data.json；临时数据仍保留在 /tmp"}});}
      }
      return;
    }
    try {
      const auto document = TopologyStore::load(output);
      auto & maps = catalog_["maps"];
      json * selected_map = nullptr;
      for (auto & map : maps) {
        if (map.value("id", "") == selected_map_id_) {selected_map = &map; break;}
      }
      if (selected_map == nullptr) {throw std::runtime_error("selected map disappeared from catalog");}
      auto & topologies = (*selected_map)["topologies"];
      bool replaced = false;
      for (auto & item : topologies) {
        if (item.value("id", "") == save_name) {
          item = {{"id", save_name}, {"label", save_name}, {"path", output.string()},
            {"enabled", true}};
          replaced = true;
          break;
        }
      }
      if (!replaced) {topologies.push_back({{"id", save_name}, {"label", save_name},
        {"path", output.string()}, {"enabled", true}});}
      (*selected_map)["default_topology_id"] = save_name;
      writeCatalog();
      selected_topology_id_ = save_name;
      current_topology_ = document;
      {
        std::lock_guard<std::mutex> lock(recording_mutex_);
        recording_state_ = "saved";
        recording_save_name_.clear();
      }
      std::string ignored;
      process_manager_.writeInput("topology_recording", "q", ignored);
      if (server_) {
        server_->broadcastText({{"type", "map.catalog"}, {"maps", maps},
          {"selected", selected_map_id_}, {"selected_topology", selected_topology_id_}});
        server_->broadcastText({{"type", "topology.snapshot"}, {"data", document.data},
          {"revision", document.revision}, {"id", selected_topology_id_}});
        server_->broadcastText({{"type", "topology.record.saved"}, {"name", save_name},
          {"path", output.string()}});
      }
    } catch (const std::exception & exception) {
      std::lock_guard<std::mutex> lock(recording_mutex_);
      recording_state_ = "error";
      if (server_) {server_->broadcastText({{"type", "error"}, {"message", exception.what()}});}
    }
  }

  std::string makeLocalizationConfig(const json & map)
  {
    const fs::path source = expandUser(map.at("localization_config_template").get<std::string>());
    if (!fs::is_regular_file(source)) {throw std::runtime_error("localization template does not exist");}
    std::istringstream input(readText(source));
    std::ostringstream output;
    std::string line;
    bool changed = false;
    while (std::getline(input, line)) {
      const auto key = line.find("global_map_path:");
      if (key != std::string::npos) {
        line = line.substr(0, key) + "global_map_path: " + map.at("pcd_path").get<std::string>();
        changed = true;
      }
      output << line << '\n';
    }
    if (!changed) {throw std::runtime_error("localization template has no global_map_path key");}
    const fs::path directory = cache_directory_ / "runtime";
    fs::create_directories(directory);
    const fs::path result = directory / (map.at("id").get<std::string>() + "_localization.yaml");
    std::ofstream(result, std::ios::trunc) << output.str();
    return result.string();
  }

  bool localizationReady() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return !last_pose_.is_null() &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_odometry_steady_).count() <
      odom_freshness_s_;
  }

  void setInitialPose(const std::string & client, const json & request, const std::string & request_id)
  {
    if (selected_map_id_.empty()) {throw std::runtime_error("select a map first");}
    const auto position = request.at("position");
    const double yaw = request.at("yaw").get<double>();
    if (!position.is_array() || position.size() != 3 || !std::isfinite(yaw)) {
      throw std::runtime_error("position[3] and finite yaw are required");
    }
    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.stamp = now();
    message.header.frame_id = initial_pose_frame_id_;
    message.pose.pose.position.x = position[0].get<double>();
    message.pose.pose.position.y = position[1].get<double>();
    // Match RViz's 2D Pose Estimate. A browser-side PCD pick may otherwise select a
    // wall or ceiling and feed a misleading height into the localization initializer.
    message.pose.pose.position.z = initial_pose_z_;
    message.pose.pose.orientation.z = std::sin(yaw * 0.5);
    message.pose.pose.orientation.w = std::cos(yaw * 0.5);
    message.pose.covariance[0] = xy_variance_;
    message.pose.covariance[7] = xy_variance_;
    message.pose.covariance[35] = yaw_variance_;
    initial_pose_publisher_->publish(message);
    RCLCPP_INFO(get_logger(),
      "Published initial pose: frame=%s x=%.3f y=%.3f z=%.3f yaw=%.3f",
      message.header.frame_id.c_str(), message.pose.pose.position.x,
      message.pose.pose.position.y, message.pose.pose.position.z, yaw);
    reply(client, request_id, true, "initial pose published");
  }

  void plan(const std::string & client, const json & request, const std::string & request_id)
  {
    if (!process_manager_.snapshot("planner").running) {throw std::runtime_error("planner is not running");}
    std_msgs::msg::Int32MultiArray message;
    message.data = {request.at("start_id").get<int>(), request.at("goal_id").get<int>()};
    plan_publisher_->publish(message);
    reply(client, request_id, true, "plan request published");
  }

  void callTrigger(const std::string & client,
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & service,
    const std::string & name, const std::string & request_id)
  {
    if (!service->service_is_ready()) {throw std::runtime_error(name + " service is not ready");}
    service->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this, client, request_id](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        try {
          const auto response = future.get();
          reply(client, request_id, response->success, response->message);
        } catch (const std::exception & exception) {reply(client, request_id, false, exception.what());}
      });
  }

  void requestCancel()
  {
    if (cancel_client_->service_is_ready()) {
      cancel_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    }
  }

  void claimControl(const std::string & client, const std::string & request_id)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!control_owner_.empty() && control_owner_ != client) {
      throw std::runtime_error("another browser owns the control lease");
    }
    control_owner_ = client;
    control_heartbeat_ = std::chrono::steady_clock::now();
    reply(client, request_id, true, "control lease acquired");
  }

  void heartbeat(const std::string & client, const std::string & request_id)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (control_owner_ != client) {throw std::runtime_error("client does not own control");}
    control_heartbeat_ = std::chrono::steady_clock::now();
    reply(client, request_id, true, "heartbeat");
  }

  void releaseControl(const std::string & client, const std::string & request_id)
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (control_owner_ != client) {throw std::runtime_error("client does not own control");}
      control_owner_.clear();
    }
    requestCancel();
    reply(client, request_id, true, "control released and navigation cancelled");
  }

  void requireControl(const std::string & client) const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (control_owner_ != client) {throw std::runtime_error("acquire the control lease first");}
  }

  void checkLease()
  {
    bool expired = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!control_owner_.empty() && std::chrono::steady_clock::now() - control_heartbeat_ > lease_timeout_) {
        control_owner_.clear();
        expired = true;
      }
    }
    if (expired) {
      requestCancel();
      if (server_) {server_->broadcastText({{"type", "control.expired"},
        {"message", "control heartbeat expired; navigation cancelled"}});}
    }
  }

  void reply(const std::string & client, const std::string & request_id,
    const bool success, const std::string & message)
  {
    if (server_) {server_->sendText(client, {{"type", "ack"}, {"request_id", request_id},
      {"success", success}, {"message", message}});}
  }

  ProcessManager process_manager_;
  std::unique_ptr<HttpWsServer> server_;
  std::string bind_address_;
  unsigned short port_{8080};
  fs::path static_root_;
  fs::path catalog_seed_;
  fs::path catalog_file_;
  fs::path cache_directory_;
  std::vector<fs::path> allowed_roots_;
  std::vector<fs::path> allowed_localization_roots_;
  json catalog_;
  std::string selected_map_id_;
  std::string selected_topology_id_;
  std::optional<TopologyDocument> current_topology_;

  std::string raw_cloud_topic_, registered_cloud_topic_, global_map_topic_;
  std::string odometry_topic_, initial_pose_topic_;
  std::string plan_topic_, pause_service_, resume_service_, cancel_service_;
  std::string ros_setup_, workspace_setup_, radar_command_, localization_command_, planner_command_;
  std::string topology_record_command_;
  fs::path topology_record_config_template_;
  fs::path topology_results_root_;
  double topology_record_sync_slop_s_{0.15};
  std::string network_interface_;
  int domain_id_{42};
  float cloud_voxel_{0.12F}, cloud_range_{20.0F};
  std::size_t cloud_max_points_{20000}, chunk_points_{32768};
  std::vector<float> map_lods_;
  std::chrono::duration<double> cloud_period_{0.2};
  std::chrono::duration<double> registered_cloud_fallback_timeout_{1.0};
  std::chrono::steady_clock::time_point last_raw_cloud_{}, last_registered_cloud_{};
  std::atomic<std::uint32_t> cloud_sequence_{0};
  std::chrono::milliseconds stop_timeout_{5000};
  std::chrono::duration<double> lease_timeout_{3.0};
  double odom_freshness_s_{0.75}, xy_variance_{0.25}, yaw_variance_{0.0685};
  double initial_pose_z_{0.0};
  std::string initial_pose_frame_id_{"map"};

  mutable std::mutex state_mutex_;
  std::string control_owner_;
  std::chrono::steady_clock::time_point control_heartbeat_{};
  std::chrono::steady_clock::time_point last_odometry_steady_{};
  json last_pose_;
  json ros_status_ = json::object();
  json topic_health_ = json::object();
  std::vector<std::array<double, 3>> trajectory_;

  mutable std::mutex recording_mutex_;
  std::string recording_state_{"idle"};
  std::string recording_save_name_;
  fs::path recording_session_;
  std::set<fs::path> recording_preexisting_sessions_;
  std::chrono::system_clock::time_point recording_started_{};
  fs::file_time_type preview_mtime_{fs::file_time_type::min()};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_cloud_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr registered_cloud_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr selected_command_subscription_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> status_subscriptions_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr plan_publisher_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pause_client_, resume_client_, cancel_client_;
  rclcpp::TimerBase::SharedPtr status_timer_, lease_timer_, recording_timer_;
};

}  // namespace route3d_web_console

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<route3d_web_console::WebConsoleNode>());
  } catch (const std::exception & exception) {
    std::cerr << "route3d_web_console fatal: " << exception.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

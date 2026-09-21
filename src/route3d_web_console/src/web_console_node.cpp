#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
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
#include <std_msgs/msg/int32.hpp>
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
  void enqueueLatestText(std::string channel, std::string text)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_texts_[std::move(channel)] = std::move(text);
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
        // Telemetry such as pose, velocity and controller status is state, not
        // an event stream. Send only the freshest value under backpressure so
        // a slow browser never replays stale robot positions.
        if (!latest_texts_.empty()) {
          auto latest = latest_texts_.begin();
          text = std::move(latest->second);
          latest_texts_.erase(latest);
        } else if (!texts_.empty()) {
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
  std::unordered_map<std::string, std::string> latest_texts_;
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
  using DisconnectCallback = std::function<void(const std::string &)>;

  HttpWsServer(std::string address, const unsigned short port, fs::path static_root,
    CommandCallback command, ConnectCallback connected, DisconnectCallback disconnected)
  : address_(std::move(address)), port_(port), static_root_(std::move(static_root)),
    command_(std::move(command)), connected_(std::move(connected)),
    disconnected_(std::move(disconnected)) {}

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
  void broadcastLatestText(const std::string & channel, const json & message)
  {
    const auto text = message.dump();
    each([&channel, &text](const auto & session) {
      session->enqueueLatestText(channel, text);
    });
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
  void sendBinary(const std::string & id, const std::vector<std::uint8_t> & packet,
    const bool replace_live)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = sessions_.find(id);
    if (found != sessions_.end()) {found->second->enqueueBinary(packet, replace_live);}
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
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sessions_.erase(id);
      sessions_closed_.notify_all();
    }
    if (disconnected_) {disconnected_(id);}
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
  DisconnectCallback disconnected_;
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
      [this](const std::shared_ptr<WebSession> & session) {sendSnapshot(session);},
      [this](const std::string & id) {
        std::lock_guard<std::mutex> lock(live_cloud_clients_mutex_);
        live_cloud_clients_.erase(id);
      });
    server_->start();
    status_timer_ = create_wall_timer(500ms, std::bind(&WebConsoleNode::publishSnapshot, this));
    recording_timer_ = create_wall_timer(500ms, std::bind(&WebConsoleNode::pollTopologyRecording, this));
    topology_generation_timer_ = create_wall_timer(
      250ms, std::bind(&WebConsoleNode::pollTopologyGeneration, this));
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
    resource_scan_max_files_ = static_cast<std::size_t>(declare_parameter<int>(
      "resources.scan_max_files", 1000));
    resource_scan_max_depth_ = declare_parameter<int>("resources.scan_max_depth", 6);
    pcd_root_ = expandUser(declare_parameter<std::string>(
      "resources.pcd_root", "/home/wei"));
    topology_root_ = expandUser(declare_parameter<std::string>(
      "resources.topology_root", "data"));
    initializer_graph_topic_ = declare_parameter<std::string>(
      "vertex_initializer.graph_file_topic", "/route3d_initial_pose/graph_file");
    initializer_vertex_topic_ = declare_parameter<std::string>(
      "vertex_initializer.vertex_id_topic", "/route3d_initial_pose/vertex_id");
    raw_cloud_topic_ = declare_parameter<std::string>("topics.raw_cloud", "/livox/lidar");
    registered_cloud_topic_ = declare_parameter<std::string>(
      "topics.registered_cloud", "/cloud_registered_body");
    mapping_cloud_topic_ = declare_parameter<std::string>(
      "topics.mapping_cloud", "/cloud_registered_body");
    global_map_topic_ = declare_parameter<std::string>("topics.global_map", "/global_map");
    odometry_topic_ = declare_parameter<std::string>("topics.odometry", "/lio_odom_hf");
    mapping_odometry_topic_ = declare_parameter<std::string>(
      "topics.mapping_odometry", "/lio_odom");
    initial_pose_topic_ = declare_parameter<std::string>("topics.initial_pose", "/initialpose");
    plan_topic_ = declare_parameter<std::string>(
      "topics.plan_request", "/route3d_dijkstra/plan_request");
    goal_topic_ = declare_parameter<std::string>(
      "topics.goal_request", "/route3d_dijkstra/goal_request");
    pause_service_ = declare_parameter<std::string>(
      "services.pause", "/route3d_pid_controller/pause");
    resume_service_ = declare_parameter<std::string>(
      "services.resume", "/route3d_pid_controller/resume");
    cancel_service_ = declare_parameter<std::string>(
      "services.cancel", "/route3d_pid_controller/cancel");
    save_map_service_ = declare_parameter<std::string>(
      "services.save_map", "/save_pcd_service");
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
    selected_map_voxel_ = static_cast<float>(declare_parameter<double>(
      "cloud.map_default_voxel_m", map_lods_.empty() ? 0.2 : map_lods_.front()));
    map_voxel_min_ = static_cast<float>(declare_parameter<double>(
      "cloud.map_min_voxel_m", 0.01));
    map_voxel_max_ = static_cast<float>(declare_parameter<double>(
      "cloud.map_max_voxel_m", 2.0));
    chunk_points_ = static_cast<std::size_t>(declare_parameter<int>(
      "cloud.map_chunk_points", 32768));
    const double pose_rate_hz = declare_parameter<double>(
      "telemetry.pose_maximum_rate_hz", 30.0);
    const double velocity_rate_hz = declare_parameter<double>(
      "telemetry.velocity_maximum_rate_hz", 10.0);
    const double status_rate_hz = declare_parameter<double>(
      "telemetry.status_maximum_rate_hz", 10.0);
    if (!std::isfinite(pose_rate_hz) || pose_rate_hz <= 0.0 ||
      !std::isfinite(velocity_rate_hz) || velocity_rate_hz <= 0.0 ||
      !std::isfinite(status_rate_hz) || status_rate_hz <= 0.0)
    {
      throw std::invalid_argument("telemetry maximum rates must be finite and greater than zero");
    }
    pose_broadcast_period_ = std::chrono::duration<double>(1.0 / pose_rate_hz);
    velocity_broadcast_period_ = std::chrono::duration<double>(1.0 / velocity_rate_hz);
    status_broadcast_period_ = std::chrono::duration<double>(1.0 / status_rate_hz);
    odom_freshness_s_ = declare_parameter<double>("localization.odometry_freshness_s", 0.75);
    initial_pose_frame_id_ = declare_parameter<std::string>(
      "localization.initial_pose_frame_id", "map");
    initial_pose_z_ = declare_parameter<double>("localization.initial_pose_z", 0.0);
    xy_variance_ = declare_parameter<double>("localization.xy_variance", 0.25);
    yaw_variance_ = declare_parameter<double>("localization.yaw_variance", 0.0685);
    ros_setup_ = declare_parameter<std::string>("process.ros_setup", "/opt/ros/jazzy/setup.bash");
    workspace_setup_ = declare_parameter<std::string>("process.workspace_setup", "");
    radar_command_ = declare_parameter<std::string>("process.radar_command", "");
    radar_stop_command_ = declare_parameter<std::string>(
      "process.radar_stop_command", "sudo -n systemctl stop driver.service");
    localization_command_ = declare_parameter<std::string>("process.localization_command", "");
    localization_default_config_template_ = expandUser(declare_parameter<std::string>(
      "localization.config_template", "/opt/mapping_ws/glio_mapping/mid360_loc.yaml"));
    transform_registered_body_cloud_ = declare_parameter<bool>(
      "cloud.registered_body_transform_with_odometry", true);
    planner_command_ = declare_parameter<std::string>("process.planner_command", "");
    loop_patrol_script_ = expandUser(declare_parameter<std::string>(
      "process.loop_patrol_script", "sh/5_start_loop_patrol.sh"));
    goal_only_loop_patrol_script_ = expandUser(declare_parameter<std::string>(
      "process.goal_only_loop_patrol_script", "sh/7_start_goal_only_loop.sh"));
    mapping_command_ = declare_parameter<std::string>(
      "process.mapping_command", "cd /opt/mapping_ws && ./run_mapping_nodes.sh mode:=mapping");
    mapping_setup_ = expandUser(declare_parameter<std::string>(
      "process.mapping_setup", "/opt/mapping_ws/install/setup.bash"));
    mapping_default_config_ = declare_parameter<std::string>(
      "mapping.default_config", "robosense_lio");
    mapping_allowed_configs_ = declare_parameter<std::vector<std::string>>(
      "mapping.allowed_configs", {"robosense_lio", "robosense_glio"});
    mapping_save_resolution_tag_ = declare_parameter<std::string>(
      "mapping.save_resolution_tag", "0.1");
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
    topology_generation_config_ = expandUser(declare_parameter<std::string>(
      "topology.generation_config_file", ""));
    topology_generation_frame_id_ = declare_parameter<std::string>(
      "topology.generation_frame_id", "camera_init");
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
      sensor_qos, std::bind(&WebConsoleNode::odometryCallback, this,
      std::placeholders::_1));
    mapping_odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      mapping_odometry_topic_, sensor_qos,
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_mapping_odometry_steady_ = std::chrono::steady_clock::now();
        latest_mapping_pose_ = message->pose.pose;
        latest_mapping_frame_id_ = message->header.frame_id;
        has_mapping_pose_ = true;
      });
    initial_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic_, rclcpp::QoS(10).reliable());
    auto initializer_graph_qos = rclcpp::QoS(1).reliable().transient_local();
    initializer_graph_publisher_ = create_publisher<std_msgs::msg::String>(
      initializer_graph_topic_, initializer_graph_qos);
    initializer_vertex_publisher_ = create_publisher<std_msgs::msg::Int32>(
      initializer_vertex_topic_, rclcpp::QoS(10).reliable());
    plan_publisher_ = create_publisher<std_msgs::msg::Int32MultiArray>(
      plan_topic_, rclcpp::QoS(10).reliable());
    goal_publisher_ = create_publisher<std_msgs::msg::Int32>(
      goal_topic_, rclcpp::QoS(10).reliable());
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
      "topics.adapter_status", "/route3d_m20_adapter/status"));
    selected_command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      declare_parameter<std::string>("topics.selected_command",
      "/route3d_m20_adapter/selected_command"), rclcpp::QoS(10).reliable(),
      [this](const geometry_msgs::msg::Twist::ConstSharedPtr message) {
        bool publish = false;
        const auto now = std::chrono::steady_clock::now();
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          publish = last_velocity_broadcast_.time_since_epoch().count() == 0 ||
            now - last_velocity_broadcast_ >= velocity_broadcast_period_;
          if (publish) {last_velocity_broadcast_ = now;}
        }
        if (publish && server_) {
          server_->broadcastLatestText("velocity", {{"type", "velocity"},
            {"vx", message->linear.x}, {"vy", message->linear.y}, {"wz", message->angular.z}});
        }
      });
  }

  void subscribeStatus(const std::string & key, const std::string & topic)
  {
    status_subscriptions_.push_back(create_subscription<std_msgs::msg::String>(topic,
      rclcpp::QoS(10).reliable().transient_local(),
      [this, key](const std_msgs::msg::String::ConstSharedPtr message) {
        json value = message->data;
        try {value = json::parse(message->data);} catch (...) {}
        bool publish = false;
        const auto now = std::chrono::steady_clock::now();
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          ros_status_[key] = value;
          const auto last_time = last_status_broadcast_.find(key);
          const auto last_value = last_status_broadcast_values_.find(key);
          const bool changed = last_value == last_status_broadcast_values_.end() ||
            last_value->second != value;
          publish = changed || last_time == last_status_broadcast_.end() ||
            now - last_time->second >= status_broadcast_period_;
          if (publish) {
            last_status_broadcast_[key] = now;
            last_status_broadcast_values_[key] = value;
          }
        }
        if (publish && server_) {
          server_->broadcastLatestText("ros.status." + key,
            {{"type", "ros.status"}, {"source", key}, {"data", value}});
        }
      }));
  }

  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    const auto & p = message->pose.pose.position;
    const auto & q = message->pose.pose.orientation;
    json pose{{"type", "pose"}, {"frame_id", message->header.frame_id},
      {"stamp", {message->header.stamp.sec, message->header.stamp.nanosec}},
      {"position", {p.x, p.y, p.z}}, {"orientation", {q.x, q.y, q.z, q.w}}};
    bool publish = false;
    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_pose_ = pose;
      last_odometry_steady_ = now;
      latest_localization_pose_ = message->pose.pose;
      latest_localization_frame_id_ = message->header.frame_id;
      has_localization_pose_ = true;
      publish = last_pose_broadcast_.time_since_epoch().count() == 0 ||
        now - last_pose_broadcast_ >= pose_broadcast_period_;
      if (publish) {last_pose_broadcast_ = now;}
      const bool moved = trajectory_.empty() ||
        std::pow(p.x - trajectory_.back()[0], 2) + std::pow(p.y - trajectory_.back()[1], 2) +
        std::pow(p.z - trajectory_.back()[2], 2) >= 0.0025;
      if (moved) {trajectory_.push_back({p.x, p.y, p.z});}
      if (trajectory_.size() > 2000U) {
        std::vector<std::array<double, 3>> compact;
        compact.reserve(1001U);
        for (std::size_t i = 0; i < trajectory_.size(); i += 2U) {compact.push_back(trajectory_[i]);}
        trajectory_ = std::move(compact);
      }
    }
    if (publish && server_) {server_->broadcastLatestText("pose", pose);}
  }

  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr message, const CloudStream stream)
  {
    // Real-time clouds are browser-demand driven. When the UI does not enable
    // "实时点云", skip conversion/encoding and do not send binary cloud traffic.
    // ROS subscriptions remain alive so feature/status detection still works.
    std::vector<std::string> live_clients;
    {
      std::lock_guard<std::mutex> lock(live_cloud_clients_mutex_);
      if (live_cloud_clients_.empty()) {return;}
      live_clients.assign(live_cloud_clients_.begin(), live_cloud_clients_.end());
    }
    const auto now = std::chrono::steady_clock::now();
    // /livox/lidar and /cloud_registered_body are body-local clouds in the deployed stack.
    // Raw lidar is only a pre-localization fallback. /cloud_registered_body is transformed
    // below by the freshest localization/mapping odometry before it is sent to the browser.
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
      if (stream == CloudStream::kLiveRegistered && transform_registered_body_cloud_) {
        geometry_msgs::msg::Pose pose;
        std::string frame_id;
        bool have_pose = false;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          const auto current = std::chrono::steady_clock::now();
          if (has_localization_pose_ && last_odometry_steady_.time_since_epoch().count() != 0 &&
            std::chrono::duration<double>(current - last_odometry_steady_).count() <= odom_freshness_s_)
          {
            pose = latest_localization_pose_;
            frame_id = latest_localization_frame_id_;
            have_pose = true;
          } else if (has_mapping_pose_ && last_mapping_odometry_steady_.time_since_epoch().count() != 0 &&
            std::chrono::duration<double>(current - last_mapping_odometry_steady_).count() <= odom_freshness_s_)
          {
            pose = latest_mapping_pose_;
            frame_id = latest_mapping_frame_id_;
            have_pose = true;
          }
        }
        if (have_pose) {
          const double qx = pose.orientation.x;
          const double qy = pose.orientation.y;
          const double qz = pose.orientation.z;
          const double qw = pose.orientation.w;
          const double norm2 = qx*qx + qy*qy + qz*qz + qw*qw;
          if (norm2 > 1.0e-12) {
            const double inv = 1.0 / std::sqrt(norm2);
            const double x = qx * inv, y = qy * inv, z = qz * inv, w = qw * inv;
            const double r00 = 1.0 - 2.0 * (y*y + z*z);
            const double r01 = 2.0 * (x*y - z*w);
            const double r02 = 2.0 * (x*z + y*w);
            const double r10 = 2.0 * (x*y + z*w);
            const double r11 = 1.0 - 2.0 * (x*x + z*z);
            const double r12 = 2.0 * (y*z - x*w);
            const double r20 = 2.0 * (x*z - y*w);
            const double r21 = 2.0 * (y*z + x*w);
            const double r22 = 1.0 - 2.0 * (x*x + y*y);
            for (auto & point : cloud.points) {
              const double px = point.x, py = point.y, pz = point.z;
              point.x = static_cast<float>(r00*px + r01*py + r02*pz + pose.position.x);
              point.y = static_cast<float>(r10*px + r11*py + r12*pz + pose.position.y);
              point.z = static_cast<float>(r20*px + r21*py + r22*pz + pose.position.z);
            }
            cloud.frame_id = frame_id.empty() ? selected_pcd_frame_id_ : frame_id;
          }
        }
      }
      const auto packets = PointCloudCodec::encode(cloud, stream, ++cloud_sequence_, cloud_max_points_);
      if (server_ && !packets.empty()) {
        for (const auto & client_id : live_clients) {
          server_->sendBinary(client_id, packets.front(), true);
        }
      }
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
      if (type == "resource.list") {sendResources(client, request_id);}
      else if (type == "map.file.select") {selectPcdFile(client, request, request_id);}
      else if (type == "localization.profile.select") {selectLocalizationProfile(client, request, request_id);}
      else if (type == "topology.file.select") {selectTopologyFile(client, request, request_id);}
      else if (type == "map.list") {sendMaps(client, request_id);}
      else if (type == "map.select") {selectMap(client, request, request_id);}
      else if (type == "map.upsert") {upsertMap(client, request, request_id);}
      else if (type == "cloud.live.set") {
        const bool enabled = request.value("enabled", false);
        {
          std::lock_guard<std::mutex> lock(live_cloud_clients_mutex_);
          if (enabled) {live_cloud_clients_.insert(client);}
          else {live_cloud_clients_.erase(client);}
        }
        if (!enabled) {
          last_raw_cloud_ = {};
          last_registered_cloud_ = {};
        }
        reply(client, request_id, true,
          enabled ? "live cloud streaming enabled for this browser" :
          "live cloud streaming disabled for this browser");
      }
      else if (type == "map.cloud_lod") {
        if (selected_pcd_path_.empty()) {throw std::runtime_error("select a PCD map first");}
        const float voxel = request.value("voxel_m", selected_map_voxel_);
        if (!std::isfinite(voxel) || voxel < map_voxel_min_ || voxel > map_voxel_max_) {
          throw std::runtime_error("map voxel size is outside the configured range");
        }
        selected_map_voxel_ = voxel;
        if (server_) {server_->broadcastText({{"type", "map.cloud.clear"},
          {"path", selected_pcd_path_.string()}});}
        streamStaticMap(selected_pcd_path_, selected_pcd_frame_id_, selected_map_voxel_);
        reply(client, request_id, true, "map downsampling applied");
      }
      else if (type == "topology.list") {sendTopologies(client, request_id);}
      else if (type == "topology.select") {selectTopology(client, request, request_id);}
      else if (type == "topology.load") {sendTopology(client, request_id);}
      else if (type == "topology.save") {saveTopology(client, request, request_id);}
      else if (type == "topology.generate") {
        startTopologyGeneration(client, request, request_id);
      }
      else if (type == "control.claim" || type == "control.heartbeat" ||
        type == "control.release")
      {
        reply(client, request_id, true, "control lease is disabled; no claim is required");
      }
      else {
        if (type == "process.start") {startProcess(client, request, request_id);}
        else if (type == "process.stop") {stopProcess(client, request, request_id);}
        else if (type == "mapping.save_map") {saveMappingMap(client, request, request_id);}
        else if (type == "localization.set_initial_pose") {setInitialPose(client, request, request_id);}
        else if (type == "localization.set_initial_pose_vertex") {
          setInitialPoseFromVertex(client, request, request_id);
        }
        else if (type == "navigation.plan") {plan(client, request, request_id);}
        else if (type == "navigation.goal") {goalOnly(client, request, request_id);}
        else if (type == "navigation.loop.start") {startLoopPatrol(client, request, request_id);}
        else if (type == "navigation.loop.stop") {stopLoopPatrol(client, request_id);}
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

  bool mappingOdometryReady() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (last_mapping_odometry_steady_.time_since_epoch().count() == 0) {return false;}
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_mapping_odometry_steady_).count() <= odom_freshness_s_;
  }

  bool serviceAvailable(const std::string & service_name) const
  {
    for (const auto & entry : get_service_names_and_types()) {
      if (entry.first == service_name) {return true;}
    }
    return false;
  }

  static bool nodeMatches(const std::vector<std::string> & nodes,
    const std::initializer_list<const char *> patterns)
  {
    for (const auto & node : nodes) {
      for (const auto * pattern : patterns) {
        if (node.find(pattern) != std::string::npos) {return true;}
      }
    }
    return false;
  }

  json detectFeatures() const
  {
    const auto nodes = get_node_names();
    const bool save_service = serviceAvailable(save_map_service_);
    const bool radar_topic = count_publishers(raw_cloud_topic_) > 0U;
    const bool localization_topic = count_publishers(odometry_topic_) > 0U;
    const bool mapping_odom = count_publishers(mapping_odometry_topic_) > 0U;
    const bool mapping_cloud = count_publishers(mapping_cloud_topic_) > 0U;
    const bool planner_nodes = nodeMatches(nodes, {
      "route3d_dijkstra_planner", "route3d_route_slicer", "route3d_pid_controller",
      "route3d_m20_adapter"});
    const bool recording_nodes = nodeMatches(nodes, {
      "route3d_online_skeleton", "route3d_data_recorder", "route3d_live"});
    const bool initializer_node = nodeMatches(nodes, {"route3d_vertex_initializer"}) ||
      count_subscribers(initializer_vertex_topic_) > 0U;
    const bool loop_patrol_node = nodeMatches(nodes, {"route3d_loop_patrol"});

    const auto radar_proc = process_manager_.snapshot("radar").running;
    const auto mapping_proc = process_manager_.snapshot("mapping").running;
    const auto localization_proc = process_manager_.snapshot("localization").running;
    const auto planner_proc = process_manager_.snapshot("planner").running;
    const auto recording_proc = process_manager_.snapshot("topology_recording").running;
    const auto save_proc = process_manager_.snapshot("map_save").running;
    const auto loop_proc = process_manager_.snapshot("loop_patrol").running;

    return {
      {"radar", {{"online", radar_proc || radar_topic}, {"managed", radar_proc},
        {"topic_publishers", count_publishers(raw_cloud_topic_)}}},
      {"mapping", {{"online", mapping_proc || (mapping_odom && mapping_cloud)},
        {"managed", mapping_proc}, {"save_service", save_service},
        {"odom_publishers", count_publishers(mapping_odometry_topic_)},
        {"cloud_publishers", count_publishers(mapping_cloud_topic_)}}},
      {"map_save", {{"online", save_proc}, {"managed", save_proc},
        {"save_service", save_service}}},
      {"localization", {{"online", localization_proc || localization_topic},
        {"managed", localization_proc}, {"odom_publishers", count_publishers(odometry_topic_)}}},
      {"planner", {{"online", planner_proc || planner_nodes}, {"managed", planner_proc}}},
      {"loop_patrol", {{"online", loop_proc || loop_patrol_node}, {"managed", loop_proc}}},
      {"topology_recording", {{"online", recording_proc || recording_nodes},
        {"managed", recording_proc}}},
      {"vertex_initializer", {{"online", initializer_node}, {"managed", false}}}
    };
  }

  json snapshot()
  {
    json processes = json::object();
    for (const auto & name : {"radar", "mapping", "map_save", "localization", "planner",
      "loop_patrol", "topology_recording", "topology_generation"})
    {
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
      {"selected_pcd", selected_pcd_path_.string()},
      {"selected_topology", selected_topology_path_.string()},
      {"map_voxel_m", selected_map_voxel_},
      {"navigation_target", navigation_target_},
      {"topology_recording_state", recording_state},
      {"topology_generation_state", topology_generation_state_},
      {"features", detectFeatures()},
      {"control_owner", ""}, {"ros", ros_status_}, {"topics", topic_health_},
      {"pose", last_pose_}, {"trajectory", trajectory_}};
  }

  void publishSnapshot() {if (server_) {server_->broadcastText(snapshot());}}

  json discoverLocalResources() const
  {
    json pcds = json::array();
    json topologies = json::array();
    json pose_sources = json::array();

    // PCD maps follow the deployment layout:
    //   <pcd_root>/<map_name>/map/<map_name>.pcd
    // The UI label is <map_name>, never the raw filename/path.
    std::error_code error;
    if (fs::is_directory(pcd_root_, error)) {
      for (const auto & entry : fs::directory_iterator(
          pcd_root_, fs::directory_options::skip_permission_denied, error))
      {
        if (error) {error.clear(); continue;}
        if (!entry.is_directory(error)) {continue;}
        const std::string map_name = entry.path().filename().string();
        const fs::path map_dir = entry.path() / "map";
        const fs::path pose_path = map_dir / "slam_data" / "trajectory" / "pose.json";
        if (fs::is_regular_file(pose_path, error)) {
          const fs::path canonical_pose = fs::weakly_canonical(pose_path, error);
          const fs::path generated = topology_root_ / map_name / "topoGraph_data.json";
          std::error_code generated_error;
          pose_sources.push_back({
            {"id", map_name}, {"label", map_name},
            {"path", error ? pose_path.string() : canonical_pose.string()},
            {"output_path", generated.string()},
            {"generated", fs::is_regular_file(generated, generated_error)}});
          error.clear();
        } else {
          error.clear();
        }
        fs::path pcd_path = map_dir / (map_name + ".pcd");

        // Compatibility fallback: if <name>.pcd does not exist, accept the only
        // PCD under the map/ subdirectory.  This keeps the folder-name selection
        // model while tolerating an older filename.
        if (!fs::is_regular_file(pcd_path, error) && fs::is_directory(map_dir, error)) {
          error.clear();
          std::vector<fs::path> candidates;
          for (const auto & file : fs::directory_iterator(
              map_dir, fs::directory_options::skip_permission_denied, error))
          {
            if (error) {error.clear(); continue;}
            if (file.is_regular_file(error) && file.path().extension() == ".pcd") {
              candidates.push_back(file.path());
            }
          }
          if (candidates.size() == 1U) {pcd_path = candidates.front();}
        }
        if (!fs::is_regular_file(pcd_path, error)) {error.clear(); continue;}
        const fs::path canonical = fs::weakly_canonical(pcd_path, error);
        pcds.push_back({
          {"id", map_name},
          {"label", map_name},
          {"path", error ? pcd_path.string() : canonical.string()}});
        error.clear();
      }
    }

    // Topology maps follow the deployment layout:
    //   <M20 workspace>/data/<name>/topoGraph_data.json
    // Other JSON files in the directory are intentionally ignored.
    error.clear();
    if (fs::is_directory(topology_root_, error)) {
      for (const auto & entry : fs::directory_iterator(
          topology_root_, fs::directory_options::skip_permission_denied, error))
      {
        if (error) {error.clear(); continue;}
        if (!entry.is_directory(error)) {continue;}
        const std::string topology_name = entry.path().filename().string();
        const fs::path path = entry.path() / "topoGraph_data.json";
        if (!fs::is_regular_file(path, error)) {error.clear(); continue;}
        try {
          const auto document = TopologyStore::load(path);
          const fs::path canonical = fs::weakly_canonical(path, error);
          topologies.push_back({
            {"id", topology_name},
            {"label", topology_name},
            {"path", error ? path.string() : canonical.string()},
            {"frame_id", document.data.value("frame_id", "")},
            {"vertices", document.data.value("vertices", json::object()).size()},
            {"edges", document.data.value("edges", json::object()).size()}});
          error.clear();
        } catch (const std::exception &) {
          // An invalid topoGraph_data.json is omitted rather than exposing a file
          // that the editor cannot safely load/save.
        }
      }
    }

    auto by_label = [](const json & a, const json & b) {
      return a.value("label", "") < b.value("label", "");
    };
    std::sort(pcds.begin(), pcds.end(), by_label);
    std::sort(topologies.begin(), topologies.end(), by_label);
    std::sort(pose_sources.begin(), pose_sources.end(), by_label);
    return {{"pcds", pcds}, {"topologies", topologies}, {"pose_sources", pose_sources}};
  }

  void sendResources(const std::string & client, const std::string & request_id)
  {
    const auto resources = discoverLocalResources();
    json profiles = json::array();
    for (const auto & map : catalog_.value("maps", json::array())) {
      if (!map.value("enabled", true)) {continue;}
      profiles.push_back({
        {"id", map.value("id", "")},
        {"label", map.value("label", map.value("id", ""))},
        {"localization_config_template", map.value("localization_config_template", "")},
        {"frame_id", map.value("frame_id", "camera_init")}});
    }
    server_->sendText(client, {
      {"type", "resource.catalog"}, {"request_id", request_id},
      {"pcds", resources.at("pcds")}, {"topologies", resources.at("topologies")},
      {"pose_sources", resources.at("pose_sources")},
      {"localization_profiles", profiles},
      {"selected_pcd", selected_pcd_path_.string()},
      {"selected_topology", selected_topology_path_.string()},
      {"selected_localization_profile", selected_map_id_},
      {"map_voxel_m", selected_map_voxel_},
      {"map_voxel_min_m", map_voxel_min_},
      {"map_voxel_max_m", map_voxel_max_}});
  }

  void selectPcdFile(
    const std::string & client, const json & request, const std::string & request_id)
  {
    if (process_manager_.snapshot("localization").running ||
      process_manager_.snapshot("planner").running ||
      process_manager_.snapshot("topology_recording").running)
    {
      throw std::runtime_error(
        "stop localization, planner and topology recording before switching the PCD map");
    }
    const fs::path path = fs::path(expandUser(request.value("path", "")));
    if (!pathInside(path, std::vector<fs::path>{pcd_root_}) ||
      !fs::is_regular_file(path) || path.extension() != ".pcd")
    {
      throw std::runtime_error("PCD path is invalid, missing or outside resources.pcd_root");
    }
    const float requested_voxel = request.value("voxel_m", selected_map_voxel_);
    if (!std::isfinite(requested_voxel) || requested_voxel < map_voxel_min_ ||
      requested_voxel > map_voxel_max_)
    {
      throw std::runtime_error("map voxel size is outside the configured range");
    }
    selected_map_voxel_ = requested_voxel;
    std::error_code error;
    selected_pcd_path_ = fs::weakly_canonical(path, error);
    if (error) {selected_pcd_path_ = path;}
    selected_pcd_frame_id_ = request.value("frame_id", "camera_init");
    for (const auto & map : catalog_.value("maps", json::array())) {
      const fs::path catalog_pcd = expandUser(map.value("pcd_path", ""));
      std::error_code compare_error;
      if (!catalog_pcd.empty() && fs::equivalent(catalog_pcd, selected_pcd_path_, compare_error)) {
        selected_pcd_frame_id_ = map.value("frame_id", selected_pcd_frame_id_);
        break;
      }
    }
    reply(client, request_id, true, "PCD map selected");
    if (server_) {
      server_->broadcastText({{"type", "map.cloud.clear"}, {"path", selected_pcd_path_.string()}});
      server_->broadcastText({{"type", "resource.selection"},
        {"selected_pcd", selected_pcd_path_.string()},
        {"selected_topology", selected_topology_path_.string()},
        {"selected_localization_profile", selected_map_id_}});
    }
    streamStaticMap(selected_pcd_path_, selected_pcd_frame_id_, selected_map_voxel_);
  }

  void selectLocalizationProfile(
    const std::string & client, const json & request, const std::string & request_id)
  {
    if (process_manager_.snapshot("localization").running) {
      throw std::runtime_error("stop localization before switching localization profile");
    }
    const std::string id = request.value("id", "");
    (void)findMap(id);  // validate enabled profile
    selected_map_id_ = id;
    reply(client, request_id, true, "localization profile selected");
    server_->broadcastText({{"type", "resource.selection"},
      {"selected_pcd", selected_pcd_path_.string()},
      {"selected_topology", selected_topology_path_.string()},
      {"selected_localization_profile", selected_map_id_}});
  }

  void publishInitializerGraphPath()
  {
    if (!initializer_graph_publisher_ || selected_topology_path_.empty()) {return;}
    std_msgs::msg::String message;
    message.data = selected_topology_path_.string();
    initializer_graph_publisher_->publish(message);
  }

  void selectTopologyFile(
    const std::string & client, const json & request, const std::string & request_id)
  {
    if (process_manager_.snapshot("planner").running ||
      process_manager_.snapshot("topology_recording").running)
    {
      throw std::runtime_error("stop planner and topology recording before switching topology");
    }
    const fs::path path = fs::path(expandUser(request.value("path", "")));
    if (!pathInside(path, std::vector<fs::path>{topology_root_}) ||
      !fs::is_regular_file(path) || path.filename() != "topoGraph_data.json")
    {
      throw std::runtime_error(
        "topology path must be <resources.topology_root>/<name>/topoGraph_data.json");
    }
    current_topology_ = TopologyStore::load(path);
    std::error_code error;
    selected_topology_path_ = fs::weakly_canonical(path, error);
    if (error) {selected_topology_path_ = path;}
    selected_topology_id_ = selected_topology_path_.string();
    publishInitializerGraphPath();
    reply(client, request_id, true, "topology file selected");
    server_->broadcastText({{"type", "topology.snapshot"}, {"data", current_topology_->data},
      {"revision", current_topology_->revision}, {"id", selected_topology_id_},
      {"path", selected_topology_path_.string()}});
    server_->broadcastText({{"type", "resource.selection"},
      {"selected_pcd", selected_pcd_path_.string()},
      {"selected_topology", selected_topology_path_.string()},
      {"selected_localization_profile", selected_map_id_}});
  }

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
    if (process_manager_.snapshot("planner").running ||
      process_manager_.snapshot("localization").running ||
      process_manager_.snapshot("topology_recording").running) {
      throw std::runtime_error("stop planner, localization and topology recording before switching maps");
    }
    const std::string id = request.value("id", "");
    const auto map = findMap(id);
    requireMapPaths(map);
    selected_map_id_ = id;
    selected_pcd_path_ = fs::path(expandUser(map.at("pcd_path").get<std::string>()));
    selected_pcd_frame_id_ = map.value("frame_id", "camera_init");
    selected_topology_id_.clear();
    selected_topology_path_.clear();
    current_topology_.reset();
    const std::string default_id = map.value("default_topology_id", "");
    if (!default_id.empty()) {
      const auto topology = findTopology(map, default_id);
      selected_topology_id_ = default_id;
      selected_topology_path_ = fs::path(expandUser(topology.at("path").get<std::string>()));
      current_topology_ = TopologyStore::load(selected_topology_path_);
      publishInitializerGraphPath();
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
    selected_topology_path_ = path;
    publishInitializerGraphPath();
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

  void streamStaticMap(const fs::path & path, const std::string & frame, const float voxel)
  {
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

  void streamStaticMap(const json & map, const float voxel)
  {
    streamStaticMap(fs::path(expandUser(map.at("pcd_path").get<std::string>())),
      map.value("frame_id", "camera_init"), voxel);
  }

  void sendTopology(const std::string & client, const std::string & request_id)
  {
    if (!current_topology_) {throw std::runtime_error("no topology selected");}
    server_->sendText(client, {{"type", "topology.snapshot"}, {"request_id", request_id},
      {"data", current_topology_->data}, {"revision", current_topology_->revision},
      {"id", selected_topology_id_}});
  }

  void saveTopology(const std::string & client, const json & request, const std::string & request_id)
  {
    if (process_manager_.snapshot("planner").running) {
      throw std::runtime_error("stop planner before saving topology so runtime graph and file stay consistent");
    }
    if (selected_topology_path_.empty() || !current_topology_) {
      throw std::runtime_error("select a local topology JSON file first");
    }
    current_topology_ = TopologyStore::save(selected_topology_path_,
      request.at("data"), request.value("revision", ""));
    publishInitializerGraphPath();
    reply(client, request_id, true, "topology saved to local file");
    server_->broadcastText({{"type", "topology.saved"}, {"data", current_topology_->data},
      {"revision", current_topology_->revision}, {"id", selected_topology_id_},
      {"path", selected_topology_path_.string()}});
  }

  void startTopologyGeneration(
    const std::string & client, const json & request, const std::string & request_id)
  {
    if (process_manager_.snapshot("topology_generation").running) {
      throw std::runtime_error("topology generation is already running");
    }
    if (process_manager_.snapshot("planner").running ||
      process_manager_.snapshot("topology_recording").running)
    {
      throw std::runtime_error(
              "stop planner and topology recording before generating a topology");
    }
    const std::string map_name = request.value("name", "");
    if (map_name.empty() || !std::regex_match(map_name, std::regex("[A-Za-z0-9._-]+")) ||
      map_name == "." || map_name == "..")
    {
      throw std::runtime_error(
              "map folder name may only contain letters, digits, dot, underscore and dash");
    }

    const fs::path pose = pcd_root_ / map_name / "map" / "slam_data" / "trajectory" /
      "pose.json";
    if (!pathInside(pose, std::vector<fs::path>{pcd_root_}) || !fs::is_regular_file(pose)) {
      throw std::runtime_error(
              "pose source is missing: <resources.pcd_root>/<name>/map/slam_data/trajectory/pose.json");
    }
    if (topology_generation_config_.empty() ||
      !fs::is_regular_file(topology_generation_config_))
    {
      throw std::runtime_error("topology.generation_config_file does not exist");
    }
    const fs::path output = topology_root_ / map_name / "topoGraph_data.json";
    if (!pathInside(output, std::vector<fs::path>{topology_root_})) {
      throw std::runtime_error("generated topology path is outside resources.topology_root");
    }
    if (fs::exists(output) && !request.value("overwrite", false)) {
      throw std::runtime_error(
              "topology already exists; enable overwrite in the web page to regenerate it");
    }
    fs::create_directories(output.parent_path());
    if (fs::exists(output)) {
      const fs::path backup_directory = output.parent_path() / ".route3d_web_backups";
      fs::create_directories(backup_directory);
      const auto suffix = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count());
      fs::copy_file(
        output, backup_directory /
        (output.filename().string() + ".before_generation." + suffix + ".bak"));
    }

    std::string command = "ros2 run route3d_odom_waypoint pose_file_to_topology " +
      shellQuote(pose.string()) + " --output " + shellQuote(output.string()) +
      " --frame-id " + shellQuote(topology_generation_frame_id_) + " --config-file " +
      shellQuote(topology_generation_config_.string());
    std::string error;
    if (!process_manager_.start(
        "topology_generation", commandPrefix() + command,
        {{"ROS_DOMAIN_ID", std::to_string(domain_id_)}}, error))
    {
      throw std::runtime_error(error);
    }
    topology_generation_name_ = map_name;
    topology_generation_output_ = output;
    topology_generation_state_ = "running";
    reply(client, request_id, true, "topology generation started: " + map_name);
  }

  void pollTopologyGeneration()
  {
    if (topology_generation_state_ != "running") {return;}
    const auto process = process_manager_.snapshot("topology_generation");
    if (process.running) {return;}

    if (process.exit_code != 0) {
      topology_generation_state_ = "error";
      if (server_) {
        server_->broadcastText({{"type", "error"},
          {"message", "轨迹生成拓扑失败，退出码 " + std::to_string(process.exit_code) +
            "；请查看 topology_generation 日志"}});
      }
      return;
    }
    try {
      auto document = TopologyStore::load(topology_generation_output_);
      current_topology_ = document;
      selected_topology_path_ = fs::weakly_canonical(topology_generation_output_);
      selected_topology_id_ = selected_topology_path_.string();
      publishInitializerGraphPath();
      topology_generation_state_ = "saved";
      if (server_) {
        server_->broadcastText({{"type", "topology.snapshot"}, {"data", document.data},
          {"revision", document.revision}, {"id", selected_topology_id_},
          {"path", selected_topology_path_.string()}});
        server_->broadcastText({{"type", "topology.generated"},
          {"name", topology_generation_name_}, {"path", selected_topology_path_.string()},
          {"vertices", document.data.value("vertices", json::object()).size()},
          {"edges", document.data.value("edges", json::object()).size()}});
        const auto resources = discoverLocalResources();
        server_->broadcastText({{"type", "resource.catalog"},
          {"pcds", resources.at("pcds")}, {"topologies", resources.at("topologies")},
          {"pose_sources", resources.at("pose_sources")},
          {"selected_pcd", selected_pcd_path_.string()},
          {"selected_topology", selected_topology_path_.string()},
          {"selected_localization_profile", selected_map_id_},
          {"map_voxel_m", selected_map_voxel_},
          {"map_voxel_min_m", map_voxel_min_}, {"map_voxel_max_m", map_voxel_max_}});
      }
    } catch (const std::exception & exception) {
      topology_generation_state_ = "error";
      if (server_) {
        server_->broadcastText({{"type", "error"},
          {"message", "生成文件无法加载：" + std::string(exception.what())}});
      }
    }
  }

  void saveMappingMap(
    const std::string & client, const json & request, const std::string & request_id)
  {
    const auto mapping = detectFeatures().at("mapping");
    if (!mapping.value("online", false) || mapping.value("odom_publishers", 0U) == 0U ||
      mapping.value("cloud_publishers", 0U) == 0U)
    {
      throw std::runtime_error(
              "mapping data is not online; start mapping (not localization) and move before saving");
    }
    if (!serviceAvailable(save_map_service_)) {
      throw std::runtime_error("/save_pcd_service is not available; start mapping first");
    }
    if (process_manager_.snapshot("map_save").running) {
      throw std::runtime_error("a map save request is already running");
    }
    const std::string map_name = validateSaveName(request.value("name", ""));
    const std::string resolution = mapping_save_resolution_tag_;
    if (!std::regex_match(resolution, std::regex("[0-9]+([.][0-9]+)?"))) {
      throw std::runtime_error("mapping.save_resolution_tag must be a positive numeric tag such as 0.1");
    }
    const fs::path map_dir = pcd_root_ / map_name / "map";
    if (!pathInside(map_dir, std::vector<fs::path>{pcd_root_})) {
      throw std::runtime_error("map save directory is outside resources.pcd_root");
    }
    fs::create_directories(map_dir);
    const fs::path filename = map_dir / (map_name + "-" + resolution);
    const fs::path generated_pcd = fs::path(filename.string() + ".pcd");
    const fs::path canonical_pcd = map_dir / (map_name + ".pcd");
    if ((fs::exists(generated_pcd) || fs::exists(canonical_pcd)) &&
      !request.value("overwrite", false))
    {
      throw std::runtime_error("map already exists; enable overwrite in the web page to replace it");
    }

    const std::string yaml = "{filename: '" + filename.string() + "'}";
    const std::string service_call = "ros2 service call " + shellQuote(save_map_service_) +
      " moveit_msgs/srv/SaveMap " + shellQuote(yaml);
    std::string command = "output=$(" + service_call + " 2>&1); rc=$?; ";
    command += "printf '%s\\n' \"$output\"; ";
    command += "if [ $rc -ne 0 ] || ! printf '%s\\n' \"$output\" | ";
    command += "grep -Eq 'success[=:][[:space:]]*(true|True)'; then ";
    command += "echo '[ERROR] save service returned failure; no PCD was generated' >&2; exit 4; fi; ";
    // Keep the user's requested filename (e.g. 510-0.1.pcd), and expose the canonical
    // <map_name>.pcd path used by the web map selector without duplicating a large PCD.
    command += "if [ ! -s " + shellQuote(generated_pcd.string()) +
      " ]; then echo '[ERROR] save service reported success but the PCD is missing or empty' "
      ">&2; exit 5; fi; ln -sfn " + shellQuote(generated_pcd.filename().string()) + " " +
      shellQuote(canonical_pcd.string());

    std::string error;
    if (!process_manager_.start("map_save", mappingCommandPrefix() + command,
      {{"ROS_DOMAIN_ID", std::to_string(domain_id_)}}, error)) {
      throw std::runtime_error(error);
    }
    reply(client, request_id, true,
      "map save started: " + filename.string() + " (PCD selector will use " +
      canonical_pcd.string() + ")");
  }

  std::string commandPrefix() const
  {
    std::string command = "source " + shellQuote(ros_setup_);
    if (!workspace_setup_.empty()) {command += " && source " + shellQuote(workspace_setup_);}
    return command + " && ";
  }

  std::string mappingCommandPrefix() const
  {
    std::string command = commandPrefix();
    if (!mapping_setup_.empty()) {
      command += "if [ -f " + shellQuote(mapping_setup_) + " ]; then source " +
        shellQuote(mapping_setup_) + "; fi; ";
    }
    return command;
  }

  void startProcess(const std::string & client, const json & request, const std::string & request_id)
  {
    const std::string target = request.value("target", "");
    std::string command;
    if (target == "radar") {
      command = radar_command_;
    } else if (target == "mapping") {
      if (detectFeatures().at("mapping").value("online", false)) {
        throw std::runtime_error("mapping is already detected as running");
      }
      const std::string config = request.value("mapping_config", mapping_default_config_);
      if (std::find(mapping_allowed_configs_.begin(), mapping_allowed_configs_.end(), config) ==
        mapping_allowed_configs_.end())
      {
        throw std::runtime_error("unsupported mapping config: " + config);
      }
      command = replaceAll(mapping_command_, "{mapping_config}", shellQuote(config));
    } else if (target == "localization") {
      if (selected_pcd_path_.empty()) {throw std::runtime_error("select a local PCD map first");}
      if (!detectFeatures().at("radar").value("online", false)) {
        throw std::runtime_error("/livox/lidar is not online; start driver.service first");
      }
      command = replaceAll(localization_command_, "{localization_config}",
        shellQuote(makeLocalizationConfig(selected_pcd_path_)));
    } else if (target == "planner") {
      if (process_manager_.snapshot("topology_recording").running) {
        throw std::runtime_error("finish topology recording before starting planner");
      }
      if (!localizationReady()) {throw std::runtime_error("fresh localization odometry is required");}
      if (!current_topology_ || selected_topology_path_.empty()) {
        throw std::runtime_error("select and load a local topology JSON first");
      }
      command = replaceAll(planner_command_, "{graph_file}",
        shellQuote(selected_topology_path_.string()));
      command = replaceAll(command, "{network_interface}", shellQuote(network_interface_));
      command = replaceAll(command, "{enable_motion}", request.value("enable_motion", false) ? "true" : "false");
    } else {
      throw std::runtime_error("invalid process target");
    }
    std::string error;
    const std::string prefix = target == "mapping" ? mappingCommandPrefix() : commandPrefix();
    if (!process_manager_.start(target, prefix + command,
      {{"ROS_DOMAIN_ID", std::to_string(domain_id_)}}, error)) {throw std::runtime_error(error);}
    reply(client, request_id, true, target + " started");
  }

  void stopProcess(const std::string & client, const json & request, const std::string & request_id)
  {
    const std::string target = request.value("target", "");
    if (target != "radar" && target != "mapping" && target != "localization" && target != "planner") {
      throw std::runtime_error("invalid process target");
    }
    if (target == "radar") {
      if (process_manager_.snapshot("planner").running) {
        throw std::runtime_error("stop planner before stopping radar");
      }
      if (process_manager_.snapshot("topology_recording").running) {
        throw std::runtime_error("finish or abort topology recording before stopping radar");
      }
      std::string error;
      // The radar is a systemd service in the deployed robot. Always stop that service,
      // even if it was started from SSH rather than by this web console.
      if (process_manager_.snapshot("radar").running) {
        process_manager_.stop("radar", stop_timeout_, error);
      }
      if (!radar_stop_command_.empty()) {
        if (!process_manager_.start("radar", commandPrefix() + radar_stop_command_,
          {{"ROS_DOMAIN_ID", std::to_string(domain_id_)}}, error)) {
          throw std::runtime_error(error);
        }
      }
      reply(client, request_id, true, "radar stop requested via driver.service");
      return;
    }
    if (target == "planner" && process_manager_.snapshot("loop_patrol").running) {
      std::string loop_error;
      process_manager_.stop("loop_patrol", stop_timeout_, loop_error);
      if (!loop_error.empty()) {
        RCLCPP_WARN(get_logger(), "failed to stop loop patrol before planner: %s", loop_error.c_str());
      }
    }
    if (!process_manager_.snapshot(target).running && detectFeatures().contains(target) &&
      detectFeatures().at(target).value("online", false)) {
      throw std::runtime_error(target + " is running outside the web console; stop it from the terminal that started it");
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
    if (!localizationReady() && !mappingOdometryReady()) {
      throw std::runtime_error("fresh localization or mapping odometry is required");
    }
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
      current_topology_ = document;
      selected_topology_path_ = output;
      selected_topology_id_ = output.string();
      publishInitializerGraphPath();
      {
        std::lock_guard<std::mutex> lock(recording_mutex_);
        recording_state_ = "saved";
        recording_save_name_.clear();
      }
      std::string ignored;
      process_manager_.writeInput("topology_recording", "q", ignored);
      if (server_) {
        server_->broadcastText({{"type", "topology.snapshot"}, {"data", document.data},
          {"revision", document.revision}, {"id", selected_topology_id_},
          {"path", selected_topology_path_.string()}});
        server_->broadcastText({{"type", "topology.record.saved"}, {"name", save_name},
          {"path", output.string()}});
        const auto resources = discoverLocalResources();
        json profiles = json::array();
        for (const auto & map : catalog_.value("maps", json::array())) {
          if (!map.value("enabled", true)) {continue;}
          profiles.push_back({{"id", map.value("id", "")},
            {"label", map.value("label", map.value("id", ""))},
            {"localization_config_template", map.value("localization_config_template", "")},
            {"frame_id", map.value("frame_id", "camera_init")}});
        }
        server_->broadcastText({{"type", "resource.catalog"},
          {"pcds", resources.at("pcds")}, {"topologies", resources.at("topologies")},
          {"pose_sources", resources.at("pose_sources")},
          {"localization_profiles", profiles},
          {"selected_pcd", selected_pcd_path_.string()},
          {"selected_topology", selected_topology_path_.string()},
          {"selected_localization_profile", selected_map_id_}});
      }
    } catch (const std::exception & exception) {
      std::lock_guard<std::mutex> lock(recording_mutex_);
      recording_state_ = "error";
      if (server_) {server_->broadcastText({{"type", "error"}, {"message", exception.what()}});}
    }
  }

  fs::path localizationTemplateForPcd(const fs::path & pcd_path) const
  {
    // Prefer a legacy catalog entry that exactly matches the selected PCD. This preserves
    // map-specific localization tuning without exposing a separate template selector in UI.
    for (const auto & profile : catalog_.value("maps", json::array())) {
      if (!profile.value("enabled", true)) {continue;}
      const fs::path candidate_pcd = expandUser(profile.value("pcd_path", ""));
      std::error_code error;
      if (!candidate_pcd.empty() && fs::exists(candidate_pcd, error) && fs::exists(pcd_path, error) &&
        fs::equivalent(candidate_pcd, pcd_path, error) && !error)
      {
        const fs::path candidate = expandUser(profile.value("localization_config_template", ""));
        if (!candidate.empty() && fs::is_regular_file(candidate)) {return candidate;}
      }
    }
    return localization_default_config_template_;
  }

  std::string makeLocalizationConfig(const fs::path & pcd_path)
  {
    const fs::path source = localizationTemplateForPcd(pcd_path);
    if (!pathInside(source, allowed_localization_roots_)) {
      throw std::runtime_error("localization template is outside allowed roots: " + source.string());
    }
    if (!fs::is_regular_file(source)) {
      throw std::runtime_error("localization template does not exist: " + source.string());
    }
    std::istringstream input(readText(source));
    std::ostringstream output;
    std::string line;
    bool changed = false;
    while (std::getline(input, line)) {
      const auto key = line.find("global_map_path:");
      if (key != std::string::npos) {
        line = line.substr(0, key) + "global_map_path: " + pcd_path.string();
        changed = true;
      }
      output << line << '\n';
    }
    if (!changed) {throw std::runtime_error("localization template has no global_map_path key");}
    const fs::path directory = cache_directory_ / "runtime";
    fs::create_directories(directory);
    const std::string map_name = pcd_path.parent_path().parent_path().filename().string();
    const fs::path result = directory / ((map_name.empty() ? "selected_map" : map_name) + "_localization.yaml");
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

  void setInitialPoseFromVertex(
    const std::string & client, const json & request, const std::string & request_id)
  {
    if (!current_topology_ || selected_topology_path_.empty()) {
      throw std::runtime_error("select a topology JSON before using vertex initialization");
    }
    const int vertex_id = request.at("vertex_id").get<int>();
    const std::string key = std::to_string(vertex_id);
    const auto & vertices = current_topology_->data.at("vertices");
    if (!vertices.contains(key)) {
      throw std::runtime_error("vertex " + key + " does not exist in selected topology");
    }

    publishInitializerGraphPath();
    if (initializer_vertex_publisher_ && initializer_vertex_publisher_->get_subscription_count() > 0U) {
      std_msgs::msg::Int32 message;
      message.data = vertex_id;
      initializer_vertex_publisher_->publish(message);
      reply(client, request_id, true,
        "vertex initial-pose request published through route3d_vertex_initializer");
      return;
    }

    // Fallback keeps the web function usable when the helper package is not running.
    const auto & vertex = vertices.at(key);
    const auto pos = vertex.at("pos");
    const auto rpy = vertex.at("rpy");
    const double roll = rpy.at(0).get<double>();
    const double pitch = rpy.at(1).get<double>();
    const double yaw = rpy.at(2).get<double>();
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);

    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.stamp = now();
    message.header.frame_id = initial_pose_frame_id_;
    message.pose.pose.position.x = pos.at(0).get<double>();
    message.pose.pose.position.y = pos.at(1).get<double>();
    message.pose.pose.position.z = pos.at(2).get<double>();
    message.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy;
    message.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy;
    message.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy;
    message.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy;
    message.pose.covariance[0] = xy_variance_;
    message.pose.covariance[7] = xy_variance_;
    message.pose.covariance[14] = xy_variance_;
    message.pose.covariance[21] = yaw_variance_;
    message.pose.covariance[28] = yaw_variance_;
    message.pose.covariance[35] = yaw_variance_;
    initial_pose_publisher_->publish(message);
    RCLCPP_WARN(get_logger(),
      "route3d_vertex_initializer has no subscriber; published vertex %d directly from web console",
      vertex_id);
    reply(client, request_id, true, "vertex initial pose published directly (initializer not running)");
  }

  json navigationTargetForVertex(const int vertex_id) const
  {
    if (!current_topology_) {throw std::runtime_error("select a topology before sending a goal");}
    const std::string key = std::to_string(vertex_id);
    const auto & vertices = current_topology_->data.at("vertices");
    if (!vertices.contains(key)) {throw std::runtime_error("goal vertex " + key + " does not exist");}
    const auto & vertex = vertices.at(key);
    const auto position = vertex.value("pos", json::array());
    const auto rpy = vertex.value("rpy", json::array());
    if (!position.is_array() || position.size() != 3U) {
      throw std::runtime_error("goal vertex " + key + " has invalid position");
    }
    const double yaw = rpy.is_array() && rpy.size() >= 3U ? rpy[2].get<double>() : 0.0;
    return {{"type", "navigation.target"}, {"vertex_id", vertex_id}, {"position", position},
      {"yaw", yaw}, {"frame_id", current_topology_->data.value("frame_id", "camera_init")}};
  }

  void selectNavigationTarget(const int vertex_id)
  {
    auto target = navigationTargetForVertex(vertex_id);
    {std::lock_guard<std::mutex> lock(state_mutex_); navigation_target_ = target;}
    if (server_) {server_->broadcastText(target);}
  }

  void plan(const std::string & client, const json & request, const std::string & request_id)
  {
    if (!detectFeatures().at("planner").value("online", false)) {
      throw std::runtime_error("planner is not running");
    }
    const int start_id = request.at("start_id").get<int>();
    const int goal_id = request.at("goal_id").get<int>();
    if (start_id < 0 || goal_id < 0 || start_id == goal_id) {
      throw std::runtime_error("start_id and goal_id must be different non-negative integers");
    }
    selectNavigationTarget(goal_id);
    std_msgs::msg::Int32MultiArray message;
    message.data = {start_id, goal_id};
    plan_publisher_->publish(message);
    reply(client, request_id, true, "plan request published");
  }

  void goalOnly(const std::string & client, const json & request, const std::string & request_id)
  {
    if (!detectFeatures().at("planner").value("online", false)) {
      throw std::runtime_error("planner is not running");
    }
    const int goal_id = request.at("goal_id").get<int>();
    if (goal_id < 0) {throw std::runtime_error("goal_id must be a non-negative integer");}
    selectNavigationTarget(goal_id);
    std_msgs::msg::Int32 message;
    message.data = goal_id;
    goal_publisher_->publish(message);
    reply(client, request_id, true,
      "goal-only request published; start vertex will be matched from current robot position");
  }

  fs::path writeLoopPatrolConfig(
    const int start_id, const int goal_id, const std::string & mode,
    const double dwell_time_s, const int max_round_trips)
  {
    if (start_id < 0 || goal_id < 0 || start_id == goal_id) {
      throw std::runtime_error("loop patrol requires two different non-negative vertex ids");
    }
    if (!std::isfinite(dwell_time_s) || dwell_time_s < 0.0 || dwell_time_s > 3600.0) {
      throw std::runtime_error("dwell_time_s must be between 0 and 3600 seconds");
    }
    if (max_round_trips < 0) {
      throw std::runtime_error("max_round_trips must be >= 0 (0 means infinite)");
    }
    fs::create_directories(cache_directory_ / "loop_patrol");
    const fs::path config = cache_directory_ / "loop_patrol" / "web_loop_patrol.yaml";
    std::ofstream stream(config, std::ios::trunc);
    if (!stream) {throw std::runtime_error("cannot create loop patrol runtime config");}
    stream << "route3d_loop_patrol:\n"
           << "  ros__parameters:\n"
           << "    patrol:\n"
           << "      start_vertex_id: " << start_id << "\n"
           << "      goal_vertex_id: " << goal_id << "\n"
           << "      initial_direction: start_to_goal\n";
    if (mode == "goal_only") {
      stream << "      request_mode: goal_only\n";
    }
    stream << "      auto_start: true\n"
           << "      wait_for_idle_before_first_request: true\n"
           << "      startup_delay_s: 0.5\n"
           << "      dwell_time_s: " << dwell_time_s << "\n"
           << "      max_round_trips: " << max_round_trips << "\n"
           << "    timeouts:\n"
           << "      plan_response_s: 10.0\n"
           << "      route_activation_s: 10.0\n"
           << "      arrival_s: 0.0\n"
           << "    topics:\n"
           << "      plan_request: /route3d_dijkstra/plan_request\n"
           << "      goal_request: /route3d_dijkstra/goal_request\n"
           << "      dijkstra_status: /route3d_dijkstra/status\n"
           << "      sliced_route: /route3d_route_slicer/tasks\n"
           << "      controller_status: /route3d_pid_controller/status\n"
           << "      status: /route3d_loop_patrol/status\n";
    stream.flush();
    if (!stream) {throw std::runtime_error("failed to write loop patrol runtime config");}
    return config;
  }

  void startLoopPatrol(
    const std::string & client, const json & request, const std::string & request_id)
  {
    if (!detectFeatures().at("planner").value("online", false)) {
      throw std::runtime_error("planner/control chain is not running");
    }
    if (process_manager_.snapshot("loop_patrol").running) {
      throw std::runtime_error("loop patrol is already running");
    }
    const int start_id = request.at("start_id").get<int>();
    const int goal_id = request.at("goal_id").get<int>();
    const std::string mode = request.value("mode", "fixed");
    if (mode != "fixed" && mode != "goal_only") {
      throw std::runtime_error("loop patrol mode must be fixed or goal_only");
    }
    const double dwell_time_s = request.value("dwell_time_s", 2.0);
    const int max_round_trips = request.value("max_round_trips", 0);
    const fs::path config = writeLoopPatrolConfig(
      start_id, goal_id, mode, dwell_time_s, max_round_trips);
    selectNavigationTarget(goal_id);
    const fs::path script = mode == "goal_only" ? goal_only_loop_patrol_script_ : loop_patrol_script_;
    if (!fs::is_regular_file(script)) {
      throw std::runtime_error("loop patrol script does not exist: " + script.string());
    }
    std::string error;
    const std::string command = shellQuote(script.string()) + " " + shellQuote(config.string());
    if (!process_manager_.start("loop_patrol", commandPrefix() + command,
      {{"ROS_DOMAIN_ID", std::to_string(domain_id_)}}, error))
    {
      throw std::runtime_error(error);
    }
    reply(client, request_id, true,
      std::string("loop patrol started: ") + (mode == "goal_only" ? "goal-only" : "fixed start/goal"));
  }

  void stopLoopPatrol(const std::string & client, const std::string & request_id)
  {
    std::string error;
    if (!process_manager_.stop("loop_patrol", stop_timeout_, error)) {
      throw std::runtime_error(error);
    }
    reply(client, request_id, true, "loop patrol stopped");
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
  fs::path pcd_root_;
  fs::path topology_root_;
  json catalog_;
  std::string selected_map_id_;  // localization profile id (legacy catalog entry)
  std::string selected_topology_id_;
  fs::path selected_pcd_path_;
  std::string selected_pcd_frame_id_{"camera_init"};
  fs::path selected_topology_path_;
  std::optional<TopologyDocument> current_topology_;
  std::size_t resource_scan_max_files_{1000};
  int resource_scan_max_depth_{6};

  std::string raw_cloud_topic_, registered_cloud_topic_, mapping_cloud_topic_, global_map_topic_;
  std::string odometry_topic_, mapping_odometry_topic_, initial_pose_topic_;
  std::string initializer_graph_topic_, initializer_vertex_topic_;
  std::string plan_topic_, goal_topic_, pause_service_, resume_service_, cancel_service_, save_map_service_;
  std::string ros_setup_, workspace_setup_, radar_command_, radar_stop_command_, localization_command_, planner_command_;
  fs::path loop_patrol_script_, goal_only_loop_patrol_script_;
  std::string mapping_command_, mapping_setup_, mapping_default_config_, mapping_save_resolution_tag_;
  std::vector<std::string> mapping_allowed_configs_;
  std::string topology_record_command_;
  fs::path topology_record_config_template_;
  fs::path topology_results_root_;
  double topology_record_sync_slop_s_{0.15};
  fs::path topology_generation_config_;
  std::string topology_generation_frame_id_{"camera_init"};
  std::string topology_generation_name_;
  fs::path topology_generation_output_;
  std::string topology_generation_state_{"idle"};
  std::string network_interface_;
  int domain_id_{42};
  float cloud_voxel_{0.12F}, cloud_range_{20.0F};
  std::size_t cloud_max_points_{20000}, chunk_points_{32768};
  std::vector<float> map_lods_;
  float selected_map_voxel_{0.2F};
  json navigation_target_;
  float map_voxel_min_{0.01F};
  float map_voxel_max_{2.0F};
  std::chrono::duration<double> cloud_period_{0.2};
  std::chrono::duration<double> pose_broadcast_period_{1.0 / 30.0};
  std::chrono::duration<double> velocity_broadcast_period_{0.1};
  std::chrono::duration<double> status_broadcast_period_{0.1};
  std::chrono::duration<double> registered_cloud_fallback_timeout_{1.0};
  std::chrono::steady_clock::time_point last_raw_cloud_{}, last_registered_cloud_{};
  std::atomic<std::uint32_t> cloud_sequence_{0};
  mutable std::mutex live_cloud_clients_mutex_;
  std::set<std::string> live_cloud_clients_;
  std::chrono::milliseconds stop_timeout_{5000};
  std::chrono::duration<double> lease_timeout_{3.0};
  double odom_freshness_s_{0.75}, xy_variance_{0.25}, yaw_variance_{0.0685};
  double initial_pose_z_{0.0};
  std::string initial_pose_frame_id_{"map"};
  fs::path localization_default_config_template_;
  bool transform_registered_body_cloud_{true};

  mutable std::mutex state_mutex_;
  std::string control_owner_;
  std::chrono::steady_clock::time_point control_heartbeat_{};
  std::chrono::steady_clock::time_point last_odometry_steady_{};
  std::chrono::steady_clock::time_point last_mapping_odometry_steady_{};
  std::chrono::steady_clock::time_point last_pose_broadcast_{};
  std::chrono::steady_clock::time_point last_velocity_broadcast_{};
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_status_broadcast_;
  std::unordered_map<std::string, json> last_status_broadcast_values_;
  geometry_msgs::msg::Pose latest_localization_pose_{};
  geometry_msgs::msg::Pose latest_mapping_pose_{};
  std::string latest_localization_frame_id_, latest_mapping_frame_id_;
  bool has_localization_pose_{false}, has_mapping_pose_{false};
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
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mapping_odometry_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr selected_command_subscription_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> status_subscriptions_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr initializer_graph_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr initializer_vertex_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr plan_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr goal_publisher_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pause_client_, resume_client_, cancel_client_;
  rclcpp::TimerBase::SharedPtr status_timer_, lease_timer_, recording_timer_,
    topology_generation_timer_;
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

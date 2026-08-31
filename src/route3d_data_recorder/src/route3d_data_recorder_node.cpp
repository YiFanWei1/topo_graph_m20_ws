#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace fs = std::filesystem;

namespace
{

using Odometry = nav_msgs::msg::Odometry;
using PointCloud = sensor_msgs::msg::PointCloud2;
using Point3 = std::array<float, 3>;

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

double quaternionNorm(const geometry_msgs::msg::Quaternion & q)
{
  return std::sqrt(
    q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}

bool finiteQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::isfinite(q.x) && std::isfinite(q.y) &&
         std::isfinite(q.z) && std::isfinite(q.w);
}

class Route3DDataRecorder final : public rclcpp::Node
{
public:
  Route3DDataRecorder()
  : Node("route3d_data_recorder")
  {
    const auto output_text = declare_parameter<std::string>("output_directory", "map_data");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/lio_odom");
    const auto cloud_topic = declare_parameter<std::string>(
      "cloud_topic", "/cloud_registered_body");
    synchronized_pose_topic_ = declare_parameter<std::string>(
      "synchronized_pose_topic", "/route3d/synchronized_pose");
    queue_size_ = static_cast<int>(declare_parameter<int>("queue_size", 100));
    sync_slop_s_ = declare_parameter<double>("sync_slop_s", 0.05);
    expected_odom_frame_ = declare_parameter<std::string>("expected_odom_frame", "");
    expected_odom_child_frame_ = declare_parameter<std::string>("expected_odom_child_frame", "");
    expected_cloud_frame_ = declare_parameter<std::string>("expected_cloud_frame", "");
    odom_topic_ = odom_topic;
    cloud_topic_ = cloud_topic;

    if (queue_size_ < 2 || !std::isfinite(sync_slop_s_) || sync_slop_s_ < 0.0) {
      throw std::invalid_argument("queue_size must be >= 2 and sync_slop_s must be non-negative");
    }

    output_directory_ = fs::absolute(fs::path(output_text));
    prepareOutputDirectory();
    writeManifest("recording");

    auto qos = rclcpp::SensorDataQoS();
    odom_subscription_ = create_subscription<Odometry>(
      odom_topic, qos,
      [this](const Odometry::ConstSharedPtr message) { onOdometry(message); });
    cloud_subscription_ = create_subscription<PointCloud>(
      cloud_topic, qos,
      [this](const PointCloud::ConstSharedPtr message) { onPointCloud(message); });
    synchronized_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      synchronized_pose_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      get_logger(),
      "recording synchronized route data: odom=%s cloud=%s slop=%.3fs output=%s",
      odom_topic_.c_str(), cloud_topic_.c_str(), sync_slop_s_, output_directory_.c_str());
  }

  ~Route3DDataRecorder() override
  {
    try {
      finalize();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "failed to finalize route data: %s", error.what());
    }
  }

private:
  void prepareOutputDirectory()
  {
    if (fs::exists(output_directory_) && !fs::is_directory(output_directory_)) {
      throw std::runtime_error("output_directory is not a directory: " + output_directory_.string());
    }
    fs::create_directories(output_directory_ / "key_frames");

    const bool has_pose = fs::exists(output_directory_ / "pose.json");
    const bool has_manifest = fs::exists(output_directory_ / "manifest.json");
    bool has_frames = false;
    for (const auto & entry : fs::directory_iterator(output_directory_ / "key_frames")) {
      (void)entry;
      has_frames = true;
      break;
    }
    if (has_pose || has_manifest || has_frames) {
      throw std::runtime_error(
        "output directory already contains route data; choose another directory");
    }
    if (fs::exists(output_directory_ / "pose.json.partial") ||
      fs::exists(output_directory_ / "manifest.json.tmp"))
    {
      throw std::runtime_error("incomplete recorder files already exist in output directory");
    }

    pose_stream_.open(output_directory_ / "pose.json.partial", std::ios::out | std::ios::trunc);
    if (!pose_stream_.good()) {
      throw std::runtime_error("cannot create pose.json.partial");
    }
    pose_stream_ << "[\n";
  }

  void onOdometry(const Odometry::ConstSharedPtr & message)
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      odometry_queue_.push_back(message);
      trimQueuesLocked();
    }
    processMatchingPairs();
  }

  void onPointCloud(const PointCloud::ConstSharedPtr & message)
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      cloud_queue_.push_back(message);
      trimQueuesLocked();
    }
    processMatchingPairs();
  }

  void trimQueuesLocked()
  {
    while (static_cast<int>(odometry_queue_.size()) > queue_size_) {
      odometry_queue_.pop_front();
      ++dropped_odometry_;
    }
    while (static_cast<int>(cloud_queue_.size()) > queue_size_) {
      cloud_queue_.pop_front();
      ++dropped_clouds_;
    }
  }

  void processMatchingPairs()
  {
    std::unique_lock<std::mutex> process_lock(process_mutex_, std::try_to_lock);
    if (!process_lock.owns_lock()) {
      return;
    }

    while (rclcpp::ok()) {
      Odometry::ConstSharedPtr odometry;
      PointCloud::ConstSharedPtr cloud;
      double sync_delta = 0.0;
      {
        std::lock_guard<std::mutex> queue_lock(queue_mutex_);
        if (odometry_queue_.empty() || cloud_queue_.empty()) {
          return;
        }

        double best_delta = std::numeric_limits<double>::max();
        size_t best_odom = 0U;
        size_t best_cloud = 0U;
        for (size_t cloud_index = 0U; cloud_index < cloud_queue_.size(); ++cloud_index) {
          for (size_t odom_index = 0U; odom_index < odometry_queue_.size(); ++odom_index) {
            const double delta = std::abs(
              stampSeconds(cloud_queue_[cloud_index]->header.stamp) -
              stampSeconds(odometry_queue_[odom_index]->header.stamp));
            if (delta < best_delta) {
              best_delta = delta;
              best_odom = odom_index;
              best_cloud = cloud_index;
            }
          }
        }

        if (best_delta > sync_slop_s_) {
          // Do not discard a potentially matchable newest message. Once a queue is full,
          // trimQueuesLocked() will discard the oldest side and allow the stream to recover.
          return;
        }

        odometry = odometry_queue_[best_odom];
        cloud = cloud_queue_[best_cloud];
        sync_delta = best_delta;
        odometry_queue_.erase(odometry_queue_.begin() + static_cast<std::ptrdiff_t>(best_odom));
        cloud_queue_.erase(cloud_queue_.begin() + static_cast<std::ptrdiff_t>(best_cloud));
      }

      recordPair(odometry, cloud, sync_delta);
    }
  }

  bool validateFrames(const Odometry & odometry, const PointCloud & cloud)
  {
    if (odom_frame_.empty()) {
      odom_frame_ = odometry.header.frame_id;
      odom_child_frame_ = odometry.child_frame_id;
      cloud_frame_ = cloud.header.frame_id;
      if ((!expected_odom_frame_.empty() && odom_frame_ != expected_odom_frame_) ||
        (!expected_odom_child_frame_.empty() && odom_child_frame_ != expected_odom_child_frame_) ||
        (!expected_cloud_frame_.empty() && cloud_frame_ != expected_cloud_frame_))
      {
        RCLCPP_ERROR(
          get_logger(),
          "frame mismatch: odom=%s child=%s cloud=%s; check expected_* parameters",
          odom_frame_.c_str(), odom_child_frame_.c_str(), cloud_frame_.c_str());
        return false;
      }
    }
    if (odometry.header.frame_id != odom_frame_ ||
      odometry.child_frame_id != odom_child_frame_ || cloud.header.frame_id != cloud_frame_)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "frame IDs changed; ignoring mismatched frame");
      return false;
    }
    return true;
  }

  void recordPair(
    const Odometry::ConstSharedPtr & odometry,
    const PointCloud::ConstSharedPtr & cloud,
    const double sync_delta)
  {
    if (finalized_ || !validateFrames(*odometry, *cloud)) {
      return;
    }

    const auto & pose = odometry->pose.pose;
    const double norm = quaternionNorm(pose.orientation);
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z) || !finiteQuaternion(pose.orientation) || norm < 1.0e-9)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "ignoring invalid odometry pose");
      return;
    }

    std::vector<Point3> points;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
      points.reserve(static_cast<size_t>(cloud->width) * static_cast<size_t>(cloud->height));
      for (; x != x.end(); ++x, ++y, ++z) {
        if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
          points.push_back({*x, *y, *z});
        } else {
          ++rejected_nonfinite_points_;
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "cannot read cloud x/y/z fields: %s", error.what());
      return;
    }

    const size_t index = frame_count_;
    writePcd(index, points);
    const double stamp = stampSeconds(odometry->header.stamp);
    if (frame_count_ > 0U) {
      pose_stream_ << ",\n";
    }
    const nlohmann::json row = {
      stamp,
      pose.position.x,
      pose.position.y,
      pose.position.z,
      pose.orientation.x / norm,
      pose.orientation.y / norm,
      pose.orientation.z / norm,
      pose.orientation.w / norm};
    pose_stream_ << "  " << row.dump();
    pose_stream_.flush();
    if (!pose_stream_.good()) {
      throw std::runtime_error("failed to append pose.json.partial");
    }

    if (frame_count_ == 0U) {
      first_stamp_ = stamp;
    }
    last_stamp_ = stamp;
    ++frame_count_;
    written_points_ += points.size();
    writeManifest("recording");

    // Publish only after the pose row and matching PCD have been committed. Consumers of
    // this topic can therefore build a live skeleton whose every input frame is traceable
    // to pose.json and key_frames/<index>.pcd.
    geometry_msgs::msg::PoseStamped synchronized_pose;
    synchronized_pose.header = odometry->header;
    synchronized_pose.pose = pose;
    synchronized_pose.pose.orientation.x /= norm;
    synchronized_pose.pose.orientation.y /= norm;
    synchronized_pose.pose.orientation.z /= norm;
    synchronized_pose.pose.orientation.w /= norm;
    synchronized_pose_publisher_->publish(synchronized_pose);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "recorded %zu synchronized frames, latest cloud points=%zu sync_delta=%.4fs",
      frame_count_, points.size(), sync_delta);
  }

  void writePcd(const size_t index, const std::vector<Point3> & points)
  {
    const fs::path final_path = output_directory_ / "key_frames" / (std::to_string(index) + ".pcd");
    const fs::path temporary_path = final_path.string() + ".tmp";
    std::ofstream output(temporary_path, std::ios::binary | std::ios::trunc);
    if (!output.good()) {
      throw std::runtime_error("cannot create PCD file: " + temporary_path.string());
    }
    output << "# .PCD v0.7 - Point Cloud Data file format\n"
           << "VERSION 0.7\n"
           << "FIELDS x y z\n"
           << "SIZE 4 4 4\n"
           << "TYPE F F F\n"
           << "COUNT 1 1 1\n"
           << "WIDTH " << points.size() << "\n"
           << "HEIGHT 1\n"
           << "VIEWPOINT 0 0 0 1 0 0 0\n"
           << "POINTS " << points.size() << "\n"
           << "DATA binary\n";
    if (!points.empty()) {
      output.write(
        reinterpret_cast<const char *>(points.data()),
        static_cast<std::streamsize>(points.size() * sizeof(Point3)));
    }
    output.flush();
    output.close();
    if (!output.good()) {
      throw std::runtime_error("failed to write PCD file: " + temporary_path.string());
    }
    fs::rename(temporary_path, final_path);
  }

  void writeManifest(const std::string & status)
  {
    const nlohmann::json manifest = {
      {"format", "route3d_map_data"},
      {"version", 1},
      {"status", status},
      {"odom_topic", odom_topic_},
      {"cloud_topic", cloud_topic_},
      {"synchronized_pose_topic", synchronized_pose_topic_},
      {"pose_source", "realtime_lio_odom"},
      {"odom_frame", odom_frame_},
      {"odom_child_frame", odom_child_frame_},
      {"cloud_frame", cloud_frame_},
      {"sync_policy", "approximate_header_stamp"},
      {"sync_slop_s", sync_slop_s_},
      {"z_semantics", "body_pose"},
      {"pcd_semantics", "points_in_cloud_frame"},
      {"synchronized_frames", frame_count_},
      {"written_points", written_points_},
      {"rejected_nonfinite_points", rejected_nonfinite_points_},
      {"dropped_odometry_messages", dropped_odometry_},
      {"dropped_cloud_messages", dropped_clouds_},
      {"first_stamp_s", frame_count_ > 0U ? first_stamp_ : 0.0},
      {"last_stamp_s", frame_count_ > 0U ? last_stamp_ : 0.0}};
    const fs::path temporary = output_directory_ / "manifest.json.tmp";
    std::ofstream output(temporary, std::ios::out | std::ios::trunc);
    output << std::setw(2) << manifest << '\n';
    output.flush();
    output.close();
    if (!output.good()) {
      throw std::runtime_error("failed to write manifest.json.tmp");
    }
    fs::rename(temporary, output_directory_ / "manifest.json");
  }

  void finalize()
  {
    if (finalized_) {
      return;
    }
    finalized_ = true;
    if (pose_stream_.is_open()) {
      pose_stream_ << "\n]\n";
      pose_stream_.flush();
      pose_stream_.close();
      fs::rename(output_directory_ / "pose.json.partial", output_directory_ / "pose.json");
    }
    writeManifest("complete");
    RCLCPP_INFO(
      get_logger(), "recording complete: %zu frames, %zu points, output=%s",
      frame_count_, written_points_, output_directory_.c_str());
  }

  fs::path output_directory_;
  std::string odom_topic_;
  std::string cloud_topic_;
  std::string synchronized_pose_topic_;
  std::string odom_frame_;
  std::string odom_child_frame_;
  std::string cloud_frame_;
  std::string expected_odom_frame_;
  std::string expected_odom_child_frame_;
  std::string expected_cloud_frame_;
  int queue_size_{100};
  double sync_slop_s_{0.05};
  bool finalized_{false};
  double first_stamp_{0.0};
  double last_stamp_{0.0};
  size_t frame_count_{0U};
  size_t written_points_{0U};
  size_t rejected_nonfinite_points_{0U};
  size_t dropped_odometry_{0U};
  size_t dropped_clouds_{0U};
  std::ofstream pose_stream_;
  std::deque<Odometry::ConstSharedPtr> odometry_queue_;
  std::deque<PointCloud::ConstSharedPtr> cloud_queue_;
  std::mutex queue_mutex_;
  std::mutex process_mutex_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<PointCloud>::SharedPtr cloud_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr synchronized_pose_publisher_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<Route3DDataRecorder>();
    rclcpp::spin(node);
    node.reset();
  } catch (const std::exception & error) {
    std::cerr << "route3d_data_recorder: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

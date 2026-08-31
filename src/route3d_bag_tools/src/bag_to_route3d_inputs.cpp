// 本程序是 Route3D 离线构图流程的第一阶段：
// 1. 从 rosbag2 中读取里程计和机体系关键帧点云；
// 2. 通过消息 header 的纳秒时间戳进行一一精确匹配；
// 3. 把匹配后的姿态写入 pose.json，把点云写入 key_frames/<index>.pcd；
// 4. 后续 route_graph_builder_3d 依赖相同的 index 找到姿态和对应点云。
//
// 这个程序只负责准备构图输入，不会在这里创建 Route3D 节点、顺序边或 shortcut 边。

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace fs = std::filesystem;

// 文件内辅助类型和函数不需要暴露给其他编译单元，因此放入匿名命名空间，
// 避免与其他源文件中同名符号发生链接冲突。
namespace
{

// 命令行参数解析后的统一配置。
struct Options
{
  fs::path bag_path;      // rosbag2目录，例如 /home/wei/bag/regu。
  fs::path output_path;   // 导出目录；为避免误覆盖，要求该目录尚不存在。
  std::string odom_topic{"/lio_odom"};  // 提供全局位置和机体姿态。
  std::string cloud_topic{"/cloud_registered_body"};  // base_link下的关键帧配准点云。
};

// pose.json中一行姿态的内存表示。
// stamp_ns保留整数纳秒，避免在消息匹配阶段使用浮点秒造成精度损失。
struct PoseRow
{
  int64_t stamp_ns{0};  // ROS header时间戳，单位ns，也是里程计与点云的匹配键。
  double tx{0.0};       // base_link原点在里程计全局坐标系中的X。
  double ty{0.0};       // base_link原点在里程计全局坐标系中的Y。
  double tz{0.0};       // base_link原点高度，不是地面高度。
  double qx{0.0};       // 以下四项是归一化后的base_link姿态四元数。
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

// 在argv中查找形如“--name value”的参数并返回value。
// 未找到时返回fallback；参数是否必须存在由parseOptions()单独检查。
std::string argumentValue(
  const int argc, char ** argv, const std::string & name, const std::string & fallback = "")
{
  // i + 1 < argc确保读取argv[i + 1]时不会越界。
  for (int i = 1; i + 1 < argc; ++i) {
    if (argv[i] == name) {
      return argv[i + 1];
    }
  }
  return fallback;
}

// 只判断命令行中是否出现指定参数，不读取其后面的值。
bool hasArgument(const int argc, char ** argv, const std::string & name)
{
  for (int i = 1; i < argc; ++i) {
    if (argv[i] == name) {
      return true;
    }
  }
  return false;
}

// 解析并校验程序启动参数。
Options parseOptions(const int argc, char ** argv)
{
  // --bag与--output是必须参数；--help只打印说明，不执行导出。
  if (hasArgument(argc, argv, "--help") ||
    !hasArgument(argc, argv, "--bag") || !hasArgument(argc, argv, "--output"))
  {
    std::cout <<
      "Usage: bag_to_route3d_inputs --bag BAG_PATH --output OUTPUT_DIR\n"
      "       [--odom-topic /lio_odom] [--cloud-topic /cloud_registered_body]\n";
    if (hasArgument(argc, argv, "--help")) {
      std::exit(0);
    }
    throw std::invalid_argument("--bag and --output are required");
  }

  Options options;
  // 话题参数没有显式传入时，保留Options结构体中的默认值。
  options.bag_path = argumentValue(argc, argv, "--bag");
  options.output_path = argumentValue(argc, argv, "--output");
  options.odom_topic = argumentValue(argc, argv, "--odom-topic", options.odom_topic);
  options.cloud_topic = argumentValue(argc, argv, "--cloud-topic", options.cloud_topic);
  return options;
}

// 将ROS的“秒+纳秒”时间戳合并成单个整数纳秒。
// 使用int64_t能够直接作为unordered_map键，执行完全相等匹配。
int64_t stampNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);
}

// rosbag2 Reader返回的是序列化字节，本模板把字节恢复成指定ROS消息类型。
// 调用方已经通过StorageFilter限制了话题，因此这里直接按MessageT反序列化。
template<typename MessageT>
MessageT deserialize(const rosbag2_storage::SerializedBagMessage & bag_message)
{
  rclcpp::SerializedMessage serialized(*bag_message.serialized_data);
  rclcpp::Serialization<MessageT> serializer;
  MessageT message;
  serializer.deserialize_message(&serialized, &message);
  return message;
}

// 从Odometry消息提取后续构图所需的时间、平移和旋转。
PoseRow poseRow(const nav_msgs::msg::Odometry & odom)
{
  const auto & position = odom.pose.pose.position;
  const auto & orientation = odom.pose.pose.orientation;

  // 正常四元数的模应接近1；这里重新归一化，避免传感器或存储误差使后续旋转矩阵失真。
  const double quaternion_norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);

  // XYZ必须是有限值，四元数也不能含NaN/Inf或接近零向量。
  if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
    !std::isfinite(quaternion_norm) || quaternion_norm < 1.0e-9)
  {
    throw std::runtime_error("Odometry contains an invalid pose");
  }

  return {
    stampNanoseconds(odom.header.stamp),
    position.x, position.y, position.z,
    // pose.json只保存归一化后的四元数。
    orientation.x / quaternion_norm,
    orientation.y / quaternion_norm,
    orientation.z / quaternion_norm,
    orientation.w / quaternion_norm
  };
}

// 把PointCloud2写成只含XYZ的PCD v0.7 binary文件。
// 返回成功写出的有限点数量，并通过rejected_points累计所有帧中的无效点数量。
size_t writeBinaryPcd(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const fs::path & path,
  size_t & rejected_points)
{
  // std::array<float, 3>在内存中连续存放x/y/z，正好对应下方PCD头声明的3个float字段。
  std::vector<std::array<float, 3>> points;
  // width*height是有序或无序PointCloud2的理论点数，预分配可减少vector扩容和复制。
  points.reserve(static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height));

  // Iterator会根据PointCloud2字段布局和point_step自动定位每个点的x/y/z，
  // 因此不需要手工计算二进制偏移；如果字段不存在，Iterator构造阶段会报错。
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
  for (; x != x.end(); ++x, ++y, ++z) {
    // 非有限点不能参与后续占据栅格和射线检测，导出阶段直接丢弃并计数。
    if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
      ++rejected_points;
      continue;
    }
    points.push_back({*x, *y, *z});
  }

  // 以binary方式打开，避免Windows风格换行或文本模式影响后面的原始浮点字节。
  std::ofstream output(path, std::ios::binary);
  if (!output.good()) {
    throw std::runtime_error("Cannot create PCD file: " + path.string());
  }

  // 头部说明每点依次包含3个4字节float，点云按单行无序点云保存。
  // VIEWPOINT保持单位姿态，因为实际全局姿态单独存放在pose.json中。
  output <<
    "# .PCD v0.7 - Point Cloud Data file format\n"
    "VERSION 0.7\n"
    "FIELDS x y z\n"
    "SIZE 4 4 4\n"
    "TYPE F F F\n"
    "COUNT 1 1 1\n"
    "WIDTH " << points.size() << "\n"
    "HEIGHT 1\n"
    "VIEWPOINT 0 0 0 1 0 0 0\n"
    "POINTS " << points.size() << "\n"
    "DATA binary\n";

  if (!points.empty()) {
    // 头部结束后直接连续写入[x0,y0,z0,x1,y1,z1,...]的float字节。
    output.write(
      reinterpret_cast<const char *>(points.data()),
      static_cast<std::streamsize>(points.size() * sizeof(points.front())));
  }

  // 同时检查头部和二进制payload是否完整写入磁盘。
  if (!output.good()) {
    throw std::runtime_error("Failed while writing PCD file: " + path.string());
  }
  return points.size();
}

// 把内存PoseRow转换为pose.json的一行数组：
// [timestamp_s, tx, ty, tz, qx, qy, qz, qw]。
nlohmann::json poseJson(const PoseRow & pose)
{
  return nlohmann::json::array(
    {
      // JSON中使用浮点秒便于其他工具读取；精确匹配已经在整数纳秒阶段完成。
      static_cast<double>(pose.stamp_ns) / 1.0e9,
      pose.tx, pose.ty, pose.tz,
      pose.qx, pose.qy, pose.qz, pose.qw
    });
}

// 在真正读取消息前确认bag中存在所需话题且消息类型正确，
// 避免后续把错误类型的序列化数据强行反序列化。
void validateTopicTypes(
  const rosbag2_cpp::Reader & reader,
  const Options & options)
{
  std::unordered_map<std::string, std::string> types;
  // 建立“话题名 -> ROS接口类型”查找表。
  for (const auto & topic : reader.get_all_topics_and_types()) {
    types[topic.name] = topic.type;
  }
  if (types[options.odom_topic] != "nav_msgs/msg/Odometry") {
    throw std::runtime_error(options.odom_topic + " is missing or is not nav_msgs/msg/Odometry");
  }
  if (types[options.cloud_topic] != "sensor_msgs/msg/PointCloud2") {
    throw std::runtime_error(
            options.cloud_topic + " is missing or is not sensor_msgs/msg/PointCloud2");
  }
}

// 完整离线导出流程。该函数成功时创建正式输出目录，失败时不留下半成品。
void exportBag(const Options & options)
{
  // 输入必须存在；输出必须不存在，防止覆盖用户已经生成的PCD和图数据。
  if (!fs::exists(options.bag_path)) {
    throw std::runtime_error("Bag path does not exist: " + options.bag_path.string());
  }
  if (fs::exists(options.output_path)) {
    throw std::runtime_error("Output path already exists: " + options.output_path.string());
  }

  // 所有文件先写入同级临时目录，全部成功后再整体rename。
  // 使用同级目录可确保rename通常发生在同一文件系统内。
  const fs::path staging = options.output_path.string() + ".tmp";
  if (fs::exists(staging)) {
    throw std::runtime_error("Staging path already exists: " + staging.string());
  }
  fs::create_directories(staging / "key_frames");

  try {
    // 第一遍Reader只读取里程计，将所有姿态按header时间戳缓存。
    rosbag2_cpp::Reader reader;
    reader.open(options.bag_path.string());
    validateTopicTypes(reader, options);
    rosbag2_storage::StorageFilter filter;
    // 必须分两遍读取：MCAP/SQLite物理记录顺序不保证相同header时间戳的里程计
    // 一定排在点云前面。先收集全部里程计，第二遍再按点云出现顺序匹配，
    // 既消除了记录顺序影响，又保证导出的key_frames编号遵循点云顺序。
    filter.topics = {options.odom_topic};
    reader.set_filter(filter);

    // 尚未被点云匹配的里程计：key=整数纳秒时间戳，value=归一化姿态。
    std::unordered_map<int64_t, PoseRow> pending_odometry;
    // 已见过的点云时间戳，用于拒绝重复关键帧。
    std::unordered_set<int64_t> cloud_stamps;
    // 按点云顺序保存成功匹配的姿态，数组下标就是PCD文件编号。
    std::vector<PoseRow> matched_poses;
    matched_poses.reserve(4096U);
    // 以下计数和frame信息最终写入manifest.json，便于检查导出质量。
    size_t rejected_points = 0U;
    size_t written_points = 0U;
    std::string odom_frame;
    std::string odom_child_frame;
    std::string cloud_frame;

    while (reader.has_next()) {
      const auto bag_message = reader.read_next();
      const auto odom = deserialize<nav_msgs::msg::Odometry>(*bag_message);
      const auto pose = poseRow(odom);

      // 第一条消息确定里程计坐标系；后续消息不允许中途切换frame。
      if (odom_frame.empty()) {
        odom_frame = odom.header.frame_id;
        odom_child_frame = odom.child_frame_id;
      } else if (odom.header.frame_id != odom_frame || odom.child_frame_id != odom_child_frame) {
        throw std::runtime_error("Odometry frame IDs changed inside the bag");
      }

      // emplace返回false表示相同header时间戳已经出现，无法建立一对一对应关系。
      if (!pending_odometry.emplace(pose.stamp_ns, pose).second) {
        throw std::runtime_error("Duplicate odometry header timestamp");
      }
    }
    reader.close();

    // 第二遍Reader只读取关键帧点云，并根据header时间戳查找第一遍缓存的里程计。
    rosbag2_cpp::Reader cloud_reader;
    cloud_reader.open(options.bag_path.string());
    filter.topics = {options.cloud_topic};
    cloud_reader.set_filter(filter);
    while (cloud_reader.has_next()) {
      const auto bag_message = cloud_reader.read_next();
      if (bag_message->topic_name == options.cloud_topic) {
        const auto cloud = deserialize<sensor_msgs::msg::PointCloud2>(*bag_message);
        const int64_t stamp_ns = stampNanoseconds(cloud.header.stamp);

        // 与里程计相同，整个bag内点云frame必须保持一致。
        if (cloud_frame.empty()) {
          cloud_frame = cloud.header.frame_id;
        } else if (cloud.header.frame_id != cloud_frame) {
          throw std::runtime_error("Point cloud frame ID changed inside the bag");
        }

        // 一个时间戳只能对应一个关键帧，否则index与姿态的对应关系存在歧义。
        if (!cloud_stamps.insert(stamp_ns).second) {
          throw std::runtime_error("Duplicate point-cloud header timestamp");
        }

        // 这里执行严格相等匹配，不做最近邻时间同步或插值。
        const auto odom_it = pending_odometry.find(stamp_ns);
        if (odom_it == pending_odometry.end()) {
          throw std::runtime_error(
                  "Point cloud has no odometry with identical header timestamp");
        }

        // index同时决定key_frames/<index>.pcd文件名和pose.json中的行号。
        const size_t index = matched_poses.size();
        written_points += writeBinaryPcd(
          cloud, staging / "key_frames" / (std::to_string(index) + ".pcd"), rejected_points);
        matched_poses.push_back(odom_it->second);

        // 匹配成功后删除姿态，保证一条里程计最多只能被一个点云消费。
        pending_odometry.erase(odom_it);

        // 大bag导出耗时较长，每250帧打印进度，避免用户误认为程序卡死。
        if ((index + 1U) % 250U == 0U) {
          std::cout << "Exported " << (index + 1U) << " synchronized frames\n";
        }
      }
    }

    // bag可能在第一帧点云前或最后一帧点云后包含额外里程计。这些消息没有对应关键帧，
    // 不参与构图，但也不影响已导出的严格匹配帧，因此只统计并提示，不把它当成错误。
    const size_t unmatched_odometry = pending_odometry.size();
    if (unmatched_odometry > 0U) {
      std::cout << "Ignored " << unmatched_odometry <<
        " odometry messages without a matching point cloud\n";
    }
    if (matched_poses.empty()) {
      throw std::runtime_error("No synchronized odometry/point-cloud pairs were exported");
    }

    // 当前构图链假设：里程计给出camera_init下的base_link姿态，点云位于base_link。
    // 若传感器驱动或bag使用其他frame，必须显式增加TF变换，不能只跳过本检查。
    if (odom_frame != "camera_init" || odom_child_frame != "base_link" || cloud_frame != "base_link") {
      throw std::runtime_error(
              "Unexpected frames: odom=" + odom_frame + " child=" + odom_child_frame +
              " cloud=" + cloud_frame);
    }

    // pose.json中第i行与key_frames/i.pcd严格对应。
    nlohmann::json poses = nlohmann::json::array();
    for (const auto & pose : matched_poses) {
      poses.push_back(poseJson(pose));
    }
    // setw(2)让JSON使用2空格缩进，方便人工查看，不影响后续解析。
    std::ofstream(staging / "pose.json") << std::setw(2) << poses << '\n';

    // manifest记录数据来源、坐标系、同步数量和点云质量，
    // 后续出现节点数不一致时可先通过该文件定位导出阶段问题。
    const nlohmann::json manifest = {
      {"bag_path", fs::absolute(options.bag_path).string()},
      {"odom_topic", options.odom_topic},
      {"cloud_topic", options.cloud_topic},
      {"odom_frame", odom_frame},
      {"odom_child_frame", odom_child_frame},
      {"cloud_frame", cloud_frame},
      {"synchronized_frames", matched_poses.size()},
      {"unmatched_odometry_messages", unmatched_odometry},
      {"written_points", written_points},
      {"rejected_nonfinite_points", rejected_points},
      {"first_stamp_ns", matched_poses.front().stamp_ns},
      {"last_stamp_ns", matched_poses.back().stamp_ns}
    };
    std::ofstream(staging / "manifest.json") << std::setw(2) << manifest << '\n';

    // 所有PCD和JSON成功完成后才发布正式目录；此前外部程序只会看到.tmp目录。
    fs::rename(staging, options.output_path);
    std::cout << "Export complete: " << matched_poses.size() << " synchronized frames, " <<
      written_points << " finite points\nOutput: " << options.output_path << '\n';
  } catch (...) {
    // 任意异常都清理临时目录，再原样抛出，让main统一打印具体错误并返回非零状态。
    std::error_code error;
    fs::remove_all(staging, error);
    throw;
  }
}

}  // namespace

// 程序唯一公开入口：成功返回0，参数、bag、消息、坐标系或文件写入错误均返回1。
int main(int argc, char ** argv)
{
  try {
    exportBag(parseOptions(argc, argv));
    return 0;
  } catch (const std::exception & exception) {
    std::cerr << "bag_to_route3d_inputs: " << exception.what() << '\n';
    return 1;
  }
}

// Route3D离线构图命令行程序的入口文件。
//
// 本文件只负责：
// 1. 解析命令行参数并填充GraphBuilderConfig；
// 2. 调用buildGraphFromMap()执行真正的节点、顺序边和shortcut构建；
// 3. 把构建结果分别保存为JSON和GeoJSON。
//
// KD-Tree候选搜索、点云坐标变换、障碍栅格和射线检测等核心算法不在本文件中，
// 它们位于graph_builder_3d.cpp。该程序是普通离线CLI，因此不需要rclcpp::init()。

#include <cstddef>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>

// 同时引入GraphBuilderConfig、buildGraphFromMap()以及图文件保存函数声明。
#include "nav2_route3d/graph_builder_3d.hpp"

// 参数解析辅助函数仅供本编译单元使用，放在匿名命名空间避免导出全局符号。
namespace
{

// 查找形如“--参数名 参数值”的命令行参数。
// 找到后返回紧随其后的字符串；没有找到则返回fallback。
// 这里只做字符串查找，不检查重复参数，也不判断参数值是否合法。
std::string argValue(int argc, char ** argv, const std::string & name, const std::string & fallback)
{
  // 从1开始跳过argv[0]中的可执行文件名；i + 1 < argc保证读取参数值时不越界。
  for (int i = 1; i + 1 < argc; ++i) {
    if (argv[i] == name) {
      return argv[i + 1];
    }
  }
  return fallback;
}

// 读取double参数。参数缺失时先把fallback转成字符串，再统一交给stod解析。
// 非数字、溢出等情况会由std::stod抛出异常，本入口当前不捕获该异常。
double argDouble(int argc, char ** argv, const std::string & name, const double fallback)
{
  return std::stod(argValue(argc, argv, name, std::to_string(fallback)));
}

// 读取有符号整数参数，当前用于允许正负偏移语义的邻近PCD窗口大小。
int argInt(int argc, char ** argv, const std::string & name, const int fallback)
{
  return std::stoi(argValue(argc, argv, name, std::to_string(fallback)));
}

// 读取非负数量参数，例如候选数量、边数量和线程数量。
// stoull先转换成无符号长整数，再转换为当前平台的size_t。
size_t argSize(int argc, char ** argv, const std::string & name, const size_t fallback)
{
  return static_cast<size_t>(std::stoull(argValue(argc, argv, name, std::to_string(fallback))));
}

// 只判断命令行中是否出现某个参数，不要求其后一定存在参数值。
// 必填参数检查和--help分支使用该函数。
bool hasArg(int argc, char ** argv, const std::string & name)
{
  for (int i = 1; i < argc; ++i) {
    if (argv[i] == name) {
      return true;
    }
  }
  return false;
}

// Live recorder sessions store the odometry coordinate frame in manifest.json. Prefer that
// frame over the legacy "map" default so online markers, offline graphs and topoSingle files
// describe the same coordinates. A malformed or absent manifest keeps backward compatibility.
std::string frameIdFromManifest(const std::filesystem::path & map_path)
{
  const auto manifest_path = map_path / "manifest.json";
  if (!std::filesystem::exists(manifest_path)) {
    return "map";
  }
  try {
    std::ifstream input(manifest_path);
    const auto manifest = nlohmann::json::parse(input);
    const auto frame_id = manifest.value("odom_frame", std::string{});
    return frame_id.empty() ? "map" : frame_id;
  } catch (const std::exception & error) {
    std::cerr << "Warning: cannot read frame_id from " << manifest_path << ": "
              << error.what() << "; using map\n";
    return "map";
  }
}

}  // namespace

// 离线构图程序主入口。
int main(int argc, char ** argv)
{
  // --map-path是唯一必填参数。出现--help或缺少--map-path时打印完整使用方法。
  if (hasArg(argc, argv, "--help") || !hasArg(argc, argv, "--map-path")) {
    std::cout <<
      "Usage: route_graph_builder_3d --map-path PATH [--poses-file poses.json] [--pcd-dir pcd]\n"
      "       [--frame-id FRAME] (default: manifest.json odom_frame, then map)\n"
      "       [--output-json route3d_graph.json] [--output-geojson route3d_graph.geojson]\n"
      "       [--radius 5.0] [--time-threshold 10.0] [--z-threshold 0.2]\n"
      "       [--sensor-height 0.7] [--current-up-threshold 1.2]\n"
      "       [--neighbor-pcd-window 3] [--grid-resolution 0.2]\n"
      "       [--anchor-segment-length 5.0]\n"
      "       [--min-shortcut-distance 1.0] [--shortcut-angular-bin-deg 20.0]\n"
      "       [--max-candidates-per-angular-bin 2] [--max-raytrace-candidates 24]\n"
      "       [--max-shortcut-edges 8] [--workers N]\n";

    // 保留当前程序的返回值约定：命令中存在--map-path时返回0，否则返回2。
    // 因此仅执行“--help”但未同时给出--map-path时，当前实现仍返回2。
    return hasArg(argc, argv, "--map-path") ? 0 : 2;
  }

  // GraphBuilderConfig集中保存后续GraphBuilder3D需要的所有输入路径和算法参数。
  nav2_route3d::GraphBuilderConfig config;

  // 数据根目录，内部应包含姿态JSON和关键帧PCD目录。
  config.map_path = argValue(argc, argv, "--map-path", "");

  // An explicit command-line frame wins; otherwise inherit the recorder's odom_frame.
  config.frame_id = hasArg(argc, argv, "--frame-id") ?
    argValue(argc, argv, "--frame-id", "map") : frameIdFromManifest(config.map_path);

  // 姿态文件名相对于map_path解析。每行格式为：
  // [timestamp, x, y, z, qx, qy, qz, qw]。
  // 本工作空间bag导出器实际生成pose.json，所以运行时会显式传入--poses-file pose.json。
  config.poses_file = argValue(argc, argv, "--poses-file", "poses.json");

  // 关键帧PCD目录同样相对于map_path解析。第i个姿态对应目录中的i.pcd。
  // bag导出器实际生成key_frames目录，所以运行时会显式传入--pcd-dir key_frames。
  config.pcd_dir = argValue(argc, argv, "--pcd-dir", "pcd");

  // 只在三维欧氏距离radius_m以内搜索其他shortcut anchor。
  // 值越大，可能找到的回环越多，但候选过滤和点云检测开销也越高。
  config.radius_m = argDouble(argc, argv, "--radius", 5.0);

  // 时间差小于等于该阈值的anchor被视为同一段连续运动，不额外建立shortcut。
  // 这样可以避免给时间上本来就相邻的路线重复加边。
  config.time_threshold_s = argDouble(argc, argv, "--time-threshold", 10.0);

  // 两个anchor的机体Z差超过该阈值时拒绝候选，主要用于避免上下楼层XY重叠误连。
  config.z_threshold_m = argDouble(argc, argv, "--z-threshold", 0.2);

  // 构造障碍高度带的下界：anchor_z - sensor_height_m。
  // 低于该范围的点通常被视为地面以下或其他楼层结构，不参加XY射线遮挡判断。
  config.sensor_height_m = argDouble(argc, argv, "--sensor-height", 0.7);

  // 构造障碍高度带的上界：anchor_z + current_up_threshold_m。
  // 高于该范围的点不作为当前机体通行高度内的障碍。
  config.current_up_threshold_m = argDouble(argc, argv, "--current-up-threshold", 1.2);

  // 对每个候选关键帧，同时加载其前后多少帧PCD。
  // 例如窗口为1时，候选N会加载N-1、N、N+1三帧以补充环境覆盖。
  config.neighbor_pcd_window = argInt(argc, argv, "--neighbor-pcd-window", 3);

  // 将障碍点投影到二维占据栅格时的单元尺寸，单位m。
  // 后续使用Bresenham检查两个anchor的XY中心线是否经过占据单元。
  config.grid_resolution_m = argDouble(argc, argv, "--grid-resolution", 0.2);

  // 沿三维累计里程每隔多少米划分一段，并选择每段中点附近姿态作为shortcut anchor。
  // 该参数只控制哪些节点参与shortcut搜索，不会删除普通图节点。
  config.anchor_segment_length_m = argDouble(argc, argv, "--anchor-segment-length", 5.0);

  // 两个anchor的直线距离小于该值时，不建立收益很小的shortcut边。
  config.min_shortcut_distance_m = argDouble(argc, argv, "--min-shortcut-distance", 1.0);

  // 按当前anchor指向候选anchor的XY方向角分桶，单位为度。
  // 分桶可避免同一方向出现大量几乎重复的候选边。
  config.shortcut_angular_bin_deg = argDouble(argc, argv, "--shortcut-angular-bin-deg", 20.0);

  // 每个方向桶最多保留多少个距离最近的候选anchor。
  config.max_candidates_per_angular_bin = argSize(argc, argv, "--max-candidates-per-angular-bin", 2U);

  // 每个anchor最多对多少个候选执行PCD占据栅格和射线检测，用于限制离线计算量。
  config.max_raytrace_candidates_per_node = argSize(argc, argv, "--max-raytrace-candidates", 24U);

  // 每个图节点最终最多允许连接多少条由半径可见性检测生成的shortcut边。
  // 达到上限后的其他候选会记录为fake_neighbor，而不会写入edges。
  config.max_shortcut_edges_per_node = argSize(argc, argv, "--max-shortcut-edges", 8U);

  // 射线遮挡检测允许使用的最大并行任务数；默认使用系统报告的硬件线程数。
  // hardware_concurrency()允许返回0，核心构图代码内部仍会保证至少使用1个worker。
  config.max_workers = argSize(argc, argv, "--workers", std::thread::hardware_concurrency());

  // 输出文件名默认相对于map_path。std::filesystem的“/”运算符负责拼接路径；
  // 若用户传入绝对输出路径，则采用该绝对路径而不是写在map_path下。
  const auto output_json = config.map_path / argValue(argc, argv, "--output-json", "route3d_graph.json");
  const auto output_geojson =
    config.map_path / argValue(argc, argv, "--output-geojson", "route3d_graph.geojson");

  // 真正的构图入口：
  // 1. 读取全部姿态并创建节点；
  // 2. 创建相邻姿态的双向teach_repeat顺序边；
  // 3. 选择anchor并通过KD-Tree、时间/Z过滤、PCD射线检测增加shortcut边。
  auto graph = nav2_route3d::buildGraphFromMap(config);

  // JSON保留完整节点、边、邻居和metadata，供RouteServer运行时加载。
  nav2_route3d::saveGraphJson(graph, output_json);

  // GeoJSON保存相同图的几何Feature表达，主要用于地图工具查看和编辑。
  nav2_route3d::saveGraphGeoJson(graph, output_geojson);

  // 输出最终节点数、边数和两个文件路径，便于与输入关键帧数量及预期shortcut数量核对。
  std::cout << "Built 3D route graph: " << graph.nodes().size() << " nodes, "
            << graph.edges().size() << " edges\n"
            << "JSON: " << output_json << "\n"
            << "GeoJSON: " << output_geojson << "\n";

  // 到达这里说明构图和两个文件写出均未抛出异常。
  return 0;
}

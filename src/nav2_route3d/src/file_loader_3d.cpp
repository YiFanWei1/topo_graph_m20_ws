#include "nav2_route3d/file_loader_3d.hpp"

// 本文件负责 nav2_route3d 的磁盘数据交换：
// - 从纯文本或 JSON 读取 poses；
// - 从项目原生 JSON 或 GeoJSON FeatureCollection 恢复 Graph3D；
// - 把 Graph3D 保存为原生 JSON 或便于 GIS/RViz 外部处理的 GeoJSON；
// - 把人工修改的 GeoJSON metadata 合并回内存图。
//
// 它只做格式转换和基础字段检查，不重新计算 PCD、shortcut 或最优路线。

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <array>

#include <nlohmann/json.hpp>

namespace nav2_route3d
{
namespace
{

// 打开并解析完整 JSON 文件。文件打不开或 JSON 语法错误时让异常传给调用方。
nlohmann::json readJson(const std::filesystem::path & path)
{
  std::ifstream input(path);
  if (!input.good()) {
    throw std::runtime_error("Cannot open JSON file: " + path.string());
  }
  nlohmann::json payload;
  // nlohmann::json 的流运算符会读取并解析整个 JSON 文档。
  input >> payload;
  return payload;
}

// 跳过文件开头的空格、换行和 Tab，查看第一个有效字符，但不主动消费该字符。
// loadPoseRows() 用它区分 JSON（'['/'{'）与八列纯文本。
char firstNonWhitespace(std::istream & input)
{
  input >> std::ws;
  return static_cast<char>(input.peek());
}

// 把固定八列 JSON 数组转换为 Pose3D：
// [timestamp, tx, ty, tz, qx, qy, qz, qw]。
Pose3D poseFromArray(const nlohmann::json & row)
{
  // 这里要求恰好 8 项，避免列错位或悄悄忽略额外数据。
  if (!row.is_array() || row.size() != 8U) {
    throw std::runtime_error("Pose row must be array(timestamp tx ty tz qx qy qz qw)");
  }
  Pose3D pose;
  // get<double>() 会进行 JSON 数字类型转换；非数值字段会抛出异常。
  pose.timestamp = row[0].get<double>();
  pose.tx = row[1].get<double>();
  pose.ty = row[2].get<double>();
  pose.tz = row[3].get<double>();
  pose.qx = row[4].get<double>();
  pose.qy = row[5].get<double>();
  pose.qz = row[6].get<double>();
  pose.qw = row[7].get<double>();
  return pose;
}

// poseFromArray() 的逆操作，保持同样的八列顺序，供原生图 JSON 序列化使用。
nlohmann::json poseToArray(const Pose3D & pose)
{
  return nlohmann::json::array(
    {pose.timestamp, pose.tx, pose.ty, pose.tz, pose.qx, pose.qy, pose.qz, pose.qw});
}

// 把 unordered_set 节点编号转成 JSON 数组。
// unordered_set 不保证遍历顺序，因此保存后的邻居数组顺序不承诺稳定。
nlohmann::json setToJsonArray(const std::unordered_set<NodeId> & ids)
{
  nlohmann::json result = nlohmann::json::array();
  for (const auto id : ids) {
    result.push_back(id);
  }
  return result;
}

// 从 GeoJSON coordinates 中读取 XYZ；GeoJSON 至少要有 XY，缺少 Z 时补 0。
std::array<double, 3> coordinates3(const nlohmann::json & coordinates)
{
  if (!coordinates.is_array() || coordinates.size() < 2U) {
    throw std::runtime_error("GeoJSON coordinate must include x/y");
  }
  return {
    coordinates[0].get<double>(),
    coordinates[1].get<double>(),
    // 二维 GeoJSON 也可加载为三维图，此时节点位于 z=0 平面。
    coordinates.size() >= 3U ? coordinates[2].get<double>() : 0.0
  };
}

}  // namespace

// 加载离线构图输入位姿，自动识别 JSON 和空白分隔纯文本两种格式。
std::vector<Pose3D> loadPoseRows(const std::filesystem::path & path)
{
  std::vector<Pose3D> poses;
  // 位姿文件是文本内容，不需要 binary 打开方式。
  std::ifstream input(path);
  if (!input.good()) {
    throw std::runtime_error("Cannot open poses file: " + path.string());
  }

  // 探测首个非空白字符后，清除流状态并回到文件开头，供真正解析重新读取。
  const auto first = firstNonWhitespace(input);
  input.clear();
  input.seekg(0);

  if (first == '[' || first == '{') {
    // 支持顶层直接为 pose 数组，也支持 {"poses": [...]} 包装形式。
    nlohmann::json payload;
    input >> payload;
    const auto & rows = payload.is_object() && payload.contains("poses") ? payload.at("poses") : payload;
    if (!rows.is_array()) {
      throw std::runtime_error("Pose JSON must be an array or object with poses array");
    }

    poses.reserve(rows.size());
    for (const auto & row : rows) {
      // 每一行都必须满足严格八列格式，任一坏行都会终止加载。
      poses.push_back(poseFromArray(row));
    }
    return poses;
  }

  // 非 JSON 输入按“一行一个位姿、空白分隔八个数字”解析。
  std::string line;
  size_t line_number = 0U;
  while (std::getline(input, line)) {
    // 保存物理行号，报错时能直接定位原文件。
    ++line_number;
    std::istringstream row(line);
    row >> std::ws;
    // 允许完全空白行；当前格式没有专门处理 # 注释行。
    if (row.eof()) {
      continue;
    }

    Pose3D pose;
    // 字段顺序与 poseFromArray() 完全相同。少列、非数字都会进入错误分支；
    // 成功读完八列后不会检查额外尾列，因此额外数据会被忽略。
    if (!(row >> pose.timestamp >> pose.tx >> pose.ty >> pose.tz >>
      pose.qx >> pose.qy >> pose.qz >> pose.qw))
    {
      throw std::runtime_error(
              "Pose text row " + std::to_string(line_number) + " must contain 8 numeric columns");
    }
    poses.push_back(pose);
  }

  // 文本格式显式拒绝空文件；JSON 空数组会在前面的 JSON 分支直接返回空 vector。
  if (poses.empty()) {
    throw std::runtime_error("Pose file contains no poses: " + path.string());
  }
  return poses;
}

// 从项目原生 JSON 或 GeoJSON FeatureCollection 加载完整 Graph3D。
Graph3D loadGraph3D(const std::filesystem::path & path)
{
  const auto payload = readJson(path);
  // GeoJSON 通过顶层 type=FeatureCollection 识别。
  if (payload.value("type", "") == "FeatureCollection") {
    // frame_id 与图级 metadata 存放在 FeatureCollection.properties 中。
    const auto props = payload.value("properties", nlohmann::json::object());
    Graph3D graph(props.value("frame_id", "map"));
    graph.metadata() = props.value("metadata", nlohmann::json::object());

    // GeoJSON features 中节点和边可以任意交错，但 Graph3D::addEdge() 要求端点已存在，
    // 因此第一遍立即创建节点，只暂存边的 properties，第二遍再统一建边。
    std::vector<nlohmann::json> deferred_edges;
    for (const auto & feature : payload.value("features", nlohmann::json::array())) {
      const auto props2 = feature.value("properties", nlohmann::json::object());
      const auto kind = props2.value("kind", "");
      if (kind == "node") {
        // 节点位置来自 geometry.coordinates，接受 [x,y] 或 [x,y,z]。
        const auto coords = coordinates3(feature.at("geometry").at("coordinates"));
        Pose3D pose;
        // 时间戳和姿态不属于标准 GeoJSON 几何，放在自定义 properties 内。
        pose.timestamp = props2.value("timestamp", 0.0);
        pose.tx = coords[0];
        pose.ty = coords[1];
        pose.tz = coords[2];
        // 缺省姿态为单位四元数；存在 orientation 时预期顺序为 [qx,qy,qz,qw]。
        const auto q = props2.value("orientation", nlohmann::json::array({0, 0, 0, 1}));
        pose.qx = q[0].get<double>();
        pose.qy = q[1].get<double>();
        pose.qz = q[2].get<double>();
        pose.qw = q[3].get<double>();
        // id 是节点必需字段；metadata 和两个邻居数组允许缺省。
        Node3D node;
        node.node_id = props2.at("id").get<NodeId>();
        node.pose = pose;
        node.metadata = props2.value("metadata", nlohmann::json::object());
        // 加载文件中显式保存的邻居集合；稍后 addEdge() 还会补充真实邻居关系。
        for (const auto & id : props2.value("real_neighbors", nlohmann::json::array())) {
          node.real_neighbors.insert(id.get<NodeId>());
        }
        for (const auto & id : props2.value("fake_neighbors", nlohmann::json::array())) {
          node.fake_neighbors.insert(id.get<NodeId>());
        }
        graph.addNode(node);
      } else if (kind == "edge") {
        // 边的 LineString geometry 仅用于显示；恢复拓扑时以 properties 中的 start/end ID 为准。
        deferred_edges.push_back(props2);
      }
    }
    // 此时所有节点均已加入图，可以安全恢复边。
    for (const auto & edge : deferred_edges) {
      graph.addEdge(
        edge.at("start_id").get<NodeId>(),
        edge.at("end_id").get<NodeId>(),
        // 缺失 cost 时这里显式使用 0.0，并不会触发 Graph3D 的自动三维距离代价。
        edge.value("cost", 0.0),
        edge.value("metadata", nlohmann::json::object()),
        edge.value("operations", std::vector<Metadata>{}),
        edge.value("bidirectional", true),
        edge.at("id").get<EdgeId>());
    }
    return graph;
  }

  // 非 FeatureCollection 输入按 nav2_route3d 原生 JSON 格式解析。
  Graph3D graph(payload.value("frame_id", "map"));
  graph.metadata() = payload.value("metadata", nlohmann::json::object());
  // 原生格式节点 pose 使用固定八列数组，而不是 GeoJSON geometry。
  for (const auto & item : payload.value("nodes", nlohmann::json::array())) {
    Node3D node;
    node.node_id = item.at("id").get<NodeId>();
    node.pose = poseFromArray(item.at("pose"));
    node.metadata = item.value("metadata", nlohmann::json::object());
    for (const auto & id : item.value("real_neighbors", nlohmann::json::array())) {
      node.real_neighbors.insert(id.get<NodeId>());
    }
    for (const auto & id : item.value("fake_neighbors", nlohmann::json::array())) {
      node.fake_neighbors.insert(id.get<NodeId>());
    }
    graph.addNode(node);
  }
  // 节点全部创建完成后再恢复边，以满足端点存在约束。
  for (const auto & item : payload.value("edges", nlohmann::json::array())) {
    graph.addEdge(
      item.at("start_id").get<NodeId>(),
      item.at("end_id").get<NodeId>(),
      // 与 GeoJSON 分支一致，缺失 cost 会被加载为显式 0.0。
      item.value("cost", 0.0),
      item.value("metadata", nlohmann::json::object()),
      item.value("operations", std::vector<Metadata>{}),
      item.value("bidirectional", true),
      item.at("id").get<EdgeId>());
  }
  return graph;
}

// 将 Graph3D 保存为项目原生 JSON，能够完整保留 pose、边、邻居、operations 和 metadata。
void saveGraphJson(const Graph3D & graph, const std::filesystem::path & path)
{
  nlohmann::json payload;
  // format/version 用于标识自定义文件格式；当前版本固定为 1。
  payload["format"] = "nav2_route3d";
  payload["version"] = 1;
  payload["frame_id"] = graph.frameId();
  payload["metadata"] = graph.metadata();
  payload["nodes"] = nlohmann::json::array();
  // graph.nodes() 是 unordered_map，因此输出数组的节点顺序不保证按 ID 排列。
  for (const auto & [_, node] : graph.nodes()) {
    nlohmann::json item;
    item["id"] = node.node_id;
    // pose 始终写成 [timestamp, tx, ty, tz, qx, qy, qz, qw]。
    item["pose"] = poseToArray(node.pose);
    item["metadata"] = node.metadata;
    item["real_neighbors"] = setToJsonArray(node.real_neighbors);
    item["fake_neighbors"] = setToJsonArray(node.fake_neighbors);
    payload["nodes"].push_back(item);
  }
  payload["edges"] = nlohmann::json::array();
  // 边同样来自 unordered_map，数组顺序不保证稳定；edge_id 才是其身份标识。
  for (const auto & [_, edge] : graph.edges()) {
    payload["edges"].push_back(
      {
        {"id", edge.edge_id},
        {"start_id", edge.start_id},
        {"end_id", edge.end_id},
        {"cost", edge.cost},
        {"bidirectional", edge.bidirectional},
        {"metadata", edge.metadata},
        {"operations", edge.operations}
      });
  }

  // dump(2) 使用两空格缩进，方便人工检查和修改。
  // 当前实现依赖 ofstream 状态，没有额外检查目录不存在或写盘失败。
  std::ofstream output(path);
  output << payload.dump(2);
}

// 将 Graph3D 保存成 GeoJSON：节点为 Point Feature，边为 LineString Feature。
void saveGraphGeoJson(const Graph3D & graph, const std::filesystem::path & path)
{
  nlohmann::json payload;
  // 标准 GeoJSON 顶层类型；自定义 frame_id/metadata 放到 properties。
  payload["type"] = "FeatureCollection";
  payload["name"] = "nav2_route3d_graph";
  payload["properties"] = {{"frame_id", graph.frameId()}, {"metadata", graph.metadata()}};
  payload["features"] = nlohmann::json::array();
  for (const auto & [_, node] : graph.nodes()) {
    // 节点的 XYZ 进入标准 Point.coordinates，其余三维路线属性进入 properties。
    payload["features"].push_back(
      {
        {"type", "Feature"},
        {"geometry", {{"type", "Point"}, {"coordinates", {node.pose.tx, node.pose.ty, node.pose.tz}}}},
        {"properties",
          {
            {"kind", "node"},
            {"id", node.node_id},
            {"timestamp", node.pose.timestamp},
            {"orientation", {node.pose.qx, node.pose.qy, node.pose.qz, node.pose.qw}},
            {"metadata", node.metadata},
            {"real_neighbors", setToJsonArray(node.real_neighbors)},
            {"fake_neighbors", setToJsonArray(node.fake_neighbors)}
          }}
      });
  }
  for (const auto & [_, edge] : graph.edges()) {
    // 用端点 ID 取得实际节点坐标，生成只含首尾两点的 LineString。
    const auto & start = graph.node(edge.start_id);
    const auto & end = graph.node(edge.end_id);
    payload["features"].push_back(
      {
        {"type", "Feature"},
        {"geometry",
          {
            {"type", "LineString"},
            {"coordinates",
              {
                {start.pose.tx, start.pose.ty, start.pose.tz},
                {end.pose.tx, end.pose.ty, end.pose.tz}
              }}
          }},
        {"properties",
          {
            {"kind", "edge"},
            {"id", edge.edge_id},
            {"start_id", edge.start_id},
            {"end_id", edge.end_id},
            {"cost", edge.cost},
            {"bidirectional", edge.bidirectional},
            {"metadata", edge.metadata},
            {"operations", edge.operations}
          }}
      });
  }
  // GeoJSON 同样采用两空格缩进写出。
  std::ofstream output(path);
  output << payload.dump(2);
}

// 把外部 GeoJSON 中节点/边的 metadata 合并到已有 Graph3D。
// 典型用途是人工或 GIS 工具为点、边添加 terrain、任务、通行策略等属性。
void mergeGeoJsonMetadata(Graph3D & graph, const std::filesystem::path & path)
{
  const auto payload = readJson(path);
  for (const auto & feature : payload.value("features", nlohmann::json::array())) {
    const auto props = feature.value("properties", nlohmann::json::object());
    const auto metadata = props.value("metadata", nlohmann::json::object());
    const auto kind = props.value("kind", "");
    // 只合并图中确实存在的 ID；未知节点/边被忽略，不会新建拓扑元素。
    if (kind == "node" && graph.hasNode(props.at("id").get<NodeId>())) {
      // json::update() 更新 metadata 的顶层键：同名键使用外部值，其余原键保留。
      graph.node(props.at("id").get<NodeId>()).metadata.update(metadata);
    } else if (kind == "edge" && graph.hasEdge(props.at("id").get<EdgeId>())) {
      // 边只更新 metadata，不修改 cost、方向、operations 或端点。
      graph.edge(props.at("id").get<EdgeId>()).metadata.update(metadata);
    }
  }
}

}  // namespace nav2_route3d

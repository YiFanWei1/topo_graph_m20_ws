#include "route3d_dijkstra_planner/topology_graph.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include <malloc.h>
#include <nlohmann/json.hpp>
#include <unistd.h>

namespace route3d_dijkstra_planner
{
namespace
{

using Clock = std::chrono::steady_clock;
using json = nlohmann::json;

struct Arguments
{
  std::int32_t vertices{100000};
  std::int32_t shortcuts{50000};
  std::int32_t queries{20};
  double directed_fraction{0.15};
  std::uint32_t seed{20260903U};
};

struct Shortcut
{
  VertexId first{0};
  VertexId second{0};
  double weight{0.0};
  std::string travel_mode;
  std::string controller_mode;
};

struct TemporaryFile
{
  explicit TemporaryFile(std::filesystem::path file_path)
  : path(std::move(file_path)) {}
  ~TemporaryFile()
  {
    std::error_code error;
    std::filesystem::remove(path, error);
  }
  std::filesystem::path path;
};

double elapsedMs(const Clock::time_point begin)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
}

double memoryMb(const std::string & field)
{
  std::ifstream input("/proc/self/status");
  std::string line;
  const auto prefix = field + ":";
  while (std::getline(input, line)) {
    if (line.rfind(prefix, 0U) != 0U) {
      continue;
    }
    std::istringstream values(line.substr(prefix.size()));
    double value_kib = 0.0;
    values >> value_kib;
    return value_kib / 1024.0;
  }
  return 0.0;
}

std::uint64_t pairKey(const VertexId first, const VertexId second)
{
  const auto low = static_cast<std::uint32_t>(std::min(first, second));
  const auto high = static_cast<std::uint32_t>(std::max(first, second));
  return (static_cast<std::uint64_t>(low) << 32U) | high;
}

double vertexZ(const VertexId id, const std::int32_t width)
{
  const auto zero_based = id - 1;
  const auto row = zero_based / width;
  const auto column = zero_based % width;
  return 0.15 * std::sin(static_cast<double>(column) * 0.07) *
         std::sin(static_cast<double>(row) * 0.05);
}

json vertexJson(const VertexId id, const std::int32_t width)
{
  const auto zero_based = id - 1;
  const auto row = zero_based / width;
  const auto column = zero_based % width;
  const auto z = vertexZ(id, width);
  return {
    {"pos", {static_cast<double>(column), static_cast<double>(row), z}},
    {"rpy", {0.0, 0.0, 0.0}},
    {"meta", {
        {"type", 0}, {"typeId", 0}, {"isCorner", false},
        {"isSlope", std::abs(z) > 0.10}, {"isJunction", true},
        {"state", "confirmed"}, {"source", "dijkstra_stress_benchmark"},
        {"sourceStamp", static_cast<double>(id)}, {"component", 0},
        {"turnDeg", 0.0}, {"chargingMode", 0}}},
    {"pcd", ""}, {"acc", 0.5}, {"turnable", true}, {"alignFinalYaw", true}};
}

json edgeJson(
  const VertexId first, const VertexId second, const double weight,
  const std::string & travel_mode, const std::string & controller_mode)
{
  const int direction = travel_mode == "bidirectional" ? 0 :
    travel_mode == "first_to_second" ? 1 : 2;
  return {
    {"v", {first, second}}, {"weight", weight}, {"rotationAllowed", true},
    {"meta", {
        {"dir", direction}, {"source", "dijkstra_stress_benchmark"},
        {"locomotionMode", 0}, {"linearSpeedMps", 0.6},
        {"angularSpeedRadps", 0.0}, {"heightOffsetM", 0.0},
        {"obstacleMode", 0}, {"travelMode", travel_mode},
        {"headingAngleRad", 0.0}, {"obstacleBoxM", {0.0, 0.0, 0.0, 0.0}},
        {"gridMapName", ""}, {"controllerMode", controller_mode}}}};
}

std::vector<Shortcut> generateShortcuts(
  const Arguments & args, const std::int32_t width,
  std::unordered_set<std::uint64_t> & pairs)
{
  std::mt19937 generator(args.seed);
  std::uniform_int_distribution<VertexId> id_distribution(1, args.vertices);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::uniform_real_distribution<double> weight_scale(0.85, 1.25);
  const std::array<std::string, 3> controllers = {"auto", "pid", "local_planner"};
  std::uniform_int_distribution<std::size_t> controller_index(0U, controllers.size() - 1U);
  std::vector<Shortcut> result;
  result.reserve(static_cast<std::size_t>(args.shortcuts));
  const std::size_t maximum_attempts = std::max<std::size_t>(
    1000U, static_cast<std::size_t>(args.shortcuts) * 30U);
  for (std::size_t attempts = 0; result.size() < static_cast<std::size_t>(args.shortcuts) &&
    attempts < maximum_attempts; ++attempts)
  {
    const auto first = id_distribution(generator);
    const auto second = id_distribution(generator);
    const auto first_zero = first - 1;
    const auto second_zero = second - 1;
    const auto dx = static_cast<double>(first_zero % width - second_zero % width);
    const auto dy = static_cast<double>(first_zero / width - second_zero / width);
    const auto geometric = std::hypot(dx, dy);
    if (geometric < 8.0 || !pairs.insert(pairKey(first, second)).second) {
      continue;
    }
    std::string travel_mode = "bidirectional";
    if (unit(generator) < args.directed_fraction) {
      travel_mode = unit(generator) < 0.5 ? "first_to_second" : "second_to_first";
    }
    result.push_back({
      first, second, geometric * weight_scale(generator), travel_mode,
      controllers[controller_index(generator)]});
  }
  if (result.size() != static_cast<std::size_t>(args.shortcuts)) {
    throw std::runtime_error("could not generate the requested shortcut count");
  }
  return result;
}

std::size_t populateGridPairs(
  const Arguments & args, const std::int32_t width,
  std::unordered_set<std::uint64_t> & pairs)
{
  std::size_t count = 0U;
  for (VertexId id = 1; id <= args.vertices; ++id) {
    const auto column = (id - 1) % width;
    if (column + 1 < width && id + 1 <= args.vertices) {
      pairs.insert(pairKey(id, id + 1));
      ++count;
    }
    if (id + width <= args.vertices) {
      pairs.insert(pairKey(id, id + width));
      ++count;
    }
  }
  return count;
}

void writeSchemaV2Graph(
  const std::filesystem::path & path, const Arguments & args, const std::int32_t width,
  const std::vector<Shortcut> & shortcuts)
{
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("cannot create temporary benchmark graph");
  }
  output << "{\"version\":1,\"schema\":{\"name\":\"route3d_topology\",\"version\":2},"
         << "\"type\":0,\"frame_id\":\"map\",\"sceneMode\":\"normal\","
         << "\"status\":\"complete\",\"topology_mode\":\"stress_benchmark\","
         << "\"vertices\":{";
  for (VertexId id = 1; id <= args.vertices; ++id) {
    if (id != 1) {
      output << ',';
    }
    output << '"' << id << "\":" << vertexJson(id, width).dump();
  }
  output << "},\"edges\":{";
  std::size_t edge_id = 0U;
  auto write_edge = [&](const json & edge) {
      if (edge_id != 0U) {
        output << ',';
      }
      ++edge_id;
      output << '"' << edge_id << "\":" << edge.dump();
    };
  for (VertexId id = 1; id <= args.vertices; ++id) {
    const auto column = (id - 1) % width;
    if (column + 1 < width && id + 1 <= args.vertices) {
      write_edge(edgeJson(id, id + 1, 1.0, "bidirectional", "pid"));
    }
    if (id + width <= args.vertices) {
      write_edge(edgeJson(id, id + width, 1.0, "bidirectional", "local_planner"));
    }
  }
  for (const auto & shortcut : shortcuts) {
    write_edge(edgeJson(
        shortcut.first, shortcut.second, shortcut.weight, shortcut.travel_mode,
        shortcut.controller_mode));
  }
  output << "},\"generation\":{\"source\":\"dijkstra_stress_benchmark\","
         << "\"component_count\":1,\"shortcut_count\":" << shortcuts.size() << "}}";
}

std::vector<std::pair<VertexId, VertexId>> queryPairs(const Arguments & args)
{
  std::vector<std::pair<VertexId, VertexId>> result = {{1, args.vertices}, {args.vertices, 1}};
  std::mt19937 generator(args.seed + 1U);
  std::uniform_int_distribution<VertexId> distribution(1, args.vertices);
  while (result.size() < static_cast<std::size_t>(args.queries)) {
    const auto start = distribution(generator);
    const auto goal = distribution(generator);
    if (start != goal) {
      result.emplace_back(start, goal);
    }
  }
  result.resize(static_cast<std::size_t>(args.queries));
  return result;
}

double percentile(std::vector<double> values, const double fraction)
{
  std::sort(values.begin(), values.end());
  const double position = static_cast<double>(values.size() - 1U) * fraction;
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  const double ratio = position - static_cast<double>(lower);
  return values[lower] * (1.0 - ratio) + values[upper] * ratio;
}

Arguments parseArguments(const int argc, char ** argv)
{
  Arguments args;
  for (int index = 1; index < argc; ++index) {
    const std::string key = argv[index];
    if (index + 1 >= argc) {
      throw std::runtime_error("missing value after " + key);
    }
    const std::string value = argv[++index];
    if (key == "--vertices") {
      args.vertices = std::stoi(value);
    } else if (key == "--shortcuts") {
      args.shortcuts = std::stoi(value);
    } else if (key == "--queries") {
      args.queries = std::stoi(value);
    } else if (key == "--directed-fraction") {
      args.directed_fraction = std::stod(value);
    } else if (key == "--seed") {
      args.seed = static_cast<std::uint32_t>(std::stoul(value));
    } else {
      throw std::runtime_error("unknown argument " + key);
    }
  }
  if (args.vertices < 2 || args.shortcuts < 0 || args.queries < 1 ||
    args.directed_fraction < 0.0 || args.directed_fraction > 1.0)
  {
    throw std::runtime_error("invalid benchmark arguments");
  }
  return args;
}

}  // namespace

int runBenchmark(const Arguments & args)
{
  const auto width = static_cast<std::int32_t>(
    std::ceil(std::sqrt(static_cast<double>(args.vertices))));
  std::unordered_set<std::uint64_t> pairs;
  pairs.reserve(static_cast<std::size_t>(args.vertices) * 3U +
    static_cast<std::size_t>(args.shortcuts));
  const auto generation_begin = Clock::now();
  const auto grid_edges = populateGridPairs(args, width, pairs);
  const auto shortcuts = generateShortcuts(args, width, pairs);
  const auto generation_ms = elapsedMs(generation_begin);
  const auto edge_count = grid_edges + shortcuts.size();

  TemporaryFile temporary(
    std::filesystem::temp_directory_path() /
    ("route3d_dijkstra_stress_" + std::to_string(::getpid()) + ".json"));
  const auto write_begin = Clock::now();
  writeSchemaV2Graph(temporary.path, args, width, shortcuts);
  const auto write_ms = elapsedMs(write_begin);
  const auto file_mb = static_cast<double>(std::filesystem::file_size(temporary.path)) /
    (1024.0 * 1024.0);
  std::cout << std::fixed << std::setprecision(3)
            << "generated Schema V2 C++ graph: vertices=" << args.vertices
            << " edges=" << edge_count << " shortcuts=" << shortcuts.size()
            << " generation_ms=" << generation_ms << " write_ms=" << write_ms
            << " file_mb=" << file_mb << std::endl;

  const auto memory_before_load = memoryMb("VmRSS");
  const auto load_begin = Clock::now();
  const auto graph = TopologyGraph::load(temporary.path, true);
  const auto load_ms = elapsedMs(load_begin);
  ::malloc_trim(0);
  const auto memory_after_load = memoryMb("VmRSS");
  const auto peak_memory = memoryMb("VmHWM");
  std::cout << "loaded C++ production graph: load_ms=" << load_ms
            << " rss_mb=" << memory_after_load << " peak_rss_mb=" << peak_memory
            << std::endl;

  std::vector<double> timings;
  std::vector<std::size_t> expanded;
  timings.reserve(static_cast<std::size_t>(args.queries));
  expanded.reserve(static_cast<std::size_t>(args.queries));
  std::size_t maximum_path_vertices = 0U;
  std::size_t query_index = 0U;
  for (const auto & pair : queryPairs(args)) {
    const auto begin = Clock::now();
    const auto result = dijkstraShortestPath(graph, pair.first, pair.second);
    const auto milliseconds = elapsedMs(begin);
    timings.push_back(milliseconds);
    expanded.push_back(result.expanded_vertices);
    maximum_path_vertices = std::max(maximum_path_vertices, result.vertex_ids.size());
    std::cout << "query=" << ++query_index << '/' << args.queries
              << " start=" << pair.first << " goal=" << pair.second
              << " time_ms=" << milliseconds << " expanded=" << result.expanded_vertices
              << " path_vertices=" << result.vertex_ids.size()
              << " cost=" << result.total_cost << std::endl;
  }

  const auto mean = std::accumulate(timings.begin(), timings.end(), 0.0) /
    static_cast<double>(timings.size());
  const auto expanded_mean = static_cast<double>(
    std::accumulate(expanded.begin(), expanded.end(), std::size_t{0})) /
    static_cast<double>(expanded.size());
  const auto maximum_time = *std::max_element(timings.begin(), timings.end());
  const auto minimum_time = *std::min_element(timings.begin(), timings.end());
  const auto maximum_expanded = *std::max_element(expanded.begin(), expanded.end());
  json result = {
    {"implementation", "cpp"}, {"vertices", args.vertices}, {"edges", edge_count},
    {"shortcuts", args.shortcuts}, {"directed_fraction", args.directed_fraction},
    {"queries", args.queries}, {"seed", args.seed}, {"schema_v2_json_mb", file_mb},
    {"generation_ms", generation_ms}, {"json_write_ms", write_ms},
    {"graph_load_ms", load_ms}, {"memory_before_load_mb", memory_before_load},
    {"memory_after_load_mb", memory_after_load}, {"peak_memory_mb", peak_memory},
    {"search_ms", {
        {"minimum", minimum_time}, {"mean", mean}, {"p50", percentile(timings, 0.50)},
        {"p95", percentile(timings, 0.95)}, {"p99", percentile(timings, 0.99)},
        {"maximum", maximum_time}}},
    {"expanded_vertices", {{"mean", expanded_mean}, {"maximum", maximum_expanded}}},
    {"maximum_path_vertices", maximum_path_vertices}};
  std::cout << "BENCHMARK_RESULT=" << result.dump() << std::endl;
  return 0;
}

}  // namespace route3d_dijkstra_planner

int main(int argc, char ** argv)
{
  try {
    return route3d_dijkstra_planner::runBenchmark(
      route3d_dijkstra_planner::parseArguments(argc, argv));
  } catch (const std::exception & error) {
    std::cerr << "dijkstra_stress_benchmark: " << error.what() << std::endl;
    return 1;
  }
}

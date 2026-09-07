#include <filesystem>
#include <fstream>
#include <unistd.h>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "route3d_web_console/topology_store.hpp"

using route3d_web_console::TopologyStore;
using nlohmann::json;

namespace
{
json graph()
{
  return {{"version", 1}, {"schema", {{"name", "route3d_topology"}, {"version", 2}}},
    {"frame_id", "camera_init"}, {"sceneMode", "normal"},
    {"vertices", {{"1", {{"pos", {0, 0, 0}}, {"rpy", {0, 0, 0}}, {"acc", 0.5},
      {"passRadiusM", 0.45}, {"meta", json::object()}}},
      {"2", {{"pos", {1, 0, 0}}, {"rpy", {0, 0, 0}}, {"acc", 0.5},
      {"passRadiusM", 0.45}, {"meta", json::object()}}}}},
    {"edges", {{"1", {{"v", {1, 2}}, {"weight", 1.0}, {"meta",
      {{"dir", 0}, {"travelMode", "bidirectional"}, {"controllerMode", "auto"}}}}}}}};
}
}

TEST(TopologyStore, ValidatesAndRejectsDirectionMismatch)
{
  auto value = graph();
  EXPECT_TRUE(TopologyStore::validate(value).empty());
  value["edges"]["1"]["meta"]["travelMode"] = "first_to_second";
  EXPECT_FALSE(TopologyStore::validate(value).empty());
}

TEST(TopologyStore, SavesAtomicallyAndDetectsConflict)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("route3d_web_test_" + std::to_string(::getpid()));
  std::filesystem::create_directories(directory);
  const auto path = directory / "graph.json";
  auto first = TopologyStore::save(path, graph(), "");
  EXPECT_EQ(first.data["version"], 2);
  auto second = first.data;
  second["sceneMode"] = "tunnel";
  EXPECT_NO_THROW(TopologyStore::save(path, second, first.revision));
  EXPECT_THROW(TopologyStore::save(path, first.data, first.revision), std::runtime_error);
  std::filesystem::remove_all(directory);
}

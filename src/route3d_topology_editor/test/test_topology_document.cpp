#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <unistd.h>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "route3d_topology_editor/topology_document.hpp"

namespace
{

using route3d_topology_editor::TopologyDocument;

std::filesystem::path temporaryPath()
{
  return std::filesystem::temp_directory_path() /
         ("route3d_topology_editor_" + std::to_string(::getpid()) + ".json");
}

TEST(TopologyDocument, LoadsEditsBacksUpAndPreservesUnknownFields)
{
  const auto path = temporaryPath();
  const nlohmann::json input = {
    {"schema", {{"name", "route3d_topology"}, {"version", 2}}},
    {"frame_id", "camera_init"},
    {"unknownRoot", "keep"},
    {"vertices", {
      {"1", {{"pos", {0.0, 0.0, 0.0}}, {"rpy", {0.0, 0.0, 0.0}},
        {"meta", nlohmann::json::object()}, {"unknownVertex", 7}}},
      {"2", {{"pos", {3.0, 4.0, 0.0}}, {"rpy", {0.0, 0.0, 0.0}},
        {"meta", nlohmann::json::object()}}},
    }},
    {"edges", {
      {"1", {{"v", {1, 2}}, {"weight", 1.0},
        {"meta", {{"obstacleMode", 0}}}}},
    }},
  };
  {
    std::ofstream output(path);
    output << input.dump(2);
  }

  TopologyDocument document;
  document.load(path);
  document.data()["vertices"]["1"]["pos"][0] = 1.0;
  TopologyDocument::recomputeIncidentWeights(document.data(), 1);
  document.save(path);

  TopologyDocument reloaded;
  reloaded.load(path);
  EXPECT_EQ(reloaded.data().at("unknownRoot"), "keep");
  EXPECT_EQ(reloaded.data().at("vertices").at("1").at("unknownVertex"), 7);
  EXPECT_DOUBLE_EQ(reloaded.data().at("edges").at("1").at("weight"),
    std::sqrt(20.0));
  EXPECT_TRUE(std::filesystem::exists(path.string() + ".bak"));

  std::filesystem::remove(path);
  std::filesystem::remove(path.string() + ".bak");
}

TEST(TopologyDocument, RejectsMissingEdgeEndpoint)
{
  const auto path = temporaryPath();
  const nlohmann::json input = {
    {"schema", {{"name", "route3d_topology"}, {"version", 2}}},
    {"vertices", {
      {"1", {{"pos", {0.0, 0.0, 0.0}}, {"rpy", {0.0, 0.0, 0.0}},
        {"meta", nlohmann::json::object()}}},
    }},
    {"edges", {
      {"1", {{"v", {1, 9}}, {"weight", 1.0}, {"meta", nlohmann::json::object()}}},
    }},
  };
  {
    std::ofstream output(path);
    output << input.dump();
  }
  TopologyDocument document;
  EXPECT_THROW(document.load(path), std::runtime_error);
  std::filesystem::remove(path);
}

TEST(TopologyDocument, FindsShortestBatchPath)
{
  const nlohmann::json document = {
    {"vertices", {
      {"1", {{"pos", {0.0, 0.0, 0.0}}}},
      {"2", {{"pos", {1.0, 0.0, 0.0}}}},
      {"3", {{"pos", {2.0, 0.0, 0.0}}}},
      {"4", {{"pos", {9.0, 0.0, 0.0}}}},
    }},
    {"edges", {
      {"10", {{"v", {1, 2}}, {"weight", 1.0}}},
      {"11", {{"v", {2, 3}}, {"weight", 1.0}}},
      {"12", {{"v", {1, 3}}, {"weight", 8.0}}},
    }},
  };

  const auto path = TopologyDocument::shortestPath(document, 1, 3);
  EXPECT_EQ(path.vertex_ids, (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(path.edge_ids, (std::vector<int>{10, 11}));
  EXPECT_THROW(TopologyDocument::shortestPath(document, 1, 4), std::runtime_error);
}

TEST(TopologyDocument, AddsAndRemovesVerticesAndEdgesWithRecorderDefaults)
{
  nlohmann::json document = {
    {"schema", {{"name", "route3d_topology"}, {"version", 2}}},
    {"vertices", {
      {"1", {{"pos", {0.0, 0.0, 0.0}}, {"rpy", {0.0, 0.0, 0.0}},
        {"meta", nlohmann::json::object()}}},
      {"2", {{"pos", {1.0, 0.0, 0.0}}, {"rpy", {0.0, 0.0, 0.0}},
        {"meta", nlohmann::json::object()}}},
    }},
    {"edges", {
      {"1", {{"v", {1, 2}}, {"weight", 1.0}, {"meta", nlohmann::json::object()}}},
    }},
  };

  const int vertex_id = TopologyDocument::addVertex(document, 2.0, 3.0, 0.4, 123.0);
  ASSERT_EQ(vertex_id, 3);
  const auto & vertex = document.at("vertices").at("3");
  EXPECT_EQ(vertex.at("meta").at("source"), "incremental_topology");
  EXPECT_DOUBLE_EQ(vertex.at("acc").get<double>(), 0.5);
  EXPECT_TRUE(vertex.at("turnable").get<bool>());
  EXPECT_FALSE(vertex.at("alignFinalYaw").get<bool>());

  const int edge_id = TopologyDocument::addEdge(document, 1, 3);
  ASSERT_EQ(edge_id, 2);
  const auto & edge = document.at("edges").at("2");
  EXPECT_EQ(edge.at("meta").at("obstacleMode"), 0);
  EXPECT_EQ(edge.at("meta").at("controllerMode"), "auto");
  EXPECT_DOUBLE_EQ(edge.at("weight").get<double>(), std::sqrt(13.16));
  EXPECT_THROW(TopologyDocument::addEdge(document, 3, 1), std::runtime_error);

  TopologyDocument::removeVertex(document, 1);
  EXPECT_FALSE(document.at("vertices").contains("1"));
  EXPECT_TRUE(document.at("edges").empty());
  const int replacement_edge_id = TopologyDocument::addEdge(document, 2, 3);
  TopologyDocument::removeEdge(document, replacement_edge_id);
  EXPECT_TRUE(document.at("edges").empty());
  EXPECT_THROW(TopologyDocument::removeEdge(document, 99), std::runtime_error);

  TopologyDocument checker;
  checker.data() = document;
  EXPECT_NO_THROW(checker.validate());
}

}  // namespace

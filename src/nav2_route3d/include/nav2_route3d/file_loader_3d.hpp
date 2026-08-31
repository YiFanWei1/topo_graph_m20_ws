#pragma once

#include <filesystem>
#include <vector>

#include "nav2_route3d/graph_3d.hpp"

namespace nav2_route3d
{

std::vector<Pose3D> loadPoseRows(const std::filesystem::path & path);
Graph3D loadGraph3D(const std::filesystem::path & path);
void saveGraphJson(const Graph3D & graph, const std::filesystem::path & path);
void saveGraphGeoJson(const Graph3D & graph, const std::filesystem::path & path);
void mergeGeoJsonMetadata(Graph3D & graph, const std::filesystem::path & path);

}  // namespace nav2_route3d

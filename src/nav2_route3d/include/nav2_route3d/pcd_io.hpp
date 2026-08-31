#pragma once

#include <filesystem>
#include <optional>
#include <vector>

#include "nav2_route3d/kdtree_3d.hpp"

namespace nav2_route3d
{

std::vector<Point3D> readPcdXYZ(const std::filesystem::path & path);
std::optional<std::filesystem::path> findPcdForIndex(
  const std::filesystem::path & pcd_dir,
  size_t index);

}  // namespace nav2_route3d

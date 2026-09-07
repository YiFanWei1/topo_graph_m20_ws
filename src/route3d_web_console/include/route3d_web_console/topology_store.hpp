#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace route3d_web_console
{

struct TopologyDocument
{
  nlohmann::json data;
  std::string revision;
};

class TopologyStore
{
public:
  static TopologyDocument load(const std::filesystem::path & path);
  static std::vector<std::string> validate(const nlohmann::json & document);
  static TopologyDocument save(
    const std::filesystem::path & path, const nlohmann::json & document,
    const std::string & expected_revision);
  static std::string revision(const nlohmann::json & document);
};

}  // namespace route3d_web_console

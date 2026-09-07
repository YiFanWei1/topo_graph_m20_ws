#include "route3d_web_console/topology_store.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>

namespace fs = std::filesystem;
using nlohmann::json;

namespace route3d_web_console
{
namespace
{

bool finiteNumber(const json & value)
{
  return value.is_number() && std::isfinite(value.get<double>());
}

bool finiteArray(const json & value, const std::size_t size)
{
  if (!value.is_array() || value.size() != size) {
    return false;
  }
  for (const auto & item : value) {
    if (!finiteNumber(item)) {
      return false;
    }
  }
  return true;
}

bool positiveId(const std::string & value, int & parsed)
{
  try {
    std::size_t end = 0;
    parsed = std::stoi(value, &end);
    return end == value.size() && parsed > 0;
  } catch (...) {
    return false;
  }
}

std::string timestamp()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t value = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
  localtime_r(&value, &local);
  std::ostringstream stream;
  stream << std::put_time(&local, "%Y%m%d_%H%M%S");
  return stream.str();
}

void writeDurably(const fs::path & path, const std::string & text)
{
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output) {
    throw std::runtime_error("cannot open temporary topology file: " + path.string());
  }
  output.write(text.data(), static_cast<std::streamsize>(text.size()));
  output.flush();
  if (!output) {
    throw std::runtime_error("cannot write temporary topology file: " + path.string());
  }
  output.close();
  const int fd = ::open(path.c_str(), O_RDONLY);
  if (fd >= 0) {
    ::fsync(fd);
    ::close(fd);
  }
}

}  // namespace

TopologyDocument TopologyStore::load(const fs::path & path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("topology file does not exist: " + path.string());
  }
  json document;
  input >> document;
  const auto errors = validate(document);
  if (!errors.empty()) {
    throw std::runtime_error("invalid topology: " + errors.front());
  }
  return {document, revision(document)};
}

std::vector<std::string> TopologyStore::validate(const json & document)
{
  std::vector<std::string> errors;
  if (!document.is_object()) {
    return {"document must be an object"};
  }
  if (!document.contains("schema") || !document["schema"].is_object() ||
    document["schema"].value("name", "") != "route3d_topology" ||
    document["schema"].value("version", 0) != 2)
  {
    errors.emplace_back("schema must be route3d_topology version 2");
  }
  if (!document.contains("frame_id") || !document["frame_id"].is_string() ||
    document["frame_id"].get<std::string>().empty())
  {
    errors.emplace_back("frame_id must be a non-empty string");
  }
  if (!document.contains("vertices") || !document["vertices"].is_object()) {
    errors.emplace_back("vertices must be an object");
    return errors;
  }
  if (!document.contains("edges") || !document["edges"].is_object()) {
    errors.emplace_back("edges must be an object");
    return errors;
  }

  std::set<int> vertex_ids;
  for (const auto & entry : document["vertices"].items()) {
    int id = 0;
    if (!positiveId(entry.key(), id)) {
      errors.push_back("vertex id must be a positive integer: " + entry.key());
      continue;
    }
    vertex_ids.insert(id);
    const auto & vertex = entry.value();
    if (!vertex.is_object() || !finiteArray(vertex.value("pos", json{}), 3) ||
      !finiteArray(vertex.value("rpy", json{}), 3))
    {
      errors.push_back("vertex " + entry.key() + " requires finite pos[3] and rpy[3]");
      continue;
    }
    const auto acc = vertex.value("acc", 0.5);
    const auto radius = vertex.value("passRadiusM", 0.45);
    if (!std::isfinite(acc) || acc <= 0.0 || !std::isfinite(radius) || radius <= 0.0) {
      errors.push_back("vertex " + entry.key() + " tolerances must be positive");
    }
  }

  std::set<std::pair<int, int>> endpoint_pairs;
  for (const auto & entry : document["edges"].items()) {
    int id = 0;
    if (!positiveId(entry.key(), id)) {
      errors.push_back("edge id must be a positive integer: " + entry.key());
      continue;
    }
    const auto & edge = entry.value();
    if (!edge.is_object() || !edge.contains("v") || !edge["v"].is_array() ||
      edge["v"].size() != 2 || !edge["v"][0].is_number_integer() ||
      !edge["v"][1].is_number_integer())
    {
      errors.push_back("edge " + entry.key() + " requires v[2]");
      continue;
    }
    const int first = edge["v"][0].get<int>();
    const int second = edge["v"][1].get<int>();
    if (first == second || !vertex_ids.count(first) || !vertex_ids.count(second)) {
      errors.push_back("edge " + entry.key() + " references invalid endpoints");
    }
    const auto pair = std::minmax(first, second);
    if (!endpoint_pairs.emplace(pair.first, pair.second).second) {
      errors.push_back("duplicate edge endpoint pair at edge " + entry.key());
    }
    if (!finiteNumber(edge.value("weight", json{})) || edge["weight"].get<double>() < 0.0) {
      errors.push_back("edge " + entry.key() + " weight must be finite and non-negative");
    }
    const auto meta = edge.value("meta", json::object());
    const int direction = meta.value("dir", -1);
    const std::string travel = meta.value("travelMode", "");
    const std::map<int, std::string> expected{{0, "bidirectional"}, {1, "first_to_second"},
      {2, "second_to_first"}};
    if (!expected.count(direction) || expected.at(direction) != travel) {
      errors.push_back("edge " + entry.key() + " dir and travelMode disagree");
    }
    const std::string controller = meta.value("controllerMode", "auto");
    if (controller != "auto" && controller != "pid" && controller != "local_planner" &&
      controller != "efficient_3d_local_planner")
    {
      errors.push_back("edge " + entry.key() + " has unsupported controllerMode");
    }
  }
  return errors;
}

TopologyDocument TopologyStore::save(
  const fs::path & path, const json & document, const std::string & expected_revision)
{
  const auto errors = validate(document);
  if (!errors.empty()) {
    throw std::runtime_error("invalid topology: " + errors.front());
  }
  if (fs::exists(path)) {
    const auto current = load(path);
    if (!expected_revision.empty() && current.revision != expected_revision) {
      throw std::runtime_error("topology revision conflict; reload before saving");
    }
  }
  fs::create_directories(path.parent_path());
  if (fs::exists(path)) {
    const fs::path backup_dir = path.parent_path() / ".route3d_web_backups";
    fs::create_directories(backup_dir);
    fs::copy_file(path, backup_dir / (path.filename().string() + "." + timestamp() + ".bak"),
      fs::copy_options::overwrite_existing);
  }
  json updated = document;
  updated["version"] = updated.value("version", 0) + 1;
  const std::string text = updated.dump(2) + "\n";
  const fs::path temporary = path.string() + ".tmp." + std::to_string(::getpid());
  writeDurably(temporary, text);
  fs::rename(temporary, path);
  const int directory_fd = ::open(path.parent_path().c_str(), O_RDONLY | O_DIRECTORY);
  if (directory_fd >= 0) {
    ::fsync(directory_fd);
    ::close(directory_fd);
  }
  return {updated, revision(updated)};
}

std::string TopologyStore::revision(const json & document)
{
  const auto hash = std::hash<std::string>{}(document.dump());
  std::ostringstream stream;
  stream << std::hex << std::setw(16) << std::setfill('0') << hash;
  return stream.str();
}

}  // namespace route3d_web_console

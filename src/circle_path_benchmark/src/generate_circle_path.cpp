#include "circle_path_benchmark/circle_path.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>

namespace
{

struct Options
{
  std::string pcd_file;
  std::string output_file;
  std::string frame_id{"camera_init"};
  circle_path_benchmark::CircleSpec circle;
  double robot_radius{0.30};
  double body_half_height{0.35};
  bool allow_unsafe{false};
};

std::string requireValue(int argc, char ** argv, int & index)
{
  if (index + 1 >= argc) {throw std::invalid_argument("missing value for " + std::string(argv[index]));}
  return argv[++index];
}

Options parseOptions(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--pcd") {options.pcd_file = requireValue(argc, argv, index);}
    else if (argument == "--output") {options.output_file = requireValue(argc, argv, index);}
    else if (argument == "--frame-id") {options.frame_id = requireValue(argc, argv, index);}
    else if (argument == "--center-x") {options.circle.center_x = std::stod(requireValue(argc, argv, index));}
    else if (argument == "--center-y") {options.circle.center_y = std::stod(requireValue(argc, argv, index));}
    else if (argument == "--path-z") {options.circle.path_z = std::stod(requireValue(argc, argv, index));}
    else if (argument == "--radius") {options.circle.radius = std::stod(requireValue(argc, argv, index));}
    else if (argument == "--spacing") {options.circle.sample_spacing = std::stod(requireValue(argc, argv, index));}
    else if (argument == "--start-angle-deg") {
      options.circle.start_angle = std::stod(requireValue(argc, argv, index)) *
        circle_path_benchmark::kPi / 180.0;
    } else if (argument == "--direction") {
      const std::string direction = requireValue(argc, argv, index);
      if (direction != "ccw" && direction != "cw") {
        throw std::invalid_argument("--direction must be ccw or cw");
      }
      options.circle.clockwise = direction == "cw";
    } else if (argument == "--robot-radius") {
      options.robot_radius = std::stod(requireValue(argc, argv, index));
    } else if (argument == "--body-half-height") {
      options.body_half_height = std::stod(requireValue(argc, argv, index));
    } else if (argument == "--allow-unsafe") {options.allow_unsafe = true;}
    else if (argument == "--help" || argument == "-h") {
      std::cout <<
        "Usage: generate_circle_path --pcd MAP.pcd --output circle.yaml "
        "--center-x X --center-y Y --path-z Z [--radius 0.8] [--spacing 0.05] "
        "[--start-angle-deg 0] [--direction ccw] [--robot-radius 0.30] "
        "[--body-half-height 0.35] [--frame-id camera_init] [--allow-unsafe]\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown argument: " + argument);
    }
  }
  if (options.pcd_file.empty() || options.output_file.empty()) {
    throw std::invalid_argument("--pcd and --output are required");
  }
  if (options.frame_id.empty() || options.robot_radius <= 0.0 ||
    options.body_half_height <= 0.0)
  {
    throw std::invalid_argument("frame_id, robot radius and body half height must be valid");
  }
  circle_path_benchmark::validateCircleSpec(options.circle);
  return options;
}

std::string yamlQuote(const std::string & value)
{
  std::string escaped;
  escaped.reserve(value.size() + 2U);
  escaped.push_back('"');
  for (const char character : value) {
    if (character == '\\' || character == '"') {escaped.push_back('\\');}
    escaped.push_back(character);
  }
  escaped.push_back('"');
  return escaped;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options options = parseOptions(argc, argv);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile(options.pcd_file, cloud) != 0 || cloud.empty()) {
      throw std::runtime_error("failed to load non-empty PCD: " + options.pcd_file);
    }

    double minimum_x = std::numeric_limits<double>::infinity();
    double maximum_x = -minimum_x;
    double minimum_y = minimum_x;
    double maximum_y = -minimum_x;
    double minimum_clearance = std::numeric_limits<double>::infinity();
    std::size_t slab_points = 0U;
    for (const auto & point : cloud) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z))
      {
        continue;
      }
      minimum_x = std::min(minimum_x, static_cast<double>(point.x));
      maximum_x = std::max(maximum_x, static_cast<double>(point.x));
      minimum_y = std::min(minimum_y, static_cast<double>(point.y));
      maximum_y = std::max(maximum_y, static_cast<double>(point.y));
      if (point.z < options.circle.path_z - options.body_half_height ||
        point.z > options.circle.path_z + options.body_half_height)
      {
        continue;
      }
      ++slab_points;
      const double radial_distance = std::hypot(
        point.x - options.circle.center_x, point.y - options.circle.center_y);
      minimum_clearance = std::min(
        minimum_clearance, std::abs(radial_distance - options.circle.radius));
    }
    if (slab_points == 0U) {
      throw std::runtime_error("PCD has no points in the configured body-height slab");
    }
    const double required_extent = options.circle.radius + options.robot_radius;
    const bool inside_bounds =
      options.circle.center_x - required_extent >= minimum_x &&
      options.circle.center_x + required_extent <= maximum_x &&
      options.circle.center_y - required_extent >= minimum_y &&
      options.circle.center_y + required_extent <= maximum_y;
    if (!inside_bounds && !options.allow_unsafe) {
      throw std::runtime_error("circle plus robot radius extends outside PCD XY bounds");
    }
    if (minimum_clearance < options.robot_radius && !options.allow_unsafe) {
      throw std::runtime_error(
              "PCD clearance " + std::to_string(minimum_clearance) +
              " m is below robot radius " + std::to_string(options.robot_radius) + " m");
    }

    const auto points = circle_path_benchmark::generateCircle(options.circle);
    const std::filesystem::path output(options.output_file);
    if (!output.parent_path().empty()) {
      std::filesystem::create_directories(output.parent_path());
    }
    const std::filesystem::path temporary = output.string() + ".tmp";
    {
      std::ofstream stream(temporary);
      if (!stream) {throw std::runtime_error("failed to open output: " + temporary.string());}
      stream << std::setprecision(10);
      stream << "version: 1\n";
      stream << "frame_id: " << yamlQuote(options.frame_id) << "\n";
      stream << "source_pcd: " << yamlQuote(std::filesystem::absolute(options.pcd_file).string()) << "\n";
      stream << "circle:\n";
      stream << "  center: [" << options.circle.center_x << ", " << options.circle.center_y << ", " << options.circle.path_z << "]\n";
      stream << "  radius: " << options.circle.radius << "\n";
      stream << "  sample_spacing: " << options.circle.sample_spacing << "\n";
      stream << "  start_angle: " << options.circle.start_angle << "\n";
      stream << "  direction: " << (options.circle.clockwise ? "cw" : "ccw") << "\n";
      stream << "validation:\n";
      stream << "  robot_radius: " << options.robot_radius << "\n";
      stream << "  body_half_height: " << options.body_half_height << "\n";
      stream << "  pcd_slab_points: " << slab_points << "\n";
      stream << "  minimum_clearance: " << minimum_clearance << "\n";
      stream << "  inside_pcd_bounds: " << (inside_bounds ? "true" : "false") << "\n";
      stream << "points:\n";
      for (const auto & point : points) {
        stream << "  - [" << point.x << ", " << point.y << ", " << point.z << ", " << point.yaw << "]\n";
      }
      stream.flush();
      if (!stream) {throw std::runtime_error("failed while writing output");}
    }
    std::filesystem::rename(temporary, output);
    std::cout << std::fixed << std::setprecision(3)
              << "Generated " << output << " points=" << points.size()
              << " radius=" << options.circle.radius
              << " clearance=" << minimum_clearance
              << " start=[" << points.front().x << ", " << points.front().y
              << ", " << points.front().z << "] yaw=" << points.front().yaw << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "generate_circle_path: " << error.what() << '\n';
    return 2;
  }
}

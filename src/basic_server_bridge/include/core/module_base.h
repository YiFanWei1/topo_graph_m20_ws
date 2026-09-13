#ifndef BASIC_SERVER_BRIDGE__CORE__MODULE_BASE_H_
#define BASIC_SERVER_BRIDGE__CORE__MODULE_BASE_H_

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include "colorful_terminal/colorful_terminal.hpp"

#include <memory>
#include <sstream>
#include <string>
#include <vector>

class ModuleBase
{
public:
  using Ptr = std::shared_ptr<ModuleBase>;

protected:
  static std::vector<std::string> split(const std::string &value, char delimiter)
  {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream stream(value);
    while (std::getline(stream, token, delimiter)) {
      tokens.push_back(token);
    }
    return tokens;
  }

private:
  YAML::Node config_node_;
  std::string name_;
  std::shared_ptr<ctl::table_out> table_out_ptr_;
  bool print_table_en_{false};

public:
  ModuleBase(
    const std::string &config_path, const std::string &prefix,
    const std::string &module_name = "default")
  : name_(module_name), table_out_ptr_(std::make_shared<ctl::table_out>(module_name))
  {
    if (config_path.empty()) {
      return;
    }

    try {
      const YAML::Node root_node = YAML::LoadFile(config_path);
      config_node_ = root_node;
      if (root_node["print_table_en"]) {
        print_table_en_ = root_node["print_table_en"].as<bool>();
      }
    } catch (const YAML::Exception &exception) {
      LOG(ERROR) << "failed to load config file '" << config_path << "': " << exception.what();
      config_node_ = YAML::Node();
      return;
    }

    if (!prefix.empty()) {
      for (const auto &prefix_key : split(prefix, '/')) {
        if (!config_node_ || !config_node_[prefix_key]) {
          VLOG(1) << name_ << ": config prefix not found: " << prefix;
          config_node_ = YAML::Node();
          break;
        }
        config_node_ = config_node_[prefix_key];
      }
    }
  }

  template<typename T>
  void readParam(const std::string &key, T &value, const T &default_value)
  {
    try {
      if (key.find('/') != std::string::npos) {
        YAML::Node current_node = YAML::Clone(config_node_);
        for (const auto &subkey : split(key, '/')) {
          if (!current_node || !current_node[subkey]) {
            value = default_value;
            VLOG(1) << name_ << ": missing parameter '" << key << "', using default";
            table_out_ptr_->add_item(key, VAR_NAME(value), value);
            return;
          }
          current_node = current_node[subkey];
        }
        value = current_node.as<T>();
      } else if (config_node_[key]) {
        value = config_node_[key].as<T>();
      } else {
        value = default_value;
        VLOG(1) << name_ << ": missing parameter '" << key << "', using default";
      }

      table_out_ptr_->add_item(key, VAR_NAME(value), value);
    } catch (const YAML::Exception &exception) {
      value = default_value;
      LOG(WARNING) << name_ << ": invalid parameter '" << key << "': " << exception.what()
                   << "; using default";
      table_out_ptr_->add_item(key, VAR_NAME(value), value);
    }
  }

  void print_table()
  {
    if (print_table_en_) {
      table_out_ptr_->make_table_and_out();
    }
  }
};

#endif  // BASIC_SERVER_BRIDGE__CORE__MODULE_BASE_H_

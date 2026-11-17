#include "joy_speed_limits.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/parameter_map.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>
#include <stdexcept>
#include <vector>

namespace ct_joy
{
  namespace
  {

// Direct YAML reader for config/params.yaml. Deliberately independent of
// rclcpp::Node::declare_parameter() so these speed limits can only be
// changed by editing the file, never via `--ros-args -p` or a launch file.
    class ParameterFile
    {
public:
      explicit ParameterFile(const std::filesystem::path & path)
      : path_(path.string())
      {
        rclcpp::ParameterMap nodes;
        try {
          nodes = rclcpp::parameter_map_from_yaml_file(path_);
        } catch(const rclcpp::exceptions::RCLError & error) {
          fail(error.what());
        }
        const auto node = nodes.find("/ct_joy");
        if (nodes.size() != 1 || node == nodes.end()) {
          fail("expected only ct_joy.ros__parameters");
        }
        for (const auto & parameter : node->second) {
          if (!parameters_.emplace(parameter.get_name(), parameter).second) {
            fail("duplicate parameter: " + parameter.get_name());
          }
        }
      }

      double positive_real(const std::string & name)
      {
        const auto found = parameters_.find(name);
        if (found == parameters_.end()) {
          fail("missing required parameter: " + name);
        }
        if (found->second.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
          fail(name + ": expected a double, got " + found->second.get_type_name());
        }
        const double value = found->second.as_double();
        parameters_.erase(found);
        if (!std::isfinite(value) || value <= 0.0) {
          fail(name + ": expected a finite positive double");
        }
        return value;
      }

      double nonnegative_real(const std::string & name)
      {
        const auto found = parameters_.find(name);
        if (found == parameters_.end()) {
          fail("missing required parameter: " + name);
        }
        if (found->second.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
          fail(name + ": expected a double, got " + found->second.get_type_name());
        }
        const double value = found->second.as_double();
        parameters_.erase(found);
        if (!std::isfinite(value) || value < 0.0) {
          fail(name + ": expected a finite nonnegative double");
        }
        return value;
      }

      std::vector < std::string > parameter_names() const
      {
        std::vector < std::string > names;
        names.reserve(parameters_.size());
        for (const auto & parameter : parameters_) {
          names.push_back(parameter.first);
        }
        return names;
      }

      void finish() const
      {
        if (!parameters_.empty()) {
          fail("unknown parameter: " + parameters_.begin()->first);
        }
      }

      [[noreturn]] void fail(const std::string & message) const
      {
        throw std::invalid_argument(path_ + ": " + message);
      }

private:
      std::string path_;
      std::map < std::string, rclcpp::Parameter > parameters_;
    };

  } // namespace

  JoySpeedLimits load_joy_speed_limits()
  {
    const std::filesystem::path path(
      ament_index_cpp::get_package_share_directory("ct_joy") + "/config/params.yaml");
    ParameterFile file(path);
    JoySpeedLimits limits;
    limits.horizontal_speed = file.positive_real("horizontal_speed");
    limits.d_gain = file.nonnegative_real("d_gain");
    limits.position_mode_horizontal_speed = file.positive_real("position_mode_horizontal_speed");
    limits.z_vel_max_speed_up = file.positive_real("z_vel_max_speed_up");
    limits.z_vel_max_speed_down = file.positive_real("z_vel_max_speed_down");
    file.positive_real("mpc_yawrate_auto_max");      // validated here; used by ct_searching_algorithms, not this package
    constexpr double kDegToRad = M_PI / 180.0;
    limits.max_yaw_rate_rad_s = file.positive_real("mpc_man_y_max") * kDegToRad;
    file.finish();
    return limits;
  }

  void reject_speed_limit_overrides(rclcpp::Node & node)
  {
    const std::filesystem::path path(
      ament_index_cpp::get_package_share_directory("ct_joy") + "/config/params.yaml");
    ParameterFile file(path);
    const auto & overrides = node.get_node_parameters_interface()->get_parameter_overrides();
    for (const auto & name : file.parameter_names()) {
      if (overrides.count(name) != 0) {
        throw std::invalid_argument(
          std::string("Parameter overrides are not allowed: ") + name +
          "; edit ct_joy/config/params.yaml instead");
      }
    }
  }

  JoyCommandLimits declare_command_limits(rclcpp::Node & node, const JoySpeedLimits & limits)
  {
    const auto & overrides = node.get_node_parameters_interface()->get_parameter_overrides();
    if (overrides.count("max_vertical_speed_mps") != 0) {
      throw std::invalid_argument(
        "max_vertical_speed_mps is no longer supported; use "
        "max_vertical_speed_up_mps and max_vertical_speed_down_mps");
    }

    const auto declare_limit = [&node](const char * name, double default_value, double yaml_limit) {
      const double requested = node.declare_parameter < double > (name, default_value);
      if (!std::isfinite(requested) || requested < 0.0) {
        throw std::invalid_argument(std::string(name) + " must be finite and nonnegative");
      }
      const double value = std::min(requested, yaml_limit);
      const auto result = node.set_parameter(rclcpp::Parameter(name, value));
      if (!result.successful) {
        throw std::runtime_error(std::string("Failed to set ") + name + ": " + result.reason);
      }
      return value;
    };

    JoyCommandLimits result;
    result.horizontal_speed =
      declare_limit("max_horizontal_speed_mps", 3.0, limits.horizontal_speed);
    result.position_mode_horizontal_speed = declare_limit(
      "max_position_mode_horizontal_speed_mps", 3.0, limits.position_mode_horizontal_speed);
    result.vertical_speed_up = declare_limit(
      "max_vertical_speed_up_mps", 1.0,
      limits.z_vel_max_speed_up);
    result.vertical_speed_down =
      declare_limit("max_vertical_speed_down_mps", 1.0, limits.z_vel_max_speed_down);
    result.yaw_rate_rad_s = declare_limit("max_yaw_rate_rps", 1.5, limits.max_yaw_rate_rad_s);
    return result;
  }

  double clamp_vertical_rate(double rate_mps, const JoyCommandLimits & limits)
  {
    return std::clamp(rate_mps, -limits.vertical_speed_down, limits.vertical_speed_up);
  }

  void clamp_horizontal_velocity(double & vx_mps, double & vy_mps, double max_speed_mps)
  {
    const double planar_speed = std::hypot(vx_mps, vy_mps);
    if (planar_speed > max_speed_mps) {
      const double scale = max_speed_mps / planar_speed;
      vx_mps *= scale;
      vy_mps *= scale;
    }
  }

}  // namespace ct_joy

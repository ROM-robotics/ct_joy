#include "joy_speed_limits.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/parameter_map.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <map>
#include <stdexcept>

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
		} catch (const rclcpp::exceptions::RCLError & error) {
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
	std::map<std::string, rclcpp::Parameter> parameters_;
};

}  // namespace

JoySpeedLimits load_joy_speed_limits()
{
	const std::filesystem::path path(
		ament_index_cpp::get_package_share_directory("ct_joy") + "/config/params.yaml");
	ParameterFile file(path);
	JoySpeedLimits limits;
	limits.horizontal_speed = file.positive_real("horizontal_speed");
	limits.z_vel_max_speed_up = file.positive_real("z_vel_max_speed_up");
	limits.z_vel_max_speed_down = file.positive_real("z_vel_max_speed_down");
	file.positive_real("mpc_yawrate_auto_max");  // validated here; used by ct_searching_algorithms, not this package
	constexpr double kDegToRad = M_PI / 180.0;
	limits.max_yaw_rate_rad_s = file.positive_real("mpc_man_y_max") * kDegToRad;
	file.finish();
	return limits;
}

void reject_speed_limit_overrides(rclcpp::Node & node)
{
	static constexpr std::array<const char *, 5> reserved = {
		"horizontal_speed", "z_vel_max_speed_up", "z_vel_max_speed_down",
		"mpc_yawrate_auto_max", "mpc_man_y_max"};
	const auto & overrides = node.get_node_parameters_interface()->get_parameter_overrides();
	for (const char * name : reserved) {
		if (overrides.count(name) != 0) {
			throw std::invalid_argument(
				std::string("Parameter overrides are not allowed: ") + name +
				"; edit ct_joy/config/params.yaml instead");
		}
	}
}

double clamp_vertical_rate(double rate_mps, const JoySpeedLimits & limits)
{
	if (rate_mps > 0.0) {
		return std::min(rate_mps, limits.z_vel_max_speed_up);
	}
	return std::max(rate_mps, -limits.z_vel_max_speed_down);
}

}  // namespace ct_joy

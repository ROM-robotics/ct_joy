#include "ct_follower_png/vision_png_runtime.hpp"
#include "ct_follower_png/px4_multirotor_follower.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/parameter_map.hpp>

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace ct_follower_png
{
namespace
{
std::filesystem::path timestamped_csv_path(
    const std::filesystem::path &directory, const std::string &csv_file_name)
{
    std::string base_name = "output";
    if (!csv_file_name.empty()) {
        std::filesystem::path name_path(csv_file_name);
        if (name_path.has_parent_path() || csv_file_name == "." || csv_file_name == "..") {
            throw std::invalid_argument("csv_file_name: path separators are not allowed");
        }
        if (name_path.extension() == ".csv") {
            name_path = name_path.stem();
        }
        if (name_path.empty()) {
            throw std::invalid_argument("csv_file_name: empty file name is not allowed");
        }
        base_name = name_path.string() + "_output";
    }
    const auto now = std::chrono::system_clock::now();
    const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
    const auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
        now - seconds).count();
    const auto time = std::chrono::system_clock::to_time_t(seconds);
    std::tm local_time{};
    if (localtime_r(&time, &local_time) == nullptr) {
        throw std::runtime_error("csv_path: cannot convert timestamp to local time");
    }
    std::ostringstream suffix;
    suffix << std::put_time(&local_time, "_%Y%m%d_%H%M%S_")
           << std::setfill('0') << std::setw(6) << microseconds;
    if (!suffix) {
        throw std::runtime_error("csv_path: cannot format timestamp");
    }
    return (directory / (base_name + suffix.str() + ".csv")).lexically_normal();
}

class ParameterFile
{
public:
    explicit ParameterFile(const std::filesystem::path &path) : path_(path.string())
    {
        rclcpp::ParameterMap nodes;
        try {
            nodes = rclcpp::parameter_map_from_yaml_file(path_);
        } catch (const rclcpp::exceptions::RCLError &error) {
            fail(error.what());
        }
        const auto node = nodes.find("/ct_follower_png");
        if (nodes.size() != 1 || node == nodes.end()) {
            fail("expected only ct_follower_png.ros__parameters");
        }
        for (const auto &parameter : node->second) {
            if (!parameters_.emplace(parameter.get_name(), parameter).second) {
                fail("duplicate parameter: " + parameter.get_name());
            }
        }
    }

    double real(const std::string &name, double minimum, double maximum)
    {
        const double value = take(name, rclcpp::ParameterType::PARAMETER_DOUBLE).as_double();
        if (!std::isfinite(value) || value < minimum || value > maximum) {
            fail(name + ": expected a finite double in [" +
                std::to_string(minimum) + ", " + std::to_string(maximum) + "]");
        }
        return value;
    }

    int integer(const std::string &name, int minimum)
    {
        const auto value = take(name, rclcpp::ParameterType::PARAMETER_INTEGER).as_int();
        if (value < minimum || value > std::numeric_limits<int>::max()) {
            fail(name + ": integer out of range");
        }
        return static_cast<int>(value);
    }

    bool boolean(const std::string &name)
    {
        return take(name, rclcpp::ParameterType::PARAMETER_BOOL).as_bool();
    }

    std::string text(const std::string &name)
    {
        auto value = take(name, rclcpp::ParameterType::PARAMETER_STRING).as_string();
        if (value.empty()) fail(name + ": empty string is not allowed");
        return value;
    }

    void finish() const
    {
        if (!parameters_.empty()) fail("unknown parameter: " + parameters_.begin()->first);
    }

    [[noreturn]] void fail(const std::string &message) const
    {
        throw std::invalid_argument(path_ + ": " + message);
    }

private:
    rclcpp::Parameter take(const std::string &name, rclcpp::ParameterType type)
    {
        const auto found = parameters_.find(name);
        if (found == parameters_.end()) fail("missing required parameter: " + name);
        if (found->second.get_type() != type) {
            fail(name + ": expected " + rclcpp::to_string(type) +
                ", got " + found->second.get_type_name());
        }
        auto parameter = found->second;
        parameters_.erase(found);
        return parameter;
    }

    std::string path_;
    std::map<std::string, rclcpp::Parameter> parameters_;
};
}  // namespace

void load_parameter_files(
    const std::string &config_directory, VisionPNGConfig &guidance, Px4FollowerConfig &aircraft,
    const std::string &csv_file_name)
{
    const std::filesystem::path directory(config_directory);
    ParameterFile vision(directory / "vision_png_params.yaml");
    ParameterFile follower(directory / "px4_x500_follower_params.yaml");
    VisionPNGConfig config;
    Px4FollowerConfig vehicle;
    constexpr double positive = std::numeric_limits<double>::min();
    constexpr double maximum = std::numeric_limits<double>::max();

    config.focal_length = vision.real("focal_length", positive, maximum);
    config.image_width = vision.integer("image_width", 1);
    config.image_height = vision.integer("image_height", 1);
    config.N_png = vision.real("N_png", 0.0, maximum);
    config.speed_cmd = vision.real("speed_cmd", 1.5, maximum);
    config.d_gain = vision.real("d_gain", 0.0, maximum);
    config.kv = vision.real("kv", 0.0, maximum);
    config.kz = vision.real("kz", 0.0, maximum);
    config.control_wz_gain_1_p = vision.real("control_wz_gain_1_p", 0.0, 0.02);
    config.control_wz_gain_2_i = vision.real("control_wz_gain_2_i", 0.0, 0.01);
    config.control_wz_gain_3_d = vision.real("control_wz_gain_3_d", 0.0, 0.01);
    config.control_vz_gain_1_p = vision.real("control_vz_gain_1_p", 0.0, 0.1);
    config.control_vz_gain_2_i = vision.real("control_vz_gain_2_i", 0.0, 0.1);
    config.control_vz_gain_3_d = vision.real("control_vz_gain_3_d", 0.0, 0.1);
    config.p_gain_vx = vision.real("p_gain_vx", 0.0, maximum);
    config.i_gain_vx = vision.real("i_gain_vx", 0.0, maximum);
    config.d_gain_vx = vision.real("d_gain_vx", 0.0, maximum);
    config.k_ey = vision.real("k_ey", 0.0, maximum);
    config.rom_vz_complementary_pid_ratio = vision.real("rom_vz_complementary_pid_ratio", 0.0, 1.0);
    config.coast_thresh = vision.integer("coast_thresh", 0);
    config.lost_thresh = vision.integer("lost_thresh", 1);
    if (config.lost_thresh <= config.coast_thresh) {
        vision.fail("lost_thresh must be greater than coast_thresh");
    }
    config.coast_thresh_timeout_seconds = vision.real("coast_thresh_timeout_seconds", 0.0, maximum);
    config.lost_thresh_timeout_seconds = vision.real("lost_thresh_timeout_seconds", positive, maximum);
    config.detection_timeout_seconds = vision.real("detection_timeout_seconds", positive, maximum);
    if (config.coast_thresh_timeout_seconds > config.detection_timeout_seconds) {
        vision.fail("coast_thresh_timeout_seconds must not exceed detection_timeout_seconds");
    }
    if (config.lost_thresh_timeout_seconds <= config.detection_timeout_seconds) {
        vision.fail("lost_thresh_timeout_seconds must exceed detection_timeout_seconds");
    }
    config.re_search_yaw_rate_after_track_lost_rd_per_sec =
        vision.real("re_search_yaw_rate_after_track_lost_rd_per_sec", positive, 3.0);
    config.return_home_timeout_seconds =
        vision.real("return_home_timeout_seconds", positive, maximum);
    config.approach_enabled = vision.boolean("approach_enabled");
    config.approach_bbox_width = vision.real("approach_bbox_width", positive, maximum);
    config.approach_resume_bbox_width = vision.real("approach_resume_bbox_width", positive, maximum);
    config.approach_resume_confirm_seconds = vision.real("approach_resume_confirm_seconds", positive, maximum);
    config.approach_kp = vision.real("approach_kp", 0.0, maximum);
    config.approach_max_speed = vision.real("approach_max_speed", 0.0, maximum);
    config.approach_accel_limit = vision.real("approach_accel_limit", positive, maximum);
    config.hybrid_blend_start_width = vision.real("hybrid_blend_start_width", 0.0, maximum);
    if (config.approach_bbox_width >= config.image_width ||
        config.approach_resume_bbox_width >= config.approach_bbox_width ||
        config.hybrid_blend_start_width >= config.approach_bbox_width) {
        vision.fail("approach_resume_bbox_width and hybrid_blend_start_width must be below "
            "approach_bbox_width, which must be below image_width");
    }
    config.nn_residual_gain = vision.real("nn_residual_gain", 0.0, maximum);
    config.yaw_rate_limit = vision.real("yaw_rate_limit", 0.5, 3.0);
    vision.finish();

    vehicle.standby_altitude = follower.real("standby_altitude", -maximum, -positive);
    vehicle.hit_radius = static_cast<float>(follower.real(
        "hit_radius", std::numeric_limits<float>::min(), std::numeric_limits<float>::max()));
    const std::filesystem::path csv_path(follower.text("csv_path"));
    const auto csv_directory =
        (csv_path.is_absolute() ? csv_path : std::filesystem::current_path() / csv_path)
            .lexically_normal();
    vehicle.csv_path = timestamped_csv_path(csv_directory, csv_file_name).string();
    follower.finish();

    guidance = config;
    aircraft = std::move(vehicle);
}

void load_parameters(rclcpp::Node &node, VisionPNGConfig &guidance, Px4FollowerConfig &aircraft)
{
    std::string csv_file_name;
    const auto &overrides = node.get_node_parameters_interface()->get_parameter_overrides();
    for (const auto &[name, value] : overrides) {
        if (name == "csv_file_name") {
            if (value.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
                throw std::invalid_argument(
                    "csv_file_name: expected string, got " + rclcpp::to_string(value.get_type()));
            }
            csv_file_name = value.get<std::string>();
            continue;
        }
        throw std::invalid_argument("Parameter overrides are not allowed: " + name +
            "; edit vision_png_params.yaml or px4_x500_follower_params.yaml instead");
    }
    load_parameter_files(
        ament_index_cpp::get_package_share_directory("ct_follower_png") + "/config",
        guidance, aircraft, csv_file_name);
}
}  // namespace ct_follower_png

#include "ct_runner_sim.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include "px4_msgs/msg/vehicle_rates_setpoint.hpp"

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <random>
#include <stdexcept>
#include <px4_msgs/msg/sensor_gps.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

UavTargetControl::UavTargetControl() : Node("ct_runner")
{
    // --- Parameter declaration and reading (read once from config/params.yaml) ---
    // 0. Common parameters
    const std::string mode_str = this->declare_parameter<std::string>("motion_mode", "circle");
    std::string ns = this->declare_parameter<std::string>("px4_namespace", "/px4_1");
    int target_system_param = static_cast<int>(this->declare_parameter<int64_t>("target_system", 0));
    max_range_ = static_cast<float>(this->declare_parameter<double>("max_range", static_cast<double>(max_range_)));

    // 1. Mode: circle
    circle_radius_ = static_cast<float>(
        this->declare_parameter<double>("ROM_circle_radius", static_cast<double>(circle_radius_)));
    circle_altitude_ = static_cast<float>(
        this->declare_parameter<double>("ROM_circle_altitude", static_cast<double>(circle_altitude_)));
    angular_speed_ = static_cast<float>(
        this->declare_parameter<double>("ROM_angular_speed", static_cast<double>(angular_speed_)));

    // 2. Mode: circle_sine
    circle_altitude_amplitude_ = static_cast<float>(
        this->declare_parameter<double>("ROM_circle_altitude_amplitude", static_cast<double>(circle_altitude_amplitude_)));
    circle_altitude_cycles_ = static_cast<int>(
        this->declare_parameter<int64_t>("ROM_circle_altitude_cycles", circle_altitude_cycles_));

    // 3. Mode: circle_sine_non_linear_accel
    circle_sine_nl_min_cycles_ = std::max(
        4, static_cast<int>(this->declare_parameter<int64_t>("circle_sine_nl_min_cycles", circle_sine_nl_min_cycles_)));
    circle_sine_nl_freq_ = static_cast<float>(
        this->declare_parameter<double>("circle_sine_nl_freq", static_cast<double>(circle_sine_nl_freq_)));
    circle_sine_nl_quad_gain_ = static_cast<float>(
        this->declare_parameter<double>("circle_sine_nl_quad_gain", static_cast<double>(circle_sine_nl_quad_gain_)));
    circle_sine_nl_pos_gain_ = static_cast<float>(
        this->declare_parameter<double>("circle_sine_nl_pos_gain", static_cast<double>(circle_sine_nl_pos_gain_)));
    circle_sine_nl_cubic_damping_ = static_cast<float>(
        this->declare_parameter<double>("circle_sine_nl_cubic_damping", static_cast<double>(circle_sine_nl_cubic_damping_)));
    circle_sine_nl_omega_scale_min_ = static_cast<float>(
        this->declare_parameter<double>("circle_sine_nl_omega_scale_min", static_cast<double>(circle_sine_nl_omega_scale_min_)));
    circle_sine_nl_omega_scale_max_ = static_cast<float>(
        this->declare_parameter<double>("circle_sine_nl_omega_scale_max", static_cast<double>(circle_sine_nl_omega_scale_max_)));
    circle_sine_nl_phase_mod_ = static_cast<float>(
        this->declare_parameter<double>("circle_sine_nl_phase_mod", static_cast<double>(circle_sine_nl_phase_mod_)));

    // 3b. Mode: circle_sine_non_linear_accel_with_different_diameters
    circle_diff_diam_min_m_ = std::max(
        1.0, this->declare_parameter<double>("circle_diff_diam_min_m", circle_diff_diam_min_m_));
    circle_diff_diam_max_m_ = std::max(
        circle_diff_diam_min_m_, this->declare_parameter<double>("circle_diff_diam_max_m", circle_diff_diam_max_m_));
    circle_diff_diam_count_ = std::max(
        2, static_cast<int>(this->declare_parameter<int64_t>("circle_diff_diam_count", circle_diff_diam_count_)));

    precomputed_diameters_m_.clear();
    precomputed_diameters_m_.reserve(static_cast<size_t>(circle_diff_diam_count_));
    std::vector<float> grid_diameters(static_cast<size_t>(circle_diff_diam_count_));
    for (int i = 0; i < circle_diff_diam_count_; ++i) {
        const double ratio = static_cast<double>(i) / static_cast<double>(circle_diff_diam_count_ - 1);
        grid_diameters[static_cast<size_t>(i)] = static_cast<float>(
            circle_diff_diam_min_m_ + ratio * (circle_diff_diam_max_m_ - circle_diff_diam_min_m_));
    }
    int left = 0;
    int right = circle_diff_diam_count_ - 1;
    bool take_left = true;
    while (left <= right) {
        if (take_left) {
            precomputed_diameters_m_.push_back(grid_diameters[static_cast<size_t>(left++)]);
        } else {
            precomputed_diameters_m_.push_back(grid_diameters[static_cast<size_t>(right--)]);
        }
        take_left = !take_left;
    }

    // 4. Mode: circle_3minutes_stop
    circle_stop_duration_sec_ = this->declare_parameter<double>("circle_stop_duration_sec", circle_stop_duration_sec_);

    // 5. Mode: sinusoidal
    base_speed_ = static_cast<float>(
        this->declare_parameter<double>("base_speed", static_cast<double>(base_speed_)));
    sin_amplitude_ = static_cast<float>(
        this->declare_parameter<double>("sin_amplitude", static_cast<double>(sin_amplitude_)));
    sin_omega_ = static_cast<float>(
        this->declare_parameter<double>("sin_omega", static_cast<double>(sin_omega_)));

    // 6. Mode: random_walk
    sigma_v_ = static_cast<float>(
        this->declare_parameter<double>("sigma_v", static_cast<double>(sigma_v_)));
    max_speed_ = static_cast<float>(
        this->declare_parameter<double>("max_speed", static_cast<double>(max_speed_)));

    // 7. Mode: up_down
    updown_min_alt_ = this->declare_parameter<double>("updown_min_alt", updown_min_alt_);
    updown_max_alt_ = this->declare_parameter<double>("updown_max_alt", updown_max_alt_);
    updown_min_speed_ = this->declare_parameter<double>("updown_min_speed", updown_min_speed_);
    updown_max_speed_ = this->declare_parameter<double>("updown_max_speed", updown_max_speed_);
    updown_accel_ = this->declare_parameter<double>("updown_accel", updown_accel_);
    updown_loop_count_ = static_cast<int>(this->declare_parameter<int64_t>("updown_loop_count", updown_loop_count_));
    updown_takeoff_setpoint_count_ = static_cast<int>(
        this->declare_parameter<int64_t>("updown_takeoff_setpoint_count", updown_takeoff_setpoint_count_));
    updown_speed_ = updown_min_speed_;

    // 8. Mode: northing
    northing_cfg_.amplitude_m = this->declare_parameter<double>("northing_amplitude_m", northing_cfg_.amplitude_m);
    northing_cfg_.min_speed_mps = this->declare_parameter<double>("northing_min_speed_mps", northing_cfg_.min_speed_mps);
    northing_cfg_.max_speed_mps = this->declare_parameter<double>("northing_max_speed_mps", northing_cfg_.max_speed_mps);
    northing_cfg_.accel_mps2 = this->declare_parameter<double>("northing_accel_mps2", northing_cfg_.accel_mps2);
    northing_cfg_.loop_count = static_cast<int>(this->declare_parameter<int64_t>("northing_loop_count", northing_cfg_.loop_count));
    northing_cfg_.takeoff_altitude_m = this->declare_parameter<double>("northing_takeoff_altitude_m", northing_cfg_.takeoff_altitude_m);
    northing_cfg_.takeoff_setpoint_count = static_cast<int>(
        this->declare_parameter<int64_t>("northing_takeoff_setpoint_count", northing_cfg_.takeoff_setpoint_count));

    // 9. Mode: easting
    easting_cfg_.amplitude_m = this->declare_parameter<double>("easting_amplitude_m", easting_cfg_.amplitude_m);
    easting_cfg_.min_speed_mps = this->declare_parameter<double>("easting_min_speed_mps", easting_cfg_.min_speed_mps);
    easting_cfg_.max_speed_mps = this->declare_parameter<double>("easting_max_speed_mps", easting_cfg_.max_speed_mps);
    easting_cfg_.accel_mps2 = this->declare_parameter<double>("easting_accel_mps2", easting_cfg_.accel_mps2);
    easting_cfg_.loop_count = static_cast<int>(this->declare_parameter<int64_t>("easting_loop_count", easting_cfg_.loop_count));
    easting_cfg_.takeoff_altitude_m = this->declare_parameter<double>("easting_takeoff_altitude_m", easting_cfg_.takeoff_altitude_m);
    easting_cfg_.takeoff_setpoint_count = static_cast<int>(
        this->declare_parameter<int64_t>("easting_takeoff_setpoint_count", easting_cfg_.takeoff_setpoint_count));

    // 10. Mode: up50m_northing
    up50m_northing_cfg_.amplitude_m = this->declare_parameter<double>("up50m_northing_amplitude_m", up50m_northing_cfg_.amplitude_m);
    up50m_northing_cfg_.min_speed_mps = this->declare_parameter<double>("up50m_northing_min_speed_mps", up50m_northing_cfg_.min_speed_mps);
    up50m_northing_cfg_.max_speed_mps = this->declare_parameter<double>("up50m_northing_max_speed_mps", up50m_northing_cfg_.max_speed_mps);
    up50m_northing_cfg_.accel_mps2 = this->declare_parameter<double>("up50m_northing_accel_mps2", up50m_northing_cfg_.accel_mps2);
    up50m_northing_cfg_.loop_count = static_cast<int>(this->declare_parameter<int64_t>("up50m_northing_loop_count", up50m_northing_cfg_.loop_count));
    up50m_northing_cfg_.takeoff_altitude_m = this->declare_parameter<double>("up50m_northing_takeoff_altitude_m", up50m_northing_cfg_.takeoff_altitude_m);
    up50m_northing_cfg_.altitude_gate = this->declare_parameter<bool>("up50m_northing_altitude_gate", up50m_northing_cfg_.altitude_gate);
    up50m_northing_cfg_.altitude_tolerance_m = this->declare_parameter<double>("up50m_northing_altitude_tolerance_m", up50m_northing_cfg_.altitude_tolerance_m);
    up50m_northing_cfg_.takeoff_setpoint_count = static_cast<int>(
        this->declare_parameter<int64_t>("up50m_northing_takeoff_setpoint_count", up50m_northing_cfg_.takeoff_setpoint_count));

    // 11. Mode: up50m_easting
    up50m_easting_cfg_.amplitude_m = this->declare_parameter<double>("up50m_easting_amplitude_m", up50m_easting_cfg_.amplitude_m);
    up50m_easting_cfg_.min_speed_mps = this->declare_parameter<double>("up50m_easting_min_speed_mps", up50m_easting_cfg_.min_speed_mps);
    up50m_easting_cfg_.max_speed_mps = this->declare_parameter<double>("up50m_easting_max_speed_mps", up50m_easting_cfg_.max_speed_mps);
    up50m_easting_cfg_.accel_mps2 = this->declare_parameter<double>("up50m_easting_accel_mps2", up50m_easting_cfg_.accel_mps2);
    up50m_easting_cfg_.loop_count = static_cast<int>(this->declare_parameter<int64_t>("up50m_easting_loop_count", up50m_easting_cfg_.loop_count));
    up50m_easting_cfg_.takeoff_altitude_m = this->declare_parameter<double>("up50m_easting_takeoff_altitude_m", up50m_easting_cfg_.takeoff_altitude_m);
    up50m_easting_cfg_.altitude_gate = this->declare_parameter<bool>("up50m_easting_altitude_gate", up50m_easting_cfg_.altitude_gate);
    up50m_easting_cfg_.altitude_tolerance_m = this->declare_parameter<double>("up50m_easting_altitude_tolerance_m", up50m_easting_cfg_.altitude_tolerance_m);
    up50m_easting_cfg_.takeoff_setpoint_count = static_cast<int>(
        this->declare_parameter<int64_t>("up50m_easting_takeoff_setpoint_count", up50m_easting_cfg_.takeoff_setpoint_count));

    // 12. Mode: stationary
    stationary_x_ = static_cast<float>(
        this->declare_parameter<double>("stationary_x", static_cast<double>(circle_radius_)));
    stationary_y_ = static_cast<float>(
        this->declare_parameter<double>("stationary_y", 0.0));
    stationary_altitude_ = static_cast<float>(
        this->declare_parameter<double>("stationary_altitude", static_cast<double>(circle_altitude_)));
    stationary_yaw_ = static_cast<float>(
        this->declare_parameter<double>("stationary_yaw", 0.0));

    // Normalize namespace to a leading '/' and no trailing '/' / namespace ကို အရှေ့ '/' တစ်ခု၊ အနောက် '/' မပါအောင် ပုံစံချခြင်း
    if (!ns.empty() && ns.front() != '/') ns = "/" + ns;
    while (ns.size() > 1 && ns.back() == '/') ns.pop_back();
    if (target_system_param <= 0) {
        size_t i = ns.size();
        while (i > 0 && std::isdigit(static_cast<unsigned char>(ns[i - 1]))) --i;
        int instance = (i < ns.size()) ? std::stoi(ns.substr(i)) : 0;
        target_system_param = instance + 1;
    }
    target_system_ = static_cast<uint8_t>(target_system_param);

    if (mode_str == "circle")                motion_mode_ = MotionMode::CIRCLE;
    else if (mode_str == "circle_sine")      motion_mode_ = MotionMode::CIRCLE_SINE;
    else if (mode_str == "circle_sine_non_linear_accel") motion_mode_ = MotionMode::CIRCLE_SINE_NON_LINEAR_ACCEL;
    else if (mode_str == "circle_sine_non_linear_accel_with_different_diameters")
        motion_mode_ = MotionMode::CIRCLE_SINE_NON_LINEAR_ACCEL_WITH_DIFFERENT_DIAMETERS;
    else if (mode_str == "circle_3minutes_stop") motion_mode_ = MotionMode::CIRCLE_3MINUTES_STOP;
    else if (mode_str == "sinusoidal")       motion_mode_ = MotionMode::SINUSOIDAL;
    else if (mode_str == "random_walk")      motion_mode_ = MotionMode::RANDOM_WALK;
    else if (mode_str == "up_down")          motion_mode_ = MotionMode::UP_DOWN;
    else if (mode_str == "northing")         motion_mode_ = MotionMode::NORTHING;
    else if (mode_str == "easting")          motion_mode_ = MotionMode::EASTING;
    else if (mode_str == "up50m_northing")   motion_mode_ = MotionMode::UP50M_NORTHING;
    else if (mode_str == "up50m_easting")    motion_mode_ = MotionMode::UP50M_EASTING;
    else if (mode_str == "stationary")       motion_mode_ = MotionMode::STATIONARY;
    else {
        RCLCPP_WARN(this->get_logger(),
            "Unknown motion_mode '%s', defaulting to 'circle'", mode_str.c_str());
        motion_mode_ = MotionMode::CIRCLE;
    }

    RCLCPP_INFO(this->get_logger(), "=== Motion mode: %s | max_range: %.1fm | namespace: %s | target_system: %d ===",
        mode_str.c_str(), max_range_, ns.c_str(), target_system_);

    RCLCPP_INFO(this->get_logger(),
        "Available motion_mode values: circle, circle_sine, circle_sine_non_linear_accel, circle_sine_non_linear_accel_with_different_diameters, circle_3minutes_stop, sinusoidal, random_walk, up_down, northing, easting, up50m_northing, up50m_easting, stationary");

    if (motion_mode_ == MotionMode::CIRCLE_SINE_NON_LINEAR_ACCEL_WITH_DIFFERENT_DIAMETERS) {
        std::string sched_str;
        for (size_t i = 0; i < precomputed_diameters_m_.size(); ++i) {
            if (i > 0) sched_str += ", ";
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%.2fm", precomputed_diameters_m_[i]);
            sched_str += buf;
        }
        RCLCPP_INFO(this->get_logger(),
            "[circle_sine_non_linear_accel_with_different_diameters] Pre-calculated %zu diameters in [%.1fm, %.1fm]: [%s]",
            precomputed_diameters_m_.size(), circle_diff_diam_min_m_, circle_diff_diam_max_m_, sched_str.c_str());
    }

    // --- Publisher configuration / Publisher စီစဉ်သတ်မှတ်ခြင်း ---
    offboard_control_mode_publisher_ =
        this->create_publisher<OffboardControlMode>(ns + "/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ =
        this->create_publisher<TrajectorySetpoint>(ns + "/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ =
        this->create_publisher<VehicleCommand>(ns + "/fmu/in/vehicle_command", 10);

    // --- Subscriber configuration / Subscriber စီစဉ်သတ်မှတ်ခြင်း ---
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    vehicle_rates_setpoint_subscription_ = this->create_subscription<VehicleRatesSetpoint>(
        ns + "/fmu/out/vehicle_rates_setpoint", qos,
        [this](const VehicleRatesSetpoint::SharedPtr msg) { (void)msg; });

    global_position_sub_ = this->create_subscription<SensorGps>(
        ns + "/fmu/out/vehicle_gps_position", qos,
        std::bind(&UavTargetControl::global_position_callback, this, std::placeholders::_1));

    vehicle_command_ack_subscription_ = this->create_subscription<VehicleCommandAck>(
        ns + "/fmu/out/vehicle_command_ack", qos,
        std::bind(&UavTargetControl::vehicle_command_ack_callback, this, std::placeholders::_1));

    local_position_subscription_ = this->create_subscription<VehicleLocalPosition>(
        ns + "/fmu/out/vehicle_local_position_v1", qos,
        std::bind(&UavTargetControl::local_position_callback, this, std::placeholders::_1));

    vehicle_global_position_subscription_ = this->create_subscription<VehicleGlobalPosition>(
        ns + "/fmu/out/vehicle_global_position", qos,
        std::bind(&UavTargetControl::vehicle_global_position_callback, this, std::placeholders::_1));

    // --- State variable initialization / အခြေအနေ variable များ အစပြုသတ်မှတ်ခြင်း ---
    offboard_setpoint_counter_ = 0;
    theta_ = 0.0f;

    pos_x_ = 5.0f;
    pos_y_ = 0.0f;
    pos_z_ = -5.0f;
    vel_x_ = 0.0f;
    vel_y_ = 0.0f;
    sim_time_ = 0.0f;

    if (motion_mode_ == MotionMode::SINUSOIDAL) {
        vel_x_ = base_speed_;
    }

    // --- Timer / အချိန်ကိုက်စက် ---
    timer_ = this->create_wall_timer(100ms, [this]() {
        // up_down publishes its own offboard mode (position -> velocity); a position-mode message here would override it
        if (motion_mode_ != MotionMode::UP_DOWN || offboard_setpoint_counter_ <= 15) {
            publish_offboard_control_mode();
        }

        if (offboard_setpoint_counter_ < 15) {
            publish_trajectory_setpoint(5.0, 0.0, -5.0);
            offboard_setpoint_counter_++;
        }
        else if (offboard_setpoint_counter_ == 15) {
            RCLCPP_INFO(this->get_logger(), "Switching to Offboard and Arming...");
            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            this->arm();
            offboard_setpoint_counter_++;
        }
        else {
            execute_motion();
        }
    });
}

void UavTargetControl::arm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void UavTargetControl::publish_offboard_control_mode() {
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void UavTargetControl::publish_trajectory_setpoint(float x, float y, float z) {
    TrajectorySetpoint msg{};
    msg.position = {x, y, z};
    msg.yaw = 0.0;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

// ============================================================
//  Boundary constraint: applies a restoring force beyond max_range_
//  နယ်ပယ်ကန့်သတ်ချက် - max_range_ ကျော်လွန်ပါက ပြန်ဆွဲအား ပေးသည်
//  The target is smoothly pulled back near the boundary, keeping it within interceptor view
//  ပစ်မှတ်သည် နယ်နိမိတ်အနီးတွင် ချောမွေ့စွာ ပြန်ဆွဲခံရပြီး ပိတ်ဆို့လေယာဉ်၏ မြင်ကွင်းအတွင်း ရှိနေစေသည်
// ============================================================
void UavTargetControl::apply_boundary() {
    float dx = pos_x_ - boundary_center_x_;
    float dy = pos_y_ - boundary_center_y_;
    float dist = std::sqrt(dx * dx + dy * dy);

    if (dist > max_range_ * 0.8f && dist > 0.01f) {
        // Apply progressive restoring force starting at 80% of range / အချင်း၏ ၈၀% မှစတင်၍ တဖြည်းဖြည်း ပြန်ဆွဲအား ပေးသည်
        float overshoot = (dist - max_range_ * 0.8f) / (max_range_ * 0.2f);
        float restore = 3.0f * overshoot * overshoot;
        float nx = dx / dist;
        float ny = dy / dist;
        vel_x_ -= restore * nx * dt_;
        vel_y_ -= restore * ny * dt_;
    }
}

// ============================================================
//  Motion mode dispatch / လှုပ်ရှားမှုပုံစံ ရွေးချယ်ခြင်း
// ============================================================
void UavTargetControl::execute_motion() {
    switch (motion_mode_) {
        case MotionMode::CIRCLE_3MINUTES_STOP:         move_in_circle_3minutes_stop();         break;
        case MotionMode::CIRCLE_SINE_NON_LINEAR_ACCEL: move_in_circle_sine_non_linear_accel(); break;
        case MotionMode::CIRCLE_SINE_NON_LINEAR_ACCEL_WITH_DIFFERENT_DIAMETERS:
            move_in_circle_sine_non_linear_accel_with_different_diameters();
            break;
        case MotionMode::CIRCLE_SINE:
        case MotionMode::CIRCLE:          move_in_circle();         break;
        case MotionMode::SINUSOIDAL:      move_sinusoidal();        break;
        case MotionMode::RANDOM_WALK:     move_random_walk();       break;
        case MotionMode::UP_DOWN:         move_up_down();           break;
        case MotionMode::NORTHING:        move_northing();          break;
        case MotionMode::EASTING:         move_easting();           break;
        case MotionMode::UP50M_NORTHING:  move_up50m_northing();    break;
        case MotionMode::UP50M_EASTING:   move_up50m_easting();     break;
        case MotionMode::STATIONARY:      move_stationary();        break;
    }
}

// ============================================================
//  1. Uniform circular motion (original, naturally bounded, no constraint needed)
//     အမြန်နှုန်းတူ စက်ဝိုင်းလှုပ်ရှားမှု (မူလ, သဘာဝအရ နယ်ပယ်အတွင်းရှိသဖြင့် ကန့်သတ်ရန်မလို)
// ============================================================
void UavTargetControl::move_in_circle() {
    theta_ += angular_speed_;
    if (theta_ > 2 * M_PI) theta_ -= 2 * M_PI;

    radius_ = circle_radius_;

    float x = radius_ * std::cos(theta_);
    float y = radius_ * std::sin(theta_);
    float altitude = circle_altitude_;
    if (motion_mode_ == MotionMode::CIRCLE_SINE) {
        altitude += circle_altitude_amplitude_
            * std::sin(circle_altitude_cycles_ * theta_);
    }

    TrajectorySetpoint msg{};
    msg.position = {x, y, -altitude};
    msg.yaw = theta_;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

// ============================================================
//  1b. Circle + Sine (>= 4 cycles per circle) with non-linear acceleration
//      စက်ဝိုင်းတစ်ပတ်လျှင် အနည်းဆုံး sine wave 4 ကြိမ်နှင့် non-linear အရှိန်ပြောင်းလဲမှု
// ============================================================
void UavTargetControl::move_in_circle_sine_non_linear_accel() {
    sim_time_ += dt_;

    // Non-linear angular acceleration law: signed quadratic harmonic + position-coupled term + cubic restoring damping
    const float s_t = std::sin(circle_sine_nl_freq_ * sim_time_);
    const float dev = nl_omega_scale_ - 1.0f;
    const float nl_accel = circle_sine_nl_quad_gain_ * s_t * std::abs(s_t)
                         + circle_sine_nl_pos_gain_ * std::cos(3.0f * theta_)
                         - circle_sine_nl_cubic_damping_ * dev * dev * dev;
    nl_omega_scale_ = std::max(
        circle_sine_nl_omega_scale_min_,
        std::min(circle_sine_nl_omega_scale_max_, nl_omega_scale_ + nl_accel * dt_));

    theta_ += angular_speed_ * nl_omega_scale_;
    if (theta_ > 2.0f * static_cast<float>(M_PI)) {
        theta_ -= 2.0f * static_cast<float>(M_PI);
    }

    radius_ = circle_radius_;

    // Enforce at least 4 (or circle_sine_nl_min_cycles_) sine wave cycles per circle revolution
    const int altitude_cycles = std::max(circle_sine_nl_min_cycles_, circle_altitude_cycles_);

    const float x = radius_ * std::cos(theta_);
    const float y = radius_ * std::sin(theta_);
    const float nl_phase = static_cast<float>(altitude_cycles) * theta_ + circle_sine_nl_phase_mod_ * std::sin(2.0f * theta_);
    const float altitude = circle_altitude_ + circle_altitude_amplitude_ * std::sin(nl_phase);

    TrajectorySetpoint msg{};
    msg.position = {x, y, -altitude};
    msg.yaw = theta_;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

// ============================================================
//  1c. Circle + Sine (>= 4 cycles per circle) with non-linear acceleration
//      and pre-calculated varying diameters (10m to 20m)
//      ကြိုတင်တွက်ချက်ထားသော Diameter (10m မှ 20m) အမျိုးမျိုးဖြင့်
//      circle_sine_non_linear_accel အတိုင်း ပျံသန်းခြင်း
// ============================================================
void UavTargetControl::move_in_circle_sine_non_linear_accel_with_different_diameters() {
    sim_time_ += dt_;

    // Non-linear angular acceleration law (same as circle_sine_non_linear_accel)
    const float s_t = std::sin(circle_sine_nl_freq_ * sim_time_);
    const float dev = nl_omega_scale_ - 1.0f;
    const float nl_accel = circle_sine_nl_quad_gain_ * s_t * std::abs(s_t)
                         + circle_sine_nl_pos_gain_ * std::cos(3.0f * theta_)
                         - circle_sine_nl_cubic_damping_ * dev * dev * dev;
    nl_omega_scale_ = std::max(
        circle_sine_nl_omega_scale_min_,
        std::min(circle_sine_nl_omega_scale_max_, nl_omega_scale_ + nl_accel * dt_));

    const float two_pi = 2.0f * static_cast<float>(M_PI);
    theta_ += angular_speed_ * nl_omega_scale_;
    if (theta_ >= two_pi) {
        theta_ -= two_pi;
        if (!precomputed_diameters_m_.empty()) {
            diff_diam_lap_index_ = (diff_diam_lap_index_ + 1) % precomputed_diameters_m_.size();
            const size_t next_idx = (diff_diam_lap_index_ + 1) % precomputed_diameters_m_.size();
            RCLCPP_INFO(this->get_logger(),
                "[circle_sine_non_linear_accel_with_different_diameters] Lap %zu: diameter %.2fm -> %.2fm",
                diff_diam_lap_index_,
                precomputed_diameters_m_[diff_diam_lap_index_],
                precomputed_diameters_m_[next_idx]);
        }
    }

    // Smoothly interpolate between pre-calculated lap diameters (10m .. 20m) over the circle revolution
    float current_diameter_m = 2.0f * circle_radius_;
    if (!precomputed_diameters_m_.empty()) {
        const float d_curr = precomputed_diameters_m_[diff_diam_lap_index_];
        const float d_next = precomputed_diameters_m_[(diff_diam_lap_index_ + 1) % precomputed_diameters_m_.size()];
        const float progress = std::max(0.0f, std::min(1.0f, theta_ / two_pi));
        const float blend = 0.5f * (1.0f - std::cos(static_cast<float>(M_PI) * progress));
        current_diameter_m = (1.0f - blend) * d_curr + blend * d_next;
    }
    radius_ = 0.5f * current_diameter_m;

    // Enforce at least 4 (or circle_sine_nl_min_cycles_) sine wave cycles per circle revolution
    const int altitude_cycles = std::max(circle_sine_nl_min_cycles_, circle_altitude_cycles_);

    const float x = radius_ * std::cos(theta_);
    const float y = radius_ * std::sin(theta_);
    const float nl_phase = static_cast<float>(altitude_cycles) * theta_ + circle_sine_nl_phase_mod_ * std::sin(2.0f * theta_);
    const float altitude = circle_altitude_ + circle_altitude_amplitude_ * std::sin(nl_phase);

    TrajectorySetpoint msg{};
    msg.position = {x, y, -altitude};
    msg.yaw = theta_;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

void UavTargetControl::move_in_circle_3minutes_stop() {
    const auto now = std::chrono::steady_clock::now();
    if (!timed_circle_started_) {
        timed_circle_started_ = true;
        timed_circle_started_at_ = now;
        RCLCPP_INFO(this->get_logger(),
            "[circle_3minutes_stop] Circle started; holding current position after %.1f seconds",
            circle_stop_duration_sec_);
    }

    if (!timed_circle_stopped_ &&
        now - timed_circle_started_at_ < std::chrono::duration<double>(circle_stop_duration_sec_)) {
        move_in_circle();
        return;
    }

    if (!timed_circle_stopped_) {
        const bool fresh_feedback = circle_feedback_valid_ &&
            now - circle_feedback_received_at_ <= std::chrono::milliseconds(500);
        circle_hold_x_ = fresh_feedback ? static_cast<float>(local_x_)
            : circle_radius_ * std::cos(theta_);
        circle_hold_y_ = fresh_feedback ? static_cast<float>(local_y_)
            : circle_radius_ * std::sin(theta_);
        circle_hold_z_ = fresh_feedback ? static_cast<float>(-local_altitude_)
            : -circle_altitude_;
        circle_hold_yaw_ = fresh_feedback ? circle_feedback_heading_ : theta_;
        timed_circle_stopped_ = true;
        if (!fresh_feedback) {
            RCLCPP_WARN(this->get_logger(),
                "[circle_3minutes_stop] No fresh valid local position; holding last circle setpoint");
        }
        RCLCPP_INFO(this->get_logger(),
            "[circle_3minutes_stop] STOP: fixed NED hold=(%.3f, %.3f, %.3f), yaw=%.3f, measured=%s",
            circle_hold_x_, circle_hold_y_, circle_hold_z_, circle_hold_yaw_,
            fresh_feedback ? "true" : "false");
    }

    TrajectorySetpoint msg{};
    msg.position = {circle_hold_x_, circle_hold_y_, circle_hold_z_};
    msg.yaw = circle_hold_yaw_;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

// ============================================================
//  2. Sinusoidal maneuver: a_T = A*sin(ωt), A=0.5m/s², ω=0.5rad/s
//     Forward constant speed + lateral sinusoidal acceleration + boundary constraint
//     ဆိုင်းလှိုင်းပုံစံလှုပ်ရှားမှု - ရှေ့ဘက်အမြန်နှုန်းတူမှု + ဘေးတိုက်ဆိုင်းလှိုင်းအရှိန် + နယ်ပယ်ကန့်သတ်ချက်
// ============================================================
void UavTargetControl::move_sinusoidal() {
    sim_time_ += dt_;

    float a_y = sin_amplitude_ * std::sin(sin_omega_ * sim_time_);
    vel_y_ += a_y * dt_;

    apply_boundary();

    pos_x_ += vel_x_ * dt_;
    pos_y_ += vel_y_ * dt_;

    TrajectorySetpoint msg{};
    msg.position = {pos_x_, pos_y_, pos_z_};
    msg.yaw = std::atan2(vel_y_, vel_x_);
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

// ============================================================
//  3. Random walk: v_T(t+Δt) = v_T(t) + N(0, σ_v), σ_v=0.2m/s
//     Irregular escape + boundary constraint
//     ကျပန်းလမ်းလျှောက်မှု - စည်းမဲ့ ထွက်ပြေးမှု + နယ်ပယ်ကန့်သတ်ချက်
// ============================================================
void UavTargetControl::move_random_walk() {
    vel_x_ += sigma_v_ * normal_dist_(rng_);
    vel_y_ += sigma_v_ * normal_dist_(rng_);

    float speed = std::sqrt(vel_x_ * vel_x_ + vel_y_ * vel_y_);
    if (speed > max_speed_) {
        vel_x_ *= max_speed_ / speed;
        vel_y_ *= max_speed_ / speed;
    }

    apply_boundary();

    pos_x_ += vel_x_ * dt_;
    pos_y_ += vel_y_ * dt_;

    TrajectorySetpoint msg{};
    msg.position = {pos_x_, pos_y_, pos_z_};
    msg.yaw = std::atan2(vel_y_, vel_x_);
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

// ============================================================
//  4. Up/down: absolute AMSL altitude oscillation between kUpDownMinAlt_/kUpDownMaxAlt_
//     Ported from ct_uav_movement_tester/solo/01_test_up_down.py
// ============================================================
void UavTargetControl::move_up_down() {
    if (updown_handoff_requested_) return;

    OffboardControlMode mode{};
    mode.position = !updown_offboard_started_;
    mode.velocity = updown_offboard_started_;
    mode.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(mode);

    if (!updown_offboard_started_) {
        TrajectorySetpoint msg{};
        msg.position = {0.0f, 0.0f, static_cast<float>(-updown_min_alt_)};
        msg.velocity = {NAN, NAN, NAN};
        msg.yaw = 0.0;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);

        if (++updown_setpoint_count_ >= updown_takeoff_setpoint_count_) {
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
            arm();
            updown_offboard_started_ = true;
        }
        return;
    }

    updown_speed_ += updown_accel_ * dt_;
    if (updown_speed_ > updown_max_speed_) updown_speed_ = updown_max_speed_;
    if (updown_speed_ < updown_min_speed_) updown_speed_ = updown_min_speed_;

    // AMSL minus the fixed ref_alt_amsl_ is wrong in grass_world (AMSL ~488 m), so use altitude above home
    if (updown_direction_ > 0 && local_altitude_ >= updown_max_alt_) {
        updown_direction_ = -1.0;
        updown_speed_ = updown_min_speed_;
    } else if (updown_direction_ < 0 && local_altitude_ <= updown_min_alt_) {
        updown_direction_ = 1.0;
        updown_speed_ = updown_min_speed_;
        if (++updown_loops_completed_ >= updown_loop_count_) {
            updown_handoff_requested_ = true;
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 3.0); // PX4_CUSTOM_MAIN_MODE_POSCTL
            return;
        }
    }

    TrajectorySetpoint msg{};
    msg.position = {NAN, NAN, NAN};
    msg.velocity = {0.0f, 0.0f, static_cast<float>(-updown_direction_ * updown_speed_)};
    msg.yaw = NAN;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

// ============================================================
//  5. Shared north/east axis oscillation engine for northing/easting/up50m_* modes
//     Ported from ct_uav_movement_tester/solo/02_test_northing.py, 03_test_easting.py,
//     08_up_50m_and_northing.py, 09_up_50m_and_easting.py (09 implemented as true easting; the
//     source file is a byte-identical copy of 08's northing code)
// ============================================================
void UavTargetControl::run_axis_oscillation(const AxisOscillationConfig &cfg, AxisOscillationState &st) {
    if (st.handoff_requested) return;
    if (!local_position_received_) return; // wait for the first local position message before arming

    if (!st.origin_captured) {
        st.origin = cfg.use_y ? local_y_ : local_x_;
        st.origin_captured = true;
        st.speed = cfg.min_speed_mps;
    }

    OffboardControlMode mode{};
    mode.position = !st.takeoff_complete;
    mode.velocity = st.takeoff_complete;
    mode.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(mode);

    if (!st.takeoff_complete) {
        TrajectorySetpoint msg{};
        msg.position = {
            static_cast<float>(cfg.use_y ? 0.0 : st.origin),
            static_cast<float>(cfg.use_y ? st.origin : 0.0),
            static_cast<float>(-cfg.takeoff_altitude_m)};
        msg.velocity = {NAN, NAN, NAN};
        msg.yaw = 0.0;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);

        if (!st.offboard_started && ++st.setpoint_count >= cfg.takeoff_setpoint_count) {
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
            arm();
            st.offboard_started = true;
        }

        if (cfg.altitude_gate) {
            if (st.offboard_started && std::abs(local_altitude_ - cfg.takeoff_altitude_m) <= cfg.altitude_tolerance_m) {
                st.takeoff_complete = true;
            }
        } else if (st.offboard_started) {
            st.takeoff_complete = true;
        }
        return;
    }

    double current = cfg.use_y ? local_y_ : local_x_;
    double max_bound = st.origin + cfg.amplitude_m;
    double min_bound = st.origin - cfg.amplitude_m;

    st.speed += cfg.accel_mps2 * dt_;
    if (st.speed > cfg.max_speed_mps) st.speed = cfg.max_speed_mps;
    if (st.speed < cfg.min_speed_mps) st.speed = cfg.min_speed_mps;

    if (st.direction > 0 && current >= max_bound) {
        st.direction = -1.0;
        st.speed = cfg.min_speed_mps;
    } else if (st.direction < 0 && current <= min_bound) {
        st.direction = 1.0;
        st.speed = cfg.min_speed_mps;
        if (++st.loops_completed >= cfg.loop_count) {
            st.handoff_requested = true;
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 3.0); // PX4_CUSTOM_MAIN_MODE_POSCTL
            return;
        }
    }

    TrajectorySetpoint msg{};
    msg.position = {NAN, NAN, NAN};
    msg.velocity = {
        static_cast<float>(cfg.use_y ? 0.0 : st.direction * st.speed),
        static_cast<float>(cfg.use_y ? st.direction * st.speed : 0.0),
        0.0f};
    msg.yaw = NAN;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

void UavTargetControl::move_northing() {
    run_axis_oscillation(northing_cfg_, northing_state_);
}

void UavTargetControl::move_easting() {
    run_axis_oscillation(easting_cfg_, easting_state_);
}

void UavTargetControl::move_up50m_northing() {
    run_axis_oscillation(up50m_northing_cfg_, up50m_northing_state_);
}

void UavTargetControl::move_up50m_easting() {
    run_axis_oscillation(up50m_easting_cfg_, up50m_easting_state_);
}

// ============================================================
//  9. Stationary hover: fly to the circle start point and hold
//     စတপ်နေထိုင်မှု - circle mode ရဲ့ စတင်နေရာသို့ သွားပြီး hover ဖြစ်နေခြင်း
//     Holds (stationary_x_, stationary_y_, -stationary_altitude_) with fixed yaw
// ============================================================
void UavTargetControl::move_stationary() {
    TrajectorySetpoint msg{};
    msg.position = {stationary_x_, stationary_y_, -stationary_altitude_};
    msg.yaw = stationary_yaw_;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

// ============================================================
//  Vehicle command & GPS callback
// ============================================================
void UavTargetControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = target_system_;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

void UavTargetControl::global_position_callback(const px4_msgs::msg::SensorGps::SharedPtr msg)
{
    (void)msg;
    if (!msg) {
        RCLCPP_WARN(this->get_logger(), "Received null GPS message");
        return;
    }
}

void UavTargetControl::local_position_callback(const VehicleLocalPosition::SharedPtr msg)
{
    local_x_ = msg->x;
    local_y_ = msg->y;
    local_altitude_ = -msg->z;
    local_position_received_ = true;
    circle_feedback_valid_ = msg->xy_valid && msg->z_valid &&
        std::isfinite(msg->x) && std::isfinite(msg->y) &&
        std::isfinite(msg->z) && std::isfinite(msg->heading);
    circle_feedback_heading_ = msg->heading;
    circle_feedback_received_at_ = std::chrono::steady_clock::now();
}

void UavTargetControl::vehicle_global_position_callback(const VehicleGlobalPosition::SharedPtr msg)
{
    global_altitude_amsl_m_ = msg->alt;
}

// Reports PX4's real accept/reject reason for arm & DO_SET_MODE commands
void UavTargetControl::vehicle_command_ack_callback(const VehicleCommandAck::SharedPtr msg)
{
    const char* result_str = "UNKNOWN";
    switch (msg->result) {
        case VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED:          result_str = "ACCEPTED"; break;
        case VehicleCommandAck::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED: result_str = "TEMPORARILY_REJECTED"; break;
        case VehicleCommandAck::VEHICLE_CMD_RESULT_DENIED:             result_str = "DENIED"; break;
        case VehicleCommandAck::VEHICLE_CMD_RESULT_UNSUPPORTED:        result_str = "UNSUPPORTED"; break;
        case VehicleCommandAck::VEHICLE_CMD_RESULT_FAILED:             result_str = "FAILED"; break;
        case VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS:        result_str = "IN_PROGRESS"; break;
        case VehicleCommandAck::VEHICLE_CMD_RESULT_CANCELLED:          result_str = "CANCELLED"; break;
        default: break;
    }

    if (msg->result == VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
        RCLCPP_INFO(this->get_logger(), "Command %d ACK: %s", msg->command, result_str);
    } else {
        RCLCPP_WARN(this->get_logger(), "Command %d ACK: %s (result=%d)", msg->command, result_str, msg->result);
    }
}

#pragma once
#include <algorithm>
#include <cmath>
#include <string>
#include <rclcpp/rclcpp.hpp>

// Dynamically adjusts IMU bias process noise (Q_ba, Q_bg) based on sonar availability.
//
// When sonar is active, position corrections constrain ba/bg — normal process noise applies.
// When sonar is absent beyond a timeout, ba/bg become unobservable and must be frozen to
// prevent double-integration runaway (~30 km error in 400s from a 0.038g unconstrained bias).
//
// Replaces the static `noiseless_imu` flag with continuous, automatic Q switching.
class ObservabilityManager
{
public:
    struct Params
    {
        double sonar_timeout_s  = 0.5;    // seconds without sonar scan → begin freezing
        double ramp_time_s      = 2.0;    // seconds to ramp Q back up when sonar returns
        double freeze_cov       = 1e-10;  // process noise when frozen (effectively zero)
        double normal_bg_cov    = 1e-4;   // normal gyro bias process noise
        double normal_ba_cov    = 1e-4;   // normal accel bias process noise
    };

    ObservabilityManager() = default;
    explicit ObservabilityManager(const Params &p) : p_(p) {}

    // Call this whenever a sonar scan is received and processed.
    void notify_sonar_scan(double timestamp)
    {
        last_sonar_time_ = timestamp;
    }

    // Returns current ba/bg process noise (diagonal scalar, same for all 3 axes).
    // Call this once per IMU propagation step, before p_imu->Process().
    //
    // Uses a piecewise ramp:
    //   sonar_age <= timeout             → normal cov
    //   timeout < sonar_age <= timeout+ramp → linear ramp freeze_cov..normal_cov (reversed: ramp DOWN when going stale)
    //   sonar_age > timeout + ramp      → freeze_cov
    //
    // When sonar comes back after a gap, ramps UP linearly over ramp_time_s.
    double bg_cov(double now) const { return compute_cov(now); }
    double ba_cov(double now) const { return compute_cov(now); }

    bool is_frozen(double now) const
    {
        return sonar_age(now) > p_.sonar_timeout_s + p_.ramp_time_s;
    }

    // For logging: seconds since last sonar scan (-1 if never seen).
    double sonar_age(double now) const
    {
        if (last_sonar_time_ < 0.0)
            return p_.sonar_timeout_s + p_.ramp_time_s + 1.0; // treat as long-absent
        return now - last_sonar_time_;
    }

    // Declare ROS2 parameters for this manager under the "mapping" namespace.
    static void declare_parameters(rclcpp::Node &node)
    {
        node.declare_parameter<double>("mapping.obs_manager_sonar_timeout", 0.5);
        node.declare_parameter<double>("mapping.obs_manager_ramp_time",     2.0);
        node.declare_parameter<double>("mapping.obs_manager_freeze_cov",    1e-10);
    }

    // Load parameters. normal_bg/ba_cov are taken from the existing b_gyr_cov / b_acc_cov config.
    void load_parameters(rclcpp::Node &node, double normal_bg_cov, double normal_ba_cov)
    {
        node.get_parameter_or("mapping.obs_manager_sonar_timeout", p_.sonar_timeout_s, 0.5);
        node.get_parameter_or("mapping.obs_manager_ramp_time",     p_.ramp_time_s,     2.0);
        node.get_parameter_or("mapping.obs_manager_freeze_cov",    p_.freeze_cov,      1e-10);
        p_.normal_bg_cov = normal_bg_cov;
        p_.normal_ba_cov = normal_ba_cov;
    }

private:
    double compute_cov(double now) const
    {
        const double age = sonar_age(now);
        if (age <= p_.sonar_timeout_s)
            return p_.normal_bg_cov;
        const double ramp_age = age - p_.sonar_timeout_s;
        if (ramp_age >= p_.ramp_time_s)
            return p_.freeze_cov;
        // linear ramp from normal down to freeze
        const double alpha = ramp_age / p_.ramp_time_s; // 0 → 1
        return p_.normal_bg_cov * (1.0 - alpha) + p_.freeze_cov * alpha;
    }

    Params p_;
    double last_sonar_time_ = -1.0;
};

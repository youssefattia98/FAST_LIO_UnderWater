#ifndef AUXILIARY_SENSOR_FUSION_HPP
#define AUXILIARY_SENSOR_FUSION_HPP

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <mutex>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "common_lib.h"
#include "use-ikfom.hpp"

class AuxiliarySensorFusion
{
public:
    using Ekf = esekfom::esekf<state_ikfom, 15, input_ikfom>;
    using DvlMsg = geometry_msgs::msg::TwistWithCovarianceStamped;
    using PressureMsg = sensor_msgs::msg::FluidPressure;
    using MagMsg = sensor_msgs::msg::MagneticField;

    struct UpdateSummary
    {
        int dvl_count = 0;
        int dvl_accepted = 0;
        int dvl_rejected = 0;
        int pressure_count = 0;
        int pressure_accepted = 0;
        int pressure_rejected = 0;
        int mag_count = 0;
        int mag_accepted = 0;
        int mag_rejected = 0;
        double dvl_res_norm_sum = 0.0;
        double dvl_res_norm_max = 0.0;
        V3D dvl_res_sum = V3D::Zero();
        V3D dvl_res_abs_max = V3D::Zero();
        V3D dvl_meas_sum = V3D::Zero();
        V3D dvl_pred_sum = V3D::Zero();
        V3D dvl_body_vel_sum = V3D::Zero();
        double pressure_res_depth_sum = 0.0;
        double pressure_res_depth_max = 0.0;
        double mag_res_norm_sum = 0.0;
        double mag_res_norm_max = 0.0;
        bool dvl_updated = false;
        bool pressure_updated = false;
        bool mag_updated = false;

        bool updated() const
        {
            return dvl_updated || pressure_updated || mag_updated;
        }

        double mean_dvl_residual() const
        {
            return dvl_count > 0 ? dvl_res_norm_sum / static_cast<double>(dvl_count) : 0.0;
        }

        V3D mean_dvl_residual_axis() const
        {
            if (dvl_count <= 0)
            {
                return V3D::Zero();
            }
            return dvl_res_sum / static_cast<double>(dvl_count);
        }

        V3D mean_dvl_measurement() const
        {
            if (dvl_count <= 0)
            {
                return V3D::Zero();
            }
            return dvl_meas_sum / static_cast<double>(dvl_count);
        }

        V3D mean_dvl_prediction() const
        {
            if (dvl_count <= 0)
            {
                return V3D::Zero();
            }
            return dvl_pred_sum / static_cast<double>(dvl_count);
        }

        V3D mean_dvl_body_velocity() const
        {
            if (dvl_count <= 0)
            {
                return V3D::Zero();
            }
            return dvl_body_vel_sum / static_cast<double>(dvl_count);
        }

        double mean_pressure_depth_residual() const
        {
            return pressure_count > 0 ? pressure_res_depth_sum / static_cast<double>(pressure_count) : 0.0;
        }

        double mean_mag_residual() const
        {
            return mag_count > 0 ? mag_res_norm_sum / static_cast<double>(mag_count) : 0.0;
        }
    };

    void declare_parameters(rclcpp::Node &node)
    {
        node.declare_parameter<bool>("dvl.enable", false);
        node.declare_parameter<std::string>("dvl.topic", "/auv/dvl");
        node.declare_parameter<std::vector<double>>("dvl.extrinsic_T", {-0.079, -0.09691, -0.25938});
        node.declare_parameter<std::vector<double>>("dvl.extrinsic_R",
                                                    {1.0, 0.0, 0.0,
                                                     0.0, 1.0, 0.0,
                                                     0.0, 0.0, 1.0});
        node.declare_parameter<double>("dvl.dvl_timeout", 0.25);

        node.declare_parameter<bool>("pressure.enable", false);
        node.declare_parameter<std::string>("pressure.topic", "/auv/pressure/scaled2");
        node.declare_parameter<std::vector<double>>("pressure.extrinsic_T", {-0.24219, -0.03954, 0.01898});
        node.declare_parameter<double>("pressure.pressure_timeout", 0.25);

        node.declare_parameter<double>("dvl.velocity_cov", 4e-4);
        node.declare_parameter<double>("dvl.innovation_gate_sigma", 5.0);
        node.declare_parameter<double>("dvl.bias_init_cov", 1e-3);
        node.declare_parameter<double>("pressure.pressure_cov", 1e4);
        node.declare_parameter<double>("pressure.bias_init_cov", 1e4);
        node.declare_parameter<double>("pressure.innovation_gate_sigma", 0.0);
        node.declare_parameter<double>("pressure.fluid_density", 1025.0);
        node.declare_parameter<double>("pressure.gravity", 9.80665);
        node.declare_parameter<double>("pressure.surface_pressure", 101325.0);
        node.declare_parameter<double>("pressure.surface_z", 0.0);

        node.declare_parameter<bool>("magnetometer.enable", false);
        node.declare_parameter<std::string>("magnetometer.topic", "/auv/imu/magnetic_field");
        node.declare_parameter<std::vector<double>>("magnetometer.hard_iron_offset", {0.0, 0.0, 0.0});
        node.declare_parameter<std::vector<double>>("magnetometer.soft_iron_matrix",
            {1., 0., 0.,  0., 1., 0.,  0., 0., 1.});
        node.declare_parameter<std::vector<double>>("magnetometer.axis_sign", {1.0, 1.0, 1.0});
        node.declare_parameter<double>("magnetometer.mag_cov", 1849.0);
        node.declare_parameter<double>("magnetometer.b_mag_init_cov", 1e6);
        node.declare_parameter<double>("magnetometer.b_mag_proc_cov", 0.001);
        node.declare_parameter<double>("magnetometer.innovation_gate_sigma", 3.0);
        node.declare_parameter<double>("magnetometer.mag_timeout", 0.5);
    }

    void load_parameters(rclcpp::Node &node)
    {
        node.get_parameter_or<bool>("dvl.enable", dvl_enable_, false);
        node.get_parameter_or<std::string>("dvl.topic", dvl_topic_, "/auv/dvl");
        node.get_parameter_or<double>("dvl.dvl_timeout", dvl_timeout_, 0.25);
        node.get_parameter_or<double>("dvl.velocity_cov", dvl_velocity_cov_, 4e-4);
        node.get_parameter_or<double>("dvl.innovation_gate_sigma", dvl_innovation_gate_sigma_, 5.0);
        node.get_parameter_or<double>("dvl.bias_init_cov", dvl_b_init_cov_, 1e-3);

        node.get_parameter_or<bool>("pressure.enable", pressure_enable_, false);
        node.get_parameter_or<std::string>("pressure.topic", pressure_topic_, "/auv/pressure/scaled2");
        node.get_parameter_or<double>("pressure.pressure_timeout", pressure_timeout_, 0.25);
        node.get_parameter_or<double>("pressure.pressure_cov", pressure_cov_, 1e4);
        node.get_parameter_or<double>("pressure.bias_init_cov", pressure_b_init_cov_, 1e4);
        node.get_parameter_or<double>("pressure.innovation_gate_sigma", pressure_innovation_gate_sigma_, 0.0);
        node.get_parameter_or<double>("pressure.fluid_density", pressure_fluid_density_, 1025.0);
        node.get_parameter_or<double>("pressure.gravity", pressure_gravity_, 9.80665);
        node.get_parameter_or<double>("pressure.surface_pressure", pressure_surface_pressure_, 101325.0);
        node.get_parameter_or<double>("pressure.surface_z", pressure_surface_z_, 0.0);

        std::vector<double> dvl_T;
        std::vector<double> dvl_R;
        std::vector<double> pressure_T;
        node.get_parameter_or<std::vector<double>>("dvl.extrinsic_T", dvl_T, {-0.079, -0.09691, -0.25938});
        node.get_parameter_or<std::vector<double>>("dvl.extrinsic_R", dvl_R,
                                                   {1.0, 0.0, 0.0,
                                                    0.0, 1.0, 0.0,
                                                    0.0, 0.0, 1.0});
        node.get_parameter_or<std::vector<double>>("pressure.extrinsic_T", pressure_T, {-0.24219, -0.03954, 0.01898});

        if (dvl_T.size() == 3)
        {
            dvl_T_ << dvl_T[0], dvl_T[1], dvl_T[2];
        }
        else
        {
            RCLCPP_WARN(node.get_logger(), "dvl.extrinsic_T must have 3 values. Using zero translation.");
            dvl_T_.setZero();
        }

        if (dvl_R.size() == 9)
        {
            dvl_R_ << dvl_R[0], dvl_R[1], dvl_R[2],
                      dvl_R[3], dvl_R[4], dvl_R[5],
                      dvl_R[6], dvl_R[7], dvl_R[8];
        }
        else
        {
            RCLCPP_WARN(node.get_logger(), "dvl.extrinsic_R must have 9 values. Using identity rotation.");
            dvl_R_.setIdentity();
        }

        if (pressure_T.size() == 3)
        {
            pressure_T_ << pressure_T[0], pressure_T[1], pressure_T[2];
        }
        else
        {
            RCLCPP_WARN(node.get_logger(), "pressure.extrinsic_T must have 3 values. Using zero translation.");
            pressure_T_.setZero();
        }

        node.get_parameter_or<bool>("magnetometer.enable", mag_enable_, false);
        node.get_parameter_or<std::string>("magnetometer.topic", mag_topic_, "/auv/imu/magnetic_field");
        node.get_parameter_or<double>("magnetometer.mag_cov", mag_cov_, 1849.0);
        node.get_parameter_or<double>("magnetometer.b_mag_init_cov", mag_b_init_cov_, 1e6);
        node.get_parameter_or<double>("magnetometer.b_mag_proc_cov", mag_b_proc_cov_, 0.001);
        node.get_parameter_or<double>("magnetometer.innovation_gate_sigma", mag_innovation_gate_sigma_, 3.0);
        node.get_parameter_or<double>("magnetometer.mag_timeout", mag_timeout_, 0.5);

        std::vector<double> hard_iron, soft_iron, axis_sign;
        node.get_parameter_or<std::vector<double>>("magnetometer.hard_iron_offset", hard_iron, {0., 0., 0.});
        node.get_parameter_or<std::vector<double>>("magnetometer.soft_iron_matrix", soft_iron,
            {1., 0., 0.,  0., 1., 0.,  0., 0., 1.});
        node.get_parameter_or<std::vector<double>>("magnetometer.axis_sign", axis_sign, {1., 1., 1.});

        if (hard_iron.size() == 3)
            mag_hard_iron_ << hard_iron[0], hard_iron[1], hard_iron[2];
        else
            mag_hard_iron_.setZero();

        if (soft_iron.size() == 9)
        {
            mag_soft_iron_ << soft_iron[0], soft_iron[1], soft_iron[2],
                              soft_iron[3], soft_iron[4], soft_iron[5],
                              soft_iron[6], soft_iron[7], soft_iron[8];
        }
        else
        {
            mag_soft_iron_.setIdentity();
        }
        if (axis_sign.size() == 3)
            mag_axis_sign_ << axis_sign[0], axis_sign[1], axis_sign[2];
        else
            mag_axis_sign_.setOnes();

        dvl_timeout_ = std::max(0.0, dvl_timeout_);
        pressure_timeout_ = std::max(0.0, pressure_timeout_);
        mag_timeout_ = std::max(0.0, mag_timeout_);
        dvl_velocity_cov_ = std::max(1e-12, dvl_velocity_cov_);
        dvl_innovation_gate_sigma_ = std::max(0.0, dvl_innovation_gate_sigma_);
        dvl_b_init_cov_ = std::max(1e-12, dvl_b_init_cov_);
        pressure_cov_ = std::max(1e-6, pressure_cov_);
        pressure_b_init_cov_ = std::max(1e-12, pressure_b_init_cov_);
        pressure_innovation_gate_sigma_ = std::max(0.0, pressure_innovation_gate_sigma_);
        pressure_fluid_density_ = std::max(1e-6, pressure_fluid_density_);
        pressure_gravity_ = std::max(1e-6, pressure_gravity_);
        mag_cov_ = std::max(1e-6, mag_cov_);
    }

    void create_subscriptions(rclcpp::Node &node,
                              const rclcpp::CallbackGroup::SharedPtr &callback_group = nullptr)
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(200000));
        qos.best_effort();
        rclcpp::SubscriptionOptions options;
        options.callback_group = callback_group;

        if (dvl_enable_)
        {
            sub_dvl_ = node.create_subscription<DvlMsg>(
                dvl_topic_, qos, std::bind(&AuxiliarySensorFusion::dvl_callback, this, std::placeholders::_1),
                options);
        }
        if (pressure_enable_)
        {
            sub_pressure_ = node.create_subscription<PressureMsg>(
                pressure_topic_, qos, std::bind(&AuxiliarySensorFusion::pressure_callback, this, std::placeholders::_1),
                options);
        }
        if (mag_enable_)
        {
            sub_mag_ = node.create_subscription<MagMsg>(
                mag_topic_, qos, std::bind(&AuxiliarySensorFusion::mag_callback, this, std::placeholders::_1),
                options);
        }
    }

    // Time-ordered event fusion: walks all DVL/pressure/mag measurements in
    // [begin_time, end_time] in chronological order, calling propagate_to(t)
    // before each so the EKF state is at the measurement's own timestamp when
    // the residual and update are computed. This avoids the previous
    // scan-epoch-batched fusion where every aux measurement was evaluated at
    // the trailing sonar/state time, producing wrong velocity residuals
    // proportional to (state_time - measurement_time).
    template <typename PropagatorFn>
    UpdateSummary process_interval_interleaved(double begin_time,
                                               double end_time,
                                               const std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> &imu_msgs,
                                               Ekf &kf,
                                               PropagatorFn &&propagate_to)
    {
        UpdateSummary summary;

        std::vector<DvlMsg::ConstSharedPtr> dvl_msgs;
        std::vector<PressureMsg::ConstSharedPtr> pres_msgs;
        std::vector<MagMsg::ConstSharedPtr> mag_msgs;
        if (dvl_enable_)      dvl_msgs  = take_dvl_measurements(begin_time, end_time);
        if (pressure_enable_) pres_msgs = take_pressure_measurements(begin_time, end_time);
        if (mag_enable_)      mag_msgs  = take_mag_measurements(begin_time, end_time);

        // Pressure: apply ALL messages in [begin_time, end_time], each at its
        // own timestamp. Matches Codex's original timestamp-ordered fusion
        // (which the user wants restored). The earlier "messages.back() only"
        // logic in process_interval was a workaround for the wrong-time bug.

        enum Kind { K_DVL, K_PRES, K_MAG };
        struct Event {
            double t;
            Kind kind;
            std::size_t idx;
        };
        std::vector<Event> events;
        events.reserve(dvl_msgs.size() + pres_msgs.size() + mag_msgs.size());
        for (std::size_t i = 0; i < dvl_msgs.size(); ++i)
            events.push_back({get_time_sec(dvl_msgs[i]->header.stamp), K_DVL, i});
        for (std::size_t i = 0; i < pres_msgs.size(); ++i)
            events.push_back({get_time_sec(pres_msgs[i]->header.stamp), K_PRES, i});
        for (std::size_t i = 0; i < mag_msgs.size(); ++i)
            events.push_back({get_time_sec(mag_msgs[i]->header.stamp), K_MAG, i});
        std::stable_sort(events.begin(), events.end(),
                         [](const Event &a, const Event &b) { return a.t < b.t; });

        for (const auto &ev : events)
        {
            // Advance EKF state to the measurement's own timestamp.
            propagate_to(ev.t);

            // Body angular velocity at the measurement time. We pick the IMU
            // sample closest to ev.t; gives a closer omega than always using
            // the most recent IMU when t_aux is mid-interval.
            const V3D omega_body = body_omega_at_time(ev.t, imu_msgs, kf.get_x());

            switch (ev.kind)
            {
                case K_DVL:
                {
                    const auto &msg = *dvl_msgs[ev.idx];
                    const state_ikfom state = kf.get_x();
                    const V3D measurement  = dvl_measurement(msg);
                    const V3D prediction   = dvl_prediction(state, omega_body);
                    const V3D body_velocity = dvl_sensor_origin_velocity_body(state, omega_body);
                    const V3D residual = measurement - prediction;
                    summary.dvl_count++;
                    summary.dvl_res_norm_sum += residual.norm();
                    summary.dvl_res_norm_max = std::max(summary.dvl_res_norm_max, residual.norm());
                    summary.dvl_res_sum += residual;
                    summary.dvl_res_abs_max = summary.dvl_res_abs_max.cwiseMax(residual.cwiseAbs());
                    summary.dvl_meas_sum += measurement;
                    summary.dvl_pred_sum += prediction;
                    summary.dvl_body_vel_sum += body_velocity;
                    const bool accepted = apply_dvl_update(msg, omega_body, kf);
                    summary.dvl_accepted += accepted ? 1 : 0;
                    summary.dvl_rejected += accepted ? 0 : 1;
                    summary.dvl_updated = accepted || summary.dvl_updated;
                    break;
                }
                case K_PRES:
                {
                    const auto &msg = *pres_msgs[ev.idx];
                    const double residual_pa = pressure_residual(msg, kf.get_x());
                    const double residual_depth = residual_pa / pressure_scale();
                    summary.pressure_count++;
                    summary.pressure_res_depth_sum += std::abs(residual_depth);
                    summary.pressure_res_depth_max = std::max(summary.pressure_res_depth_max,
                                                              std::abs(residual_depth));
                    const bool accepted = apply_pressure_update(msg, kf);
                    summary.pressure_accepted += accepted ? 1 : 0;
                    summary.pressure_rejected += accepted ? 0 : 1;
                    summary.pressure_updated = accepted || summary.pressure_updated;
                    break;
                }
                case K_MAG:
                {
                    const auto &msg = *mag_msgs[ev.idx];
                    const state_ikfom state = kf.get_x();
                    const V3D residual = mag_reference_ready_ ? mag_residual(msg, state) : V3D::Zero();
                    summary.mag_count++;
                    summary.mag_res_norm_sum += residual.norm();
                    summary.mag_res_norm_max = std::max(summary.mag_res_norm_max, residual.norm());
                    const bool accepted = apply_mag_update(msg, kf);
                    summary.mag_accepted += accepted ? 1 : 0;
                    summary.mag_rejected += accepted ? 0 : 1;
                    summary.mag_updated = accepted || summary.mag_updated;
                    break;
                }
            }
        }

        return summary;
    }

    UpdateSummary process_interval(double begin_time,
                                   double end_time,
                                   const std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> &imu_msgs,
                                   Ekf &kf)
    {
        UpdateSummary summary;
        const V3D omega_body = latest_body_omega(imu_msgs, kf.get_x());

        if (dvl_enable_)
        {
            const auto messages = take_dvl_measurements(begin_time, end_time);
            for (const auto &msg : messages)
            {
                const state_ikfom state = kf.get_x();
                const V3D measurement = dvl_measurement(*msg);
                const V3D prediction = dvl_prediction(state, omega_body);
                const V3D body_velocity = dvl_sensor_origin_velocity_body(state, omega_body);
                const V3D residual = measurement - prediction;
                summary.dvl_count++;
                summary.dvl_res_norm_sum += residual.norm();
                summary.dvl_res_norm_max = std::max(summary.dvl_res_norm_max, residual.norm());
                summary.dvl_res_sum += residual;
                summary.dvl_res_abs_max = summary.dvl_res_abs_max.cwiseMax(residual.cwiseAbs());
                summary.dvl_meas_sum += measurement;
                summary.dvl_pred_sum += prediction;
                summary.dvl_body_vel_sum += body_velocity;
                const bool accepted = apply_dvl_update(*msg, omega_body, kf);
                summary.dvl_accepted += accepted ? 1 : 0;
                summary.dvl_rejected += accepted ? 0 : 1;
                summary.dvl_updated = accepted || summary.dvl_updated;
            }
        }

        if (pressure_enable_)
        {
            const auto messages = take_pressure_measurements(begin_time, end_time);
            if (!messages.empty())
            {
                const auto &msg = messages.back();
                const double residual_pa = pressure_residual(*msg, kf.get_x());
                const double residual_depth = residual_pa / pressure_scale();
                summary.pressure_count = 1;
                summary.pressure_res_depth_sum = std::abs(residual_depth);
                summary.pressure_res_depth_max = std::abs(residual_depth);
                summary.pressure_updated = apply_pressure_update(*msg, kf);
                summary.pressure_accepted = summary.pressure_updated ? 1 : 0;
                summary.pressure_rejected = summary.pressure_updated ? 0 : 1;
            }
        }

        if (mag_enable_)
        {
            const auto messages = take_mag_measurements(begin_time, end_time);
            for (const auto &msg : messages)
            {
                const state_ikfom state = kf.get_x();
                const V3D residual = mag_reference_ready_ ? mag_residual(*msg, state) : V3D::Zero();
                summary.mag_count++;
                summary.mag_res_norm_sum += residual.norm();
                summary.mag_res_norm_max = std::max(summary.mag_res_norm_max, residual.norm());
                const bool accepted = apply_mag_update(*msg, kf);
                summary.mag_accepted += accepted ? 1 : 0;
                summary.mag_rejected += accepted ? 0 : 1;
                summary.mag_updated = accepted || summary.mag_updated;
            }
        }

        return summary;
    }

    void warn_timeouts(rclcpp::Node &node, double end_time) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (dvl_enable_ && last_timestamp_dvl_ > 0.0 && end_time - last_timestamp_dvl_ > dvl_timeout_)
        {
            RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 5000,
                                 "No DVL message for %.2f s (timeout %.2f s).",
                                 end_time - last_timestamp_dvl_, dvl_timeout_);
        }
        if (pressure_enable_ && last_timestamp_pressure_ > 0.0 && end_time - last_timestamp_pressure_ > pressure_timeout_)
        {
            RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 5000,
                                 "No pressure message for %.2f s (timeout %.2f s).",
                                 end_time - last_timestamp_pressure_, pressure_timeout_);
        }
        if (mag_enable_ && last_timestamp_mag_ > 0.0 && end_time - last_timestamp_mag_ > mag_timeout_)
        {
            RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 5000,
                                 "No magnetometer message for %.2f s (timeout %.2f s).",
                                 end_time - last_timestamp_mag_, mag_timeout_);
        }
    }

    bool dvl_enabled() const { return dvl_enable_; }
    bool pressure_enabled() const { return pressure_enable_; }
    bool mag_enabled() const { return mag_enable_; }
    const std::string &dvl_topic() const { return dvl_topic_; }
    const std::string &pressure_topic() const { return pressure_topic_; }
    const std::string &mag_topic() const { return mag_topic_; }
    double dvl_timeout() const { return dvl_timeout_; }
    double pressure_timeout() const { return pressure_timeout_; }
    double mag_timeout() const { return mag_timeout_; }
    const V3D &dvl_T() const { return dvl_T_; }
    const V3D &pressure_T() const { return pressure_T_; }
    double dvl_velocity_cov() const { return dvl_velocity_cov_; }
    double dvl_innovation_gate_sigma() const { return dvl_innovation_gate_sigma_; }
    double dvl_b_init_cov() const { return dvl_b_init_cov_; }
    double pressure_b_init_cov() const { return pressure_b_init_cov_; }
    double mag_b_init_cov() const { return mag_b_init_cov_; }
    double mag_b_proc_cov() const { return mag_b_proc_cov_; }

    // Set the camera_init frame's z-coordinate in the world frame so the pressure
    // depth model works correctly when world_to_camera_init_T.z != 0.
    // Call this after loading world_to_camera_init_T from config.
    void set_camera_init_z_in_world(double z) { camera_init_z_in_world_ = z; }

    const V3D &earth_field_world() const { return mag_earth_field_world_; }

private:
    static M3D skew(const V3D &v)
    {
        M3D out;
        out << 0.0, -v.z(), v.y(),
               v.z(), 0.0, -v.x(),
               -v.y(), v.x(), 0.0;
        return out;
    }

    static bool finite_vector(const Eigen::VectorXd &v)
    {
        return v.allFinite();
    }

    double pressure_scale() const
    {
        return pressure_fluid_density_ * pressure_gravity_;
    }

    // Use the message covariance when valid, but never below the configured fallback.
    // This prevents unrealistically tight bag-recorded covariances from overwhelming the EKF.
    double covariance_or_fallback(double value, double fallback) const
    {
        if (!std::isfinite(value) || value <= 1e-12)
        {
            return fallback;
        }
        return std::max(value, fallback);
    }

    V3D latest_body_omega(const std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> &imu_msgs,
                          const state_ikfom &state) const
    {
        if (imu_msgs.empty())
        {
            return V3D::Zero();
        }
        const auto &gyro = imu_msgs.back()->angular_velocity;
        return V3D(gyro.x, gyro.y, gyro.z) - V3D(state.bg[0], state.bg[1], state.bg[2]);
    }

    // Pick the IMU sample whose stamp is closest to target_t and return the
    // body-frame angular velocity it implies (raw gyro minus current bg).
    V3D body_omega_at_time(double target_t,
                           const std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> &imu_msgs,
                           const state_ikfom &state) const
    {
        if (imu_msgs.empty()) return V3D::Zero();
        auto best = imu_msgs.begin();
        double best_diff = std::abs(get_time_sec((*best)->header.stamp) - target_t);
        for (auto it = imu_msgs.begin(); it != imu_msgs.end(); ++it)
        {
            const double d = std::abs(get_time_sec((*it)->header.stamp) - target_t);
            if (d < best_diff) { best_diff = d; best = it; }
        }
        const auto &gyro = (*best)->angular_velocity;
        return V3D(gyro.x, gyro.y, gyro.z) - V3D(state.bg[0], state.bg[1], state.bg[2]);
    }

    V3D dvl_measurement(const DvlMsg &msg) const
    {
        const auto &linear = msg.twist.twist.linear;
        return V3D(linear.x, linear.y, linear.z);
    }

    V3D dvl_sensor_origin_velocity_body(const state_ikfom &state, const V3D &omega_body) const
    {
        const M3D R_wb = state.rot.toRotationMatrix();
        const V3D body_velocity = R_wb.transpose() * V3D(state.vel[0], state.vel[1], state.vel[2]);
        return body_velocity + omega_body.cross(dvl_T_);
    }

    V3D dvl_prediction(const state_ikfom &state, const V3D &omega_body) const
    {
        return dvl_R_.transpose() * dvl_sensor_origin_velocity_body(state, omega_body) +
               V3D(state.b_dvl[0], state.b_dvl[1], state.b_dvl[2]);
    }

    V3D dvl_residual(const DvlMsg &msg, const state_ikfom &state, const V3D &omega_body) const
    {
        return dvl_measurement(msg) - dvl_prediction(state, omega_body);
    }

    double pressure_raw_depth(const state_ikfom &state) const
    {
        // state.pos is in camera_init coordinates.
        // pressure_surface_z_ is the water-surface z in WORLD coordinates (usually 0.0).
        // The surface in camera_init coordinates is: surface_z_world - camera_init_z_in_world_.
        // This correctly handles world_to_camera_init_T.z != 0 (e.g., robot starts underwater).
        const V3D sensor_ci = V3D(state.pos[0], state.pos[1], state.pos[2]) +
                              state.rot.toRotationMatrix() * pressure_T_;
        const double surface_z_ci = pressure_surface_z_ - camera_init_z_in_world_;
        return surface_z_ci - sensor_ci.z();
    }

    double pressure_prediction(const state_ikfom &state) const
    {
        const double depth = std::max(0.0, pressure_raw_depth(state));
        return pressure_surface_pressure_ + pressure_scale() * depth + state.b_pressure[0];
    }

    double pressure_residual(const PressureMsg &msg, const state_ikfom &state) const
    {
        return msg.fluid_pressure - pressure_prediction(state);
    }

    bool apply_dvl_update(const DvlMsg &msg, const V3D &omega_body, Ekf &kf)
    {
        const state_ikfom state = kf.get_x();
        const V3D residual = dvl_residual(msg, state, omega_body);

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, state_ikfom::DOF);

        const M3D R_wb = state.rot.toRotationMatrix();
        const V3D vel_body = R_wb.transpose() * V3D(state.vel[0], state.vel[1], state.vel[2]);
        H.block<3, 3>(0, 12) = dvl_R_.transpose() * R_wb.transpose();
        H.block<3, 3>(0, 3) = dvl_R_.transpose() * skew(vel_body);
        H.block<3, 3>(0, 15) = dvl_R_.transpose() * skew(dvl_T_);
        H.block<3, 3>(0, 23) = M3D::Identity();

        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(3, 3);
        R(0, 0) = covariance_or_fallback(msg.twist.covariance[0], dvl_velocity_cov_);
        R(1, 1) = covariance_or_fallback(msg.twist.covariance[7], dvl_velocity_cov_);
        R(2, 2) = covariance_or_fallback(msg.twist.covariance[14], dvl_velocity_cov_);

        if (dvl_innovation_gate_sigma_ > 0.0)
        {
            for (int i = 0; i < 3; ++i)
            {
                if (std::abs(residual[i]) > dvl_innovation_gate_sigma_ * std::sqrt(R(i, i)))
                {
                    return false;
                }
            }
        }

        return apply_linear_update(residual, H, R, kf);
    }

    bool finalize_pressure_reference_if_needed(const state_ikfom &state)
    {
        if (pressure_ref_finalized_)
        {
            return true;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (pressure_ref_finalized_)
        {
            return true;
        }
        if (!pressure_samples_ready_ || pressure_init_samples_collected_ <= 0)
        {
            return false;
        }

        const double mean_pressure =
            pressure_init_sum_ / static_cast<double>(pressure_init_samples_collected_);
        const double initial_depth = std::max(0.0, pressure_raw_depth(state));
        pressure_surface_pressure_ =
            mean_pressure - pressure_scale() * initial_depth - state.b_pressure[0];
        pressure_ref_finalized_ = true;
        return true;
    }

    bool apply_pressure_update(const PressureMsg &msg, Ekf &kf)
    {
        const state_ikfom state = kf.get_x();
        if (!finalize_pressure_reference_if_needed(state))
        {
            return false;
        }

        const double measured_depth = (msg.fluid_pressure - pressure_surface_pressure_) / pressure_scale();
        if (pressure_raw_depth(state) <= 0.0 && measured_depth <= 0.0)
        {
            return false;
        }

        Eigen::VectorXd residual(1);
        residual(0) = pressure_residual(msg, state);

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, state_ikfom::DOF);
        H(0, 2) = -pressure_scale();
        H.block<1, 3>(0, 3) = pressure_scale() *
                               (Eigen::RowVector3d::UnitZ() * state.rot.toRotationMatrix() * skew(pressure_T_));
        H(0, 26) = 1.0;

        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(1, 1);
        R(0, 0) = covariance_or_fallback(msg.variance, pressure_cov_);

        if (pressure_innovation_gate_sigma_ > 0.0)
        {
            const double sigma_pres = std::sqrt(R(0, 0));
            if (std::abs(residual(0)) > pressure_innovation_gate_sigma_ * sigma_pres)
                return false;
        }

        return apply_linear_update(residual, H, R, kf);
    }

    bool apply_linear_update(const Eigen::VectorXd &residual,
                             const Eigen::MatrixXd &H,
                             const Eigen::MatrixXd &R,
                             Ekf &kf)
    {
        if (residual.size() == 0 || H.rows() != residual.size() || H.cols() != state_ikfom::DOF ||
            R.rows() != residual.size() || R.cols() != residual.size() || !finite_vector(residual) ||
            !H.allFinite() || !R.allFinite())
        {
            return false;
        }

        typename Ekf::cov P = kf.get_P();
        Eigen::MatrixXd S = H * P * H.transpose() + R;
        Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
        if (ldlt.info() != Eigen::Success)
        {
            return false;
        }

        const Eigen::MatrixXd I_meas = Eigen::MatrixXd::Identity(residual.size(), residual.size());
        const Eigen::MatrixXd K = P * H.transpose() * ldlt.solve(I_meas);
        const Eigen::VectorXd dx_dyn = K * residual;
        if (!finite_vector(dx_dyn))
        {
            return false;
        }

        typename Ekf::vectorized_state dx = Ekf::vectorized_state::Zero();
        dx = dx_dyn;

        state_ikfom state = kf.get_x();
        state.boxplus(dx);

        const typename Ekf::cov I_state = Ekf::cov::Identity();
        typename Ekf::cov KH = K * H;
        typename Ekf::cov P_new = (I_state - KH) * P * (I_state - KH).transpose() + K * R * K.transpose();
        P_new = 0.5 * (P_new + P_new.transpose()).eval();

        kf.change_x(state);
        kf.change_P(P_new);
        return true;
    }

    std::vector<DvlMsg::ConstSharedPtr> take_dvl_measurements(double begin_time, double end_time)
    {
        std::vector<DvlMsg::ConstSharedPtr> messages;
        std::lock_guard<std::mutex> lock(mutex_);
        while (!dvl_buffer_.empty())
        {
            const double stamp = get_time_sec(dvl_buffer_.front()->header.stamp);
            if (stamp <= begin_time + 1e-6)
            {
                dvl_buffer_.pop_front();
                continue;
            }
            if (stamp > end_time + 1e-6)
            {
                break;
            }
            messages.push_back(dvl_buffer_.front());
            dvl_buffer_.pop_front();
        }
        return messages;
    }

    std::vector<PressureMsg::ConstSharedPtr> take_pressure_measurements(double begin_time, double end_time)
    {
        std::vector<PressureMsg::ConstSharedPtr> messages;
        std::lock_guard<std::mutex> lock(mutex_);
        while (!pressure_buffer_.empty())
        {
            const double stamp = get_time_sec(pressure_buffer_.front()->header.stamp);
            if (stamp <= begin_time + 1e-6)
            {
                pressure_buffer_.pop_front();
                continue;
            }
            if (stamp > end_time + 1e-6)
            {
                break;
            }
            messages.push_back(pressure_buffer_.front());
            pressure_buffer_.pop_front();
        }
        return messages;
    }

    void dvl_callback(const DvlMsg::ConstSharedPtr msg)
    {
        const double timestamp = get_time_sec(msg->header.stamp);
        std::lock_guard<std::mutex> lock(mutex_);
        if (timestamp < last_timestamp_dvl_)
        {
            dvl_buffer_.clear();
        }
        last_timestamp_dvl_ = timestamp;
        dvl_buffer_.push_back(msg);
    }

    void pressure_callback(const PressureMsg::ConstSharedPtr msg)
    {
        const double timestamp = get_time_sec(msg->header.stamp);
        std::lock_guard<std::mutex> lock(mutex_);
        if (timestamp < last_timestamp_pressure_)
        {
            pressure_buffer_.clear();
            pressure_init_samples_collected_ = 0;
            pressure_init_sum_ = 0.0;
            pressure_samples_ready_ = false;
            pressure_ref_finalized_ = false;
        }
        last_timestamp_pressure_ = timestamp;

        if (!pressure_samples_ready_)
        {
            pressure_init_sum_ += msg->fluid_pressure;
            pressure_init_samples_collected_++;
            if (pressure_init_samples_collected_ >= kPressureInitSamples)
            {
                pressure_samples_ready_ = true;
            }
            return;
        }

        pressure_buffer_.push_back(msg);
    }

    void mag_callback(const MagMsg::ConstSharedPtr msg)
    {
        const double timestamp = get_time_sec(msg->header.stamp);
        std::lock_guard<std::mutex> lock(mutex_);
        if (timestamp < last_timestamp_mag_)
        {
            mag_buffer_.clear();
            mag_init_sum_.setZero();
            mag_init_count_ = 0;
            mag_samples_ready_ = false;
            mag_reference_ready_ = false;
        }
        last_timestamp_mag_ = timestamp;

        if (!mag_samples_ready_)
        {
            mag_init_sum_ += mag_corrected(*msg);
            ++mag_init_count_;
            if (mag_init_count_ >= kMagInitSamples)
            {
                mag_samples_ready_ = true;
            }
            return;
        }

        mag_buffer_.push_back(msg);
    }

    // Apply offline hard-iron/soft-iron calibration, then the explicit sensor-to-body axis sign.
    V3D mag_corrected(const MagMsg &msg) const
    {
        const V3D raw(msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z);
        return mag_axis_sign_.cwiseProduct(mag_soft_iron_ * (raw - mag_hard_iron_));
    }

    // h(state) = R_ci_body^T * B_ci + b_mag
    V3D mag_prediction(const state_ikfom &state) const
    {
        return state.rot.toRotationMatrix().transpose() * mag_earth_field_world_ +
               V3D(state.b_mag[0], state.b_mag[1], state.b_mag[2]);
    }

    V3D mag_residual(const MagMsg &msg, const state_ikfom &state) const
    {
        return mag_corrected(msg) - mag_prediction(state);
    }

    bool finalize_mag_reference_if_needed(const state_ikfom &state)
    {
        if (mag_reference_ready_)
        {
            return true;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (mag_reference_ready_)
        {
            return true;
        }
        if (!mag_samples_ready_ || mag_init_count_ <= 0)
        {
            return false;
        }

        const V3D initial_body_field = mag_init_sum_ / static_cast<double>(mag_init_count_);
        mag_earth_field_world_ = state.rot.toRotationMatrix() * initial_body_field;
        mag_reference_ready_ = true;
        return true;
    }

    std::vector<MagMsg::ConstSharedPtr> take_mag_measurements(double begin_time, double end_time)
    {
        std::vector<MagMsg::ConstSharedPtr> messages;
        std::lock_guard<std::mutex> lock(mutex_);
        while (!mag_buffer_.empty())
        {
            const double stamp = get_time_sec(mag_buffer_.front()->header.stamp);
            if (stamp <= begin_time + 1e-6)
            {
                mag_buffer_.pop_front();
                continue;
            }
            if (stamp > end_time + 1e-6)
                break;
            messages.push_back(mag_buffer_.front());
            mag_buffer_.pop_front();
        }
        return messages;
    }

    bool apply_mag_update(const MagMsg &msg, Ekf &kf)
    {
        const state_ikfom state = kf.get_x();
        if (!finalize_mag_reference_if_needed(state))
        {
            return false;
        }

        const V3D residual = mag_residual(msg, state);

        // b_mag is at DOF 27-29
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, state_ikfom::DOF);
        const V3D B_body_pred = state.rot.toRotationMatrix().transpose() * mag_earth_field_world_;
        H.block<3, 3>(0, 3)  = skew(B_body_pred);  // ∂h/∂rot
        H.block<3, 3>(0, 27) = M3D::Identity();     // ∂h/∂b_mag

        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3) * mag_cov_;

        // Innovation covariance: use S = H*P*H^T + R for a proper gate.
        // This lets large residuals through when state uncertainty is large (e.g.,
        // during b_mag init) and tightens automatically as covariance collapses.
        if (mag_innovation_gate_sigma_ > 0.0)
        {
            const typename Ekf::cov &P_state = kf.get_P();
            const Eigen::MatrixXd S_innov = H * P_state * H.transpose() + R;
            for (int i = 0; i < 3; ++i)
            {
                const double innov_sigma = std::sqrt(S_innov(i, i));
                if (std::abs(residual[i]) > mag_innovation_gate_sigma_ * innov_sigma)
                    return false;
            }
        }

        return apply_linear_update(residual, H, R, kf);
    }

    bool dvl_enable_ = false;
    bool pressure_enable_ = false;
    bool mag_enable_ = false;
    std::string dvl_topic_ = "/auv/dvl";
    std::string pressure_topic_ = "/auv/pressure/scaled2";
    std::string mag_topic_ = "/auv/imu/magnetic_field";
    double dvl_timeout_ = 0.25;
    double pressure_timeout_ = 0.25;
    double mag_timeout_ = 0.5;
    double dvl_velocity_cov_ = 4e-4;
    double dvl_innovation_gate_sigma_ = 5.0;
    double dvl_b_init_cov_ = 1e-3;
    double pressure_cov_ = 1e4;
    double pressure_b_init_cov_ = 1e4;
    double pressure_innovation_gate_sigma_ = 0.0;
    double camera_init_z_in_world_ = 0.0;
    double pressure_fluid_density_ = 1025.0;
    double pressure_gravity_ = 9.80665;
    double pressure_surface_pressure_ = 101325.0;
    double pressure_surface_z_ = 0.0;
    static constexpr int kPressureInitSamples = 10;
    int    pressure_init_samples_collected_ = 0;
    double pressure_init_sum_               = 0.0;
    bool   pressure_samples_ready_          = false;
    bool   pressure_ref_finalized_          = false;
    double mag_cov_ = 1849.0;
    double mag_b_init_cov_ = 1e6;
    double mag_b_proc_cov_ = 0.001;
    double mag_innovation_gate_sigma_ = 3.0;
    V3D dvl_T_ = V3D::Zero();
    M3D dvl_R_ = M3D::Identity();
    V3D pressure_T_ = V3D::Zero();
    V3D mag_earth_field_world_ = V3D::Zero();  // initialized from first corrected mag samples
    V3D mag_hard_iron_ = V3D::Zero();
    M3D mag_soft_iron_ = M3D::Identity();
    V3D mag_axis_sign_ = V3D::Ones();
    static constexpr int kMagInitSamples = 10;
    V3D mag_init_sum_ = V3D::Zero();
    int mag_init_count_ = 0;
    bool mag_samples_ready_ = false;
    bool mag_reference_ready_ = false;

    mutable std::mutex mutex_;
    std::deque<DvlMsg::ConstSharedPtr> dvl_buffer_;
    std::deque<PressureMsg::ConstSharedPtr> pressure_buffer_;
    std::deque<MagMsg::ConstSharedPtr> mag_buffer_;
    double last_timestamp_dvl_ = -1.0;
    double last_timestamp_pressure_ = -1.0;
    double last_timestamp_mag_ = -1.0;
    rclcpp::Subscription<DvlMsg>::SharedPtr sub_dvl_;
    rclcpp::Subscription<PressureMsg>::SharedPtr sub_pressure_;
    rclcpp::Subscription<MagMsg>::SharedPtr sub_mag_;
};

#endif

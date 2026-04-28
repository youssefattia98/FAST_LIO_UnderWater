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

#include "common_lib.h"
#include "use-ikfom.hpp"

class AuxiliarySensorFusion
{
public:
    using Ekf = esekfom::esekf<state_ikfom, 12, input_ikfom>;
    using DvlMsg = geometry_msgs::msg::TwistWithCovarianceStamped;
    using PressureMsg = sensor_msgs::msg::FluidPressure;

    struct UpdateSummary
    {
        int dvl_count = 0;
        int pressure_count = 0;
        double dvl_res_norm_sum = 0.0;
        double dvl_res_norm_max = 0.0;
        double pressure_res_depth_sum = 0.0;
        double pressure_res_depth_max = 0.0;
        bool dvl_updated = false;
        bool pressure_updated = false;

        bool updated() const
        {
            return dvl_updated || pressure_updated;
        }

        double mean_dvl_residual() const
        {
            return dvl_count > 0 ? dvl_res_norm_sum / static_cast<double>(dvl_count) : 0.0;
        }

        double mean_pressure_depth_residual() const
        {
            return pressure_count > 0 ? pressure_res_depth_sum / static_cast<double>(pressure_count) : 0.0;
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
        node.declare_parameter<double>("pressure.pressure_cov", 1e4);
        node.declare_parameter<double>("pressure.fluid_density", 1025.0);
        node.declare_parameter<double>("pressure.gravity", 9.80665);
        node.declare_parameter<double>("pressure.surface_pressure", 101325.0);
        node.declare_parameter<double>("pressure.surface_z", 0.0);
    }

    void load_parameters(rclcpp::Node &node)
    {
        node.get_parameter_or<bool>("dvl.enable", dvl_enable_, false);
        node.get_parameter_or<std::string>("dvl.topic", dvl_topic_, "/auv/dvl");
        node.get_parameter_or<double>("dvl.dvl_timeout", dvl_timeout_, 0.25);
        node.get_parameter_or<double>("dvl.velocity_cov", dvl_velocity_cov_, 4e-4);

        node.get_parameter_or<bool>("pressure.enable", pressure_enable_, false);
        node.get_parameter_or<std::string>("pressure.topic", pressure_topic_, "/auv/pressure/scaled2");
        node.get_parameter_or<double>("pressure.pressure_timeout", pressure_timeout_, 0.25);
        node.get_parameter_or<double>("pressure.pressure_cov", pressure_cov_, 1e4);
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

        dvl_timeout_ = std::max(0.0, dvl_timeout_);
        pressure_timeout_ = std::max(0.0, pressure_timeout_);
        dvl_velocity_cov_ = std::max(1e-12, dvl_velocity_cov_);
        pressure_cov_ = std::max(1e-6, pressure_cov_);
        pressure_fluid_density_ = std::max(1e-6, pressure_fluid_density_);
        pressure_gravity_ = std::max(1e-6, pressure_gravity_);
    }

    void create_subscriptions(rclcpp::Node &node)
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(200000));
        qos.best_effort();

        if (dvl_enable_)
        {
            sub_dvl_ = node.create_subscription<DvlMsg>(
                dvl_topic_, qos, std::bind(&AuxiliarySensorFusion::dvl_callback, this, std::placeholders::_1));
        }
        if (pressure_enable_)
        {
            sub_pressure_ = node.create_subscription<PressureMsg>(
                pressure_topic_, qos, std::bind(&AuxiliarySensorFusion::pressure_callback, this, std::placeholders::_1));
        }
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
                const V3D residual = dvl_residual(*msg, kf.get_x(), omega_body);
                summary.dvl_count++;
                summary.dvl_res_norm_sum += residual.norm();
                summary.dvl_res_norm_max = std::max(summary.dvl_res_norm_max, residual.norm());
                summary.dvl_updated = apply_dvl_update(*msg, omega_body, kf) || summary.dvl_updated;
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
    }

    bool dvl_enabled() const { return dvl_enable_; }
    bool pressure_enabled() const { return pressure_enable_; }
    const std::string &dvl_topic() const { return dvl_topic_; }
    const std::string &pressure_topic() const { return pressure_topic_; }
    double dvl_timeout() const { return dvl_timeout_; }
    double pressure_timeout() const { return pressure_timeout_; }
    const V3D &dvl_T() const { return dvl_T_; }
    const V3D &pressure_T() const { return pressure_T_; }

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

    double covariance_or_fallback(double value, double fallback) const
    {
        return std::isfinite(value) && value > 1e-12 ? value : fallback;
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

    V3D dvl_prediction(const state_ikfom &state, const V3D &omega_body) const
    {
        const M3D R_wb = state.rot.toRotationMatrix();
        const V3D body_velocity = R_wb.transpose() * V3D(state.vel[0], state.vel[1], state.vel[2]);
        const V3D sensor_origin_velocity = body_velocity + omega_body.cross(dvl_T_);
        return dvl_R_.transpose() * sensor_origin_velocity + V3D(state.b_dvl[0], state.b_dvl[1], state.b_dvl[2]);
    }

    V3D dvl_residual(const DvlMsg &msg, const state_ikfom &state, const V3D &omega_body) const
    {
        const auto &linear = msg.twist.twist.linear;
        const V3D z(linear.x, linear.y, linear.z);
        return z - dvl_prediction(state, omega_body);
    }

    double pressure_raw_depth(const state_ikfom &state) const
    {
        const V3D sensor_world = V3D(state.pos[0], state.pos[1], state.pos[2]) +
                                 state.rot.toRotationMatrix() * pressure_T_;
        return pressure_surface_z_ - sensor_world.z();
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
        H.block<3, 3>(0, 12) = dvl_R_.transpose() * R_wb.transpose();
        H.block<3, 3>(0, 23) = M3D::Identity();

        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(3, 3);
        R(0, 0) = covariance_or_fallback(msg.twist.covariance[0], dvl_velocity_cov_);
        R(1, 1) = covariance_or_fallback(msg.twist.covariance[7], dvl_velocity_cov_);
        R(2, 2) = covariance_or_fallback(msg.twist.covariance[14], dvl_velocity_cov_);

        return apply_linear_update(residual, H, R, kf);
    }

    bool apply_pressure_update(const PressureMsg &msg, Ekf &kf)
    {
        const state_ikfom state = kf.get_x();
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
        }
        last_timestamp_pressure_ = timestamp;
        pressure_buffer_.push_back(msg);
    }

    bool dvl_enable_ = false;
    bool pressure_enable_ = false;
    std::string dvl_topic_ = "/auv/dvl";
    std::string pressure_topic_ = "/auv/pressure/scaled2";
    double dvl_timeout_ = 0.25;
    double pressure_timeout_ = 0.25;
    double dvl_velocity_cov_ = 4e-4;
    double pressure_cov_ = 1e4;
    double pressure_fluid_density_ = 1025.0;
    double pressure_gravity_ = 9.80665;
    double pressure_surface_pressure_ = 101325.0;
    double pressure_surface_z_ = 0.0;
    V3D dvl_T_ = V3D::Zero();
    M3D dvl_R_ = M3D::Identity();
    V3D pressure_T_ = V3D::Zero();

    mutable std::mutex mutex_;
    std::deque<DvlMsg::ConstSharedPtr> dvl_buffer_;
    std::deque<PressureMsg::ConstSharedPtr> pressure_buffer_;
    double last_timestamp_dvl_ = -1.0;
    double last_timestamp_pressure_ = -1.0;
    rclcpp::Subscription<DvlMsg>::SharedPtr sub_dvl_;
    rclcpp::Subscription<PressureMsg>::SharedPtr sub_pressure_;
};

#endif

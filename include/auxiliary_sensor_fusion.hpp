#ifndef AUXILIARY_SENSOR_FUSION_HPP
#define AUXILIARY_SENSOR_FUSION_HPP

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "common_lib.h"
#include "dvl_measurement_model.hpp"
#include "magnetometer_heading_model.hpp"
#include "sensor_parameter_utils.hpp"
#include "use-ikfom.hpp"

class AuxiliarySensorFusion
{
public:
    using Ekf = esekfom::esekf<state_ikfom, process_noise_ikfom::DOF, input_ikfom>;
    using DvlMsg = geometry_msgs::msg::TwistWithCovarianceStamped;
    using PressureMsg = sensor_msgs::msg::FluidPressure;
    using MagMsg = sensor_msgs::msg::MagneticField;

    enum class MeasurementKind
    {
        Dvl,
        Pressure,
        Magnetometer
    };

    struct TimedMeasurement
    {
        double timestamp = 0.0;
        MeasurementKind kind = MeasurementKind::Dvl;
        DvlMsg::ConstSharedPtr dvl;
        PressureMsg::ConstSharedPtr pressure;
        MagMsg::ConstSharedPtr magnetometer;
    };

    struct LateMeasurementCounts
    {
        std::size_t dvl = 0;
        std::size_t pressure = 0;
        std::size_t magnetometer = 0;

        std::size_t total() const
        {
            return dvl + pressure + magnetometer;
        }
    };

    void declare_parameters(rclcpp::Node &node);

    void load_parameters(rclcpp::Node &node);

    void create_subscriptions(rclcpp::Node &node,
                              const rclcpp::CallbackGroup::SharedPtr &callback_group = nullptr);

    std::vector<TimedMeasurement> take_timed_measurements(double begin_time,
                                                          double end_time);

    bool apply_timed_measurement(
        const TimedMeasurement &measurement,
        const std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> &imu_msgs,
        Ekf &kf);

    LateMeasurementCounts take_late_measurement_counts();

    void warn_timeouts(rclcpp::Node &node, double end_time) const;

    bool dvl_enabled() const;
    bool pressure_enabled() const;
    bool mag_enabled() const;
    double dvl_b_init_cov() const;
    double pressure_b_init_cov() const;

    // Pressure measures depth along the world vertical axis, while the FAST-LIO
    // state is expressed in camera_init. Store the static camera_init pose so the
    // pressure model can project local sensor position onto world z.
    void set_camera_init_pose_in_world(const V3D &t, const M3D &R);

    void initialize_pressure_reference_pose(const state_ikfom &state);

private:
    struct ImuAngularSample
    {
        V3D raw_gyro = V3D::Zero();
        M3D covariance = M3D::Zero();
        bool covariance_valid = false;
        bool valid = false;
    };

    struct DvlObservation
    {
        V3D measurement = V3D::Zero();
        M3D covariance = M3D::Identity();
        ImuAngularSample imu;
        bool valid = false;
    };

    struct DvlLinearization
    {
        V3D measurement = V3D::Zero();
        V3D prediction = V3D::Zero();
        V3D residual = V3D::Zero();
        Eigen::Matrix<double, 3, state_ikfom::DOF> H =
            Eigen::Matrix<double, 3, state_ikfom::DOF>::Zero();
        M3D R = M3D::Identity();
        bool valid = false;
    };

    static bool finite_vector(const Eigen::VectorXd &v);

    double pressure_scale(const state_ikfom &state) const;

    // Use the message covariance when valid, but never below the configured fallback.
    // This prevents unrealistically tight bag-recorded covariances from overwhelming the EKF.
    double covariance_or_fallback(double value, double fallback) const;

    ImuAngularSample imu_angular_sample(const sensor_msgs::msg::Imu &msg) const;

    // Pick the raw IMU sample whose stamp is closest to the DVL timestamp.
    ImuAngularSample imu_angular_sample_at_time(
        double target_t,
        const std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> &imu_msgs) const;

    V3D dvl_measurement(const DvlMsg &msg) const;

    M3D dvl_measurement_covariance(const DvlMsg &msg) const;

    DvlObservation make_dvl_observation(const DvlMsg &msg,
                                        const ImuAngularSample &imu_sample,
                                        const state_ikfom &state) const;

    DvlLinearization build_dvl_linearization(const DvlObservation &observation,
                                             const state_ikfom &state) const;

    bool dvl_measurement_is_valid(const DvlLinearization &dvl) const;

    Eigen::RowVector3d world_z_axis_in_camera_init() const;

    double pressure_sensor_z_world(const state_ikfom &state) const;

    double pressure_prediction(const state_ikfom &state) const;

    double pressure_residual(const PressureMsg &msg, const state_ikfom &state) const;

    bool pressure_reference_ready_or_collect(const PressureMsg &msg,
                                             const state_ikfom &state);

    bool apply_dvl_update(const DvlMsg &msg,
                          const ImuAngularSample &imu_sample,
                          Ekf &kf);

    bool apply_pressure_update(const PressureMsg &msg, Ekf &kf);

    bool apply_pressure_depth_update(const Eigen::VectorXd &residual,
                                     const Eigen::MatrixXd &H,
                                     const Eigen::MatrixXd &R,
                                     Ekf &kf);

    typename Ekf::cov transport_covariance_after_boxplus(
        const typename Ekf::cov &P,
        state_ikfom prior_state,
        state_ikfom updated_state,
        const typename Ekf::vectorized_state &dx) const;

    bool apply_linear_update(const Eigen::VectorXd &residual,
                             const Eigen::MatrixXd &H,
                             const Eigen::MatrixXd &R,
                             Ekf &kf);

    std::vector<DvlMsg::ConstSharedPtr> take_dvl_measurements(double begin_time, double end_time);

    std::vector<PressureMsg::ConstSharedPtr> take_pressure_measurements(double begin_time, double end_time);

    void dvl_callback(const DvlMsg::ConstSharedPtr msg);

    void pressure_callback(const PressureMsg::ConstSharedPtr msg);

    void mag_callback(const MagMsg::ConstSharedPtr msg);

    // R_BM rotates the calibrated magnetometer-frame field into the IMU/body frame.
    V3D mag_corrected(const MagMsg &msg) const;

    M3D mag_calibrated_covariance() const;

    void reset_mag_reference_locked();

    bool mag_reference_is_ready() const;

    bool collect_mag_reference_sample(const state_ikfom &state,
                                      const V3D &magnetic_body);

    void accumulate_corrected_mag_reference_sample(
        const state_ikfom &updated_state,
        const V3D &magnetic_body);

    std::vector<MagMsg::ConstSharedPtr> take_mag_measurements(double begin_time, double end_time);

    bool apply_mag_update(const MagMsg &msg, Ekf &kf);

    bool dvl_enable_ = false;
    bool pressure_enable_ = false;
    bool mag_enable_ = false;
    std::string dvl_topic_ = "/auv/dvl";
    std::string pressure_topic_ = "/auv/pressure/scaled2";
    std::string mag_topic_ = "/auv/imu/magnetic_field";
    double dvl_frequency_ = 15.0;
    double dvl_timeout_ = 0.25;
    double pressure_timeout_ = 0.25;
    double mag_timeout_ = 0.5;
    double dvl_velocity_cov_ = 4e-4;
    double dvl_b_init_cov_ = 1e-8;
    double pressure_cov_ = 1e4;
    double pressure_b_init_cov_ = 1e4;
    V3D camera_init_T_in_world_ = V3D::Zero();
    M3D camera_init_R_in_world_ = M3D::Identity();
    double pressure_fluid_density_ = 1025.0;
    double pressure_reference_pa_ = 0.0;
    double pressure_reference_sensor_z_world_ = 0.0;
    static constexpr int kPressureReferenceSamples = 20;
    double pressure_init_sum_ = 0.0;
    int pressure_init_samples_collected_ = 0;
    bool pressure_ref_finalized_ = false;
    bool pressure_reference_pose_ready_ = false;
    double last_pressure_raw_ = 0.0;
    bool last_pressure_raw_valid_ = false;
    double mag_cov_ = 1849.0;
    double mag_heading_cov_floor_ = 1e-6;
    static constexpr int kMagReferenceSamples = 20;
    V3D mag_reference_sum_local_ = V3D::Zero();
    std::vector<V3D> mag_reference_samples_local_;
    int mag_reference_sample_count_ = 0;
    bool mag_reference_ready_ = false;
    V3D last_mag_raw_ = V3D::Zero();
    bool last_mag_raw_valid_ = false;
    // Fixed when reference collection starts; magnetometer updates rotate only about this axis.
    V3D mag_vertical_local_ = V3D::UnitZ();
    V3D mag_reference_local_ = V3D::Zero();
    V3D mag_horizontal_reference_local_ = V3D::Zero();
    double mag_reference_heading_variance_ = 0.0;
    V3D dvl_T_ = V3D::Zero();
    M3D dvl_R_ = M3D::Identity();
    V3D pressure_T_ = V3D::Zero();
    M3D mag_R_BM_ = M3D::Identity();
    V3D mag_hard_iron_ = V3D::Zero();
    M3D mag_soft_iron_ = M3D::Identity();

    mutable std::mutex mutex_;
    std::deque<DvlMsg::ConstSharedPtr> dvl_buffer_;
    std::deque<PressureMsg::ConstSharedPtr> pressure_buffer_;
    std::deque<MagMsg::ConstSharedPtr> mag_buffer_;
    LateMeasurementCounts late_measurement_counts_;
    double last_timestamp_dvl_ = -1.0;
    double last_timestamp_pressure_ = -1.0;
    double last_timestamp_mag_ = -1.0;
    rclcpp::Subscription<DvlMsg>::SharedPtr sub_dvl_;
    rclcpp::Subscription<PressureMsg>::SharedPtr sub_pressure_;
    rclcpp::Subscription<MagMsg>::SharedPtr sub_mag_;
};

#endif

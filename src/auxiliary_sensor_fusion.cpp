#include "auxiliary_sensor_fusion.hpp"

void AuxiliarySensorFusion::declare_parameters(rclcpp::Node &node)
{
    node.declare_parameter<bool>("dvl.enable", false);
    node.declare_parameter<std::string>("dvl.topic", "/auv/dvl");
    node.declare_parameter<std::vector<double>>("dvl.translation", {-0.079, -0.09691, -0.25938});
    node.declare_parameter<std::vector<double>>("dvl.rotation", {0.0, 0.0, 0.0});
    node.declare_parameter<double>("dvl.frequency", 15.0);

    node.declare_parameter<bool>("pressure.enable", false);
    node.declare_parameter<std::string>("pressure.topic", "/auv/pressure/scaled2");
    node.declare_parameter<std::vector<double>>("pressure.translation", {-0.24219, -0.03954, 0.01898});
    node.declare_parameter<double>("pressure.frequency", 4.0);

    node.declare_parameter<double>("dvl.covariance", 4e-4);
    node.declare_parameter<double>("dvl.init_covariance", 1e-8);
    node.declare_parameter<double>("pressure.covariance", 1e4);
    node.declare_parameter<double>("pressure.init_covariance", 1e4);
    node.declare_parameter<double>("pressure.fluid_density", 1025.0);

    node.declare_parameter<bool>("magnetometer.enable", false);
    node.declare_parameter<std::string>("magnetometer.topic", "/auv/imu/magnetic_field");
    node.declare_parameter<std::vector<double>>("magnetometer.rotation", {0.0, 0.0, 0.0});
    node.declare_parameter<std::vector<double>>("magnetometer.hard_iron_offset", {0.0, 0.0, 0.0});
    node.declare_parameter<std::vector<double>>("magnetometer.soft_iron_matrix",
        {1., 0., 0.,  0., 1., 0.,  0., 0., 1.});
    node.declare_parameter<double>("magnetometer.covariance", 1849.0);
    node.declare_parameter<double>("magnetometer.timeout", 0.5);
}

void AuxiliarySensorFusion::load_parameters(rclcpp::Node &node)
{
    node.get_parameter_or<bool>("dvl.enable", dvl_enable_, false);
    node.get_parameter_or<std::string>("dvl.topic", dvl_topic_, "/auv/dvl");
    node.get_parameter_or<double>("dvl.frequency", dvl_frequency_, 15.0);
    node.get_parameter_or<double>("dvl.covariance", dvl_velocity_cov_, 4e-4);
    node.get_parameter_or<double>("dvl.init_covariance", dvl_b_init_cov_, 1e-8);

    node.get_parameter_or<bool>("pressure.enable", pressure_enable_, false);
    node.get_parameter_or<std::string>("pressure.topic", pressure_topic_, "/auv/pressure/scaled2");
    double pressure_frequency;
    node.get_parameter_or<double>("pressure.frequency", pressure_frequency, 4.0);
    if (!std::isfinite(pressure_frequency) || pressure_frequency <= 0.0)
        RCLCPP_WARN(node.get_logger(), "pressure.frequency must be positive. Using 4 Hz.");
    pressure_timeout_ = uwfl2::timeout_from_frequency(pressure_frequency, 4.0);
    node.get_parameter_or<double>("pressure.covariance", pressure_cov_, 1e4);
    node.get_parameter_or<double>("pressure.init_covariance", pressure_b_init_cov_, 1e4);
    node.get_parameter_or<double>("pressure.fluid_density", pressure_fluid_density_, 1025.0);

    std::vector<double> dvl_T;
    std::vector<double> dvl_R;
    std::vector<double> pressure_T;
    node.get_parameter_or<std::vector<double>>("dvl.translation", dvl_T, {-0.079, -0.09691, -0.25938});
    node.get_parameter_or<std::vector<double>>("dvl.rotation", dvl_R, {0.0, 0.0, 0.0});
    node.get_parameter_or<std::vector<double>>("pressure.translation", pressure_T, {-0.24219, -0.03954, 0.01898});

    if (dvl_T.size() == 3)
    {
        dvl_T_ << dvl_T[0], dvl_T[1], dvl_T[2];
    }
    else
    {
        RCLCPP_WARN(node.get_logger(), "dvl.translation must have 3 values. Using zero translation.");
        dvl_T_.setZero();
    }

    dvl_R_ = uwfl2::rotation_from_rpy_degrees(dvl_R);

    if (pressure_T.size() == 3)
    {
        pressure_T_ << pressure_T[0], pressure_T[1], pressure_T[2];
    }
    else
    {
        RCLCPP_WARN(node.get_logger(), "pressure.translation must have 3 values. Using zero translation.");
        pressure_T_.setZero();
    }

    node.get_parameter_or<bool>("magnetometer.enable", mag_enable_, false);
    node.get_parameter_or<std::string>("magnetometer.topic", mag_topic_, "/auv/imu/magnetic_field");
    node.get_parameter_or<double>("magnetometer.covariance", mag_cov_, 1849.0);
    node.get_parameter_or<double>("magnetometer.timeout", mag_timeout_, 0.5);

    std::vector<double> mag_rotation, hard_iron, soft_iron;
    node.get_parameter_or<std::vector<double>>("magnetometer.rotation", mag_rotation, {0.0, 0.0, 0.0});
    node.get_parameter_or<std::vector<double>>("magnetometer.hard_iron_offset", hard_iron, {0., 0., 0.});
    node.get_parameter_or<std::vector<double>>("magnetometer.soft_iron_matrix", soft_iron,
        {1., 0., 0.,  0., 1., 0.,  0., 0., 1.});

    mag_R_BM_ = uwfl2::rotation_from_rpy_degrees(mag_rotation);

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

    if (!std::isfinite(dvl_frequency_) || dvl_frequency_ <= 0.0)
    {
        RCLCPP_WARN(node.get_logger(),
                    "dvl.frequency must be positive. Falling back to 15 Hz.");
        dvl_frequency_ = 15.0;
    }
    dvl_timeout_ = uwfl2::timeout_from_frequency(dvl_frequency_, 15.0);
    pressure_timeout_ = std::max(0.0, pressure_timeout_);
    mag_timeout_ = std::max(0.0, mag_timeout_);
    dvl_velocity_cov_ = std::max(1e-12, dvl_velocity_cov_);
    dvl_b_init_cov_ = std::max(1e-12, dvl_b_init_cov_);
    pressure_cov_ = std::max(1e-6, pressure_cov_);
    pressure_b_init_cov_ = std::max(1e-12, pressure_b_init_cov_);
    pressure_fluid_density_ = std::max(1e-6, pressure_fluid_density_);
    // Magnetometer covariance may be expressed in Tesla^2 in simulation
    // (around 1e-15) or in uT^2 for real bags. Keep only a numerical floor
    // here; a 1e-6 floor silently disables Tesla-scale magnetometer fusion.
    mag_cov_ = std::max(1e-18, mag_cov_);
}

void AuxiliarySensorFusion::create_subscriptions(rclcpp::Node &node,
                          const rclcpp::CallbackGroup::SharedPtr &callback_group)
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

std::vector<AuxiliarySensorFusion::TimedMeasurement> AuxiliarySensorFusion::take_timed_measurements(double begin_time,
                                                      double end_time)
{
    std::vector<TimedMeasurement> measurements;
    if (dvl_enable_)
    {
        const auto messages = take_dvl_measurements(begin_time, end_time);
        measurements.reserve(measurements.size() + messages.size());
        for (const auto &msg : messages)
        {
            measurements.push_back(
                {get_time_sec(msg->header.stamp), MeasurementKind::Dvl, msg, nullptr, nullptr});
        }
    }
    if (pressure_enable_)
    {
        const auto messages = take_pressure_measurements(begin_time, end_time);
        measurements.reserve(measurements.size() + messages.size());
        for (const auto &msg : messages)
        {
            measurements.push_back(
                {get_time_sec(msg->header.stamp), MeasurementKind::Pressure, nullptr, msg, nullptr});
        }
    }
    if (mag_enable_)
    {
        const auto messages = take_mag_measurements(begin_time, end_time);
        measurements.reserve(measurements.size() + messages.size());
        for (const auto &msg : messages)
        {
            measurements.push_back(
                {get_time_sec(msg->header.stamp), MeasurementKind::Magnetometer, nullptr, nullptr, msg});
        }
    }

    std::stable_sort(measurements.begin(), measurements.end(),
                     [](const TimedMeasurement &a, const TimedMeasurement &b) {
                         return a.timestamp < b.timestamp;
                     });
    return measurements;
}

bool AuxiliarySensorFusion::apply_timed_measurement(
    const TimedMeasurement &measurement,
    const std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> &imu_msgs,
    Ekf &kf)
{
    const ImuAngularSample imu_sample =
        imu_angular_sample_at_time(measurement.timestamp, imu_msgs);

    switch (measurement.kind)
    {
        case MeasurementKind::Dvl:
            return measurement.dvl &&
                   apply_dvl_update(*measurement.dvl, imu_sample, kf);
        case MeasurementKind::Pressure:
            return measurement.pressure &&
                   apply_pressure_update(*measurement.pressure, kf);
        case MeasurementKind::Magnetometer:
            return measurement.magnetometer &&
                   apply_mag_update(*measurement.magnetometer, kf);
    }
    return false;
}

AuxiliarySensorFusion::LateMeasurementCounts AuxiliarySensorFusion::take_late_measurement_counts()
{
    std::lock_guard<std::mutex> lock(mutex_);
    const LateMeasurementCounts counts = late_measurement_counts_;
    late_measurement_counts_ = {};
    return counts;
}

AuxiliarySensorFusion::Availability AuxiliarySensorFusion::availability() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return {last_timestamp_dvl_, last_timestamp_pressure_, last_timestamp_mag_,
            dvl_timeout_, pressure_timeout_, mag_timeout_};
}

void AuxiliarySensorFusion::warn_timeouts(rclcpp::Node &node, double end_time) const
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

bool AuxiliarySensorFusion::dvl_enabled() const
{ return dvl_enable_; }

bool AuxiliarySensorFusion::pressure_enabled() const
{ return pressure_enable_; }

bool AuxiliarySensorFusion::mag_enabled() const
{ return mag_enable_; }

double AuxiliarySensorFusion::dvl_b_init_cov() const
{ return dvl_b_init_cov_; }

double AuxiliarySensorFusion::pressure_b_init_cov() const
{ return pressure_b_init_cov_; }

void AuxiliarySensorFusion::set_camera_init_pose_in_world(const V3D &t, const M3D &R)
{
    camera_init_T_in_world_ = t;
    camera_init_R_in_world_ = R;
}

void AuxiliarySensorFusion::initialize_pressure_reference_pose(const state_ikfom &state)
{
    if (!pressure_enable_)
    {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pressure_reference_pose_ready_)
    {
        pressure_reference_sensor_z_world_ = pressure_sensor_z_world(state);
        pressure_reference_pose_ready_ = true;
    }
}

bool AuxiliarySensorFusion::finite_vector(const Eigen::VectorXd &v)
{
    return v.allFinite();
}

double AuxiliarySensorFusion::pressure_scale(const state_ikfom &state) const
{
    const V3D gravity(state.grav[0], state.grav[1], state.grav[2]);
    return pressure_fluid_density_ * std::max(1e-6, gravity.norm());
}

double AuxiliarySensorFusion::covariance_or_fallback(double value, double fallback) const
{
    if (!std::isfinite(value) || value <= 1e-12)
    {
        return fallback;
    }
    return std::max(value, fallback);
}

AuxiliarySensorFusion::ImuAngularSample AuxiliarySensorFusion::imu_angular_sample(const sensor_msgs::msg::Imu &msg) const
{
    ImuAngularSample sample;
    const auto &gyro = msg.angular_velocity;
    sample.raw_gyro = V3D(gyro.x, gyro.y, gyro.z);
    sample.valid = sample.raw_gyro.allFinite();

    if (msg.angular_velocity_covariance[0] < 0.0)
    {
        return sample;
    }
    M3D covariance;
    covariance << msg.angular_velocity_covariance[0],
                  msg.angular_velocity_covariance[1],
                  msg.angular_velocity_covariance[2],
                  msg.angular_velocity_covariance[3],
                  msg.angular_velocity_covariance[4],
                  msg.angular_velocity_covariance[5],
                  msg.angular_velocity_covariance[6],
                  msg.angular_velocity_covariance[7],
                  msg.angular_velocity_covariance[8];
    covariance = 0.5 * (covariance + covariance.transpose()).eval();
    if (!covariance.allFinite() || covariance.cwiseAbs().maxCoeff() <= 1e-18)
    {
        return sample;
    }
    Eigen::SelfAdjointEigenSolver<M3D> solver(covariance);
    if (solver.info() == Eigen::Success && solver.eigenvalues().minCoeff() >= -1e-12)
    {
        sample.covariance = covariance;
        sample.covariance_valid = true;
    }
    return sample;
}

AuxiliarySensorFusion::ImuAngularSample AuxiliarySensorFusion::imu_angular_sample_at_time(
    double target_t,
    const std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> &imu_msgs) const
{
    if (imu_msgs.empty()) return ImuAngularSample{};
    auto best = imu_msgs.begin();
    double best_diff = std::abs(get_time_sec((*best)->header.stamp) - target_t);
    for (auto it = imu_msgs.begin(); it != imu_msgs.end(); ++it)
    {
        const double d = std::abs(get_time_sec((*it)->header.stamp) - target_t);
        if (d < best_diff) { best_diff = d; best = it; }
    }
    return imu_angular_sample(**best);
}

V3D AuxiliarySensorFusion::dvl_measurement(const DvlMsg &msg) const
{
    const auto &linear = msg.twist.twist.linear;
    return V3D(linear.x, linear.y, linear.z);
}

M3D AuxiliarySensorFusion::dvl_measurement_covariance(const DvlMsg &msg) const
{
    M3D covariance;
    covariance << msg.twist.covariance[0], msg.twist.covariance[1], msg.twist.covariance[2],
                  msg.twist.covariance[6], msg.twist.covariance[7], msg.twist.covariance[8],
                  msg.twist.covariance[12], msg.twist.covariance[13], msg.twist.covariance[14];
    covariance = 0.5 * (covariance + covariance.transpose()).eval();

    bool use_message_covariance = covariance.allFinite();
    if (use_message_covariance)
    {
        Eigen::SelfAdjointEigenSolver<M3D> solver(covariance);
        use_message_covariance =
            solver.info() == Eigen::Success &&
            solver.eigenvalues().minCoeff() >= -1e-12 &&
            covariance.diagonal().minCoeff() > 1e-12;
    }
    if (!use_message_covariance)
    {
        covariance = M3D::Identity() * dvl_velocity_cov_;
    }
    else
    {
        for (int i = 0; i < 3; ++i)
        {
            covariance(i, i) = std::max(covariance(i, i), dvl_velocity_cov_);
        }
    }
    return covariance;
}

AuxiliarySensorFusion::DvlObservation AuxiliarySensorFusion::make_dvl_observation(const DvlMsg &msg,
                                    const ImuAngularSample &imu_sample,
                                    const state_ikfom &state) const
{
    DvlObservation observation;
    observation.measurement = dvl_measurement(msg);
    observation.covariance = dvl_measurement_covariance(msg);
    observation.imu = imu_sample;
    observation.valid = observation.measurement.allFinite() &&
                        observation.covariance.allFinite() &&
                        observation.imu.valid;
    return observation;
}

AuxiliarySensorFusion::DvlLinearization AuxiliarySensorFusion::build_dvl_linearization(const DvlObservation &observation,
                                         const state_ikfom &state) const
{
    DvlLinearization result;
    if (!observation.valid)
    {
        return result;
    }

    const auto model = underwater_fastlio::dvl::evaluate(
        state, observation.imu.raw_gyro, dvl_R_, dvl_T_);
    result.measurement = observation.measurement;
    result.prediction = model.prediction_dvl;
    result.residual = result.measurement - result.prediction;
    result.H = model.H;
    result.R = observation.covariance;

    if (observation.imu.covariance_valid && dvl_T_.squaredNorm() > 0.0)
    {
        const M3D C_DV = dvl_R_.transpose();
        const M3D p_cross = underwater_fastlio::dvl::skew(dvl_T_);
        result.R += C_DV * p_cross * observation.imu.covariance *
                    p_cross.transpose() * C_DV.transpose();
    }
    result.R = 0.5 * (result.R + result.R.transpose()).eval();
    Eigen::LLT<M3D> llt(result.R);
    result.valid = result.measurement.allFinite() &&
                   result.prediction.allFinite() &&
                   result.residual.allFinite() &&
                   result.H.allFinite() &&
                   result.R.allFinite() &&
                   llt.info() == Eigen::Success;
    return result;
}

bool AuxiliarySensorFusion::dvl_measurement_is_valid(const DvlLinearization &dvl) const
{
    return dvl.valid;
}

Eigen::RowVector3d AuxiliarySensorFusion::world_z_axis_in_camera_init() const
{
    return camera_init_R_in_world_.row(2);
}

double AuxiliarySensorFusion::pressure_sensor_z_world(const state_ikfom &state) const
{
    V3D pressure_offset_ci = V3D::Zero();
    pressure_offset_ci.z() = pressure_T_.z();
    const V3D sensor_ci = state.pos + pressure_offset_ci;
    return camera_init_T_in_world_.z() + world_z_axis_in_camera_init().dot(sensor_ci);
}

double AuxiliarySensorFusion::pressure_prediction(const state_ikfom &state) const
{
    const double relative_depth = pressure_reference_sensor_z_world_ - pressure_sensor_z_world(state);
    return pressure_reference_pa_ + pressure_scale(state) * relative_depth +
           state.b_pressure[0];
}

double AuxiliarySensorFusion::pressure_residual(const PressureMsg &msg, const state_ikfom &state) const
{
    return msg.fluid_pressure - pressure_prediction(state);
}

bool AuxiliarySensorFusion::pressure_reference_ready_or_collect(const PressureMsg &msg,
                                         const state_ikfom &state)
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
    if (!pressure_reference_pose_ready_)
    {
        return false;
    }

    // The state has been propagated to this pressure timestamp. Express
    // every startup sample at the one stored reference pose before taking
    // the mean, so vehicle motion during initialization cannot create an
    // artificial depth offset.
    const double relative_depth =
        pressure_reference_sensor_z_world_ - pressure_sensor_z_world(state);
    pressure_init_sum_ +=
        msg.fluid_pressure - pressure_scale(state) * relative_depth;
    ++pressure_init_samples_collected_;
    if (pressure_init_samples_collected_ < kPressureReferenceSamples)
    {
        return false;
    }

    const double mean_pressure =
        pressure_init_sum_ / static_cast<double>(pressure_init_samples_collected_);
    pressure_reference_pa_ = mean_pressure - state.b_pressure[0];
    pressure_ref_finalized_ = true;
    // Reference samples calibrate the local datum and are not reused as
    // independent Kalman measurements. Fusion starts with the next sample.
    return false;
}

bool AuxiliarySensorFusion::apply_dvl_update(const DvlMsg &msg,
                      const ImuAngularSample &imu_sample,
                      Ekf &kf)
{
    // Until the startup-relative magnetic heading is initialized, yaw is
    // unobservable in a stand-alone body-frame velocity update. Deferring
    // these first few DVL samples prevents an arbitrary startup yaw while
    // retaining the complete DVL Jacobian and gain thereafter.
    if (mag_enable_ && !mag_reference_is_ready())
    {
        return false;
    }
    const state_ikfom state = kf.get_x();
    const DvlLinearization dvl =
        build_dvl_linearization(make_dvl_observation(msg, imu_sample, state), state);
    if (!dvl_measurement_is_valid(dvl))
    {
        return false;
    }
    return apply_linear_update(dvl.residual, dvl.H, dvl.R, kf);
}

bool AuxiliarySensorFusion::apply_pressure_update(const PressureMsg &msg, Ekf &kf)
{
    const state_ikfom state = kf.get_x();
    if (!pressure_reference_ready_or_collect(msg, state))
    {
        return false;
    }

    Eigen::VectorXd residual(1);
    residual(0) = pressure_residual(msg, state);

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, state_ikfom::DOF);
    // Pressure measures World-z depth, but it must not correct horizontal
    // x/y. Use the World-z prediction and let the EKF correct only local depth.
    H(0, 2) = -pressure_scale(state) * world_z_axis_in_camera_init().z();
    H(0, 26) = 1.0;

    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(1, 1);
    R(0, 0) = covariance_or_fallback(msg.variance, pressure_cov_);

    return apply_pressure_depth_update(residual, H, R, kf);
}

bool AuxiliarySensorFusion::apply_pressure_depth_update(const Eigen::VectorXd &residual,
                                 const Eigen::MatrixXd &H,
                                 const Eigen::MatrixXd &R,
                                 Ekf &kf)
{
    if (residual.size() != 1 || H.rows() != 1 || H.cols() != state_ikfom::DOF ||
        R.rows() != 1 || R.cols() != 1 || !finite_vector(residual) ||
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

    Eigen::MatrixXd K = P * H.transpose() * ldlt.solve(Eigen::MatrixXd::Identity(1, 1));
    for (int row = 0; row < K.rows(); ++row)
    {
        if (row != 2 && row != 26)
        {
            K.row(row).setZero();
        }
    }

    const Eigen::VectorXd dx_dyn = K * residual;
    if (!finite_vector(dx_dyn))
    {
        return false;
    }

    typename Ekf::vectorized_state dx = Ekf::vectorized_state::Zero();
    dx = dx_dyn;

    state_ikfom updated_state = kf.get_x();
    updated_state.boxplus(dx);

    const typename Ekf::cov I_state = Ekf::cov::Identity();
    typename Ekf::cov KH = K * H;
    typename Ekf::cov P_new = (I_state - KH) * P * (I_state - KH).transpose() + K * R * K.transpose();
    P_new = 0.5 * (P_new + P_new.transpose()).eval();

    kf.change_x(updated_state);
    kf.change_P(P_new);
    return true;
}

typename AuxiliarySensorFusion::Ekf::cov AuxiliarySensorFusion::transport_covariance_after_boxplus(
    const typename Ekf::cov &P,
    state_ikfom prior_state,
    state_ikfom updated_state,
    const typename Ekf::vectorized_state &dx) const
{
    if (prior_state.SO3_state.empty() || prior_state.S2_state.empty())
    {
        prior_state.build_SO3_state();
        prior_state.build_S2_state();
    }
    if (updated_state.SO3_state.empty() || updated_state.S2_state.empty())
    {
        updated_state.build_SO3_state();
        updated_state.build_S2_state();
    }

    typename Ekf::cov transport = Ekf::cov::Identity();
    for (const auto &entry : updated_state.SO3_state)
    {
        const int index = entry.first;
        MTK::vect<3, double> segment;
        segment << dx(index), dx(index + 1), dx(index + 2);
        transport.template block<3, 3>(index, index) =
            MTK::A_matrix(segment).transpose();
    }
    for (const auto &entry : updated_state.S2_state)
    {
        const int index = entry.first;
        MTK::vect<2, double> segment;
        segment << dx(index), dx(index + 1);
        Eigen::Matrix<double, 2, 3> Nx;
        Eigen::Matrix<double, 3, 2> Mx;
        updated_state.S2_Nx_yy(Nx, index);
        prior_state.S2_Mx(Mx, segment, index);
        transport.template block<2, 2>(index, index) = Nx * Mx;
    }

    typename Ekf::cov transported = transport * P * transport.transpose();
    return 0.5 * (transported + transported.transpose()).eval();
}

bool AuxiliarySensorFusion::apply_linear_update(const Eigen::VectorXd &residual,
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
    Eigen::MatrixXd K = P * H.transpose() * ldlt.solve(I_meas);
    const Eigen::VectorXd dx_dyn = K * residual;
    if (!finite_vector(dx_dyn))
    {
        return false;
    }

    typename Ekf::vectorized_state dx = Ekf::vectorized_state::Zero();
    dx = dx_dyn;

    const state_ikfom prior_state = kf.get_x();
    state_ikfom updated_state = prior_state;
    updated_state.boxplus(dx);

    const typename Ekf::cov I_state = Ekf::cov::Identity();
    typename Ekf::cov KH = K * H;
    typename Ekf::cov P_new = (I_state - KH) * P * (I_state - KH).transpose() + K * R * K.transpose();
    P_new = 0.5 * (P_new + P_new.transpose()).eval();
    P_new = transport_covariance_after_boxplus(P_new, prior_state, updated_state, dx);

    kf.change_x(updated_state);
    kf.change_P(P_new);
    return true;
}

std::vector<AuxiliarySensorFusion::DvlMsg::ConstSharedPtr> AuxiliarySensorFusion::take_dvl_measurements(double begin_time, double end_time)
{
    std::vector<DvlMsg::ConstSharedPtr> messages;
    std::lock_guard<std::mutex> lock(mutex_);
    while (!dvl_buffer_.empty())
    {
        const double stamp = get_time_sec(dvl_buffer_.front()->header.stamp);
        if (stamp <= begin_time + 1e-6)
        {
            ++late_measurement_counts_.dvl;
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

std::vector<AuxiliarySensorFusion::PressureMsg::ConstSharedPtr> AuxiliarySensorFusion::take_pressure_measurements(double begin_time, double end_time)
{
    std::vector<PressureMsg::ConstSharedPtr> messages;
    std::lock_guard<std::mutex> lock(mutex_);
    while (!pressure_buffer_.empty())
    {
        const double stamp = get_time_sec(pressure_buffer_.front()->header.stamp);
        if (stamp <= begin_time + 1e-6)
        {
            ++late_measurement_counts_.pressure;
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

void AuxiliarySensorFusion::dvl_callback(const DvlMsg::ConstSharedPtr msg)
{
    const double timestamp = get_time_sec(msg->header.stamp);
    std::lock_guard<std::mutex> lock(mutex_);
    if (last_timestamp_dvl_ >= 0.0 &&
        std::abs(timestamp - last_timestamp_dvl_) <= 1e-9)
    {
        return;
    }
    if (timestamp < last_timestamp_dvl_)
    {
        dvl_buffer_.clear();
    }
    last_timestamp_dvl_ = timestamp;
    dvl_buffer_.push_back(msg);
}

void AuxiliarySensorFusion::pressure_callback(const PressureMsg::ConstSharedPtr msg)
{
    const double timestamp = get_time_sec(msg->header.stamp);
    std::lock_guard<std::mutex> lock(mutex_);
    if (last_timestamp_pressure_ >= 0.0 &&
        std::abs(timestamp - last_timestamp_pressure_) <= 1e-9)
    {
        return;
    }
    if (timestamp < last_timestamp_pressure_)
    {
        pressure_buffer_.clear();
        pressure_init_sum_ = 0.0;
        pressure_init_samples_collected_ = 0;
        pressure_ref_finalized_ = false;
        pressure_reference_pose_ready_ = false;
        last_pressure_raw_ = 0.0;
        last_pressure_raw_valid_ = false;
    }
    last_timestamp_pressure_ = timestamp;
    // The pressure driver can republish one hardware reading with fresh
    // ROS timestamps. Do not treat those correlated copies as independent
    // reference samples or Kalman measurements.
    if (last_pressure_raw_valid_ &&
        msg->fluid_pressure == last_pressure_raw_)
    {
        return;
    }
    last_pressure_raw_ = msg->fluid_pressure;
    last_pressure_raw_valid_ = true;
    pressure_buffer_.push_back(msg);
}

void AuxiliarySensorFusion::mag_callback(const MagMsg::ConstSharedPtr msg)
{
    const double timestamp = get_time_sec(msg->header.stamp);
    std::lock_guard<std::mutex> lock(mutex_);
    if (last_timestamp_mag_ >= 0.0 &&
        std::abs(timestamp - last_timestamp_mag_) <= 1e-9)
    {
        return;
    }
    if (timestamp < last_timestamp_mag_)
    {
        mag_buffer_.clear();
        reset_mag_reference_locked();
    }
    last_timestamp_mag_ = timestamp;

    // Some drivers republish one hardware reading at the IMU rate with a
    // fresh ROS timestamp. Treat an identical field vector as the same
    // magnetic sample so it cannot initialize or update the filter twice.
    const V3D raw(msg->magnetic_field.x,
                  msg->magnetic_field.y,
                  msg->magnetic_field.z);
    if (last_mag_raw_valid_ &&
        (raw.array() == last_mag_raw_.array()).all())
    {
        return;
    }
    last_mag_raw_ = raw;
    last_mag_raw_valid_ = true;
    mag_buffer_.push_back(msg);
}

V3D AuxiliarySensorFusion::mag_corrected(const MagMsg &msg) const
{
    const V3D raw(msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z);
    return mag_R_BM_ * mag_soft_iron_ * (raw - mag_hard_iron_);
}

M3D AuxiliarySensorFusion::mag_calibrated_covariance() const
{
    const M3D calibration = mag_R_BM_ * mag_soft_iron_;
    return mag_cov_ * calibration * calibration.transpose();
}

void AuxiliarySensorFusion::reset_mag_reference_locked()
{
    mag_reference_sum_local_.setZero();
    mag_reference_samples_local_.clear();
    mag_reference_sample_count_ = 0;
    mag_reference_ready_ = false;
    mag_vertical_local_ = V3D::UnitZ();
    mag_reference_local_.setZero();
    mag_horizontal_reference_local_.setZero();
    mag_reference_heading_variance_ = 0.0;
    last_mag_raw_.setZero();
    last_mag_raw_valid_ = false;
}

bool AuxiliarySensorFusion::mag_reference_is_ready() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return mag_reference_ready_;
}

bool AuxiliarySensorFusion::collect_mag_reference_sample(const state_ikfom &state,
                                  const V3D &magnetic_body)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (mag_reference_ready_)
    {
        return true;
    }

    if (mag_reference_sample_count_ == 0)
    {
        const V3D gravity_local(state.grav[0], state.grav[1], state.grav[2]);
        mag_vertical_local_ = V3D::UnitZ();
        if (gravity_local.norm() > 1e-6)
        {
            mag_vertical_local_ = (-gravity_local).normalized();
        }
    }

    if (!magnetic_body.allFinite() || magnetic_body.norm() <= 1e-12)
    {
        return false;
    }

    const V3D sample_local = state.rot.toRotationMatrix() * magnetic_body;
    V3D running_mean = V3D::Zero();
    if (mag_reference_sample_count_ > 0)
    {
        running_mean =
            mag_reference_sum_local_ / static_cast<double>(mag_reference_sample_count_);
    }
    if (!underwater_fastlio::magnetometer::reference_sample_is_inlier(
            sample_local, running_mean, mag_reference_sample_count_))
    {
        return false;
    }

    // Seed the startup-relative heading immediately. Waiting for the
    // entire initialization window before applying any heading update
    // lets an early gyro error rotate every R_i m_i sample and biases the
    // reference itself. Subsequent samples are accumulated only after
    // their heading update has placed them in this seeded local frame.
    if (mag_reference_sample_count_ == 0)
    {
        const V3D horizontal =
            sample_local - mag_vertical_local_ *
                               mag_vertical_local_.dot(sample_local);
        if (horizontal.norm() <= 1e-12)
        {
            return false;
        }
        mag_reference_sum_local_ = sample_local;
        mag_reference_samples_local_.push_back(sample_local);
        mag_reference_sample_count_ = 1;
        mag_reference_local_ = sample_local;
        mag_horizontal_reference_local_ = horizontal.normalized();
        return false;
    }

    // A provisional reference exists, so this sample can constrain
    // heading before it is admitted to the final initialization mean.
    return true;
}

void AuxiliarySensorFusion::accumulate_corrected_mag_reference_sample(
    const state_ikfom &updated_state,
    const V3D &magnetic_body)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (mag_reference_ready_ || mag_reference_sample_count_ == 0)
    {
        return;
    }

    const V3D sample_local =
        updated_state.rot.toRotationMatrix() * magnetic_body;
    const V3D running_mean =
        mag_reference_sum_local_ /
        static_cast<double>(mag_reference_sample_count_);
    if (!underwater_fastlio::magnetometer::reference_sample_is_inlier(
            sample_local, running_mean, mag_reference_sample_count_))
    {
        return;
    }

    mag_reference_sum_local_ += sample_local;
    mag_reference_samples_local_.push_back(sample_local);
    ++mag_reference_sample_count_;
    if (mag_reference_sample_count_ < kMagReferenceSamples)
    {
        return;
    }

    mag_reference_local_ =
        mag_reference_sum_local_ / static_cast<double>(mag_reference_sample_count_);
    const V3D n = mag_vertical_local_;
    const V3D horizontal =
        mag_reference_local_ - n * n.dot(mag_reference_local_);
    if (mag_reference_local_.norm() <= 1e-12 || horizontal.norm() <= 1e-12)
    {
        reset_mag_reference_locked();
        return;
    }

    mag_horizontal_reference_local_ = horizontal.normalized();
    double heading_sq_sum = 0.0;
    for (const V3D &sample : mag_reference_samples_local_)
    {
        V3D sample_horizontal = sample - n * n.dot(sample);
        if (sample_horizontal.norm() <= 1e-12)
        {
            continue;
        }
        sample_horizontal.normalize();
        const double angle = std::atan2(
            n.dot(mag_horizontal_reference_local_.cross(sample_horizontal)),
            mag_horizontal_reference_local_.dot(sample_horizontal));
        heading_sq_sum += angle * angle;
    }
    mag_reference_heading_variance_ =
        heading_sq_sum /
        (static_cast<double>(mag_reference_sample_count_) *
         static_cast<double>(mag_reference_sample_count_));
    mag_reference_ready_ = true;
}

std::vector<AuxiliarySensorFusion::MagMsg::ConstSharedPtr> AuxiliarySensorFusion::take_mag_measurements(double begin_time, double end_time)
{
    std::vector<MagMsg::ConstSharedPtr> messages;
    std::lock_guard<std::mutex> lock(mutex_);
    while (!mag_buffer_.empty())
    {
        const double stamp = get_time_sec(mag_buffer_.front()->header.stamp);
        if (stamp <= begin_time + 1e-6)
        {
            ++late_measurement_counts_.magnetometer;
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

bool AuxiliarySensorFusion::apply_mag_update(const MagMsg &msg, Ekf &kf)
{
    const state_ikfom state = kf.get_x();
    const V3D measured = mag_corrected(msg);

    if (!collect_mag_reference_sample(state, measured))
    {
        return false;
    }
    V3D h0;
    V3D vertical_local;
    double reference_variance = 0.0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        h0 = mag_horizontal_reference_local_;
        vertical_local = mag_vertical_local_;
        reference_variance = mag_reference_heading_variance_;
    }

    const auto observation = underwater_fastlio::magnetometer::evaluate(
        state.rot.toRotationMatrix(), measured, h0, vertical_local);
    if (!observation.valid)
    {
        return false;
    }

    const M3D calibrated_covariance = mag_calibrated_covariance();
    double heading_covariance =
        (observation.magnetic_jacobian * calibrated_covariance *
         observation.magnetic_jacobian.transpose())(0, 0) +
        reference_variance;
    heading_covariance = std::max(1e-12, heading_covariance);

    Eigen::RowVectorXd H = Eigen::RowVectorXd::Zero(state_ikfom::DOF);
    H.segment<3>(3) = observation.observation_jacobian;
    typename Ekf::cov P = kf.get_P();
    const double innovation_variance = (H * P * H.transpose())(0, 0) + heading_covariance;
    if (!H.allFinite() || !P.allFinite() || !std::isfinite(innovation_variance) ||
        innovation_variance <= 0.0)
    {
        return false;
    }

    // A local magnetic heading directly observes attitude, not gyro bias.
    // Keep its constrained gain attitude-only so transient disagreement
    // cannot be stored as a persistent bias and integrated into yaw.
    const Eigen::VectorXd K = underwater_fastlio::magnetometer::constrained_gain(
        P, H, innovation_variance, observation.g, 3);
    const Eigen::VectorXd dx_dyn = K * observation.innovation;
    if (!finite_vector(dx_dyn))
    {
        return false;
    }

    typename Ekf::vectorized_state dx = Ekf::vectorized_state::Zero();
    dx = dx_dyn;
    state_ikfom updated_state = state;
    updated_state.boxplus(dx);

    const V3D g_after =
        updated_state.rot.toRotationMatrix().transpose() * vertical_local;
    const double tilt_axis_change = (g_after - observation.g).norm();
    Eigen::VectorXd protected_dx = dx_dyn;
    protected_dx.segment<3>(3).setZero();
    protected_dx.segment<3>(15).setZero();
    const double protected_state_change = protected_dx.cwiseAbs().maxCoeff();
    if (tilt_axis_change > 1e-10 || protected_state_change > 1e-14)
    {
        return false;
    }

    Eigen::MatrixXd P_new = underwater_fastlio::magnetometer::joseph_covariance(
        P, H, K, heading_covariance);
    P_new = underwater_fastlio::magnetometer::transport_attitude_covariance(
        P_new, dx_dyn.segment<3>(3), 3);
    if (!underwater_fastlio::magnetometer::covariance_is_psd(P_new, 1e-9))
    {
        return false;
    }

    accumulate_corrected_mag_reference_sample(updated_state, measured);

    kf.change_x(updated_state);
    typename Ekf::cov fixed_P = P_new;
    kf.change_P(fixed_P);
    return true;
}

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "IMU_Processing.hpp"
#include "auxiliary_sensor_fusion.hpp"
#include "mapping_input_buffers.hpp"
#include "imu_observation_calendar.hpp"
#include "replay_trace.hpp"
#include <thread>

namespace
{
TEST(InputBuffers, RegressionClearsPairedSonarQueuesAndPredictionHistory)
{
    uwfl2::MappingInputBuffers buffers;
    auto cloud = std::make_shared<PointCloudXYZI>();
    buffers.PushSonar(cloud, 2.0);
    buffers.PushSonar(cloud, 3.0);
    buffers.lidar_pushed = true;
    buffers.PushSonar(cloud, 1.0);
    EXPECT_FALSE(buffers.lidar_pushed);
    ASSERT_EQ(buffers.time_buffer.size(), 1U);
    ASSERT_EQ(buffers.lidar_buffer.size(), 1U);
    EXPECT_DOUBLE_EQ(buffers.time_buffer.front(), 1.0);
    for (const double time : {2.0, 3.0, 1.0})
    {
        auto message = std::make_shared<sensor_msgs::msg::Imu>();
        message->header.stamp = get_ros_time(time);
        buffers.PushImu(message);
    }
    EXPECT_EQ(buffers.imu_buffer.size(), 1U);
    EXPECT_EQ(buffers.odometry_prediction_imu_buffer.size(), 1U);
    EXPECT_DOUBLE_EQ(buffers.last_timestamp_imu, 1.0);
}

TEST(InputBuffers, ConcurrentSensorsKeepQueuesPairedAndHistoryBounded)
{
    uwfl2::MappingInputBuffers buffers;
    auto cloud = std::make_shared<PointCloudXYZI>();
    std::thread sonar([&] {
        for (int i = 0; i < 200; ++i) buffers.PushSonar(cloud, 1.0 + i * 0.1);
    });
    std::thread imu([&] {
        for (int i = 0; i < 4100; ++i)
        {
            auto message = std::make_shared<sensor_msgs::msg::Imu>();
            message->header.stamp = get_ros_time(1.0 + i * 0.001);
            buffers.PushImu(message);
        }
    });
    sonar.join();
    imu.join();
    EXPECT_EQ(buffers.time_buffer.size(), 200U);
    EXPECT_EQ(buffers.lidar_buffer.size(), buffers.time_buffer.size());
    EXPECT_EQ(buffers.imu_buffer.size(), 4100U);
    EXPECT_EQ(buffers.odometry_prediction_imu_buffer.size(), 4000U);
    EXPECT_EQ(buffers.imu_buffer.back(), buffers.odometry_prediction_imu_buffer.back());
}

TEST(InputBuffers, PredictionHistoryRetainsCorrectionEpochWhenPredictorIsAhead)
{
    uwfl2::MappingInputBuffers buffers;
    for (int i = 0; i <= 300; ++i)
    {
        auto message = std::make_shared<sensor_msgs::msg::Imu>();
        message->header.stamp = get_ros_time(1.0 + i * 0.01);
        buffers.PushImu(message);
    }
    const auto ahead = buffers.PredictionSamples(3.0, 2.0);
    ASSERT_GE(ahead.size(), 2U);
    EXPECT_LE(get_time_sec(ahead.front()->header.stamp), 3.0);
    // A correction at 2 s must replay real samples, not use the 3 s input early.
    const auto rewind = buffers.PredictionSamples(2.0, 2.0);
    ASSERT_GE(rewind.size(), 2U);
    EXPECT_LE(get_time_sec(rewind.front()->header.stamp), 2.0);
    EXPECT_GE(get_time_sec(rewind[1]->header.stamp), 2.0);
    EXPECT_GE(rewind.size(), 200U);
    EXPECT_EQ(buffers.odometry_prediction_imu_buffer.back(), buffers.imu_buffer.back());
}

TEST(InputBuffers, PredictionHistoryDoesNotFabricateSamplesBeyondHardLimit)
{
    uwfl2::MappingInputBuffers buffers;
    for (int i = 0; i < 4100; ++i)
    {
        auto message = std::make_shared<sensor_msgs::msg::Imu>();
        message->header.stamp = get_ros_time(1.0 + i * 0.001);
        buffers.PushImu(message);
    }
    const auto samples = buffers.PredictionSamples(1.0, 1.0);
    EXPECT_EQ(samples.size(), 4000U);
    ASSERT_FALSE(samples.empty());
    EXPECT_GT(get_time_sec(samples.front()->header.stamp), 1.0);
    EXPECT_EQ(buffers.odometry_prediction_imu_buffer.size(), 4000U);
}

using Ekf = ImuProcess::Ekf;

TEST(InputBuffers, TransportWatermarkIsIndependentOfSonarAvailability)
{
    using Clock = uwfl2::MappingInputBuffers::Clock;
    const auto epoch = Clock::time_point(std::chrono::seconds(10));
    uwfl2::MappingInputBuffers buffers;
    buffers.last_imu_receipt = epoch;
    EXPECT_DOUBLE_EQ(buffers.PropagationWatermark(28.0, epoch), 27.98);
    buffers.NoteSonarReceipt(epoch);
    EXPECT_DOUBLE_EQ(buffers.PropagationWatermark(28.0, epoch), 27.98);
    auto cloud = std::make_shared<PointCloudXYZI>();
    buffers.last_timestamp_imu = 28.0;
    buffers.PushSonar(cloud, 27.53);
    EXPECT_FALSE(buffers.sonar_processing);
    EXPECT_EQ(buffers.sonar_received, 1U);
    EXPECT_NEAR(buffers.max_sonar_acquisition_lag, 0.47, 1e-12);
    EXPECT_DOUBLE_EQ(buffers.PropagationWatermark(28.0, epoch), 27.98);
    EXPECT_DOUBLE_EQ(buffers.PropagationWatermark(
        28.0, epoch + std::chrono::milliseconds(249)), 27.98);
    EXPECT_DOUBLE_EQ(buffers.PropagationWatermark(
        28.0, epoch + std::chrono::milliseconds(250)), 28.0);
}

struct RosContext
{
    RosContext() { rclcpp::init(0, nullptr); }
    ~RosContext() { rclcpp::shutdown(); }
};

TEST(SensorParameters, MagnetometerDeclaresRenamedCovariance)
{
    RosContext context;
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("magnetometer.covariance", 0.03)});
    rclcpp::Node node("magnetometer_parameters", options);
    AuxiliarySensorFusion fusion;
    fusion.declare_parameters(node);
    fusion.load_parameters(node);
    EXPECT_DOUBLE_EQ(node.get_parameter("magnetometer.covariance").as_double(), 0.03);
    EXPECT_FALSE(node.has_parameter("magnetometer.mag_cov"));
    EXPECT_FALSE(node.has_parameter("magnetometer.heading_cov_floor"));
    EXPECT_FALSE(node.has_parameter("magnetometer.extrinsic_R"));
    EXPECT_FALSE(node.has_parameter("dvl.extrinsic_T"));
    EXPECT_FALSE(node.has_parameter("pressure.timeout"));
    EXPECT_DOUBLE_EQ(node.get_parameter("pressure.frequency").as_double(), 4.0);
    EXPECT_EQ(node.get_parameter("dvl.rotation").as_double_array(),
              (std::vector<double>{0.0, 0.0, 0.0}));
}

void NoMeasurement(state_ikfom &, esekfom::dyn_share_datastruct<double> &data)
{
    data.valid = false;
}

void InitializeFilter(Ekf &filter)
{
    double epsilon[state_ikfom::DOF];
    std::fill_n(epsilon, state_ikfom::DOF, 1e-3);
    filter.init_dyn_share(get_f, df_dx, df_dw, NoMeasurement, 4, epsilon);
}

TEST(ReplayTrace, FingerprintsLogicalFieldsRatherThanSerializedPadding)
{
    sensor_msgs::msg::Imu message;
    message.header.stamp = get_ros_time(1.0);
    message.header.frame_id = "imu";
    message.linear_acceleration.z = 9.81;
    const auto fingerprint = uwfl2::ReplayTrace::Fingerprint(message);
    auto copy = message;
    EXPECT_EQ(fingerprint, uwfl2::ReplayTrace::Fingerprint(copy));
    ++copy.header.stamp.nanosec;
    EXPECT_NE(fingerprint, uwfl2::ReplayTrace::Fingerprint(copy));
    copy = message;
    copy.header.frame_id = "other_imu";
    EXPECT_NE(fingerprint, uwfl2::ReplayTrace::Fingerprint(copy));
    copy = message;
    copy.angular_velocity.z = 0.01;
    EXPECT_NE(fingerprint, uwfl2::ReplayTrace::Fingerprint(copy));
    copy = message;
    copy.angular_velocity_covariance[0] = 0.02;
    EXPECT_NE(fingerprint, uwfl2::ReplayTrace::Fingerprint(copy));
}

TEST(ReplayTrace, RecordingCannotChangeMainStateOrCovariance)
{
    Ekf filter;
    InitializeFilter(filter);
    auto before = filter.get_x();
    const auto covariance = filter.get_P();
    uwfl2::ReplayTrace trace;
    trace.Record("fixture", 1.0, 1.0, 0, 0, filter);
    auto after = filter.get_x();
    Eigen::Matrix<double, state_ikfom::DOF, 1> difference;
    after.boxminus(difference, before);
    EXPECT_DOUBLE_EQ(difference.norm(), 0.0);
    EXPECT_DOUBLE_EQ((filter.get_P() - covariance).norm(), 0.0);
}

MeasureGroup StationaryInterval(double begin, double end)
{
    MeasureGroup measurements;
    measurements.lidar_beg_time = begin;
    measurements.lidar_end_time = end;
    for (int i = 0; i <= 20; ++i)
    {
        auto imu = std::make_shared<sensor_msgs::msg::Imu>();
        imu->header.stamp = rclcpp::Time(static_cast<int64_t>(
            (begin + (end - begin) * i / 20.0) * 1e9));
        imu->linear_acceleration.z = 9.81;
        measurements.imu.push_back(imu);
    }
    return measurements;
}

TEST(ImuCharacterization, ConfiguredBiasNoiseSurvivesEmptyScansAndLongAbsence)
{
    Ekf filter;
    InitializeFilter(filter);
    ImuProcess imu;
    const V3D gyro_noise(0.04, 0.05, 0.06);
    const V3D accel_noise(0.08, 0.09, 0.10);
    imu.set_gyr_bias_cov(gyro_noise);
    imu.set_acc_bias_cov(accel_noise);
    PointCloudXYZI::Ptr output(new PointCloudXYZI());
    imu.Process(StationaryInterval(1.0, 1.2), filter, output);
    ASSERT_TRUE(imu.IsInitialized());
    for (int i = 0; i < 30; ++i)
    {
        const double begin = 1.2 + i * 0.2;
        imu.Process(StationaryInterval(begin, begin + 0.2), filter, output);
        EXPECT_TRUE((imu.Q.block<3, 3>(6, 6).diagonal().isApprox(gyro_noise, 0.0)));
        EXPECT_TRUE((imu.Q.block<3, 3>(9, 9).diagonal().isApprox(accel_noise, 0.0)));
        EXPECT_TRUE(filter.get_P().allFinite());
    }
}

TEST(ImuCharacterization, TimedUpdatesRetainEachConfiguredBiasNoise)
{
    Ekf filter;
    InitializeFilter(filter);
    ImuProcess imu;
    const V3D gyro_noise(0.04, 0.05, 0.06);
    const V3D accel_noise(0.08, 0.09, 0.10);
    imu.set_gyr_bias_cov(gyro_noise);
    imu.set_acc_bias_cov(accel_noise);
    PointCloudXYZI::Ptr output(new PointCloudXYZI());
    imu.Process(StationaryInterval(1.0, 1.2), filter, output);
    ASSERT_TRUE(imu.IsInitialized());
    int events = 0;
    imu.Process(StationaryInterval(1.2, 1.4), filter, output, {1.3},
                [&](std::size_t, Ekf &) { ++events; return false; });
    EXPECT_EQ(events, 1);
    EXPECT_TRUE((imu.Q.block<3, 3>(6, 6).diagonal().isApprox(gyro_noise, 0.0)));
    EXPECT_TRUE((imu.Q.block<3, 3>(9, 9).diagonal().isApprox(accel_noise, 0.0)));
}

TEST(ImuCalendar, PacketBoundariesDoNotChangeEpochsOrAveragingSamples)
{
    auto source = StationaryInterval(1.2, 1.4).imu;
    for (const auto &sample : StationaryInterval(1.41, 2.0).imu)
        source.push_back(sample);
    uwfl2::ImuObservationCalendar whole, packets;
    whole.Reset(1.2);
    packets.Reset(1.2);
    const auto expected = whole.Take(source, 2.0);
    std::vector<uwfl2::ImuObservationCalendar::Observation> actual;
    for (const auto &sample : source)
    {
        // Deliberately repeat the boundary sample to exercise once-only consumption.
        const auto events = packets.Take({sample, sample}, get_time_sec(sample->header.stamp));
        actual.insert(actual.end(), events.begin(), events.end());
    }
    ASSERT_EQ(actual.size(), expected.size());
    ASSERT_EQ(actual.size(), 4U);
    for (std::size_t i = 0; i < expected.size(); ++i)
    {
        EXPECT_DOUBLE_EQ(actual[i].timestamp, expected[i].timestamp);
        EXPECT_EQ(actual[i].samples, expected[i].samples);
    }
    EXPECT_TRUE(whole.Take(source, 2.0).empty());
}

TEST(ImuCalendar, FutureSamplesAndAcquisitionGapsDoNotFabricateObservations)
{
    uwfl2::ImuObservationCalendar calendar;
    calendar.Reset(1.2);
    const auto source = StationaryInterval(1.2, 1.4).imu;
    EXPECT_TRUE(calendar.Take(source, 1.3).empty());
    ASSERT_EQ(calendar.Take(source, 1.4).size(), 1U);
    auto resumed = std::make_shared<sensor_msgs::msg::Imu>();
    resumed->header.stamp = get_ros_time(3.0);
    const auto events = calendar.Take({resumed}, 3.0);
    ASSERT_EQ(events.size(), 1U);
    EXPECT_DOUBLE_EQ(events.front().timestamp, 3.0);
    EXPECT_EQ(events.front().samples.size(), 1U);
}

TEST(ImuCharacterization, EmptyMissingAndDisabledSonarShareStateAndCovariance)
{
    Ekf whole_filter, packet_filter;
    InitializeFilter(whole_filter);
    InitializeFilter(packet_filter);
    ImuProcess whole_imu, packet_imu;
    PointCloudXYZI::Ptr whole_cloud(new PointCloudXYZI()), packet_cloud(new PointCloudXYZI());
    const auto startup = StationaryInterval(1.0, 1.2);
    whole_imu.Process(startup, whole_filter, whole_cloud);
    packet_imu.Process(startup, packet_filter, packet_cloud);
    ASSERT_TRUE(whole_imu.IsInitialized());
    auto saved_state = whole_filter.get_x();
    auto saved_covariance = whole_filter.get_P();
    packet_filter.change_x(saved_state);
    packet_filter.change_P(saved_covariance);
    uwfl2::ImuObservationCalendar whole_calendar, packet_calendar;
    whole_calendar.Reset(1.2);
    packet_calendar.Reset(1.2);
    auto source = StationaryInterval(1.2, 2.2);
    // Use a canonical 100 Hz acquisition stream for both packetizations.
    source.imu.clear();
    for (int i = 1; i <= 100; ++i)
    {
        auto sample = std::make_shared<sensor_msgs::msg::Imu>();
        sample->header.stamp = get_ros_time(1.2 + i * 0.01);
        sample->linear_acceleration.z = 9.81;
        sample->angular_velocity.z = 0.05 + 0.02 * std::sin(i * 0.1);
        sample->linear_acceleration.x = 0.2 * std::sin(i * 0.1);
        source.imu.push_back(sample);
    }
    auto process = [](ImuProcess &imu, Ekf &filter, uwfl2::ImuObservationCalendar &calendar,
                      const MeasureGroup &packet, PointCloudXYZI::Ptr cloud) {
        const auto events = calendar.Take(packet.imu, packet.lidar_end_time);
        std::vector<double> stamps;
        for (const auto &event : events) stamps.push_back(event.timestamp);
        imu.Process(packet, filter, cloud, stamps, [](std::size_t, Ekf &event_filter) {
            // A real timestamped scalar correction, applied identically in all modes.
            auto state = event_filter.get_x();
            auto covariance = event_filter.get_P();
            Eigen::Matrix<double, 1, state_ikfom::DOF> H = Eigen::Matrix<double, 1, state_ikfom::DOF>::Zero();
            H(0, 14) = 1.0;
            const double R = 0.01;
            const auto gain = (covariance * H.transpose() / ((H * covariance * H.transpose())(0, 0) + R)).eval();
            state.boxplus((gain * (-state.vel[2])).eval());
            const auto A = (Ekf::cov::Identity() - gain * H).eval();
            event_filter.change_x(state);
            Ekf::cov corrected = (A * covariance * A.transpose() + gain * R * gain.transpose()).eval();
            event_filter.change_P(corrected);
            return true;
        });
    };
    process(whole_imu, whole_filter, whole_calendar, source, whole_cloud);
    for (const auto &sample : source.imu)
    {
        MeasureGroup packet;
        packet.lidar_beg_time = get_time_sec(sample->header.stamp) - 0.01;
        packet.lidar_end_time = get_time_sec(sample->header.stamp);
        packet.imu.push_back(sample);
        process(packet_imu, packet_filter, packet_calendar, packet, packet_cloud);
    }
    auto actual = packet_filter.get_x();
    auto expected = whole_filter.get_x();
    Eigen::Matrix<double, state_ikfom::DOF, 1> error;
    actual.boxminus(error, expected);
    EXPECT_LT(error.cwiseAbs().maxCoeff(), 1e-8);
    EXPECT_LT((packet_filter.get_P() - whole_filter.get_P()).cwiseAbs().maxCoeff(), 1e-8);
}

TEST(ImuCharacterization, StartupPreservesRotationAndInitializesBiasCovariances)
{
    Ekf filter;
    InitializeFilter(filter);
    state_ikfom initial;
    initial.rot = SO3(Eigen::AngleAxisd(0.4, V3D::UnitZ()));
    filter.change_x(initial);
    ImuProcess imu;
    imu.set_initial_cov(V3D::Constant(0.02), V3D::Constant(0.03), 0.04);
    imu.set_initial_aux_cov(V3D::Constant(0.05), 0.06);
    auto measurements = StationaryInterval(1.0, 1.2);
    PointCloudXYZI::Ptr output(new PointCloudXYZI());
    imu.Process(measurements, filter, output);
    ASSERT_TRUE(imu.IsInitialized());
    EXPECT_TRUE(filter.get_x().rot.toRotationMatrix().isApprox(initial.rot.toRotationMatrix()));
    EXPECT_NEAR(filter.get_x().grav[2], -9.81000042, 1e-7);
    EXPECT_NEAR(filter.get_x().bg.norm(), 0.0, 1e-12);
    EXPECT_DOUBLE_EQ(filter.get_P()(15, 15), 0.02);
    EXPECT_DOUBLE_EQ(filter.get_P()(18, 18), 0.03);
    EXPECT_DOUBLE_EQ(filter.get_P()(21, 21), 0.04);
    EXPECT_DOUBLE_EQ(filter.get_P()(23, 23), 0.05);
    EXPECT_DOUBLE_EQ(filter.get_P()(26, 26), 0.06);
}

TEST(ImuCharacterization, TimedCorrectionCanChangeDeskewWithoutChangingFinalFilter)
{
    Ekf rebuilt_filter, recorded_filter;
    InitializeFilter(rebuilt_filter);
    InitializeFilter(recorded_filter);
    ImuProcess rebuilt_imu, recorded_imu;
    PointCloudXYZI::Ptr rebuilt_cloud(new PointCloudXYZI()), recorded_cloud(new PointCloudXYZI());
    const auto startup = StationaryInterval(1.0, 1.2);
    rebuilt_imu.Process(startup, rebuilt_filter, rebuilt_cloud);
    recorded_imu.Process(startup, recorded_filter, recorded_cloud);
    auto scan = StationaryInterval(1.2, 1.4);
    for (int i = 0; i <= 20; ++i)
    {
        PointType point;
        point.x = 10.0F; point.y = 0.0F; point.z = 0.0F;
        point.curvature = static_cast<float>(i * 10);
        scan.lidar->push_back(point);
    }
    auto correction = [](bool rebuild, Ekf &filter) {
        auto state = filter.get_x();
        Eigen::Matrix<double, state_ikfom::DOF, 1> dx =
            Eigen::Matrix<double, state_ikfom::DOF, 1>::Zero();
        dx[5] = 0.02;
        state.boxplus(dx);
        filter.change_x(state);
        return rebuild;
    };
    rebuilt_imu.Process(scan, rebuilt_filter, rebuilt_cloud, {1.3},
        [&](std::size_t, Ekf &filter) { return correction(true, filter); });
    recorded_imu.Process(scan, recorded_filter, recorded_cloud, {1.3},
        [&](std::size_t, Ekf &filter) { return correction(false, filter); });
    auto actual = rebuilt_filter.get_x();
    auto expected = recorded_filter.get_x();
    Eigen::Matrix<double, state_ikfom::DOF, 1> difference;
    actual.boxminus(difference, expected);
    EXPECT_LT(difference.norm(), 1e-12);
    EXPECT_LT((rebuilt_filter.get_P() - recorded_filter.get_P()).norm(), 1e-12);
    ASSERT_EQ(rebuilt_cloud->size(), recorded_cloud->size());
    double maximum = 0.0;
    for (std::size_t i = 0; i < rebuilt_cloud->size(); ++i)
    {
        const auto a = rebuilt_cloud->points[i].getVector3fMap();
        const auto b = recorded_cloud->points[i].getVector3fMap();
        ASSERT_TRUE(a.allFinite());
        ASSERT_TRUE(b.allFinite());
        maximum = std::max(maximum, static_cast<double>((a - b).norm()));
    }
    RecordProperty("deskew_max_difference_m", maximum);
    EXPECT_GT(maximum, 0.01);
}

TEST(ImuCharacterization, NoScanPropagatesAndTimedCallbacksRemainOrdered)
{
    Ekf filter;
    InitializeFilter(filter);
    ImuProcess imu;
    PointCloudXYZI::Ptr output(new PointCloudXYZI());
    imu.Process(StationaryInterval(1.0, 1.2), filter, output);
    std::vector<std::size_t> calls;
    imu.Process(StationaryInterval(1.2, 1.4), filter, output,
                {1.25, 1.3, 1.35}, [&](std::size_t index, Ekf &) {
                    calls.push_back(index);
                    return false;
                });
    EXPECT_EQ(calls, (std::vector<std::size_t>{0, 1, 2}));
    EXPECT_LT(filter.get_x().pos.norm(), 1e-6);
    EXPECT_TRUE(filter.get_P().allFinite());
    EXPECT_TRUE(filter.get_P().isApprox(filter.get_P().transpose(), 1e-10));
    imu.Process(StationaryInterval(1.4, 1.6), filter, output);
    EXPECT_LT(filter.get_x().pos.norm(), 1e-6);
}

TEST(ImuCharacterization, RepeatedStartupUsesIdenticalInputsAndCorrectionEpochs)
{
    Ekf first, second;
    InitializeFilter(first);
    InitializeFilter(second);
    state_ikfom initial;
    initial.rot = SO3(Eigen::AngleAxisd(0.2, V3D(1, 2, 3).normalized()));
    first.change_x(initial);
    second.change_x(initial);
    ImuProcess first_imu, second_imu;
    PointCloudXYZI::Ptr first_cloud(new PointCloudXYZI());
    PointCloudXYZI::Ptr second_cloud(new PointCloudXYZI());
    // Explicit fixture correction epochs, not inferred from published odometry.
    for (const int64_t end_ns : {1200000000LL, 1400000000LL, 1600000000LL})
    {
        const double end = end_ns / 1e9;
        const auto interval = StationaryInterval(end - 0.2, end);
        first_imu.Process(interval, first, first_cloud);
        second_imu.Process(interval, second, second_cloud);
        Ekf::vectorized_state difference = Ekf::vectorized_state::Zero();
        first.get_x().boxminus(difference, second.get_x());
        EXPECT_LE(difference.norm(), 1e-12);
        EXPECT_LE((first.get_P() - second.get_P()).norm(), 1e-12);
        EXPECT_LE((first_imu.Q - second_imu.Q).norm(), 1e-12);
    }
}

TEST(ImuCharacterization, PredictionResetIsDeterministicAndDoesNotChangeMainFilter)
{
    Ekf main, first, second;
    InitializeFilter(main);
    InitializeFilter(first);
    InitializeFilter(second);
    ImuProcess imu;
    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
    imu.Process(StationaryInterval(1.0, 1.2), main, cloud);
    input_ikfom input;
    input.gyro = V3D(0.01, -0.02, 0.03);
    input.acc = V3D(0.2, -0.1, 9.81);
    for (int reset = 0; reset < 3; ++reset)
    {
        auto state = main.get_x();
        auto covariance = main.get_P();
        first.change_x(state);
        first.change_P(covariance);
        second.change_x(state);
        second.change_P(covariance);
        auto first_noise = imu.Q, second_noise = imu.Q;
        for (int step = 0; step < 20; ++step)
        {
            double first_dt = 0.01, second_dt = 0.01;
            first.predict(first_dt, first_noise, input);
            second.predict(second_dt, second_noise, input);
            Ekf::vectorized_state difference = Ekf::vectorized_state::Zero();
            first.get_x().boxminus(difference, second.get_x());
            EXPECT_LE(difference.norm(), 1e-12);
            EXPECT_LE((first.get_P() - second.get_P()).norm(), 1e-12);
        }
        Ekf::vectorized_state main_difference = Ekf::vectorized_state::Zero();
        main.get_x().boxminus(main_difference, state);
        EXPECT_EQ(main_difference.norm(), 0.0);
        EXPECT_EQ((main.get_P() - covariance).norm(), 0.0);
        imu.Process(StationaryInterval(1.2 + reset * 0.2, 1.4 + reset * 0.2),
                    main, cloud);
    }
}

TEST(DvlCharacterization, NativeFrameAndRightPerturbationJacobian)
{
    state_ikfom state;
    state.rot = SO3(Eigen::AngleAxisd(0.6, V3D(1, 2, 3).normalized()));
    state.vel = V3D(0.4, -0.2, 0.1);
    const M3D rotation = Eigen::AngleAxisd(-0.2, V3D::UnitY()).toRotationMatrix();
    const V3D gyro(0.02, 0.01, -0.03), lever(0.1, -0.2, 0.05);
    const auto model = underwater_fastlio::dvl::evaluate(state, gyro, rotation, lever);
    EXPECT_TRUE(model.prediction_dvl.isApprox(rotation.transpose() *
        (state.rot.toRotationMatrix().transpose() * state.vel + gyro.cross(lever))));
    constexpr double step = 1e-6;
    for (const int block : {3, 12, 15, 23})
    {
        for (int axis = 0; axis < 3; ++axis)
        {
            Eigen::Matrix<double, state_ikfom::DOF, 1> delta =
                Eigen::Matrix<double, state_ikfom::DOF, 1>::Zero();
            delta[block + axis] = step;
            auto plus = state, minus = state;
            plus.boxplus(delta);
            delta *= -1.0;
            minus.boxplus(delta);
            const V3D numerical = (underwater_fastlio::dvl::evaluate(plus, gyro, rotation, lever).prediction_dvl -
                underwater_fastlio::dvl::evaluate(minus, gyro, rotation, lever).prediction_dvl) / (2 * step);
            EXPECT_TRUE(numerical.isApprox(model.H.col(block + axis), 1e-7));
        }
    }
}

TEST(MagneticCharacterization, HeadingOnlyGainAndJosephCovariance)
{
    namespace mag = underwater_fastlio::magnetometer;
    const M3D rotation = (Eigen::AngleAxisd(0.2, V3D::UnitZ()) *
                          Eigen::AngleAxisd(0.3, V3D::UnitY())).toRotationMatrix();
    const V3D field = Eigen::AngleAxisd(0.3, V3D::UnitY()).toRotationMatrix().transpose() * V3D(1, 0, 0.4);
    const auto observation = mag::evaluate(rotation, field, V3D::UnitX());
    ASSERT_TRUE(observation.valid);
    Eigen::RowVectorXd H = Eigen::RowVectorXd::Zero(27);
    H.segment<3>(3) = observation.observation_jacobian;
    const Eigen::MatrixXd P = Eigen::MatrixXd::Identity(27, 27);
    const double variance = (H * P * H.transpose())(0, 0) + 0.01;
    const auto gain = mag::constrained_gain(P, H, variance, observation.g);
    EXPECT_EQ(gain.head<3>().norm(), 0.0);
    EXPECT_EQ(gain.tail(21).norm(), 0.0);
    const V3D correction = gain.segment<3>(3) * observation.innovation;
    const M3D corrected = rotation * Eigen::AngleAxisd(correction.norm(), correction.normalized()).toRotationMatrix();
    EXPECT_TRUE((corrected.transpose() * V3D::UnitZ()).isApprox(observation.g, 1e-12));
    EXPECT_LT(std::abs(mag::evaluate(corrected, field, V3D::UnitX()).innovation),
              std::abs(observation.innovation));
    const Eigen::MatrixXd posterior = mag::joseph_covariance(P, H, gain, 0.01);
    EXPECT_TRUE(posterior.isApprox(posterior.transpose(), 1e-12));
    EXPECT_GE(Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(posterior).eigenvalues().minCoeff(), -1e-12);
}

TEST(MagneticCharacterization, EquivalentFieldVarianceMatchesOnlyAtTheReferenceField)
{
    namespace mag = underwater_fastlio::magnetometer;
    const M3D rotation = (Eigen::AngleAxisd(0.2, V3D::UnitZ()) *
                          Eigen::AngleAxisd(0.3, V3D::UnitY()) *
                          Eigen::AngleAxisd(-0.4, V3D::UnitX())).toRotationMatrix();
    const V3D field = rotation.transpose() * V3D(1.0, 0.2, 0.4);
    const M3D calibration = V3D(1.2, 0.8, 1.1).asDiagonal();
    const auto observation = mag::evaluate(rotation, field, V3D::UnitX());
    ASSERT_TRUE(observation.valid);
    const double sensitivity = (observation.magnetic_jacobian * calibration).squaredNorm();
    const double field_variance = 0.001;
    const double heading_floor = 0.02;
    const double equivalent = field_variance + heading_floor / sensitivity;
    EXPECT_NEAR(equivalent * sensitivity, field_variance * sensitivity + heading_floor, 1e-14);
    const auto changed = mag::evaluate(rotation, field * 1.2, V3D::UnitX());
    ASSERT_TRUE(changed.valid);
    const double changed_sensitivity = (changed.magnetic_jacobian * calibration).squaredNorm();
    EXPECT_NEAR((equivalent - field_variance) * changed_sensitivity,
                heading_floor / (1.2 * 1.2), 1e-14);
}

TEST(AuxiliaryCharacterization, MissingMessagesLeaveStateAndCovarianceUnchanged)
{
    AuxiliarySensorFusion fusion;
    Ekf filter;
    InitializeFilter(filter);
    const auto state = filter.get_x();
    const auto covariance = filter.get_P();
    for (const auto kind : {AuxiliarySensorFusion::MeasurementKind::Dvl,
                            AuxiliarySensorFusion::MeasurementKind::Pressure,
                            AuxiliarySensorFusion::MeasurementKind::Magnetometer})
    {
        AuxiliarySensorFusion::TimedMeasurement measurement;
        measurement.kind = kind;
        EXPECT_FALSE(fusion.apply_timed_measurement(measurement, {}, filter));
        Ekf::vectorized_state difference = Ekf::vectorized_state::Zero();
        filter.get_x().boxminus(difference, state);
        EXPECT_EQ(difference.norm(), 0.0);
        EXPECT_TRUE(filter.get_P().isApprox(covariance, 0.0));
    }
}

TEST(PressureCharacterization, TwentyReferenceSamplesAreNotReusedAsUpdates)
{
    RosContext context;
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter("pressure.enable", true)});
        rclcpp::Node node("pressure_characterization", options);
        AuxiliarySensorFusion fusion;
        fusion.declare_parameters(node);
        fusion.load_parameters(node);
        Ekf filter;
        InitializeFilter(filter);
        fusion.initialize_pressure_reference_pose(filter.get_x());
        const auto prior = filter.get_P();
        auto message = std::make_shared<AuxiliarySensorFusion::PressureMsg>();
        message->fluid_pressure = 110000.0;
        AuxiliarySensorFusion::TimedMeasurement measurement;
        measurement.kind = AuxiliarySensorFusion::MeasurementKind::Pressure;
        measurement.pressure = message;
        for (int i = 0; i < 20; ++i)
        {
            EXPECT_FALSE(fusion.apply_timed_measurement(measurement, {}, filter));
            EXPECT_TRUE(filter.get_P().isApprox(prior, 0.0));
        }
        message->fluid_pressure += 100.0;
        EXPECT_TRUE(fusion.apply_timed_measurement(measurement, {}, filter));
        EXPECT_LT(filter.get_x().pos.z(), 0.0);
        EXPECT_EQ(filter.get_x().pos.x(), 0.0);
        EXPECT_EQ(filter.get_x().pos.y(), 0.0);
        EXPECT_TRUE(filter.get_P().allFinite());
    }
}

TEST(DvlCharacterization, ConfiguredCovarianceFloorsTightMessageCovariance)
{
    RosContext context;
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter("dvl.enable", true),
                                     rclcpp::Parameter("dvl.covariance", 0.02)});
        rclcpp::Node node("dvl_characterization", options);
        AuxiliarySensorFusion fusion;
        fusion.declare_parameters(node);
        fusion.load_parameters(node);
        Ekf first, second;
        InitializeFilter(first);
        InitializeFilter(second);
        auto message = std::make_shared<AuxiliarySensorFusion::DvlMsg>();
        message->twist.twist.linear.x = 0.3;
        AuxiliarySensorFusion::TimedMeasurement measurement;
        measurement.timestamp = 1.1;
        measurement.kind = AuxiliarySensorFusion::MeasurementKind::Dvl;
        measurement.dvl = message;
        const auto imu = StationaryInterval(1.0, 1.2).imu;
        ASSERT_TRUE(fusion.apply_timed_measurement(measurement, imu, first));
        message->twist.covariance[0] = 0.001;
        message->twist.covariance[7] = 0.001;
        message->twist.covariance[14] = 0.001;
        ASSERT_TRUE(fusion.apply_timed_measurement(measurement, imu, second));
        EXPECT_GT(first.get_x().vel.x(), 0.0);
        EXPECT_TRUE(first.get_x().vel.isApprox(second.get_x().vel, 0.0));
        EXPECT_TRUE(first.get_P().isApprox(second.get_P(), 0.0));
    }
}

TEST(AuxiliaryCharacterization, QueuesMergeByTimestampAndConsumeOnce)
{
    RosContext context;
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter("dvl.enable", true),
                                     rclcpp::Parameter("pressure.enable", true),
                                     rclcpp::Parameter("magnetometer.enable", true)});
        auto node = std::make_shared<rclcpp::Node>("queue_characterization", options);
        AuxiliarySensorFusion fusion;
        fusion.declare_parameters(*node);
        fusion.load_parameters(*node);
        fusion.create_subscriptions(*node);
        auto dvl = node->create_publisher<AuxiliarySensorFusion::DvlMsg>("/auv/dvl", rclcpp::SensorDataQoS());
        auto pressure = node->create_publisher<AuxiliarySensorFusion::PressureMsg>("/auv/pressure/scaled2", rclcpp::SensorDataQoS());
        auto mag = node->create_publisher<AuxiliarySensorFusion::MagMsg>("/auv/imu/magnetic_field", rclcpp::SensorDataQoS());
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        const auto discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while ((!dvl->get_subscription_count() || !pressure->get_subscription_count() ||
                !mag->get_subscription_count()) && std::chrono::steady_clock::now() < discovery_deadline)
        {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        ASSERT_GT(dvl->get_subscription_count(), 0u);
        ASSERT_GT(pressure->get_subscription_count(), 0u);
        ASSERT_GT(mag->get_subscription_count(), 0u);
        AuxiliarySensorFusion::DvlMsg dvl_message;
        AuxiliarySensorFusion::PressureMsg pressure_message;
        AuxiliarySensorFusion::MagMsg mag_message;
        dvl_message.header.stamp = rclcpp::Time(int64_t{1200000000});
        pressure_message.header.stamp = rclcpp::Time(int64_t{1100000000});
        pressure_message.fluid_pressure = 110000;
        mag_message.header.stamp = rclcpp::Time(int64_t{1300000000});
        mag_message.magnetic_field.x = 1.0;
        dvl->publish(dvl_message);
        pressure->publish(pressure_message);
        mag->publish(mag_message);
        const auto receive_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
        while (std::chrono::steady_clock::now() < receive_deadline)
        {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        const auto received = fusion.take_timed_measurements(1.0, 1.4);
        ASSERT_EQ(received.size(), 3u);
        // Callback arrival order does not define the merged sensor-time order.
        EXPECT_LT(received[0].timestamp, received[1].timestamp);
        EXPECT_LT(received[1].timestamp, received[2].timestamp);
        EXPECT_TRUE(fusion.take_timed_measurements(1.0, 1.4).empty());
        auto drain = [&]() {
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
            while (std::chrono::steady_clock::now() < deadline)
            {
                executor.spin_some();
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        };
        dvl->publish(dvl_message);  // Duplicate timestamp.
        pressure_message.header.stamp = rclcpp::Time(int64_t{1400000000});
        mag_message.header.stamp = rclcpp::Time(int64_t{1500000000});
        pressure->publish(pressure_message);  // Same hardware reading, new stamp.
        mag->publish(mag_message);
        drain();
        EXPECT_TRUE(fusion.take_timed_measurements(1.0, 1.6).empty());
        dvl_message.header.stamp = rclcpp::Time(int64_t{2000000000});
        dvl->publish(dvl_message);
        dvl_message.header.stamp = rclcpp::Time(int64_t{1500000000});
        dvl->publish(dvl_message);  // Regression clears the queued future sample.
        drain();
        const auto regressed = fusion.take_timed_measurements(1.4, 2.1);
        ASSERT_EQ(regressed.size(), 1u);
        EXPECT_DOUBLE_EQ(regressed.front().timestamp, 1.5);
        dvl_message.header.stamp = rclcpp::Time(int64_t{1800000000});
        dvl->publish(dvl_message);
        drain();
        EXPECT_TRUE(fusion.take_timed_measurements(1.5, 1.7).empty());
        EXPECT_EQ(fusion.take_timed_measurements(1.7, 1.9).size(), 1u);
        dvl_message.header.stamp = rclcpp::Time(int64_t{1000000000});
        dvl->publish(dvl_message);
        drain();
        EXPECT_TRUE(fusion.take_timed_measurements(1.5, 2.0).empty());
        EXPECT_EQ(fusion.take_late_measurement_counts().dvl, 1u);
        EXPECT_EQ(fusion.take_late_measurement_counts().total(), 0u);
        executor.remove_node(node);
    }
}
}  // namespace

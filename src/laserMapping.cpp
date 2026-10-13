// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: FAST-LIO contributors

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <cmath>
#include <thread>
#include <csignal>
#include <chrono>
#include <algorithm>
#include <array>
#include <atomic>
#include <deque>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <unistd.h>
#include <so3_math.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include "IMU_Processing.hpp"
#include "laser_mapping_node.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "auxiliary_sensor_fusion.hpp"
#include "loop_closure/config_profile.hpp"
#include "loop_closure/loop_closure_manager.hpp"
#include "loop_closure/state_transport.hpp"
#include "corrected_map_visualization.hpp"
#include "lidar_scan_quality.hpp"
#include "odometry_covariance.hpp"
#include "odometry_output.hpp"
#include "mapping_input_buffers.hpp"
#include "imu_observation_calendar.hpp"
#include "sonar_availability.hpp"
#include "sonar_recovery.hpp"
#include "sonar_registration.hpp"
#include <set>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include "sonar_correlated_rows.hpp"
#include "preprocess.h"
#include "sensor_parameter_utils.hpp"
#include "map_file_utils.hpp"
#include <ikd-Tree/ikd_Tree.h>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV_DEFAULT (0.001)
double LASER_POINT_COV_XY = LASER_POINT_COV_DEFAULT;
double LASER_POINT_COV_Z = LASER_POINT_COV_DEFAULT;

/*** Time Log Variables ***/
int add_point_size = 0;
/**************************/

float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;


string map_file_path, lid_topic, imu_topic, world_frame;

double gyroscope_covariance = 0.1, accelerometer_covariance = 0.1;
double gyroscope_bias_covariance = 0.0001, accelerometer_bias_covariance = 0.0001;
double init_gyroscope_bias_covariance = 0.0001, init_accelerometer_bias_covariance = 0.001;
vector<double> initial_gyroscope_bias;
double init_grav_cov = 0.00001;
double init_b_dvl_cov = 1e-8, init_b_pressure_cov = 1e4;
double imu_orientation_cov = 3.0461742e-6;  // (0.1 deg)^2
bool imu_orientation_ref_ready = false;
Eigen::Quaterniond imu_orientation_ref = Eigen::Quaterniond::Identity();
bool accel_attitude_ref_ready = false;
double accel_attitude_cov = 1.2184697e-3;  // (2 deg)^2
double accel_attitude_norm_gate = 2.0;
double filter_size_surf_min = 0, filter_size_map_min = 0;
double cube_len = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
double lidar_timeout = 0.25, sonar_frequency_hz = 4.0,
       imu_rate_hz = 100.0;
constexpr double odometry_publish_rate_hz = 100.0;
double initial_gravity_estimate = G_m_s2;
int    effct_feat_num = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0;
int    lidar_update_max_effective_features = 0;
int sonar_geometry_rank = 0;
double sonar_typical_range = 1.0;
constexpr int minimum_scan_points = 1, minimum_effective_features = 1;
uwfl2::LidarScanQualityPolicy lidar_scan_quality{1, 1};
bool   point_selected_surf[100000] = {0};
bool   flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool auxiliary_fusion_enabled = false;
bool sonar_adaptive_registration = false;
constexpr std::size_t sonar_map_point_budget = 200000;
uwfl2::SonarMapUncertainty sonar_map_uncertainty(sonar_map_point_budget);
std::unique_ptr<uwfl2::SonarRegistrationView> sonar_registration_view;
std::vector<uwfl2::SonarReferencePoint> sonar_scan_points;
std::vector<uwfl2::SonarCell> sonar_update_cells;
std::set<uwfl2::SonarCell> sonar_accepted_cells;
PointVector sonar_integration_world;
double sonar_registration_leaf = 0;
Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF> sonar_registration_prior;
std::uint64_t sonar_registration_generation = 0;
std::size_t sonar_plane_rows = 0, sonar_point_rows = 0, sonar_unresolved_plane_normals = 0;
double sonar_mean_squared_whitened_residual = 0.0;
double sonar_view_build_seconds = 0;

constexpr double AUX_SENSOR_REORDER_WINDOW_SEC = 0.02;
constexpr auto AUX_SENSOR_REORDER_WALL_GRACE = std::chrono::milliseconds(20);

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(3, 0.0);

PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));

pcl::VoxelGrid<PointType> downSizeFilterSurf;

std::shared_ptr<KD_TREE<PointType>> ikdtree =
    std::make_shared<KD_TREE<PointType>>();

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
using MainEkf = esekfom::esekf<state_ikfom, process_noise_ikfom::DOF, input_ikfom>;
MainEkf kf;
state_ikfom state_point;
vect3 pos_lid;

geometry_msgs::msg::Quaternion geoQuat;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

void SigHandle(int sig)
{
    (void)sig;
    flg_exit = true;
    rclcpp::shutdown();
}

void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree->acquire_removed_points(points_history);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    if(cub_needrm.size() > 0) ikdtree->Delete_Point_Boxes(cub_needrm);
}

double expected_imu_period()
{
    return 1.0 / (imu_rate_hz > 1.0 ? imu_rate_hz : 1.0);
}

double expected_imu_timeout()
{
    return 2.5 * expected_imu_period();
}

double imu_only_packet_duration()
{
    const double requested_period = 1.0 / odometry_publish_rate_hz;
    // Target the requested cadence and finish on the nearest real IMU sample.
    return std::max(expected_imu_period(),
                    std::min(lidar_timeout, requested_period));
}

int process_increments = 0;
std::size_t map_incremental()
{
    if (sonar_adaptive_registration)
        return static_cast<std::size_t>(ikdtree->Add_Points(sonar_integration_world, true));
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        if (sonar_adaptive_registration && !point_selected_surf[i]) continue;
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    add_point_size = ikdtree->Add_Points(PointToAdd, true);
    ikdtree->Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    return static_cast<std::size_t>(add_point_size);
}

bool rebuild_sonar_registration_view()
{
    const auto started = std::chrono::steady_clock::now();
    PointVector active_points;
    if (!ikdtree->Snapshot_Points(active_points, sonar_map_point_budget)) return false;
    std::vector<Eigen::Vector3d> positions;
    positions.reserve(active_points.size());
    for (const auto &point : active_points) positions.emplace_back(point.x, point.y, point.z);
    const auto references = sonar_map_uncertainty.Select(positions);
    if (!references) return false;
    const bool valid = sonar_registration_view->Replace(++sonar_registration_generation, *references);
    sonar_view_build_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    return valid;
}

bool record_sonar_scan_uncertainty()
{
    if (sonar_update_cells.size() != sonar_scan_points.size()) return false;
    sonar_accepted_cells.clear();
    sonar_integration_world.clear();
    for (std::size_t i = 0; i < sonar_update_cells.size(); ++i)
        if (point_selected_surf[i]) sonar_accepted_cells.insert(sonar_update_cells[i]);
    std::vector<uwfl2::SonarReferencePoint> sources;
    sources.reserve(sonar_scan_points.size());
    const M3D rotation = state_point.rot.toRotationMatrix() * state_point.offset_R_L_I.toRotationMatrix();
    for (std::size_t i = 0; i < sonar_scan_points.size(); ++i)
    {
        if (!point_selected_surf[i]) continue;
        const auto &source = sonar_scan_points[i];
        const V3D body = state_point.offset_R_L_I * source.position + state_point.offset_T_L_I;
        const V3D world = state_point.rot * body + state_point.pos;
        PointType point{};
        point.x = world.x(); point.y = world.y(); point.z = world.z();
        sonar_integration_world.push_back(point);
        sources.push_back({V3D(point.x, point.y, point.z), rotation * source.covariance * rotation.transpose()});
    }
    return !sources.empty() && sonar_map_uncertainty.Record(sources);
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI());
std::mutex mapping_output_mutex;

std::string save_to_pcd(const PointCloudXYZI &map)
{
    const auto path = uwfl2::reserve_map_path(map_file_path);
    try
    {
        pcl::PCDWriter pcd_writer;
        if (pcd_writer.writeBinary(path.string(), map) != 0)
            throw std::runtime_error("PCD writer failed: " + path.string());
    }
    catch (...)
    {
        std::error_code error;
        std::filesystem::remove(path, error);
        throw;
    }
    return path.string();
}

void update_state_outputs()
{
    state_point = kf.get_x();
    euler_cur = SO3ToEuler(state_point.rot);
    pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
    geoQuat.x = state_point.rot.coeffs()[0];
    geoQuat.y = state_point.rot.coeffs()[1];
    geoQuat.z = state_point.rot.coeffs()[2];
    geoQuat.w = state_point.rot.coeffs()[3];
}

bool apply_imu_orientation_update(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg,
                                  MainEkf &filter)
{
    if (!imu_msg || imu_msg->orientation_covariance[0] < 0.0)
    {
        return false;
    }

    const auto &q_msg = imu_msg->orientation;
    Eigen::Quaterniond q_meas(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
    if (!std::isfinite(q_meas.w()) || !std::isfinite(q_meas.x()) ||
        !std::isfinite(q_meas.y()) || !std::isfinite(q_meas.z()) ||
        q_meas.norm() < 1e-6)
    {
        return false;
    }
    q_meas.normalize();
    if (!imu_orientation_ref_ready)
    {
        imu_orientation_ref = q_meas;
        imu_orientation_ref_ready = true;
        return false;
    }

    Eigen::Quaterniond q_relative = imu_orientation_ref.conjugate() * q_meas;
    q_relative.normalize();
    state_ikfom state = filter.get_x();
    const M3D R_err =
        state.rot.toRotationMatrix().transpose() * q_relative.toRotationMatrix();
    const V3D residual = Log(R_err);

    MainEkf::cov P = filter.get_P();
    Eigen::Matrix<double, 3, state_ikfom::DOF> H =
        Eigen::Matrix<double, 3, state_ikfom::DOF>::Zero();
    H.block<3, 3>(0, 3).setIdentity();
    const M3D R = M3D::Identity() * std::max(1e-12, imu_orientation_cov);
    const M3D S = H * P * H.transpose() + R;
    Eigen::LDLT<M3D> ldlt(S);
    if (ldlt.info() != Eigen::Success)
    {
        return false;
    }
    const Eigen::Matrix<double, state_ikfom::DOF, 3> K =
        P * H.transpose() * ldlt.solve(M3D::Identity());
    const Eigen::Matrix<double, state_ikfom::DOF, 1> dx = K * residual;
    if (!dx.allFinite())
    {
        return false;
    }
    const auto prior_state = state;
    state.boxplus(dx);

    const MainEkf::cov I = MainEkf::cov::Identity();
    const MainEkf::cov KH = K * H;
    MainEkf::cov P_new =
        ((I - KH) * P * (I - KH).transpose() + K * R * K.transpose()).eval();
    P_new = ((P_new + P_new.transpose()) * 0.5).eval();
    P_new = AuxiliarySensorFusion::transport_covariance_after_boxplus(
        P_new, prior_state, state, dx);
    filter.change_x(state);
    filter.change_P(P_new);
    return true;
}


bool apply_accel_attitude_update(
    const std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> &imu_msgs,
    MainEkf &filter)
{
    if (imu_msgs.empty()) return false;
    V3D acc_meas = V3D::Zero();
    for (const auto &sample : imu_msgs)
    {
        acc_meas += V3D(sample->linear_acceleration.x,
                        sample->linear_acceleration.y,
                        sample->linear_acceleration.z);
    }
    acc_meas /= static_cast<double>(imu_msgs.size());

    state_ikfom state = filter.get_x();
    acc_meas -= V3D(state.ba[0], state.ba[1], state.ba[2]);
    const double acc_norm = acc_meas.norm();
    if (!std::isfinite(acc_norm) || acc_norm < 1e-6 ||
        std::abs(acc_norm - initial_gravity_estimate) > accel_attitude_norm_gate)
    {
        return false;
    }
    const bool initialize_from_filter_gravity = !accel_attitude_ref_ready;
    accel_attitude_ref_ready = true;
    V3D grav_local(state.grav[0], state.grav[1], state.grav[2]);
    if (grav_local.norm() < 1e-6)
    {
        grav_local = V3D(0.0, 0.0, -initial_gravity_estimate);
    }
    const V3D predicted =
        state.rot.toRotationMatrix().transpose() * (-grav_local.normalized());
    // IMU initialization has already estimated gravity from the stationary
    // startup window. Condition the attitude covariance on that same prior
    // once, without allowing the first post-init (possibly moving) sample to
    // rotate the state. Later calls use the current measured acceleration.
    const V3D measured = initialize_from_filter_gravity
        ? predicted
        : acc_meas / acc_norm;
    if (!measured.allFinite() || !predicted.allFinite())
    {
        return false;
    }

    Eigen::Quaterniond predicted_to_measured =
        Eigen::Quaterniond::FromTwoVectors(predicted, measured);
    predicted_to_measured.normalize();
    V3D residual = -Log(predicted_to_measured.toRotationMatrix());
    residual -= predicted * residual.dot(predicted);
    if (!residual.allFinite())
    {
        return false;
    }

    MainEkf::cov P = filter.get_P();
    Eigen::Matrix<double, 3, state_ikfom::DOF> H =
        Eigen::Matrix<double, 3, state_ikfom::DOF>::Zero();
    // A normalized accelerometer constrains only tilt.  Using an identity
    // attitude block would also condition the unobserved rotation about
    // gravity as if its residual were zero, making heading spuriously
    // overconfident.  The tangent-plane projector keeps that yaw gauge free.
    const M3D tilt_projector =
        M3D::Identity() - predicted * predicted.transpose();
    H.block<3, 3>(0, 3) = tilt_projector;
    const M3D R = M3D::Identity() * std::max(1e-8, accel_attitude_cov);
    const M3D S = H * P * H.transpose() + R;
    Eigen::LDLT<M3D> ldlt(S);
    if (ldlt.info() != Eigen::Success)
    {
        return false;
    }
    const Eigen::Matrix<double, state_ikfom::DOF, 3> K =
        P * H.transpose() * ldlt.solve(M3D::Identity());
    const Eigen::Matrix<double, state_ikfom::DOF, 1> dx = K * residual;
    if (!dx.allFinite())
    {
        return false;
    }
    const auto prior_state = state;
    state.boxplus(dx);

    const MainEkf::cov I = MainEkf::cov::Identity();
    const MainEkf::cov KH = K * H;
    MainEkf::cov P_new =
        ((I - KH) * P * (I - KH).transpose() + K * R * K.transpose()).eval();
    P_new = ((P_new + P_new.transpose()) * 0.5).eval();
    P_new = AuxiliarySensorFusion::transport_covariance_after_boxplus(
        P_new, prior_state, state, dx);
    filter.change_x(state);
    filter.change_P(P_new);
    return true;
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br)
{
    const auto odometry = uwfl2::make_odometry(
        state_point, kf.get_P(), get_ros_time(lidar_end_time), geoQuat);
    pubOdomAftMapped->publish(odometry);
    tf_br->sendTransform(uwfl2::make_odometry_transform(odometry));
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    if (sonar_adaptive_registration)
    {
        corr_normvect->resize(feats_down_size);
        std::vector<uwfl2::SonarCorrelatedRow> rows;
        sonar_plane_rows = sonar_point_rows = sonar_unresolved_plane_normals = 0;
        const M3D rotation = s.rot.toRotationMatrix();
        const M3D sonar_rotation = rotation * s.offset_R_L_I.toRotationMatrix();
        uwfl2::SonarCorrespondenceConfig config{2.0, 0.1, 3.0, 12, 1.0};
        config.registration_covariance = V3D(LASER_POINT_COV_XY, LASER_POINT_COV_XY, LASER_POINT_COV_Z).asDiagonal();
        std::vector<std::optional<uwfl2::SonarCorrelatedRow>> matches(feats_down_size);
        std::vector<unsigned char> plane_match(feats_down_size, 0);
        std::vector<unsigned char> unresolved_normal(feats_down_size, 0);
        #ifdef MP_EN
            omp_set_num_threads(MP_PROC_NUM);
            #pragma omp parallel for
        #endif
        for (int i = 0; i < feats_down_size; ++i)
        {
            point_selected_surf[i] = false;
            const auto &source = sonar_scan_points[i];
            const V3D body = s.offset_R_L_I * source.position + s.offset_T_L_I;
            Eigen::Matrix<double, 3, state_ikfom::DOF> derivative = Eigen::Matrix<double, 3, state_ikfom::DOF>::Zero();
            derivative.leftCols<3>() = M3D::Identity();
            M3D skew; skew << SKEW_SYM_MATRX(body);
            derivative.middleCols<3>(3) = -rotation * skew;
            const M3D predicted_point_covariance = derivative * sonar_registration_prior * derivative.transpose();
            auto match = uwfl2::MatchSonarPoint(*sonar_registration_view, sonar_registration_generation,
                config, body, rotation, s.pos, sonar_rotation * source.covariance * sonar_rotation.transpose(), predicted_point_covariance);
            unresolved_normal[i] = match.unresolved_plane_normal;
            Nearest_Points[i].clear();
            for (const auto &neighbor : match.search.neighbors)
            {
                PointType point;
                point.x = neighbor.position.x(); point.y = neighbor.position.y(); point.z = neighbor.position.z();
                Nearest_Points[i].push_back(point);
            }
            if (!match.row) continue;
            auto row = *match.row;
            point_selected_surf[i] = true;
            uwfl2::SonarCorrelatedRow correlated;
            correlated.measurement = row;
            correlated.cached_plane = match.cached_plane;
            if (match.cached_plane)
            {
                const auto &plane = *match.cached_plane;
                correlated.plane_noise_jacobian << (rotation * body + s.pos - plane.center).transpose(),
                    -plane.normal.transpose();
            }
            matches[i] = correlated;
            plane_match[i] = match.is_plane;
        }
        for (int i = 0; i < feats_down_size; ++i)
        {
            if (unresolved_normal[i]) ++sonar_unresolved_plane_normals;
            if (!matches[i]) continue;
            rows.push_back(*matches[i]);
            corr_normvect->points[rows.size() - 1].intensity = matches[i]->measurement.residual;
            if (plane_match[i]) ++sonar_plane_rows; else ++sonar_point_rows;
        }
        effct_feat_num = rows.size();
        corr_normvect->resize(rows.size());
        lidar_update_max_effective_features = std::max(lidar_update_max_effective_features, effct_feat_num);
        if (rows.empty()) { sonar_geometry_rank = 0; ekfom_data.valid = false; return; }
        const auto system = uwfl2::WhitenSonarRows(rows);
        if (!system.valid) { sonar_geometry_rank = 0; ekfom_data.valid = false; return; }
        ekfom_data.h_x = system.jacobian;
        ekfom_data.h = system.innovation;
        sonar_geometry_rank = uwfl2::SonarGeometryRank(ekfom_data.h_x, sonar_typical_range);
        sonar_mean_squared_whitened_residual = ekfom_data.h.squaredNorm() / rows.size();
        return;
    }

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree->Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
            }
        }
    }
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            effct_feat_num ++;
        }
    }
    lidar_update_max_effective_features =
        std::max(lidar_update_max_effective_features, effct_feat_num);
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    if (!lidar_scan_quality.features_are_sufficient(
            static_cast<std::size_t>(effct_feat_num)))
    {
        sonar_geometry_rank = 0;
        ekfom_data.valid = false;
        return;
    }
    // Auxiliary measurements are already applied at their own timestamps.
    // Keep the original FAST-LIO2 scan-to-map update layout here.
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);
    ekfom_data.h = Eigen::VectorXd::Zero(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;

        if (auxiliary_fusion_enabled)
        {
            // The underwater path supports separate horizontal and vertical
            // scan covariance by whitening each point-to-plane residual.
            const double nz2 = std::clamp(
                static_cast<double>(norm_p.z) * static_cast<double>(norm_p.z),
                0.0, 1.0);
            const double point_cov = std::max(
                1e-12, LASER_POINT_COV_XY * (1.0 - nz2) +
                           LASER_POINT_COV_Z * nz2);
            const double inv_sigma = 1.0 / std::sqrt(point_cov);
            ekfom_data.h_x.block<1, 12>(i, 0) *= inv_sigma;
            ekfom_data.h(i) *= inv_sigma;
        }
    }

    sonar_geometry_rank = uwfl2::SonarGeometryRank(ekfom_data.h_x, sonar_typical_range);

}

class LaserMappingNode : public rclcpp::Node
{
public:
    LaserMappingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("laser_mapping", options)
    {
        DeclareParameters();
        LoadParameters();
        sonar_availability_.SetFrequency(sonar_frequency_hz);
        InitializeEstimator();
        InitializeLoopClosure();

        /*** ROS subscribe initialization ***/
        sensor_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        lidar_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        processing_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        backend_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        prediction_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sensor_options;
        sensor_options.callback_group = sensor_callback_group_;
        rclcpp::SubscriptionOptions lidar_options;
        lidar_options.callback_group = lidar_callback_group_;

        auto tf_qos = rclcpp::QoS(rclcpp::KeepLast(1000));
        tf_qos.best_effort();
        initial_pose_tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", tf_qos,
            [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
                initial_pose_tf_callback(*message);
            },
            sensor_options);

        if (lid_topic.empty())
        {
            RCLCPP_WARN(this->get_logger(),
                        "sonar.topic is empty. Sonar subscription is disabled; node will run IMU-only odometry.");
        }
        else
        {
            auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(200000));
            // Best-effort subscriptions match both best-effort and reliable
            // sensor publishers and are supported by ROS 2 Humble and newer.
            lidar_qos.best_effort();
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                lid_topic, lidar_qos,
                [this](sensor_msgs::msg::PointCloud2::UniquePtr message) {
                    input_buffers_.NoteSonarReceipt();
                    ensure_sonar_frame_attached_to_body(message->header.frame_id);
                    standard_pcl_cbk(std::move(message));
                },
                lidar_options);
        }
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::QoS(rclcpp::KeepLast(200000)),
            [this](sensor_msgs::msg::Imu::UniquePtr message) {
                imu_cbk(std::move(message));
            }, sensor_options);
        aux_fusion_.create_subscriptions(*this, sensor_callback_group_);
        if (corrected_map_publish_enabled_)
        {
            pubCorrectedMap_ =
                this->create_publisher<visualization_msgs::msg::MarkerArray>(
                    corrected_map_topic_,
                    rclcpp::QoS(1).transient_local().reliable());
        }
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 20);
        navigation_health_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/uwfl2/navigation_health", 1);
        sonar_diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/uwfl2/sonar_diagnostics", rclcpp::QoS(200));
        if (sonar_adaptive_registration)
            provisional_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/uwfl2/provisional_map", rclcpp::QoS(1).transient_local());
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        //------------------------------------------------------------------------------------------------------
        // Drive processing from wall time so rosbag replay can drain buffered
        // messages even after simulated /clock stops publishing. Sensor callbacks
        // are isolated in their own callback group and only fill mutex-protected
        // buffers; the EKF state remains owned by this processing callback.
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1),
                                         std::bind(&LaserMappingNode::timer_callback, this),
                                         processing_callback_group_);
        navigation_health_timer_ = this->create_wall_timer(std::chrono::seconds(1),
            std::bind(&LaserMappingNode::publish_navigation_health, this),
            processing_callback_group_);
        if (pubCorrectedMap_)
        {
            corrected_map_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(20),
                std::bind(&LaserMappingNode::maybe_publish_corrected_map, this),
                backend_callback_group_);
        }
        const auto prediction_timer_period =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(
                    1.0 / odometry_publish_rate_hz));
        prediction_timer_ = this->create_wall_timer(
            prediction_timer_period,
            std::bind(&LaserMappingNode::publish_high_rate_odometry_prediction,
                      this),
            prediction_callback_group_);
        map_save_srv_ = this->create_service<std_srvs::srv::Trigger>("map_save", std::bind(&LaserMappingNode::map_save_callback, this, std::placeholders::_1, std::placeholders::_2));
        if (loop_closure_)
        {
            if (loop_visualization_enabled_)
            {
                const auto visualization_qos =
                    rclcpp::QoS(2).transient_local().reliable();
                raw_graph_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
                    "/uwfl2_lc/raw_path", visualization_qos);
                optimized_graph_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
                    "/uwfl2_lc/optimized_path", visualization_qos);
                loop_markers_pub_ =
                    this->create_publisher<visualization_msgs::msg::MarkerArray>(
                        "/uwfl2_lc/markers", visualization_qos);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Node init finished.");
    }

private:

    static std::string normalized_frame_id(std::string frame)
    {
        while (!frame.empty() && frame.front() == '/') frame.erase(frame.begin());
        return frame;
    }

    void note_initial_pose_input_locked(double acquisition_time)
    {
        const auto now = std::chrono::steady_clock::now();
        if (!initial_pose_wait_started_)
        {
            initial_pose_wait_started_ = true;
            initial_pose_wait_start_wall_ = now;
            initial_pose_first_sensor_time_ = acquisition_time;
            initial_pose_latest_sensor_time_ = acquisition_time;
        }
        else if (std::isfinite(acquisition_time))
        {
            if (!std::isfinite(initial_pose_first_sensor_time_))
                initial_pose_first_sensor_time_ = acquisition_time;
            initial_pose_latest_sensor_time_ = std::max(
                initial_pose_latest_sensor_time_, acquisition_time);
        }
    }

    void note_initial_pose_input(double acquisition_time)
    {
        std::lock_guard<std::mutex> lock(initial_pose_mutex_);
        note_initial_pose_input_locked(acquisition_time);
    }

    void finalize_initial_world_pose_locked(
        const V3D &translation, const Eigen::Quaterniond &orientation,
        bool fallback)
    {
        Eigen::Quaterniond normalized = orientation;
        normalized.normalize();

        geometry_msgs::msg::TransformStamped world_to_camera_init;
        world_to_camera_init.header.stamp = this->get_clock()->now();
        world_to_camera_init.header.frame_id = world_frame;
        world_to_camera_init.child_frame_id = "camera_init";
        world_to_camera_init.transform.translation.x = translation.x();
        world_to_camera_init.transform.translation.y = translation.y();
        world_to_camera_init.transform.translation.z = translation.z();
        world_to_camera_init.transform.rotation.x = normalized.x();
        world_to_camera_init.transform.rotation.y = normalized.y();
        world_to_camera_init.transform.rotation.z = normalized.z();
        world_to_camera_init.transform.rotation.w = normalized.w();
        static_tf_broadcaster_->sendTransform(world_to_camera_init);

        // Pressure is evaluated in World vertical, so it needs the same
        // camera_init pose selected for the static transform.
        aux_fusion_.set_camera_init_pose_in_world(
            translation, normalized.toRotationMatrix());
        initial_world_pose_ready_ = true;

        if (fallback)
        {
            RCLCPP_WARN(this->get_logger(),
                        "No reliable 20-distinct-sample '%s' -> 'base_link' TF "
                        "initialization was available. Using an identity "
                        "World -> camera_init pose.",
                        world_frame.c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),
                        "Initialized World -> camera_init from %zu '%s' -> "
                        "'base_link' TF samples: translation [%.6f, %.6f, %.6f], "
                        "quaternion [%.6f, %.6f, %.6f, %.6f].",
                        initial_pose_sample_count_, world_frame.c_str(),
                        translation.x(), translation.y(), translation.z(),
                        normalized.x(), normalized.y(), normalized.z(), normalized.w());
        }
    }

    void initial_pose_tf_callback(const tf2_msgs::msg::TFMessage &message)
    {
        std::lock_guard<std::mutex> lock(initial_pose_mutex_);
        if (initial_world_pose_ready_) return;

        for (const auto &transform : message.transforms)
        {
            const double stamp = get_time_sec(transform.header.stamp);
            note_initial_pose_input_locked(stamp);
            if (normalized_frame_id(transform.header.frame_id) !=
                    normalized_frame_id(world_frame) ||
                normalized_frame_id(transform.child_frame_id) != "base_link")
                continue;

            const auto &p = transform.transform.translation;
            const auto &r = transform.transform.rotation;
            const V3D translation(p.x, p.y, p.z);
            Eigen::Quaterniond orientation(r.w, r.x, r.y, r.z);
            if (!translation.allFinite() || !orientation.coeffs().allFinite() ||
                orientation.norm() <= 1e-12)
                continue;

            orientation.normalize();
            ++initial_pose_raw_sample_count_;

            if (!initial_pose_samples_.empty())
            {
                const auto &last = initial_pose_samples_.back();
                const V3D last_translation(last[0], last[1], last[2]);
                Eigen::Quaterniond last_orientation(
                    last[6], last[3], last[4], last[5]);
                last_orientation.normalize();
                const double angular_distance = 2.0 * std::acos(std::clamp(
                    std::abs(last_orientation.dot(orientation)), 0.0, 1.0));
                if ((translation - last_translation).norm() <=
                        kInitialPoseDuplicateTranslation &&
                    angular_distance <= kInitialPoseDuplicateAngle)
                {
                    ++initial_pose_repeated_sample_count_;
                    continue;
                }
            }

            initial_pose_samples_.push_back({
                translation.x(), translation.y(), translation.z(),
                orientation.x(), orientation.y(), orientation.z(), orientation.w()});
            if (initial_pose_samples_.size() < kInitialPoseSampleCount) continue;

            const std::size_t sample_count = initial_pose_samples_.size();
            std::vector<std::size_t> best_inliers;
            double best_cost = std::numeric_limits<double>::infinity();
            for (std::size_t candidate = 0; candidate < sample_count; ++candidate)
            {
                const auto &candidate_sample = initial_pose_samples_[candidate];
                const V3D candidate_translation(
                    candidate_sample[0], candidate_sample[1], candidate_sample[2]);
                const Eigen::Quaterniond candidate_orientation(
                    candidate_sample[6], candidate_sample[3],
                    candidate_sample[4], candidate_sample[5]);
                std::vector<std::size_t> inliers;
                double cost = 0.0;
                for (std::size_t index = 0; index < sample_count; ++index)
                {
                    const auto &sample = initial_pose_samples_[index];
                    const V3D sample_translation(sample[0], sample[1], sample[2]);
                    const Eigen::Quaterniond sample_orientation(
                        sample[6], sample[3], sample[4], sample[5]);
                    const double translation_distance =
                        (sample_translation - candidate_translation).norm();
                    const double angular_distance = 2.0 * std::acos(std::clamp(
                        std::abs(candidate_orientation.dot(sample_orientation)),
                        0.0, 1.0));
                    if (translation_distance <= kInitialPoseInlierTranslation &&
                        angular_distance <= kInitialPoseInlierAngle)
                    {
                        inliers.push_back(index);
                        cost += translation_distance / kInitialPoseInlierTranslation +
                                angular_distance / kInitialPoseInlierAngle;
                    }
                }
                if (inliers.size() > best_inliers.size() ||
                    (inliers.size() == best_inliers.size() && cost < best_cost))
                {
                    best_inliers = std::move(inliers);
                    best_cost = cost;
                }
            }

            const std::size_t minimum_consensus =
                (7 * sample_count + 9) / 10;  // ceil(70 percent)
            if (best_inliers.size() < minimum_consensus) continue;

            V3D robust_translation = V3D::Zero();
            Eigen::Matrix4d quaternion_outer = Eigen::Matrix4d::Zero();
            for (const std::size_t index : best_inliers)
            {
                const auto &sample = initial_pose_samples_[index];
                robust_translation += V3D(sample[0], sample[1], sample[2]);
                const Eigen::Vector4d quaternion(
                    sample[3], sample[4], sample[5], sample[6]);
                quaternion_outer += quaternion * quaternion.transpose();
            }
            robust_translation /= static_cast<double>(best_inliers.size());

            // Weiszfeld iterations form a true 3-D geometric median instead
            // of independently mixing x/y/z component medians.
            for (int iteration = 0; iteration < 20; ++iteration)
            {
                V3D weighted_sum = V3D::Zero();
                double weight_sum = 0.0;
                for (const std::size_t index : best_inliers)
                {
                    const auto &sample = initial_pose_samples_[index];
                    const V3D point(sample[0], sample[1], sample[2]);
                    const double distance = (point - robust_translation).norm();
                    const double weight = 1.0 / std::max(distance, 1e-6);
                    weighted_sum += weight * point;
                    weight_sum += weight;
                }
                const V3D next = weighted_sum / weight_sum;
                if ((next - robust_translation).norm() < 1e-9) break;
                robust_translation = next;
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(
                quaternion_outer);
            if (solver.info() != Eigen::Success) continue;
            Eigen::Quaterniond mean_orientation;
            mean_orientation.coeffs() = solver.eigenvectors().col(3);
            initial_pose_sample_count_ = best_inliers.size();
            RCLCPP_INFO(this->get_logger(),
                        "Initial TF filtering: %zu raw, %zu repeated, %zu "
                        "distinct, %zu consensus inliers.",
                        initial_pose_raw_sample_count_,
                        initial_pose_repeated_sample_count_, sample_count,
                        best_inliers.size());
            finalize_initial_world_pose_locked(
                robust_translation, mean_orientation, false);
            return;
        }
    }

    bool ensure_initial_world_pose()
    {
        std::lock_guard<std::mutex> lock(initial_pose_mutex_);
        if (initial_world_pose_ready_) return true;
        if (!initial_pose_wait_started_) return false;

        const double sensor_elapsed =
            std::isfinite(initial_pose_first_sensor_time_) &&
                    std::isfinite(initial_pose_latest_sensor_time_)
                ? initial_pose_latest_sensor_time_ - initial_pose_first_sensor_time_
                : 0.0;
        const double wall_elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - initial_pose_wait_start_wall_).count();
        if (sensor_elapsed < kInitialPoseWaitSeconds &&
            wall_elapsed < kInitialPoseWaitSeconds)
            return false;

        finalize_initial_world_pose_locked(
            V3D::Zero(), Eigen::Quaterniond::Identity(), true);
        return true;
    }

    void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
    {
        const double cur_time = get_time_sec(msg->header.stamp);
        note_initial_pose_input(cur_time);
        PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
        // Conversion remains outside the shared input lock.
        p_pre->process(msg, ptr);
        if (!input_buffers_.PushSonar(ptr, cur_time))
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Dropped sonar scan with invalid, duplicate, or out-of-order "
                "acquisition timestamp %.9f.",
                cur_time);
        }
    }

    void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
    {
        note_initial_pose_input(get_time_sec(msg_in->header.stamp));
        sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
        input_buffers_.PushImu(msg);
    }

    bool auxiliary_callbacks_ready(double target_time, double latest_imu_time)
    {
        if (!auxiliary_fusion_enabled)
        {
            aux_reorder_waiting = false;
            aux_reorder_target_time = -1.0;
            return true;
        }

        const auto now = std::chrono::steady_clock::now();
        if (!aux_reorder_waiting ||
            std::abs(aux_reorder_target_time - target_time) > 1e-6)
        {
            aux_reorder_waiting = true;
            aux_reorder_target_time = target_time;
            aux_reorder_wait_start = now;
        }

        const bool sensor_time_ready =
            latest_imu_time >= target_time + AUX_SENSOR_REORDER_WINDOW_SEC;
        const bool wall_time_ready =
            now - aux_reorder_wait_start >= AUX_SENSOR_REORDER_WALL_GRACE;
        if (!sensor_time_ready && !wall_time_ready)
        {
            return false;
        }

        aux_reorder_waiting = false;
        aux_reorder_target_time = -1.0;
        return true;
    }

    enum class ImuPacketResult { Unavailable, Waiting, Ready };

    bool sync_packages(MeasureGroup &meas, bool &imu_only_measure)
    {
        while (!input_buffers_.lidar_pushed && !input_buffers_.lidar_buffer.empty() && input_buffers_.last_processed_time > 0.0 &&
               input_buffers_.time_buffer.front() < input_buffers_.last_processed_time - 1e-4)
        {
            ++input_buffers_.sonar_late_dropped;
            input_buffers_.max_sonar_late_age = std::max(
                input_buffers_.max_sonar_late_age,
                input_buffers_.last_processed_time - input_buffers_.time_buffer.front());
            input_buffers_.lidar_buffer.pop_front();
            input_buffers_.time_buffer.pop_front();
        }

        if (input_buffers_.lidar_buffer.empty() || input_buffers_.imu_buffer.empty()) {
            return false;
        }

        // A future scan must not batch an outage into one long propagation
        // packet. Navigate up to its start, retaining its full deskew interval.
        if (input_buffers_.NavigationPacketPrecedesSonar(
                imu_only_packet_duration(), expected_imu_period(), expected_imu_timeout(), lidar_timeout))
            return false;

        // A queued scan owns its acquisition interval. Process() already splits
        // propagation at each auxiliary epoch; outer packet splitting changes
        // initialization and discards motion history needed for scan deskew.
        /*** push a lidar scan ***/
        if(!input_buffers_.lidar_pushed)
        {
            meas.lidar = input_buffers_.lidar_buffer.front();
            meas.lidar_beg_time = input_buffers_.time_buffer.front();
            if (meas.lidar->points.size() <= 1) // time too little
            {
                lidar_end_time = meas.lidar_beg_time + input_buffers_.lidar_mean_scantime;
            }
            else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * input_buffers_.lidar_mean_scantime)
            {
                lidar_end_time = meas.lidar_beg_time + input_buffers_.lidar_mean_scantime;
            }
            else
            {
                input_buffers_.scan_num ++;
                lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
                input_buffers_.lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - input_buffers_.lidar_mean_scantime) / input_buffers_.scan_num;
            }

            meas.lidar_end_time = lidar_end_time;

            input_buffers_.lidar_pushed = true;
        }

        if (input_buffers_.last_timestamp_imu < lidar_end_time)
        {
            return false;
        }
        if (!auxiliary_callbacks_ready(lidar_end_time, input_buffers_.last_timestamp_imu))
        {
            return false;
        }

        /*** push imu data, and pop from imu buffer ***/
        double imu_time = get_time_sec(input_buffers_.imu_buffer.front()->header.stamp);
        meas.imu.clear();
        while ((!input_buffers_.imu_buffer.empty()) && (imu_time < lidar_end_time))
        {
            imu_time = get_time_sec(input_buffers_.imu_buffer.front()->header.stamp);
            if(imu_time > lidar_end_time) break;
            meas.imu.push_back(input_buffers_.imu_buffer.front());
            input_buffers_.imu_buffer.pop_front();
        }

        // Retain the next real sample in its input queue, but expose the pair
        // bracketing a fractional scan endpoint to propagation and DVL.
        if (!input_buffers_.imu_buffer.empty() &&
            get_time_sec(input_buffers_.imu_buffer.front()->header.stamp) > lidar_end_time)
            meas.imu.push_back(input_buffers_.imu_buffer.front());

        input_buffers_.lidar_buffer.pop_front();
        input_buffers_.time_buffer.pop_front();
        input_buffers_.lidar_pushed = false;
        input_buffers_.last_scan_end_time = meas.lidar_end_time;
        return true;
    }

    ImuPacketResult sync_imu_only_packages(MeasureGroup &meas)
    {
        const bool pending_scan = !input_buffers_.lidar_buffer.empty();
        if ((pending_scan && !input_buffers_.NavigationPacketPrecedesSonar(
                imu_only_packet_duration(), expected_imu_period(), expected_imu_timeout(), lidar_timeout)) ||
            input_buffers_.lidar_pushed || input_buffers_.sonar_processing ||
            input_buffers_.imu_buffer.empty()) {
            return ImuPacketResult::Unavailable;
        }

        const double latest_imu_time = get_time_sec(input_buffers_.imu_buffer.back()->header.stamp);
        const double first_imu_time = get_time_sec(input_buffers_.imu_buffer.front()->header.stamp);
        double packet_begin_time = input_buffers_.last_processed_time > 0.0 ? input_buffers_.last_processed_time : first_imu_time;
        if (input_buffers_.last_processed_time > 0.0 && first_imu_time > packet_begin_time + expected_imu_timeout())
        {
            packet_begin_time = first_imu_time;
        }
        const double target_packet_end_time = packet_begin_time + imu_only_packet_duration();
        const double propagation_horizon =
            input_buffers_.PropagationWatermark(latest_imu_time);
        if (propagation_horizon < target_packet_end_time - 1e-6)
        {
            return ImuPacketResult::Unavailable;
        }
        // Select a complete packet before popping anything. A partial packet
        // would change the next correction epoch with callback timing.
        const auto packet_tail = std::find_if(
            input_buffers_.imu_buffer.begin(), input_buffers_.imu_buffer.end(),
            [&](const auto &imu) {
                return get_time_sec(imu->header.stamp) >=
                    target_packet_end_time - 0.5 * expected_imu_period();
            });
        if (packet_tail == input_buffers_.imu_buffer.end())
        {
            return ImuPacketResult::Unavailable;
        }
        const double packet_tail_time = get_time_sec((*packet_tail)->header.stamp);
        if (packet_tail_time > propagation_horizon + 1e-6)
        {
            return ImuPacketResult::Unavailable;
        }
        if (!auxiliary_callbacks_ready(target_packet_end_time, latest_imu_time))
        {
            return ImuPacketResult::Waiting;
        }

        meas.imu.clear();
        meas.lidar.reset(new PointCloudXYZI());
        meas.lidar_beg_time = packet_begin_time;
        double last_included_imu_time = packet_begin_time;

        while (!input_buffers_.imu_buffer.empty())
        {
            const double imu_time = get_time_sec(input_buffers_.imu_buffer.front()->header.stamp);
            // A near-future sample is still a real acquisition. A microsecond
            // duplicate tolerance can erase an IMU observation after sonar.
            if (input_buffers_.last_processed_time > 0.0 &&
                imu_time <= input_buffers_.last_processed_time + uwfl2::kAcquisitionTimeTolerance)
            {
                input_buffers_.imu_buffer.pop_front();
                continue;
            }
            meas.imu.push_back(input_buffers_.imu_buffer.front());
            input_buffers_.imu_buffer.pop_front();
            last_included_imu_time = imu_time;
            // Select the real IMU sample nearest the requested output epoch.
            // Without the half-sample tolerance, a 200.078 Hz stream misses the
            // exact 10 ms boundary by a few microseconds and is decimated by
            // three samples (about 66.7 Hz) instead of two (about 100 Hz).
            if (imu_time >= target_packet_end_time - 0.5 * expected_imu_period()) break;
        }

        if (meas.imu.empty())
        {
            return ImuPacketResult::Unavailable;
        }
        meas.lidar_end_time = last_included_imu_time;
        lidar_end_time = meas.lidar_end_time;
        return ImuPacketResult::Ready;
    }

    void DeclareParameters()
    {
        this->declare_parameter<bool>("publish.corrected_map_enable", false);
        this->declare_parameter<string>("publish.corrected_map_topic",
                                        "/uwfl2/corrected_map");

        this->declare_parameter<string>("sonar.topic", "/points_raw");
        this->declare_parameter<double>("sonar.frequency", 4.0);
        this->declare_parameter<vector<double>>("sonar.translation", {0.0, 0.0, 0.0});
        this->declare_parameter<vector<double>>("sonar.rotation", {0.0, 0.0, 0.0});
        this->declare_parameter<float>("sonar.max_range", 300.0F);
        this->declare_parameter<double>("sonar.min_range", 0.01);
        this->declare_parameter<double>("sonar.xy_covariance", LASER_POINT_COV_DEFAULT);
        this->declare_parameter<double>("sonar.z_covariance", LASER_POINT_COV_DEFAULT);
        this->declare_parameter<bool>("sonar.adaptive_registration", false);
        this->declare_parameter<double>("sonar.range_noise_std_m", 0.0);
        this->declare_parameter<double>("sonar.angular_noise_std_deg", 0.0);

        this->declare_parameter<string>("imu.topic", "/imu/data");
        this->declare_parameter<double>("imu.frequency", 100.0);
        this->declare_parameter<double>("imu.initial_gravity_estimate", G_m_s2);
        this->declare_parameter<double>("imu.gyroscope_covariance", 0.1);
        this->declare_parameter<double>("imu.accelerometer_covariance", 0.1);
        this->declare_parameter<double>("imu.gyroscope_bias_covariance", 0.0001);
        this->declare_parameter<double>("imu.accelerometer_bias_covariance", 0.0001);
        this->declare_parameter<vector<double>>(
            "imu.initial_gyroscope_bias", vector<double>{});
        this->declare_parameter<double>("imu.init_gyroscope_bias_covariance", 0.0001);
        this->declare_parameter<double>("imu.init_accelerometer_bias_covariance", 0.001);
        this->declare_parameter<double>("imu.init_gravity_covariance", 0.00001);
        this->declare_parameter<double>("imu.orientation_covariance", 3.0461742e-6);
        this->declare_parameter<double>("imu.accel_attitude_covariance", 1.2184697e-3);
        this->declare_parameter<double>("imu.accel_attitude_norm_gate", 2.0);

        this->declare_parameter<int>("mapping.max_iteration", 4);
        this->declare_parameter<double>("mapping.filter_size_surf", 0.5);
        this->declare_parameter<double>("mapping.filter_size_map", 0.5);
        this->declare_parameter<double>("mapping.cube_side_length", 200.0);
        this->declare_parameter<string>("mapping.map_file_path", "");
        this->declare_parameter<string>("mapping.world_frame", "world");
        this->declare_parameter<bool>("loop_closure.enable", false);
        this->declare_parameter<string>("loop_closure.profile", "balanced");
        aux_fusion_.declare_parameters(*this);

    }

    void LoadParameters()
    {
        this->get_parameter_or<bool>("publish.corrected_map_enable",
                                     corrected_map_publish_enabled_, false);
        this->get_parameter_or<string>("publish.corrected_map_topic",
                                       corrected_map_topic_, "/uwfl2/corrected_map");
        this->get_parameter_or<string>("sonar.topic", lid_topic, "/points_raw");
        this->get_parameter_or<double>("sonar.frequency", sonar_frequency_hz, 4.0);
        this->get_parameter_or<vector<double>>("sonar.translation", extrinT,
                                               {0.0, 0.0, 0.0});
        this->get_parameter_or<vector<double>>("sonar.rotation", extrinR,
                                               {0.0, 0.0, 0.0});
        this->get_parameter_or<float>("sonar.max_range", DET_RANGE, 300.0F);
        this->get_parameter_or<double>("sonar.min_range", p_pre->blind, 0.01);
        this->get_parameter_or<double>("sonar.xy_covariance", LASER_POINT_COV_XY,
                                       double(LASER_POINT_COV_DEFAULT));
        this->get_parameter_or<double>("sonar.z_covariance", LASER_POINT_COV_Z,
                                       double(LASER_POINT_COV_DEFAULT));
        sonar_adaptive_registration = this->get_parameter("sonar.adaptive_registration").as_bool();
        if (sonar_adaptive_registration)
        {
            const double range_std = this->get_parameter("sonar.range_noise_std_m").as_double();
            const double angle_std = this->get_parameter("sonar.angular_noise_std_deg").as_double() * M_PI / 180.0;
            if (!std::isfinite(range_std) || range_std <= 0 || !std::isfinite(angle_std) || angle_std <= 0)
                throw std::invalid_argument("Adaptive registration requires a known, positive sonar noise profile");
            p_imu->SetSonarNoiseVariance(V3D(range_std * range_std, angle_std * angle_std, angle_std * angle_std));
            sonar_registration_view = std::make_unique<uwfl2::SonarRegistrationView>(2.0, sonar_map_point_budget, 4096);
        }

        this->get_parameter_or<string>("imu.topic", imu_topic, "/imu/data");
        this->get_parameter_or<double>("imu.frequency", imu_rate_hz, 100.0);
        this->get_parameter_or<double>("imu.initial_gravity_estimate",
                                       initial_gravity_estimate, G_m_s2);
        this->get_parameter_or<double>("imu.gyroscope_covariance", gyroscope_covariance, 0.1);
        this->get_parameter_or<double>("imu.accelerometer_covariance", accelerometer_covariance, 0.1);
        this->get_parameter_or<double>("imu.gyroscope_bias_covariance", gyroscope_bias_covariance, 0.0001);
        this->get_parameter_or<double>("imu.accelerometer_bias_covariance", accelerometer_bias_covariance, 0.0001);
        this->get_parameter_or<vector<double>>(
            "imu.initial_gyroscope_bias", initial_gyroscope_bias, {});
        if (!initial_gyroscope_bias.empty() &&
            (initial_gyroscope_bias.size() != 3 ||
             !std::all_of(initial_gyroscope_bias.begin(),
                          initial_gyroscope_bias.end(),
                          [](double value) { return std::isfinite(value); })))
        {
            throw std::invalid_argument(
                "imu.initial_gyroscope_bias must be empty or contain 3 finite values");
        }
        this->get_parameter_or<double>("imu.init_gyroscope_bias_covariance", init_gyroscope_bias_covariance, 0.0001);
        this->get_parameter_or<double>("imu.init_accelerometer_bias_covariance", init_accelerometer_bias_covariance, 0.001);
        this->get_parameter_or<double>("imu.init_gravity_covariance", init_grav_cov,
                                       0.00001);
        this->get_parameter_or<double>("imu.orientation_covariance",
                                       imu_orientation_cov, 3.0461742e-6);
        this->get_parameter_or<double>("imu.accel_attitude_covariance",
                                       accel_attitude_cov, 1.2184697e-3);
        this->get_parameter_or<double>("imu.accel_attitude_norm_gate",
                                       accel_attitude_norm_gate, 2.0);

        this->get_parameter_or<int>("mapping.max_iteration", NUM_MAX_ITERATIONS, 4);
        this->get_parameter_or<double>("mapping.filter_size_surf",
                                       filter_size_surf_min, 0.5);
        this->get_parameter_or<double>("mapping.filter_size_map",
                                       filter_size_map_min, 0.5);
        this->get_parameter_or<double>("mapping.cube_side_length", cube_len, 200.0);
        this->get_parameter_or<string>("mapping.map_file_path", map_file_path, "");
        this->get_parameter_or<string>("mapping.world_frame", world_frame, "world");
        LASER_POINT_COV_XY = std::max(1e-12, LASER_POINT_COV_XY);
        LASER_POINT_COV_Z = std::max(1e-12, LASER_POINT_COV_Z);
        imu_orientation_cov = std::max(1e-12, imu_orientation_cov);
        accel_attitude_cov = std::max(1e-8, accel_attitude_cov);
        accel_attitude_norm_gate = std::max(0.0, accel_attitude_norm_gate);
        if (!std::isfinite(sonar_frequency_hz) || sonar_frequency_hz <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(),
                        "sonar.frequency must be positive. Falling back to 4 Hz.");
            sonar_frequency_hz = 4.0;
        }
        lidar_timeout = uwfl2::timeout_from_frequency(sonar_frequency_hz, 4.0);
        if (imu_rate_hz <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(),
                        "imu.frequency must be positive. Falling back to 100 Hz.");
            imu_rate_hz = 100.0;
        }
        if (!std::isfinite(initial_gravity_estimate) ||
            initial_gravity_estimate <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(),
                        "imu.initial_gravity_estimate must be positive. Falling back to %.2f m/s^2.",
                        G_m_s2);
            initial_gravity_estimate = G_m_s2;
        }
        aux_fusion_.load_parameters(*this);
        auxiliary_fusion_enabled = aux_fusion_.dvl_enabled() ||
                                   aux_fusion_.pressure_enabled() ||
                                   aux_fusion_.mag_enabled();
        const double disabled_aux_cov = 1e-12;
        init_b_dvl_cov = aux_fusion_.dvl_enabled() ? aux_fusion_.dvl_b_init_cov() : disabled_aux_cov;
        init_b_pressure_cov = aux_fusion_.pressure_enabled()
                                  ? aux_fusion_.pressure_b_init_cov()
                                  : disabled_aux_cov;

    }

    void InitializeEstimator()
    {
        // /*** variables definition ***/
        // int effect_feat_num = 0, frame_num = 0;
        // double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
        // bool flg_EKF_converged, EKF_stop_flg = 0;



        memset(point_selected_surf, true, sizeof(point_selected_surf));
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        memset(point_selected_surf, true, sizeof(point_selected_surf));

        if (extrinT.size() != 3)
        {
            RCLCPP_WARN(this->get_logger(),
                        "sonar.translation must have 3 values. Using zero translation.");
            extrinT = {0.0, 0.0, 0.0};
        }
        Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU = uwfl2::rotation_from_rpy_degrees(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gravity(initial_gravity_estimate);
        p_imu->set_gyr_cov(V3D(gyroscope_covariance, gyroscope_covariance, gyroscope_covariance));
        p_imu->set_acc_cov(V3D(accelerometer_covariance, accelerometer_covariance, accelerometer_covariance));
        p_imu->set_gyr_bias_cov(V3D(gyroscope_bias_covariance, gyroscope_bias_covariance, gyroscope_bias_covariance));
        p_imu->set_acc_bias_cov(V3D(accelerometer_bias_covariance, accelerometer_bias_covariance, accelerometer_bias_covariance));
        if (initial_gyroscope_bias.size() == 3)
        {
            const V3D configured_bias(initial_gyroscope_bias[0],
                                      initial_gyroscope_bias[1],
                                      initial_gyroscope_bias[2]);
            p_imu->set_initial_gyroscope_bias(configured_bias);
            RCLCPP_INFO(this->get_logger(),
                        "Using fixed initial gyroscope bias [%.6f, %.6f, %.6f] rad/s.",
                        configured_bias.x(), configured_bias.y(), configured_bias.z());
        }
        p_imu->set_initial_cov(V3D(init_gyroscope_bias_covariance, init_gyroscope_bias_covariance, init_gyroscope_bias_covariance),
                               V3D(init_accelerometer_bias_covariance, init_accelerometer_bias_covariance, init_accelerometer_bias_covariance),
                               init_grav_cov);
        p_imu->set_initial_aux_cov(V3D(init_b_dvl_cov, init_b_dvl_cov, init_b_dvl_cov),
                                   init_b_pressure_cov);

        fill(epsi, epsi + state_ikfom::DOF, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);
        odometry_prediction_kf_.init_dyn_share(
            get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    }

    void InitializeLoopClosure()
    {
        bool loop_enabled = false;
        string loop_profile;
        this->get_parameter_or<bool>("loop_closure.enable", loop_enabled, false);
        this->get_parameter_or<string>("loop_closure.profile", loop_profile, "balanced");
        if (loop_profile != "balanced" && loop_profile != "simulation")
        {
            throw std::invalid_argument(
                "Unknown loop_closure.profile '" + loop_profile +
                "' (expected balanced or simulation)");
        }
        const auto loop_profile_path = std::filesystem::path(
            ament_index_cpp::get_package_share_directory("fast_lio")) /
            "config" / "loop_closure" / (loop_profile + ".yaml");
        auto loop_config =
            uwfl2::loop_closure::load_config_profile(loop_profile_path);
        loop_config.enabled = loop_enabled;
        loop_visualization_enabled_ = loop_enabled;
        loop_config.shadow_map.voxel_size_m = filter_size_map_min;
        if (loop_config.enabled)
        {
            loop_closure_ =
                std::make_unique<uwfl2::loop_closure::LoopClosureManager>(loop_config);
            RCLCPP_INFO(this->get_logger(),
                        "Loop closure enabled with '%s' profile: automatic place-based "
                        "detection and RViz markers active (no elapsed-time gate).",
                        loop_profile.c_str());
        }

    }

    void ensure_sonar_frame_attached_to_body(const std::string &frame_id)
    {
        if (frame_id.empty())
        {
            if (!empty_sonar_frame_warned_)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Incoming sonar cloud has an empty frame_id; RViz cannot place it in the UWFL2 TF tree.");
                empty_sonar_frame_warned_ = true;
            }
            return;
        }
        if (frame_id == "body" || frame_id == sonar_frame_id_)
        {
            return;
        }
        if (!sonar_frame_id_.empty())
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Sonar frame changed from '%s' to '%s'; refusing to create a second static sensor transform.",
                         sonar_frame_id_.c_str(), frame_id.c_str());
            return;
        }

        Eigen::Quaterniond body_to_sonar_quat(Lidar_R_wrt_IMU);
        body_to_sonar_quat.normalize();
        geometry_msgs::msg::TransformStamped body_to_sonar;
        body_to_sonar.header.stamp = this->get_clock()->now();
        body_to_sonar.header.frame_id = "body";
        body_to_sonar.child_frame_id = frame_id;
        body_to_sonar.transform.translation.x = Lidar_T_wrt_IMU.x();
        body_to_sonar.transform.translation.y = Lidar_T_wrt_IMU.y();
        body_to_sonar.transform.translation.z = Lidar_T_wrt_IMU.z();
        body_to_sonar.transform.rotation.x = body_to_sonar_quat.x();
        body_to_sonar.transform.rotation.y = body_to_sonar_quat.y();
        body_to_sonar.transform.rotation.z = body_to_sonar_quat.z();
        body_to_sonar.transform.rotation.w = body_to_sonar_quat.w();
        static_tf_broadcaster_->sendTransform(body_to_sonar);
        sonar_frame_id_ = frame_id;
        RCLCPP_INFO(this->get_logger(),
                    "Attached sonar frame '%s' to UWFL2 body using sonar.rotation/translation.",
                    sonar_frame_id_.c_str());
    }

    void reset_high_rate_odometry_prediction_locked()
    {
        auto state = kf.get_x();
        auto covariance = kf.get_P();
        odometry_prediction_kf_.change_x(state);
        odometry_prediction_kf_.change_P(covariance);
        odometry_prediction_Q_ = p_imu->Q;
        odometry_prediction_acceleration_scale_ =
            p_imu->AccelerationScale();
        odometry_prediction_time_ = lidar_end_time;
        odometry_prediction_correction_time_ = lidar_end_time;
        odometry_prediction_observation_time_ = imu_observation_calendar_.NextEpoch();
        next_odometry_prediction_publish_time_ =
            lidar_end_time + 1.0 / odometry_publish_rate_hz;
        odometry_prediction_ready_ = true;
    }

    void publish_predicted_odometry(
        const state_ikfom &state, const MainEkf::cov &covariance,
        double timestamp)
    {
        const Eigen::Quaterniond orientation(
            state.rot.toRotationMatrix());
        geometry_msgs::msg::Quaternion quaternion;
        quaternion.x = orientation.x();
        quaternion.y = orientation.y();
        quaternion.z = orientation.z();
        quaternion.w = orientation.w();
        const auto odometry = uwfl2::make_odometry(
            state, covariance, get_ros_time(timestamp), quaternion);
        pubOdomAftMapped_->publish(odometry);
        tf_broadcaster_->sendTransform(uwfl2::make_odometry_transform(odometry));
    }

    void publish_high_rate_odometry_prediction()
    {
        double pending_sonar_time =
            std::numeric_limits<double>::infinity();
        {
            std::lock_guard<std::mutex> lock(input_buffers_.mtx_buffer);
            if (input_buffers_.lidar_pushed)
            {
                pending_sonar_time = lidar_end_time;
            }
            else if (!input_buffers_.time_buffer.empty() && !input_buffers_.lidar_buffer.empty())
            {
                const double scan_begin_time = input_buffers_.time_buffer.front();
                const auto &scan = input_buffers_.lidar_buffer.front();
                double scan_duration = input_buffers_.lidar_mean_scantime;
                if (scan && scan->points.size() > 1U)
                {
                    const double measured_duration =
                        scan->points.back().curvature / 1000.0;
                    if (measured_duration >= 0.5 * input_buffers_.lidar_mean_scantime)
                    {
                        scan_duration = measured_duration;
                    }
                }
                pending_sonar_time = scan_begin_time + scan_duration;
            }
        }

        {
            std::lock_guard<std::mutex> lock(
                odometry_prediction_state_mutex_);
            if (!odometry_prediction_ready_ ||
                odometry_prediction_time_ < 0.0)
            {
                return;
            }

            const auto imu_samples = input_buffers_.PredictionSamples(
                odometry_prediction_time_, odometry_prediction_correction_time_);
            if (imu_samples.size() < 2U)
            {
                return;
            }

            struct PredictedOutput
            {
                state_ikfom state;
                MainEkf::cov covariance;
                double timestamp = -1.0;
            };
            std::vector<PredictedOutput> outputs;

            for (std::size_t i = 1; i < imu_samples.size(); ++i)
            {
                const auto &head = imu_samples[i - 1];
                const auto &tail = imu_samples[i];
                const double head_time = get_time_sec(head->header.stamp);
                const double tail_time = get_time_sec(tail->header.stamp);
                if (tail_time <= odometry_prediction_time_ + 1e-9)
                {
                    continue;
                }
                if (head_time > odometry_prediction_time_ + 1e-6)
                {
                    odometry_prediction_ready_ = false;
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 5000,
                        "IMU prediction history does not cover the corrected epoch. "
                        "Waiting for a fresh filter correction.");
                    return;
                }
                if (head_time >= pending_sonar_time - 1e-9 ||
                    tail_time > pending_sonar_time + 1e-9 ||
                    tail_time > odometry_prediction_observation_time_ + 1e-9)
                {
                    break;
                }
                const double segment_begin =
                    std::max(head_time, odometry_prediction_time_);
                const double segment_dt = tail_time - segment_begin;
                if (!std::isfinite(segment_dt) || segment_dt <= 0.0 ||
                    segment_dt > 0.1)
                {
                    continue;
                }

                input_ikfom input;
                input.gyro = V3D(
                    0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                    0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                    0.5 * (head->angular_velocity.z + tail->angular_velocity.z));
                input.acc = odometry_prediction_acceleration_scale_ * V3D(
                    0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                    0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                    0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z));

                const double output_period =
                    1.0 / odometry_publish_rate_hz;
                // Driver timestamps can arrive in bursts. Integrate the same
                // averaged IMU input at fixed output epochs inside the segment.
                while (next_odometry_prediction_publish_time_ <=
                       tail_time + 1e-9)
                {
                    const double output_time =
                        next_odometry_prediction_publish_time_;
                    if (output_time > odometry_prediction_time_ + 1e-9)
                    {
                        double prediction_dt =
                            output_time - odometry_prediction_time_;
                        odometry_prediction_kf_.predict(
                            prediction_dt,
                            odometry_prediction_Q_, input);
                        odometry_prediction_time_ = output_time;
                    }
                    if (output_time >
                        last_odometry_prediction_published_time_ + 1e-6)
                    {
                        PredictedOutput output{
                            odometry_prediction_kf_.get_x(),
                            odometry_prediction_kf_.get_P(), output_time};
                        if (coalesce_odometry_output_ && !outputs.empty())
                        {
                            outputs.back() = std::move(output);
                        }
                        else
                        {
                            outputs.push_back(std::move(output));
                        }
                    }
                    next_odometry_prediction_publish_time_ += output_period;
                }
                if (tail_time > odometry_prediction_time_ + 1e-9)
                {
                    double prediction_dt =
                        tail_time - odometry_prediction_time_;
                    odometry_prediction_kf_.predict(
                        prediction_dt,
                        odometry_prediction_Q_, input);
                    odometry_prediction_time_ = tail_time;
                }
            }
            if (!outputs.empty())
            {
                const auto wall_now = std::chrono::steady_clock::now();
                if (odometry_rate_window_sensor_time_ < 0.0)
                {
                    odometry_rate_window_sensor_time_ = outputs.back().timestamp;
                    odometry_rate_window_wall_time_ = wall_now;
                }
                const double wall_elapsed =
                    std::chrono::duration<double>(
                        wall_now - odometry_rate_window_wall_time_)
                        .count();
                if (wall_elapsed >= 0.25)
                {
                    const double sensor_elapsed =
                        outputs.back().timestamp -
                        odometry_rate_window_sensor_time_;
                    const double replay_rate =
                        std::max(0.0, sensor_elapsed) / wall_elapsed;
                    if (replay_rate > 2.0)
                    {
                        coalesce_odometry_output_ = true;
                    }
                    else if (replay_rate < 1.5)
                    {
                        coalesce_odometry_output_ = false;
                    }
                    odometry_rate_window_sensor_time_ =
                        outputs.back().timestamp;
                    odometry_rate_window_wall_time_ = wall_now;
                }
            }

            const std::size_t first_output =
                coalesce_odometry_output_ && !outputs.empty()
                    ? outputs.size() - 1U
                    : 0U;
            for (std::size_t i = first_output; i < outputs.size(); ++i)
            {
                const auto &output = outputs[i];
                if (output.timestamp >
                    last_odometry_prediction_published_time_ + 1e-6)
                {
                    last_odometry_prediction_published_time_ = output.timestamp;
                    publish_predicted_odometry(
                        output.state, output.covariance, output.timestamp);
                }
            }
        }
    }

    void publish_estimator_outputs()
    {
        if (!p_imu->IsInitialized())
        {
            publish_odometry(pubOdomAftMapped_, tf_broadcaster_);
        }
        else
        {
            std::lock_guard<std::mutex> lock(
                odometry_prediction_state_mutex_);
            // A prediction may already be newer when scan processing is
            // delayed. Never send TF/Odometry backward; reset the private
            // predictor from the corrected state and let its next output carry
            // the correction forward monotonically.
            if (lidar_end_time >
                last_odometry_prediction_published_time_ + 1e-6)
            {
                publish_odometry(pubOdomAftMapped_, tf_broadcaster_);
                last_odometry_prediction_published_time_ = lidar_end_time;
            }
            reset_high_rate_odometry_prediction_locked();
        }
    }

    void publish_navigation_health()
    {
        diagnostic_msgs::msg::DiagnosticArray message;
        message.header.stamp = this->now();
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "uwfl2/navigation";
        status.hardware_id = "uwfl2";
        double imu_stamp, sonar_stamp, scan_stamp, frontier, sonar_lag, sonar_late_age;
        std::size_t sonar_received, sonar_late_dropped,
                    sonar_nonmonotonic_dropped;
        uwfl2::MappingInputBuffers::Clock::time_point imu_receipt;
        {
            std::lock_guard<std::mutex> lock(input_buffers_.mtx_buffer);
            imu_stamp = input_buffers_.last_timestamp_imu;
            sonar_stamp = input_buffers_.last_timestamp_lidar;
            scan_stamp = input_buffers_.last_scan_end_time;
            frontier = input_buffers_.last_processed_time;
            imu_receipt = input_buffers_.last_imu_receipt;
            sonar_received = input_buffers_.sonar_received;
            sonar_late_dropped = input_buffers_.sonar_late_dropped;
            sonar_nonmonotonic_dropped =
                input_buffers_.sonar_nonmonotonic_dropped;
            sonar_lag = input_buffers_.max_sonar_acquisition_lag;
            sonar_late_age = input_buffers_.max_sonar_late_age;
        }
        const auto aux = aux_fusion_.availability();
        // Acquisition ages must not mix synthetic/bag epochs with host wall time.
        // With simulation time enabled, /clock also reveals a stalled IMU stream.
        const double now = this->get_parameter("use_sim_time").as_bool()
            ? std::max(this->now().seconds(), imu_stamp) : imu_stamp;
        const auto available = [now](double stamp, double timeout) {
            return stamp >= 0.0 && now - stamp <= timeout;
        };
        const double imu_receipt_age = std::chrono::duration<double>(
            uwfl2::MappingInputBuffers::Clock::now() - imu_receipt).count();
        const bool imu_ok = available(imu_stamp, expected_imu_timeout()) &&
            (this->get_parameter("use_sim_time").as_bool() ||
             imu_receipt_age <= std::max(0.25, expected_imu_timeout()));
        const bool sonar_ok = !lid_topic.empty() && available(sonar_stamp, 2.0 * lidar_timeout);
        const bool dvl_ok = aux_fusion_.dvl_enabled() &&
            available(last_dvl_applied_time_, 2.0 * aux.dvl_timeout);
        const bool usable_sonar = sonar_ok &&
            available(scan_stamp, 2.0 * lidar_timeout) &&
            sonar_condition_ == "accepted_geometry_rank_unassessed";
        const bool initialized = p_imu->IsInitialized();
        status.level = !initialized || !imu_ok ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
            (!usable_sonar && !dvl_ok ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
                                      diagnostic_msgs::msg::DiagnosticStatus::OK);
        status.message = !initialized ? "initializing" : !imu_ok ? "IMU unavailable" :
            !usable_sonar && !dvl_ok ? "degraded: no usable sonar or fused DVL" : "aided navigation";
        auto add = [&](const std::string &key, const std::string &value) {
            diagnostic_msgs::msg::KeyValue item;
            item.key = key; item.value = value; status.values.push_back(item);
        };
        add("sonar_condition", !sonar_ok ? (lid_topic.empty() ? "disabled" : "missing") : sonar_condition_);
        add("geometry_rank", "unassessed; partial/full information classification deferred to CP-3");
        add("sonar_input_condition", uwfl2::SonarConditionName(sonar_availability_.At(now).condition));
        add("last_sonar_geometry_rank", std::to_string(last_sonar_geometry_rank_));
        add("sonar_recovering", sonar_recovery_.Recovering() ? "true" : "false");
        add("recovery_candidates", std::to_string(sonar_recovery_.Consecutive()));
        add("provisional_scans", std::to_string(provisional_scans_.size()));
        add("imu_available", imu_ok ? "true" : "false");
        add("imu_receipt_age_wall_seconds", std::to_string(imu_receipt_age));
        add("dvl_received", aux_fusion_.dvl_enabled() && available(aux.dvl_stamp, 2.0 * aux.dvl_timeout) ? "true" : "false");
        add("dvl_recently_fused", dvl_ok ? "true" : "false");
        add("pressure_received", aux_fusion_.pressure_enabled() && available(aux.pressure_stamp, 2.0 * aux.pressure_timeout) ? "true" : "false");
        add("magnetometer_received", aux_fusion_.mag_enabled() && available(aux.magnetometer_stamp, 2.0 * aux.magnetometer_timeout) ? "true" : "false");
        add("state_timestamp", std::to_string(frontier));
        add("availability_time_reference", this->get_parameter("use_sim_time").as_bool()
            ? "sensor_time_and_ros_clock" : "latest_imu_acquisition_time");
        add("processing_lag_seconds", std::to_string(imu_stamp - frontier));
        add("imu_observations", std::to_string(imu_observation_count_));
        add("aux_considered", std::to_string(aux_considered_count_));
        add("aux_applied", std::to_string(aux_applied_count_));
        add("aux_late_dropped", std::to_string(late_aux_count_));
        add("sonar_received", std::to_string(sonar_received));
        add("sonar_late_dropped", std::to_string(sonar_late_dropped));
        add("sonar_nonmonotonic_dropped",
            std::to_string(sonar_nonmonotonic_dropped));
        add("maximum_sonar_acquisition_lag_seconds", std::to_string(sonar_lag));
        add("maximum_sonar_late_age_seconds", std::to_string(sonar_late_age));
        if (initialized)
        {
            const MainEkf::cov covariance = kf.get_P();
            const bool finite = covariance.allFinite();
            const double asymmetry = finite ? (covariance - covariance.transpose()).norm() : INFINITY;
            Eigen::SelfAdjointEigenSolver<MainEkf::cov> eigen;
            if (finite) eigen.compute((covariance + covariance.transpose()) * 0.5);
            const double minimum = finite && eigen.info() == Eigen::Success ? eigen.eigenvalues().minCoeff() : -INFINITY;
            add("covariance_trace", std::to_string(covariance.trace()));
            add("position_covariance_trace", std::to_string(covariance.block<3, 3>(0, 0).trace()));
            add("attitude_covariance_trace", std::to_string(covariance.block<3, 3>(3, 3).trace()));
            add("covariance_asymmetry", std::to_string(asymmetry));
            add("covariance_min_eigenvalue", std::to_string(minimum));
            if (!finite || asymmetry > 1e-8 || minimum < -1e-10)
            {
                status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                status.message = "invalid estimator covariance";
            }
        }
        message.status.push_back(status);
        navigation_health_pub_->publish(message);
    }

    void publish_sonar_diagnostics(const std::string &stage, double timestamp,
                                   const uwfl2::LidarUpdateResult &result = {},
                                   const std::optional<uwfl2::SonarDetection> &interval_detection = std::nullopt)
    {
        if (stage == "geometry") last_sonar_geometry_rank_ = sonar_geometry_rank;
        if (sonar_diagnostics_pub_->get_subscription_count() == 0) return;
        diagnostic_msgs::msg::DiagnosticArray message;
        message.header.stamp = get_ros_time(timestamp);
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "uwfl2/sonar";
        status.hardware_id = "uwfl2";
        const auto detection = interval_detection.value_or(sonar_availability_.At(timestamp));
        status.message = uwfl2::SonarConditionName(detection.condition);
        auto add = [&](const std::string &key, const std::string &value) {
            diagnostic_msgs::msg::KeyValue entry;
            entry.key = key; entry.value = value; status.values.push_back(entry);
        };
        add("stage", stage);
        add("input_condition", status.message);
        add("finite_points", std::to_string(sonar_input_metrics_.finite_points));
        add("count_ratio", std::to_string(detection.count_ratio));
        add("azimuth_ratio", std::to_string(detection.azimuth_ratio));
        add("elevation_ratio", std::to_string(detection.elevation_ratio));
        add("missing_deadline_seconds", std::to_string(sonar_availability_.MissingDeadline()));
        add("acquisition_age_seconds", std::to_string(detection.acquisition_age));
        add("effective_points", std::to_string(effct_feat_num));
        add("registration_mode", sonar_adaptive_registration ? "adaptive_hybrid" : "original");
        add("plane_rows", std::to_string(sonar_plane_rows));
        add("point_rows", std::to_string(sonar_point_rows));
        add("unresolved_plane_normals", std::to_string(sonar_unresolved_plane_normals));
        add("registration_leaf_m", std::to_string(sonar_voxel_controller_ ? sonar_voxel_controller_->Leaf() : filter_size_surf_min));
        add("view_build_seconds", std::to_string(sonar_view_build_seconds));
        add("geometry_rank", std::to_string(sonar_geometry_rank));
        add("geometry_condition", effct_feat_num == 0 ? "unassessed_or_unusable" :
            sonar_geometry_rank < 6 ? "partially_informative" : "informative");
        add("accepted", result.accepted ? "true" : "false");
        add("update_reason", uwfl2::lidar_scan_status(result.reason));
        add("recovery_candidates", std::to_string(sonar_recovery_.Consecutive()));
        add("mean_squared_whitened_residual", std::to_string(sonar_mean_squared_whitened_residual));
        add("dx_position_m", std::to_string(sonar_dx_position_));
        add("dx_rotation_rad", std::to_string(sonar_dx_rotation_));
        std::vector<double> residuals;
        for (int i = 0; i < effct_feat_num; ++i)
            residuals.push_back(std::abs(corr_normvect->points[i].intensity));
        double residual_sum = 0.0;
        for (double residual : residuals) residual_sum += residual;
        add("residual_mean_m", std::to_string(residuals.empty() ? 0.0 : residual_sum / residuals.size()));
        add("residual_p95_m", std::to_string(uwfl2::SonarQuantile(residuals, 0.95)));
        add("map_points_inserted", std::to_string(result.map_points_inserted));
        add("active_map_points", std::to_string(ikdtree->validnum()));
        {
            std::lock_guard<std::mutex> lock(mapping_output_mutex);
            add("pending_history_scans", std::to_string(pending_mapping_scans_.size()));
        }
        add("scan_processing_seconds", std::to_string(std::chrono::duration<double>(
            std::chrono::steady_clock::now() - sonar_processing_start_).count()));
        add("acquisition_lag_seconds", std::to_string(std::max(0.0,
            input_buffers_.last_processed_time - timestamp)));
        message.status.push_back(status);
        sonar_diagnostics_pub_->publish(message);
    }

    bool prepare_adaptive_sonar_scan(double timestamp)
    {
        const auto &covariances = p_imu->ScanMeasurementCovariances();
        if (covariances.size() != feats_undistort->size() || covariances.empty()) return false;
        std::vector<uwfl2::SonarReferencePoint> points;
        points.reserve(covariances.size());
        for (std::size_t i = 0; i < covariances.size(); ++i)
        {
            const auto &point = feats_undistort->points[i];
            points.push_back({V3D(point.x, point.y, point.z), covariances[i]});
        }
        if (!sonar_voxel_controller_)
        {
            const auto initial = uwfl2::VoxelizeSonarPoints(points, filter_size_surf_min, 100000);
            if (!initial.complete || initial.points.empty()) return false;
            std::vector<double> ranges;
            for (const auto &point : initial.points) ranges.push_back(point.position.norm());
            const double median = std::max(1e-3, uwfl2::SonarQuantile(ranges, 0.5));
            const std::size_t count = initial.points.size();
            const double gain = filter_size_surf_min / (20.0 * count);
            // Frozen scale-relative settings, calibrated from the initial scan
            // at the existing leaf size; no donor LiDAR numerical presets.
            sonar_voxel_controller_ = std::make_unique<uwfl2::SonarVoxelController>(uwfl2::SonarVoxelControlConfig{
                filter_size_surf_min, filter_size_surf_min / 3, filter_size_surf_min * 2,
                20, std::max<std::size_t>(1, count / 2), std::max<std::size_t>(1, count * 3 / 2),
                median * 2, 1, 0.5, 0.5, gain, gain * 10, 0, gain / sonar_frequency_hz});
        }
        const double interval = last_adaptive_sonar_time_ < 0 ? 1 / sonar_frequency_hz : timestamp - last_adaptive_sonar_time_;
        if (interval <= 0) return false;
        const auto scan = uwfl2::PrepareAdaptiveSonarScan(*sonar_voxel_controller_, points, interval, 100000);
        if (!scan.complete) return false;
        last_adaptive_sonar_time_ = timestamp;
        sonar_scan_points = scan.update.points;
        sonar_update_cells = scan.update.cells;
        sonar_registration_leaf = scan.leaf;
        feats_down_body->resize(sonar_scan_points.size());
        for (std::size_t i = 0; i < sonar_scan_points.size(); ++i)
        {
            PointType point;
            point.x = sonar_scan_points[i].position.x(); point.y = sonar_scan_points[i].position.y(); point.z = sonar_scan_points[i].position.z();
            point.intensity = point.curvature = 0;
            feats_down_body->points[i] = point;
            sonar_scan_points[i].position = V3D(point.x, point.y, point.z);
            point_selected_surf[i] = true;
        }
        return true;
    }

    void timer_callback()
    {
        if (!ensure_initial_world_pose()) return;
        try_commit_loop_correction();
        if (provisional_map_reset_requested_)
        {
            provisional_map_reset_requested_ = false;
            clear_provisional_map();
        }
        maybe_publish_loop_visualization();
        bool imu_only_measure = false;
        bool has_measurement = false;
        {
            std::lock_guard<std::mutex> lock(input_buffers_.mtx_buffer);
            has_measurement = sync_packages(Measures, imu_only_measure);
            if (!has_measurement)
            {
                imu_only_measure = sync_imu_only_packages(Measures) == ImuPacketResult::Ready;
                has_measurement = imu_only_measure;
            }
        }

        if(has_measurement)
        {
            uwfl2::LidarUpdateResult lidar_update_result;
            const auto interval_availability = sonar_availability_;
            effct_feat_num = 0;
            feats_down_size = 0;
            sonar_plane_rows = sonar_point_rows = sonar_unresolved_plane_normals = 0;
            sonar_mean_squared_whitened_residual = 0.0;
            sonar_view_build_seconds = 0;
            add_point_size = 0;
            sonar_geometry_rank = 0;
            sonar_dx_position_ = sonar_dx_rotation_ = 0.0;
            if (sonar_adaptive_registration && ikdtree->Root_Node != nullptr &&
                (interval_availability.At(Measures.lidar_end_time).condition ==
                     uwfl2::SonarInputCondition::Missing ||
                 (!imu_only_measure && (!Measures.lidar || Measures.lidar->empty()))))
                sonar_recovery_.RequireValidation();
            if (!imu_only_measure)
            {
                sonar_processing_start_ = std::chrono::steady_clock::now();
                sonar_input_metrics_ = uwfl2::MeasureSonarInput(Measures.lidar->points);
                sonar_typical_range = sonar_input_metrics_.median_range;
                sonar_availability_.Observe(Measures.lidar_beg_time, sonar_input_metrics_);
                publish_sonar_diagnostics("input", Measures.lidar_beg_time);
            }
            else if (sonar_availability_.At(Measures.lidar_end_time).condition ==
                     uwfl2::SonarInputCondition::Missing &&
                     Measures.lidar_end_time - last_missing_diagnostic_time_ >= 0.05)
            {
                last_missing_diagnostic_time_ = Measures.lidar_end_time;
                publish_sonar_diagnostics("availability", Measures.lidar_end_time);
            }
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
            }

            double process_begin_time = input_buffers_.last_processed_time > 0.0 ? input_buffers_.last_processed_time : Measures.lidar_beg_time;
            if (!p_imu->IsInitialized())
            {
                p_imu->Process(Measures, kf, feats_undistort);
                if (!p_imu->IsInitialized())
                {
                    input_buffers_.last_processed_time = Measures.lidar_end_time;
                    update_state_outputs();
                    return;
                }
                process_begin_time = p_imu->InitializationTime();
                input_buffers_.last_processed_time = process_begin_time;
                imu_observation_calendar_.Reset(process_begin_time);
                if (auxiliary_fusion_enabled)
                {
                    aux_fusion_.initialize_pressure_reference_pose(kf.get_x());
                }
                // Preserve post-window samples/events in this packet, but do
                // not map a scan whose early portion predates initialization.
                while (!Measures.imu.empty() &&
                       get_time_sec(Measures.imu.front()->header.stamp) <= process_begin_time + 1e-9)
                    Measures.imu.pop_front();
                if (Measures.lidar_end_time <= process_begin_time + 1e-9)
                {
                    update_state_outputs();
                    return;
                }
                if (Measures.imu.empty())
                    Measures.imu.push_back(p_imu->InitializationSample());
                Measures.lidar.reset(new PointCloudXYZI());
                Measures.lidar_beg_time = process_begin_time;
                imu_only_measure = true;
            }

            std::vector<AuxiliarySensorFusion::TimedMeasurement> timed_measurements;
            if (auxiliary_fusion_enabled)
            {
                aux_fusion_.initialize_pressure_reference_pose(kf.get_x());
                timed_measurements = aux_fusion_.take_timed_measurements(
                    process_begin_time, Measures.lidar_end_time);
                const auto late = aux_fusion_.take_late_measurement_counts();
                late_aux_count_ += late.total();
                if (aux_timeline_started_ && late.total() > 0)
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 5000,
                        "Dropped out-of-sequence auxiliary measurements: DVL=%zu pressure=%zu magnetometer=%zu.",
                        late.dvl, late.pressure, late.magnetometer);
                aux_timeline_started_ = true;
            }
            const auto observations =
                imu_observation_calendar_.Take(Measures.imu, Measures.lidar_end_time);
            struct Event { double timestamp; bool attitude; std::size_t index; };
            std::vector<Event> events;
            for (std::size_t i = 0; i < timed_measurements.size(); ++i)
                events.push_back({timed_measurements[i].timestamp, false, i});
            for (std::size_t i = 0; i < observations.size(); ++i)
                events.push_back({observations[i].timestamp, true, i});
            // Equal-time auxiliary events retain their order, followed by IMU observations.
            std::stable_sort(events.begin(), events.end(),
                [](const Event &a, const Event &b) { return a.timestamp < b.timestamp; });
            std::vector<double> timestamps;
            for (const auto &event : events) timestamps.push_back(event.timestamp);
            const auto measurement_imu = p_imu->ImuSamplesForMeasurements(Measures.imu);
            p_imu->Process(Measures, kf, feats_undistort, timestamps,
                [&](std::size_t index, ImuProcess::Ekf &filter) {
                    const auto &event = events[index];
                    // A queued returning scan cannot hide the missing interval
                    // traversed by this propagation packet. Reporting uses the
                    // causal pre-scan detector; it never modifies the filter.
                    const auto detection = interval_availability.At(event.timestamp);
                    if (!imu_only_measure && event.timestamp < Measures.lidar_beg_time &&
                        detection.condition == uwfl2::SonarInputCondition::Missing &&
                        event.timestamp - last_missing_diagnostic_time_ >= 0.05)
                    {
                        last_missing_diagnostic_time_ = event.timestamp;
                        publish_sonar_diagnostics("availability", event.timestamp, {}, detection);
                    }
                    if (event.attitude)
                    {
                        const auto &window = observations[event.index].samples;
                        const bool orientation = apply_imu_orientation_update(window.back(), filter);
                        const bool tilt = apply_accel_attitude_update(window, filter);
                        ++imu_observation_count_;
                        return orientation || tilt;
                    }
                    const auto &measurement = timed_measurements[event.index];
                    const bool applied = aux_fusion_.apply_timed_measurement(measurement, measurement_imu, filter);
                    ++aux_considered_count_;
                    if (applied) ++aux_applied_count_;
                    if (applied && measurement.kind == AuxiliarySensorFusion::MeasurementKind::Dvl)
                        last_dvl_applied_time_ = measurement.timestamp;
                    return applied;
                });
            if (auxiliary_fusion_enabled)
                aux_fusion_.warn_timeouts(*this, Measures.lidar_end_time);
            input_buffers_.last_processed_time = Measures.lidar_end_time;
            update_state_outputs();

            if (imu_only_measure)
            {
                // Packet scheduling is not a sonar availability observation.
                // The health publisher derives missing input from acquisition age.
                if (lid_topic.empty()) sonar_condition_ = "disabled";
                if (lid_topic.empty())
                {
                    // Intentional no-lidar mode. Startup already reported this once.
                }
                else if (input_buffers_.last_timestamp_lidar <= 0.0)
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                         "No LiDAR messages received on '%s'. Running IMU-only odometry.",
                                         lid_topic.c_str());
                }
                else
                {
                    const double lidar_gap = Measures.lidar_end_time - input_buffers_.last_timestamp_lidar;
                    if (lidar_gap >= lidar_timeout)
                    {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                             "No LiDAR scan for %.2f s (timeout %.2f s). Running IMU-only odometry.",
                                             lidar_gap, lidar_timeout);
                    }
                }
                publish_estimator_outputs();
                return;
            }

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                if (sonar_adaptive_registration) sonar_recovery_.RequireValidation();
                sonar_condition_ = "empty";
                RCLCPP_WARN(this->get_logger(), "No point, publish IMU-only odometry for this scan.\n");
                lidar_update_result = lidar_scan_quality.reject_sparse_input(0);
                publish_sonar_diagnostics("geometry", Measures.lidar_end_time, lidar_update_result);
                publish_estimator_outputs();
                return;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            if (!sonar_adaptive_registration) lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            if (sonar_adaptive_registration)
            {
                if (!prepare_adaptive_sonar_scan(Measures.lidar_end_time))
                {
                    sonar_recovery_.RequireValidation();
                    sonar_condition_ = "unusable_noise_metadata";
                    lidar_update_result = lidar_scan_quality.reject_sparse_input(0);
                    publish_sonar_diagnostics("geometry", Measures.lidar_end_time, lidar_update_result);
                    publish_estimator_outputs();
                    return;
                }
            }
            else
            {
                downSizeFilterSurf.setInputCloud(feats_undistort);
                downSizeFilterSurf.filter(*feats_down_body);
            }

            feats_down_size = feats_down_body->points.size();
            if (!lidar_scan_quality.input_is_sufficient(
                    static_cast<std::size_t>(feats_down_size)))
            {
                lidar_update_result = lidar_scan_quality.reject_sparse_input(
                    static_cast<std::size_t>(feats_down_size));
                if (sonar_adaptive_registration) sonar_recovery_.RequireValidation();
                sonar_condition_ = "unusable_sparse_input";
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Rejected sparse sonar scan: %d downsampled points (minimum %d).",
                    feats_down_size, minimum_scan_points);
                publish_sonar_diagnostics("geometry", Measures.lidar_end_time, lidar_update_result);
                publish_estimator_outputs();
                return;
            }
            /*** initialize the map kdtree ***/
            if(ikdtree->Root_Node == nullptr)
            {
                RCLCPP_INFO(this->get_logger(), "Initialize the map kdtree");
                if(lidar_scan_quality.map_initialization_is_sufficient(
                       static_cast<std::size_t>(feats_down_size)))
                {
                    ikdtree->set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    if (sonar_adaptive_registration && !record_sonar_scan_uncertainty())
                    {
                        sonar_condition_ = "unusable_noise_metadata";
                        publish_estimator_outputs();
                        return;
                    }
                    if (sonar_adaptive_registration)
                    {
                        lasermap_fov_segment();
                        ikdtree->Build(sonar_integration_world);
                    }
                    else ikdtree->Build(feats_down_world->points);
                    sonar_condition_ = "map_initialization";
                    sonar_recovery_ = uwfl2::SonarRecovery{};
                    clear_provisional_map();
                    lidar_update_result =
                        lidar_scan_quality.accept_map_initialization(
                            static_cast<std::size_t>(feats_down_size));
                    lidar_update_result.map_points_inserted =
                        static_cast<std::size_t>(ikdtree->validnum());
                    if (loop_closure_)
                    {
                        ++active_tree_generation_;
                        loop_closure_->notify_active_tree_generation(
                            active_tree_generation_);
                    }

                    accumulate_mapping_output(Measures.lidar_end_time);
                    submit_loop_keyframe(Measures.lidar_end_time);
                }
                else
                {
                    sonar_condition_ = "unusable_sparse_input";
                    lidar_update_result = lidar_scan_quality.reject_sparse_input(
                        static_cast<std::size_t>(feats_down_size));
                }
                publish_sonar_diagnostics("geometry", Measures.lidar_end_time, lidar_update_result);
                publish_estimator_outputs();
                return;
            }

            if (sonar_adaptive_registration && !rebuild_sonar_registration_view())
            {
                sonar_recovery_.RequireValidation();
                sonar_condition_ = "unusable_map_uncertainty";
                lidar_update_result = lidar_scan_quality.reject_sparse_input(feats_down_size);
                publish_sonar_diagnostics("geometry", Measures.lidar_end_time, lidar_update_result);
                publish_estimator_outputs();
                return;
            }

            /*** ICP and iterated Kalman filter update ***/
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);

            /*** iterated state estimation ***/
            double solve_H_time = 0;

            const double lidar_update_cov =
                (auxiliary_fusion_enabled || sonar_adaptive_registration) ? 1.0 : LASER_POINT_COV_XY;
            lidar_update_max_effective_features = 0;
            uwfl2::LidarUpdateTransaction<MainEkf> lidar_transaction(kf);
            const auto sonar_prior = kf.get_x();
            sonar_registration_prior = kf.get_P();
            const bool update_applied =
                kf.update_iterated_dyn_share_modified(lidar_update_cov,
                                                      solve_H_time);
            lidar_update_result = lidar_scan_quality.evaluate_update(
                static_cast<std::size_t>(feats_down_size),
                static_cast<std::size_t>(lidar_update_max_effective_features),
                update_applied);
            if (sonar_adaptive_registration)
            {
                if (!lidar_update_result.accepted) sonar_recovery_.RequireValidation();
                else
                {
                    Eigen::Matrix<double, state_ikfom::DOF, 1> delta;
                    kf.get_x().boxminus(delta, sonar_prior);
                    // Compare recovery candidates in the fixed local frame, not
                    // different body tangents as the vehicle rotates.
                    uwfl2::SonarRecovery::PoseCovariance transport =
                        uwfl2::SonarRecovery::PoseCovariance::Identity();
                    transport.bottomRightCorner<3, 3>() = sonar_prior.rot.toRotationMatrix();
                    const auto decision = sonar_recovery_.Candidate(active_tree_generation_,
                        transport * delta.head<6>(),
                        transport * sonar_registration_prior.topLeftCorner<6, 6>() * transport.transpose(),
                        static_cast<double>(effct_feat_num) / feats_down_size,
                        sonar_mean_squared_whitened_residual);
                    if (decision == uwfl2::SonarRecovery::Decision::Validating ||
                        decision == uwfl2::SonarRecovery::Decision::Inconsistent)
                    {
                        lidar_update_result.accepted = false;
                        lidar_update_result.reason = decision == uwfl2::SonarRecovery::Decision::Validating
                            ? uwfl2::LidarScanRejectionReason::kRecoveryValidation
                            : uwfl2::LidarScanRejectionReason::kInconsistentRecovery;
                    }
                    else if (decision == uwfl2::SonarRecovery::Decision::Resumed)
                        clear_provisional_map();
                }
            }
            lidar_transaction.finish(lidar_update_result);
            Eigen::Matrix<double, state_ikfom::DOF, 1> sonar_delta;
            kf.get_x().boxminus(sonar_delta, sonar_prior);
            sonar_dx_position_ = sonar_delta.head<3>().norm();
            sonar_dx_rotation_ = sonar_delta.segment<3>(3).norm();
            if (!lidar_update_result.accepted)
            {
                sonar_condition_ = lidar_update_result.reason == uwfl2::LidarScanRejectionReason::kRecoveryValidation
                    ? "recovery_validation" :
                    lidar_update_result.reason == uwfl2::LidarScanRejectionReason::kInconsistentRecovery
                        ? "inconsistent_recovery" : "unusable_correspondences";
                update_state_outputs();
                if (sonar_adaptive_registration && effct_feat_num == 0)
                    publish_provisional_scan(Measures.lidar_end_time);

                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Held sonar update: %s, %d effective features (minimum %d).",
                    uwfl2::lidar_scan_status(lidar_update_result.reason),
                    lidar_update_max_effective_features,
                    minimum_effective_features);
                publish_sonar_diagnostics("geometry", Measures.lidar_end_time, lidar_update_result);
                publish_estimator_outputs();
                return;
            }

            update_state_outputs();

            if (sonar_adaptive_registration && !record_sonar_scan_uncertainty())
            {
                sonar_recovery_.RequireValidation();
                auto state = sonar_prior;
                auto covariance = sonar_registration_prior;
                kf.change_x(state); kf.change_P(covariance);
                update_state_outputs();
                sonar_condition_ = "unusable_noise_metadata";
                lidar_update_result.accepted = false;
                publish_sonar_diagnostics("geometry", Measures.lidar_end_time, lidar_update_result);
                publish_estimator_outputs();
                return;
            }

            /******* Publish odometry *******/
            // Count-based acceptance does not establish full six-DOF observability.
            sonar_condition_ = "accepted_geometry_rank_unassessed";
            publish_estimator_outputs();

            /*** add the feature points to map kdtree ***/
            if (sonar_adaptive_registration) lasermap_fov_segment();
            lidar_update_result.map_points_inserted = map_incremental();

            if (loop_closure_)
            {
                ++active_tree_generation_;
                loop_closure_->notify_active_tree_generation(
                    active_tree_generation_);
            }

            accumulate_mapping_output(Measures.lidar_end_time);
            submit_loop_keyframe(Measures.lidar_end_time);
            publish_sonar_diagnostics("geometry", Measures.lidar_end_time, lidar_update_result);
        }
    }

    struct ProvisionalScan
    {
        PointCloudXYZI::Ptr points;
        state_ikfom anchor;
        MainEkf::cov anchor_covariance;
    };

    void clear_provisional_map()
    {
        if (provisional_scans_.empty()) return;
        provisional_scans_.clear();
        publish_provisional_map(lidar_end_time);
    }

    void publish_provisional_scan(double timestamp)
    {
        // Display only: retain uncertain navigation anchors, never feed these
        // points to correspondence search, the trusted tree, or loop closure.
        PointCloudXYZI::Ptr points(new PointCloudXYZI());
        points->resize(feats_undistort->size());
        for (std::size_t i = 0; i < points->size(); ++i)
            pointBodyToWorld(&feats_undistort->points[i], &points->points[i]);
        provisional_scans_.push_back({points, kf.get_x(), kf.get_P()});
        if (provisional_scans_.size() > 20) provisional_scans_.pop_front();
        publish_provisional_map(timestamp);
    }

    void publish_provisional_map(double timestamp)
    {
        if (!provisional_map_pub_ || provisional_map_pub_->get_subscription_count() == 0) return;
        PointCloudXYZI cloud;
        for (const auto &scan : provisional_scans_) cloud += *scan.points;
        sensor_msgs::msg::PointCloud2 message;
        pcl::toROSMsg(cloud, message);
        message.header.frame_id = "camera_init";
        message.header.stamp = get_ros_time(timestamp);
        provisional_map_pub_->publish(message);
    }

    void map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        try
        {
            const PointCloudXYZI snapshot = corrected_mapping_snapshot();
            if (snapshot.empty())
            {
                res->success = false;
                res->message = "Map is empty; no PCD was written.";
            }
            else
            {
                const auto saved_path = save_to_pcd(snapshot);
                res->success = true;
                res->message = "Corrected map saved to " + saved_path + " with " +
                               std::to_string(snapshot.size()) + " points.";
                RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
            }
        }
        catch (const std::exception &error)
        {
            res->success = false;
            res->message = std::string("Map save failed: ") + error.what();
        }
    }

    using MappingScan = uwfl2::loop_closure::MappingScan;
    using MappingScanVector =
        std::vector<MappingScan, Eigen::aligned_allocator<MappingScan>>;
    using MappingScanQueue =
        std::deque<MappingScan, Eigen::aligned_allocator<MappingScan>>;

    static void append_mapping_scans(PointCloudXYZI &cloud,
                                     const MappingScanVector &scans)
    {
        std::size_t additional_points = 0;
        for (const MappingScan &scan : scans)
        {
            if (scan.points_world)
            {
                additional_points += scan.points_world->size();
            }
        }
        const std::size_t required_capacity =
            cloud.points.size() + additional_points;
        if (required_capacity > cloud.points.capacity())
        {
            const std::size_t geometric_capacity =
                cloud.points.capacity() > 0U
                    ? cloud.points.capacity() + cloud.points.capacity() / 2U
                    : 1024U;
            cloud.points.reserve(
                std::max(required_capacity, geometric_capacity));
        }
        for (const MappingScan &scan : scans)
        {
            if (!scan.points_world)
            {
                continue;
            }
            for (const auto &point : *scan.points_world)
            {
                PointType converted;
                converted.x = point.x;
                converted.y = point.y;
                converted.z = point.z;
                converted.intensity = point.intensity;
                converted.normal_x = converted.normal_y = converted.normal_z = 0.0F;
                converted.curvature = 0.0F;
                cloud.points.push_back(converted);
            }
        }
        cloud.width = static_cast<std::uint32_t>(cloud.size());
        cloud.height = 1;
        cloud.is_dense = false;
    }

    visualization_msgs::msg::Marker mapping_points_marker(
        const PointCloudXYZI &cloud, std::int32_t id,
        const builtin_interfaces::msg::Time &stamp) const
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "camera_init";
        marker.header.stamp = stamp;
        marker.ns = "uwfl2_corrected_map";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.03;
        marker.scale.y = 0.03;
        marker.points.reserve(cloud.size());
        marker.colors.reserve(cloud.size());
        for (const PointType &point : cloud.points)
        {
            geometry_msgs::msg::Point output;
            output.x = point.x;
            output.y = point.y;
            output.z = point.z;
            marker.points.push_back(output);

            // Fixed absolute-z coloring keeps equal heights visually
            // consistent across independently published scan markers.
            const float z_normalized = std::clamp(
                static_cast<float>((point.z + 5.0) / 10.0), 0.0F, 1.0F);
            std_msgs::msg::ColorRGBA color;
            color.r = std::clamp(1.5F - std::abs(4.0F * z_normalized - 3.0F),
                                 0.0F, 1.0F);
            color.g = std::clamp(1.5F - std::abs(4.0F * z_normalized - 2.0F),
                                 0.0F, 1.0F);
            color.b = std::clamp(1.5F - std::abs(4.0F * z_normalized - 1.0F),
                                 0.0F, 1.0F);
            color.a = 1.0F;
            marker.colors.push_back(color);
        }
        return marker;
    }

    PointCloudXYZI select_new_corrected_map_display_points(
        const PointCloudXYZI &cloud, bool reset)
    {
        if (reset)
        {
            corrected_map_display_voxels_.clear();
        }

        PointCloudXYZI selected;
        selected.reserve(cloud.size());
        for (const PointType &point : cloud.points)
        {
            if (corrected_map_display_voxels_.insert(
                    point.x, point.y, point.z))
            {
                selected.push_back(point);
            }
        }
        selected.width = static_cast<std::uint32_t>(selected.size());
        selected.height = 1;
        selected.is_dense = false;
        return selected;
    }

    void enqueue_mapping_scan_for_visualization(const MappingScan &scan)
    {
        if (!corrected_map_publish_enabled_ || !pubCorrectedMap_ ||
            !scan.points_world ||
            (pubCorrectedMap_->get_subscription_count() == 0 &&
             pubCorrectedMap_->get_intra_process_subscription_count() == 0))
        {
            return;
        }

        std::lock_guard<std::mutex> lock(corrected_map_queue_mutex_);
        if (corrected_map_queue_.size() >= 32U)
        {
            corrected_map_queue_.clear();
            corrected_map_publish_requested_.store(true);
        }
        corrected_map_queue_.push_back(scan);
    }

    void append_mapping_scan_display_points(
        const MappingScan &scan, PointCloudXYZI &display_cloud)
    {
        if (!scan.points_world)
        {
            return;
        }

        display_cloud.reserve(
            display_cloud.size() + scan.points_world->size());
        for (const auto &point : *scan.points_world)
        {
            if (!corrected_map_display_voxels_.insert(
                    point.x, point.y, point.z))
            {
                continue;
            }
            PointType converted;
            converted.x = point.x;
            converted.y = point.y;
            converted.z = point.z;
            converted.intensity = point.intensity;
            converted.normal_x = converted.normal_y = converted.normal_z = 0.0F;
            converted.curvature = 0.0F;
            display_cloud.push_back(converted);
        }
    }

    void accumulate_mapping_output(double timestamp)
    {
        // Retain history for on-demand map saving even when display and LC are off.
        if (!feats_undistort || feats_undistort->empty())
        {
            return;
        }

        PointCloudXYZI::Ptr dense_cloud(new PointCloudXYZI());
        dense_cloud->reserve(feats_undistort->size());
        auto dense_covariances = std::make_shared<std::vector<M3D>>();
        const auto &scan_covariances = p_imu->ScanMeasurementCovariances();
        const M3D sonar_rotation = state_point.rot.toRotationMatrix() * state_point.offset_R_L_I.toRotationMatrix();
        for (std::size_t i = 0; i < feats_undistort->size(); ++i)
        {
            const PointType &point_body = feats_undistort->points[i];
            if (sonar_adaptive_registration)
            {
                const auto cell = uwfl2::SonarVoxelCell(V3D(point_body.x, point_body.y, point_body.z), sonar_registration_leaf);
                if (!cell || sonar_accepted_cells.count(*cell) == 0) continue;
            }
            PointType point_world;
            RGBpointBodyToWorld(&point_body, &point_world);
            if (std::isfinite(point_world.x) && std::isfinite(point_world.y) &&
                std::isfinite(point_world.z))
            {
                dense_cloud->push_back(point_world);
                if (sonar_adaptive_registration)
                    dense_covariances->push_back(sonar_rotation * scan_covariances.at(i) * sonar_rotation.transpose());
            }
        }
        if (dense_cloud->empty())
        {
            return;
        }
        dense_cloud->width = static_cast<std::uint32_t>(dense_cloud->size());
        dense_cloud->height = 1;
        dense_cloud->is_dense = false;

        PointCloudXYZI compact_cloud;
        pcl::VoxelGrid<PointType> compact_filter;
        const float compact_leaf = static_cast<float>(filter_size_map_min);
        compact_filter.setLeafSize(compact_leaf, compact_leaf, compact_leaf);
        compact_filter.setInputCloud(dense_cloud);
        if (!sonar_adaptive_registration) compact_filter.filter(compact_cloud);
        auto compact_covariances = std::make_shared<std::vector<M3D>>();
        if (sonar_adaptive_registration)
        {
            std::vector<uwfl2::SonarReferencePoint> references;
            for (std::size_t i = 0; i < dense_cloud->size(); ++i)
            {
                const auto &point = dense_cloud->points[i];
                references.push_back({V3D(point.x, point.y, point.z), (*dense_covariances)[i]});
            }
            const auto compact = uwfl2::VoxelizeSonarPoints(references, filter_size_map_min, 100000);
            if (!compact.complete) throw std::runtime_error("Cannot preserve compact scan covariance");
            compact_cloud.clear();
            for (const auto &reference : compact.points)
            {
                PointType point;
                point.x = reference.position.x(); point.y = reference.position.y(); point.z = reference.position.z();
                compact_cloud.push_back(point);
                compact_covariances->push_back(reference.covariance);
            }
        }

        const auto copy_points = [](const PointCloudXYZI &cloud) {
            auto points =
                std::make_shared<std::vector<uwfl2::loop_closure::PointXYZI>>();
            points->reserve(cloud.size());
            for (const PointType &point : cloud.points)
            {
                points->push_back(
                    {point.x, point.y, point.z, point.intensity});
            }
            return points;
        };

        MappingScan scan;
        scan.timestamp = timestamp;
        scan.T_local_vehicle.rotation = Eigen::Quaterniond(
            state_point.rot.toRotationMatrix()).normalized();
        scan.T_local_vehicle.translation = state_point.pos;
        scan.points_world = copy_points(*dense_cloud);
        scan.compact_points_world = copy_points(compact_cloud);
        if (sonar_adaptive_registration)
        {
            scan.measurement_covariances_world = dense_covariances;
            scan.compact_measurement_covariances_world = compact_covariances;
        }
        {
            std::lock_guard<std::mutex> lock(mapping_output_mutex);
            // Keep scan-owned dense storage until a save or loop correction
            // needs one contiguous PCL cloud. Appending every scan directly to
            // a growing vector repeatedly copied the full mission map.
            pending_mapping_scans_.push_back(scan);
        }
        // The backend owns visualization filtering and DDS publication. The
        // front end only enqueues shared scan storage after committing it.
        enqueue_mapping_scan_for_visualization(scan);
    }

    MappingScanVector pending_mapping_scans() const
    {
        std::lock_guard<std::mutex> lock(mapping_output_mutex);
        return pending_mapping_scans_;
    }

    PointCloudXYZI corrected_mapping_snapshot() const
    {
        PointCloudXYZI snapshot;
        MappingScanVector pending;
        {
            std::lock_guard<std::mutex> lock(mapping_output_mutex);
            snapshot = *pcl_wait_pub;
            pending = pending_mapping_scans_;
        }
        append_mapping_scans(snapshot, pending);
        return snapshot;
    }

    void commit_mapping_submap(const MappingScanVector &scans)
    {
        if (scans.empty())
        {
            return;
        }
        std::lock_guard<std::mutex> lock(mapping_output_mutex);
        append_mapping_scans(*pcl_wait_pub, scans);
        pending_mapping_scans_.clear();
    }

private:
    struct CorrectedMappingOutput
    {
        PointCloudXYZI::Ptr cloud;
        MappingScanVector pending_scans;
    };

    CorrectedMappingOutput prepare_corrected_mapping_output(
        const uwfl2::loop_closure::ShadowMapResult &shadow_map,
        const uwfl2::loop_closure::Pose3d &latest_correction) const
    {
        CorrectedMappingOutput output;
        if (!shadow_map.corrected_history_points)
        {
            return output;
        }

        output.cloud = std::make_shared<PointCloudXYZI>();
        output.cloud->points.reserve(
            shadow_map.corrected_history_points->size());
        for (const auto &point : *shadow_map.corrected_history_points)
        {
            PointType converted;
            converted.x = point.x;
            converted.y = point.y;
            converted.z = point.z;
            converted.intensity = point.intensity;
            converted.normal_x = converted.normal_y = converted.normal_z = 0.0F;
            converted.curvature = 0.0F;
            output.cloud->points.push_back(converted);
        }

        const auto pending = pending_mapping_scans();
        output.pending_scans.reserve(pending.size());
        for (const MappingScan &scan : pending)
        {
            if (!scan.points_world)
            {
                continue;
            }
            const auto transform_points = [&](const auto &source) {
                if (!source)
                {
                    return std::shared_ptr<const std::vector<
                        uwfl2::loop_closure::PointXYZI>>{};
                }
                auto corrected_points = std::make_shared<std::vector<
                    uwfl2::loop_closure::PointXYZI>>();
                corrected_points->reserve(source->size());
                for (const auto &point : *source)
                {
                    const Eigen::Vector3d corrected =
                        latest_correction.rotation *
                            Eigen::Vector3d(point.x, point.y, point.z) +
                        latest_correction.translation;
                    if (corrected.allFinite())
                    {
                        corrected_points->push_back(
                            {static_cast<float>(corrected.x()),
                             static_cast<float>(corrected.y()),
                             static_cast<float>(corrected.z()),
                             point.intensity});
                    }
                }
                return std::shared_ptr<const std::vector<
                    uwfl2::loop_closure::PointXYZI>>(corrected_points);
            };
            MappingScan corrected_scan = scan;
            corrected_scan.T_local_vehicle = uwfl2::loop_closure::compose(
                latest_correction, scan.T_local_vehicle);
            corrected_scan.points_world = transform_points(scan.points_world);
            corrected_scan.compact_points_world =
                transform_points(scan.compact_points_world);
            const auto transform_covariances = [&](const auto &source) {
                if (!source) return std::shared_ptr<const std::vector<M3D>>{};
                auto covariances = std::make_shared<std::vector<M3D>>();
                const M3D rotation = latest_correction.rotation.toRotationMatrix();
                for (const auto &covariance : *source)
                    covariances->push_back(rotation * covariance * rotation.transpose());
                return std::shared_ptr<const std::vector<M3D>>(covariances);
            };
            corrected_scan.measurement_covariances_world = transform_covariances(scan.measurement_covariances_world);
            corrected_scan.compact_measurement_covariances_world = transform_covariances(scan.compact_measurement_covariances_world);
            output.pending_scans.push_back(std::move(corrected_scan));
        }
        output.cloud->width =
            static_cast<std::uint32_t>(output.cloud->points.size());
        output.cloud->height = 1;
        output.cloud->is_dense = false;
        return output;
    }

    void try_commit_loop_correction()
    {
        if (!loop_closure_)
        {
            return;
        }
        const auto candidate = loop_closure_->take_pending_correction();
        if (!candidate)
        {
            return;
        }
        const auto commit_started = std::chrono::steady_clock::now();
        const auto elapsed_ms = [&]() {
            return std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - commit_started)
                .count();
        };
        const auto reject = [&]() {
            loop_closure_->notify_correction_result(false, elapsed_ms());
        };
        if (!candidate->shadow_map || !candidate->registration ||
            !candidate->scan || !candidate->shadow_map->valid ||
            !candidate->registration->valid || !candidate->shadow_map->tree ||
            candidate->registration->scan_generation !=
                candidate->scan->scan_generation)
        {
            reject();
            return;
        }

        const auto graph = loop_closure_->graph_snapshot();
        if (candidate->registration->scan_generation != latest_scan_generation_ ||
            candidate->scan->active_tree_generation != active_tree_generation_ ||
            candidate->registration->graph_version != graph.version)
        {
            loop_closure_->request_registration_refresh(
                candidate->shadow_map, candidate->refresh_attempt + 1);
            reject();
            return;
        }

        const auto &registration = *candidate->registration;
        const auto correction = uwfl2::loop_closure::compose(
            registration.T_local_vehicle_registered,
            uwfl2::loop_closure::inverse(registration.T_local_vehicle_raw));
        if (!correction.finite())
        {
            reject();
            return;
        }

        state_ikfom corrected_state = kf.get_x();
        const Eigen::Matrix3d correction_rotation =
            correction.rotation.toRotationMatrix();
        corrected_state.pos =
            correction_rotation * corrected_state.pos + correction.translation;
        corrected_state.rot = SO3(
            correction_rotation * corrected_state.rot.toRotationMatrix());
        corrected_state.vel = correction_rotation * corrected_state.vel;
        const auto &original_state = kf.get_x();
        const bool protected_states_unchanged =
            corrected_state.offset_R_L_I.toRotationMatrix().isApprox(
                original_state.offset_R_L_I.toRotationMatrix(), 0.0) &&
            corrected_state.offset_T_L_I.isApprox(original_state.offset_T_L_I, 0.0) &&
            corrected_state.bg.isApprox(original_state.bg, 0.0) &&
            corrected_state.ba.isApprox(original_state.ba, 0.0) &&
            V3D(corrected_state.grav[0], corrected_state.grav[1],
                corrected_state.grav[2])
                .isApprox(V3D(original_state.grav[0], original_state.grav[1],
                              original_state.grav[2]),
                          0.0) &&
            corrected_state.b_dvl.isApprox(original_state.b_dvl, 0.0) &&
            corrected_state.b_pressure.isApprox(original_state.b_pressure, 0.0);
        if (!protected_states_unchanged)
        {
            reject();
            return;
        }

        uwfl2::loop_closure::Matrix6d correction_covariance =
            registration.covariance +
            registration.graph_anchor_covariance_position_rotation;
        correction_covariance =
            0.5 * (correction_covariance + correction_covariance.transpose());
        const auto transported_covariance =
            uwfl2::loop_closure::transport_uwfl2_covariance(
                kf.get_P(), correction_rotation);
        const auto corrected_covariance =
            uwfl2::loop_closure::apply_pose_covariance_update(
                transported_covariance, correction_covariance);
        if (!corrected_state.pos.allFinite() ||
            !corrected_state.vel.allFinite() ||
            !corrected_state.rot.toRotationMatrix().allFinite() ||
            !uwfl2::loop_closure::covariance27_is_valid(corrected_covariance))
        {
            reject();
            return;
        }

        PointVector registered_scan;
        registered_scan.reserve(candidate->scan->sonar_points->size());
        const auto T_local_sonar = uwfl2::loop_closure::compose(
            registration.T_local_vehicle_registered,
            candidate->scan->T_vehicle_sonar);
        for (const auto &point : *candidate->scan->sonar_points)
        {
            const Eigen::Vector3d world =
                T_local_sonar.rotation * Eigen::Vector3d(point.x, point.y, point.z) +
                T_local_sonar.translation;
            if (!world.allFinite())
            {
                continue;
            }
            PointType transformed;
            transformed.x = static_cast<float>(world.x());
            transformed.y = static_cast<float>(world.y());
            transformed.z = static_cast<float>(world.z());
            transformed.intensity = point.intensity;
            registered_scan.push_back(transformed);
        }
        if (registered_scan.empty())
        {
            reject();
            return;
        }

        const CorrectedMappingOutput corrected_mapping_output =
            prepare_corrected_mapping_output(*candidate->shadow_map, correction);

        const auto old_state = kf.get_x();
        const auto old_covariance = kf.get_P();
        const auto old_tree = ikdtree;
        const BoxPointType old_bounds = LocalMap_Points;
        const bool old_initialized = Localmap_Initialized;
        const std::uint64_t old_generation = active_tree_generation_;
        auto replacement_uncertainty = uwfl2::SonarMapUncertainty(sonar_map_point_budget);
        std::unique_ptr<uwfl2::SonarRegistrationView> replacement_view;
        try
        {
            auto replacement_tree = candidate->shadow_map->tree;
            if (sonar_adaptive_registration)
            {
                const auto &points = candidate->shadow_map->covariance_points;
                const auto &covariances = candidate->shadow_map->measurement_covariances;
                if (!points || !covariances || points->size() != covariances->size() ||
                    latest_loop_sonar_points_.size() != registered_scan.size())
                {
                    reject();
                    return;
                }
                std::vector<uwfl2::SonarReferencePoint> sources;
                sources.reserve(points->size() + registered_scan.size());
                for (std::size_t i = 0; i < points->size(); ++i)
                    sources.push_back({V3D((*points)[i].x, (*points)[i].y, (*points)[i].z), (*covariances)[i]});
                const M3D rotation = T_local_sonar.rotation.toRotationMatrix();
                for (std::size_t i = 0; i < registered_scan.size(); ++i)
                {
                    const auto &raw = candidate->scan->sonar_points->at(i);
                    if (!latest_loop_sonar_points_[i].position.isApprox(V3D(raw.x, raw.y, raw.z), 0))
                    {
                        reject();
                        return;
                    }
                    sources.push_back({V3D(registered_scan[i].x, registered_scan[i].y, registered_scan[i].z),
                        rotation * latest_loop_sonar_points_[i].covariance * rotation.transpose()});
                }
                if (!replacement_uncertainty.Record(sources))
                {
                    reject();
                    return;
                }
            }
            replacement_tree->Add_Points(registered_scan, true);
            if (sonar_adaptive_registration)
            {
                PointVector points;
                if (!replacement_tree->Snapshot_Points(points, sonar_map_point_budget))
                {
                    reject();
                    return;
                }
                std::vector<V3D> positions;
                for (const auto &point : points) positions.emplace_back(point.x, point.y, point.z);
                const auto references = replacement_uncertainty.Select(positions);
                replacement_view = std::make_unique<uwfl2::SonarRegistrationView>(2.0, sonar_map_point_budget, 4096);
                if (!references || !replacement_view->Replace(sonar_registration_generation + 1, *references))
                {
                    reject();
                    return;
                }
            }
            const BoxPointType replacement_bounds = replacement_tree->tree_range();
            auto mutable_covariance = corrected_covariance;
            kf.change_x(corrected_state);
            kf.change_P(mutable_covariance);
            ikdtree = std::move(replacement_tree);
            LocalMap_Points = replacement_bounds;
            Localmap_Initialized = true;
            active_tree_generation_ =
                std::max(old_generation + 1,
                         candidate->shadow_map->shadow_tree_generation);
            loop_closure_->notify_active_tree_generation(active_tree_generation_);
            state_point = kf.get_x();
            pos_lid = state_point.pos +
                      state_point.rot * state_point.offset_T_L_I;
            position_last = state_point.pos;
            loop_closure_->notify_correction_result(true, elapsed_ms());
            // Unregistered display geometry is not graph-corrected history.
            // Clear it after the atomic transaction, never reinterpret its anchors.
            provisional_map_reset_requested_ = true;
            RCLCPP_INFO(
                this->get_logger(),
                "Loop covariance posterior committed: position trace %.6g -> %.6g m^2, "
                "attitude trace %.6g -> %.6g rad^2.",
                transported_covariance.block<3, 3>(0, 0).trace(),
                corrected_covariance.block<3, 3>(0, 0).trace(),
                transported_covariance.block<3, 3>(3, 3).trace(),
                corrected_covariance.block<3, 3>(3, 3).trace());
            if (corrected_mapping_output.cloud)
            {
                {
                    std::lock_guard<std::mutex> lock(mapping_output_mutex);
                    pcl_wait_pub = corrected_mapping_output.cloud;
                    pending_mapping_scans_ =
                        corrected_mapping_output.pending_scans;
                }
                {
                    std::lock_guard<std::mutex> lock(corrected_map_queue_mutex_);
                    corrected_map_queue_.clear();
                }
                corrected_map_publish_requested_.store(true);
            }
            // The private 100 Hz predictor must restart from the same corrected
            // state and covariance as the front end after the atomic LC commit.
            std::lock_guard<std::mutex> prediction_lock(
                odometry_prediction_state_mutex_);
            reset_high_rate_odometry_prediction_locked();
            if (sonar_adaptive_registration)
            {
                sonar_map_uncertainty = std::move(replacement_uncertainty);
                sonar_registration_view = std::move(replacement_view);
                ++sonar_registration_generation;
            }
        }
        catch (...)
        {
            auto rollback_state = old_state;
            auto rollback_covariance = old_covariance;
            kf.change_x(rollback_state);
            kf.change_P(rollback_covariance);
            ikdtree = old_tree;
            LocalMap_Points = old_bounds;
            Localmap_Initialized = old_initialized;
            active_tree_generation_ = old_generation;
            state_point = kf.get_x();
            pos_lid = state_point.pos +
                      state_point.rot * state_point.offset_T_L_I;
            std::lock_guard<std::mutex> prediction_lock(
                odometry_prediction_state_mutex_);
            reset_high_rate_odometry_prediction_locked();
            reject();
        }
    }

    void maybe_publish_corrected_map()
    {
        if (!corrected_map_publish_enabled_ || !pubCorrectedMap_)
        {
            return;
        }
        const std::size_t subscriber_count =
            pubCorrectedMap_->get_subscription_count() +
            pubCorrectedMap_->get_intra_process_subscription_count();
        if (subscriber_count > corrected_map_subscriber_count_)
        {
            corrected_map_publish_requested_.store(true);
        }
        corrected_map_subscriber_count_ = subscriber_count;
        if (subscriber_count == 0)
        {
            std::lock_guard<std::mutex> lock(corrected_map_queue_mutex_);
            corrected_map_queue_.clear();
            return;
        }

        const bool requested = corrected_map_publish_requested_.exchange(false);
        if (requested)
        {
            const PointCloudXYZI snapshot = corrected_mapping_snapshot();
            const PointCloudXYZI display_snapshot =
                select_new_corrected_map_display_points(snapshot, true);
            next_corrected_map_marker_id_ = 1;
            visualization_msgs::msg::MarkerArray output;
            visualization_msgs::msg::Marker clear;
            clear.header.frame_id = "camera_init";
            clear.header.stamp = this->get_clock()->now();
            clear.action = visualization_msgs::msg::Marker::DELETEALL;
            output.markers.push_back(clear);
            if (!display_snapshot.empty())
            {
                output.markers.push_back(mapping_points_marker(
                    display_snapshot, 0, clear.header.stamp));
            }
            pubCorrectedMap_->publish(output);
            return;
        }

        // Drain and combine pending scans into one DDS message. At x1 this is
        // normally one sonar frame; accelerated replay avoids publishing a
        // separate MarkerArray for every sensor-time frame.
        PointCloudXYZI display_cloud;
        double latest_timestamp = -1.0;
        for (int drained = 0; drained < 32; ++drained)
        {
            MappingScan scan;
            {
                std::lock_guard<std::mutex> lock(corrected_map_queue_mutex_);
                if (corrected_map_queue_.empty())
                {
                    break;
                }
                scan = std::move(corrected_map_queue_.front());
                corrected_map_queue_.pop_front();
            }
            append_mapping_scan_display_points(scan, display_cloud);
            latest_timestamp = std::max(latest_timestamp, scan.timestamp);
        }
        if (!display_cloud.empty())
        {
            display_cloud.width =
                static_cast<std::uint32_t>(display_cloud.size());
            display_cloud.height = 1;
            display_cloud.is_dense = false;
            visualization_msgs::msg::MarkerArray output;
            const auto nanoseconds = static_cast<std::int64_t>(
                std::llround(latest_timestamp * 1e9));
            output.markers.push_back(mapping_points_marker(
                display_cloud, next_corrected_map_marker_id_++,
                rclcpp::Time(nanoseconds, RCL_ROS_TIME)));
            pubCorrectedMap_->publish(output);
        }
    }

    static geometry_msgs::msg::Pose pose_message(
        const uwfl2::loop_closure::Pose3d &pose)
    {
        geometry_msgs::msg::Pose message;
        const auto normalized = pose.normalized();
        message.position.x = normalized.translation.x();
        message.position.y = normalized.translation.y();
        message.position.z = normalized.translation.z();
        message.orientation.w = normalized.rotation.w();
        message.orientation.x = normalized.rotation.x();
        message.orientation.y = normalized.rotation.y();
        message.orientation.z = normalized.rotation.z();
        return message;
    }

    void publish_graph_paths()
    {
        if (!loop_closure_ || !raw_graph_path_pub_ || !optimized_graph_path_pub_)
        {
            return;
        }
        const auto graph = loop_closure_->graph_snapshot();
        nav_msgs::msg::Path raw_path;
        nav_msgs::msg::Path optimized_path;
        raw_path.header.frame_id = "camera_init";
        optimized_path.header.frame_id = "camera_init";
        raw_path.header.stamp = this->get_clock()->now();
        optimized_path.header.stamp = raw_path.header.stamp;
        raw_path.poses.reserve(graph.raw_poses.size());
        optimized_path.poses.reserve(graph.optimized_poses.size());
        for (std::size_t index = 0; index < graph.raw_poses.size(); ++index)
        {
            geometry_msgs::msg::PoseStamped raw;
            geometry_msgs::msg::PoseStamped optimized;
            raw.header.frame_id = raw_path.header.frame_id;
            optimized.header.frame_id = optimized_path.header.frame_id;
            const auto nanoseconds = static_cast<std::int64_t>(
                std::llround(graph.timestamps[index] * 1e9));
            raw.header.stamp = rclcpp::Time(nanoseconds, RCL_ROS_TIME);
            optimized.header.stamp = raw.header.stamp;
            raw.pose = pose_message(graph.raw_poses[index]);
            optimized.pose = pose_message(graph.optimized_poses[index]);
            raw_path.poses.push_back(std::move(raw));
            optimized_path.poses.push_back(std::move(optimized));
        }
        raw_graph_path_pub_->publish(raw_path);
        optimized_graph_path_pub_->publish(optimized_path);
    }

    void maybe_publish_loop_visualization()
    {
        if (!loop_closure_ || !loop_visualization_enabled_ ||
            !loop_markers_pub_)
        {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (now < next_loop_visualization_publish_)
        {
            return;
        }
        next_loop_visualization_publish_ = now + std::chrono::milliseconds(500);
        publish_graph_paths();

        const auto graph = loop_closure_->graph_snapshot();
        const auto events = loop_closure_->visualization_events();
        const auto stats = loop_closure_->stats();
        const auto stamp = this->get_clock()->now();
        visualization_msgs::msg::MarkerArray output;

        visualization_msgs::msg::Marker clear;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        output.markers.push_back(clear);

        const auto marker = [&](int id, int type, const std::string &name) {
            visualization_msgs::msg::Marker result;
            result.header.frame_id = "camera_init";
            result.header.stamp = stamp;
            result.ns = name;
            result.id = id;
            result.type = type;
            result.action = visualization_msgs::msg::Marker::ADD;
            result.pose.orientation.w = 1.0;
            return result;
        };
        auto raw_points = marker(1, visualization_msgs::msg::Marker::SPHERE_LIST,
                                 "raw_keyframes");
        raw_points.scale.x = raw_points.scale.y = raw_points.scale.z = 0.45;
        raw_points.color.r = 1.0F;
        raw_points.color.g = 0.55F;
        raw_points.color.a = 0.7F;
        auto optimized_points = marker(
            2, visualization_msgs::msg::Marker::SPHERE_LIST,
            "optimized_keyframes");
        optimized_points.scale.x = optimized_points.scale.y =
            optimized_points.scale.z = 0.55;
        optimized_points.color.g = 1.0F;
        optimized_points.color.b = 0.25F;
        optimized_points.color.a = 0.9F;
        for (std::size_t index = 0; index < graph.raw_poses.size(); ++index)
        {
            geometry_msgs::msg::Point raw;
            raw.x = graph.raw_poses[index].translation.x();
            raw.y = graph.raw_poses[index].translation.y();
            raw.z = graph.raw_poses[index].translation.z();
            raw_points.points.push_back(raw);
            geometry_msgs::msg::Point optimized;
            optimized.x = graph.optimized_poses[index].translation.x();
            optimized.y = graph.optimized_poses[index].translation.y();
            optimized.z = graph.optimized_poses[index].translation.z();
            optimized_points.points.push_back(optimized);
        }
        output.markers.push_back(std::move(raw_points));
        output.markers.push_back(std::move(optimized_points));

        auto proposed = marker(3, visualization_msgs::msg::Marker::LINE_LIST,
                               "proposed_loops");
        auto rejected = marker(4, visualization_msgs::msg::Marker::LINE_LIST,
                               "rejected_loops");
        auto accepted = marker(5, visualization_msgs::msg::Marker::LINE_LIST,
                               "accepted_loops");
        proposed.scale.x = rejected.scale.x = 0.12;
        accepted.scale.x = 0.24;
        proposed.color.r = proposed.color.g = 1.0F;
        proposed.color.a = 0.8F;
        rejected.color.r = 1.0F;
        rejected.color.a = 0.35F;
        accepted.color.g = 1.0F;
        accepted.color.a = 1.0F;
        const auto append_edge = [&](visualization_msgs::msg::Marker &lines,
                                     const auto &event) {
            const auto source = std::find(graph.ids.begin(), graph.ids.end(),
                                          event.source_id);
            const auto target = std::find(graph.ids.begin(), graph.ids.end(),
                                          event.target_id);
            if (source == graph.ids.end() || target == graph.ids.end())
            {
                return;
            }
            for (const auto iterator : {source, target})
            {
                const std::size_t index = static_cast<std::size_t>(
                    std::distance(graph.ids.begin(), iterator));
                geometry_msgs::msg::Point point;
                point.x = graph.optimized_poses[index].translation.x();
                point.y = graph.optimized_poses[index].translation.y();
                point.z = graph.optimized_poses[index].translation.z();
                lines.points.push_back(point);
            }
        };
        for (const auto &event : events)
        {
            append_edge(event.accepted ? accepted
                                       : (event.rejected ? rejected : proposed),
                        event);
        }
        output.markers.push_back(std::move(proposed));
        output.markers.push_back(std::move(rejected));
        output.markers.push_back(std::move(accepted));

        const auto registration = loop_closure_->registration_snapshot();
        if (registration && registration->valid)
        {
            auto correction = marker(6, visualization_msgs::msg::Marker::ARROW,
                                     "state_correction");
            correction.scale.x = 0.18;
            correction.scale.y = 0.42;
            correction.scale.z = 0.50;
            correction.color.r = 0.75F;
            correction.color.b = 1.0F;
            correction.color.a = 1.0F;
            geometry_msgs::msg::Point begin;
            begin.x = registration->T_local_vehicle_raw.translation.x();
            begin.y = registration->T_local_vehicle_raw.translation.y();
            begin.z = registration->T_local_vehicle_raw.translation.z();
            geometry_msgs::msg::Point end;
            end.x = registration->T_local_vehicle_registered.translation.x();
            end.y = registration->T_local_vehicle_registered.translation.y();
            end.z = registration->T_local_vehicle_registered.translation.z();
            correction.points = {begin, end};
            output.markers.push_back(std::move(correction));
        }

        auto status = marker(7, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
                             "loop_status");
        status.scale.z = 1.2;
        status.color.r = status.color.g = status.color.b = status.color.a = 1.0F;
        if (!graph.optimized_poses.empty())
        {
            status.pose.position.x = graph.optimized_poses.back().translation.x();
            status.pose.position.y = graph.optimized_poses.back().translation.y();
            status.pose.position.z = graph.optimized_poses.back().translation.z() + 2.0;
        }
        status.text = "UWFL2-LC | keyframes: " +
                      std::to_string(graph.node_count) + " | proposed: " +
                      std::to_string(stats.std_proposed) + " | accepted: " +
                      std::to_string(stats.std_accepted) + " | rejected: " +
                      std::to_string(stats.std_graph_rejected) + " | committed: " +
                      std::to_string(stats.corrections_committed);
        output.markers.push_back(std::move(status));
        loop_markers_pub_->publish(output);
    }

    void submit_loop_keyframe(double timestamp)
    {
        if (!loop_closure_ || !feats_down_body)
        {
            return;
        }
        const auto &state = kf.get_x();
        uwfl2::loop_closure::Pose3d T_local_vehicle;
        T_local_vehicle.rotation =
            Eigen::Quaterniond(state.rot.toRotationMatrix()).normalized();
        T_local_vehicle.translation = state.pos;

        uwfl2::loop_closure::Pose3d T_vehicle_sonar;
        T_vehicle_sonar.rotation =
            Eigen::Quaterniond(state.offset_R_L_I.toRotationMatrix()).normalized();
        T_vehicle_sonar.translation = state.offset_T_L_I;

        const auto pose_covariance =
            uwfl2::loop_closure::extract_graph_pose_covariance(
                kf.get_P(), state.rot.toRotationMatrix());
        PointVector supported_points;
        if (sonar_adaptive_registration)
        {
            latest_loop_sonar_points_.clear();
            for (std::size_t i = 0; i < sonar_scan_points.size(); ++i)
                if (point_selected_surf[i])
                {
                    supported_points.push_back(feats_down_body->points[i]);
                    latest_loop_sonar_points_.push_back(sonar_scan_points[i]);
                }
        }
        ++latest_scan_generation_;
        const auto scan_points = loop_closure_->notify_latest_scan(
            timestamp, latest_scan_generation_, active_tree_generation_,
            T_local_vehicle, T_vehicle_sonar,
            sonar_adaptive_registration ? supported_points : feats_down_body->points);
        if (sonar_adaptive_registration && !scan_points) latest_loop_sonar_points_.clear();
        if (!scan_points ||
            !loop_closure_->should_select_keyframe(
                timestamp, T_local_vehicle, scan_points->size()))
        {
            return;
        }
        const auto mapping_scans = pending_mapping_scans();
        const bool submitted = loop_closure_->try_submit_scan_history(
            timestamp, T_local_vehicle, pose_covariance, T_vehicle_sonar,
            scan_points, mapping_scans,
            active_tree_generation_);
        if (submitted)
        {
            commit_mapping_submap(mapping_scans);
        }
    }

    uwfl2::MappingInputBuffers input_buffers_;
    bool aux_reorder_waiting = false;
    double aux_reorder_target_time = -1.0;
    std::chrono::steady_clock::time_point aux_reorder_wait_start;
    AuxiliarySensorFusion aux_fusion_;
    std::string sonar_condition_ = "initializing";
    uwfl2::SonarAvailability sonar_availability_;
    uwfl2::SonarInputMetrics sonar_input_metrics_;
    std::unique_ptr<uwfl2::SonarVoxelController> sonar_voxel_controller_;
    double last_adaptive_sonar_time_ = -1;
    double last_missing_diagnostic_time_ = -1.0;
    double sonar_dx_position_ = 0.0, sonar_dx_rotation_ = 0.0;
    int last_sonar_geometry_rank_ = 0;
    std::chrono::steady_clock::time_point sonar_processing_start_ = std::chrono::steady_clock::now();
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sonar_diagnostics_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr navigation_health_pub_;
    rclcpp::TimerBase::SharedPtr navigation_health_timer_;
    uwfl2::ImuObservationCalendar imu_observation_calendar_;
    uwfl2::SonarRecovery sonar_recovery_;
    std::deque<ProvisionalScan> provisional_scans_;
    bool provisional_map_reset_requested_ = false;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr provisional_map_pub_;
    std::size_t imu_observation_count_ = 0;
    std::size_t aux_considered_count_ = 0;
    std::size_t aux_applied_count_ = 0;
    std::size_t late_aux_count_ = 0;
    double last_dvl_applied_time_ = -1.0;
    std::unique_ptr<uwfl2::loop_closure::LoopClosureManager> loop_closure_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        pubCorrectedMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_graph_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimized_graph_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        loop_markers_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr initial_pose_tf_sub_;
    rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;
    rclcpp::CallbackGroup::SharedPtr lidar_callback_group_;
    rclcpp::CallbackGroup::SharedPtr processing_callback_group_;
    rclcpp::CallbackGroup::SharedPtr prediction_callback_group_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    static constexpr std::size_t kInitialPoseSampleCount = 20;
    static constexpr double kInitialPoseWaitSeconds = 5.0;
    static constexpr double kInitialPoseDuplicateTranslation = 1e-7;
    static constexpr double kInitialPoseDuplicateAngle = 1e-6;
    static constexpr double kInitialPoseInlierTranslation = 1.0;
    static constexpr double kInitialPoseInlierAngle =
        0.2617993877991494;  // 15 degrees
    std::mutex initial_pose_mutex_;
    bool initial_world_pose_ready_ = false;
    bool initial_pose_wait_started_ = false;
    std::chrono::steady_clock::time_point initial_pose_wait_start_wall_{};
    double initial_pose_first_sensor_time_ =
        std::numeric_limits<double>::quiet_NaN();
    double initial_pose_latest_sensor_time_ =
        std::numeric_limits<double>::quiet_NaN();
    std::size_t initial_pose_sample_count_ = 0;
    std::size_t initial_pose_raw_sample_count_ = 0;
    std::size_t initial_pose_repeated_sample_count_ = 0;
    std::vector<std::array<double, 7>> initial_pose_samples_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr corrected_map_timer_;
    rclcpp::TimerBase::SharedPtr prediction_timer_;
    rclcpp::CallbackGroup::SharedPtr backend_callback_group_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;

    bool corrected_map_publish_enabled_ = false;
    std::string corrected_map_topic_ = "/uwfl2/corrected_map";
    std::string sonar_frame_id_;
    bool empty_sonar_frame_warned_ = false;
    std::atomic<bool> corrected_map_publish_requested_{false};
    std::size_t corrected_map_subscriber_count_ = 0;
    std::int32_t next_corrected_map_marker_id_ = 1;
    std::mutex corrected_map_queue_mutex_;
    MappingScanQueue corrected_map_queue_;
    uwfl2::CorrectedMapDisplayVoxels corrected_map_display_voxels_{0.03};
    MainEkf odometry_prediction_kf_;
    Eigen::Matrix<double, process_noise_ikfom::DOF,
                  process_noise_ikfom::DOF> odometry_prediction_Q_ =
        process_noise_cov();
    std::mutex odometry_prediction_state_mutex_;
    double odometry_prediction_acceleration_scale_ = 1.0;
    double odometry_prediction_time_ = -1.0;
    double odometry_prediction_correction_time_ = -1.0;
    // A queued observation must be applied by the live filter before prediction
    // continues beyond it; callback bursts are not sensor-free time intervals.
    double odometry_prediction_observation_time_ = -1.0;
    double next_odometry_prediction_publish_time_ = -1.0;
    double last_odometry_prediction_published_time_ = -1.0;
    std::chrono::steady_clock::time_point odometry_rate_window_wall_time_{};
    double odometry_rate_window_sensor_time_ = -1.0;
    bool coalesce_odometry_output_ = false;
    bool odometry_prediction_ready_ = false;
    bool loop_visualization_enabled_ = false;
    std::chrono::steady_clock::time_point next_loop_visualization_publish_{};
    bool aux_timeline_started_ = false;
    std::uint64_t active_tree_generation_ = 0;
    std::uint64_t latest_scan_generation_ = 0;
    std::vector<uwfl2::SonarReferencePoint> latest_loop_sonar_points_;
    MappingScanVector pending_mapping_scans_;
    int effect_feat_num = 0;
    double deltaT, deltaR;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[state_ikfom::DOF] = {0.001};
};

std::shared_ptr<rclcpp::Node> MakeLaserMappingNode(const rclcpp::NodeOptions &options)
{
    return std::make_shared<LaserMappingNode>(options);
}

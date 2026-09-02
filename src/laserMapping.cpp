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
#include <atomic>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "IMU_Processing.hpp"
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
#include <std_srvs/srv/trigger.hpp>
#include "fast_lio/srv/inject_loop.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "auxiliary_sensor_fusion.hpp"
#include "loop_closure/loop_closure_manager.hpp"
#include "loop_closure/state_transport.hpp"
#include "lidar_scan_quality.hpp"
#include "observability_manager.hpp"
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV_DEFAULT (0.001)
double LASER_POINT_COV_XY = LASER_POINT_COV_DEFAULT;
double LASER_POINT_COV_Z = LASER_POINT_COV_DEFAULT;

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   pcd_save_en = false, path_en = true;
/**************************/

float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic, world_frame;

double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
V3D imu_gyro_scale(1.0, 1.0, 1.0);  // per-axis multiplicative correction for raw gyro (e.g. 1/0.74 for a known scale error)
double init_b_gyr_cov = 0.0001, init_b_acc_cov = 0.001, init_grav_cov = 0.00001;
double init_b_dvl_cov = 1e-8, init_b_pressure_cov = 1e4;
bool noiseless_imu = false;
double imu_orientation_cov = 3.0461742e-6;  // (0.1 deg)^2
double imu_orientation_gate_sigma = 0.0;
bool imu_orientation_ref_ready = false;
Eigen::Quaterniond imu_orientation_ref = Eigen::Quaterniond::Identity();
bool accel_attitude_ref_ready = false;
double accel_attitude_cov = 1.2184697e-3;  // (2 deg)^2
double accel_attitude_norm_gate = 2.0;
std::deque<std::pair<double, V3D>> accel_attitude_window;
double accel_attitude_last_stamp = -1.0;
double accel_attitude_last_update_stamp = -1.0;
ObservabilityManager obs_manager;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
double last_processed_time = -1.0, lidar_timeout = 0.25, imu_rate_hz = 100.0;
double odometry_publish_rate_hz = 100.0;
double gravity_m_s2 = G_m_s2;
int    effct_feat_num = 0, scan_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
int    lidar_update_max_effective_features = 0;
int    minimum_scan_points = 5, minimum_effective_features = 1;
uwfl2::LidarScanQualityPolicy lidar_scan_quality;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool    is_first_lidar = true;
bool auxiliary_fusion_enabled = false;

constexpr double AUX_SENSOR_REORDER_WINDOW_SEC = 0.02;
constexpr auto AUX_SENSOR_REORDER_WALL_GRACE = std::chrono::milliseconds(20);
bool aux_reorder_waiting = false;
double aux_reorder_target_time = -1.0;
std::chrono::steady_clock::time_point aux_reorder_wait_start;

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

std::atomic<std::uint64_t> lidar_callbacks_received{0};
std::atomic<std::uint64_t> imu_callbacks_received{0};
std::atomic<std::uint64_t> lidar_timestamp_regressions{0};
std::atomic<std::uint64_t> imu_timestamp_regressions{0};
std::atomic<std::uint64_t> lidar_buffer_messages_cleared{0};
std::atomic<std::uint64_t> imu_buffer_messages_cleared{0};
std::atomic<std::uint64_t> stale_lidar_scans_discarded{0};
std::atomic<std::uint64_t> imu_only_packets_processed{0};

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

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

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

void SigHandle(int sig)
{
    (void)sig;
    flg_exit = true;
    sig_buffer.notify_all();
    rclcpp::shutdown();
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
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

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree->acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
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
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree->Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg) 
{
    ++lidar_callbacks_received;
    const double cur_time = get_time_sec(msg->header.stamp);
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    // Point-cloud conversion is intentionally outside the shared sensor-buffer
    // lock and in its own callback group. Large sonar messages must not starve
    // IMU and auxiliary callbacks during accelerated rosbag replay.
    p_pre->process(msg, ptr);

    std::lock_guard<std::mutex> lock(mtx_buffer);
    scan_count ++;
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        ++lidar_timestamp_regressions;
        lidar_buffer_messages_cleared.fetch_add(lidar_buffer.size());
        lidar_buffer.clear();
    }
    if (is_first_lidar)
    {
        is_first_lidar = false;
    }

    lidar_buffer.push_back(ptr);
    time_buffer.push_back(cur_time);
    last_timestamp_lidar = cur_time;
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    ++imu_callbacks_received;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    msg->angular_velocity.x *= imu_gyro_scale.x();
    msg->angular_velocity.y *= imu_gyro_scale.y();
    msg->angular_velocity.z *= imu_gyro_scale.z();

    double timestamp = get_time_sec(msg->header.stamp);

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ++imu_timestamp_regressions;
        imu_buffer_messages_cleared.fetch_add(imu_buffer.size());
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
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
    const double requested_period = odometry_publish_rate_hz > 0.0
                                        ? 1.0 / odometry_publish_rate_hz
                                        : lidar_timeout;
    // Target the requested cadence and finish on the nearest real IMU sample.
    return std::max(expected_imu_period(),
                    std::min(lidar_timeout, requested_period));
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

bool sync_packages(MeasureGroup &meas)
{
    while (!lidar_pushed && !lidar_buffer.empty() && last_processed_time > 0.0 &&
           time_buffer.front() < last_processed_time - 1e-4)
    {
        ++stale_lidar_scans_discarded;
        lidar_buffer.pop_front();
        time_buffer.pop_front();
    }

    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }
    if (!auxiliary_callbacks_ready(lidar_end_time, last_timestamp_imu))
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

bool sync_imu_only_packages(MeasureGroup &meas)
{
    // An empty topic explicitly selects INS mode. A misspelled/unavailable
    // topic also enters INS mode, and an established LiDAR stream enters the
    // same propagation path only after its configured timeout. Normal scan
    // intervals remain scan-bounded. The auxiliary reorder window below keeps
    // fallback propagation behind the newest sensor time, allowing an on-time
    // scan callback to take priority before its timestamp is crossed.
    const bool explicit_ins_mode = lid_topic.empty();
    const bool no_lidar_received = is_first_lidar;
    if (!lidar_buffer.empty() || lidar_pushed || imu_buffer.empty()) {
        return false;
    }

    const double latest_imu_time = get_time_sec(imu_buffer.back()->header.stamp);
    const double first_imu_time = get_time_sec(imu_buffer.front()->header.stamp);
    const bool lidar_timed_out =
        last_timestamp_lidar > 0.0 &&
        latest_imu_time - last_timestamp_lidar >= lidar_timeout;
    const bool initial_lidar_timed_out =
        no_lidar_received && latest_imu_time - first_imu_time >= lidar_timeout;
    if (!explicit_ins_mode && !lidar_timed_out && !initial_lidar_timed_out)
    {
        return false;
    }
    if (last_processed_time > 0.0 && latest_imu_time <= last_processed_time + 1e-6) {
        return false;
    }
    meas.imu.clear();
    meas.lidar.reset(new PointCloudXYZI());
    double packet_begin_time = last_processed_time > 0.0 ? last_processed_time : first_imu_time;
    if (last_processed_time > 0.0 && first_imu_time > packet_begin_time + expected_imu_timeout())
    {
        packet_begin_time = first_imu_time;
    }
    const double target_packet_end_time = packet_begin_time + imu_only_packet_duration();
    // Once a LiDAR stream has been established, retain one timeout of IMU
    // history during an outage. A returning scan can then still be fused at
    // its sensor timestamp instead of being discarded as out of sequence.
    const double propagation_horizon =
        (!explicit_ins_mode && last_timestamp_lidar > 0.0)
            ? latest_imu_time - lidar_timeout
            : latest_imu_time;
    if (propagation_horizon < target_packet_end_time - 1e-6)
    {
        return false;
    }
    if (!auxiliary_callbacks_ready(target_packet_end_time, latest_imu_time))
    {
        return false;
    }

    meas.lidar_beg_time = packet_begin_time;
    double last_included_imu_time = packet_begin_time;

    while (!imu_buffer.empty())
    {
        const double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        if (last_processed_time > 0.0 && imu_time <= last_processed_time + 1e-6)
        {
            imu_buffer.pop_front();
            continue;
        }
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
        last_included_imu_time = imu_time;
        // Select the real IMU sample nearest the requested output epoch.
        // Without the half-sample tolerance, a 200.078 Hz stream misses the
        // exact 10 ms boundary by a few microseconds and is decimated by
        // three samples (about 66.7 Hz) instead of two (about 100 Hz).
        if (imu_time >= target_packet_end_time - 0.5 * expected_imu_period()) break;
    }

    if (meas.imu.empty())
    {
        return false;
    }
    meas.lidar_end_time = last_included_imu_time;
    lidar_end_time = meas.lidar_end_time;
    return true;
}

int process_increments = 0;
std::size_t map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
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

    double st_time = omp_get_wtime();
    add_point_size = ikdtree->Add_Points(PointToAdd, true);
    ikdtree->Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
    return static_cast<std::size_t>(add_point_size);
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI());
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
std::mutex mapping_output_mutex;

void publish_effect_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void save_to_pcd(const PointCloudXYZI &map)
{
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(map_file_path, map);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
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

const char *g_publish_mode = "init";

bool apply_imu_orientation_update(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
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
    state_ikfom state = kf.get_x();
    const M3D R_err =
        state.rot.toRotationMatrix().transpose() * q_relative.toRotationMatrix();
    const V3D residual = Log(R_err);

    MainEkf::cov P = kf.get_P();
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
    if (imu_orientation_gate_sigma > 0.0)
    {
        const double nis = residual.dot(ldlt.solve(residual));
        if (!std::isfinite(nis) ||
            nis > 3.0 * imu_orientation_gate_sigma * imu_orientation_gate_sigma)
        {
            return false;
        }
    }

    const Eigen::Matrix<double, state_ikfom::DOF, 3> K =
        P * H.transpose() * ldlt.solve(M3D::Identity());
    const Eigen::Matrix<double, state_ikfom::DOF, 1> dx = K * residual;
    if (!dx.allFinite())
    {
        return false;
    }
    state.boxplus(dx);

    const MainEkf::cov I = MainEkf::cov::Identity();
    const MainEkf::cov KH = K * H;
    MainEkf::cov P_new =
        ((I - KH) * P * (I - KH).transpose() + K * R * K.transpose()).eval();
    P_new = ((P_new + P_new.transpose()) * 0.5).eval();
    kf.change_x(state);
    kf.change_P(P_new);
    return true;
}

bool apply_accel_attitude_update(
    const std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> &imu_msgs)
{
    constexpr double averaging_window_s = 0.2;
    for (const auto &imu_msg : imu_msgs)
    {
        if (!imu_msg)
        {
            continue;
        }
        const double timestamp = get_time_sec(imu_msg->header.stamp);
        if (timestamp <= accel_attitude_last_stamp + 1e-9)
        {
            continue;
        }
        accel_attitude_window.emplace_back(
            timestamp,
            V3D(imu_msg->linear_acceleration.x,
                imu_msg->linear_acceleration.y,
                imu_msg->linear_acceleration.z));
        accel_attitude_last_stamp = timestamp;
    }
    if (accel_attitude_window.empty())
    {
        return false;
    }
    const double newest_stamp = accel_attitude_window.back().first;
    if (accel_attitude_last_update_stamp > 0.0 &&
        newest_stamp - accel_attitude_last_update_stamp <
            averaging_window_s - 1e-6)
    {
        return false;
    }
    const double oldest_allowed =
        accel_attitude_window.back().first - averaging_window_s;
    while (accel_attitude_window.size() > 1 &&
           accel_attitude_window.front().first < oldest_allowed)
    {
        accel_attitude_window.pop_front();
    }

    V3D acc_meas = V3D::Zero();
    for (const auto &sample : accel_attitude_window)
    {
        acc_meas += sample.second;
    }
    acc_meas /= static_cast<double>(accel_attitude_window.size());
    accel_attitude_last_update_stamp = newest_stamp;

    state_ikfom state = kf.get_x();
    acc_meas -= V3D(state.ba[0], state.ba[1], state.ba[2]);
    const double acc_norm = acc_meas.norm();
    if (!std::isfinite(acc_norm) || acc_norm < 1e-6 ||
        std::abs(acc_norm - gravity_m_s2) > accel_attitude_norm_gate)
    {
        return false;
    }
    const bool initialize_from_filter_gravity = !accel_attitude_ref_ready;
    accel_attitude_ref_ready = true;
    V3D grav_local(state.grav[0], state.grav[1], state.grav[2]);
    if (grav_local.norm() < 1e-6)
    {
        grav_local = V3D(0.0, 0.0, -gravity_m_s2);
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

    MainEkf::cov P = kf.get_P();
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
    state.boxplus(dx);

    const MainEkf::cov I = MainEkf::cov::Identity();
    const MainEkf::cov KH = K * H;
    MainEkf::cov P_new =
        ((I - KH) * P * (I - KH).transpose() + K * R * K.transpose()).eval();
    P_new = ((P_new + P_new.transpose()) * 0.5).eval();
    kf.change_x(state);
    kf.change_P(P_new);
    return true;
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    set_posestamp(odomAftMapped.pose);

    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }
    pubOdomAftMapped->publish(odomAftMapped);

    geometry_msgs::msg::TransformStamped trans;
    trans.header.frame_id = "camera_init";
    trans.child_frame_id = "body";
    trans.header.stamp = get_ros_time(lidar_end_time);
    trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
    trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    tf_br->sendTransform(trans);
}

void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time); // ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 

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
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    if (!lidar_scan_quality.features_are_sufficient(
            static_cast<std::size_t>(effct_feat_num)))
    {
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

    solve_time += omp_get_wtime() - solve_start_;
}

class LaserMappingNode : public rclcpp::Node
{
public:
    LaserMappingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("laser_mapping", options)
    {
        this->declare_parameter<bool>("publish.path_en", true);
        this->declare_parameter<bool>("publish.effect_map_en", false);
        this->declare_parameter<bool>("publish.map_en", false);
        this->declare_parameter<double>("publish.corrected_map_interval_s", 0.2);
        this->declare_parameter<int>("max_iteration", 4);
        this->declare_parameter<string>("map_file_path", "");
        this->declare_parameter<string>("common.lid_topic", "/points_raw");
        this->declare_parameter<string>("common.imu_topic", "/imu/data");
        this->declare_parameter<string>("common.world_frame", "world");
        this->declare_parameter<vector<double>>("common.world_to_camera_init_T", {0.0, 0.0, 0.0});
        this->declare_parameter<vector<double>>("common.world_to_camera_init_R",
                             {1.0, 0.0, 0.0,
                              0.0, 1.0, 0.0,
                                                 0.0, 0.0, 1.0});
        this->declare_parameter<double>("common.odometry_publish_rate_hz", 100.0);
        this->declare_parameter<double>("common.imu_rate_hz", 100.0);
        this->declare_parameter<double>("common.lidar_timeout", 0.25);
        this->declare_parameter<double>("common.gravity_m_s2", G_m_s2);
        this->declare_parameter<double>("filter_size_corner", 0.5);
        this->declare_parameter<double>("filter_size_surf", 0.5);
        this->declare_parameter<double>("filter_size_map", 0.5);
        this->declare_parameter<double>("cube_side_length", 200.);
        this->declare_parameter<float>("mapping.det_range", 300.);
        this->declare_parameter<double>("mapping.fov_degree", 180.);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        this->declare_parameter<double>("mapping.init_b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.init_b_acc_cov", 0.001);
        this->declare_parameter<double>("mapping.init_grav_cov", 0.00001);
        this->declare_parameter<double>("mapping.imu_orientation_cov", 3.0461742e-6);
        this->declare_parameter<double>("mapping.imu_orientation_gate_sigma", 0.0);
        this->declare_parameter<double>("mapping.accel_attitude_cov", 1.2184697e-3);
        this->declare_parameter<double>("mapping.accel_attitude_norm_gate", 2.0);
        this->declare_parameter<bool>("mapping.noiseless_imu", false);
        this->declare_parameter<vector<double>>("mapping.imu_gyro_scale", {1.0, 1.0, 1.0});
        ObservabilityManager::declare_parameters(*this);
        this->declare_parameter<double>("mapping.laser_point_cov", LASER_POINT_COV_DEFAULT);
        this->declare_parameter<int>("mapping.minimum_scan_points", 5);
        this->declare_parameter<int>("mapping.minimum_effective_features", 1);
        this->declare_parameter<double>("preprocess.blind", 0.01);
        this->declare_parameter<int>("preprocess.scan_line", 16);
        this->declare_parameter<int>("preprocess.timestamp_unit", US);
        this->declare_parameter<int>("preprocess.scan_rate", 10);
        this->declare_parameter<int>("point_filter_num", 2);
        this->declare_parameter<bool>("feature_extract_enable", false);
        this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
        this->declare_parameter<int>("pcd_save.interval", -1);
        this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
        this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());
        this->declare_parameter<bool>("loop_closure.enable", false);
        this->declare_parameter<bool>("loop_closure.automatic_detection_enable", false);
        this->declare_parameter<bool>("loop_closure.visualization_enable", false);
        this->declare_parameter<double>("loop_closure.keyframe_translation_m", 1.0);
        this->declare_parameter<double>("loop_closure.keyframe_rotation_deg", 10.0);
        this->declare_parameter<double>("loop_closure.keyframe_minimum_interval_s", 0.5);
        this->declare_parameter<double>("loop_closure.keyframe_maximum_interval_s", 5.0);
        this->declare_parameter<int>("loop_closure.keyframe_minimum_points", 20);
        this->declare_parameter<int>("loop_closure.queue_capacity", 8);
        this->declare_parameter<string>("loop_closure.diagnostics_directory", "");
        this->declare_parameter<double>("loop_closure.prior_rotation_sigma_rad", 1e-4);
        this->declare_parameter<double>("loop_closure.prior_translation_sigma_m", 1e-4);
        this->declare_parameter<double>("loop_closure.odometry_rotation_variance_floor", 1e-8);
        this->declare_parameter<double>("loop_closure.odometry_translation_variance_floor", 1e-6);
        this->declare_parameter<int>("loop_closure.loop_minimum_keyframe_separation", 5);
        this->declare_parameter<double>("loop_closure.loop_maximum_initial_translation_error_m", 10.0);
        this->declare_parameter<double>("loop_closure.loop_maximum_initial_rotation_error_deg", 45.0);
        this->declare_parameter<double>("loop_closure.loop_maximum_initial_nis", 12.592);
        this->declare_parameter<double>("loop_closure.loop_maximum_pose_correction_translation_m", 20.0);
        this->declare_parameter<double>("loop_closure.loop_maximum_pose_correction_rotation_deg", 45.0);
        this->declare_parameter<double>("loop_closure.corrected_map_radius_m", 80.0);
        this->declare_parameter<int>("loop_closure.corrected_map_maximum_keyframes", 1000);
        this->declare_parameter<int>("loop_closure.corrected_map_maximum_input_points", 3000000);
        this->declare_parameter<int>("loop_closure.registration_maximum_iterations", 12);
        this->declare_parameter<int>("loop_closure.registration_minimum_effective_points", 30);
        this->declare_parameter<double>("loop_closure.registration_maximum_neighbor_distance_m", 2.25);
        this->declare_parameter<double>("loop_closure.registration_plane_threshold_m", 0.12);
        this->declare_parameter<double>("loop_closure.registration_maximum_translation_m", 2.0);
        this->declare_parameter<double>("loop_closure.registration_maximum_rotation_deg", 15.0);
        this->declare_parameter<int>("loop_closure.std_minimum_keyframe_separation", 20);
        this->declare_parameter<double>(
            "loop_closure.std_minimum_loop_duration_s", 0.0);
        this->declare_parameter<double>("loop_closure.std_voxel_size_m", 0.6);
        this->declare_parameter<int>("loop_closure.std_minimum_triangle_matches", 5);
        this->declare_parameter<int>("loop_closure.std_minimum_ransac_inliers", 5);
        this->declare_parameter<double>("loop_closure.std_overlap_minimum", 0.20);
        this->declare_parameter<double>("loop_closure.std_overlap_distance_m", 0.35);
        this->declare_parameter<int>("loop_closure.std_required_confirmations", 2);
        this->declare_parameter<double>(
            "loop_closure.std_single_detection_overlap_minimum", 0.80);
        this->declare_parameter<double>(
            "loop_closure.std_accepted_loop_cooldown_s", 0.0);
        this->declare_parameter<double>(
            "loop_closure.std_confirmation_translation_m", 1.0);
        this->declare_parameter<double>(
            "loop_closure.std_confirmation_rotation_deg", 10.0);
        aux_fusion_.declare_parameters(*this);

        this->get_parameter_or<bool>("publish.path_en", path_en, true);
        this->get_parameter_or<bool>("publish.effect_map_en", effect_pub_en, false);
        this->get_parameter_or<bool>("publish.map_en", map_pub_en, false);
        this->get_parameter_or<double>("publish.corrected_map_interval_s",
                                       corrected_map_interval_s_, 0.2);
        corrected_map_interval_s_ = std::max(0.05, corrected_map_interval_s_);
        this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
        this->get_parameter_or<string>("map_file_path", map_file_path, "");
        this->get_parameter_or<string>("common.lid_topic", lid_topic, "/points_raw");
        this->get_parameter_or<string>("common.imu_topic", imu_topic,"/imu/data");
        this->get_parameter_or<string>("common.world_frame", world_frame, "world");
        std::vector<double> world_to_camera_init_T;
        std::vector<double> world_to_camera_init_R;
        this->get_parameter_or<vector<double>>("common.world_to_camera_init_T",
                            world_to_camera_init_T,
                            {0.0, 0.0, 0.0});
        this->get_parameter_or<vector<double>>("common.world_to_camera_init_R",
                            world_to_camera_init_R,
                            {1.0, 0.0, 0.0,
                             0.0, 1.0, 0.0,
                             0.0, 0.0, 1.0});
        this->get_parameter_or<double>("common.imu_rate_hz", imu_rate_hz, 100.0);
        this->get_parameter_or<double>("common.lidar_timeout", lidar_timeout, 0.25);
        this->get_parameter_or<double>("common.odometry_publish_rate_hz",
                                       odometry_publish_rate_hz, 100.0);
        this->get_parameter_or<double>("common.gravity_m_s2", gravity_m_s2, G_m_s2);
        this->get_parameter_or<double>("filter_size_corner",filter_size_corner_min,0.5);
        this->get_parameter_or<double>("filter_size_surf",filter_size_surf_min,0.5);
        this->get_parameter_or<double>("filter_size_map",filter_size_map_min,0.5);
        this->get_parameter_or<double>("cube_side_length",cube_len,200.f);
        this->get_parameter_or<float>("mapping.det_range",DET_RANGE,300.f);
        this->get_parameter_or<double>("mapping.fov_degree",fov_deg,180.f);
        this->get_parameter_or<double>("mapping.gyr_cov",gyr_cov,0.1);
        this->get_parameter_or<double>("mapping.acc_cov",acc_cov,0.1);
        this->get_parameter_or<double>("mapping.b_gyr_cov",b_gyr_cov,0.0001);
        this->get_parameter_or<double>("mapping.b_acc_cov",b_acc_cov,0.0001);
        this->get_parameter_or<double>("mapping.init_b_gyr_cov",init_b_gyr_cov,0.0001);
        this->get_parameter_or<double>("mapping.init_b_acc_cov",init_b_acc_cov,0.001);
        this->get_parameter_or<double>("mapping.init_grav_cov",init_grav_cov,0.00001);
        this->get_parameter_or<double>("mapping.imu_orientation_cov", imu_orientation_cov, 3.0461742e-6);
        this->get_parameter_or<double>("mapping.imu_orientation_gate_sigma", imu_orientation_gate_sigma, 0.0);
        this->get_parameter_or<double>("mapping.accel_attitude_cov", accel_attitude_cov, 1.2184697e-3);
        this->get_parameter_or<double>("mapping.accel_attitude_norm_gate", accel_attitude_norm_gate, 2.0);
        this->get_parameter_or<bool>("mapping.noiseless_imu",noiseless_imu,false);
        {
            vector<double> scale_vec = {1.0, 1.0, 1.0};
            this->get_parameter_or<vector<double>>("mapping.imu_gyro_scale", scale_vec, {1.0, 1.0, 1.0});
            if (scale_vec.size() == 3)
                imu_gyro_scale = V3D(scale_vec[0], scale_vec[1], scale_vec[2]);
            else
                RCLCPP_WARN(this->get_logger(), "mapping.imu_gyro_scale must have 3 values. Using [1,1,1].");
        }
        double legacy_laser_point_cov = LASER_POINT_COV_DEFAULT;
        this->get_parameter_or<double>("mapping.laser_point_cov", legacy_laser_point_cov, double(LASER_POINT_COV_DEFAULT));
        this->declare_parameter<double>("mapping.laser_point_cov_xy", legacy_laser_point_cov);
        this->declare_parameter<double>("mapping.laser_point_cov_z", legacy_laser_point_cov);
        this->get_parameter_or<double>("mapping.laser_point_cov_xy", LASER_POINT_COV_XY, legacy_laser_point_cov);
        this->get_parameter_or<double>("mapping.laser_point_cov_z", LASER_POINT_COV_Z, legacy_laser_point_cov);
        LASER_POINT_COV_XY = std::max(1e-12, LASER_POINT_COV_XY);
        LASER_POINT_COV_Z = std::max(1e-12, LASER_POINT_COV_Z);
        this->get_parameter_or<int>("mapping.minimum_scan_points",
                                    minimum_scan_points, 5);
        this->get_parameter_or<int>("mapping.minimum_effective_features",
                                    minimum_effective_features, 1);
        minimum_scan_points = std::max(1, minimum_scan_points);
        minimum_effective_features = std::max(1, minimum_effective_features);
        lidar_scan_quality = uwfl2::LidarScanQualityPolicy(
            static_cast<std::size_t>(minimum_scan_points),
            static_cast<std::size_t>(minimum_effective_features));
        imu_orientation_cov = std::max(1e-12, imu_orientation_cov);
        imu_orientation_gate_sigma = std::max(0.0, imu_orientation_gate_sigma);
        accel_attitude_cov = std::max(1e-8, accel_attitude_cov);
        accel_attitude_norm_gate = std::max(0.0, accel_attitude_norm_gate);
        this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 0.01);
        this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);
        this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->time_unit, US);
        this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
        this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 2);
        this->get_parameter_or<bool>("feature_extract_enable", p_pre->feature_enabled, false);
        this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en, false);
        this->get_parameter_or<int>("pcd_save.interval", pcd_save_interval, -1);
        this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR, vector<double>());
        aux_fusion_.load_parameters(*this);
        auxiliary_fusion_enabled = aux_fusion_.dvl_enabled() ||
                                   aux_fusion_.pressure_enabled() ||
                                   aux_fusion_.mag_enabled();
        if (imu_rate_hz <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "common.imu_rate_hz must be positive. Falling back to 100 Hz.");
            imu_rate_hz = 100.0;
        }
        if (odometry_publish_rate_hz <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(),
                        "common.odometry_publish_rate_hz must be positive. Falling back to 100 Hz.");
            odometry_publish_rate_hz = 100.0;
        }
        if (gravity_m_s2 <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "common.gravity_m_s2 must be positive. Falling back to %.2f m/s^2.", G_m_s2);
            gravity_m_s2 = G_m_s2;
        }
        if (noiseless_imu)
        {
            // Simulation-only mode: IMU has zero bias/noise. Freeze ba/bg/grav so
            // LiDAR residuals cannot rewrite them. Only valid for simulated IMUs.
            b_acc_cov = std::min(b_acc_cov, 1e-10);
            b_gyr_cov = std::min(b_gyr_cov, 1e-10);
            init_b_acc_cov = 1e-10;
            init_b_gyr_cov = 1e-10;
            init_grav_cov = 1e-10;
        }
        // else: init_b_acc_cov / init_b_gyr_cov / init_grav_cov already loaded from YAML above.
        const double disabled_aux_cov = 1e-12;
        init_b_dvl_cov = aux_fusion_.dvl_enabled() ? aux_fusion_.dvl_b_init_cov() : disabled_aux_cov;
        init_b_pressure_cov = aux_fusion_.pressure_enabled()
                                  ? aux_fusion_.pressure_b_init_cov()
                                  : disabled_aux_cov;

        path.header.stamp = this->get_clock()->now();
        path.header.frame_id ="camera_init";

        // /*** variables definition ***/
        // int effect_feat_num = 0, frame_num = 0;
        // double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
        // bool flg_EKF_converged, EKF_stop_flg = 0;

        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

        _featsArray.reset(new PointCloudXYZI());

        memset(point_selected_surf, true, sizeof(point_selected_surf));
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        memset(point_selected_surf, true, sizeof(point_selected_surf));

        if (extrinT.size() != 3)
        {
            RCLCPP_WARN(this->get_logger(),
                        "mapping.extrinsic_T must have 3 values. Using zero translation.");
            extrinT = {0.0, 0.0, 0.0};
        }
        if (extrinR.size() != 9)
        {
            RCLCPP_WARN(this->get_logger(),
                        "mapping.extrinsic_R must have 9 values. Using identity rotation.");
            extrinR = {1.0, 0.0, 0.0,
                       0.0, 1.0, 0.0,
                       0.0, 0.0, 1.0};
        }
        Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gravity(gravity_m_s2);
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
        p_imu->set_initial_cov(V3D(init_b_gyr_cov, init_b_gyr_cov, init_b_gyr_cov),
                               V3D(init_b_acc_cov, init_b_acc_cov, init_b_acc_cov),
                               init_grav_cov);
        p_imu->set_initial_aux_cov(V3D(init_b_dvl_cov, init_b_dvl_cov, init_b_dvl_cov),
                                   init_b_pressure_cov);
        obs_manager.load_parameters(*this, b_gyr_cov, b_acc_cov);

        fill(epsi, epsi + state_ikfom::DOF, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        uwfl2::loop_closure::LoopClosureConfig loop_config;
        this->get_parameter_or<bool>("loop_closure.enable", loop_config.enabled, false);
        this->get_parameter_or<bool>("loop_closure.automatic_detection_enable",
                                     loop_config.automatic_detection_enabled, false);
        this->get_parameter_or<bool>("loop_closure.visualization_enable",
                                     loop_visualization_enabled_, false);
        double keyframe_rotation_deg = 10.0;
        int keyframe_minimum_points = 20;
        int loop_queue_capacity = 8;
        int loop_minimum_keyframe_separation = 5;
        double loop_maximum_initial_rotation_error_deg = 45.0;
        double loop_maximum_pose_correction_rotation_deg = 45.0;
        int corrected_map_maximum_keyframes = 1000;
        int corrected_map_maximum_input_points = 3000000;
        int registration_maximum_iterations = 12;
        int registration_minimum_effective_points = 30;
        int std_minimum_keyframe_separation = 20;
        int std_minimum_triangle_matches = 5;
        int std_minimum_ransac_inliers = 5;
        int std_required_confirmations = 2;
        double registration_maximum_rotation_deg = 15.0;
        string diagnostics_directory;
        this->get_parameter_or<double>("loop_closure.keyframe_translation_m",
                                       loop_config.keyframes.translation_m, 1.0);
        this->get_parameter_or<double>("loop_closure.keyframe_rotation_deg",
                                       keyframe_rotation_deg, 10.0);
        this->get_parameter_or<double>("loop_closure.keyframe_minimum_interval_s",
                                       loop_config.keyframes.minimum_interval_s, 0.5);
        this->get_parameter_or<double>("loop_closure.keyframe_maximum_interval_s",
                                       loop_config.keyframes.maximum_interval_s, 5.0);
        this->get_parameter_or<int>("loop_closure.keyframe_minimum_points",
                                    keyframe_minimum_points, 20);
        this->get_parameter_or<int>("loop_closure.queue_capacity", loop_queue_capacity, 8);
        this->get_parameter_or<string>("loop_closure.diagnostics_directory",
                                       diagnostics_directory, "");
        this->get_parameter_or<double>("loop_closure.prior_rotation_sigma_rad",
                                       loop_config.pose_graph.prior_rotation_sigma_rad, 1e-4);
        this->get_parameter_or<double>("loop_closure.prior_translation_sigma_m",
                                       loop_config.pose_graph.prior_translation_sigma_m, 1e-4);
        this->get_parameter_or<double>("loop_closure.odometry_rotation_variance_floor",
                                       loop_config.pose_graph.odometry_rotation_variance_floor, 1e-8);
        this->get_parameter_or<double>("loop_closure.odometry_translation_variance_floor",
                                       loop_config.pose_graph.odometry_translation_variance_floor, 1e-6);
        this->get_parameter_or<int>("loop_closure.loop_minimum_keyframe_separation",
                                    loop_minimum_keyframe_separation, 5);
        this->get_parameter_or<double>("loop_closure.loop_maximum_initial_translation_error_m",
                                       loop_config.pose_graph.loop_maximum_initial_translation_error_m, 10.0);
        this->get_parameter_or<double>("loop_closure.loop_maximum_initial_rotation_error_deg",
                                       loop_maximum_initial_rotation_error_deg, 45.0);
        this->get_parameter_or<double>("loop_closure.loop_maximum_initial_nis",
                                       loop_config.pose_graph.loop_maximum_initial_nis, 12.592);
        this->get_parameter_or<double>("loop_closure.loop_maximum_pose_correction_translation_m",
                                       loop_config.pose_graph.loop_maximum_pose_correction_translation_m, 20.0);
        this->get_parameter_or<double>("loop_closure.loop_maximum_pose_correction_rotation_deg",
                                       loop_maximum_pose_correction_rotation_deg, 45.0);
        this->get_parameter_or<double>("loop_closure.corrected_map_radius_m",
                                       loop_config.shadow_map.radius_m, 80.0);
        this->get_parameter_or<int>("loop_closure.corrected_map_maximum_keyframes",
                                    corrected_map_maximum_keyframes, 1000);
        this->get_parameter_or<int>("loop_closure.corrected_map_maximum_input_points",
                                    corrected_map_maximum_input_points, 3000000);
        this->get_parameter_or<int>("loop_closure.registration_maximum_iterations",
                                    registration_maximum_iterations, 12);
        this->get_parameter_or<int>("loop_closure.registration_minimum_effective_points",
                                    registration_minimum_effective_points, 30);
        this->get_parameter_or<double>("loop_closure.registration_maximum_neighbor_distance_m",
                                       loop_config.registration.maximum_neighbor_distance_m, 2.25);
        this->get_parameter_or<double>("loop_closure.registration_plane_threshold_m",
                                       loop_config.registration.plane_fit_threshold_m, 0.12);
        this->get_parameter_or<double>("loop_closure.registration_maximum_translation_m",
                                       loop_config.registration.maximum_registration_translation_m, 2.0);
        this->get_parameter_or<double>("loop_closure.registration_maximum_rotation_deg",
                                       registration_maximum_rotation_deg, 15.0);
        this->get_parameter_or<int>("loop_closure.std_minimum_keyframe_separation",
                                    std_minimum_keyframe_separation, 20);
        this->get_parameter_or<double>(
            "loop_closure.std_minimum_loop_duration_s",
            loop_config.std_detection.minimum_loop_duration_s, 0.0);
        this->get_parameter_or<double>("loop_closure.std_voxel_size_m",
                                       loop_config.std_detection.voxel_size_m, 0.6);
        this->get_parameter_or<int>("loop_closure.std_minimum_triangle_matches",
                                    std_minimum_triangle_matches, 5);
        this->get_parameter_or<int>("loop_closure.std_minimum_ransac_inliers",
                                    std_minimum_ransac_inliers, 5);
        this->get_parameter_or<double>("loop_closure.std_overlap_minimum",
                                       loop_config.std_detection.geometric_overlap_minimum, 0.20);
        this->get_parameter_or<double>("loop_closure.std_overlap_distance_m",
                                       loop_config.std_detection.geometric_overlap_distance_m, 0.35);
        this->get_parameter_or<int>("loop_closure.std_required_confirmations",
                                    std_required_confirmations, 2);
        this->get_parameter_or<double>(
            "loop_closure.std_single_detection_overlap_minimum",
            loop_config.std_detection.single_detection_overlap_minimum, 0.80);
        this->get_parameter_or<double>(
            "loop_closure.std_accepted_loop_cooldown_s",
            loop_config.std_detection.accepted_loop_cooldown_s, 0.0);
        this->get_parameter_or<double>(
            "loop_closure.std_confirmation_translation_m",
            loop_config.std_detection.confirmation_translation_m, 1.0);
        double std_confirmation_rotation_deg = 10.0;
        this->get_parameter_or<double>(
            "loop_closure.std_confirmation_rotation_deg",
            std_confirmation_rotation_deg, 10.0);
        loop_config.std_detection.confirmation_rotation_rad =
            std::max(0.0, std_confirmation_rotation_deg) * PI_M / 180.0;
        loop_config.keyframes.rotation_rad =
            std::max(0.0, keyframe_rotation_deg) * PI_M / 180.0;
        loop_config.keyframes.minimum_points =
            static_cast<std::size_t>(std::max(1, keyframe_minimum_points));
        loop_config.queue_capacity =
            static_cast<std::size_t>(std::max(1, loop_queue_capacity));
        loop_config.pose_graph.loop_minimum_keyframe_separation =
            static_cast<std::size_t>(std::max(1, loop_minimum_keyframe_separation));
        loop_config.pose_graph.loop_maximum_initial_rotation_error_rad =
            std::max(0.0, loop_maximum_initial_rotation_error_deg) * PI_M / 180.0;
        loop_config.pose_graph.loop_maximum_pose_correction_rotation_rad =
            std::max(0.0, loop_maximum_pose_correction_rotation_deg) * PI_M / 180.0;
        loop_config.std_detection.maximum_pose_distance_m =
            1.5 * std::max(0.0,
                loop_config.pose_graph.loop_maximum_initial_translation_error_m);
        loop_config.std_detection.maximum_prior_translation_error_m =
            loop_config.pose_graph.loop_maximum_initial_translation_error_m;
        loop_config.std_detection.maximum_prior_rotation_rad =
            loop_config.pose_graph.loop_maximum_initial_rotation_error_rad;
        loop_config.shadow_map.voxel_size_m = filter_size_map_min;
        loop_config.shadow_map.maximum_keyframes = static_cast<std::size_t>(
            std::max(1, corrected_map_maximum_keyframes));
        loop_config.shadow_map.maximum_input_points = static_cast<std::size_t>(
            std::max(1, corrected_map_maximum_input_points));
        loop_config.registration.maximum_iterations =
            std::max(1, registration_maximum_iterations);
        loop_config.registration.minimum_effective_points = static_cast<std::size_t>(
            std::max(6, registration_minimum_effective_points));
        loop_config.registration.maximum_registration_rotation_rad =
            std::max(0.0, registration_maximum_rotation_deg) * PI_M / 180.0;
        loop_config.std_detection.minimum_keyframe_separation =
            static_cast<std::size_t>(std::max(1, std_minimum_keyframe_separation));
        loop_config.std_detection.minimum_triangle_matches =
            static_cast<std::size_t>(std::max(1, std_minimum_triangle_matches));
        loop_config.std_detection.minimum_ransac_inliers =
            static_cast<std::size_t>(std::max(1, std_minimum_ransac_inliers));
        loop_config.std_detection.required_consistent_detections =
            static_cast<std::size_t>(std::max(1, std_required_confirmations));
        loop_config.diagnostics_directory = diagnostics_directory;
        benchmark_diagnostics_directory_ = diagnostics_directory;
        if (!benchmark_diagnostics_directory_.empty())
        {
            std::filesystem::create_directories(
                benchmark_diagnostics_directory_);
            front_end_timing_.open(
                benchmark_diagnostics_directory_ / "front_end_timing.csv");
            front_end_timing_
                << "scan_timestamp,status,elapsed_ms,effective_features,"
                   "input_points,scan_accepted,map_points_inserted,map_points,"
                   "sonar_dx_rot_deg,sonar_dx_pos_m,sonar_dx_roll_deg,"
                   "sonar_dx_pitch_deg,sonar_dx_yaw_deg,sonar_dx_x_m,"
                   "sonar_dx_y_m,sonar_dx_z_m,imu_aux_ms,"
                   "fov_downsample_ms,lidar_iekf_ms,correspondence_ms,"
                   "measurement_model_ms,odom_publish_ms,map_incremental_ms,"
                   "loop_bookkeeping_ms\n";
        }
        if (loop_config.enabled)
        {
            loop_closure_ =
                std::make_unique<uwfl2::loop_closure::LoopClosureManager>(loop_config);
            RCLCPP_INFO(this->get_logger(),
                        "Loop closure enabled: asynchronous full-SE(3) keyframes and pose graph active.");
        }

        /*** ROS subscribe initialization ***/
        sensor_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        lidar_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        processing_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        backend_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sensor_options;
        sensor_options.callback_group = sensor_callback_group_;
        rclcpp::SubscriptionOptions lidar_options;
        lidar_options.callback_group = lidar_callback_group_;

        if (lid_topic.empty())
        {
            RCLCPP_WARN(this->get_logger(),
                        "common.lid_topic is empty. LiDAR subscription is disabled; node will run IMU-only odometry.");
        }
        else
        {
            auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(200000));
            // Best-effort subscriptions match both best-effort and reliable
            // sensor publishers and are supported by ROS 2 Humble and newer.
            lidar_qos.best_effort();
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                lid_topic, lidar_qos, standard_pcl_cbk, lidar_options);
        }
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::QoS(rclcpp::KeepLast(200000)), imu_cbk, sensor_options);
        aux_fusion_.create_subscriptions(*this, sensor_callback_group_);
        if (map_pub_en)
        {
            pubCorrectedMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/uwfl2/corrected_map", rclcpp::QoS(1).transient_local().reliable());
        }
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 20);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        if (world_to_camera_init_T.size() != 3)
        {
            RCLCPP_WARN(this->get_logger(),
                        "common.world_to_camera_init_T must have 3 values. Using zero translation.");
            world_to_camera_init_T = {0.0, 0.0, 0.0};
        }
        Eigen::Matrix3d world_to_camera_init_rot = Eigen::Matrix3d::Identity();
        if (world_to_camera_init_R.size() == 9)
        {
            world_to_camera_init_rot << world_to_camera_init_R[0], world_to_camera_init_R[1], world_to_camera_init_R[2],
                                        world_to_camera_init_R[3], world_to_camera_init_R[4], world_to_camera_init_R[5],
                                        world_to_camera_init_R[6], world_to_camera_init_R[7], world_to_camera_init_R[8];
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                        "common.world_to_camera_init_R must have 9 values. Using identity rotation.");
        }
        Eigen::Quaterniond world_to_camera_init_quat(world_to_camera_init_rot);
        world_to_camera_init_quat.normalize();

        geometry_msgs::msg::TransformStamped world_to_camera_init;
        world_to_camera_init.header.stamp = this->get_clock()->now();
        world_to_camera_init.header.frame_id = world_frame;
        world_to_camera_init.child_frame_id = "camera_init";
        world_to_camera_init.transform.translation.x = world_to_camera_init_T[0];
        world_to_camera_init.transform.translation.y = world_to_camera_init_T[1];
        world_to_camera_init.transform.translation.z = world_to_camera_init_T[2];
        world_to_camera_init.transform.rotation.x = world_to_camera_init_quat.x();
        world_to_camera_init.transform.rotation.y = world_to_camera_init_quat.y();
        world_to_camera_init.transform.rotation.z = world_to_camera_init_quat.z();
        world_to_camera_init.transform.rotation.w = world_to_camera_init_quat.w();
        static_tf_broadcaster_->sendTransform(world_to_camera_init);

        // Inform the pressure model how camera_init sits in World, so pressure
        // constrains true World-vertical depth rather than tilted local z.
        aux_fusion_.set_camera_init_pose_in_world(
            V3D(world_to_camera_init_T[0], world_to_camera_init_T[1], world_to_camera_init_T[2]),
            world_to_camera_init_rot);

        //------------------------------------------------------------------------------------------------------
        // Drive processing from wall time so rosbag replay can drain buffered
        // messages even after simulated /clock stops publishing. Sensor callbacks
        // are isolated in their own callback group and only fill mutex-protected
        // buffers; the EKF state remains owned by this processing callback.
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1),
                                         std::bind(&LaserMappingNode::timer_callback, this),
                                         processing_callback_group_);
        if (pubCorrectedMap_)
        {
            const auto corrected_map_period = std::chrono::duration_cast<
                std::chrono::milliseconds>(std::chrono::duration<double>(
                corrected_map_interval_s_));
            corrected_map_timer_ = this->create_wall_timer(
                corrected_map_period,
                std::bind(&LaserMappingNode::maybe_publish_corrected_map, this),
                backend_callback_group_);
        }

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
            loop_inject_srv_ = this->create_service<fast_lio::srv::InjectLoop>(
                "/uwfl2_lc/inject_loop",
                std::bind(&LaserMappingNode::inject_loop_callback, this,
                          std::placeholders::_1, std::placeholders::_2),
                rmw_qos_profile_services_default, backend_callback_group_);
        }

        RCLCPP_INFO(this->get_logger(), "Node init finished.");
    }

    ~LaserMappingNode() override
    {
        write_front_end_summary();
    }

private:
    struct FrontEndStageTiming
    {
        double imu_aux_ms = 0.0;
        double fov_downsample_ms = 0.0;
        double lidar_iekf_ms = 0.0;
        double correspondence_ms = 0.0;
        double measurement_model_ms = 0.0;
        double odom_publish_ms = 0.0;
        double map_incremental_ms = 0.0;
        double loop_bookkeeping_ms = 0.0;
    };

    void timer_callback()
    {
        try_commit_loop_correction();
        maybe_publish_loop_visualization();
        bool imu_only_measure = false;
        bool has_measurement = false;
        {
            std::lock_guard<std::mutex> lock(mtx_buffer);
            has_measurement = sync_packages(Measures);
            if (!has_measurement)
            {
                imu_only_measure = sync_imu_only_packages(Measures);
                has_measurement = imu_only_measure;
            }
        }

        if(has_measurement)
        {
            const double scan_processing_started = omp_get_wtime();
            V3D sonar_attitude_correction = V3D::Zero();
            V3D sonar_position_correction = V3D::Zero();
            uwfl2::LidarUpdateResult lidar_update_result;
            effct_feat_num = 0;
            feats_down_size = 0;
            add_point_size = 0;
            FrontEndStageTiming stage_timing;
            const auto finish_scan_timing = [&](const char *status) {
                if (!imu_only_measure)
                {
                    write_front_end_timing(
                        Measures.lidar_end_time, status,
                        1000.0 * (omp_get_wtime() - scan_processing_started),
                        sonar_attitude_correction, sonar_position_correction,
                        lidar_update_result, stage_timing);
                }
            };
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                if (!imu_only_measure)
                {
                    finish_scan_timing("first_scan");
                    return;
                }
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            const double process_begin_time = last_processed_time > 0.0 ? last_processed_time : Measures.lidar_beg_time;
            if (!noiseless_imu && auxiliary_fusion_enabled)
            {
                const double now = Measures.lidar_end_time;
                const double dyn_bg = obs_manager.bg_cov(now);
                const double dyn_ba = obs_manager.ba_cov(now);
                p_imu->set_gyr_bias_cov(V3D(dyn_bg, dyn_bg, dyn_bg));
                p_imu->set_acc_bias_cov(V3D(dyn_ba, dyn_ba, dyn_ba));
            }
            if (!p_imu->IsInitialized())
            {
                p_imu->Process(Measures, kf, feats_undistort);
                if (p_imu->IsInitialized() && auxiliary_fusion_enabled)
                {
                    aux_fusion_.initialize_pressure_reference_pose(kf.get_x());
                }
                last_processed_time = Measures.lidar_end_time;
                update_state_outputs();
                stage_timing.imu_aux_ms =
                    1000.0 * (omp_get_wtime() - t0);
                finish_scan_timing("imu_initialization");
                return;
            }

            AuxiliarySensorFusion::UpdateSummary aux_summary;
            if (!auxiliary_fusion_enabled)
            {
                // This is the original FAST-LIO2 path: one scan-bounded IMU
                // propagation/deskew and no extra attitude or auxiliary update.
                p_imu->Process(Measures, kf, feats_undistort);
            }
            else
            {
                aux_fusion_.initialize_pressure_reference_pose(kf.get_x());
                const auto timed_measurements = aux_fusion_.take_timed_measurements(
                    process_begin_time, Measures.lidar_end_time);
                const auto late_measurements = aux_fusion_.take_late_measurement_counts();
                aux_late_dvl_.fetch_add(late_measurements.dvl);
                aux_late_pressure_.fetch_add(late_measurements.pressure);
                aux_late_magnetometer_.fetch_add(late_measurements.magnetometer);
                if (aux_timeline_started_ && late_measurements.total() > 0)
                {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 5000,
                        "Dropped out-of-sequence auxiliary measurements: DVL=%zu pressure=%zu magnetometer=%zu. "
                        "Check sensor acquisition timestamps and transport latency.",
                        late_measurements.dvl, late_measurements.pressure,
                        late_measurements.magnetometer);
                }
                aux_timeline_started_ = true;
                std::vector<double> timed_measurement_stamps;
                timed_measurement_stamps.reserve(timed_measurements.size());
                for (const auto &measurement : timed_measurements)
                {
                    timed_measurement_stamps.push_back(measurement.timestamp);
                }

                auto apply_timed_measurement =
                    [&](std::size_t measurement_index, ImuProcess::Ekf &event_kf) -> bool
                {
                    return aux_fusion_.apply_timed_measurement(
                        timed_measurements[measurement_index], Measures.imu,
                        event_kf, aux_summary);
                };

                p_imu->Process(Measures, kf, feats_undistort,
                               timed_measurement_stamps, apply_timed_measurement);
                aux_fusion_.warn_timeouts(*this, Measures.lidar_end_time);
            }
            // These are IMU observations, not auxiliary-sensor observations.
            // Apply them once after either propagation path so disabling DVL,
            // pressure, and magnetometer cannot silently change IMU behavior.
            if (!Measures.imu.empty())
            {
                apply_imu_orientation_update(Measures.imu.back());
                apply_accel_attitude_update(Measures.imu);
            }
            last_processed_time = Measures.lidar_end_time;
            update_state_outputs();
            stage_timing.imu_aux_ms = 1000.0 * (omp_get_wtime() - t0);

            if (imu_only_measure)
            {
                ++imu_only_packets_processed;
                if (lid_topic.empty())
                {
                    // Intentional no-lidar mode. Startup already reported this once.
                }
                else if (last_timestamp_lidar <= 0.0)
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                         "No LiDAR messages received on '%s'. Running IMU-only odometry.",
                                         lid_topic.c_str());
                }
                else
                {
                    const double lidar_gap = Measures.lidar_end_time - last_timestamp_lidar;
                    if (lidar_gap >= lidar_timeout)
                    {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                             "No LiDAR scan for %.2f s (timeout %.2f s). Running IMU-only odometry.",
                                             lidar_gap, lidar_timeout);
                    }
                }
                g_publish_mode = aux_summary.updated() ? "aux_only" : "imu_only";
                publish_odometry(pubOdomAftMapped_, tf_broadcaster_);
                return;
            }

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                RCLCPP_WARN(this->get_logger(), "No point, publish IMU-only odometry for this scan.\n");
                lidar_update_result = lidar_scan_quality.reject_sparse_input(0);
                g_publish_mode = aux_summary.updated() ? "aux_only" : "imu_only";
                publish_odometry(pubOdomAftMapped_, tf_broadcaster_);
                finish_scan_timing("no_points");
                return;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            const double fov_downsample_started = omp_get_wtime();
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            stage_timing.fov_downsample_ms =
                1000.0 * (t1 - fov_downsample_started);
            feats_down_size = feats_down_body->points.size();
            if (!lidar_scan_quality.input_is_sufficient(
                    static_cast<std::size_t>(feats_down_size)))
            {
                lidar_update_result = lidar_scan_quality.reject_sparse_input(
                    static_cast<std::size_t>(feats_down_size));
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Rejected sparse sonar scan: %d downsampled points (minimum %d).",
                    feats_down_size, minimum_scan_points);
                g_publish_mode = aux_summary.updated() ? "aux_only" : "imu_only";
                publish_odometry(pubOdomAftMapped_, tf_broadcaster_);
                finish_scan_timing(uwfl2::lidar_scan_status(
                    lidar_update_result.reason));
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
                    ikdtree->Build(feats_down_world->points);
                    lidar_update_result =
                        lidar_scan_quality.accept_map_initialization(
                            static_cast<std::size_t>(feats_down_size));
                    lidar_update_result.map_points_inserted =
                        static_cast<std::size_t>(ikdtree->validnum());
                    obs_manager.notify_sonar_scan(Measures.lidar_end_time);
                    if (loop_closure_)
                    {
                        ++active_tree_generation_;
                        loop_closure_->notify_active_tree_generation(
                            active_tree_generation_);
                    }
                    const double loop_bookkeeping_started = omp_get_wtime();
                    accumulate_mapping_output();
                    submit_loop_keyframe(Measures.lidar_end_time);
                    stage_timing.loop_bookkeeping_ms =
                        1000.0 * (omp_get_wtime() - loop_bookkeeping_started);
                }
                else
                {
                    lidar_update_result = lidar_scan_quality.reject_sparse_input(
                        static_cast<std::size_t>(feats_down_size));
                }
                g_publish_mode = lidar_update_result.accepted
                                     ? "kdtree_init"
                                     : (aux_summary.updated() ? "aux_only"
                                                              : "imu_only");
                publish_odometry(pubOdomAftMapped_, tf_broadcaster_);
                finish_scan_timing(lidar_update_result.accepted
                                       ? "tree_initialization"
                                       : uwfl2::lidar_scan_status(
                                             lidar_update_result.reason));
                return;
            }
            int featsFromMapNum = ikdtree->validnum();
            kdtree_size_st = ikdtree->size();

            /*** ICP and iterated Kalman filter update ***/
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            const V3D position_before_sonar = kf.get_x().pos;
            const M3D rotation_before_sonar = kf.get_x().rot.toRotationMatrix();
            const double lidar_update_cov =
                auxiliary_fusion_enabled ? 1.0 : LASER_POINT_COV_XY;
            lidar_update_max_effective_features = 0;
            uwfl2::LidarUpdateTransaction<MainEkf> lidar_transaction(kf);
            const bool update_applied =
                kf.update_iterated_dyn_share_modified(lidar_update_cov,
                                                      solve_H_time);
            lidar_update_result = lidar_scan_quality.evaluate_update(
                static_cast<std::size_t>(feats_down_size),
                static_cast<std::size_t>(lidar_update_max_effective_features),
                update_applied);
            lidar_transaction.finish(lidar_update_result);
            if (!lidar_update_result.accepted)
            {
                update_state_outputs();
                stage_timing.lidar_iekf_ms =
                    1000.0 * (omp_get_wtime() - t_update_start);
                stage_timing.correspondence_ms = 1000.0 * match_time;
                stage_timing.measurement_model_ms = 1000.0 * solve_time;
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Rejected sonar update: %d effective features (minimum %d).",
                    lidar_update_max_effective_features,
                    minimum_effective_features);
                g_publish_mode = aux_summary.updated() ? "aux_only" : "imu_only";
                publish_odometry(pubOdomAftMapped_, tf_broadcaster_);
                finish_scan_timing(uwfl2::lidar_scan_status(
                    lidar_update_result.reason));
                return;
            }
            obs_manager.notify_sonar_scan(Measures.lidar_end_time);
            const M3D sonar_rotation_delta =
                rotation_before_sonar.transpose() *
                kf.get_x().rot.toRotationMatrix();
            sonar_attitude_correction = Log(sonar_rotation_delta);
            sonar_position_correction = kf.get_x().pos - position_before_sonar;
            update_state_outputs();

            double t_update_end = omp_get_wtime();
            stage_timing.lidar_iekf_ms =
                1000.0 * (t_update_end - t_update_start);
            stage_timing.correspondence_ms = 1000.0 * match_time;
            stage_timing.measurement_model_ms = 1000.0 * solve_time;

            /******* Publish odometry *******/
            const double odom_publish_started = omp_get_wtime();
            g_publish_mode = "lidar_update";
            publish_odometry(pubOdomAftMapped_, tf_broadcaster_);
            stage_timing.odom_publish_ms =
                1000.0 * (omp_get_wtime() - odom_publish_started);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            lidar_update_result.map_points_inserted = map_incremental();
            const double map_incremental_finished = omp_get_wtime();
            stage_timing.map_incremental_ms =
                1000.0 * (map_incremental_finished - t3);
            if (loop_closure_)
            {
                ++active_tree_generation_;
                loop_closure_->notify_active_tree_generation(
                    active_tree_generation_);
            }
            const double loop_bookkeeping_started = omp_get_wtime();
            accumulate_mapping_output();
            submit_loop_keyframe(Measures.lidar_end_time);
            t5 = omp_get_wtime();
            stage_timing.loop_bookkeeping_ms =
                1000.0 * (t5 - loop_bookkeeping_started);
            
            finish_scan_timing("lidar_update");
        }
    }

    void write_front_end_timing(double timestamp, const char *status,
                                double elapsed_ms,
                                const V3D &sonar_attitude_correction,
                                const V3D &sonar_position_correction,
                                const uwfl2::LidarUpdateResult &lidar_result,
                                const FrontEndStageTiming &stage_timing)
    {
        if (!front_end_timing_)
        {
            return;
        }
        std::lock_guard<std::mutex> lock(front_end_diagnostics_mutex_);
        front_end_timing_ << std::setprecision(17) << timestamp << ',' << status
                          << ',' << elapsed_ms << ','
                          << lidar_result.effective_features << ','
                          << lidar_result.input_points << ','
                          << (lidar_result.accepted ? 1 : 0) << ','
                          << lidar_result.map_points_inserted << ','
                          << (ikdtree ? ikdtree->validnum() : 0) << ','
                          << sonar_attitude_correction.norm() * 180.0 / M_PI << ','
                          << sonar_position_correction.norm() << ','
                          << sonar_attitude_correction.x() * 180.0 / M_PI << ','
                          << sonar_attitude_correction.y() * 180.0 / M_PI << ','
                          << sonar_attitude_correction.z() * 180.0 / M_PI << ','
                          << sonar_position_correction.x() << ','
                          << sonar_position_correction.y() << ','
                          << sonar_position_correction.z() << ','
                          << stage_timing.imu_aux_ms << ','
                          << stage_timing.fov_downsample_ms << ','
                          << stage_timing.lidar_iekf_ms << ','
                          << stage_timing.correspondence_ms << ','
                          << stage_timing.measurement_model_ms << ','
                          << stage_timing.odom_publish_ms << ','
                          << stage_timing.map_incremental_ms << ','
                          << stage_timing.loop_bookkeeping_ms << '\n';
        if (++front_end_rows_since_flush_ >= 64)
        {
            front_end_timing_.flush();
            front_end_rows_since_flush_ = 0;
        }
        ++timed_scans_;
    }

    void write_front_end_summary()
    {
        if (benchmark_diagnostics_directory_.empty())
        {
            return;
        }
        std::size_t pending_lidar = 0;
        std::size_t pending_imu = 0;
        {
            std::lock_guard<std::mutex> buffer_lock(mtx_buffer);
            pending_lidar = lidar_buffer.size();
            pending_imu = imu_buffer.size();
        }
        std::lock_guard<std::mutex> lock(front_end_diagnostics_mutex_);
        if (front_end_timing_)
        {
            front_end_timing_.flush();
            front_end_rows_since_flush_ = 0;
        }
        const auto output =
            benchmark_diagnostics_directory_ / "front_end_summary.json";
        const auto temporary =
            benchmark_diagnostics_directory_ / "front_end_summary.json.tmp";
        std::ofstream summary(temporary);
        summary << "{\n"
                << "  \"lidar_callbacks_received\": "
                << lidar_callbacks_received.load() << ",\n"
                << "  \"imu_callbacks_received\": "
                << imu_callbacks_received.load() << ",\n"
                << "  \"lidar_timestamp_regressions\": "
                << lidar_timestamp_regressions.load() << ",\n"
                << "  \"imu_timestamp_regressions\": "
                << imu_timestamp_regressions.load() << ",\n"
                << "  \"lidar_buffer_messages_cleared\": "
                << lidar_buffer_messages_cleared.load() << ",\n"
                << "  \"imu_buffer_messages_cleared\": "
                << imu_buffer_messages_cleared.load() << ",\n"
                << "  \"stale_lidar_scans_discarded\": "
                << stale_lidar_scans_discarded.load() << ",\n"
                << "  \"imu_only_packets_processed\": "
                << imu_only_packets_processed.load() << ",\n"
                << "  \"aux_late_dvl\": " << aux_late_dvl_.load() << ",\n"
                << "  \"aux_late_pressure\": "
                << aux_late_pressure_.load() << ",\n"
                << "  \"aux_late_magnetometer\": "
                << aux_late_magnetometer_.load() << ",\n"
                << "  \"timed_scans\": " << timed_scans_.load() << ",\n"
                << "  \"pending_lidar_messages\": " << pending_lidar << ",\n"
                << "  \"pending_imu_messages\": " << pending_imu << "\n"
                << "}\n";
        summary.close();
        std::error_code error;
        std::filesystem::rename(temporary, output, error);
        if (error)
        {
            std::filesystem::remove(output, error);
            error.clear();
            std::filesystem::rename(temporary, output, error);
        }
    }

    void map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        RCLCPP_INFO(this->get_logger(), "Saving map to %s...", map_file_path.c_str());
        if (pcd_save_en)
        {
            const PointCloudXYZI snapshot = corrected_mapping_snapshot();
            if (snapshot.empty())
            {
                res->success = false;
                res->message = "Map is empty; no PCD was written.";
            }
            else
            {
                try
                {
                    save_to_pcd(snapshot);
                    res->success = true;
                    res->message = "Corrected map saved with " +
                                   std::to_string(snapshot.size()) + " points.";
                }
                catch (const std::exception &error)
                {
                    res->success = false;
                    res->message = std::string("Map save failed: ") + error.what();
                }
            }
        }
        else
        {
            res->success = false;
            res->message = "Map save disabled.";
        }
        write_front_end_summary();
    }

    void accumulate_mapping_output()
    {
        // Loop closure always needs keyframe-owned map history. With loop
        // closure disabled, retain FAST-LIO2's original behavior and only
        // accumulate history when PCD saving is enabled.
        if ((!loop_closure_ && !pcd_save_en && !map_pub_en) ||
            !feats_down_body ||
            feats_down_body->empty())
        {
            return;
        }

        PointCloudXYZI compact_world;
        compact_world.points.resize(feats_down_body->size());
        for (std::size_t index = 0; index < feats_down_body->size(); ++index)
        {
            RGBpointBodyToWorld(&feats_down_body->points[index],
                                &compact_world.points[index]);
        }
        compact_world.width = static_cast<std::uint32_t>(compact_world.size());
        compact_world.height = 1;
        compact_world.is_dense = false;
        if (loop_closure_)
        {
            std::lock_guard<std::mutex> lock(mapping_output_mutex);
            pending_map_points_world_.reserve(
                pending_map_points_world_.size() + compact_world.size());
            for (const auto &point : compact_world.points)
            {
                pending_map_points_world_.push_back(
                    {point.x, point.y, point.z, point.intensity});
            }
        }
        else
        {
            std::lock_guard<std::mutex> lock(mapping_output_mutex);
            *pcl_wait_pub += compact_world;
        }
        if (map_pub_en)
        {
            corrected_map_publish_requested_.store(true);
        }
    }

    std::vector<uwfl2::loop_closure::PointXYZI>
    compact_map_points(
        const std::vector<uwfl2::loop_closure::PointXYZI> &pending) const
    {
        if (pending.empty())
        {
            return {};
        }
        PointCloudXYZI::Ptr input(new PointCloudXYZI());
        input->points.reserve(pending.size());
        for (const auto &point : pending)
        {
            PointType converted;
            converted.x = point.x;
            converted.y = point.y;
            converted.z = point.z;
            converted.intensity = point.intensity;
            converted.normal_x = converted.normal_y = converted.normal_z = 0.0F;
            converted.curvature = 0.0F;
            input->points.push_back(converted);
        }
        input->width = static_cast<std::uint32_t>(input->size());
        input->height = 1;
        input->is_dense = false;

        PointCloudXYZI filtered;
        pcl::VoxelGrid<PointType> filter;
        // LTA-OM retains voxelized keyframe submaps rather than every registered
        // scan point. Use the live map resolution so the corrected history and
        // replacement ikd-tree have the same spatial detail and memory scale.
        const float leaf = static_cast<float>(
            std::max({1e-3, filter_size_surf_min, filter_size_map_min}));
        filter.setLeafSize(leaf, leaf, leaf);
        filter.setInputCloud(input);
        filter.filter(filtered);

        std::vector<uwfl2::loop_closure::PointXYZI> output;
        output.reserve(filtered.size());
        for (const auto &point : filtered.points)
        {
            output.push_back({point.x, point.y, point.z, point.intensity});
        }
        return output;
    }

    std::vector<uwfl2::loop_closure::PointXYZI>
    compact_pending_map_points() const
    {
        std::vector<uwfl2::loop_closure::PointXYZI> pending;
        {
            std::lock_guard<std::mutex> lock(mapping_output_mutex);
            pending = pending_map_points_world_;
        }
        return compact_map_points(pending);
    }

    PointCloudXYZI corrected_mapping_snapshot() const
    {
        PointCloudXYZI snapshot;
        std::vector<uwfl2::loop_closure::PointXYZI> pending;
        {
            std::lock_guard<std::mutex> lock(mapping_output_mutex);
            snapshot = *pcl_wait_pub;
            pending = pending_map_points_world_;
        }
        for (const auto &point : compact_map_points(pending))
        {
            PointType converted;
            converted.x = point.x;
            converted.y = point.y;
            converted.z = point.z;
            converted.intensity = point.intensity;
            converted.normal_x = converted.normal_y = converted.normal_z = 0.0F;
            converted.curvature = 0.0F;
            snapshot.points.push_back(converted);
        }
        snapshot.width = static_cast<std::uint32_t>(snapshot.size());
        snapshot.height = 1;
        snapshot.is_dense = false;
        if (snapshot.empty())
        {
            return snapshot;
        }

        PointCloudXYZI::Ptr input(new PointCloudXYZI(snapshot));
        PointCloudXYZI filtered;
        pcl::VoxelGrid<PointType> filter;
        const float leaf =
            static_cast<float>(std::max(1e-3, filter_size_map_min));
        filter.setLeafSize(leaf, leaf, leaf);
        filter.setInputCloud(input);
        filter.filter(filtered);
        filtered.width = static_cast<std::uint32_t>(filtered.size());
        filtered.height = 1;
        filtered.is_dense = false;
        return filtered;
    }

    void commit_mapping_submap(
        const std::vector<uwfl2::loop_closure::PointXYZI> &points)
    {
        if (points.empty())
        {
            return;
        }
        PointCloudXYZI submap;
        submap.points.reserve(points.size());
        for (const auto &point : points)
        {
            PointType converted;
            converted.x = point.x;
            converted.y = point.y;
            converted.z = point.z;
            converted.intensity = point.intensity;
            converted.normal_x = converted.normal_y = converted.normal_z = 0.0F;
            converted.curvature = 0.0F;
            submap.points.push_back(converted);
        }
        submap.width = static_cast<std::uint32_t>(submap.size());
        submap.height = 1;
        submap.is_dense = false;
        std::lock_guard<std::mutex> lock(mapping_output_mutex);
        *pcl_wait_pub += submap;
        pending_map_points_world_.clear();
    }

private:
    struct CorrectedMappingOutput
    {
        PointCloudXYZI::Ptr cloud;
        std::vector<uwfl2::loop_closure::PointXYZI> pending_points;
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

        const auto compact_pending = compact_pending_map_points();
        output.pending_points.reserve(compact_pending.size());
        for (const auto &point : compact_pending)
        {
            const Eigen::Vector3d corrected =
                latest_correction.rotation *
                    Eigen::Vector3d(point.x, point.y, point.z) +
                latest_correction.translation;
            if (!corrected.allFinite())
            {
                continue;
            }
            output.pending_points.push_back(
                {static_cast<float>(corrected.x()),
                 static_cast<float>(corrected.y()),
                 static_cast<float>(corrected.z()), point.intensity});
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
        const auto reject = [&](const char *reason) {
            loop_closure_->notify_correction_result(false, elapsed_ms(), reason);
        };
        if (!candidate->shadow_map || !candidate->registration ||
            !candidate->scan || !candidate->shadow_map->valid ||
            !candidate->registration->valid || !candidate->shadow_map->tree ||
            candidate->registration->scan_generation !=
                candidate->scan->scan_generation)
        {
            reject("stale_or_invalid_candidate");
            return;
        }

        const auto graph = loop_closure_->graph_snapshot();
        if (candidate->registration->scan_generation != latest_scan_generation_ ||
            candidate->scan->active_tree_generation != active_tree_generation_ ||
            candidate->registration->graph_version != graph.version)
        {
            loop_closure_->request_registration_refresh(
                candidate->shadow_map, candidate->refresh_attempt + 1);
            reject("stale_candidate_refresh_requested");
            return;
        }

        const auto &registration = *candidate->registration;
        const auto correction = uwfl2::loop_closure::compose(
            registration.T_local_vehicle_registered,
            uwfl2::loop_closure::inverse(registration.T_local_vehicle_raw));
        if (!correction.finite())
        {
            reject("non_finite_correction");
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
            reject("protected_state_changed");
            return;
        }

        uwfl2::loop_closure::Matrix6d correction_covariance =
            registration.covariance +
            registration.graph_anchor_covariance_position_rotation;
        correction_covariance =
            0.5 * (correction_covariance + correction_covariance.transpose());
        const auto corrected_covariance =
            uwfl2::loop_closure::transport_uwfl2_covariance(
                kf.get_P(), correction_rotation, correction_covariance);
        if (!corrected_state.pos.allFinite() ||
            !corrected_state.vel.allFinite() ||
            !corrected_state.rot.toRotationMatrix().allFinite() ||
            !uwfl2::loop_closure::covariance27_is_valid(corrected_covariance))
        {
            reject("invalid_state_or_covariance");
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
            reject("empty_registered_scan");
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
        try
        {
            auto replacement_tree = candidate->shadow_map->tree;
            replacement_tree->Add_Points(registered_scan, true);
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
            loop_closure_->notify_correction_result(
                true, elapsed_ms(), "committed");
            if (corrected_mapping_output.cloud)
            {
                std::lock_guard<std::mutex> lock(mapping_output_mutex);
                pcl_wait_pub = corrected_mapping_output.cloud;
                pending_map_points_world_ = corrected_mapping_output.pending_points;
                corrected_map_publish_requested_ = true;
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
            reject("transaction_exception_rollback");
        }
    }

    void maybe_publish_corrected_map()
    {
        if (!map_pub_en || !pubCorrectedMap_)
        {
            return;
        }
        if (pubCorrectedMap_->get_subscription_count() == 0 &&
            pubCorrectedMap_->get_intra_process_subscription_count() == 0)
        {
            return;
        }

        const bool requested = corrected_map_publish_requested_.exchange(false);
        if (!requested)
        {
            return;
        }

        const PointCloudXYZI snapshot = corrected_mapping_snapshot();
        if (snapshot.empty())
        {
            corrected_map_publish_requested_.store(requested);
            return;
        }

        sensor_msgs::msg::PointCloud2 message;
        pcl::toROSMsg(snapshot, message);
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "camera_init";
        pubCorrectedMap_->publish(message);

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

    void inject_loop_callback(
        const std::shared_ptr<fast_lio::srv::InjectLoop::Request> request,
        std::shared_ptr<fast_lio::srv::InjectLoop::Response> response)
    {
        uwfl2::loop_closure::LoopConstraint constraint;
        constraint.from_id = request->from_id;
        constraint.to_id = request->to_id;
        constraint.T_from_to.translation = {
            request->relative_pose.position.x,
            request->relative_pose.position.y,
            request->relative_pose.position.z};
        constraint.T_from_to.rotation = Eigen::Quaterniond(
            request->relative_pose.orientation.w,
            request->relative_pose.orientation.x,
            request->relative_pose.orientation.y,
            request->relative_pose.orientation.z);
        for (int row = 0; row < 6; ++row)
        {
            for (int column = 0; column < 6; ++column)
            {
                constraint.covariance(row, column) =
                    request->covariance[static_cast<std::size_t>(row * 6 + column)];
            }
        }
        constraint.test_override = request->test_override;
        const auto evaluation = loop_closure_->inject_loop(constraint);
        response->accepted = evaluation.accepted;
        response->reason = evaluation.reason;
        response->graph_version = evaluation.graph_version;
        response->graph_error_before = evaluation.graph_error_before;
        response->graph_error_after = evaluation.graph_error_after;
        response->loop_translation_error_before =
            evaluation.loop_translation_error_before;
        response->loop_translation_error_after =
            evaluation.loop_translation_error_after;
        response->loop_rotation_error_before_deg =
            evaluation.loop_rotation_error_before_rad * 180.0 / PI_M;
        response->loop_rotation_error_after_deg =
            evaluation.loop_rotation_error_after_rad * 180.0 / PI_M;
        publish_graph_paths();
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
            uwfl2::loop_closure::extract_graph_pose_covariance(kf.get_P());
        ++latest_scan_generation_;
        const auto scan_points = loop_closure_->notify_latest_scan(
            timestamp, latest_scan_generation_, active_tree_generation_,
            T_local_vehicle, T_vehicle_sonar, feats_down_body->points);
        if (!scan_points ||
            !loop_closure_->should_select_keyframe(
                timestamp, T_local_vehicle, scan_points->size()))
        {
            return;
        }
        const auto compact_map_points = compact_pending_map_points();
        const bool submitted = loop_closure_->try_submit_snapshot(
            timestamp, T_local_vehicle, pose_covariance, T_vehicle_sonar,
            scan_points, compact_map_points,
            active_tree_generation_);
        if (submitted)
        {
            commit_mapping_submap(compact_map_points);
            corrected_map_publish_requested_ = true;
        }
    }

    AuxiliarySensorFusion aux_fusion_;
    std::unique_ptr<uwfl2::loop_closure::LoopClosureManager> loop_closure_;
    std::filesystem::path benchmark_diagnostics_directory_;
    std::ofstream front_end_timing_;
    std::mutex front_end_diagnostics_mutex_;
    std::size_t front_end_rows_since_flush_ = 0;
    std::atomic<std::uint64_t> aux_late_dvl_{0};
    std::atomic<std::uint64_t> aux_late_pressure_{0};
    std::atomic<std::uint64_t> aux_late_magnetometer_{0};
    std::atomic<std::uint64_t> timed_scans_{0};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCorrectedMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_graph_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimized_graph_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        loop_markers_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;
    rclcpp::CallbackGroup::SharedPtr lidar_callback_group_;
    rclcpp::CallbackGroup::SharedPtr processing_callback_group_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr corrected_map_timer_;
    rclcpp::CallbackGroup::SharedPtr backend_callback_group_;
    rclcpp::Service<fast_lio::srv::InjectLoop>::SharedPtr loop_inject_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;

    bool effect_pub_en = false, map_pub_en = false;
    std::atomic<bool> corrected_map_publish_requested_{true};
    double corrected_map_interval_s_ = 0.2;
    bool loop_visualization_enabled_ = false;
    std::chrono::steady_clock::time_point next_loop_visualization_publish_{};
    bool aux_timeline_started_ = false;
    std::uint64_t active_tree_generation_ = 0;
    std::uint64_t latest_scan_generation_ = 0;
    std::vector<uwfl2::loop_closure::PointXYZI> pending_map_points_world_;
    int effect_feat_num = 0;
    double deltaT, deltaR;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[state_ikfom::DOF] = {0.001};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    signal(SIGINT, SigHandle);

    auto node = std::make_shared<LaserMappingNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.spin();

    if (rclcpp::ok())
        rclcpp::shutdown();
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    return 0;
}

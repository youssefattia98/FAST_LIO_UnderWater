#pragma once

#include <algorithm>
#include <cstddef>
#include <memory>
#include <vector>
#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <csignal>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "use-ikfom.hpp"

/// *************Preconfiguration

#define MAX_INI_COUNT (10)

inline const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ekf = esekfom::esekf<state_ikfom, process_noise_ikfom::DOF, input_ikfom>;
  using TimedUpdateCallback = std::function<bool(std::size_t, Ekf &)>;

  ImuProcess();
  ~ImuProcess();
  
  void Reset();
  // void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void Reset(double start_timestamp, const sensor_msgs::msg::Imu::ConstSharedPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4,4) &T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  void set_initial_cov(const V3D &b_g, const V3D &b_a, double grav);
  // Auxiliary-bias initial covariances (added with DVL/pressure fusion). Locking
  // these in noiseless-IMU sim mode is what stops LiDAR scan-match residuals from
  // bleeding into b_dvl/b_pressure via the P-inverse cross-correlations and
  // breaking DVL/pressure observability mid-bag.
  void set_initial_aux_cov(const V3D &b_dvl, double b_pressure);
  void set_gravity(const double gravity_m_s2);
  bool IsInitialized() const;
  double AccelerationScale() const;
  Eigen::Matrix<double, process_noise_ikfom::DOF, process_noise_ikfom::DOF> Q;
  void Process(const MeasureGroup &meas,
               Ekf &kf_state,
               PointCloudXYZI::Ptr pcl_un_,
               const std::vector<double> &update_times = {},
               const TimedUpdateCallback &timed_update = {});

  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  V3D init_cov_bias_gyr;
  V3D init_cov_bias_acc;
  double init_cov_grav;
  V3D init_cov_b_dvl;
  double init_cov_b_pressure;
  double first_lidar_time;

 private:
  void IMU_init(const MeasureGroup &meas,
                Ekf &kf_state,
                int &N);
  void UndistortPcl(const MeasureGroup &meas,
                    Ekf &kf_state,
                    PointCloudXYZI &pcl_in_out,
                    const std::vector<double> &update_times,
                    const TimedUpdateCallback &timed_update);
  void UndistortPclFastLio(const MeasureGroup &meas,
                           Ekf &kf_state,
                           PointCloudXYZI &pcl_in_out);
  bool ReconstructContinuousDeskewPoses(
      const std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> &imu_msgs,
      double scan_begin_time,
      double scan_end_time,
      const state_ikfom &scan_end_state);

  PointCloudXYZI::Ptr cur_pcl_un_;
  // sensor_msgs::ImuConstPtr last_imu_;
  sensor_msgs::msg::Imu::ConstSharedPtr last_imu_;
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> v_imu_;
  vector<Pose6D> IMUpose;
  vector<M3D>    v_rot_pcl_;
  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double gravity_m_s2_;
  double start_timestamp_;
  double last_lidar_end_time_;
  int    init_iter_num = 1;
  bool   b_first_frame_ = true;
  bool   imu_need_init_ = true;
};

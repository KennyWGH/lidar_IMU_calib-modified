/*
 * LI_Calib: An Open Platform for LiDAR-IMU Calibration
 * Copyright (C) 2020 Jiajun Lv
 * Copyright (C) 2020 Kewei Hu
 * Copyright (C) 2020 Jinhong Xu
 * Copyright (C) 2020 LI_Calib Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include <core/lidar_odometry.h>
#include <utils/pcl_utils.h>
#include <utils/math_utils.h>

namespace licalib {

LiDAROdometry::LiDAROdometry(double ndt_resolution)
        : map_cloud_(new VPointCloud()) 
{
  ndt_omp_ = ndtInit(ndt_resolution);
}

// wgh-- 初始化NDT配准器。
pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr
LiDAROdometry::ndtInit(double ndt_resolution) 
{
  auto ndt_omp = pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr(
          new pclomp::NormalDistributionsTransform<VPoint, VPoint>());
  ndt_omp->setResolution(ndt_resolution);
  ndt_omp->setNumThreads(4);
  ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
  ndt_omp->setTransformationEpsilon(1e-3);
  ndt_omp->setStepSize(0.01);
  ndt_omp->setMaximumIterations(50);
  return ndt_omp;
}

// wgh-- 喂入新的一帧点云并进行NDT配准+关键帧过滤。
// wgh-- 注意后两个形参有默认参数，pose_predict=Identity，update_map=true。
void LiDAROdometry::feedScan(double timestamp,
                             VPointCloud::Ptr cur_scan,
                             Eigen::Matrix4d pose_predict,
                             const bool update_map) 
{
  OdomData odom_cur;
  odom_cur.timestamp = timestamp;
  odom_cur.pose = Eigen::Matrix4d::Identity();

  VPointCloud::Ptr scan_in_target(new VPointCloud());
  // 如果是第一帧点云，直接按关键帧插入，该帧的位姿按identity处理。
  if (map_cloud_->empty()) {
    scan_in_target = cur_scan;
  } 
  // 非第一帧点云，进行正常的ndt配准。
  else {
    // 输入的位姿预测是帧间增量，因此这里要在上一帧位姿上做累乘，形成world下的位姿预测。
    Eigen::Matrix4d T_LtoM_predict = odom_data_.back().pose * pose_predict;
    registration(cur_scan, T_LtoM_predict, odom_cur.pose, scan_in_target);
  }
  odom_data_.push_back(odom_cur);

  if (update_map/*默认true*/) {
    updateKeyScan(cur_scan, odom_cur);
  }
}

// wgh-- 执行ndt配准的函数。
void LiDAROdometry::registration(const VPointCloud::Ptr& cur_scan,
                                 const Eigen::Matrix4d& pose_predict,
                                 Eigen::Matrix4d& pose_out,
                                 VPointCloud::Ptr scan_in_target) {
  VPointCloud::Ptr p_filtered_cloud(new VPointCloud());
  downsampleCloud(cur_scan, p_filtered_cloud, 0.5); //50cm体素过滤，点数少可保证效率。

  ndt_omp_->setInputSource(p_filtered_cloud);
  ndt_omp_->align(*scan_in_target, pose_predict.cast<float>());

  pose_out = ndt_omp_->getFinalTransformation().cast<double>();
}

// wgh-- 尝试添加新的关键帧。
void LiDAROdometry::updateKeyScan(const VPointCloud::Ptr& cur_scan,
                                  const OdomData& odom_data) 
{
  if (checkKeyScan(odom_data)) {
    VPointCloud::Ptr filtered_cloud(new VPointCloud());
    downsampleCloud(cur_scan, filtered_cloud, 0.1); //10cm体素滤波

    VPointCloud::Ptr scan_in_target(new VPointCloud()); //直接变换到world坐标系下，省得后边还要变换。
    pcl::transformPointCloud(*filtered_cloud, *scan_in_target, odom_data.pose);

    // 增量式更新ndt配准的target地图。
    *map_cloud_ += *scan_in_target;
    ndt_omp_->setInputTarget(map_cloud_);
    key_frame_index_.push_back(odom_data_.size());
  }
}

// wgh-- 关键帧判断(也即运动过滤)。
bool LiDAROdometry::checkKeyScan(const OdomData& odom_data) 
{
  // 注意这里是static变量，意味着会被不断更新，总是表示上一个关键帧的位姿。
  static Eigen::Vector3d position_last(0,0,0);
  static Eigen::Vector3d ypr_last(0,0,0);

  Eigen::Vector3d position_now = odom_data.pose.block<3,1>(0,3);
  double dist = (position_now - position_last).norm();

  const Eigen::Matrix3d rotation (odom_data.pose.block<3,3> (0,0));
  Eigen::Vector3d ypr = mathutils::R2ypr(rotation);
  Eigen::Vector3d delta_angle = ypr - ypr_last;
  for (size_t i = 0; i < 3; i++)
    delta_angle(i) = normalize_angle(delta_angle(i));
  delta_angle = delta_angle.cwiseAbs();

  // 运动过滤参数：平移20cm，或者r/p/y任一个变化了5度。
  if (key_frame_index_.size() == 0 || dist > 0.2
     || delta_angle(0) > 5.0 || delta_angle(1) > 5.0 || delta_angle(2) > 5.0) {
    position_last = position_now;
    ypr_last = ypr;
    return true;
  }
  return false;
}

// wgh-- 如名，将输入的点云设置为下一次配准的参考点云(也即所谓target)。该函数用于外部调用。
void LiDAROdometry::setTargetMap(VPointCloud::Ptr map_cloud_in) 
{
  map_cloud_->clear();
  pcl::copyPointCloud(*map_cloud_in, *map_cloud_);
  ndt_omp_->setInputTarget(map_cloud_);
}

// wgh-- 清楚掉整个关键帧队列数据。
void LiDAROdometry::clearOdomData() 
{
  key_frame_index_.clear();
  odom_data_.clear();
}


}



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
#include <ui/calib_helper.h>
#include <core/scan_undistortion.h>
#include <utils/tic_toc.h>

#include <boost/filesystem.hpp>
#include <memory>
#include <sstream>

namespace licalib {

/**
 * @brief 核心类构造函数，完成rosbag包的读入和数据转存，以及各算法模块的初始化。
 * @param nh 
**/
CalibrHelper::CalibrHelper(ros::NodeHandle& nh)
        : calib_step_(Start),
          iteration_step_(0),
          opt_time_offset_(false),
          plane_lambda_(0.6),
          ndt_resolution_(0.5),
          associated_radius_(0.05) 
{
  std::string topic_lidar;
  double bag_start, bag_durr; // wgh-- 从哪个位置开始读bag包；读取多久时间？为负，则读取至bag末尾。
  double scan4map;
  double knot_distance;
  double time_offset_padding;

  // 从ROS参数服务中加载参数。
  nh.param<std::string>("path_bag", bag_path_, "V1_01_easy.bag");
  nh.param<std::string>("topic_imu", topic_imu_, "/imu0");
  nh.param<std::string>("topic_lidar", topic_lidar, "/velodyne_packets");
  nh.param<double>("bag_start", bag_start, 0);
  nh.param<double>("bag_durr", bag_durr, -1);
  nh.param<double>("scan4map", scan4map, 15);
  nh.param<double>("ndtResolution", ndt_resolution_, 0.5);
  nh.param<double>("time_offset_padding", time_offset_padding, 0.015);
  nh.param<double>("knot_distance", knot_distance, 0.02); // 重要参数，B样条曲线控制点间距。

  // wgh-- 如果找不到rosbag包，把系统状态写为error，阻止系统执行后续算法动作。
  if (!createCacheFolder(bag_path_)) {
    calib_step_ = Error;
  }

  // wgh-- 以下代码段负责 (1-判断雷达类型是否符合 (2-从rosbag读取点云+imu数据
  {
    std::string lidar_model;
    nh.param<std::string>("LidarModel", lidar_model, "VLP_16");
    IO::LidarModelType lidar_model_type = IO::LidarModelType::VLP_16;
    if (lidar_model == "VLP_16") {
      lidar_model_type = IO::LidarModelType::VLP_16;
    } else {
      calib_step_ = Error;
      ROS_WARN("LiDAR model %s not support yet.", lidar_model.c_str());
    }
    /// read dataset
    // wgh-- 从rosbag读入点云数据和imu数据，并按时间戳裁剪头和尾确保两类数据时间重叠。
    std::cout << "\nLoad dataset from " << bag_path_ << std::endl;
    IO::LioDataset lio_dataset_temp(lidar_model_type);
    lio_dataset_temp.read(bag_path_, topic_imu_, topic_lidar, bag_start, bag_durr);
    dataset_reader_ = lio_dataset_temp.get_data();
    dataset_reader_->adjustDataset();
  }

  // wgh-- 初始化各个算法模块
  map_time_ = dataset_reader_->get_start_time();
  scan4map_time_ = map_time_ + scan4map;
  double end_time = dataset_reader_->get_end_time();

  traj_manager_ = std::make_shared<TrajectoryManager>(
          map_time_, end_time, knot_distance, time_offset_padding);

  scan_undistortion_ = std::make_shared<ScanUndistortion>(
          traj_manager_, dataset_reader_);

  lidar_odom_ = std::make_shared<LiDAROdometry>(ndt_resolution_);

  rotation_initializer_ = std::make_shared<InertialInitializer>();

  surfel_association_ = std::make_shared<SurfelAssociation>(
          associated_radius_, plane_lambda_);
}

// wgh-- 在rosbag包所在路径创建一个同名文件夹，用于保存标定结果。
bool CalibrHelper::createCacheFolder(const std::string& bag_path) 
{
  boost::filesystem::path p(bag_path);
  if (p.extension() != ".bag") {
    return false;
  }
  cache_path_ = p.parent_path().string() + "/" + p.stem().string();
  boost::filesystem::create_directory(cache_path_);
  return true;
}

// wgh-- 标定初始化只干一件事：初始化旋转外参。
void CalibrHelper::Initialization() 
{
  if (Start != calib_step_) {
    ROS_WARN("[Initialization] Need status: Start.");
    return;
  }

  // 把所有的imu数据喂到算法模块内部(核心类，维护B样条曲线，执行batch优化)。
  for (const auto& imu_data: dataset_reader_->get_imu_data()) {
    traj_manager_->feedIMUData(imu_data);
  }
  // 把所有imu数据喂进来后，马上初始化SO3轨迹，初始化包括两个内容：
  // 1-把所有imu测量的角速度喂进去，2-设定初始时刻的位姿为identity(表示世界)。
  traj_manager_->initialSO3TrajWithGyro();

  // 关于以下点云格式：TPointCloud是<PointXYZIT>点云，VPointCloud是<pcl::PointXYZI>点云。
  for(const TPointCloud& raw_scan: dataset_reader_->get_scan_data()) {
    VPointCloud::Ptr cloud(new VPointCloud);
    TPointCloud2VPointCloud(raw_scan.makeShared(), cloud);
    double scan_timestamp = pcl_conversions::fromPCL(raw_scan.header.stamp).toSec();

    // 在无初值的情况下，调用NDT激光里程计进行帧间配准。
    lidar_odom_->feedScan(scan_timestamp, cloud);

    // 只有累积到30帧关键帧时，才会开始尝试初始化旋转外参；若不成功，就每新增10关键帧再尝试一次。
    if (lidar_odom_->get_odom_data().size() < 30
        || (lidar_odom_->get_odom_data().size() % 10 != 0))
      continue;
    // 如果旋转外参初始化成功，则保存旋转外参，退出；否则，继续累积数据，继续尝试，直至初始化成功。
    if (rotation_initializer_->EstimateRotation(
          traj_manager_, lidar_odom_->get_odom_data())) 
    {
      Eigen::Quaterniond qItoLidar = rotation_initializer_->getQ_ItoS();
      // 初始化成功后，把旋转外参初值给到B样条轨迹里！
      traj_manager_->getCalibParamManager()->set_q_LtoI(qItoLidar.conjugate());

      Eigen::Vector3d euler_ItoL = qItoLidar.toRotationMatrix().eulerAngles(0,1,2);
      std::cout << "[Initialization] Done. Euler_ItoL initial degree: "
                << (euler_ItoL*180.0/M_PI).transpose() << std::endl;
      calib_step_ = InitializationDone;
      break; // 旋转外参初始化成功，退出循环。
    }
  }
  if (calib_step_ != InitializationDone)
    ROS_WARN("[Initialization] fails.");
}

// wgh-- 用旋转外参去运动畸变+NDT激光里程计，构建Surfels地图+地图去畸变，构建point2surfel关联。
void CalibrHelper::DataAssociation() 
{
  std::cout << "[Association] start ...." << std::endl;
  TicToc timer;
  timer.tic();

  /// set surfel pap
  // wgh-- 如果刚刚完成旋转外参初始化，这里开始帧去畸变+NDT激光里程计建图(得到surfel地图)，并对surfel地图去畸变。
  if (InitializationDone == calib_step_ ) {
    Mapping(); 
    scan_undistortion_->undistortScanInMap(lidar_odom_->get_odom_data());

    surfel_association_->setSurfelMap(lidar_odom_->getNDTPtr(), map_time_);
  }
  // wgh-- 如果已经进行到迭代Batch优化环节了，只需对地图直接去畸变即可。
  else if (BatchOptimizationDone == calib_step_ || RefineDone == calib_step_) {
    scan_undistortion_->undistortScanInMap();

    plane_lambda_ = 0.7;
    surfel_association_->setPlaneLambda(plane_lambda_);
    auto ndt_omp = LiDAROdometry::ndtInit(ndt_resolution_);
    ndt_omp->setInputTarget(scan_undistortion_->get_map_cloud());
    surfel_association_->setSurfelMap(ndt_omp, map_time_);
  }
  else {
      ROS_WARN("[DataAssociation] Please follow the step.");
      return;
  }

  /// get association
  // 获得point2surface关联！
  for (auto const &scan_raw : dataset_reader_->get_scan_data()) {
    auto iter = scan_undistortion_->get_scan_data_in_map().find(
            scan_raw.header.stamp);
    if (iter == scan_undistortion_->get_scan_data_in_map().end()) {
      continue;
    }
    // 计算帧与surfel_association_内部已经保存的平面信息的关联。
    surfel_association_->getAssociation(iter->second, scan_raw.makeShared(), 2);
  }
  surfel_association_->averageTimeDownSmaple();
  std::cout << "Surfel point number: "
            << surfel_association_->get_surfel_points().size() << std::endl;
  std::cout<<GREEN<<"[Association] "<<timer.toc()<<" ms"<<RESET<<std::endl;

  if (surfel_association_->get_surfel_points().size() > 10){
    calib_step_ = DataAssociationDone;
  } else {
    ROS_WARN("[DataAssociation] fails.");
  }
}

// wgh-- 如名，执行batch优化。
void CalibrHelper::BatchOptimization() {
  if (DataAssociationDone != calib_step_) {
    ROS_WARN("[BatchOptimization] Need status: DataAssociationDone.");
    return;
  }
  std::cout << "\n================ Iteration " << iteration_step_ << " ==================\n";

  TicToc timer;
  timer.tic();
  // wgh-- 执行优化的地方(喂入point2surface约束，加上原有的imu角速度&加速度约束，进行优化)
  traj_manager_->trajInitFromSurfel(surfel_association_, opt_time_offset_);

  calib_step_ = BatchOptimizationDone;
  saveCalibResult(cache_path_ + "/calib_result.csv");
  std::cout<<GREEN<<"[BatchOptimization] "<<timer.toc()<<" ms"<<RESET<<std::endl;
}

// wgh-- 重复“Data_Association + Batch_Optimization”过程
void CalibrHelper::Refinement() 
{
  if (BatchOptimizationDone > calib_step_) {
    ROS_WARN("[Refinement] Need status: BatchOptimizationDone.");
    return;
  }
  iteration_step_++;
  std::cout << "\n================ Iteration " << iteration_step_ << " ==================\n";

  DataAssociation();
  if (DataAssociationDone != calib_step_) {
    ROS_WARN("[Refinement] Need status: DataAssociationDone.");
    return;
  }
  TicToc timer;
  timer.tic();

  // wgh-- 执行优化的地方
  traj_manager_->trajInitFromSurfel(surfel_association_, opt_time_offset_);
  calib_step_ = RefineDone;
  saveCalibResult(cache_path_ + "/calib_result.csv");

  std::cout<<GREEN<<"[Refinement] "<<timer.toc()<<" ms"<<RESET<<std::endl;
}

// wgh-- 本函数就是去畸变+ndt激光里程计做帧间配准和建图；形参`默认参数=false`。
void CalibrHelper::Mapping(bool relocalization) 
{
  bool update_map = true;
  // 如果是在原来里程计信息的基础上做位姿推断(re-localiztion的含义)，就执行clear。
  if (relocalization/*默认false*/) {
    lidar_odom_->clearOdomData();
    update_map = false;
  } 
  // 否则，重新构建激光里程计(相当于全新的)。
  else {
    scan_undistortion_->undistortScan(); // 逐帧执行去畸变，随后在下边被喂入NDT配准。
    lidar_odom_ = std::make_shared<LiDAROdometry>(ndt_resolution_);
  }

  double last_scan_t = 0;
  for (const auto& scan_raw: dataset_reader_->get_scan_data()) {
    double scan_t = pcl_conversions::fromPCL(scan_raw.header.stamp).toSec();
    if (scan_t > scan4map_time_) update_map = false;
    // 获取去畸变后的点云。
    auto iter = scan_undistortion_->get_scan_data().find(scan_raw.header.stamp);
    if (iter != scan_undistortion_->get_scan_data().end()) {
      Eigen::Matrix4d pose_predict = Eigen::Matrix4d::Identity();
      Eigen::Quaterniond q_L2toL1 = Eigen::Quaterniond::Identity();
      if (last_scan_t > 0 &&
          traj_manager_->evaluateLidarRelativeRotation(last_scan_t, scan_t, q_L2toL1)) {
        pose_predict.block<3,3>(0,0) = q_L2toL1.toRotationMatrix();
      }
      // 在有较准确初值的情况下，调用NDT激光里程计，逐帧做位姿推断(建图)。
      lidar_odom_->feedScan(scan_t, iter->second, pose_predict, update_map);
      last_scan_t = scan_t;
    }
  }
}

// wgh-- 如函数名。
void CalibrHelper::saveCalibResult(const std::string& calib_result_file) const 
{
  // 如果保存保定结果的文件不存在，则创建文件。
  if (!boost::filesystem::exists(calib_result_file)) {
    std::ofstream outfile;
    outfile.open(calib_result_file, std::ios::app);
    outfile << "bag_path" << ","
            << "imu_topic" << "," << "map_time" << "," << "iteration_step" << ","
            << "p_IinL.x" << "," << "p_IinL.y" << "," << "p_IinL.z" << ","
            << "q_ItoL.x" << "," << "q_ItoL.y" << "," << "q_ItoL" << ","
            << "q_ItoL.w" << ","
            << "time_offset" << ","
            << "gravity.x" << "," << "gravity.y" << "," << "gravity.z" << ","
            << "gyro_bias.x" << "," << "gyro_bias.y" << "," <<"gyro_bias.z" << ","
            << "acce_bias.z" << "," << "acce_bias.y" << "," <<"acce_bias.z" << "\n";
    outfile.close();
  }

  std::stringstream ss;
  ss << bag_path_;
  ss << "," << topic_imu_;
  ss << "," << map_time_;
  ss << "," << iteration_step_;
  std::string info;
  ss >> info;
  // 保存标定结果到文件中。
  traj_manager_->getCalibParamManager()->save_result(calib_result_file, info);
}

// wgh-- 如函数名。
void CalibrHelper::saveMap() const 
{
  if (calib_step_ <= Start)
    return;
  std::string NDT_target_map_path = cache_path_ + "/NDT_target_map.pcd";
  lidar_odom_->saveTargetMap(NDT_target_map_path);

  std::string surfel_map_path = cache_path_ + "/surfel_map.pcd";
  surfel_association_->saveSurfelsMap(surfel_map_path);

  if (RefineDone == calib_step_) {
    std::string refined_map_path = cache_path_ + "/refined_map.pcd";
    std::cout << "Save refined map to " << refined_map_path << "; size: "
              << scan_undistortion_->get_map_cloud()->size() << std::endl;
    pcl::io::savePCDFileASCII(refined_map_path, *scan_undistortion_->get_map_cloud());
  }
}

}

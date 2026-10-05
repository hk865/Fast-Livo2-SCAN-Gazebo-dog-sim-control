#include "LIVMapper.h"
#include <cassert>
#include <iostream>
#include <thread>

sensor_msgs::msg::Imu::SharedPtr sample(double stamp)
{
  auto imu = std::make_shared<sensor_msgs::msg::Imu>();
  const int64_t ns = std::llround(stamp * 1e9);
  imu->header.stamp.sec = ns / 1000000000;
  imu->header.stamp.nanosec = ns % 1000000000;
  imu->linear_acceleration.z = 9.81;
  return imu;
}

void partition(LIVMapper &mapper, double delta, int lidar_type, bool expect_current)
{
  mapper.p_pre->lidar_type = lidar_type;
  mapper.slam_mode_ = LIVO;
  mapper.lid_raw_data_buffer.clear(); mapper.lid_header_time_buffer.clear();
  mapper.img_buffer.clear(); mapper.img_time_buffer.clear(); mapper.imu_buffer.clear();
  mapper.img_buffer.emplace_back(1, 1, CV_8UC1);
  mapper.img_time_buffer.push_back(1.1);
  for (auto frame : {std::pair<double,float>{1.0, 1}, {1.1 + delta, 2}, {1.2, 3}})
  {
    auto cloud = std::make_shared<PointCloudXYZI>();
    PointType point; point.x = frame.second; point.y = 0; point.z = 1; point.curvature = 0;
    cloud->push_back(point);
    mapper.lid_raw_data_buffer.push_back(cloud);
    mapper.lid_header_time_buffer.push_back(frame.first);
  }
  for (int i = 0; i <= 20; ++i) mapper.imu_buffer.push_back(sample(1.0 + .01 * i));
  LidarMeasureGroup measurement;
  measurement.last_lio_update_time = 1.0;
  assert(mapper.sync_packages(measurement));
  bool current = false;
  for (const auto &point : measurement.pcl_proc_cur->points) if (point.x == 2) current = true;
  std::cout << "partition type=" << lidar_type << " delta=" << delta
            << " current=" << current << " expected=" << expect_current
            << " pending=" << measurement.pcl_proc_next->size() << std::endl;
  if (current != expect_current) throw std::runtime_error("Current synchronous instantaneous cloud was postponed");
}

void zero_deskew(bool legacy, int lidar_type = 0)
{
  ImuProcess process;
  process.lidar_type = lidar_type;
  process.set_imu_init_frame_num(300);
  process.configure_stationary_initialization(true, StationaryImuInitialization::Limits());
  StatesGroup state;
  LidarMeasureGroup initialization;
  initialization.lio_vio_flg = LIO;
  MeasureGroup init;
  init.lio_time = 3.99;
  for (int i = 0; i < 300; ++i) init.imu.push_back(sample(1 + .01 * i));
  initialization.measures.push_back(init);
  process.Process2(initialization, state, std::make_shared<PointCloudXYZI>());
  assert(!process.imu_need_init);
  if (!legacy && lidar_type == 0) assert(std::abs(initialization.last_lio_update_time - 3.99) < 1e-9);
  // Complete one stationary propagation using the real implementation before
  // testing the start-boundary point, so this is independent of initial storage.
  LidarMeasureGroup warm;
  warm.lio_vio_flg = LIO;
  MeasureGroup m; m.lio_time = 4.0; m.imu.push_back(sample(4.0)); warm.measures.push_back(m);
  PointType stationary; stationary.x = 10; stationary.y = 0; stationary.z = 0; stationary.curvature = 0;
  warm.pcl_proc_cur->push_back(stationary);
  PointCloudXYZI output;
  process.UndistortPcl(warm, state, output);
  assert(std::abs(output[0].x - 10.0) < 1e-5);
  assert(state.pos_end.norm() < 1e-9);
  state.pos_end.setZero(); state.vel_end = V3D(1, 0, 0);
  LidarMeasureGroup moving;
  moving.lio_vio_flg = LIO;
  MeasureGroup motion; motion.lio_time = 4.1;
  for (int i = 1; i <= 10; ++i) motion.imu.push_back(sample(4.0 + .01 * i));
  moving.measures.push_back(motion);
  for (int i = 0; i < 4; ++i) moving.pcl_proc_cur->push_back(stationary);
  PointType current = stationary; current.curvature = 100;
  moving.pcl_proc_cur->push_back(current);
  process.UndistortPcl(moving, state, output);
  const double expected = legacy || lidar_type != 0 ? 10.0 : 9.9;
  std::cout << "start-point x=" << output[0].x << " expected=" << expected
            << " propagated-position=" << state.pos_end.transpose() << std::endl;
  for (int i = 0; i < 4; ++i) assert(std::abs(output[i].x - expected) < 1e-5);
  assert(std::abs(output[4].x - 10.0) < 1e-5);
}

int main(int argc, char **argv)
{
  const bool legacy = argc > 1 && std::string(argv[1]) == "--legacy-evidence";
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions board_options; board_options.automatically_declare_parameters_from_overrides(true);
  auto board = std::make_shared<rclcpp::Node>("parameter_blackboard", board_options);
  rclcpp::executors::SingleThreadedExecutor board_executor; board_executor.add_node(board);
  std::thread board_thread([&] { board_executor.spin(); });
  bool passed = true;
  try
  {
    auto node = std::make_shared<rclcpp::Node>("lidar_regression");
    LIVMapper mapper(node, "lidar_regression");
    partition(mapper, 0, 0, !legacy);
    partition(mapper, -0.01, 0, true);
    partition(mapper, +0.01, 0, false);
    if (!legacy)
    {
      partition(mapper, +2e-9, 0, true);
      partition(mapper, -2e-9, 0, true);
    }
    partition(mapper, 0, 2, false); // Timed sensors preserve their old strict boundary.
    zero_deskew(legacy);
    zero_deskew(false, 2); // Timed-sensor zero-point behavior is unchanged.
  }
  catch (const std::exception &error) { passed = false; std::cerr << error.what() << std::endl; }
  board_executor.cancel(); board_thread.join(); rclcpp::shutdown();
  std::cout << (passed ? "linked production instantaneous lidar regression passed" : "regression failed") << std::endl;
  return passed ? 0 : 1;
}

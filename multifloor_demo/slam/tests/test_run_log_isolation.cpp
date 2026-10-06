#include "LIVMapper.h"
#include <cassert>
#include <fstream>
#include <iostream>
#include <thread>

std::string contents(const std::filesystem::path &path)
{
  std::ifstream file(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
}

void initialized_imu(ImuProcess &process, const std::string &marker)
{
  process.lidar_type = 0;
  process.set_imu_init_frame_num(300);
  process.configure_stationary_initialization(true, StationaryImuInitialization::Limits());
  StatesGroup state;
  LidarMeasureGroup measurement;
  measurement.lio_vio_flg = LIO;
  MeasureGroup group; group.lio_time = 3.99;
  for (int i = 0; i < 300; ++i)
  {
    auto sample = std::make_shared<sensor_msgs::msg::Imu>();
    const int64_t ns = 1000000000LL + i * 10000000LL;
    sample->header.stamp.sec = ns / 1000000000;
    sample->header.stamp.nanosec = ns % 1000000000;
    sample->linear_acceleration.z = 9.81;
    group.imu.push_back(sample);
  }
  measurement.measures.push_back(group);
  process.Process2(measurement, state, std::make_shared<PointCloudXYZI>());
  assert(!process.imu_need_init);
  assert(process.fout_imu.is_open());
  process.fout_imu << marker << std::flush;
}

void timestamp_precision(const std::filesystem::path &run)
{
  setenv("DEMO_RUN_DIR", run.c_str(), 1);
  ImuProcess process; process.lidar_type = 0; process.first_lidar_time = 0;
  process.set_imu_init_frame_num(300);
  process.configure_stationary_initialization(true, StationaryImuInitialization::Limits());
  StatesGroup state;
  auto sample = [](int64_t ns) {
    auto result = std::make_shared<sensor_msgs::msg::Imu>();
    result->header.stamp.sec = ns / 1000000000;
    result->header.stamp.nanosec = ns % 1000000000;
    result->linear_acceleration.z = 9.81;
    return result;
  };
  LidarMeasureGroup initialization; initialization.lio_vio_flg = LIO;
  MeasureGroup init; init.lio_time = 1000.122;
  for (int i = 0; i < 300; ++i) init.imu.push_back(sample(997132000000LL + i * 10000000LL));
  initialization.measures.push_back(init);
  process.Process2(initialization, state, std::make_shared<PointCloudXYZI>());
  assert(!process.imu_need_init);
  double previous = 1000.122;
  for (const auto times : {std::vector<int64_t>{1000123000000LL, 1000124000000LL},
                          {1000125000000LL}, {2000001000000LL, 2000002000000LL}})
  {
    LidarMeasureGroup measurement; measurement.lio_vio_flg = LIO;
    MeasureGroup group; group.lio_time = times.back() / 1e9;
    for (auto ns : times) group.imu.push_back(sample(ns));
    measurement.measures.push_back(group);
    PointType point; point.x = 10; point.y = 0; point.z = 0;
    point.curvature = (group.lio_time - previous) * 1000;
    measurement.pcl_proc_cur->push_back(point);
    process.Process2(measurement, state, std::make_shared<PointCloudXYZI>());
    previous = group.lio_time;
  }
  process.fout_imu.flush();
  std::ifstream input(process.log_directory / "imu.txt");
  std::vector<double> logged;
  double stamp, unused;
  while (input >> stamp)
  {
    logged.push_back(stamp);
    for (int i = 0; i < 6; ++i) input >> unused;
  }
  assert(logged.size() == 5);
  for (size_t i = 1; i < logged.size(); ++i) assert(logged[i] > logged[i - 1]);
  for (double expected : {1000.123, 1000.124, 2000.001})
  {
    bool found = false;
    for (double value : logged) if (std::abs(value - expected) <= 1e-9) found = true;
    assert(found);
  }
  std::cout << "actual IMU propagation log stamps 1000.123, 1000.124, 2000.001 remain unique within 1 ns\n";
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  const std::filesystem::path parent = std::getenv("DEMO_RUN_DIR");
  const auto run_a = parent / "run_a";
  const auto run_b = parent / "run_b";
  const auto source_log = std::filesystem::path(ROOT_DIR) / "Log";
  const auto old_pre = contents(source_log / "mat_pre.txt");
  const auto old_out = contents(source_log / "mat_out.txt");
  const auto old_imu = contents(source_log / "imu.txt");
  rclcpp::NodeOptions options; options.automatically_declare_parameters_from_overrides(true);
  auto board = std::make_shared<rclcpp::Node>("parameter_blackboard", options);
  rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(board);
  std::thread thread([&] { executor.spin(); });
  bool passed = true;
  try
  {
    setenv("DEMO_RUN_DIR", run_a.c_str(), 1);
    auto node_a = std::make_shared<rclcpp::Node>("log_fixture_a");
    LIVMapper a(node_a, "log_fixture_a");
    a.fout_pre << "A-pre" << std::flush;
    a.fout_out << "A-out" << std::flush;
    setenv("DEMO_RUN_DIR", run_b.c_str(), 1);
    auto node_b = std::make_shared<rclcpp::Node>("log_fixture_b");
    LIVMapper b(node_b, "log_fixture_b");
    b.fout_pre << "B-pre" << std::flush;
    b.fout_out << "B-out" << std::flush;
    // Initialize A after the environment has switched to B. Its constructor
    // must already have captured A, without opening any shared source log.
    initialized_imu(*a.p_imu, "A-imu");
    initialized_imu(*b.p_imu, "B-imu");
    assert(a.log_directory == run_a / "fastlivo_debug");
    assert(a.p_imu->log_directory == a.log_directory);
    assert(b.p_imu->log_directory == b.log_directory);
    assert(contents(a.log_directory / "mat_pre.txt") == "A-pre");
    assert(contents(a.log_directory / "mat_out.txt") == "A-out");
    assert(contents(a.log_directory / "imu.txt") == "A-imu");
    assert(contents(b.log_directory / "mat_pre.txt") == "B-pre");
    assert(contents(b.log_directory / "mat_out.txt") == "B-out");
    assert(contents(b.log_directory / "imu.txt") == "B-imu");
    timestamp_precision(parent / "precision");
    unsetenv("DEMO_RUN_DIR");
    ImuProcess standalone;
    initialized_imu(standalone, "independent-no-env");
    assert(standalone.log_directory != a.log_directory && standalone.log_directory != b.log_directory);
    assert(standalone.log_directory != source_log);
    assert(contents(standalone.log_directory / "imu.txt") == "independent-no-env");
    assert(contents(source_log / "mat_pre.txt") == old_pre);
    assert(contents(source_log / "mat_out.txt") == old_out);
    assert(contents(source_log / "imu.txt") == old_imu);
    std::cout << "actual Mapper construction/Process2 run A, B and no-env logs are isolated; shared source files unchanged\n";
    std::filesystem::remove_all(standalone.log_directory.parent_path());
  }
  catch (const std::exception &error) { passed = false; std::cerr << error.what() << std::endl; }
  executor.cancel(); thread.join(); rclcpp::shutdown();
  return passed ? 0 : 1;
}

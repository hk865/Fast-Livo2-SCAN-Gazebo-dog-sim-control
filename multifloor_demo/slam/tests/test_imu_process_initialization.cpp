#include "IMU_Processing.h"
#include <cassert>
#include <iostream>

void feed(ImuProcess &p, StatesGroup &s, double stamp, V3D a, V3D w)
{
  auto imu = std::make_shared<sensor_msgs::msg::Imu>();
  const int64_t ns=std::llround(stamp*1e9);
  imu->header.stamp.sec=ns/1000000000;
  imu->header.stamp.nanosec=ns%1000000000;
  imu->linear_acceleration.x=a.x(); imu->linear_acceleration.y=a.y(); imu->linear_acceleration.z=a.z();
  imu->angular_velocity.x=w.x(); imu->angular_velocity.y=w.y(); imu->angular_velocity.z=w.z();
  LidarMeasureGroup lidar;
  lidar.lio_vio_flg=LIO;
  MeasureGroup measurement;
  measurement.lio_time=stamp; measurement.imu.push_back(imu);
  lidar.measures.push_back(measurement);
  p.Process2(lidar,s,std::make_shared<PointCloudXYZI>());
}
int main()
{
  ImuProcess p; StatesGroup s;
  p.lidar_type=0;
  p.set_imu_init_frame_num(300);
  p.configure_stationary_initialization(true,StationaryImuInitialization::Limits());
  for(int i=0;i<280;++i) { feed(p,s,1.+i*.01,V3D(0,0,9.81),V3D::Zero()); assert(p.imu_need_init); }
  feed(p,s,3.8,V3D(0,0,9.81),V3D(.2,0,0)); assert(p.imu_need_init);
  for(int i=0;i<299;++i) { feed(p,s,3.81+i*.01,V3D(.03,.01,9.81),V3D::Zero()); assert(p.imu_need_init); }
  feed(p,s,6.80,V3D(.03,.01,9.81),V3D::Zero());
  assert(!p.imu_need_init);
  assert((s.gravity-(-V3D(.03,.01,9.81).normalized()*G_m_s2)).norm()<1e-9);
  std::cout << "production ImuProcess integration passed: motion resets; 299 pending, 300 complete\n";
}

#include "stationary_imu_initialization.h"
#include <cassert>
#include <iostream>
#include <limits>

using V = Eigen::Vector3d;
void stable(StationaryImuInitialization &g, double start, int n, V acc = V(0,0,9.81))
{
  for (int i = 0; i < n; ++i) g.add(start + i * .01, acc, V::Zero());
}
void tests()
{
  StationaryImuInitialization g;
  stable(g, 1., 299); assert(!g.ready(300));
  g.add(3.99, V(0,0,9.81), V::Zero()); assert(g.ready(300));
  // A real motion sample invalidates all 300 preceding samples.
  g.add(4., V(0,0,9.81), V(.1,0,0)); assert(g.count == 0);
  stable(g, 4.01, 299); assert(!g.ready(300));
  g.add(7., V(0,0,9.81), V::Zero()); assert(g.ready(300));
  g.add(7.01, V(0,0,0), V::Zero()); assert(g.count == 0);
  g.add(7.02, V(0,0,9.81), V::Zero());
  g.add(7.02, V(0,0,9.81), V::Zero()); assert(g.count == 0);
  g.add(7.03, V(0,0,9.81), V::Zero());
  g.add(7.2, V(0,0,9.81), V::Zero()); assert(g.count == 1);
  g.add(7.21, V(std::numeric_limits<double>::quiet_NaN(),0,9.81), V::Zero()); assert(g.count == 0);
  g.reset(); stable(g, 1., 10);
  for (int i=0;i<40;++i) g.add(1.1+i*.01,V(i%2?.8:-.8,0,9.81),V::Zero());
  assert(g.reset_count>0 && !g.ready(300));
  // Sampling faster cannot replace the required stationary time span.
  g.reset(); for(int i=0;i<300;++i) g.add(1.+i*.001,V(0,0,9.81),V::Zero());
  assert(g.count==300 && !g.ready(300));
  // Mount/body tilt does not invalidate gravity-magnitude checks.
  g.reset(); stable(g,1.,300,V(4.905,0,9.81*std::sqrt(.75)));
  assert(g.ready(300));
  std::cout << "stationary initialization contract cases passed\n";
}
int main(int argc, char **)
{
  if(argc==1) { tests(); return 0; }
  StationaryImuInitialization g;
  double stamp, ax, ay, az, wx, wy, wz;
  while(std::cin >> stamp >> ax >> ay >> az >> wx >> wy >> wz)
  {
    g.add(stamp,V(ax,ay,az),V(wx,wy,wz));
    if(g.ready(300)) break;
  }
  const V s=g.acceleration_std();
  std::cout.precision(12);
  std::cout << "{\"ready\":" << (g.ready(300)?"true":"false")
    << ",\"count\":"<<g.count<<",\"first_stamp\":"<<g.first_stamp<<",\"last_stamp\":"<<g.last_stamp
    << ",\"span_s\":"<<g.span()<<",\"resets\":"<<g.reset_count
    << ",\"mean_acc\":["<<g.mean_acc.x()<<","<<g.mean_acc.y()<<","<<g.mean_acc.z()<<"]"
    << ",\"acc_std\":["<<s.x()<<","<<s.y()<<","<<s.z()<<"]"
    << ",\"max_gyro_norm\":"<<g.max_gyro_norm<<"}\n";
}

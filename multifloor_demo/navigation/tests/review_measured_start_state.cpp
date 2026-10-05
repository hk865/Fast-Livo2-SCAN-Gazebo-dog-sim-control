// Independent tests against the actual production state policy header.
#include <plan_manage/measured_start_state.h>
#include <iostream>
#include <limits>
#include <stdexcept>
using scan_planner::MeasuredStartState;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);std::cout<<message<<" PASS\n";}
int main(){
  MeasuredStartState state;
  state.update(Eigen::Vector3d(-.157,.048,.000442),1000000000,false);
  require(state.velocity(false).norm()<=.120000000001,"measured velocity spike bounded to physical speed");
  state.stop();
  require(state.velocity(false).isZero(0),"freeze edge clears prior filtered motion before next odom");
  state.update(Eigen::Vector3d(-.020989,-.004017,.000577),1100000000,true);
  require(state.velocity(true).isZero(0)&&state.velocity(false).isZero(0),"actual run111 frozen measurement cannot restore old planned derivative");
  state.update(Eigen::Vector3d(.1,.02,0),1200000000,false);
  auto expected=state.velocity(false);
  state.update(Eigen::Vector3d(-100,100,100),1200000000,false);
  state.update(Eigen::Vector3d(-100,100,100),1199999999,false);
  require((state.velocity(false)-expected).isZero(0),"duplicate and old timestamps cannot change boundary velocity");
  state.update(Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),0,0),1300000000,false);
  require((state.velocity(false)-expected).isZero(0),"nonfinite measured velocity rejected");
  MeasuredStartState dense,sparse;
  dense.update(Eigen::Vector3d::Zero(),1000000000,false);sparse.update(Eigen::Vector3d::Zero(),1000000000,false);
  for(int i=1;i<=10;++i)dense.update(Eigen::Vector3d(.10,.02,0),1000000000+i*100000000,false);
  sparse.update(Eigen::Vector3d(.10,.02,0),2000000000,false);
  require((dense.velocity(false)-sparse.velocity(false)).norm()<1e-12,"constant-velocity lowpass follows sensor time not callback count");
  require(state.velocity(true).isZero(0),"read during frozen interval is always exactly zero");
}

#include <plan_manage/reference_progress.h>
#include <cassert>
#include <cmath>
#include <iostream>
using scan_planner::recover_connected_reference_progress;
using scan_planner::connected_free_reference_target;
int main(){
  auto stair=[](double t){return Eigen::Vector3d(0,-.4*t,.15*t);};
  const auto measured=stair(12.);
  assert((stair(10.)-measured).norm()>.65);
  const double recovered=recover_connected_reference_progress(10.,30.,.08125,measured,.65,stair);
  assert(std::abs(recovered-12.)<.082&&recovered>=10.);
  auto floors=[](double t){return t<=10?Eigen::Vector3d(.4*t,0,0):Eigen::Vector3d(8.-.4*t,0,1.5);};
  assert(recover_connected_reference_progress(0.,20.,.05,Eigen::Vector3d(2,0,1.5),.65,floors)==0.);
  const double upper=recover_connected_reference_progress(13.,20.,.05,Eigen::Vector3d(2,0,1.5),.65,floors);
  assert(std::abs(upper-15.)<.051);
  auto folded=[](double t){if(t<=1.)return Eigen::Vector3d(t,0,0);if(t<=2.)return Eigen::Vector3d(1,.1*(t-1),0);return Eigen::Vector3d(3.-t,.1,0);};
  const double same_segment=recover_connected_reference_progress(0.,2.3,.02,Eigen::Vector3d(.8,.1,0),.65,folded);
  assert(same_segment<1.&&std::abs(same_segment-.8)<.021);
  auto gap=[](double t){return Eigen::Vector3d(t<.5?t:t+3,0,0);};
  assert(recover_connected_reference_progress(0.,3.,.05,Eigen::Vector3d(3.7,0,0),.65,gap)==0.);
  auto line=[](double t){return Eigen::Vector3d(t,0,0);};
  const double nearby=connected_free_reference_target(2.,1.,20.,.05,.65,line,[](const Eigen::Vector3d&p){return p.x()>2.3;});
  assert(nearby>2.3&&nearby<2.36);
  assert(connected_free_reference_target(2.,1.,20.,.05,.65,line,[](const Eigen::Vector3d&p){return p.x()>5.;})<0.);
  assert(connected_free_reference_target(2.,1.8,20.,.05,.65,line,[](const Eigen::Vector3d&p){return p.x()<1.7;})<0.);
  assert(connected_free_reference_target(0.,0.,3.,.05,.65,gap,[](const Eigen::Vector3d&p){return p.x()>3.;})<0.);
  std::cout<<"PASS: lag recovery, connected prefix, 3D floor, no folded-route shortcut, discontinuity rejection, bounded collision target, no backward/future shortcut\n";
}

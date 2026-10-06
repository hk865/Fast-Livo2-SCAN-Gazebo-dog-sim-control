// Isolated synthetic estimator fixture: no ROS init/spin, simulator, model, or device.
#include "voxel_map.h"
#include "diagnostics.h"
#include <fstream>
#include <iostream>
int main(int argc,char **argv) {
  if(argc!=2) return 2;
  VoxelMapConfig cfg{};
  cfg.max_voxel_size_=.5; cfg.max_layer_=1; cfg.max_iterations_=5;
  cfg.layer_init_num_={5,5,5}; cfg.max_points_num_=50;
  cfg.planner_threshold_=.005;cfg.beam_err_=.05;cfg.dept_err_=.02;cfg.sigma_num_=3;
  std::unordered_map<VOXEL_LOCATION,VoxelOctoTree*> map;
  VoxelMapManager manager(cfg,map);
  manager.extR_=Eigen::Matrix3d::Identity();manager.extT_=Eigen::Vector3d::Zero();
  manager.state_.gravity=Eigen::Vector3d(0,0,-9.81);
  for(int x=-13;x<=13;++x) for(int y=-13;y<=13;++y) {
    pcl::PointXYZINormal p{};p.x=x*.06;p.y=y*.06;p.z=-.30;p.intensity=1;
    manager.feats_down_body_->push_back(p);
  }
  manager.feats_down_size_=manager.feats_down_body_->size();
  *manager.feats_down_world_=*manager.feats_down_body_; // identity build pose and extrinsics
  fastlivo_diag::Logger::instance().set_context(1,130000000000ULL);
  manager.BuildVoxelMap();
  StatesGroup predicted=manager.state_;
  predicted.pos_end.z()=.01;predicted.vel_end.z()=.03;
  predicted.cov(5,9)=predicted.cov(9,5)=.001;
  manager.state_=predicted;
  fastlivo_diag::Logger::instance().set_context(1,130100000000ULL);
  manager.StateEstimation(predicted);
  std::vector<double> values;
  fastlivo_diag::state(values,manager.state_);fastlivo_diag::append(values,manager.state_.cov);
  std::ofstream out(argv[1],std::ios::binary);
  out.write(reinterpret_cast<const char*>(values.data()),values.size()*sizeof(double));
  std::cout<<"points="<<manager.feats_down_size_<<" effective="<<manager.effct_feat_num_
           <<" state_values="<<values.size()<<" z="<<manager.state_.pos_end.z()<<" vz="<<manager.state_.vel_end.z()<<'\n';
  return out && manager.effct_feat_num_>0 ? 0:3;
}

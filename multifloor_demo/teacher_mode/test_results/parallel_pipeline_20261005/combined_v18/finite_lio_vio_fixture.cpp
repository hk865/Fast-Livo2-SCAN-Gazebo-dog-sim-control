// Isolated synthetic estimator fixture: no ROS init/spin, simulator, model, or device.
#include "voxel_map.h"
#include "diagnostics.h"
#include <fstream>
#include <iostream>
int run_lio(int argc,char **argv) {
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
           <<" query_rows="<<manager.diagnostic_query_rows_.size()<<" state_values="<<values.size()<<" z="<<manager.state_.pos_end.z()<<" vz="<<manager.state_.vel_end.z()<<'\n';
  return out && manager.effct_feat_num_>0 ? 0:3;
}

#include "vio.h"
#include <memory>
int run_vio(const char* output,bool inverse){
  StatesGroup current,predicted;
  current.gravity=predicted.gravity=Eigen::Vector3d(0,0,-9.81);
  current.inv_expo_time=predicted.inv_expo_time=1;
  current.pos_end=predicted.pos_end=Eigen::Vector3d::Zero();
  vk::PinholeCamera camera(128,128,1,100,100,64,64,0,0,0,0,0);
  cv::Mat image(128,128,CV_8UC1);
  for(int y=0;y<128;++y) for(int x=0;x<128;++x) image.at<uint8_t>(y,x)=40+x/2+y/3;
  VIOManager manager;
  manager.cam=&camera;manager.state=&current;manager.state_propagat=&predicted;
  manager.visual_submap=new SubSparseMap;
  manager.Rci=Eigen::Matrix3d::Identity();manager.Pci=Eigen::Vector3d::Zero();
  manager.Jdphi_dR=Eigen::Matrix3d::Identity();manager.Jdp_dR=Eigen::Matrix3d::Zero();
  manager.width=128;manager.height=128;manager.fx=100;manager.fy=100;manager.cx=64;manager.cy=64;
  manager.patch_size=4;manager.patch_size_half=2;manager.patch_size_total=16;
  manager.total_points=9;manager.max_iterations=3;manager.img_point_cov=1000;
  manager.exposure_estimate_en=false;manager.has_ref_patch_cache=true;
  manager.G.setZero();manager.H_T_H.setZero();
  manager.new_frame_.reset(new Frame(&camera,image));
  std::vector<std::unique_ptr<VisualPoint>> owners;
  for(int j=0;j<3;++j) for(int i=0;i<3;++i){
    Eigen::Vector3d location((i-1)*.24,(j-1)*.24,2);
    owners.emplace_back(new VisualPoint(location));
    manager.visual_submap->voxel_points.push_back(owners.back().get());
    manager.visual_submap->search_levels.push_back(0);manager.visual_submap->errors.push_back(0);
    manager.visual_submap->inv_expo_list.push_back(1);
    auto pixel=camera.world2cam(location);std::vector<float> patch;
    for(int x=0;x<4;++x) for(int y=0;y<4;++y)
      patch.push_back(image.at<uint8_t>(int(pixel[1])+x-2,int(pixel[0])+y-2)+.125f);
    manager.visual_submap->warp_patch.push_back(patch);
  }
  manager.H_sub_inv.resize(9*16,6);
  for(int r=0;r<9*16;++r) for(int c=0;c<6;++c) manager.H_sub_inv(r,c)=.01*((r+3*c)%11-5);
  fastlivo_diag::Logger::instance().set_context(2,130100000000ULL);
  // Match the OpenMP main-thread default after the existing LIO two-thread loop.
  omp_set_num_threads(2);
  if(inverse)manager.updateStateInverse(image,0);else manager.updateState(image,0);
  std::vector<double> values;
  fastlivo_diag::state(values,current);fastlivo_diag::append(values,current.cov);
  fastlivo_diag::append(values,manager.G);fastlivo_diag::append(values,manager.H_T_H);
  for(double v:values)if(!std::isfinite(v))return 8;
  std::ofstream out(output,std::ios::binary);out.write(reinterpret_cast<const char*>(values.data()),values.size()*sizeof(double));
  std::cout<<"mode="<<(inverse?"inverse":"forward")<<" points="<<manager.total_points<<" values="<<values.size()<<" pos="<<current.pos_end.transpose()<<'\n';
  return out?0:9;
}
int main(int argc,char** argv){
  if(argc!=3)return 2;
  std::string mode=argv[1];
  if(mode=="lio"){char* local[]={argv[0],argv[2]};return run_lio(2,local);}
  if(mode=="vio")return run_vio(argv[2],false);
  if(mode=="inverse")return run_vio(argv[2],true);
  return 3;
}

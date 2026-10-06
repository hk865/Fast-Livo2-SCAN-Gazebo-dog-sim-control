// Synthetic component benchmark only. No ROS init, model or simulation.
#include "voxel_map.h"
#include "diagnostics.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
using Clock=std::chrono::steady_clock;
static double elapsed(Clock::time_point a) { return std::chrono::duration<double,std::milli>(Clock::now()-a).count(); }
static void append_residuals(std::vector<double>& out,const std::vector<PointToPlane>& rows) {
  out.push_back(rows.size());
  for(const auto& r:rows) {
    fastlivo_diag::append(out,r.point_b_); fastlivo_diag::append(out,r.point_w_);
    fastlivo_diag::append(out,r.normal_); fastlivo_diag::append(out,r.center_);
    fastlivo_diag::append(out,r.plane_var_); fastlivo_diag::append(out,r.body_cov_);
    out.push_back(r.d_);out.push_back(r.dis_to_plane_);out.push_back(r.layer_);
  }
}
int main(int argc,char** argv) {
  if(argc!=4) return 2;
  const int repetitions=std::stoi(argv[3]);
  if(repetitions<30) return 3;
  const auto fixture_start=Clock::now();
  VoxelMapConfig cfg{};
  cfg.max_voxel_size_=.5;cfg.max_layer_=1;cfg.max_iterations_=5;
  cfg.layer_init_num_={5,5,5};cfg.max_points_num_=50;
  cfg.planner_threshold_=.005;cfg.beam_err_=.05;cfg.dept_err_=.02;cfg.sigma_num_=3;
  std::unordered_map<VOXEL_LOCATION,VoxelOctoTree*> map;
  VoxelMapManager manager(cfg,map);
  manager.extR_=Eigen::Matrix3d::Identity();manager.extT_=Eigen::Vector3d::Zero();
  manager.state_.gravity=Eigen::Vector3d(0,0,-9.81);
  for(int plane=0;plane<3;++plane) for(int x=-45;x<=45;++x) for(int y=-45;y<=45;++y) {
    pcl::PointXYZINormal p{};p.intensity=1;
    if(plane==0) {p.x=x*.06;p.y=y*.06;p.z=-.30;}
    if(plane==1) {p.x=3.0;p.y=x*.06;p.z=.6+y*.03;}
    if(plane==2) {p.x=x*.06;p.y=3.0;p.z=.6+y*.03;}
    manager.feats_down_body_->push_back(p);
  }
  manager.feats_down_size_=manager.feats_down_body_->size();
  *manager.feats_down_world_=*manager.feats_down_body_;
  fastlivo_diag::Logger::instance().set_context(1,130000000000ULL);
  auto start=Clock::now();manager.BuildVoxelMap();const double build_ms=elapsed(start);
  StatesGroup predicted=manager.state_;
  predicted.pos_end.z()=.01;predicted.vel_end.z()=.03;
  predicted.cov(5,9)=predicted.cov(9,5)=.001;
  auto state_once=[&](){manager.state_=predicted;fastlivo_diag::Logger::instance().set_context(2,130100000000ULL);manager.StateEstimation(predicted);};
  for(int i=0;i<5;++i)state_once();
  std::vector<double> state_times;
  for(int i=0;i<repetitions;++i){start=Clock::now();state_once();state_times.push_back(elapsed(start));}
  std::vector<double> state_values;fastlivo_diag::state(state_values,manager.state_);fastlivo_diag::append(state_values,manager.state_.cov);
  std::vector<double> canonical=state_values;append_residuals(canonical,manager.ptpl_list_);
  for(double value:canonical)if(!std::isfinite(value))return 8;
  const auto points=manager.pv_list_;
  std::vector<PointToPlane> residuals;
  std::vector<pointWithVar> input;
  for(int i=0;i<8;++i){input=points;manager.BuildResidualListOMP(input,residuals);}
  std::vector<double> residual_times;
  for(int i=0;i<repetitions;++i){input=points;start=Clock::now();manager.BuildResidualListOMP(input,residuals);residual_times.push_back(elapsed(start));}
  const double fixture_ms=elapsed(fixture_start);
  append_residuals(canonical,residuals);
  for(double value:canonical)if(!std::isfinite(value))return 8;
  std::ofstream binary(argv[1],std::ios::binary);binary.write(reinterpret_cast<const char*>(canonical.data()),canonical.size()*sizeof(double));
  std::ofstream output(argv[2]);
  output<<"{\"points\":"<<manager.feats_down_size_<<",\"accepted_residuals\":"<<manager.effct_feat_num_
    <<",\"map_root_voxels\":"<<map.size()<<",\"repetitions\":"<<repetitions
    <<",\"state_warmup\":5,\"residual_warmup\":8,\"canonical_values\":"<<canonical.size()
    <<",\"canonical_state_cov_values\":"<<state_values.size()<<",\"query_rows\":"<<manager.diagnostic_query_rows_.size()
    <<",\"build_map_ms\":"<<build_ms<<",\"fixture_without_output_io_ms\":"<<fixture_ms<<",\"StateEstimation_ms\":[";
  for(size_t i=0;i<state_times.size();++i)output<<(i?",":"")<<state_times[i];
  output<<"],\"BuildResidualListOMP_ms\":[";
  for(size_t i=0;i<residual_times.size();++i)output<<(i?",":"")<<residual_times[i];
  output<<"]}\n";
  std::cout<<"points="<<manager.feats_down_size_<<" residuals="<<manager.effct_feat_num_<<" values="<<canonical.size()<<" fixture_ms="<<fixture_ms<<"\n";
  return binary&&output&&manager.effct_feat_num_>0?0:9;
}

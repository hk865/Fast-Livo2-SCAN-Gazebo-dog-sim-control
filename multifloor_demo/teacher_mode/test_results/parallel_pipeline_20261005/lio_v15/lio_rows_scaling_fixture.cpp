// Synthetic component benchmark only. No ROS init, model or simulation.
#include "voxel_map.h"
#include "diagnostics.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <time.h>
static double cpu_ms(clockid_t id){timespec t{};clock_gettime(id,&t);return double(t.tv_sec)*1000+double(t.tv_nsec)*1e-6;}
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
// Baseline reference lifted byte-for-byte from V12 StateEstimation.
static void baseline_rows(VoxelMapManager &manager,StatesGroup &state_propagat,Eigen::MatrixXd &Hsub,
  Eigen::MatrixXd &Hsub_T_R_inv,Eigen::VectorXd &R_inv,Eigen::VectorXd &meas_vec, bool diagnostic_detail,
  std::vector<double>& diagnostic_solver_sigma,std::vector<M3D>& diagnostic_solver_body_world) {
  using namespace Eigen;
  auto& ptpl_list_=manager.ptpl_list_;auto& state_=manager.state_;auto& extR_=manager.extR_;auto& extT_=manager.extT_;
  const int effct_feat_num_=manager.effct_feat_num_;
    for (int i = 0; i < effct_feat_num_; i++)
    {
      auto &ptpl = ptpl_list_[i];
      V3D point_this(ptpl.point_b_);
      point_this = extR_ * point_this + extT_;
      V3D point_body(ptpl.point_b_);
      M3D point_crossmat;
      point_crossmat << SKEW_SYM_MATRX(point_this);

      /*** get the normal vector of closest surface/corner ***/

      V3D point_world = state_propagat.rot_end * point_this + state_propagat.pos_end;
      Eigen::Matrix<double, 1, 6> J_nq;
      J_nq.block<1, 3>(0, 0) = point_world - ptpl_list_[i].center_;
      J_nq.block<1, 3>(0, 3) = -ptpl_list_[i].normal_;

      M3D var;
      // V3D normal_b = state_.rot_end.inverse() * ptpl_list_[i].normal_;
      // V3D point_b = ptpl_list_[i].point_b_;
      // double cos_theta = fabs(normal_b.dot(point_b) / point_b.norm());
      // ptpl_list_[i].body_cov_ = ptpl_list_[i].body_cov_ * (1.0 / cos_theta) * (1.0 / cos_theta);

      // point_w cov
      // var = state_propagat.rot_end * extR_ * ptpl_list_[i].body_cov_ * (state_propagat.rot_end * extR_).transpose() +
      //       state_propagat.cov.block<3, 3>(3, 3) + (-point_crossmat) * state_propagat.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose();

      // point_w cov (another_version)
      // var = state_propagat.rot_end * extR_ * ptpl_list_[i].body_cov_ * (state_propagat.rot_end * extR_).transpose() +
      //       state_propagat.cov.block<3, 3>(3, 3) - point_crossmat * state_propagat.cov.block<3, 3>(0, 0) * point_crossmat;

      // point_body cov
      var = state_propagat.rot_end * extR_ * ptpl_list_[i].body_cov_ * (state_propagat.rot_end * extR_).transpose();

      double sigma_l = J_nq * ptpl_list_[i].plane_var_ * J_nq.transpose();
      if(diagnostic_detail) { diagnostic_solver_sigma[i]=sigma_l; diagnostic_solver_body_world[i]=var; }

      R_inv(i) = 1.0 / (0.001 + sigma_l + ptpl_list_[i].normal_.transpose() * var * ptpl_list_[i].normal_);
      // R_inv(i) = 1.0 / (sigma_l + ptpl_list_[i].normal_.transpose() * var * ptpl_list_[i].normal_);

      /*** calculate the Measuremnt Jacobian matrix H ***/
      V3D A(point_crossmat * state_.rot_end.transpose() * ptpl_list_[i].normal_);
      Hsub.row(i) << VEC_FROM_ARRAY(A), ptpl_list_[i].normal_[0], ptpl_list_[i].normal_[1], ptpl_list_[i].normal_[2];
      Hsub_T_R_inv.col(i) << A[0] * R_inv(i), A[1] * R_inv(i), A[2] * R_inv(i), ptpl_list_[i].normal_[0] * R_inv(i),
          ptpl_list_[i].normal_[1] * R_inv(i), ptpl_list_[i].normal_[2] * R_inv(i);
      meas_vec(i) = -ptpl_list_[i].dis_to_plane_;
    }
}
int main(int argc,char** argv) {
  if(argc!=5) return 2;
  const int half_extent=std::stoi(argv[4]);
  if(half_extent<3 || half_extent>45)return 3;
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
  for(int plane=0;plane<3;++plane) for(int x=-half_extent;x<=half_extent;++x) for(int y=-half_extent;y<=half_extent;++y) {
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
  for(int i=0;i<10;++i)state_once();
  std::vector<double> state_times;
  for(int i=0;i<repetitions;++i){start=Clock::now();state_once();state_times.push_back(elapsed(start));}
  std::vector<double> state_values;fastlivo_diag::state(state_values,manager.state_);fastlivo_diag::append(state_values,manager.state_.cov);
  std::vector<double> canonical=state_values;append_residuals(canonical,manager.ptpl_list_);
  for(double value:canonical)if(!std::isfinite(value))return 8;
  const auto points=manager.pv_list_;
  std::vector<PointToPlane> residuals;
  std::vector<pointWithVar> input;
  for(int i=0;i<10;++i){input=points;manager.BuildResidualListOMP(input,residuals);}
  std::vector<double> residual_times;
  for(int i=0;i<repetitions;++i){input=points;start=Clock::now();manager.BuildResidualListOMP(input,residuals);residual_times.push_back(elapsed(start));}
  // Isolated row generation on the identical final production constraints.
  const int n=manager.effct_feat_num_;
  Eigen::MatrixXd Hsub(n,6),Hsub_T_R_inv(6,n);Eigen::VectorXd R_inv(n),meas_vec(n);
  std::vector<double> diagnostic_solver_sigma(n);std::vector<M3D> diagnostic_solver_body_world(n);
  auto rows_once=[&](bool detail) {
#ifdef V15_BUILD
    manager.BuildJacobianRows(predicted,Hsub,Hsub_T_R_inv,R_inv,meas_vec,detail,diagnostic_solver_sigma,diagnostic_solver_body_world);
#else
    baseline_rows(manager,predicted,Hsub,Hsub_T_R_inv,R_inv,meas_vec,detail,diagnostic_solver_sigma,diagnostic_solver_body_world);
#endif
  };
  for(int i=0;i<10;++i)rows_once(false);
  std::vector<double> row_times,row_process_cpu_ms,row_thread_cpu_ms;
  for(int i=0;i<repetitions;++i){const double pc=cpu_ms(CLOCK_PROCESS_CPUTIME_ID),tc=cpu_ms(CLOCK_THREAD_CPUTIME_ID);start=Clock::now();rows_once(false);row_times.push_back(elapsed(start));row_thread_cpu_ms.push_back(cpu_ms(CLOCK_THREAD_CPUTIME_ID)-tc);row_process_cpu_ms.push_back(cpu_ms(CLOCK_PROCESS_CPUTIME_ID)-pc);}
  rows_once(true);
  fastlivo_diag::append(canonical,Hsub);fastlivo_diag::append(canonical,Hsub_T_R_inv);
  fastlivo_diag::append(canonical,R_inv);fastlivo_diag::append(canonical,meas_vec);
  for(double v:diagnostic_solver_sigma)canonical.push_back(v);
  for(const auto& v:diagnostic_solver_body_world)fastlivo_diag::append(canonical,v);
  Eigen::Matrix<double,DIM_STATE,DIM_STATE> HTH;HTH.setZero();HTH.block<6,6>(0,0)=Hsub_T_R_inv*Hsub;auto&& HTz=Hsub_T_R_inv*meas_vec;
  fastlivo_diag::append(canonical,HTH.block<6,6>(0,0));fastlivo_diag::append(canonical,HTz);
  const double fixture_ms=elapsed(fixture_start);
  append_residuals(canonical,residuals);
  for(double value:canonical)if(!std::isfinite(value))return 8;
  std::ofstream binary(argv[1],std::ios::binary);binary.write(reinterpret_cast<const char*>(canonical.data()),canonical.size()*sizeof(double));
  std::ofstream output(argv[2]);
  output<<"{\"points\":"<<manager.feats_down_size_<<",\"accepted_residuals\":"<<manager.effct_feat_num_
    <<",\"map_root_voxels\":"<<manager.voxel_map_.size()<<",\"repetitions\":"<<repetitions
    <<",\"state_warmup\":10,\"residual_warmup\":10,\"canonical_values\":"<<canonical.size()
    <<",\"canonical_state_cov_values\":"<<state_values.size()<<",\"query_rows\":"<<manager.diagnostic_query_rows_.size()
    <<",\"build_map_ms\":"<<build_ms<<",\"fixture_without_output_io_ms\":"<<fixture_ms<<",\"StateEstimation_ms\":[";
  for(size_t i=0;i<state_times.size();++i)output<<(i?",":"")<<state_times[i];
  output<<"],\"BuildResidualListOMP_ms\":[";
  for(size_t i=0;i<residual_times.size();++i)output<<(i?",":"")<<residual_times[i];
  output<<"],\"JacobianRows_ms\":[";
  for(size_t i=0;i<row_times.size();++i)output<<(i?",":"")<<row_times[i];
  output<<"],\"JacobianRows_process_cpu_ms\":[";
  for(size_t i=0;i<row_process_cpu_ms.size();++i)output<<(i?",":"")<<row_process_cpu_ms[i];
  output<<"],\"JacobianRows_thread_cpu_ms\":[";
  for(size_t i=0;i<row_thread_cpu_ms.size();++i)output<<(i?",":"")<<row_thread_cpu_ms[i];
  output<<"],\"jacobian_warmup\":10,\"rows_diagnostic_in_timing\":false}\n";
  std::cout<<"points="<<manager.feats_down_size_<<" residuals="<<manager.effct_feat_num_<<" values="<<canonical.size()<<" fixture_ms="<<fixture_ms<<"\n";
  return binary&&output&&manager.effct_feat_num_>0?0:9;
}

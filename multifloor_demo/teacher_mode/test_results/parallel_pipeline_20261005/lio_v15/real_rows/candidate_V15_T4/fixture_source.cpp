#define V15_BUILD 1
// Actual recorded ptpl component replay, not full estimator replay.
#include "voxel_map.h"
#include "diagnostics.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <cstring>
#include <time.h>
using Clock=std::chrono::steady_clock;
static double elapsed(Clock::time_point a){return std::chrono::duration<double,std::milli>(Clock::now()-a).count();}
static double cpu_ms(clockid_t id){timespec t{};clock_gettime(id,&t);return double(t.tv_sec)*1000+double(t.tv_nsec)*1e-6;}
template<class Mat>void read_matrix(std::ifstream& f,Mat& out){for(int i=0;i<out.rows();++i)for(int j=0;j<out.cols();++j)f.read(reinterpret_cast<char*>(&out(i,j)),8);}
static void read_state(std::ifstream& f,StatesGroup& s){read_matrix(f,s.rot_end);read_matrix(f,s.pos_end);f.read(reinterpret_cast<char*>(&s.inv_expo_time),8);read_matrix(f,s.vel_end);read_matrix(f,s.bias_g);read_matrix(f,s.bias_a);read_matrix(f,s.gravity);}
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

int main(int argc,char**argv){
 if(argc!=5)return 2;const int repetitions=std::stoi(argv[4]);if(repetitions<1)return 3;
 std::ifstream in(argv[1],std::ios::binary);char magic[16];in.read(magic,16);if(std::memcmp(magic,"FLIVOJACROW0001\0",16))return 4;
 uint64_t n;in.read(reinterpret_cast<char*>(&n),8);if(n<1||n>100000)return 5;
 VoxelMapConfig config{};std::unordered_map<VOXEL_LOCATION,VoxelOctoTree*> map;VoxelMapManager manager(config,map);
 read_matrix(in,manager.extR_);read_matrix(in,manager.extT_);StatesGroup predicted;read_state(in,predicted);read_state(in,manager.state_);read_matrix(in,manager.state_.cov);
 manager.effct_feat_num_=n;manager.ptpl_list_.resize(n);
 for(uint64_t i=0;i<n;++i){double row[107];in.read(reinterpret_cast<char*>(row),sizeof(row));auto& p=manager.ptpl_list_[i];p.point_b_=Eigen::Map<Eigen::Vector3d>(row+21);p.normal_=Eigen::Map<Eigen::Vector3d>(row+27);p.center_=Eigen::Map<Eigen::Vector3d>(row+30);p.dis_to_plane_=static_cast<float>(row[34]);for(int r=0;r<6;++r)for(int c=0;c<6;++c)p.plane_var_(r,c)=row[44+r*6+c];for(int r=0;r<3;++r)for(int c=0;c<3;++c)p.body_cov_(r,c)=row[80+r*3+c];}
 if(!in)return 6;
 Eigen::MatrixXd Hsub(n,6),Hsub_T_R_inv(6,n);Eigen::VectorXd R_inv(n),meas_vec(n);std::vector<double> diagnostic_solver_sigma(n);std::vector<M3D> diagnostic_solver_body_world(n);
 auto once=[&](bool detail){
#ifdef V15_BUILD
 manager.BuildJacobianRows(predicted,Hsub,Hsub_T_R_inv,R_inv,meas_vec,detail,diagnostic_solver_sigma,diagnostic_solver_body_world);
#else
 baseline_rows(manager,predicted,Hsub,Hsub_T_R_inv,R_inv,meas_vec,detail,diagnostic_solver_sigma,diagnostic_solver_body_world);
#endif
 };
 const int warmup=repetitions>=30?10:0;for(int i=0;i<warmup;++i)once(false);
 std::vector<double> wall,process,thread;
 for(int i=0;i<repetitions;++i){const double pc=cpu_ms(CLOCK_PROCESS_CPUTIME_ID),tc=cpu_ms(CLOCK_THREAD_CPUTIME_ID);auto begin=Clock::now();once(false);wall.push_back(elapsed(begin));thread.push_back(cpu_ms(CLOCK_THREAD_CPUTIME_ID)-tc);process.push_back(cpu_ms(CLOCK_PROCESS_CPUTIME_ID)-pc);}
 once(true);
 std::vector<double> canonical;fastlivo_diag::append(canonical,Hsub);fastlivo_diag::append(canonical,Hsub_T_R_inv);fastlivo_diag::append(canonical,R_inv);fastlivo_diag::append(canonical,meas_vec);for(double v:diagnostic_solver_sigma)canonical.push_back(v);for(auto& v:diagnostic_solver_body_world)fastlivo_diag::append(canonical,v);Eigen::Matrix<double,DIM_STATE,DIM_STATE> HTH;HTH.setZero();HTH.block<6,6>(0,0)=Hsub_T_R_inv*Hsub;auto&& HTz=Hsub_T_R_inv*meas_vec;fastlivo_diag::append(canonical,HTH.block<6,6>(0,0));fastlivo_diag::append(canonical,HTz);
 for(double v:canonical)if(!std::isfinite(v))return 8;
 std::ofstream out(argv[2],std::ios::binary);out.write(reinterpret_cast<char*>(canonical.data()),canonical.size()*8);
 std::ofstream report(argv[3]);report<<"{\"rows\":"<<n<<",\"warmup\":"<<warmup<<",\"repetitions\":"<<repetitions<<",\"canonical_values\":"<<canonical.size()<<",\"wall_ms\":[";for(size_t i=0;i<wall.size();++i)report<<(i?",":"")<<wall[i];report<<"],\"process_cpu_ms\":[";for(size_t i=0;i<process.size();++i)report<<(i?",":"")<<process[i];report<<"],\"thread_cpu_ms\":[";for(size_t i=0;i<thread.size();++i)report<<(i?",":"")<<thread[i];report<<"]}\n";
 return out&&report?0:9;
}

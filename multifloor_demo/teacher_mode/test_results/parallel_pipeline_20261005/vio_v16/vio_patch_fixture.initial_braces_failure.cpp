// Synthetic VIO update boundary fixture; no ROS init/spin, simulator or device.
#include "vio.h"
#include "feature.h"
#include "diagnostics.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <iomanip>
#include <cmath>
struct Fixture {
  StatesGroup current,predicted,seed;
  vk::PinholeCamera camera{512,512,1,180,180,256,256,0,0,0,0,0};
  cv::Mat image{512,512,CV_8UC1};
  VIOManager manager;
  std::vector<std::unique_ptr<VisualPoint>> owners;
  bool inverse,nullable,mixed,exposure,fresh,levels,wrong_jacobian,all_null;
  int count,level;
  Fixture(int n,bool inv,int lev,bool nul,bool mix,bool expo,bool cache,bool multi,bool wrong,bool empty)
  :inverse(inv),nullable(nul),mixed(mix),exposure(expo),fresh(cache),levels(multi),wrong_jacobian(wrong),all_null(empty),count(n),level(lev){
    current.gravity=predicted.gravity=Eigen::Vector3d(0,0,-9.81);
    current.inv_expo_time=predicted.inv_expo_time=expo?.97:1.;
    current.pos_end=predicted.pos_end=Eigen::Vector3d(.003,-.002,.002);
    current.cov(5,9)=current.cov(9,5)=.001;predicted.cov=current.cov;seed=current;
    for(int y=0;y<512;++y)for(int x=0;x<512;++x)
      image.at<uint8_t>(y,x)=static_cast<uint8_t>(80+(.17*x+.12*y)+7*std::sin(x*.11)+6*std::cos(y*.09));
    manager.cam=&camera;manager.state=&current;manager.state_propagat=&predicted;
    manager.visual_submap=new SubSparseMap;
    manager.Rci=Eigen::Matrix3d::Identity();manager.Pci=Eigen::Vector3d::Zero();
    manager.Jdphi_dR=Eigen::Matrix3d::Identity();manager.Jdp_dR=Eigen::Matrix3d::Zero();
    manager.width=512;manager.height=512;manager.fx=180;manager.fy=180;manager.cx=256;manager.cy=256;
    manager.patch_size=8;manager.patch_size_half=4;manager.patch_size_total=64;manager.patch_pyrimid_level=3;
    manager.total_points=n;manager.max_iterations=4;manager.img_point_cov=1000;
    manager.exposure_estimate_en=expo;manager.has_ref_patch_cache=!cache;manager.inverse_composition_en=inv;
    manager.G.setZero();manager.H_T_H.setZero();manager.new_frame_.reset(new Frame(&camera,image));
    const int side=static_cast<int>(std::ceil(std::sqrt(std::max(1,n))));
    for(int i=0;i<n;++i){
      const double x=((i%side+.5)/side-.5)*2.2,y=((i/side+.5)/side-.5)*2.2;
      Eigen::Vector3d location(x,y,2.+.03*(i%7));
      owners.emplace_back(new VisualPoint(location));VisualPoint* point=owners.back().get();
      const auto pixel=camera.world2cam(location);const int search=mix?(i%2):0;
      std::vector<float> patch;
      for(int pyramid=0;pyramid<3;++pyramid){
        const int scale=1<<(pyramid+search),ui=static_cast<int>(std::floor(pixel[0]/scale))*scale,vi=static_cast<int>(std::floor(pixel[1]/scale))*scale;
        const float su=(pixel[0]-ui)/scale,sv=(pixel[1]-vi)/scale;
        const float tl=(1.-su)*(1.-sv),tr=su*(1.-sv),bl=(1.-su)*sv,br=su*sv;
        for(int px=0;px<8;++px)for(int py=0;py<8;++py){
          const int ix=ui+(py-4)*scale,iy=vi+(px-4)*scale;
          float v=tl*image.at<uint8_t>(iy,ix)+tr*image.at<uint8_t>(iy,ix+scale)+bl*image.at<uint8_t>(iy+scale,ix)+br*image.at<uint8_t>(iy+scale,ix+scale);
          patch.push_back(v+.125f+.001f*(i%5));
        }
      }
      auto feature=new Feature(point,new float[192]{},pixel,location.normalized(),Sophus::SE3d(),0);
      feature->img_=image;feature->inv_expo_time_=expo?1.03:1.;point->ref_patch=feature;point->addFrameRef(feature);point->has_ref_patch_=true;
      manager.visual_submap->voxel_points.push_back((all_null||(nullable&&i%7==0))?nullptr:point);
      manager.visual_submap->search_levels.push_back(search);manager.visual_submap->errors.push_back(.333f);
      manager.visual_submap->inv_expo_list.push_back(expo?1.03:1.);manager.visual_submap->warp_patch.push_back(patch);
    }
    manager.H_sub_inv.resize(n*64,6);
    for(int r=0;r<n*64;++r)for(int c=0;c<6;++c)manager.H_sub_inv(r,c)=(wrong_jacobian?-.2:.01)*((r+3*c)%11-5);
  }
  void reset(){current=seed;predicted=seed;manager.G.setZero();manager.H_T_H.setZero();manager.has_ref_patch_cache=!fresh;std::fill(manager.visual_submap->errors.begin(),manager.visual_submap->errors.end(),.333f);}
  void run(){
    if(levels)manager.computeJacobianAndUpdateEKF(image);
    else if(inverse)manager.updateStateInverse(image,level);
    else manager.updateState(image,level);
  }
  bool save(const char*path){
    std::vector<double> values;fastlivo_diag::state(values,current);fastlivo_diag::append(values,current.cov);
    fastlivo_diag::append(values,manager.G);fastlivo_diag::append(values,manager.H_T_H);fastlivo_diag::append(values,manager.H_sub_inv);
    for(float f:manager.visual_submap->errors)values.push_back(f);
    for(double v:values)if(!std::isfinite(v))return false;
    std::ofstream out(path,std::ios::binary);out.write(reinterpret_cast<const char*>(values.data()),values.size()*sizeof(double));return bool(out);
  }
};
int main(int argc,char**argv){
  // mode points level null mixed exposure fresh_cache multi_levels wrong_jacobian all_null repetitions output
  if(argc!=13)return 2;
  const bool inverse=std::string(argv[1])=="inverse";const int points=std::stoi(argv[2]),level=std::stoi(argv[3]),reps=std::stoi(argv[11]);
  Fixture fixture(points,inverse,level,std::stoi(argv[4]),std::stoi(argv[5]),std::stoi(argv[6]),std::stoi(argv[7]),std::stoi(argv[8]),std::stoi(argv[9]),std::stoi(argv[10]));
  // Match the main-thread ICV left by the unchanged LIO4 loop, without Eigen blanket changes.
  omp_set_num_threads(4);omp_set_dynamic(0);fastlivo_diag::Logger::instance().set_context(2,130100000000ULL);
  std::vector<double> milliseconds;const int warmups=reps>1?8:0;
  for(int k=-warmups;k<reps;++k){fixture.reset();auto start=std::chrono::steady_clock::now();fixture.run();auto end=std::chrono::steady_clock::now();if(k>=0)milliseconds.push_back(std::chrono::duration<double,std::milli>(end-start).count());}
  if(!fixture.save(argv[12]))return 8;
  std::cout<<std::setprecision(17)<<"{\"mode\":\""<<(inverse?"inverse":"forward")<<"\",\"points\":"<<points<<",\"level\":"<<level<<",\"null\":"<<argv[4]<<",\"mixed\":"<<argv[5]<<",\"exposure\":"<<argv[6]<<",\"fresh\":"<<argv[7]<<",\"multi\":"<<argv[8]<<",\"wrong_jacobian\":"<<argv[9]<<",\"all_null\":"<<argv[10]<<",\"warmups\":"<<warmups<<",\"repetitions\":"<<reps<<",\"milliseconds\":[";
  for(size_t i=0;i<milliseconds.size();++i)std::cout<<(i?",":"")<<milliseconds[i];std::cout<<"]}"<<std::endl;return 0;
}

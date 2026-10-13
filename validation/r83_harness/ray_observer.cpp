// Run-local point direction correction and observation. Every original call is forwarded once.
// No render/readback/subscription/noise/GT navigation is added.
#include <gz/sensors/GpuLidarSensor.hh>
#include <gz/sim/rendering/RenderUtil.hh>
#include <gz/rendering/ogre2/Ogre2GpuRays.hh>
#include <gz/rendering/ogre2/Ogre2Scene.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/msgs/PointCloudPackedUtils.hh>
#include <OgreTextureGpu.h>
#include <OgreTextureBox.h>
#include <OgrePass.h>
#include <OgreTextureUnitState.h>
#include <OgreHlmsSamplerblock.h>
#include <OgreGpuProgramParams.h>
#include <Compositor/Pass/PassQuad/OgreCompositorPassQuad.h>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fstream>
#include <iomanip>
#include <string>
#include <thread>

namespace diag {
using Sensor=gz::sensors::GpuLidarSensor;
using Lidar=gz::sensors::Lidar;
using Util=gz::sim::RenderUtil;
using Rays=gz::rendering::Ogre2GpuRays;
using Scene=gz::rendering::Ogre2Scene;
using Duration=std::chrono::steady_clock::duration;
using Publisher=gz::transport::Node::Publisher;
constexpr size_t pixels=480*64, uvbytes=pixels*16, cloudcap=1024*1024;
constexpr const char *ogre_plugin="/opt/ros/jazzy/opt/gz_rendering_vendor/lib/gz-rendering-8/engine-plugins/libgz-rendering8-ogre2.so";
bool enabled=false;
bool rectify=false,calibration_loaded=false;
std::array<float,pixels*3> calibrated_direction{};
std::array<unsigned char,uvbytes> calibrated_uv{};
std::atomic<uint64_t> rectified_frames{0},rectified_points{0},original_noise_calls{0},original_cloud_calls{0},original_cloud_success{0};
std::atomic<uint64_t> updates_without_pointcloud{0},geometry_checked_frames{0};
thread_local Sensor *current_sensor=nullptr;
thread_local uint64_t current_source_ns=0;
thread_local const float *current_ranges=nullptr;
thread_local uint32_t current_noise_calls=0,current_publish_calls=0,current_face_units=0;
thread_local bool current_uv_sampler=false;
std::atomic<bool> done{false};
std::atomic<uint32_t> file_errors{0}, foreign_drops{0};
thread_local uint8_t thread_token;
std::atomic<uintptr_t> producer{0};
uint64_t wall() { return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
int rectify_xyz(unsigned char *data,size_t n,uint32_t step,const uint32_t *offset,const float *ranges){
  if(!data||!ranges||!calibration_loaded||n!=pixels||step!=32)return -1;
  for(unsigned k=0;k<3;++k)if(offset[k]!=4*k)return -1;
  int count=0;
  for(size_t i=0;i<n;++i){
    float xyz[3];for(unsigned k=0;k<3;++k)std::memcpy(&xyz[k],data+i*step+offset[k],4);
    const float range=ranges[i*3];
    if(!std::isfinite(range)||range<=0||!std::isfinite(xyz[0])||!std::isfinite(xyz[1])||!std::isfinite(xyz[2]))continue;
    for(unsigned k=0;k<3;++k){const float value=range*calibrated_direction[i*3+k];std::memcpy(data+i*step+offset[k],&value,4);}++count;
  }return count;
}
const char *packed_layout(const gz::msgs::PointCloudPacked &cloud,uint32_t *offset){
  if(cloud.width()!=480||cloud.height()!=64||cloud.point_step()!=32||cloud.row_step()!=480*32||cloud.data().size()!=pixels*32||cloud.is_bigendian())return "complete packed layout";
  uint32_t seen[3]{};for(unsigned k=0;k<3;++k)offset[k]=UINT32_MAX;
  for(const auto &field:cloud.field())for(unsigned k=0;k<3;++k)if(field.name()==std::string(1,"xyz"[k])){
    if(++seen[k]!=1||field.datatype()!=gz::msgs::PointCloudPacked::Field::FLOAT32||field.count()!=1||field.offset()!=4*k)return "XYZ GZ datatype/duplicate/count/offset";
    offset[k]=field.offset();
  }
  for(unsigned k=0;k<3;++k)if(seen[k]!=1)return "XYZ field missing";
  return nullptr;
}
[[noreturn]] void geometry_fail(const char *reason){std::fprintf(stderr,"GO2_R50_GEOMETRY_BINDING_FAILED %s\n",reason);std::abort();}
void finish_update(bool result){
  if(rectify && result && (current_noise_calls!=1||current_publish_calls>1))geometry_fail("successful Update noise/publication count");
  // The original point publisher may have no subscriber during startup.
  // Never invent a publication or connection; every actual cloud is corrected.
  if(rectify && result && current_publish_calls==0)++updates_without_pointcloud;
}
void number(std::ostream &stream,double value){
  if(std::isfinite(value))stream<<value;
  else stream<<std::quoted(std::isnan(value)?"NaN":value<0?"-Infinity":"+Infinity");
}
template<class F> F next(const char *name) {
  void *p=dlsym(RTLD_NEXT,name);
  if(!p) { std::fprintf(stderr,"GO2_R50_MISSING_ORIGINAL %s\n",name);std::abort(); }
  return reinterpret_cast<F>(p);
}
template<class F> F plugin_next(const char *name) {
  // The installed plugin loader uses RTLD_LOCAL. NOLOAD never loads a backend.
  void *handle=dlopen(ogre_plugin,RTLD_NOLOAD|RTLD_LAZY);
  void *p=handle?dlsym(handle,name):nullptr; Dl_info origin{};
  const std::string module=(p&&dladdr(p,&origin)&&origin.dli_fname)?origin.dli_fname:"";
  const std::string prefix="/opt/ros/jazzy/opt/gz_rendering_vendor/lib/gz-rendering-8/engine-plugins/";
  const bool same_frozen_module=(module==prefix+"libgz-rendering8-ogre2.so" || module==prefix+"libgz-rendering8-ogre2.so.8" || module==prefix+"libgz-rendering8-ogre2.so.8.2.3" || module==prefix+"libgz-rendering-ogre2.so");
  if(!p || !same_frozen_module) {
    std::fprintf(stderr,"GO2_R50_ORIGINAL_PLUGIN_UNVERIFIED %s\n",name);std::abort();
  }
  return reinterpret_cast<F>(p);
}
bool own_thread() {
  const auto me=reinterpret_cast<uintptr_t>(&thread_token);uintptr_t empty=0;
  producer.compare_exchange_strong(empty,me);
  if(producer.load()!=me) { ++foreign_drops;return false; }return true;
}
template<size_t N> void text(char (&dst)[N],const std::string &src) { std::snprintf(dst,N,"%s",src.c_str()); }
struct Texture { uint64_t address=0,rays=0;uint32_t width=0,height=0,depth=0,format=0;char name[160]{}; };
struct Sampler {
  uint64_t quad=0,pass=0,texture=0,camera=0,parameters=0;
  uint32_t unit=0,width=0,height=0,uniform_mask=0;int filter[6]{};char name[160]{};
  double camera_pose[7]{},camera_frustum[4]{};float uniform[10]{};
};
struct Config {
  uint64_t rays=0,texture=0,wall_ns=0;uint32_t w=0,h=0,format=0;
  uint64_t row_bytes=0,image_bytes=0;uint32_t count=0;Texture textures[32]{};
  std::array<unsigned char,uvbytes> uv{};
};
Config config;std::atomic<uint32_t> config_state{0}; // 0 collecting, 1 sealed, 2 written
thread_local Rays *sample_owner=nullptr,*setup_owner=nullptr;
struct Batch { Util *util=nullptr;uint64_t generation=0,step=0,ns=0; };
Batch batches[8]; // accessed only at verified callers while original outer mutex is held
thread_local Batch consumed{};
thread_local gz::rendering::Scene *consumed_scene=nullptr,*applied_scene=nullptr;
thread_local uint64_t applied_generation=0;
// Verified caller return offsets are filled before freezing this source.
constexpr uintptr_t ecm_return=0x29dbd, util_return=0x225d3;
bool known_caller(void *return_address,uintptr_t offset) {
  Dl_info info{};
  if(!dladdr(return_address,&info)||!info.dli_fname||!info.dli_fbase)return false;
  const char *base="/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/libgz-sim8-sensors-system.so";
  const std::string module(info.dli_fname);
  return (module==base || module==std::string(base)+".8" || module==std::string(base)+".8.11.0" || module=="/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/libgz-sim-sensors-system.so") &&
      reinterpret_cast<uintptr_t>(return_address)-reinterpret_cast<uintptr_t>(info.dli_fbase)==offset;
}
Batch *batch(Util *u,bool create) {
  for(auto &b:batches)if(b.util==u)return &b;
  if(create)for(auto &b:batches)if(!b.util){b.util=u;return &b;}
  return nullptr;
}
struct Frame {
  std::atomic<uint32_t> state{0};
  uint64_t source=0,publish=0,begin_wall=0,end_wall=0,readback_wall=0,noise_wall=0,pub_wall=0;
  uint64_t sensor=0,rays=0,util=0,scene=0,ray_scene=0,generation=0,step=0,physics_ns=0,applied=0;
  uint32_t stages=0,w=0,h=0,channels=0,callbacks=0,ok=0,pub_ok=0;
  uint32_t point_step=0,row_step=0,cloud_w=0,cloud_h=0,cloud_bytes=0,bigendian=0;
  uint32_t xyz[3]{UINT32_MAX,UINT32_MAX,UINT32_MAX};
  double pose[7]{},pose_delta=0;
  uint32_t sampler_count=0,sampler_drops=0;Sampler samplers[64]{};
  std::array<float,pixels*3> pre{},post{};
  std::array<unsigned char,cloudcap> cloud{};
  std::array<unsigned char,cloudcap> rectified_cloud{};
};
Frame frames[3];thread_local Frame *active=nullptr;
uint32_t selected=0;
using UpdateFn=bool(*)(Sensor*,const Duration&);
using FrameFn=void(*)(Sensor*,const float*,unsigned,unsigned,unsigned,const std::string&);
using NoiseFn=void(*)(Lidar*);
using PubFn=bool(*)(Publisher*,const google::protobuf::Message&);
using EcmFn=void(*)(Util*,const gz::sim::UpdateInfo&,const gz::sim::EntityComponentManager&);
using UtilFn=void(*)(Util*);
UpdateFn update_fn;FrameFn frame_fn;NoiseFn noise_fn;PubFn pub_fn;EcmFn ecm_fn;UtilFn util_fn;
void pose(Sensor *s,double *v) {
  const auto rays=s->GpuRays();if(!rays)return;
  const auto p=rays->WorldPose();v[0]=p.Pos().X();v[1]=p.Pos().Y();v[2]=p.Pos().Z();
  v[3]=p.Rot().W();v[4]=p.Rot().X();v[5]=p.Rot().Y();v[6]=p.Rot().Z();
}
void binary(const std::string &p,const void *data,size_t n) {
  std::ofstream f(p,std::ios::binary);f.write(static_cast<const char*>(data),n);f.close();if(!f)++file_errors;
}
void save_frame(const std::string &dir,Frame &f,unsigned index) {
  const auto prefix=dir+"/frame_"+std::to_string(index);
  binary(prefix+"_pre.f32",f.pre.data(),sizeof(f.pre));
  binary(prefix+"_post.f32",f.post.data(),sizeof(f.post));
  binary(prefix+"_cloud.bin",f.cloud.data(),f.cloud_bytes);
  if(rectify)binary(prefix+"_rectified_cloud.bin",f.rectified_cloud.data(),f.cloud_bytes);
  std::ofstream j(prefix+".json");j<<std::setprecision(17);
  j<<"{\"schema\":\"R50_actual_ray_frame/v1\",\"source_ns\":"<<f.source<<",\"publication_ns\":"<<f.publish
   <<",\"wall_ns\":["<<f.begin_wall<<","<<f.readback_wall<<","<<f.noise_wall<<","<<f.pub_wall<<","<<f.end_wall<<"]"
   <<",\"sensor\":"<<f.sensor<<",\"rays\":"<<f.rays<<",\"util\":"<<f.util<<",\"scene\":"<<f.scene<<",\"ray_scene\":"<<f.ray_scene
   <<",\"generation\":"<<f.generation<<",\"physical_step\":"<<f.step<<",\"physics_ns\":"<<f.physics_ns<<",\"applied_generation\":"<<f.applied
   <<",\"stages\":"<<f.stages<<",\"width\":"<<f.w<<",\"height\":"<<f.h<<",\"channels\":"<<f.channels<<",\"callbacks\":"<<f.callbacks
   <<",\"update_ok\":"<<f.ok<<",\"publication_ok\":"<<f.pub_ok<<",\"readback_pose_max_delta\":"<<f.pose_delta
   <<",\"diagnostic_world_sensor_pose\":[";
  for(unsigned i=0;i<7;++i){if(i)j<<",";j<<f.pose[i];}
  j<<"],\"cloud\":{\"width\":"<<f.cloud_w<<",\"height\":"<<f.cloud_h<<",\"point_step\":"<<f.point_step<<",\"row_step\":"<<f.row_step
   <<",\"bytes\":"<<f.cloud_bytes<<",\"bigendian\":"<<f.bigendian<<",\"xyz_offsets\":["<<f.xyz[0]<<","<<f.xyz[1]<<","<<f.xyz[2]<<"]},\"samplers\":[";
  for(unsigned i=0;i<f.sampler_count;++i){const auto &s=f.samplers[i];if(i)j<<",";
    j<<"{\"quad\":"<<s.quad<<",\"pass\":"<<s.pass<<",\"texture\":"<<s.texture<<",\"unit\":"<<s.unit<<",\"name\":"<<std::quoted(s.name)
     <<",\"width\":"<<s.width<<",\"height\":"<<s.height<<",\"filter_address\":[";
    for(unsigned k=0;k<6;++k){if(k)j<<",";j<<s.filter[k];}
    j<<"],\"camera\":"<<s.camera<<",\"camera_pose\":[";
    for(unsigned k=0;k<7;++k){if(k)j<<",";j<<s.camera_pose[k];}
    j<<"],\"camera_frustum\":[";for(unsigned k=0;k<4;++k){if(k)j<<",";j<<s.camera_frustum[k];}
    j<<"],\"parameters\":"<<s.parameters<<",\"uniform_mask\":"<<s.uniform_mask<<",\"uniform_values\":[";
    for(unsigned k=0;k<10;++k){if(k)j<<",";number(j,s.uniform[k]);}j<<"]}";
  }
  j<<"],\"sampler_drops\":"<<f.sampler_drops<<",\"sampler_observation_phase\":\"post_original_execute\",\"camera_pose_uniform_semantics\":\"post_execute_CPU_state_not_draw_time_certificate\",\"first_pass_depth\":\"NOT_CAPTURED\",\"geometry_checked_frames\":"<<geometry_checked_frames.load()<<",\"GZ_FLOAT32\":"<<int(gz::msgs::PointCloudPacked::Field::FLOAT32)<<",\"additional_GPU_readbacks\":0}\n";
  j.close();if(!j)++file_errors;
}
void writer() {
  const char *p=std::getenv("GO2_RAY_DIAG_DIR");if(!p)return;const std::string dir(p);
  auto drain=[&](){
    if(config_state.load(std::memory_order_acquire)==1){
      binary(dir+"/sample_uv_rgba32f.bin",config.uv.data(),config.image_bytes);
      std::ofstream j(dir+"/sample_config.json");
      j<<"{\"schema\":\"R50_actual_sample_config/v1\",\"rays\":"<<config.rays<<",\"texture\":"<<config.texture<<",\"wall_ns\":"<<config.wall_ns
       <<",\"width\":"<<config.w<<",\"height\":"<<config.h<<",\"format\":"<<config.format<<",\"row_bytes\":"<<config.row_bytes<<",\"image_bytes\":"<<config.image_bytes<<",\"textures\":[";
      for(unsigned i=0;i<config.count;++i){auto &t=config.textures[i];if(i)j<<",";
        j<<"{\"address\":"<<t.address<<",\"rays\":"<<t.rays<<",\"name\":"<<std::quoted(t.name)<<",\"width\":"<<t.width<<",\"height\":"<<t.height<<",\"depth\":"<<t.depth<<",\"format\":"<<t.format<<"}";
      }j<<"]}\n";j.close();if(!j)++file_errors;config_state.store(2,std::memory_order_release);
    }
    for(unsigned i=0;i<3;++i)if(frames[i].state.load(std::memory_order_acquire)==1){save_frame(dir,frames[i],i);frames[i].state.store(2,std::memory_order_release);}
  };
  while(!done.load(std::memory_order_acquire)){drain();std::this_thread::sleep_for(std::chrono::milliseconds(5));}drain();
  std::ofstream j(dir+"/observer_stats.json");j<<"{\"selected\":"<<selected<<",\"file_errors\":"<<file_errors.load()<<",\"foreign_drops\":"<<foreign_drops.load()<<",\"config_state\":"<<config_state.load()<<",\"rectified_frames\":"<<rectified_frames.load()<<",\"rectified_points\":"<<rectified_points.load()<<",\"original_noise_calls\":"<<original_noise_calls.load()<<",\"original_cloud_calls\":"<<original_cloud_calls.load()<<",\"original_cloud_success\":"<<original_cloud_success.load()<<",\"updates_without_pointcloud\":"<<updates_without_pointcloud.load()<<",\"geometry_checked_frames\":"<<geometry_checked_frames.load()<<",\"GZ_FLOAT32\":"<<int(gz::msgs::PointCloudPacked::Field::FLOAT32)<<",\"additional_GPU_readbacks\":0}\n";
}
struct Lifetime {
  std::thread consumer;
  Lifetime(){
    rectify=std::getenv("GO2_RECTIFY_RAYS") && std::strcmp(std::getenv("GO2_RECTIFY_RAYS"),"1")==0;
    if(rectify){
      const char *uv=std::getenv("GO2_RECTIFY_UV"),*direction=std::getenv("GO2_RECTIFY_DIRECTIONS");
      if(!uv||!direction)geometry_fail("missing frozen calibration paths");
      std::ifstream a(uv,std::ios::binary),b(direction,std::ios::binary);a.read(reinterpret_cast<char*>(calibrated_uv.data()),uvbytes);b.read(reinterpret_cast<char*>(calibrated_direction.data()),sizeof(calibrated_direction));
      if(!a||!b||a.peek()!=EOF||b.peek()!=EOF)geometry_fail("calibration size");
      for(size_t i=0;i<pixels;++i){double norm=0;for(unsigned k=0;k<3;++k){const auto v=calibrated_direction[i*3+k];if(!std::isfinite(v))geometry_fail("nonfinite direction");norm+=double(v)*v;}if(std::abs(norm-1)>1e-6)geometry_fail("direction norm");}
      calibration_loaded=true;
    }
    update_fn=next<UpdateFn>("_ZN2gz7sensors2v814GpuLidarSensor6UpdateERKNSt6chrono8durationIlSt5ratioILl1ELl1000000000EEEE");
    frame_fn=next<FrameFn>("_ZN2gz7sensors2v814GpuLidarSensor15OnNewLidarFrameEPKfjjjRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE");
    noise_fn=next<NoiseFn>("_ZN2gz7sensors2v85Lidar10ApplyNoiseEv");
    pub_fn=next<PubFn>("_ZN2gz9transport3v134Node9Publisher7PublishERKN6google8protobuf7MessageE");
    ecm_fn=next<EcmFn>("_ZN2gz3sim2v810RenderUtil13UpdateFromECMERKNS1_10UpdateInfoERKNS1_22EntityComponentManagerE");
    util_fn=next<UtilFn>("_ZN2gz3sim2v810RenderUtil6UpdateEv");
    enabled=std::getenv("GO2_RAY_DIAG_DIR")!=nullptr;if(enabled)consumer=std::thread(writer);
  }
  ~Lifetime(){done.store(true,std::memory_order_release);if(consumer.joinable())consumer.join();}
} lifetime;
}

extern "C" int go2_r50_validate_loaded_plugin() {
  const auto a=diag::plugin_next<void(*)(diag::Rays*)>("_ZN2gz9rendering2v812Ogre2GpuRays19CreateSampleTextureEv");
  const auto b=diag::plugin_next<void(*)(diag::Rays*)>("_ZN2gz9rendering2v812Ogre2GpuRays12Setup1stPassEv");
  const auto c=diag::plugin_next<void(*)(diag::Scene*)>("_ZN2gz9rendering2v810Ogre2Scene9PreRenderEv");
  return a&&b&&c;
}
extern "C" int go2_r50_validate_sensor_caller(void *address,unsigned which) {
  return diag::known_caller(address,which?diag::util_return:diag::ecm_return);
}
extern "C" int go2_r50_check_packed_layout(unsigned mode){
  gz::msgs::PointCloudPacked cloud;
  gz::msgs::InitPointCloudPacked(cloud,"velodyne",true,{{"xyz",gz::msgs::PointCloudPacked::Field::FLOAT32},{"intensity",gz::msgs::PointCloudPacked::Field::FLOAT32},{"ring",gz::msgs::PointCloudPacked::Field::UINT16}});
  cloud.set_width(480);cloud.set_height(64);cloud.set_row_step(480*cloud.point_step());cloud.set_data(std::string(diag::pixels*cloud.point_step(),'\0'));
  if(mode==1)cloud.mutable_field(0)->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT64);
  if(mode==2)*cloud.add_field()=cloud.field(0);
  if(mode==3)cloud.mutable_field(2)->set_name("missing_z");
  if(mode==4)cloud.mutable_field(1)->set_offset(0);
  if(mode==5)cloud.mutable_field(2)->set_count(2);
  if(mode==6)cloud.set_width(479);
  if(mode==7)cloud.set_data(cloud.data().substr(1));
  if(mode==8)cloud.set_is_bigendian(true);
  uint32_t offset[3]{};return diag::packed_layout(cloud,offset)==nullptr;
}
extern "C" int go2_r50_rectify_xyz(unsigned char *data,size_t n,uint32_t step,const uint32_t *offset,const float *ranges){return diag::rectify_xyz(data,n,step,offset,ranges);}

void gz::rendering::Ogre2GpuRays::CreateSampleTexture(){
  static auto fn=diag::plugin_next<void(*)(diag::Rays*)>("_ZN2gz9rendering2v812Ogre2GpuRays19CreateSampleTextureEv");
  auto old=diag::sample_owner;diag::sample_owner=this;fn(this);diag::sample_owner=old;
}
void gz::rendering::Ogre2GpuRays::Setup1stPass(){
  static auto fn=diag::plugin_next<void(*)(diag::Rays*)>("_ZN2gz9rendering2v812Ogre2GpuRays12Setup1stPassEv");
  auto old=diag::setup_owner;diag::setup_owner=this;fn(this);diag::setup_owner=old;
  // Seal only after all original first-pass texture dimensions have been observed.
  if(diag::config.rays==reinterpret_cast<uintptr_t>(this)&&diag::config.image_bytes)diag::config_state.store(1,std::memory_order_release);
}
void Ogre::TextureGpu::setResolution(uint32 w,uint32 h,uint32 d){
  static auto fn=diag::next<void(*)(TextureGpu*,uint32,uint32,uint32)>("_ZN4Ogre10TextureGpu13setResolutionEjjj");fn(this,w,h,d);
  if(diag::enabled && (diag::sample_owner||diag::setup_owner) && diag::own_thread() && diag::config_state.load()==0 && diag::config.count<32){
    auto &t=diag::config.textures[diag::config.count++];t.address=reinterpret_cast<uintptr_t>(this);
    t.rays=reinterpret_cast<uintptr_t>(diag::sample_owner?diag::sample_owner:diag::setup_owner);
    t.width=w;t.height=h;t.depth=d;diag::text(t.name,getNameStr());
  }
}
void Ogre::TextureGpu::_transitionTo(GpuResidency::GpuResidency residency,uint8 *data,bool autodelete){
  static auto fn=diag::next<void(*)(TextureGpu*,GpuResidency::GpuResidency,uint8*,bool)>("_ZN4Ogre10TextureGpu13_transitionToENS_12GpuResidency12GpuResidencyEPhb");
  if(diag::enabled && diag::sample_owner && data && diag::own_thread() && !diag::config.image_bytes){
    auto &c=diag::config;const auto row=_getSysRamCopyBytesPerRow(0),image=_getSysRamCopyBytesPerImage(0);
    if(getWidth()==480 && getHeight()==64 && getPixelFormat()==PFG_RGBA32_FLOAT && row==480*16 && image==diag::uvbytes){
      c.rays=reinterpret_cast<uintptr_t>(diag::sample_owner);c.texture=reinterpret_cast<uintptr_t>(this);c.w=getWidth();c.h=getHeight();c.format=getPixelFormat();
      c.row_bytes=row;c.image_bytes=image;c.wall_ns=diag::wall();std::memcpy(c.uv.data(),data,image);
    }
  }
  fn(this,residency,data,autodelete);
}
void Ogre::CompositorPassQuad::execute(const Camera *camera){
  static auto fn=diag::next<void(*)(CompositorPassQuad*,const Camera*)>("_ZN4Ogre18CompositorPassQuad7executeEPKNS_6CameraE");
  // The original execute binds dynamic texture sources and renders once.
  // Observe only after that binding; no early lazy _getTexturePtr calls.
  fn(this,camera);
  auto f=diag::active;auto pass=getPass();
  if(diag::rectify && diag::current_sensor && pass){
    for(size_t i=0;i<pass->getNumTextureUnitStates();++i){
      auto unit=pass->getTextureUnitState(i);auto tex=unit?unit->_getTexturePtr():nullptr;auto sb=unit?unit->getSamplerblock():nullptr;
      if(!tex||!sb)continue;
      const auto name=tex->getNameStr();
      const auto fail_sampler=[&](const char *reason){
        std::fprintf(stderr,"GO2_R50_SAMPLER_PREDICATES source_ns=%llu unit=%zu name=%s texture=%p width=%u height=%u filter=%d,%d,%d UVW=%d,%d,%d\n",static_cast<unsigned long long>(diag::current_source_ns),i,name.c_str(),static_cast<void*>(tex),tex->getWidth(),tex->getHeight(),int(sb->mMinFilter),int(sb->mMagFilter),int(sb->mMipFilter),int(sb->mU),int(sb->mV),int(sb->mW));
        diag::geometry_fail(reason);
      };
      if(name.find("_first_pass_")!=std::string::npos){
        if(tex->getWidth()!=128||tex->getHeight()!=128)fail_sampler("actual face resolution");
        if(sb->mMinFilter!=FO_POINT||sb->mMagFilter!=FO_POINT||sb->mMipFilter!=FO_NONE)fail_sampler("actual face nearest filter");
        if(sb->mU!=TAM_WRAP||sb->mV!=TAM_WRAP||sb->mW!=TAM_WRAP)fail_sampler("actual face address modes");
        ++diag::current_face_units;
      }
      if(reinterpret_cast<uintptr_t>(tex)==diag::config.texture){
        if(sb->mMinFilter!=FO_POINT||sb->mMagFilter!=FO_POINT||sb->mMipFilter!=FO_NONE)fail_sampler("actual UV nearest filter");
        if(sb->mU!=TAM_WRAP||sb->mV!=TAM_WRAP||sb->mW!=TAM_WRAP)fail_sampler("actual UV address modes");
        diag::current_uv_sampler=true;
      }
    }
  }
  if(f && pass){
    for(size_t i=0;i<pass->getNumTextureUnitStates();++i){
      auto unit=pass->getTextureUnitState(i);auto tex=unit?unit->_getTexturePtr():nullptr;auto sb=unit?unit->getSamplerblock():nullptr;
      if(!tex||!sb)continue;
      if(f->sampler_count==64){++f->sampler_drops;continue;}
      auto &s=f->samplers[f->sampler_count++];s.quad=reinterpret_cast<uintptr_t>(this);s.pass=reinterpret_cast<uintptr_t>(pass);s.texture=reinterpret_cast<uintptr_t>(tex);
      s.unit=i;s.width=tex->getWidth();s.height=tex->getHeight();diag::text(s.name,tex->getNameStr());
      s.filter[0]=sb->mMinFilter;s.filter[1]=sb->mMagFilter;s.filter[2]=sb->mMipFilter;s.filter[3]=sb->mU;s.filter[4]=sb->mV;s.filter[5]=sb->mW;
      auto camera=getCamera();
      if(camera){const auto &q=camera->getDerivedOrientation();const auto &p=camera->getDerivedPosition();s.camera=reinterpret_cast<uintptr_t>(camera);
        s.camera_pose[0]=p.x;s.camera_pose[1]=p.y;s.camera_pose[2]=p.z;s.camera_pose[3]=q.w;s.camera_pose[4]=q.x;s.camera_pose[5]=q.y;s.camera_pose[6]=q.z;
        s.camera_frustum[0]=camera->getFOVy().valueRadians();s.camera_frustum[1]=camera->getAspectRatio();s.camera_frustum[2]=camera->getNearClipDistance();s.camera_frustum[3]=camera->getFarClipDistance();
      }
      if(pass->hasFragmentProgram()){
        auto params=pass->getFragmentProgramParameters();const auto *cp=params.get();s.parameters=reinterpret_cast<uintptr_t>(cp);
        const char *names[]={"projectionParams","near","far","min","max","particleStddev","particleScatterRatio","rnd"};
        const unsigned offset[]={0,2,3,4,5,6,7,8},size[]={2,1,1,1,1,1,1,1};
        for(unsigned k=0;cp && k<8;++k){const auto def=cp->_findNamedConstantDefinition(names[k],false);
          if(def && def->isFloat() && def->physicalIndex+size[k]<=cp->getFloatConstantList().size()){
            const auto data=cp->getFloatPointer(def->physicalIndex);for(unsigned z=0;z<size[k];++z)s.uniform[offset[k]+z]=data[z];s.uniform_mask|=1u<<k;
          }
        }
      }
    }
  }
}
void gz::sim::RenderUtil::UpdateFromECM(const UpdateInfo &info,const EntityComponentManager &ecm){
  const bool known=diag::enabled&&diag::known_caller(__builtin_extract_return_addr(__builtin_return_address(0)),diag::ecm_return);
  diag::ecm_fn(this,info,ecm);
  if(known){auto b=diag::batch(this,true);if(b){++b->generation;b->step=info.iterations;b->ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();}}
}
void gz::sim::RenderUtil::Update(){
  const bool known=diag::enabled&&diag::known_caller(__builtin_extract_return_addr(__builtin_return_address(0)),diag::util_return);
  diag::Batch b{};if(known){auto p=diag::batch(this,false);if(p)b=*p;}
  diag::consumed=b;diag::consumed_scene=known?Scene().get():nullptr;diag::applied_generation=0;diag::applied_scene=nullptr;
  diag::util_fn(this);diag::consumed_scene=known?Scene().get():nullptr;
}
void gz::rendering::Ogre2Scene::PreRender(){
  static auto fn=diag::plugin_next<void(*)(diag::Scene*)>("_ZN2gz9rendering2v810Ogre2Scene9PreRenderEv");fn(this);
  if(diag::enabled && static_cast<gz::rendering::Scene*>(this)==diag::consumed_scene){diag::applied_generation=diag::consumed.generation;diag::applied_scene=diag::consumed_scene;}
}
bool gz::sensors::GpuLidarSensor::Update(const std::chrono::steady_clock::duration &now){
  const uint64_t ns=std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  const auto old_sensor=diag::current_sensor;const auto old_source=diag::current_source_ns;
  if(diag::rectify && old_sensor)diag::geometry_fail("unexpected nested GPU lidar Update");
  diag::current_sensor=this;diag::current_source_ns=ns;diag::current_ranges=nullptr;
  diag::current_noise_calls=0;diag::current_publish_calls=0;diag::current_face_units=0;diag::current_uv_sampler=false;
  if(!diag::enabled || ns<8000000000ULL || diag::selected==3 || !diag::own_thread()){
    const bool result=diag::update_fn(this,now);diag::finish_update(result);diag::current_sensor=old_sensor;diag::current_source_ns=old_source;return result;
  }
  auto &f=diag::frames[diag::selected++];f.source=ns;f.begin_wall=diag::wall();f.sensor=reinterpret_cast<uintptr_t>(this);f.rays=reinterpret_cast<uintptr_t>(GpuRays().get());
  f.util=reinterpret_cast<uintptr_t>(diag::consumed.util);f.scene=reinterpret_cast<uintptr_t>(diag::consumed_scene);f.ray_scene=reinterpret_cast<uintptr_t>(GpuRays()->Scene().get());f.generation=diag::consumed.generation;f.step=diag::consumed.step;f.physics_ns=diag::consumed.ns;
  f.stages=1;diag::pose(this,f.pose);auto old=diag::active;diag::active=&f;f.ok=diag::update_fn(this,now);diag::finish_update(f.ok);diag::active=old;diag::current_sensor=old_sensor;diag::current_source_ns=old_source;f.applied=diag::applied_generation;f.end_wall=diag::wall();f.state.store(1,std::memory_order_release);return f.ok;
}
void gz::sensors::GpuLidarSensor::OnNewLidarFrame(const float *scan,unsigned w,unsigned h,unsigned channels,const std::string &format){
  auto f=diag::active;
  if(f && f->sensor==reinterpret_cast<uintptr_t>(this) && scan && w==480 && h==64 && channels==3){
    ++f->callbacks;f->w=w;f->h=h;f->channels=channels;f->readback_wall=diag::wall();std::memcpy(f->pre.data(),scan,sizeof(f->pre));double p[7]{};diag::pose(this,p);
    for(size_t i=0;i<7;++i){f->pose_delta=std::max(f->pose_delta,std::abs(p[i]-f->pose[i]));}f->stages|=2;
  }diag::frame_fn(this,scan,w,h,channels,format);
}
void gz::sensors::Lidar::ApplyNoise(){
  diag::noise_fn(this);auto f=diag::active;
  if(diag::current_sensor && static_cast<Lidar*>(diag::current_sensor)==this){diag::current_ranges=laserBuffer;++diag::current_noise_calls;++diag::original_noise_calls;}
  if(f && static_cast<Lidar*>(reinterpret_cast<Sensor*>(f->sensor))==this && (f->stages&2) && laserBuffer){std::memcpy(f->post.data(),laserBuffer,sizeof(f->post));f->noise_wall=diag::wall();f->stages|=4;}
}
bool gz::transport::Node::Publisher::Publish(const google::protobuf::Message &msg){
  auto f=diag::active;auto cloud=(f||(diag::rectify&&diag::current_sensor))?dynamic_cast<const gz::msgs::PointCloudPacked*>(&msg):nullptr;
  if(f && cloud && (f->stages&4) && cloud->data().size()<=diag::cloudcap){
    f->publish=uint64_t(cloud->header().stamp().sec())*1000000000ULL+cloud->header().stamp().nsec();f->pub_wall=diag::wall();
    f->cloud_w=cloud->width();f->cloud_h=cloud->height();f->point_step=cloud->point_step();f->row_step=cloud->row_step();f->bigendian=cloud->is_bigendian();f->cloud_bytes=cloud->data().size();
    for(const auto &field:cloud->field())for(unsigned i=0;i<3;++i)if(field.name()==std::string(1,"xyz"[i]))f->xyz[i]=field.offset();
    std::memcpy(f->cloud.data(),cloud->data().data(),f->cloud_bytes);f->stages|=8;
  }
  const google::protobuf::Message *output=&msg;gz::msgs::PointCloudPacked corrected;
  if(diag::rectify && cloud){
    ++diag::original_cloud_calls;
    const uint64_t stamp=uint64_t(cloud->header().stamp().sec())*1000000000ULL+cloud->header().stamp().nsec();
    if(!diag::current_sensor || !diag::current_ranges || diag::current_noise_calls!=1 || ++diag::current_publish_calls!=1 || stamp!=diag::current_source_ns)diag::geometry_fail("single original noise/source/publication");
    const auto current_rays=reinterpret_cast<uintptr_t>(diag::current_sensor->GpuRays().get());
    const bool uv_equal=diag::config.image_bytes==diag::uvbytes && std::memcmp(diag::config.uv.data(),diag::calibrated_uv.data(),diag::uvbytes)==0;
    if(!diag::current_uv_sampler||diag::current_face_units!=6||diag::config.image_bytes!=diag::uvbytes||diag::config.rays!=current_rays||!uv_equal){
      std::fprintf(stderr,"GO2_R50_GEOMETRY_PREDICATES source_ns=%llu uv_sampler=%d face_units=%u image_bytes=%llu expected_image_bytes=%zu config_rays=%llu current_rays=%llu uv_equal=%d\n",static_cast<unsigned long long>(stamp),diag::current_uv_sampler,diag::current_face_units,static_cast<unsigned long long>(diag::config.image_bytes),diag::uvbytes,static_cast<unsigned long long>(diag::config.rays),static_cast<unsigned long long>(current_rays),uv_equal);
      if(!diag::current_uv_sampler)diag::geometry_fail("runtime UV sampler absent");
      if(diag::current_face_units!=6)diag::geometry_fail("runtime six face texture bindings");
      if(diag::config.image_bytes!=diag::uvbytes)diag::geometry_fail("runtime UV image bytes");
      if(diag::config.rays!=current_rays)diag::geometry_fail("runtime GpuRays identity");
      diag::geometry_fail("runtime UV bytes changed");
    }
    if(diag::geometry_checked_frames.fetch_add(1)==0)std::fprintf(stderr,"GO2_R50_GEOMETRY_FIRST_PASS source_ns=%llu face_units=%u uv_sampler=%d uv_equal=%d rays_equal=%d GZ_FLOAT32=%d\n",static_cast<unsigned long long>(stamp),diag::current_face_units,diag::current_uv_sampler,uv_equal,diag::config.rays==current_rays,int(gz::msgs::PointCloudPacked::Field::FLOAT32));
    uint32_t offset[3]{};
    if(const auto reason=diag::packed_layout(*cloud,offset))diag::geometry_fail(reason);
    corrected=*cloud;auto payload=corrected.mutable_data();
    const int points=diag::rectify_xyz(reinterpret_cast<unsigned char*>(payload->data()),diag::pixels,32,offset,diag::current_ranges);
    if(points<0)diag::geometry_fail("XYZ field offsets");
    if(f)std::memcpy(f->rectified_cloud.data(),payload->data(),payload->size());
    output=&corrected;++diag::rectified_frames;diag::rectified_points.fetch_add(points);
  }
  const bool result=diag::pub_fn(this,*output);if(diag::rectify && cloud && result)++diag::original_cloud_success;if(f && cloud && (f->stages&8))f->pub_ok=result;return result;
}

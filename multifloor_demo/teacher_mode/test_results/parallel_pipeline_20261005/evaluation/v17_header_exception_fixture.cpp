// Prospective limited production LIVMapper header/deferred-image-error fixture.
// Not yet built/executed. No subscriptions, spin, mapping loop, Gazebo or model.
// Run one case per process against the NEW matching candidate header/library.
#include "LIVMapper.h"
#include <rclcpp/rclcpp.hpp>
#include <cassert>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <thread>
#include <sys/syscall.h>
#include <unistd.h>

uint64_t tid(){return uint64_t(syscall(SYS_gettid));}
bool trace_ok(const std::filesystem::path&path,uint64_t source,uint64_t receive,uint64_t owner){
  std::ifstream in(path);std::string line;std::vector<std::string> rows;
  while(std::getline(in,line))if(!line.empty())rows.push_back(line);
  if(rows.size()!=1)return false;
  std::vector<std::string> f;std::stringstream stream(rows[0]);
  while(std::getline(stream,line,','))f.push_back(line);
  if(f.size()!=13||std::stoull(f[0])!=1||std::stoull(f[2])!=source||
     std::stoull(f[7])!=receive||std::stoull(f[8])!=owner||receive==owner)return false;
  const double r=std::stod(f[3]),d=std::stod(f[4]),p=std::stod(f[5]),e=std::stod(f[6]);
  return r<=d&&d<=p&&p<=e&&std::stoull(f[9])<=std::stoull(f[10])&&
         std::stoull(f[11])==0&&std::stoull(f[12])==0;
}
sensor_msgs::msg::PointCloud2::SharedPtr standard_cloud(){
  auto m=std::make_shared<sensor_msgs::msg::PointCloud2>();
  m->header.stamp.sec=11;m->height=1;m->width=5;m->point_step=16;m->row_step=80;m->is_bigendian=false;m->is_dense=false;
  for(int i=0;i<4;++i){sensor_msgs::msg::PointField field;field.name=std::vector<std::string>{"x","y","z","intensity"}[i];field.offset=i*4;field.datatype=sensor_msgs::msg::PointField::FLOAT32;field.count=1;m->fields.push_back(field);}
  const float n=std::numeric_limits<float>::quiet_NaN();
  const float p[5][4]={{1,0,0,5},{n,0,0,6},{.01f,0,0,7},{-2,1,0,8},{0,0,3,9}};
  m->data.resize(sizeof(p));std::memcpy(m->data.data(),p,sizeof(p));return m;
}
void write_points(std::ofstream&out,const PointCloudXYZI&cloud){
  uint64_t n=cloud.size();out.write(reinterpret_cast<char*>(&n),sizeof(n));
  for(const auto&p:cloud){const float values[]={p.x,p.y,p.z,p.intensity,p.normal_x,p.normal_y,p.normal_z,p.curvature};out.write(reinterpret_cast<const char*>(values),sizeof(values));}
}

int main(int argc,char**argv) {
  if(argc<4) return 2;
  const std::string mode=argv[1],which=argv[2];
  const auto output=std::filesystem::absolute(argv[3]);
  if(mode!="direct"&&mode!="queued")return 2;
  // Caller supplies frozen navigation and camera --params-file arguments.
  // The domain must be a private non-physical test domain, and output must be owned.
  std::filesystem::create_directories(output);
  setenv("DEMO_RUN_DIR",output.c_str(),1);
  setenv("FASTLIVO_DIAGNOSTIC_DIR",(output/"raw_diagnostics").c_str(),1);
  setenv("FASTLIVO_INGRESS_PIPELINE","1",1);
  rclcpp::init(argc,argv);
  rclcpp::NodeOptions options;
  options.start_parameter_services(false).start_parameter_event_publisher(false).enable_rosout(false);
  if(which=="hilti_rejected")options.parameter_overrides({rclcpp::Parameter("preprocess.hilti_en",true)});
  auto node=std::make_shared<rclcpp::Node>("isolated_ingress_header_fixture",options);
  // Only the camera names: production readParameters declares its own names.
  // Frozen YAML overrides resolve these values; avoid remote parameter/CLI waits.
  node->declare_parameter<std::string>("cam_model","Pinhole");
  node->declare_parameter<int>("cam_width",640);node->declare_parameter<int>("cam_height",480);
  node->declare_parameter<double>("scale",1.0);
  for(const auto&name:{"cam_fx","cam_fy","cam_cx","cam_cy","cam_d0","cam_d1","cam_d2","cam_d3"})node->declare_parameter<double>(name,0.0);
  if(which=="hilti_rejected"){
    bool rejected=false;std::string reason;
    try{LIVMapper mapper(node,node->get_name());}
    catch(const std::runtime_error&e){reason=e.what();rejected=reason=="V17 pipeline requires generic cloud, IMU+RGB, GPS off, HILTI off";}
    std::ofstream(output/"guard_error.txt")<<reason;
    rclcpp::shutdown();return rejected?0:12;
  }
  {
    LIVMapper mapper(node,node->get_name());
    // Deliberately do not initializeSubscribersAndPublishers/run/start_ingress.
    mapper.last_timestamp_img=10.0;
    mapper.img_time_offset=0;
    mapper.ingress_trace.open(output/"fixture_ingress.csv");
    if(!mapper.ingress_trace)return 3;
    mapper.ingress_trace<<std::setprecision(17); // Same production start_ingress format.
    if(which=="cloud_standard_filter"){
      auto msg=standard_cloud();PointCloudXYZI::Ptr reference(new PointCloudXYZI());
      mapper.p_pre->process(msg,reference);
      if(reference->size()!=3)return 13;
      uint64_t received=tid();
      if(mode=="queued"){
        std::thread worker([&]{received=tid();mapper.receive_cloud(msg);});worker.join();mapper.commit_ingress();
      }else{mapper.ingress_enabled=false;mapper.standard_pcl_cbk(msg);}
      if(mapper.lid_raw_data_buffer.size()!=1||mapper.lid_raw_data_buffer.front()->size()!=3||mapper.last_timestamp_lidar!=11)return 14;
      const auto&actual=*mapper.lid_raw_data_buffer.front();
      for(size_t i=0;i<actual.size();++i){
        const auto&a=actual[i];const auto&b=(*reference)[i];
        const float x[]={a.x,a.y,a.z,a.intensity,a.normal_x,a.normal_y,a.normal_z,a.curvature};
        const float y[]={b.x,b.y,b.z,b.intensity,b.normal_x,b.normal_y,b.normal_z,b.curvature};
        if(std::memcmp(x,y,sizeof(x)))return 15;
      }
      mapper.ingress_trace.flush();
      if(mode=="queued"&&!trace_ok(output/"fixture_ingress.csv",11000000000ULL,received,tid()))return 16;
      std::ofstream canonical(output/"canonical.bin",std::ios::binary);write_points(canonical,actual);
      std::cout<<"limited_cloud_case=standard_filter points="<<actual.size()<<" mode="<<mode<<'\n';
      rclcpp::shutdown();return canonical?0:17;
    }
    auto msg=std::make_shared<sensor_msgs::msg::Image>();
    msg->height=2;msg->width=2;msg->step=2;
    msg->encoding="THIS_IS_NOT_AN_IMAGE_ENCODING";
    msg->data={1,2,3,4};
    int64_t stamp=10000000000LL;
    bool expected_throw=false,expected_insert=false;
    if(which=="duplicate_bad")stamp=10000000000LL;
    else if(which=="near_duplicate_bad")stamp=10000500000LL;
    else if(which=="backward_bad")stamp=9000000000LL;
    else if(which=="under_20ms_bad")stamp=10010000000LL;
    else if(which=="fresh_bad") {stamp=10050000000LL;expected_throw=true;}
    else if(which=="fresh_valid") {
      stamp=10050000000LL;expected_insert=true;
      msg->encoding="bgr8";msg->step=6;
      msg->data={1,2,3,4,5,6,7,8,9,10,11,12};
    } else if(which=="fresh_empty") {
      stamp=10050000000LL;expected_insert=true;msg->data.clear();
    } else return 2;
    msg->header.stamp.sec=stamp/1000000000LL;
    msg->header.stamp.nanosec=stamp%1000000000LL;
    bool decoder_threw=false;
    try {(void)mapper.getImageFromMsg(msg);}catch(const std::exception&){decoder_threw=true;}
    if(which.find("bad")!=std::string::npos && !decoder_threw)return 4;
    bool threw=false;std::string what;
    uint64_t received=tid();
    if(mode=="queued") {
      // Exercise actual receiver on a distinct worker; owner runs actual commit.
      std::thread receiver([&]{received=tid();mapper.receive_image(msg);});receiver.join();
      const auto accepted=mapper.ingress_queue.stats();
      if(accepted.accepted!=1||accepted.pending!=1||!accepted.failure.empty())return 5;
      try {mapper.commit_ingress();}
      catch(const std::exception&e){threw=true;what=e.what();}
    }else{
      mapper.ingress_enabled=false;
      try {mapper.img_cbk(msg);}
      catch(const std::exception&e){threw=true;what=e.what();}
    }
    // Original decoder exception occurs with this manual mutex locked. Release
    // solely to let this isolated fixture destruct; do not repair production here.
    if(threw)mapper.mtx_buffer.unlock();
    if(threw!=expected_throw||mapper.img_buffer.size()!=size_t(expected_insert))return 6;
    const double expected_stamp=expected_insert?stamp2Sec(msg->header.stamp):10.0;
    if(mapper.last_timestamp_img!=expected_stamp)return 7;
    if(expected_insert && which=="fresh_valid") {
      const auto&image=mapper.img_buffer.front();
      if(image.rows!=2||image.cols!=2||image.type()!=CV_8UC3||!image.isContinuous()||
         std::memcmp(image.data,msg->data.data(),12)!=0)return 8;
      // Receiver decoder clone must not alias mutable original message bytes.
      const auto value=image.data[0];msg->data[0]^=0xff;
      if(image.data[0]!=value)return 9;
    }
    if(expected_insert && which=="fresh_empty" && !mapper.img_buffer.front().empty())return 10;
    const auto final=mapper.ingress_queue.stats();
    if(mode=="queued" && (final.delivered!=1||final.pending||
       mapper.ingress_committed!=uint64_t(!expected_throw)||mapper.committing_ingress))return 11;
    mapper.ingress_trace.flush();
    if(mode=="queued"&&!expected_throw&&!trace_ok(output/"fixture_ingress.csv",stamp,received,tid()))return 18;
    if(mode=="queued"&&expected_throw){
      // No normal completed trace is invented for an exception, but counters prove
      // actual delivered!=committed and the caller recorded original exception.
      if(std::filesystem::file_size(output/"fixture_ingress.csv")!=0)return 19;
    }
    std::ofstream canonical(output/"canonical.bin",std::ios::binary);
    const uint64_t buffered=mapper.img_buffer.size();canonical.write(reinterpret_cast<const char*>(&buffered),sizeof(buffered));
    canonical.write(reinterpret_cast<const char*>(&mapper.last_timestamp_img),sizeof(mapper.last_timestamp_img));
    for(const auto&image:mapper.img_buffer){const uint64_t fields[]={uint64_t(image.rows),uint64_t(image.cols),uint64_t(image.type()),uint64_t(image.total()*image.elemSize())};canonical.write(reinterpret_cast<const char*>(fields),sizeof(fields));if(!image.empty())for(int y=0;y<image.rows;++y)canonical.write(reinterpret_cast<const char*>(image.ptr(y)),image.cols*image.elemSize());}
    std::ofstream(output/"callback_error.txt")<<what;
    std::cout<<"limited_header_case="<<which<<" mode="<<mode
             <<" decoder_threw="<<decoder_threw<<" original_callback_threw="<<threw
             <<" buffered="<<mapper.img_buffer.size()<<" last="<<mapper.last_timestamp_img
             <<" accepted="<<final.accepted<<" delivered="<<final.delivered
             <<" committed="<<mapper.ingress_committed<<" error="<<what<<'\n';
  }
  rclcpp::shutdown();return 0;
}

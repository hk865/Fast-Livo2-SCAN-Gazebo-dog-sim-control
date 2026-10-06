// Actual Gazebo entity poses, diagnostic only. This node never publishes or
// calls a service, never records robot poses, and has no navigation connection.
#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>
#include <chrono>
#include <csignal>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

static volatile std::sig_atomic_t stopped=0;
static void End(int){stopped=1;}
static double Wall(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}

int main(int argc,char **argv){
  if(argc==2&&std::string(argv[1])=="--prepare"){
    std::cout<<"{\"status\":\"prepared_unverified\",\"subscribes\":false,\"publishes\":false,\"services\":false,\"allowed_entity\":\"moving_obstacle\"}\n";return 0;
  }
  if(argc!=4){std::cerr<<"Usage: obstacle_pose_observer TOPIC OUTPUT_JSONL WALL_DURATION_SECONDS; or --prepare\n";return 2;}
  std::string topic=argv[1];
  if(topic!="/world/teacher_demo/pose/info"&&topic!="/model/moving_obstacle/pose"&&topic!="/model/moving_obstacle/pose_static"){
    std::cerr<<"Only the Teacher world or moving_obstacle model pose topic is permitted\n";return 2;
  }
  double duration=std::stod(argv[3]);if(!std::isfinite(duration)||duration<1||duration>900)return 2;
  std::ofstream output(argv[2],std::ios::app);if(!output)return 2;
  output<<std::setprecision(17);std::mutex mutex;uint64_t samples=0;uint32_t entity=0;bool initialized=false,failed=false;
  gz::transport::Node node;
  std::function<void(const gz::msgs::Pose_V&)> callback=[&](const gz::msgs::Pose_V &message){
    const double received=Wall();std::lock_guard<std::mutex>lock(mutex);
    for(const auto &pose:message.pose()){
      if(pose.name()!="moving_obstacle"&&pose.name()!="teacher_demo::moving_obstacle")continue;
      const auto &p=pose.position();const auto &q=pose.orientation();
      const auto &header=pose.has_header()&&pose.header().has_stamp()?pose.header():message.header();
      const bool timestamp=header.has_stamp();const auto &stamp=header.stamp();
      if(!timestamp||stamp.sec()<0||stamp.nsec()<0||stamp.nsec()>=1000000000||
         !std::isfinite(p.x())||!std::isfinite(p.y())||!std::isfinite(p.z())||
         !std::isfinite(q.x())||!std::isfinite(q.y())||!std::isfinite(q.z())||!std::isfinite(q.w())){
        failed=true;output<<"{\"status\":\"failed\",\"reason\":\"Actual model pose has invalid stamp/geometry\"}"<<std::endl;continue;
      }
      if(initialized&&entity!=pose.id()){
        failed=true;output<<"{\"status\":\"failed\",\"reason\":\"moving_obstacle entity id changed\"}"<<std::endl;continue;
      }
      initialized=true;entity=pose.id();samples++;
      const auto ns=stamp.sec()*1000000000LL+stamp.nsec();
      output<<"{\"schema\":1,\"status\":\"actual_observed\",\"source\":\"Gazebo actual gz.msgs.Pose_V diagnostic only; not a command acknowledgement\",\"navigation_input\":false,\"topic\":\""<<topic
        <<"\",\"model\":\"moving_obstacle\",\"entity_id\":"<<entity<<",\"sample\":"<<samples
        <<",\"stamp_ns\":"<<ns<<",\"received_monotonic_wall\":"<<received
        <<",\"position\":["<<p.x()<<","<<p.y()<<","<<p.z()<<"],\"quaternion_xyzw\":["<<q.x()<<","<<q.y()<<","<<q.z()<<","<<q.w()<<"]}"<<std::endl;
    }
  };
  if(!node.Subscribe(topic,callback)){std::cerr<<"Pose subscription failed\n";return 2;}
  std::signal(SIGINT,End);std::signal(SIGTERM,End);const double end=Wall()+duration;
  while(!stopped&&Wall()<end)std::this_thread::sleep_for(std::chrono::milliseconds(20));
  node.Unsubscribe(topic);std::lock_guard<std::mutex>lock(mutex);
  output<<"{\"kind\":\"observer_end\",\"samples\":"<<samples<<",\"status\":\""<<(samples&&!failed?"observed_unverified":"failed")<<"\",\"navigation_input\":false}"<<std::endl;
  return samples&&!failed?0:3;
}

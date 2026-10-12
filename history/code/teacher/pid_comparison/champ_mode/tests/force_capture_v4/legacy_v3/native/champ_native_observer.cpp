// Passive CHAMP evidence recorder. Never writes JointForceCmd, joint state or body wrench.
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <sstream>
#include <vector>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/SystemPluginInfo.hh>
namespace champ_compare {
static std::string Quote(const std::string &s) {
  std::string out = "\"";
  for (unsigned char c : s) {
    if (c == '\\' || c == '"') { out += '\\'; out += c; }
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if (c == '\t') out += "\\t";
    else if (c >= 0x20) out += c;
  }
  return out + "\"";
}

template <class T, size_t N>
static void Array(std::ostream &out, const std::array<T, N> &a) {
  out << '[';
  for (size_t i = 0; i < N; ++i) { if (i) out << ','; out << a[i]; }
  out << ']';
}

static void Vector(std::ostream &out, const gz::math::Vector3d &v) {
  out << '[' << v.X() << ',' << v.Y() << ',' << v.Z() << ']';
}
static void MessageVector(std::ostream &out, const gz::msgs::Vector3d &v) {
  out << '[' << v.x() << ',' << v.y() << ',' << v.z() << ']';
}


class NativeObserver final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate {
 public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &events) override {
    modelEntity=entity; eventManager=&events;
    gz::sim::Model model(entity); base=gz::sim::Link(model.LinkByName(ecm,"base_link"));
    if (!base.Valid(ecm)) throw std::runtime_error("CHAMP observer base missing");
    base.EnableVelocityChecks(ecm);
    const auto inertia=ecm.Component<gz::sim::components::Inertial>(base.Entity());
    if (!inertia) throw std::runtime_error("CHAMP base inertia missing");
    comOffset=inertia->Data().Pose().Pos();
    const char *path=std::getenv("CHAMP_ACTUATOR_LOG");
    if (!path || !*path) path=std::getenv("TEACHER_ACTUATOR_LOG");
    if (!path || !*path) throw std::runtime_error("CHAMP_ACTUATOR_LOG is required");
    endTime=sdf->Get<double>("duration_s",120.).first;
    donePath=sdf->Get<std::string>("done_file","").first;
    statePath=sdf->Get<std::string>("state_file","").first;
    if(donePath.empty()||statePath.empty())throw std::runtime_error("CHAMP done_file and state_file required");
    if (!std::isfinite(endTime) || endTime<5 || endTime>600) throw std::runtime_error("Invalid CHAMP duration");
    log.open(path,std::ios::out|std::ios::trunc); if (!log) throw std::runtime_error("Cannot open observer log");
    log<<std::setprecision(17);
    for (size_t i=0;i<12;++i) {
      joints[i]=gz::sim::Joint(model.JointByName(ecm,names[i]));
      if (!joints[i].Valid(ecm)) throw std::runtime_error("CHAMP observer joint missing");
      joints[i].EnablePositionCheck(ecm); joints[i].EnableVelocityCheck(ecm);
    }
    const std::array<std::string,5> links={"base_link","rf_lower_leg_link","lf_lower_leg_link","rh_lower_leg_link","lh_lower_leg_link"};
    for (size_t group=0;group<5;++group) {
      gz::sim::Link link(model.LinkByName(ecm,links[group]));
      if (!link.Valid(ecm)) throw std::runtime_error("CHAMP contact link missing");
      for (auto collision:link.Collisions(ecm)) {
        const auto name=ecm.Component<gz::sim::components::Name>(collision);
        if (group && (!name || name->Data().find("foot")==std::string::npos)) continue;
        contactEntities[group].push_back(collision);
        if (!ecm.Component<gz::sim::components::ContactSensorData>(collision))
          ecm.CreateComponent(collision,gz::sim::components::ContactSensorData());
      }
      if (contactEntities[group].empty()) throw std::runtime_error("CHAMP contact geometry missing");
    }
    log<<"{\"kind\":\"actuator_contract\",\"writer\":\"gz_ros2_control only; observer has no actuation API\","
       <<"\"state_phase\":\"PostUpdate\",\"state_time_offset_s\":0,\"body_pose_resets\":0,\"observer_joint_position_reset_count\":0,"
       <<"\"torque_source\":\"JointForceCmd component after physics; commanded force, not measured motor torque\","
       <<"\"qtarget_source\":\"separate ROS JointTrajectory original header/receive clock evidence\",\"joint_order\":[";
    for(size_t i=0;i<12;++i){if(i)log<<',';log<<Quote(names[i]);}
    log<<"],\"base_com_offset_body_m\":";Vector(log,comOffset);log<<"}\n";log.flush();
  }
  void PostUpdate(const gz::sim::UpdateInfo &info,const gz::sim::EntityComponentManager &ecm) override {
    if(info.paused)return;
    const double time=std::chrono::duration<double>(info.simTime).count();
    if(!ownershipChecked) {
      ownershipChecked=true;const auto plugins=ecm.Component<gz::sim::components::SystemPluginInfo>(modelEntity);
      int writers=0;bool valid=plugins!=nullptr;log<<"{\"kind\":\"model_plugin_ownership\",\"plugins\":[";
      bool first=true;
      if(plugins)for(const auto &p:plugins->Data().plugins()){
        if(!first)log<<',';
        first=false;log<<"{\"name\":"<<Quote(p.name())<<",\"filename\":"<<Quote(p.filename())<<'}';
        std::string id=p.name()+" "+p.filename();std::transform(id.begin(),id.end(),id.begin(),[](unsigned char c){return std::tolower(c);});
        if(id.find("gz_ros2_control")!=std::string::npos)++writers;
        else if(id.find("teacheractuator")!=std::string::npos||id.find("teacher_actuator")!=std::string::npos||id.find("velocity")!=std::string::npos||id.find("joint-controller")!=std::string::npos)valid=false;
      }
      valid=valid&&writers==1;log<<"],\"ros2_control_writers\":"<<writers<<",\"passed\":"<<(valid?"true":"false")<<"}\n";log.flush();
      if(!valid){log<<"{\"kind\":\"fault\",\"t\":"<<time<<",\"code\":8,\"reason\":\"multiple_or_missing_control_writer\"}\n";log.flush();eventManager->Emit<gz::sim::events::Stop>();return;}
    }
    auto pose=base.WorldPose(ecm);
    auto origin=base.WorldLinearVelocity(ecm), com=base.WorldLinearVelocity(ecm,comOffset), omega=base.WorldAngularVelocity(ecm);
    std::array<double,12> q{},qd{},tau{};std::array<int,12> tauValid{};
    bool valid=pose.has_value()&&origin.has_value()&&com.has_value()&&omega.has_value();
    for(size_t i=0;i<12;++i){auto p=joints[i].Position(ecm),v=joints[i].Velocity(ecm);
      if(!p||!v||p->size()!=1||v->size()!=1){valid=false;continue;}q[i]=(*p)[0];qd[i]=(*v)[0];
      const auto force=ecm.Component<gz::sim::components::JointForceCmd>(joints[i].Entity());
      if(force&&force->Data().size()==1&&std::isfinite(force->Data()[0])){tau[i]=force->Data()[0];tauValid[i]=1;}
      valid=valid&&std::isfinite(q[i])&&std::isfinite(qd[i]);
    }
    if(!valid){log<<"{\"kind\":\"state_unavailable\",\"t\":"<<time<<"}\n";return;}
    if(!pose->Pos().IsFinite()||!pose->Rot().IsFinite()||!origin->IsFinite()||!com->IsFinite()||!omega->IsFinite())throw std::runtime_error("Nonfinite CHAMP native state");
    std::array<int,5> contactCounts{};
    for(size_t g=0;g<5;++g)for(auto c:contactEntities[g]){const auto data=ecm.Component<gz::sim::components::ContactSensorData>(c);if(data)contactCounts[g]+=data->Data().contact_size();else contactCounts[g]=-1;}
    log<<"{\"kind\":\"physics_step\",\"t\":"<<time<<",\"dt\":"<<std::chrono::duration<double>(info.dt).count()
       <<",\"state_time_offset_s\":0,\"state_phase\":\"PostUpdate\",\"iteration\":"<<iteration++<<",\"mode\":\"CHAMP\",\"fault\":0,\"q\":";
    Array(log,q);log<<",\"qd\":";Array(log,qd);log<<",\"qtarget\":null,\"tau\":";Array(log,tau);log<<",\"tau_available\":";Array(log,tauValid);
    log<<",\"contacts\":";Array(log,contactCounts);
    log<<",\"position\":";Vector(log,pose->Pos());log<<",\"quaternion_wxyz\":["<<pose->Rot().W()<<','<<pose->Rot().X()<<','<<pose->Rot().Y()<<','<<pose->Rot().Z()<<']';
    log<<",\"body_lin_vel_origin\":";Vector(log,pose->Rot().RotateVectorReverse(*origin));log<<",\"body_lin_vel_com\":";Vector(log,pose->Rot().RotateVectorReverse(*com));
    log<<",\"body_ang_vel\":";Vector(log,pose->Rot().RotateVectorReverse(*omega));log<<",\"contact_pairs\":[";
    bool first = true;
    for (size_t group = 0; group < 5; ++group) for (auto collision : contactEntities[group]) {
      const auto contacts = ecm.Component<gz::sim::components::ContactSensorData>(collision);
      if (!contacts) continue;
      for (const auto &contact : contacts->Data().contact()) {
        if (!first) log << ',';
        first = false;
        double maxDepth = 0;
        for (double depth : contact.depth()) maxDepth = std::max(maxDepth, depth);
        log << "{\"group\":" << group << ",\"a\":" << Quote(contact.collision1().name())
            << ",\"b\":" << Quote(contact.collision2().name()) << ",\"points\":" << contact.position_size()
            << ",\"max_depth_m\":" << maxDepth
            << ",\"geometry_source\":\"actual gz.msgs.Contact fields; absent fields remain empty\""
            << ",\"positions_world\":[";
        for (int i=0; i<contact.position_size(); ++i) {
          if (i) log << ',';
          MessageVector(log, contact.position(i));
        }
        log << "],\"normals_world\":[";
        for (int i=0; i<contact.normal_size(); ++i) {
          if (i) log << ',';
          MessageVector(log, contact.normal(i));
        }
        log << "],\"depths_m\":[";
        for (int i=0; i<contact.depth_size(); ++i) { if (i) log << ','; log << contact.depth(i); }
        log << "],\"wrenches_raw\":[";
        for (int i=0; i<contact.wrench_size(); ++i) {
          if (i) log << ',';
          const auto &w=contact.wrench(i);
          log << "{\"body_1_name\":" << Quote(w.body_1_name()) << ",\"body_2_name\":" << Quote(w.body_2_name());
          auto rawWrench=[&](const auto &value) {
            log << "{\"force\":";
            if (value.has_force()) MessageVector(log,value.force()); else log << "null";
            log << ",\"torque\":";
            if (value.has_torque()) MessageVector(log,value.torque()); else log << "null";
            log << '}';
          };
          log << ",\"body_1_wrench\":";
          if (w.has_body_1_wrench()) rawWrench(w.body_1_wrench()); else log << "null";
          log << ",\"body_2_wrench\":";
          if (w.has_body_2_wrench()) rawWrench(w.body_2_wrench()); else log << "null";
          log << '}';
        }
        log << "],\"wrench_frame_scope\":\"raw message body wrench fields; no inferred vertical load measurement\"}";
      }
    }
    log << "]}\n";

    if(iteration%4==0) {
      const std::string temporary=statePath+".tmp";std::ofstream state(temporary,std::ios::trunc);
      state<<std::setprecision(17)<<"{\"schema\":1,\"state_phase\":\"PostUpdate\",\"state_time_offset_s\":0,\"world_sim_time\":"<<time<<",\"position\":";
      Vector(state,pose->Pos());state<<",\"quaternion_wxyz\":["<<pose->Rot().W()<<','<<pose->Rot().X()<<','<<pose->Rot().Y()<<','<<pose->Rot().Z()<<']';
      state<<",\"body_lin_vel_com\":";Vector(state,pose->Rot().RotateVectorReverse(*com));state<<",\"body_ang_vel\":";Vector(state,pose->Rot().RotateVectorReverse(*omega));
      state<<",\"q\":";Array(state,q);state<<",\"qd\":";Array(state,qd);state<<",\"tau\":";Array(state,tau);state<<",\"tau_available\":";Array(state,tauValid);state<<",\"contacts\":";Array(state,contactCounts);state<<"}";
      state.flush();if(!state)throw std::runtime_error("Native snapshot writer failed");state.close();if(std::rename(temporary.c_str(),statePath.c_str())!=0)throw std::runtime_error("Native snapshot atomic replace failed");
    }
    const bool done=static_cast<bool>(std::ifstream(donePath));
    if(iteration%200==0||done)log.flush();
    if(!log)throw std::runtime_error("CHAMP observer evidence writer failed");
    // The separate command monitor owns completion. A bounded timeout fails closed.
    if(done||time>=endTime+2) {
      log<<"{\"kind\":\"observer_stop\",\"t\":"<<time<<",\"monitor_done_file\":"<<(done?"true":"false")<<",\"duration_watchdog\":"<<(!done?"true":"false")<<"}\n";log.flush();
      eventManager->Emit<gz::sim::events::Stop>();
    }
  }
 private:
  std::array<std::string,12> names={"rf_hip_joint","rf_upper_leg_joint","rf_lower_leg_joint","lf_hip_joint","lf_upper_leg_joint","lf_lower_leg_joint","rh_hip_joint","rh_upper_leg_joint","rh_lower_leg_joint","lh_hip_joint","lh_upper_leg_joint","lh_lower_leg_joint"};
  std::array<gz::sim::Joint,12> joints;
  std::array<std::vector<gz::sim::Entity>,5> contactEntities;
  gz::sim::Link base;gz::sim::Entity modelEntity=gz::sim::kNullEntity;
  gz::math::Vector3d comOffset;gz::sim::EventManager *eventManager=nullptr;
  std::ofstream log;std::string donePath,statePath;double endTime=120.;bool ownershipChecked=false;uint64_t iteration=0;
};
}
GZ_ADD_PLUGIN(champ_compare::NativeObserver,gz::sim::System,champ_compare::NativeObserver::ISystemConfigure,champ_compare::NativeObserver::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(champ_compare::NativeObserver,"champ_compare::NativeObserver")

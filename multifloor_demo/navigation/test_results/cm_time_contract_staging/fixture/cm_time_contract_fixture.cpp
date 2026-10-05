// Test-only controlled CM input. No command/state interfaces, Gazebo or GT.
#include <controller_manager/controller_manager.hpp>
#include <controller_interface/controller_interface.hpp>
#include <rcl/time.h>
#include <dlfcn.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using controller_interface::return_type;
struct Sample { int64_t time_ns; int64_t period_ns; };
class Probe : public controller_interface::ControllerInterface
{
public:
  std::vector<Sample> samples;
  controller_interface::CallbackReturn on_init() override
  { return controller_interface::CallbackReturn::SUCCESS; }
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  { return {controller_interface::interface_configuration_type::NONE,{}}; }
  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  { return {controller_interface::interface_configuration_type::NONE,{}}; }
  return_type update(const rclcpp::Time & time,const rclcpp::Duration & period) override
  {
    samples.push_back({time.nanoseconds(),period.nanoseconds()});
    if(period.nanoseconds()<0) throw std::runtime_error("Test probe rejects negative period; no hidden reset");
    return return_type::OK;
  }
};

static const char * urdf=R"(<robot name="cm_time_contract">
<link name="base"/><link name="tip"/>
<joint name="test_joint" type="revolute"><parent link="base"/><child link="tip"/>
<axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="10" velocity="10"/></joint>
<ros2_control name="fixture_hardware" type="system"><hardware><plugin>mock_components/GenericSystem</plugin></hardware>
<joint name="test_joint"><command_interface name="effort"/><state_interface name="position"/>
<state_interface name="velocity"/></joint></ros2_control></robot>)";

struct Rig
{
  std::shared_ptr<rclcpp::Executor> executor;
  std::shared_ptr<controller_manager::ControllerManager> manager;
  std::shared_ptr<Probe> probe;
  Rig(bool sim,unsigned rate,std::string name)
  {
    executor=std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto options=controller_manager::get_cm_node_options();
    options.parameter_overrides({rclcpp::Parameter("use_sim_time",sim),rclcpp::Parameter("update_rate",250)});
    manager=std::make_shared<controller_manager::ControllerManager>(executor,urdf,true,name,"",options);
    probe=std::make_shared<Probe>();
    // Match load_controller's clock type; the generic convenience template defaults to SYSTEM.
    controller_manager::ControllerSpec spec;
    spec.c=probe;spec.info.name="probe";spec.info.type="test_only/Probe";
    spec.last_update_cycle_time=std::make_shared<rclcpp::Time>(0,0,manager->get_trigger_clock()->get_clock_type());
    if(!manager->add_controller(spec)) throw std::runtime_error("add_controller failed");
    probe->get_node()->set_parameter(rclcpp::Parameter("update_rate",static_cast<int>(rate)));
    if(manager->configure_controller("probe")!=return_type::OK) throw std::runtime_error("configure failed");
    probe->get_node()->activate();
    if(probe->get_lifecycle_state().label()!="active") throw std::runtime_error("Probe was not active");
  }
  void clock(int64_t ns)
  {
    auto ptr=manager->get_clock()->get_clock_handle();
    if(rcl_enable_ros_time_override(ptr)!=RCL_RET_OK || rcl_set_ros_time_override(ptr,ns)!=RCL_RET_OK)
      throw std::runtime_error("Clock override failed");
    if(!manager->get_clock()->started()) throw std::runtime_error("Controlled ROS clock not started");
  }
  return_type tick(int64_t ns)
  { return manager->update(rclcpp::Time(ns,RCL_ROS_TIME),rclcpp::Duration::from_nanoseconds(4000000)); }
  ~Rig() { if(manager) manager->shutdown_controllers(); }
};

int main(int argc,char **argv)
{
  bool candidate=argc>1 && std::string(argv[1])=="candidate";
  rclcpp::init(argc,argv);
  int failures=0;
  auto check=[&](std::string name,bool ok){std::cout<<"CHECK "<<name<<" "<<(ok?"PASS":"FAIL")<<"\n";if(!ok)++failures;};
  try
  {
    Dl_info info{};
    dladdr(reinterpret_cast<void*>(&controller_manager::get_cm_node_options),&info);
    std::cout<<"LOADED_CM "<<(info.dli_fname?info.dli_fname:"unknown")<<"\n";
    constexpr int64_t base=16456000000LL;
    {
      Rig r(true,250,"cm_pause_250");r.clock(base);
      for(int i=0;i<10;i++)r.tick(base+i*4000000LL);
      check("250_actual_callback_count",r.probe->samples.size()==10);
      check("250_first_period_4ms",r.probe->samples[0].period_ns==4000000);
      for(size_t i=0;i<r.probe->samples.size();i++)
        std::cout<<"SAMPLE pause250 "<<i<<" "<<r.probe->samples[i].time_ns<<" "<<r.probe->samples[i].period_ns<<"\n";
      bool periods=true,times=true;
      for(size_t i=1;i<r.probe->samples.size();i++)
      {
        periods&=r.probe->samples[i].period_ns==(candidate?4000000:0);
        times&=r.probe->samples[i].time_ns==(candidate?base+i*4000000LL:base);
      }
      check("250_pause_period_contract",periods);check("250_pause_time_contract",times);
      r.clock(base+50000000);r.tick(base+40000000);
      check("250_clock_resume_period",r.probe->samples.back().period_ns==(candidate?4000000:50000000));
      r.tick(base+40000000);
      check("repeated_physical_input_not_hidden",r.probe->samples.back().period_ns==0);
    }
    {
      Rig r(true,100,"cm_gating_100");
      for(int i=0;i<31;i++){int64_t t=base+i*4000000LL;r.clock(t);r.tick(t);}
      size_t healthy=r.probe->samples.size();
      check("100_healthy_gating_not_250",healthy>=10 && healthy<=15);
      r.clock(base+120000000);
      for(int i=31;i<61;i++)r.tick(base+i*4000000LL);
      size_t paused=r.probe->samples.size()-healthy;
      check("100_pause_keeps_rate_gating",candidate?(paused>=10&&paused<=15):(paused==0));
      for(size_t i=0;i<r.probe->samples.size();i++)
        std::cout<<"SAMPLE gating100 "<<i<<" "<<r.probe->samples[i].time_ns<<" "<<r.probe->samples[i].period_ns<<"\n";
    }
    {
      Rig r(true,250,"cm_ros_backwards");r.clock(base);r.tick(base);
      r.clock(base-4000000);auto ret=r.tick(base+4000000);
      check("ROS_reset_not_physical_reset",candidate?
        (ret==return_type::OK && r.probe->samples.back().period_ns==4000000):
        (ret==return_type::ERROR && r.probe->samples.back().period_ns==-4000000));
    }
    {
      Rig r(true,250,"cm_physical_backwards");r.clock(base);r.tick(base);
      r.clock(base-4000000);auto ret=r.tick(base-4000000);
      check("physical_backwards_not_silently_reset",ret==return_type::ERROR && r.probe->samples.back().period_ns==-4000000);
    }
    {
      Rig r(false,250,"cm_nonsim");
      r.tick(base);std::this_thread::sleep_for(std::chrono::milliseconds(4));r.tick(base+4000000);
      check("non_sim_unchanged_positive_period",r.probe->samples.size()==2 && r.probe->samples.back().period_ns>0);
      check("non_sim_time_not_synthetic_input",r.probe->samples.back().time_ns>1000000000000000LL);
    }
  }
  catch(const std::exception &e){std::cerr<<"FIXTURE_EXCEPTION "<<e.what()<<"\n";++failures;}
  rclcpp::shutdown();
  std::cout<<"RESULT "<<(failures==0?"PASS":"FAIL")<<" "<<failures<<"\n";
  return failures?1:0;
}

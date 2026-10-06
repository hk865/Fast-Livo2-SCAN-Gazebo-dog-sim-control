// Excluded actual installed JTC contract. No Gazebo, robot, GT or motion topic.
// Controlled memory is the hardware state/effort interface; actual ROS topic
// supplies trajectories. Only interpolate_from_desired_state differs.
#include <joint_trajectory_controller/joint_trajectory_controller.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <controller_interface/controller_interface_params.hpp>
#include <rcl/time.h>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <cmath>
#include <dlfcn.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

using JTC=joint_trajectory_controller::JointTrajectoryController;
using Point=trajectory_msgs::msg::JointTrajectoryPoint;
using Msg=trajectory_msgs::msg::JointTrajectory;
using controller_interface::return_type;
class Access : public JTC
{
public:
  const Point & actual() const {return state_current_;}
  const Point & desired() const {return state_desired_;}
  const Point & next() const {return command_next_;}
  const Point & error() const {return state_error_;}
  const Point & last() const {return last_commanded_state_;}
  bool closed_loop() const {return use_closed_loop_pid_adapter_;}
  Msg::SharedPtr pending()
  {return *new_trajectory_msg_.readFromRT();}
  bool received(const Msg::SharedPtr & msg,const Msg::SharedPtr & before)
  {
    auto p=new_trajectory_msg_.readFromRT();
    return *p && *p!=before && (*p)->points.back().time_from_start==msg->points.back().time_from_start
      && (*p)->points.back().positions==msg->points.back().positions;
  }
};
const std::vector<std::string> names={"lf_hip_joint","lf_upper_leg_joint","lf_lower_leg_joint",
  "rf_hip_joint","rf_upper_leg_joint","rf_lower_leg_joint","lh_hip_joint","lh_upper_leg_joint",
  "lh_lower_leg_joint","rh_hip_joint","rh_upper_leg_joint","rh_lower_leg_joint"};
const std::vector<double> nominal={0.,.7894648909568787,-1.5789296627044678,
   0.,.7894648909568787,-1.5789296627044678,0.,.7894648909568787,-1.5789296627044678,
   0.,.7894648909568787,-1.5789296627044678};
std::string urdf()
{
  std::string s="<robot name='controlled_jtc_fixture'><link name='base'/>";
  for(size_t i=0;i<names.size();++i)
    s+="<link name='tip"+std::to_string(i)+"'/><joint name='"+names[i]+"' type='revolute'>"
       "<parent link='base'/><child link='tip"+std::to_string(i)+"'/><axis xyz='0 1 0'/>"
       "<limit lower='-10' upper='10' effort='1000' velocity='1000'/></joint>";
  return s+"</robot>";
}
void array(std::ostream &o,const std::vector<double>&v)
{o<<"[";for(size_t i=0;i<v.size();++i){if(i)o<<",";o<<v[i];}o<<"]";}
struct Rig
{
  bool flag;
  std::shared_ptr<Access> jtc;
  std::shared_ptr<rclcpp::Node> publisher_node;
  rclcpp::Publisher<Msg>::SharedPtr publisher;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::vector<double> q=nominal,v=std::vector<double>(12,0.),effort=std::vector<double>(12,0.);
  std::vector<double> last_producer_target=nominal;
  std::vector<hardware_interface::CommandInterface::SharedPtr> commands;
  std::vector<hardware_interface::StateInterface::SharedPtr> states;
  int64_t time_ns=16000000000LL;
  int64_t last_returned_period_ns=-1;
  bool last_returned_success=false;
  std::ofstream out;
  explicit Rig(bool desired,const std::string &params,const std::string &output):flag(desired),out(output)
  {
    for(size_t i=0;i<12;++i){q[i]+=.015*(i%3-1.);v[i]=.02*(i%2?1.:-1.);effort[i]=.3*(i%2?1.:-1.);}
    jtc=std::make_shared<Access>();
    auto options=jtc->define_custom_node_options();
    options.arguments({"--ros-args","--params-file",params});
    options.parameter_overrides({rclcpp::Parameter("use_sim_time",true),
       rclcpp::Parameter("interpolate_from_desired_state",flag),rclcpp::Parameter("update_rate",250)});
    controller_interface::ControllerInterfaceParams ci;
    ci.controller_name=flag?"jtc_fixture_true":"jtc_fixture_false";
    ci.robot_description=urdf();ci.update_rate=250;ci.controller_manager_update_rate=250;ci.node_options=options;
    if(jtc->init(ci)!=return_type::OK)throw std::runtime_error("Actual controller init failed");
    clock();
    if(jtc->configure().label()!="inactive")throw std::runtime_error("Actual configure failed");
    std::vector<hardware_interface::LoanedCommandInterface> c;
    std::vector<hardware_interface::LoanedStateInterface> s;
    for(size_t i=0;i<12;++i)
    {
      commands.push_back(std::make_shared<hardware_interface::CommandInterface>(names[i],"effort",&effort[i]));
      c.emplace_back(commands.back(),nullptr);
      states.push_back(std::make_shared<hardware_interface::StateInterface>(names[i],"position",&q[i]));s.emplace_back(states.back());
      states.push_back(std::make_shared<hardware_interface::StateInterface>(names[i],"velocity",&v[i]));s.emplace_back(states.back());
    }
    jtc->assign_interfaces(std::move(c),std::move(s));
    if(jtc->get_node()->activate().label()!="active")throw std::runtime_error("Actual activate failed");
    publisher_node=std::make_shared<rclcpp::Node>("controlled_reference_publisher");
    publisher=publisher_node->create_publisher<Msg>("/"+ci.controller_name+"/joint_trajectory",10);
    executor.add_node(jtc->get_node()->get_node_base_interface());executor.add_node(publisher_node);
    auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
    while(publisher->get_subscription_count()!=1 && std::chrono::steady_clock::now()<deadline)
      {executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));}
    if(publisher->get_subscription_count()!=1)throw std::runtime_error("Actual JTC DDS subscription not discovered");
    auto client=publisher_node->create_client<rcl_interfaces::srv::GetParameters>("/"+ci.controller_name+"/get_parameters");
    if(!client->wait_for_service(std::chrono::seconds(2)))throw std::runtime_error("Actual parameter service missing");
    auto request=std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names={"interpolate_from_desired_state","open_loop_control"};
    auto response=client->async_send_request(request);
    if(executor.spin_until_future_complete(response,std::chrono::seconds(2))!=rclcpp::FutureReturnCode::SUCCESS)
      throw std::runtime_error("Actual parameter service timed out");
    auto values=response.get()->values;
    if(values.size()!=2 || values[0].type!=rcl_interfaces::msg::ParameterType::PARAMETER_BOOL
       || values[0].bool_value!=flag || values[1].type!=rcl_interfaces::msg::ParameterType::PARAMETER_BOOL
       || values[1].bool_value)throw std::runtime_error("Actual selected BOOL/open-loop service contract differs");
    out<<std::setprecision(17);
    out<<"{\"kind\":\"configuration\",\"interpolate_from_desired_state\":"<<(flag?"true":"false")
       <<",\"actual_parameter\":"<<(jtc->get_node()->get_parameter("interpolate_from_desired_state").as_bool()?"true":"false")
       <<",\"open_loop_control\":"<<(jtc->get_node()->get_parameter("open_loop_control").as_bool()?"true":"false")
       <<",\"closed_loop_effort_pid\":"<<(jtc->closed_loop()?"true":"false")
       <<",\"actual_command_subscriptions\":"<<publisher->get_subscription_count()<<",\"is_async\":"<<(jtc->is_async()?"true":"false")<<"}\n";
    out<<"{\"kind\":\"parameter_service\",\"flag_type\":"<<int(values[0].type)
       <<",\"flag_bool\":"<<(values[0].bool_value?"true":"false")
       <<",\"open_loop_bool\":"<<(values[1].bool_value?"true":"false")
       <<",\"p\":"<<jtc->get_node()->get_parameter("gains.lf_lower_leg_joint.p").as_double()
       <<",\"i\":"<<jtc->get_node()->get_parameter("gains.lf_lower_leg_joint.i").as_double()
       <<",\"d\":"<<jtc->get_node()->get_parameter("gains.lf_lower_leg_joint.d").as_double()<<"}\n";
    sample("activation",0);
  }
  void clock()
  {
    auto c=jtc->get_node()->get_clock()->get_clock_handle();
    if(rcl_enable_ros_time_override(c)!=RCL_RET_OK || rcl_set_ros_time_override(c,time_ns)!=RCL_RET_OK)
      throw std::runtime_error("Clock override failed");
  }
  void sample(const std::string&stage,int k)
  {
    out<<"{\"kind\":\"sample\",\"stage\":\""<<stage<<"\",\"step\":"<<k<<",\"time_ns\":"<<time_ns
       <<",\"controlled_input_period_ns\":4000000,\"actual_returned_period_ns\":"<<last_returned_period_ns
       <<",\"actual_returned_success\":"<<(last_returned_success?"true":"false")<<",\"hardware_position\":";array(out,q);
    out<<",\"hardware_velocity\":";array(out,v);out<<",\"output_effort\":";array(out,effort);
    out<<",\"actual_position\":";array(out,jtc->actual().positions);out<<",\"actual_velocity\":";array(out,jtc->actual().velocities);
    out<<",\"actual_effort_from_command_interface\":";array(out,jtc->actual().effort);
    out<<",\"reference_position\":";array(out,jtc->desired().positions);out<<",\"reference_velocity\":";array(out,jtc->desired().velocities);
    out<<",\"next_position\":";array(out,jtc->next().positions);out<<",\"next_velocity\":";array(out,jtc->next().velocities);
    out<<",\"next_effort\":";array(out,jtc->next().effort);out<<",\"error_position\":";array(out,jtc->error().positions);
    out<<",\"error_velocity\":";array(out,jtc->error().velocities);out<<",\"last_commanded_position\":";array(out,jtc->last().positions);
    out<<",\"last_commanded_velocity\":";array(out,jtc->last().velocities);out<<"}\n";
  }
  void reference(const std::vector<double>&target,int64_t horizon_ns=16666666)
  {
    // Retain the old pointer until delivery: repeated identical target content
    // alone cannot prove a new subscription callback, nor may address reuse.
    const auto before=jtc->pending();
    auto msg=std::make_shared<Msg>();msg->joint_names=names;msg->points.resize(1);
    msg->points[0].positions=target;
    msg->points[0].time_from_start.sec=horizon_ns/1000000000LL;
    msg->points[0].time_from_start.nanosec=horizon_ns%1000000000LL;
    publisher->publish(*msg);
    auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while(!jtc->received(msg,before) && std::chrono::steady_clock::now()<deadline)
      {executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));}
    if(!jtc->received(msg,before))throw std::runtime_error("Actual new DDS trajectory callback not observed");
    last_producer_target=target;
    out<<"{\"kind\":\"actual_input\",\"time_ns\":"<<time_ns<<",\"header_ns\":0,\"horizon_ns\":"<<horizon_ns
       <<",\"new_callback_observed\":true,\"target\":";array(out,target);out<<"}\n";
  }
  void tick(const std::string&stage,int k)
  {
    time_ns+=4000000;clock();
    const auto status=jtc->trigger_update(rclcpp::Time(time_ns,RCL_ROS_TIME),rclcpp::Duration::from_nanoseconds(4000000));
    last_returned_period_ns=status.period?status.period->nanoseconds():-1;
    last_returned_success=status.successful && status.result==return_type::OK;
    if(!last_returned_success)throw std::runtime_error("Actual trigger_update error");
    sample(stage,k);executor.spin_some();
  }
  void feedback_step()
  {
    // Declared deterministic toy actuator: real effort output feeds measured
    // states next update. It is not a Go2 plant or a no-slip assumption.
    for(size_t i=0;i<12;++i){double a=effort[i]/.3-3.*v[i];v[i]+=.004*a;q[i]+=.004*v[i];}
  }
  ~Rig()
  {
    if(jtc){jtc->get_node()->deactivate();jtc->release_interfaces();jtc->get_node()->cleanup();}
  }
};
int main(int argc,char**argv)
{
  if(argc<4){std::cerr<<"parameters output flag required\n";return 2;}
  const std::string params=argv[1],output=argv[2];bool flag=std::string(argv[3])=="1";
  rclcpp::init(argc,argv);int failure=0;
  try
  {
    Rig r(flag,params,output);
    {std::ifstream maps("/proc/self/maps");std::ofstream saved(output+".maps");saved<<maps.rdbuf();}
    r.reference(nominal);r.tick("first_reference",0);
    // Controlled measured drift with repeated fixed reference. Each cycle
    // reads new measured state, distinct from last commanded reference.
    for(int k=1;k<=30;++k)
    {
      for(size_t i=0;i<12;++i){r.q[i]+=.001*(i%2?1.:-1.);r.v[i]=.25*(i%2?1.:-1.);}
      r.reference(nominal);r.tick("constant_target_measured_drift",k);
    }
    for(int k=0;k<150;++k)
    {
      r.feedback_step();auto target=nominal;
      for(size_t i=0;i<12;++i)target[i]+=.035*std::sin(k*.024+(i%3));
      r.reference(target);r.tick("closed_loop_moving_target",k);
    }
    auto start=r.last_producer_target;
    // Same position-only producer stream and producer q0 in both trials.
    // Quintic smoothstep is the original StopReturn geometry; duration is a
    // declared .3s test interval, not the actual Adapter's ACK protocol.
    for(int k=1;k<=75;++k)
    {
      r.feedback_step();auto target=nominal;double a=k/75.;double blend=10*a*a*a-15*a*a*a*a+6*a*a*a*a*a;
      for(size_t i=0;i<12;++i)target[i]=start[i]+blend*(nominal[i]-start[i]);
      r.reference(target);r.tick("nominal_stop_return",k);
    }
    r.reference(nominal);
    for(int k=0;k<25;++k){r.feedback_step();r.tick("idle_constant_nominal",k);}
    for(int k=0;k<12;++k)
    {
      r.feedback_step();auto target=nominal;
      for(size_t i=0;i<12;++i)target[i]+=.025*std::sin(k*.024+(i%3));
      r.reference(target);r.tick("rapid_restart_after_nominal",k);
    }
  }
  catch(const std::exception&e){std::cerr<<"FIXTURE_EXCEPTION "<<e.what()<<"\n";failure=1;}
  rclcpp::shutdown();return failure;
}

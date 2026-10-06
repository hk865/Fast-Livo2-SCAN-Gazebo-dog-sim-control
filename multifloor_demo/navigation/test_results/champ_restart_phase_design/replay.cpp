#include <body_controller/body_controller.h>
#include <leg_controller/leg_controller.h>
#include <kinematics/kinematics.h>
#include "geometry_input.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

// Offline calls only. No ROS, publishers, Gazebo or production artifact writes.
struct Input { unsigned long us; float vx, vy, wz; };
std::vector<Input> read_inputs(const char *path) {
  std::ifstream file(path); std::string line; std::vector<Input> inputs;
  while (std::getline(file, line)) {
    std::istringstream stream(line); Input x{};
    if (stream >> x.us >> x.vx >> x.vy >> x.wz) inputs.push_back(x);
  }
  if (inputs.empty()) throw std::runtime_error("No input rows");
  return inputs;
}
void emit_array(const float *values, size_t n) {
  std::cout << '[';
  for (size_t i=0; i<n; ++i) { if(i) std::cout << ','; std::cout << values[i]; }
  std::cout << ']';
}
int main(int argc, char **argv) {
  if (argc != 6) return 2;
  auto inputs=read_inputs(argv[1]);
  unsigned long onset=std::stoul(argv[2]), step=std::stoul(argv[3]), length=std::stoul(argv[4]);
  bool warm=std::stoi(argv[5]);
  champ::GaitConfig config(">>", false, .9f, .3f, .25f, .5f, 0, .07f, .005f, .35f, .30f);
  champ::QuadrupedBase base(config);
  for (size_t i=0; i<4; ++i) for(size_t j=0; j<4; ++j)
    base.legs[i]->joint_chain[j]->setTranslation(origins[i][j][0], origins[i][j][1], origins[i][j][2]);
  champ::BodyController body(base);
  champ::LegController legs(base, 0ul);
  champ::Kinematics ik(base);
  champ::Pose pose; pose.position.z=config.nominal_height;
  float q[12]{};
  geometry::Transformation feet[4];
  auto tick=[&](unsigned long time, Input x, bool emit) {
    champ::Velocities requested; requested.linear.x=x.vx; requested.linear.y=x.vy; requested.angular.z=x.wz;
    body.poseCommand(feet, pose);
    legs.velocityCommand(feet, requested, time);
    ik.inverse(q, feet);
    if (!emit) return;
    float points[12], steps[4], rotations[4], phase[4];
    float tangent=requested.angular.z*base.lf.center_to_nominal();
    float sx=champ::LegController::raibertHeuristic(config.stance_duration,requested.linear.x);
    float sy=champ::LegController::raibertHeuristic(config.stance_duration,requested.linear.y);
    float st=champ::LegController::raibertHeuristic(config.stance_duration,tangent);
    float theta=sinf((st/2)/base.lf.center_to_nominal())*2;
    for(size_t i=0;i<4;++i) {
      points[3*i]=feet[i].X(); points[3*i+1]=feet[i].Y(); points[3*i+2]=feet[i].Z();
      champ::LegController::transformLeg(steps[i],rotations[i],*base.legs[i],sx,sy,theta);
      phase[i]=base.legs[i]->gait_phase();
    }
    std::cout << std::setprecision(10) << "{\"time_us\":" << time << ",\"elapsed_us\":"
              << static_cast<long long>(time)-static_cast<long long>(onset) << ",\"command\":";
    float cmd[3]={requested.linear.x,requested.linear.y,requested.angular.z}; emit_array(cmd,3);
    std::cout << ",\"stance\":"; emit_array(legs.phase_generator.stance_phase_signal,4);
    std::cout << ",\"swing\":"; emit_array(legs.phase_generator.swing_phase_signal,4);
    std::cout << ",\"gait_phase\":"; emit_array(phase,4);
    std::cout << ",\"foot_hip\":"; emit_array(points,12);
    std::cout << ",\"q\":"; emit_array(q,12);
    std::cout << ",\"step_length\":"; emit_array(steps,4);
    std::cout << ",\"rotation\":"; emit_array(rotations,4);
    std::cout << ",\"has_started\":" << (legs.phase_generator.has_started?"true":"false") << "}\n";
  };
  if(warm) {
    unsigned long first=onset>=1000000ul?onset-1000000ul:50000ul;
    tick(first, {first,.03f,0,.06f}, false);
    tick(onset>=1000000ul?onset-1000ul:100000ul, {0,0,0,0}, true);
  } else tick(onset-1000ul,{0,0,0,0},true);
  size_t current=0;
  for (unsigned long t=onset;t<=onset+length;t+=step) {
    while(current+1<inputs.size() && inputs[current+1].us<=t) ++current;
    tick(t,inputs[current],true);
  }
}

// Independent audit: execute the production PolynomialTraj header, no planner/robot mutation.
#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>
#include <traj_utils/polynomial_traj.h>
int main(){
  Eigen::Vector3d start(.304,-.152,-.00305),end(-.05343189813310946,1.9996617509652172,-.001359314385753837);
  std::string name;Eigen::Vector3d v,a;double factor,speed;
  std::cout<<std::setprecision(17);
  while(std::cin>>name>>v.x()>>v.y()>>v.z()>>a.x()>>a.y()>>a.z()>>factor>>speed){
    double T=factor*(end-start).norm()/speed;
    auto p=PolynomialTraj::one_segment_traj_gen(start,v,a,end,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),T);p.init();
    double max_side=0,max_x=start.x(),min_x=start.x(),max_v=0,max_a=0,length=0;
    Eigen::Vector3d previous=start;auto direction=(end-start).normalized();
    for(int i=0;i<=5000;++i){double t=T*i/5000;auto q=p.evaluate(t);auto offset=q-start;
      max_side=std::max(max_side,(offset-direction*offset.dot(direction)).norm());
      max_x=std::max(max_x,q.x());min_x=std::min(min_x,q.x());max_v=std::max(max_v,p.evaluateVel(t).norm());max_a=std::max(max_a,p.evaluateAcc(t).norm());
      length+=(q-previous).norm();previous=q;
    }
    // Same pre-occupancy local-target stepping as current getLocalTarget.
    Eigen::Vector3d local=end;double local_t=T;
    for(double t=0;t<T;t+=2.5/20/.30){auto q=p.evaluate(t);if((q-start).norm()>=2.5){local=q;local_t=t;break;}}
    std::cout<<name<<" "<<T<<" "<<max_side<<" "<<min_x<<" "<<max_x<<" "<<max_v<<" "<<max_a<<" "<<length<<" "<<local_t<<" "<<local.transpose()<<"\n";
  }
}

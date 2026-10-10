#include "aims_mpcc_rt/execution.hpp"
#include <iostream>
#include <stdexcept>
using namespace aims_mpcc_rt;
static void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
int main(int argc,char** argv){
  require(argc==2,"independent acceleration bundle required");
  auto bundle=Bundle::load(argv[1]);const auto c=bundle.config();Core core(bundle);
  require(!c.combined_accel_constraint_enabled,"test requires ellipse removed");
  const auto r=bundle.reference().at(0.);const double now=10.;
  State initial{r.x,r.y,r.yaw,1.2,0.,.3};Applied applied{0.,.3,0.};
  Plan p;p.success=true;p.status=0;p.dt=c.dt;p.source_epoch=p.forecast_epoch=now;
  p.artifact_fingerprint=bundle.fingerprint();p.initial_applied=applied;p.states.push_back(initial);
  for(int k=0;k<c.horizon;++k){p.controls.push_back({0.,.3,1.2});
    p.states.push_back(core.transition(p.states.back(),p.controls.back(),.3,c.dt));}
  auto candidate=core.reanchor(p,initial,applied,now,Alignment{});
  require(candidate.success,"independent feasible candidate rejected by remaining ellipse");
  require(!candidate.constraint_violations.count("operating_envelope")&&
    !candidate.constraint_violations.count("terminal_operating_envelope"),"generated ellipse remains");
  OutputSampler sampler(c);sampler.reset(1.2,.3,now-.02);
  auto certified=certify_execution(candidate,sampler,initial,Alignment{},c,bundle.reference(),now,1.2);
  require(certified.success&&certified.max_utilization>1.1,"execution still gates on ellipse");
  auto old=c;old.combined_accel_constraint_enabled=true;
  OutputSampler strict(old);strict.reset(1.2,.3,now-.02);
  require(!certify_execution(candidate,strict,initial,Alignment{},old,bundle.reference(),now,1.2).success,
    "existing enabled ellipse behavior changed");
  for(int i=0;i<3;++i){auto invalid=p;
    invalid.controls[0][i]=(i==0?c.accel_limit+.1:i==1?c.steer_limit+.1:c.max_speed+.1);
    auto checked=core.reanchor(invalid,initial,applied,now,Alignment{});
    require(checked.original_constraint_violations.at("input_bounds")>.09,
      "original input-bound violation must remain recorded");
    // Existing prefix transport can produce a distinct bounded alternate.
    // It must never accept the invalid controls unchanged.
    if(checked.success)for(const auto& u:checked.controls)
      require(u[0]<=c.accel_limit+1e-8&&u[0]>=-c.brake_limit-1e-8&&
        std::abs(u[1])<=c.steer_limit+1e-8&&u[2]<=c.max_speed+1e-8,
        "transported alternate bypassed independent hard bounds");}
  auto excessive=initial;excessive[3]=c.max_speed+.1;
  require(!certify_execution(candidate,sampler,excessive,Alignment{},c,bundle.reference(),now,1.2).success,
    "physical speed bound removed");
  auto stop=sampler.sample(nullptr,now,initial,true,1.2);
  require(stop.continuous_acceleration<0.&&stop.continuous_speed<1.2,
    "stop must retain independent braking even above old ellipse");
  require(!sampler.braking_budget().available,"disabled ellipse still supplies a braking budget");
  OutputSampler capped(c);capped.reset(1.2,.3,now-.02);
  auto cap=capped.sample(&candidate,now,initial,false,1.2,.9);
  require(cap.continuous_acceleration<0.&&cap.continuous_acceleration>=-c.brake_limit,
    "speed cap must retain independent braking bounds");
  std::cout<<"independent acceleration candidate, execution, stopping and retained bounds passed\n";
}

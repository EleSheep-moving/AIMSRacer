#include "aims_mpcc_rt/core.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace aims_mpcc_rt;
void require(bool value,const char *message){if(!value)throw std::runtime_error(message);}
int main(int argc,char **argv) {
  require(argc==2,"bundle argument required");
  auto bundle=Bundle::load(argv[1]);Core core(bundle);auto r=bundle.reference().at(0.);
  State start{r.x,r.y,r.yaw,.4,0.,0.};
  Plan original;original.success=true;original.status=0;original.native_passes=1;original.dt=bundle.config().dt;
  original.source_epoch=10.;original.forecast_epoch=10.02;
  original.artifact_fingerprint=bundle.fingerprint();
  original.controls.assign(bundle.config().horizon,Control{0.,0.,.4});
  // During delayed activation the old output continues turning the actuator.
  Applied old_prefix{0.,.005,0.};Control old_control{0.,.005,.4};
  auto forecast=core.transition(start,old_control,old_prefix[1],.02);
  forecast[4]=bundle.reference().project(forecast[0],forecast[1]);
  original.states.push_back(forecast);
  for(const auto &u:original.controls)original.states.push_back(core.transition(original.states.back(),u,0.,bundle.config().dt));
  auto actual=core.transition(forecast,old_control,old_prefix[1],.03);
  actual[4]=bundle.reference().project(actual[0],actual[1]);
  require(actual[5]>start[5],"old applied prefix changes actual actuator state");
  auto anchored=core.reanchor(original,actual,old_prefix,10.05);
  require(anchored.success,"physically consistent delayed handover accepted");
  require(anchored.controls==original.controls,"unexecuted first control retained");
  for(int i=0;i<6;++i)require(std::abs(anchored.states.front()[i]-actual[i])<1e-7,"actual state is the reanchored initial condition");
  require(anchored.forecast_epoch==10.05,"actual activation epoch anchors the plan");
  require(anchored.source_epoch==original.source_epoch,"reanchor preserves original source and TTL epoch");
  require(anchored.native_passes==original.native_passes,"reanchor adds no optimizer passes");
  auto mismatched=original;mismatched.artifact_fingerprint="wrong_bundle";
  require(!core.reanchor(mismatched,actual,old_prefix,10.05).success,"different solver artifact cannot reanchor this plan");
  auto jerk=core.reanchor(original,actual,Applied{.3,.005,0.},10.05);
  require(!jerk.success,"violated first acceleration jerk rejects unchanged controls");
  require(jerk.controls==original.controls,"rejected jerk candidate controls are not clipped");
  auto rate=core.reanchor(original,actual,Applied{0.,.05,0.},10.05);
  require(!rate.success,"violated first steering rate acceleration rejects unchanged controls");
  require(rate.controls==original.controls,"rejected steering candidate controls are not clipped");
  std::cout<<"PASS: delayed real-prefix handover reanchors original controls, preserves TTL, rejects invalid jerk/steering prefix\n";
}

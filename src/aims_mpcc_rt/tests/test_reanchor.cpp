#include "aims_mpcc_rt/core.hpp"
#include <algorithm>
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
  require(!anchored.prefix_transported&&anchored.reason=="accepted","already feasible candidate is not transported");
  for(int i=0;i<6;++i)require(std::abs(anchored.states.front()[i]-actual[i])<1e-7,"actual state is the reanchored initial condition");
  require(anchored.forecast_epoch==10.05,"actual activation epoch anchors the plan");
  require(anchored.source_epoch==original.source_epoch,"reanchor preserves original source and TTL epoch");
  require(anchored.native_passes==original.native_passes,"reanchor adds no optimizer passes");
  auto previously_transported=original;previously_transported.prefix_transported=true;
  require(!core.reanchor(previously_transported,actual,old_prefix,10.05).prefix_transported,"each reanchor reports transport in this invocation only");
  auto mismatched=original;mismatched.artifact_fingerprint="wrong_bundle";
  require(!core.reanchor(mismatched,actual,old_prefix,10.05).success,"different solver artifact cannot reanchor this plan");
  if(bundle.config().command_profile=="rate_bounded_v2"){
    auto free_prefix=core.reanchor(original,actual,Applied{.5,.005,5.},10.05);
    require(free_prefix.success&&!free_prefix.prefix_transported,"removed jerk/angular-acceleration limits cannot reject unchanged feasible controls");
    require(free_prefix.controls==original.controls,"v2 tries unchanged controls first");
    auto retained_rate=core.reanchor(original,actual,Applied{0.,.4,0.},10.10);
    require(!retained_rate.success,"retained steering rate rejects excessive endpoint ramp");
    auto unsafe=actual;unsafe[3]=bundle.config().max_speed;unsafe[5]=bundle.config().steer_limit;
    auto envelope=core.reanchor(original,unsafe,Applied{.5,.005,5.},10.05);
    require(!envelope.success&&envelope.max_violation>1e-4,"v2 retains strict physical operating ellipse");
    std::cout<<"PASS: v2 unchanged reanchor ignores removed higher-order bounds and retains steering-rate/envelope checks\n";
    return 0;
  }
  auto jerk=core.reanchor(original,actual,Applied{.3,.005,0.},10.05);
  require(!jerk.success,"violated first acceleration jerk rejects unchanged controls");
  require(jerk.controls==original.controls,"rejected jerk candidate controls are not clipped");
  auto rate=core.reanchor(original,actual,Applied{0.,.2,0.},10.05);
  require(!rate.success,"violated first steering rate acceleration rejects unchanged controls");
  require(rate.controls==original.controls,"rejected steering candidate controls are not clipped");
  // A delayed old endpoint reduces stage-zero rate while the following fixed
  // endpoint keeps stage-one rate unchanged, exceeding its acceleration bound.
  // Rates [d,2d,3d,2d,d,0] form a feasible rise and fall at the hard slew limit.
  auto transported_source=original;transported_source.states={start};
  transported_source.initial_applied={0.,0.,0.};
  double d=bundle.config().steer_acceleration*bundle.config().dt;
  double endpoint=0.;
  for(int k=0;k<bundle.config().horizon;++k){
    double step=k<3?(k+1)*d:k<6?(5-k)*d:0.;
    endpoint+=step*bundle.config().dt;
    transported_source.controls[k]={0.,endpoint,.4};
  }
  auto certified=core.reanchor(transported_source,start,{0.,0.,0.},10.02);
  require(certified.success,"original rise/fall slew schedule is feasible");
  Applied delayed_prefix{0.,.002,.04};
  auto delayed=core.transition(start,{0.,delayed_prefix[1],.4},delayed_prefix[1],.019);
  InternalState check{};std::copy(delayed.begin(),delayed.end(),check.begin());
  std::copy(delayed_prefix.begin(),delayed_prefix.end(),check.begin()+6);
  Parameters check_parameters{};check_parameters[5]=1.;check_parameters[7]=bundle.config().jerk_limit;check_parameters[9]=1.;
  auto first=bundle.evaluate(0,check,std::vector<double>(certified.controls[0].begin(),certified.controls[0].end()),check_parameters);
  std::copy(first.begin(),first.end(),check.begin());
  auto second=bundle.evaluate(1,check,std::vector<double>(certified.controls[1].begin(),certified.controls[1].end()),check_parameters);
  require(second[2]>bundle.config().steer_acceleration*bundle.config().dt+1e-4,"unchanged endpoints demonstrably violate the second-stage slew row");
  auto moved=core.reanchor(certified,delayed,delayed_prefix,10.039);
  require(moved.success,"small delayed prefix transports the second-stage slew schedule");
  require(moved.prefix_transported,"successful alternate candidate reports prefix transport");
  require(moved.reason=="accepted with bounded prefix transport","transport acceptance is explicit");
  require(moved.max_violation<1e-4,"transport retains the original nonlinear tolerance");
  require(moved.controls!=certified.controls,"transport certifies a distinct endpoint candidate");
  auto too_late=core.reanchor(certified,delayed,delayed_prefix,10.071);
  require(!too_late.success&&!too_late.prefix_transported,"transport stops after the 50 ms eligibility window");
  require(too_late.reason.find("prefix transport ineligible")!=std::string::npos,"late transport reports ineligibility");
  auto too_early=core.reanchor(certified,delayed,delayed_prefix,10.019);
  require(!too_early.success&&!too_early.prefix_transported,"negative takeover delay cannot transport");
  auto large_prefix=core.reanchor(certified,delayed,{.3,.2,1.},10.039);
  require(!large_prefix.success&&!large_prefix.prefix_transported,"arbitrary prefix jumps are not transported");
  require(large_prefix.controls==certified.controls,"ineligible transport preserves original rejected controls");
  require(moved.source_epoch==certified.source_epoch,"transport preserves source TTL epoch");
  require(moved.forecast_epoch==10.039,"transport uses the actual takeover epoch");
  require(moved.native_passes==certified.native_passes,"transport does not solve again");
  Applied prefix=delayed_prefix;
  for(size_t k=0;k<moved.controls.size();++k){
    const auto &u=moved.controls[k];
    require(u[2]==certified.controls[k][2],"transport preserves progress order and values");
    require(std::abs(u[0]-prefix[0])<=bundle.config().jerk_limit*bundle.config().dt+1e-12,"hard jerk bound retained");
    require(u[0]>=-bundle.config().brake_limit&&u[0]<=bundle.config().accel_limit,"hard acceleration bounds retained");
    double rate=(u[1]-prefix[1])/bundle.config().dt;
    require(std::abs(rate)<=bundle.config().steer_rate+1e-12,"hard steering rate bound retained");
    require(std::abs(rate-prefix[2])<=bundle.config().steer_acceleration*bundle.config().dt+1e-12,"hard steering acceleration bound retained");
    require(std::abs(u[1])<=bundle.config().steer_limit+1e-12,"hard steering endpoint bound retained");
    prefix={u[0],u[1],rate};
  }
  // A small adverse acceleration-prefix change violates the original first
  // jerk row. Transport its schedule and clamp a later endpoint at the hard
  // braking limit, preserving the original jerk certificate at every interval.
  auto accelerating=original;accelerating.states={start};accelerating.states[0][3]=.8;
  accelerating.initial_applied={0.,0.,0.};
  for(int k=0;k<bundle.config().horizon;++k)
    accelerating.controls[k]={std::min(bundle.config().accel_limit,(k+1)*bundle.config().jerk_limit*bundle.config().dt),0.,.4};
  auto accelerating_original=core.reanchor(accelerating,accelerating.states.front(),accelerating.initial_applied,10.02);
  require(accelerating_original.success,"original bounded acceleration schedule is feasible");
  auto accelerating_moved=core.reanchor(accelerating_original,accelerating.states.front(),{-.01,0.,0.},10.039);
  require(accelerating_moved.success&&accelerating_moved.prefix_transported,"small acceleration-prefix jump transports saturated jerk schedule");
  require(std::abs(accelerating_moved.controls[0][0]-(accelerating.controls[0][0]-.01))<1e-12,"acceleration offset anchors the first interval");
  auto clipped_source=original;clipped_source.states={start};clipped_source.states[0][3]=.8;
  clipped_source.initial_applied={0.,0.,0.};
  for(int k=0;k<bundle.config().horizon;++k)
    clipped_source.controls[k]={std::clamp((1-k)*bundle.config().jerk_limit*bundle.config().dt,-bundle.config().brake_limit,bundle.config().accel_limit),0.,.4};
  auto clipped_original=core.reanchor(clipped_source,clipped_source.states.front(),clipped_source.initial_applied,10.02);
  require(clipped_original.success,"original bounded braking schedule is feasible");
  // The first positive jerk interval triggers transport; later braking stages
  // reach the hard limit after the same negative acceleration offset.
  auto clipped=core.reanchor(clipped_original,clipped_source.states.front(),{-.01,0.,0.},10.039);
  require(clipped.success&&clipped.prefix_transported,"transport keeps saturated braking inputs within hard bounds");
  require(clipped.controls.back()[0]>=-bundle.config().brake_limit,"transport acceleration clamp preserves hard braking limit");
  // Immutable actual state already outside the ellipse remains inadmissible,
  // even when its applied prefix meets every transport eligibility condition.
  auto envelope_state=start;envelope_state[3]=bundle.config().max_speed;envelope_state[5]=bundle.config().steer_limit;
  auto envelope=core.reanchor(certified,envelope_state,{0.,0.,0.},10.039);
  require(!envelope.success&&!envelope.prefix_transported,"strict operating ellipse is never relaxed by transport");
  require(envelope.max_violation>1e-4,"original ellipse certificate remains failed");
  require(envelope.reason.find("nonlinear candidate rejected; prefix transport rejected")!=std::string::npos,"failed alternate certificate preserves original reason and reports rejection");
  require(envelope.controls==certified.controls,"failed transport preserves original rejected candidate");
  // Frozen recorded-route snapshot sequence713. Its original feasible plan
  // fails the physical ellipse at stage1,t=.10 after takeover; endpoint-only
  // transport also fails. Retain the FIRST failure's exact certificate.
  const auto &cfg=bundle.config();
  if(cfg.dt==.1&&cfg.horizon==10&&cfg.wheelbase==.36&&cfg.steering_tau==.08&&
     cfg.understeer_coefficient==0.&&cfg.envelope_accel==.5&&cfg.envelope_brake==.5&&
     cfg.lateral_accel_limit==1.&&cfg.max_speed==1.5&&cfg.jerk_limit==1.&&
     cfg.steer_limit==.45&&cfg.steer_rate==2.&&cfg.steer_acceleration==2.){
    auto fixture=original;fixture.states={start};
    fixture.states[0][3]=.957551143327432;fixture.states[0][5]=.2685336189148317;
    fixture.initial_applied={.04014561583541888,.29941416716923136,.31674777580685687};
    fixture.controls={{.14014561583521193,.3239175575268194,.938563348053184},
      {.2053985384634625,.32842094788440623,.9440399576788912},
      {.14700130183402987,.3129243382419943,.9537036174757647},
      {.04700130183429872,.27742772859958414,.9671070020697237},
      {-.052998698163919,.2219311189571755,.9787856886127208},
      {.001352815776939558,.1464345093147671,.9902606326871212},
      {.10135281576711033,.050937899672358525,1.0046134096659383},
      {.0809620710371372,-.06455870997004835,1.0179775217773606},
      {-.01903792895689755,-.20005531961243528,1.0223211866239856},
      {-.1190379289536022,-.3413543715679318,1.018076436450086}};
    auto fixture_original=core.reanchor(fixture,fixture.states.front(),fixture.initial_applied,10.02);
    require(fixture_original.success,"recorded sequence713 original certificate is feasible");
    auto fixture_actual=fixture.states.front();fixture_actual[3]=.9627606903321908;fixture_actual[5]=.27124454318075514;
    Applied fixture_prefix{.04720943583494497,.3013690412044525,.3026201358077967};
    auto fixture_failed=core.reanchor(fixture_original,fixture_actual,fixture_prefix,10.027067305999954);
    require(!fixture_failed.success&&!fixture_failed.prefix_transported,"recorded future ellipse violation remains rejected");
    require(std::abs(fixture_failed.max_violation-.009527677490100084)<1e-12,"failed transport keeps the original exact ellipse maximum");
    require(fixture_failed.controls==fixture_original.controls,"failed recorded transport retains original controls");
    require(fixture_failed.reason.find("prefix transport rejected")!=std::string::npos,"recorded ellipse rejection reports alternate transport failure");
  }
  std::cout<<"PASS: conditional bounded prefix transport, unchanged feasible controls, hard bounds, TTL/pass preservation and strict ellipse rejection\n";
}

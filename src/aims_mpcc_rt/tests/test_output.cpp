#include "aims_mpcc_rt/output.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace aims_mpcc_rt;
int main(){
  Config c{};c.max_speed=1.5;c.steer_limit=.45;c.steer_rate=2.;
  c.steer_acceleration=2.;c.accel_limit=.5;c.brake_limit=.5;c.jerk_limit=1.;
  c.minimum_drive_speed=.2;c.wheelbase=.36;c.lateral_accel_limit=1.;c.steering_tau=.08;
  c.envelope_accel=c.envelope_brake=.5;
  OutputSampler out(c);out.reset(0.,0.,10.);
  Plan p;p.success=true;p.dt=.1;p.source_epoch=10.;p.forecast_epoch=10.;
  p.controls.assign(10,Control{.5,.2,1.});p.states.assign(11,State{0,0,0,1,0,0});
  State current{};
  auto a=out.sample(&p,10.02,current,false,.8);
  assert(a.continuous_speed>0&&a.continuous_speed<.001);
  assert(std::abs(a.continuous_acceleration-.02)<1e-10);
  assert(a.acceleration==0.); // previous-control metadata follows legacy wire floor convention
  assert(std::abs(a.steering_rate-.04)<1e-10);
  assert(std::abs(a.steering)<.001);
  assert(a.acceleration_limited&&a.steering_rate_limited);
  assert(a.requested_acceleration==.5&&std::abs(a.requested_steering_rate-2.)<1e-10);
  assert(a.speed==.2); // wire floor is separate from continuous state
  auto expired=out.sample(&p,11.,current,false,.8);
  assert(expired.expired);
  assert(expired.acceleration<=0.); // expired plans cannot accelerate
  out.reset(.5,.1,20.);
  auto stop=out.sample(nullptr,20.02,current,true,.8);
  assert(stop.continuous_speed<.5);
  assert(stop.speed<.5);
  assert(std::abs(stop.steering-.1)<1e-12);
  out.reset(.5,.1,30.);
  p.source_epoch=p.forecast_epoch=30.;
  auto finish=out.sample(&p,30.02,current,false,.8,.1);
  assert(finish.continuous_acceleration<0.);
  out.reset(.1,0.,40.);p.source_epoch=p.forecast_epoch=40.;
  auto near_finish=out.sample(&p,40.02,current,false,.8,0.);
  assert(near_finish.continuous_speed>0.&&near_finish.continuous_speed<.1);
  assert(near_finish.speed==0.); // finish reserve must suppress the motor floor
  out.reset(1.,.45,50.);current[3]=1.;current[5]=.45;
  auto unavailable_budget=out.sample(nullptr,50.02,current,false,.8);
  assert(unavailable_budget.continuous_acceleration<0.);
  assert(unavailable_budget.continuous_speed<1.); // recovery cannot freeze at an infeasible lateral load
  assert(out.braking_budget().available&&!out.braking_budget().feasible);
  c.command_profile="rate_bounded_v2";
  OutputSampler selected(c);selected.reset(0.,0.,60.);
  p.source_epoch=60.;p.forecast_epoch=60.02;
  auto immediate=selected.sample(&p,60.02,State{},false,.8);
  assert(std::abs(immediate.continuous_acceleration-.5)<1e-10);
  assert(std::abs(immediate.continuous_speed-.01)<1e-10);
  assert(std::abs(immediate.steering-.04)<1e-10);
  assert(!immediate.acceleration_limited&&!immediate.steering_rate_limited);
  assert(std::abs(immediate.commanded_target_steering-.04)<1e-10);
  // Reanchor installs forecast_epoch at this callback's decision time. The
  // packet being emitted owns the upcoming hold, including stage-zero input.
  selected.reset(0.,0.,65.);p.source_epoch=65.;p.forecast_epoch=65.02;
  auto takeover=selected.sample(&p,65.02,State{},false,.8);
  assert(std::abs(takeover.continuous_acceleration-.5)<1e-10);
  assert(std::abs(takeover.continuous_speed-.01)<1e-10);
  assert(std::abs(takeover.steering-.04)<1e-10);
  auto subsequent=selected.sample(&p,65.04,State{},false,.8);
  assert(std::abs(subsequent.continuous_speed-.02)<1e-10);
  assert(std::abs(subsequent.steering-.08)<1e-10);
  // Actual elapsed time, stage crossing and ramp reversal. Stage0 ends at
  // 0.2, stage1 returns to zero: the target crosses the 100 ms boundary.
  selected.reset(.3,0.,70.);p.source_epoch=70.;p.forecast_epoch=70.04;p.initial_applied={0.,0.,3.};
  p.controls[0]={.5,.2,1.};p.controls[1]={-.5,0.,1.};
  auto t1=selected.sample(&p,70.04,State{},false,.8);
  assert(std::abs(t1.continuous_speed-.32)<1e-10&&std::abs(t1.steering-.04)<1e-10);
  auto t2=selected.sample(&p,70.08,State{},false,.8);
  auto crossing=selected.sample(&p,70.115,State{},false,.8);
  assert(std::abs(t2.steering-.12)<1e-10);
  assert(std::abs(crossing.continuous_speed-.3475)<1e-10);
  assert(std::abs(crossing.continuous_acceleration-(.0075/.035))<1e-10);
  assert(std::abs(crossing.steering-.19)<1e-10);
  assert(std::abs(crossing.steering_rate-2.)<1e-10);
  assert(!crossing.acceleration_limited); // planned interval averaging is not a limiter
  auto reversed=selected.sample(&p,70.14,State{},false,.8);
  assert(std::abs(reversed.continuous_speed-.335)<1e-10);
  assert(std::abs(reversed.steering-.16)<1e-10&&reversed.steering_rate<0.);
  // Nonzero prefix: targets belong to the upcoming packet, so historical
  // prefix acceleration is not reapplied before the new plan's activation.
  selected.reset(.4,.1,80.,-.2,.5);p.source_epoch=80.;p.forecast_epoch=80.01;
  p.initial_applied={-.2,.1,.5};p.controls[0]={.5,.2,1.};
  auto activation=selected.sample(&p,80.03,State{},false,.8);
  assert(std::abs(activation.continuous_speed-.415)<1e-10);
  assert(std::abs(activation.steering-.14)<1e-10);
  // Retained bounds operate immediately without higher-order stopping reserve.
  selected.reset(1.499,.44,90.);p.source_epoch=p.forecast_epoch=90.;
  p.initial_applied={0.,.44,0.};p.controls[0]={5.,1.,1.};
  auto saturated=selected.sample(&p,90.04,State{},false,.8);
  assert(saturated.continuous_speed==c.max_speed&&saturated.steering==c.steer_limit);
  assert(std::abs(saturated.steering_rate)<=c.steer_rate);
  selected.reset(.01,.1,100.);auto bounded_stop=selected.sample(nullptr,100.04,State{},true,.8);
  assert(bounded_stop.continuous_speed==0.&&bounded_stop.speed==0.&&bounded_stop.steering==.1);
  selected.reset(.4,.1,110.);p.source_epoch=p.forecast_epoch=109.;
  auto v2_expired=selected.sample(&p,110.02,State{},false,.8);
  assert(v2_expired.expired&&v2_expired.continuous_acceleration<0.);
  selected.reset(.4,.1,120.);bool fault=false;
  try{selected.sample(nullptr,120.06,State{},true,.8);}catch(const std::runtime_error&){fault=true;}
  assert(fault); // never discard elapsed time through an unreported dt clamp
  // An initially feasible command can approach the speed cap with delayed
  // feedback. Override braking must budget the actual/predicted command speed,
  // not only the stale lower measurement, and retain the physical ellipse.
  c.minimum_drive_speed=0.;OutputSampler capped(c);capped.reset(1.05,.3,130.);
  p.source_epoch=130.;p.forecast_epoch=130.02;p.initial_applied={0.,.3,0.};
  p.controls.assign(10,Control{0.,.3,1.05});
  State actual{};actual[3]=1.05;actual[5]=.3;State delayed=actual;delayed[3]=1.;
  auto budgeted=capped.sample(&p,130.02,delayed,false,.8,.9);
  const double lateral=actual[3]*actual[3]*std::tan(actual[5])/c.wheelbase;
  assert(lateral*lateral<1.); // the original hold is initially safe
  const double utilization=std::pow(budgeted.continuous_acceleration/c.envelope_brake,2)+lateral*lateral;
  assert(utilization<=1.+1e-10);
  assert(budgeted.continuous_acceleration<0.&&capped.braking_budget().feasible);
  // A higher internal target is a conservative proxy, not proof that the
  // current physical decision state is already outside the lateral envelope.
  capped.reset(1.15,.3,140.);p.source_epoch=140.;p.forecast_epoch=140.02;
  auto proxy=capped.sample(&p,140.02,actual,false,.8,.9);
  const double proxy_utilization=std::pow(proxy.continuous_acceleration/c.envelope_brake,2)+lateral*lateral;
  assert(proxy_utilization<=1.+1e-10);
  assert(proxy.continuous_acceleration<0.&&capped.braking_budget().feasible);
  // A 50 ms held-target proxy can be infeasible even though the certified
  // next nominal 20 ms packet and subsequent unwind are physically feasible.
  capped.reset(1.05,.33,150.);p.source_epoch=p.forecast_epoch=149.94;
  p.initial_applied={0.,.25,0.};p.controls.assign(10,Control{0.,.15,1.05});p.controls[0]={0.,.35,1.05};
  auto conservative=capped.sample(&p,150.02,actual,false,.8,.9);
  const double conservative_utilization=std::pow(conservative.continuous_acceleration/c.envelope_brake,2)+lateral*lateral;
  assert(conservative_utilization<=1.+1e-10);
  assert(!capped.braking_budget().feasible&&capped.braking_budget().current_feasible);
  assert(conservative.continuous_acceleration==0.); // zero conservative capacity, never unrestricted braking
  std::cout<<"bounded command sampler passed\n";
}

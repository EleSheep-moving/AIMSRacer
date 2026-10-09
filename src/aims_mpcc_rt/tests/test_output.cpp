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
  std::cout<<"bounded command sampler passed\n";
}

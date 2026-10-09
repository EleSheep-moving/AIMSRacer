#include "aims_mpcc_rt/execution.hpp"
#include <chrono>
#include <algorithm>
#include <sstream>
namespace aims_mpcc_rt {
ExecutionCertificate certify_execution(const Plan& plan,const OutputSampler& live,const State& initial,
  const Alignment& alignment,const Config& cfg,const Reference& ref,double now,double ttl,double cap,
  const std::vector<double>& output_intervals){
  const auto started=std::chrono::steady_clock::now();ExecutionCertificate result;
  auto finish=[&](bool success,const std::string& reason){
    result.success=success;result.reason=reason;
    result.duration_s=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
    return result;
  };
  auto finite=[](const auto& values){return std::all_of(values.begin(),values.end(),[](double v){return std::isfinite(v);});};
  if(!plan.success||plan.controls.empty()||!finite(initial)||!finite(alignment)||
     !std::isfinite(now)||!std::isfinite(ttl)||ttl<=0.||!std::isfinite(plan.dt)||plan.dt<=0.||
     !std::isfinite(plan.forecast_epoch)||!std::isfinite(plan.source_epoch)||
     !std::isfinite(cfg.wheelbase)||cfg.wheelbase<=0.||cfg.steering_tau<=0.||
     cfg.envelope_accel<=0.||cfg.envelope_brake<=0.||cfg.lateral_accel_limit<=0.||std::isnan(cap))
    return finish(false,"invalid executed trace input");
  for(const auto& u:plan.controls)if(!finite(u))return finish(false,"nonfinite executed control");
  const std::array<double,12> limits{cfg.steering_tau,cfg.envelope_accel,cfg.envelope_brake,cfg.lateral_accel_limit,
    cfg.max_speed,cfg.steer_limit,cfg.steer_rate,cfg.accel_limit,cfg.brake_limit,cfg.half_width,cfg.front_extent,cfg.rear_extent};
  if(!finite(limits)||cfg.understeer_coefficient<0.||!std::isfinite(cfg.understeer_coefficient))
    return finish(false,"nonfinite executed model or footprint geometry");
  for(double interval:output_intervals)if(!std::isfinite(interval)||interval<=0.||interval>.05+1e-10)
    return finish(false,"unsupported executed output interval");
  const double until=std::min(plan.source_epoch+ttl,plan.forecast_epoch+plan.controls.size()*plan.dt);
  if(now<plan.forecast_epoch||now<plan.source_epoch||now>=until)
    return finish(false,"executed trace has no valid coverage");
  auto sampler=live;sampler.set_previous_endpoint(plan.initial_applied[1]);State state=initial;
  auto inspect=[&](double acceleration){
    ++result.checks;
    result.final_state=state;
    if(!finite(state)||!std::isfinite(acceleration))return false;
    const double lateral=state[3]*state[3]*std::tan(state[5])/
      (cfg.wheelbase*(1.+cfg.understeer_coefficient*state[3]*state[3]));
    const double axis=acceleration>=0.?cfg.envelope_accel:cfg.envelope_brake;
    const double utilization=std::pow(acceleration/axis,2)+std::pow(lateral/cfg.lateral_accel_limit,2);
    if(!std::isfinite(utilization))return false;
    result.max_utilization=std::max(result.max_utilization,utilization);
    if(cfg.enforce_corridor){
      const auto r=ref.at(state[4]);
      if(!std::isfinite(r.x)||!std::isfinite(r.y)||!std::isfinite(r.yaw))return false;
      const double cy=std::cos(alignment[2]),sy=std::sin(alignment[2]);
      const double px=cy*state[0]-sy*state[1]+alignment[0],py=sy*state[0]+cy*state[1]+alignment[1];
      const double yaw=state[2]+alignment[2];
      for(double along:{cfg.front_extent,-cfg.rear_extent})for(double across:{-cfg.half_width,cfg.half_width}){
        const double dx=px+along*std::cos(yaw)-across*std::sin(yaw)-r.x;
        const double dy=py+along*std::sin(yaw)+across*std::cos(yaw)-r.y;
        const double lateral_corner=-std::sin(r.yaw)*dx+std::cos(r.yaw)*dy;
        result.max_corridor_violation=std::max({result.max_corridor_violation,
          lateral_corner-ref.left_width(),-ref.right_width()-lateral_corner});
      }
    }
    return state[3]>=-1e-8&&state[3]<=cfg.max_speed+1e-8&&
      std::abs(state[5])<=cfg.steer_limit+1e-8&&acceleration>=-cfg.brake_limit-1e-8&&
      acceleration<=cfg.accel_limit+1e-8&&utilization<=1.+1e-4&&result.max_corridor_violation<=1e-4;
  };
  auto violation_reason=[&](double acceleration){
    std::ostringstream text;text<<"executed trace violates retained physical bounds: E="<<result.max_utilization
      <<" v="<<state[3]<<" steering="<<state[5]<<" acceleration="<<acceleration
      <<" corridor="<<result.max_corridor_violation;return text.str();
  };
  try{
    std::size_t tick=0;
    for(double time=now;time<until-1e-10;){
      const auto command=sampler.sample(&plan,time,state,false,ttl,cap);
      if(command.expired||!std::isfinite(command.steering)||!std::isfinite(command.continuous_speed)||
         std::abs(command.steering)>cfg.steer_limit+1e-8||std::abs(command.steering_rate)>cfg.steer_rate+1e-8)
        return finish(false,"invalid bounded output in executed trace");
      // Independent 2 ms midpoint spatial integration and exact held steering
      // response. The ideal longitudinal acceleration is the internal command,
      // not the wire floor's metadata zero, and is not a motor identification.
      const double interval=output_intervals.empty()?.02:output_intervals[tick++%output_intervals.size()];
      const double end=std::min(until,time+interval),a=command.continuous_acceleration;
      if(!inspect(a))return finish(false,violation_reason(a));
      for(double t=time;t<end-1e-12;){
        const auto k=std::min(plan.controls.size()-1,std::size_t(std::floor((t-plan.forecast_epoch+1e-10)/plan.dt)));
        const double stage_end=plan.forecast_epoch+(k+1)*plan.dt;
        const double h=std::min({.002,end-t,stage_end-t});
        if(h<=0.)return finish(false,"executed progress integration made no stage progress");
        const double v=std::max(0.,state[3]+a*h/2.);
        const double delta=command.steering+(state[5]-command.steering)*std::exp(-h/(2.*cfg.steering_tau));
        const double yaw_rate=v*std::tan(delta)/(cfg.wheelbase*(1.+cfg.understeer_coefficient*v*v));
        const double middle_yaw=state[2]+yaw_rate*h/2.;
        state[0]+=h*v*std::cos(middle_yaw);state[1]+=h*v*std::sin(middle_yaw);state[2]+=h*yaw_rate;
        state[3]=std::max(0.,state[3]+a*h);state[4]+=h*plan.controls[k][2];
        state[5]=command.steering+(state[5]-command.steering)*std::exp(-h/cfg.steering_tau);
        if(!inspect(a))return finish(false,violation_reason(a));
        t+=h;
      }
      time=end;
    }
  }catch(const std::exception& e){return finish(false,std::string("executed trace unavailable: ")+e.what());}
  return finish(true,output_intervals.empty()?"nominal 20 ms executed trace certified with ideal acceleration":
    "specified output intervals certified with ideal acceleration");
}
}

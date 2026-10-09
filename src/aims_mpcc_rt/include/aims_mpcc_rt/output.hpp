#pragma once
#include "aims_mpcc_rt/core.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <limits>

namespace aims_mpcc_rt {
struct OutputCommand {
  double speed{},continuous_speed{},steering{},acceleration{},steering_rate{};
  bool expired{};
  double continuous_acceleration{};
};
struct BrakingBudget {bool available{},feasible{};double lateral_utilization{},accel_capacity{},brake_capacity{};};
// Bounded command integration is distinct from physical vehicle speed. The
// minimum motor setpoint does not become a discontinuous OCP speed state.
class OutputSampler {
 public:
  explicit OutputSampler(Config config):cfg_(config){}
  void reset(double speed,double steering,double now){
    speed_=std::clamp(speed,0.,cfg_.max_speed);steering_=steering;
    acceleration_=rate_=0.;last_=now;
  }
  OutputCommand sample(const Plan* plan,double now,const State& measured,
                       bool stopping,double ttl,double speed_cap=std::numeric_limits<double>::infinity()){
    if(!std::isfinite(now)||now<last_) throw std::invalid_argument("output clock moved backwards");
    const double dt=std::min(.05,now-last_);last_=now;
    budget_={};
    bool valid=plan&&plan->success&&plan->dt>0&&!plan->controls.empty()&&
      now>=plan->forecast_epoch&&now>=plan->source_epoch&&now-plan->source_epoch<=ttl&&
      now-plan->forecast_epoch<plan->controls.size()*plan->dt;
    double desired_a=dt>0?-speed_/dt:0.,desired_rate=0.;
    if(valid&&!stopping){
      auto k=std::min(plan->controls.size()-1,std::size_t((now-plan->forecast_epoch)/plan->dt));
      desired_a=plan->controls[k][0];
      double previous=k?plan->controls[k-1][1]:previous_endpoint_;
      desired_rate=(plan->controls[k][1]-previous)/plan->dt;
      if(dt>0&&speed_cap<speed_)desired_a=(std::max(0.,speed_cap)-speed_)/dt;
    }
    if(dt>0){
      auto stop_bound=[dt](double distance,double change){
        return std::sqrt(std::pow(change*dt,2)+2*change*std::max(0.,distance))-change*dt;
      };
      double lo=std::max({-cfg_.steer_rate,rate_-cfg_.steer_acceleration*dt,
                        -stop_bound(cfg_.steer_limit+steering_,cfg_.steer_acceleration)});
      double hi=std::min({cfg_.steer_rate,rate_+cfg_.steer_acceleration*dt,
                         stop_bound(cfg_.steer_limit-steering_,cfg_.steer_acceleration)});
      if(lo>hi) throw std::runtime_error("steering continuation infeasible");
      rate_=std::clamp(desired_rate,lo,hi);
      steering_=std::clamp(steering_+rate_*dt,-cfg_.steer_limit,cfg_.steer_limit);
      double a_lo=std::max({-cfg_.brake_limit,acceleration_-cfg_.jerk_limit*dt,
                            -stop_bound(speed_,cfg_.jerk_limit)});
      double a_hi=std::min({cfg_.accel_limit,acceleration_+cfg_.jerk_limit*dt,
                            stop_bound(cfg_.max_speed-speed_,cfg_.jerk_limit)});
      if(a_lo>a_hi) throw std::runtime_error("speed continuation infeasible");
      // Match the legacy nominal 20 ms recovery/speed-override certificate.
      // An infeasible lateral budget must not suppress bounded stopping.
      if(!stopping&&(!valid||speed_cap<speed_)){
        double speed=std::max({std::abs(measured[3]),std::abs(measured[3]+a_lo*.02),std::abs(measured[3]+a_hi*.02)});
        double angle=std::max(std::abs(measured[5]),std::abs(steering_));
        budget_.available=std::isfinite(speed)&&std::isfinite(angle)&&cfg_.steering_tau>=.001&&
          cfg_.understeer_coefficient>=0.&&cfg_.wheelbase>0.&&angle<std::acos(-1.)/2;
        if(budget_.available){
          double lateral=speed*speed*std::tan(angle)/(cfg_.wheelbase*(1+cfg_.understeer_coefficient*speed*speed));
          budget_.lateral_utilization=std::pow(lateral/cfg_.lateral_accel_limit,2);
          double capacity=std::sqrt(std::max(0.,1-budget_.lateral_utilization));
          budget_.accel_capacity=std::min(cfg_.accel_limit,cfg_.envelope_accel*capacity);
          budget_.brake_capacity=std::min(cfg_.brake_limit,cfg_.envelope_brake*capacity);
          budget_.feasible=budget_.lateral_utilization<=1.&&
            std::max(a_lo,-budget_.brake_capacity)<=std::min(a_hi,budget_.accel_capacity);
          if(budget_.feasible)desired_a=std::clamp(desired_a,-budget_.brake_capacity,budget_.accel_capacity);
        }
      }
      acceleration_=std::clamp(desired_a,a_lo,a_hi);
      double next=std::clamp(speed_+acceleration_*dt,0.,cfg_.max_speed);
      acceleration_=(next-speed_)/dt;speed_=next;
    }
    double wire=speed_;
    if(valid&&!stopping&&speed_cap>0.&&wire>1e-6&&wire<cfg_.minimum_drive_speed) wire=cfg_.minimum_drive_speed;
    else if(wire<cfg_.minimum_drive_speed) wire=0.;
    return {wire,speed_,steering_,wire==speed_?acceleration_:0.,rate_,!valid,acceleration_};
  }
  void set_previous_endpoint(double value){previous_endpoint_=value;}
  double continuous_speed() const{return speed_;}
  const BrakingBudget& braking_budget()const{return budget_;}
 private:
  Config cfg_;
  BrakingBudget budget_;
  double speed_=0.,steering_=0.,acceleration_=0.,rate_=0.,last_=0.,previous_endpoint_=0.;
};
} // namespace aims_mpcc_rt

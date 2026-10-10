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
  double requested_acceleration{},requested_steering_rate{},commanded_target_steering{};
  bool acceleration_limited{},steering_rate_limited{};
};
struct BrakingBudget {bool available{},feasible{};double lateral_utilization{},accel_capacity{},brake_capacity{};
  bool current_feasible{};double current_lateral_utilization{};};
// Bounded command integration is distinct from physical vehicle speed. The
// minimum motor setpoint does not become a discontinuous OCP speed state.
class OutputSampler {
 public:
  explicit OutputSampler(Config config):cfg_(config){}
  void reset(double speed,double steering,double now,double acceleration=0.,double rate=0.){
    speed_=std::clamp(speed,0.,cfg_.max_speed);steering_=steering;
    acceleration_=acceleration;rate_=rate;last_=now;
  }
  OutputCommand sample(const Plan* plan,double now,const State& measured,
                       bool stopping,double ttl,double speed_cap=std::numeric_limits<double>::infinity()){
    if(!std::isfinite(now)||now<last_) throw std::invalid_argument("output clock moved backwards");
    if(cfg_.command_profile=="rate_bounded_v2")return sample_rate_bounded(plan,now,measured,stopping,ttl,speed_cap);
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
    double requested_a=desired_a,requested_rate=desired_rate,target_steering=steering_+desired_rate*dt;
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
      if(cfg_.combined_accel_constraint_enabled&&!stopping&&(!valid||speed_cap<speed_)){
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
    return {wire,speed_,steering_,wire==speed_?acceleration_:0.,rate_,!valid,acceleration_,
            requested_a,requested_rate,target_steering,std::abs(acceleration_-requested_a)>1e-9,
            std::abs(rate_-requested_rate)>1e-9};
  }
  void set_previous_endpoint(double value){previous_endpoint_=value;}
  double continuous_speed() const{return speed_;}
  const BrakingBudget& braking_budget()const{return budget_;}
 private:
  OutputCommand sample_rate_bounded(const Plan* plan,double now,const State& measured,
                                    bool stopping,double ttl,double speed_cap){
    const double elapsed=now-last_;
    if(elapsed>.05+1e-10)throw std::runtime_error("output scheduling interval exceeds 50 ms");
    if(!std::isfinite(speed_)||!std::isfinite(steering_)||!std::isfinite(ttl)||ttl<=0.||
       std::isnan(speed_cap))throw std::invalid_argument("finite output state and limits required");
    budget_={};
    bool valid=plan&&plan->success&&std::isfinite(plan->dt)&&plan->dt>0.&&
      !plan->controls.empty()&&std::isfinite(plan->forecast_epoch)&&std::isfinite(plan->source_epoch)&&
      now>=plan->forecast_epoch&&now>=plan->source_epoch&&now-plan->source_epoch<=ttl&&
      now-plan->forecast_epoch<plan->controls.size()*plan->dt;
    // A published speed target and steering sample own the upcoming hold.
    // Native activation sets forecast_epoch to now, so the first packet must
    // already carry u0 and the model's first held 20 ms steering endpoint.
    // Command targets advance by actual elapsed time; forwarded history stays
    // at its real publication epoch and is never backdated to last_.
    double requested_a=acceleration_,requested_rate=rate_,target_steering=steering_;
    if(elapsed>0.){
      if(valid&&!stopping){
        const double phase=std::clamp(now-plan->forecast_epoch+.02,0.,plan->controls.size()*plan->dt);
        const size_t k=std::min(plan->controls.size()-1,size_t(std::floor((phase+1e-10)/plan->dt)));
        const double previous=k?plan->controls[k-1][1]:plan->initial_applied[1];
        const double endpoint=plan->controls[k][1];
        if(!std::isfinite(previous)||!std::isfinite(endpoint))throw std::invalid_argument("finite planned steering required");
        const double fraction=std::clamp((phase-k*plan->dt)/plan->dt,0.,1.);
        target_steering=previous+(endpoint-previous)*fraction;
      }
      const double previous_command_angle=steering_;
      requested_rate=(target_steering-steering_)/elapsed;
      rate_=std::clamp((std::clamp(target_steering,-cfg_.steer_limit,cfg_.steer_limit)-steering_)/elapsed,
                       -cfg_.steer_rate,cfg_.steer_rate);
      steering_=std::clamp(steering_+rate_*elapsed,-cfg_.steer_limit,cfg_.steer_limit);
      const double initial_speed=speed_,until=now+elapsed;
      double requested_integral=0.;
      const double coverage=valid?std::min(plan->source_epoch+ttl,
        plan->forecast_epoch+plan->controls.size()*plan->dt):now;
      for(double t=now;t<until-1e-12;){
        double end=until,desired_a=-speed_/(until-t);
        const bool planned_interval=valid&&!stopping&&t<coverage-1e-12;
        if(planned_interval){
          const double phase=std::max(0.,t-plan->forecast_epoch);
          const size_t k=std::min(plan->controls.size()-1,size_t(std::floor((phase+1e-10)/plan->dt)));
          end=std::min({end,coverage,plan->forecast_epoch+(k+1)*plan->dt});
          desired_a=plan->controls[k][0];
          if(!std::isfinite(desired_a))throw std::invalid_argument("finite planned acceleration required");
        }
        const double h=end-t;
        if(h<=1e-12)throw std::runtime_error("output stage boundary made no progress");
        requested_integral+=desired_a*h;
        if(!stopping&&speed_cap<speed_)desired_a=(std::max(0.,speed_cap)-speed_)/h;
        // Source TTL or horizon coverage can end inside an otherwise valid
        // publication packet. Its recovery portion needs the same physical
        // budget as a packet that was already expired at publication.
        if(cfg_.combined_accel_constraint_enabled&&!stopping&&(!planned_interval||speed_cap<speed_)){
          // Runtime supplies the decision-epoch physical prediction. The
          // internal command speed can exceed delayed feedback, so include
          // both across the supported 50 ms hold bound.
          const double bounded_a=std::clamp(desired_a,-cfg_.brake_limit,cfg_.accel_limit);
          double speed=std::max({std::abs(measured[3]),std::abs(speed_),
            std::max(0.,measured[3]+bounded_a*.05),std::max(0.,speed_+bounded_a*.05)});
          double angle=std::max({std::abs(measured[5]),std::abs(previous_command_angle),std::abs(steering_)});
          budget_.available=std::isfinite(speed)&&std::isfinite(angle)&&cfg_.steering_tau>=.001&&
            cfg_.understeer_coefficient>=0.&&cfg_.wheelbase>0.&&angle<std::acos(-1.)/2;
          if(budget_.available){
            double lateral=speed*speed*std::tan(angle)/(cfg_.wheelbase*(1+cfg_.understeer_coefficient*speed*speed));
            budget_.lateral_utilization=std::pow(lateral/cfg_.lateral_accel_limit,2);
            if(budget_.lateral_utilization>1.){
              // An infeasible command-speed/angle proxy must not disable the
              // actual physical state's braking budget. Use its bounded held
              // lag trajectory, with a full supported 50 ms speed preview.
              const double physical_speed=std::max(std::abs(measured[3]),std::max(0.,measured[3]+bounded_a*.05));
              const double held_delta=steering_+(measured[5]-steering_)*std::exp(-.05/cfg_.steering_tau);
              const double physical_angle=std::max(std::abs(measured[5]),std::abs(held_delta));
              const double physical_lateral=physical_speed*physical_speed*std::tan(physical_angle)/
                (cfg_.wheelbase*(1+cfg_.understeer_coefficient*physical_speed*physical_speed));
              budget_.lateral_utilization=std::pow(physical_lateral/cfg_.lateral_accel_limit,2);
            }
            double capacity=std::sqrt(std::max(0.,1-budget_.lateral_utilization));
            budget_.accel_capacity=std::min(cfg_.accel_limit,cfg_.envelope_accel*capacity);
            budget_.brake_capacity=std::min(cfg_.brake_limit,cfg_.envelope_brake*capacity);
            budget_.feasible=budget_.lateral_utilization<=1.;
            const double current_lateral=measured[3]*measured[3]*std::tan(measured[5])/
              (cfg_.wheelbase*(1+cfg_.understeer_coefficient*measured[3]*measured[3]));
            budget_.current_lateral_utilization=std::pow(current_lateral/cfg_.lateral_accel_limit,2);
            budget_.current_feasible=std::isfinite(budget_.current_lateral_utilization)&&budget_.current_lateral_utilization<=1.;
            // A conservative future hold can be infeasible while the current
            // physical state is safe. Keep its zero capacity in that case;
            // do not turn a failed proxy into unrestricted full braking.
            // The nominal prospective executed certificate checks the actual
            // upcoming 20 ms hold and following planned unwind independently.
            if(budget_.feasible||budget_.current_feasible)
              desired_a=std::clamp(desired_a,-budget_.brake_capacity,budget_.accel_capacity);
          }
        }
        speed_=std::clamp(speed_+std::clamp(desired_a,-cfg_.brake_limit,cfg_.accel_limit)*h,0.,cfg_.max_speed);
        t=end;
      }
      // One packet has one ideal acceleration. Across a stage boundary its
      // metadata is the mean that reproduces this packet's speed increment.
      acceleration_=(speed_-initial_speed)/elapsed;
      requested_a=requested_integral/elapsed;
    }
    last_=now;
    double wire=speed_;
    if(valid&&!stopping&&speed_cap>0.&&wire>1e-6&&wire<cfg_.minimum_drive_speed)wire=cfg_.minimum_drive_speed;
    else if(wire<cfg_.minimum_drive_speed)wire=0.;
    return {wire,speed_,steering_,wire==speed_?acceleration_:0.,rate_,!valid,acceleration_,
            requested_a,requested_rate,target_steering,std::abs(acceleration_-requested_a)>1e-9,
            std::abs(rate_-requested_rate)>1e-9};
  }
  Config cfg_;
  BrakingBudget budget_;
  double speed_=0.,steering_=0.,acceleration_=0.,rate_=0.,last_=0.,previous_endpoint_=0.;
};
} // namespace aims_mpcc_rt

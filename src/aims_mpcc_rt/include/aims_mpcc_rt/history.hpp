#pragma once
#include "aims_mpcc_rt/core.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <stdexcept>

namespace aims_mpcc_rt {
struct HistoryCommand {double speed{},steering{},acceleration{},steering_rate{};};
class AppliedHistory {
 public:
  explicit AppliedHistory(Config config):cfg_(config){}
  void clear(){records_.clear();}
  void record(double now,double steering,double speed,double acceleration,double rate){
    if(!std::isfinite(now)||!std::isfinite(steering)||!std::isfinite(speed)||
       !std::isfinite(acceleration)||!std::isfinite(rate))
      throw std::invalid_argument("nonfinite applied command");
    if(!records_.empty()&&now<records_.back().time)
      throw std::invalid_argument("applied history moved backwards");
    double estimate=at(now).first;
    records_.push_back({now,estimate,{speed,steering,acceleration,rate}});
    if(records_.size()>1024)records_.pop_front();
  }
  std::pair<double,Applied> at(double now)const{
    for(auto it=records_.rbegin();it!=records_.rend();++it)if(it->time<=now){
      auto c=it->command;
      double delta=c.steering+(it->estimate-c.steering)*std::exp(-(now-it->time)/cfg_.steering_tau);
      return {delta,{c.acceleration,c.steering,c.steering_rate}};
    }
    return {0.,{0.,0.,0.}};
  }
  HistoryCommand command_at(double now)const{
    for(auto it=records_.rbegin();it!=records_.rend();++it)if(it->time<=now)return it->command;
    throw std::invalid_argument("actual input history does not cover measurement epoch");
  }
  bool covers(double now)const{return std::isfinite(now)&&!records_.empty()&&records_.front().time<=now;}
  double newest()const{return records_.empty()?-1.:records_.back().time;}
  State predict(State x,double begin,double end,double known_until,
      std::function<HistoryCommand(double,const State&)> future={},HistoryCommand* final_command=nullptr)const{
    if(!std::isfinite(begin)||!std::isfinite(end)||!std::isfinite(known_until)||
       begin>end||known_until<begin||known_until>end)
      throw std::invalid_argument("invalid prediction interval");
    auto command=command_at(begin);double time=begin,next_future=known_until;
    while(time<end-1e-10){
      if(time<=known_until+1e-10)
        command=command_at(time>=known_until-1e-10?known_until:time);
      if(future&&time>=known_until-1e-10&&time>=next_future-1e-10){
        command=future(time,x);next_future=time+.02;
      }
      double boundary=std::min(end,time+.005);
      if(time<known_until-1e-10)boundary=std::min(boundary,known_until);
      for(const auto& r:records_)if(r.time>time+1e-10&&r.time<=known_until){boundary=std::min(boundary,r.time);break;}
      if(future&&next_future>time+1e-10)boundary=std::min(boundary,next_future);
      x=step(x,command,boundary-time);time=boundary;
    }
    if(final_command)*final_command=command;
    return x;
  }
 private:
  State step(State x,HistoryCommand c,double dt)const{
    double target=std::clamp(c.speed,0.,cfg_.max_speed);
    if(target<cfg_.minimum_drive_speed)target=0.;
    double speed=std::max(0.,x[3]);
    double a=std::clamp((target-speed)/dt,-cfg_.brake_limit,cfg_.accel_limit);
    auto delta=[&](double t){return c.steering+(x[5]-c.steering)*std::exp(-t/cfg_.steering_tau);};
    auto rhs=[&](double t,double yaw){
      double v=std::max(0.,speed+a*t);
      return std::array<double,3>{v*std::cos(yaw),v*std::sin(yaw),
        v*std::tan(delta(t))/(cfg_.wheelbase*(1+cfg_.understeer_coefficient*v*v))};
    };
    auto k1=rhs(0.,x[2]);auto k2=rhs(dt/2,x[2]+dt*k1[2]/2);
    auto k3=rhs(dt/2,x[2]+dt*k2[2]/2);auto k4=rhs(dt,x[2]+dt*k3[2]);
    for(int i=0;i<3;++i)x[i]+=dt*(k1[i]+2*k2[i]+2*k3[i]+k4[i])/6;
    x[3]=std::max(0.,speed+dt*a);x[5]=delta(dt);
    return x;
  }
  struct Record{double time,estimate;HistoryCommand command;};
  Config cfg_;std::deque<Record> records_;
};
} // namespace aims_mpcc_rt

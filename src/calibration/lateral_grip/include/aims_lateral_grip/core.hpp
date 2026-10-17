#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace aims_lateral_grip {
struct Config {
  bool selected{false};
  double wheelbase{0}, radius{0}, speed_limit{0}, steering_limit{0};
  std::vector<double> speeds;
  double ramp_rate{0}, speed_tolerance{.1}, settle_seconds{1}, hold_seconds{3}, stage_timeout{30};
  int minimum_hold_samples{10};
  double stop_seconds{1}, stop_timeout{15};
  double freshness{.25}, standstill_speed{.05};
  bool valid() const {
    for (double v : {wheelbase,radius,speed_limit,steering_limit,ramp_rate,speed_tolerance,settle_seconds,hold_seconds,stage_timeout,
                    stop_seconds,stop_timeout,freshness,standstill_speed})
      if (!std::isfinite(v)||v<=0) return false;
    if (!selected||minimum_hold_samples<2||stage_timeout<=settle_seconds+hold_seconds||speeds.empty()||stop_timeout<=stop_seconds||
        std::atan(wheelbase/radius)>steering_limit) return false;
    double prev=0;
    for(double v:speeds){if(!std::isfinite(v)||v<=prev||v>speed_limit)return false;prev=v;}
    return true;
  }
};
struct Sample { double source{0}, received{0}; bool valid{false}; };
// Wheel, motor telemetry, RC, authority and selected final command. RC and Bool
// authority have no source header; node uses receipt ROS time and records this.
struct Inputs {
  double clock{0},steady{0},speed{0};
  bool authority{false},exclusive{false},rc_ok{false},telemetry_ok{false},final_ok{false};
  std::array<Sample,5> samples;
};
struct Request {int mode{1};double steering{0},speed{0},current{0};};
enum class State {Idle,Coast,Run,Complete,Aborted};
inline const char* name(State s){switch(s){case State::Idle:return "IDLE";case State::Coast:return "COAST";case State::Run:return "RUN";case State::Complete:return "COMPLETE";case State::Aborted:return "ABORTED";}return "UNKNOWN";}
class Core {
 public:
  explicit Core(Config config):config_(std::move(config)){}
  bool active() const{return state_==State::Run||state_==State::Coast;}
  bool enable(const Inputs&i){
    if(used_||!config_.valid())return false;
    if(!health(i).empty()||std::abs(i.speed)>config_.standstill_speed)return false;
    used_=true;state_=State::Coast;since_=i.steady;stopped_since_=i.steady;standstill_source_since_=i.samples[0].source;
    last_clock_=i.clock;last_steady_=i.steady;reason_="waiting_standstill";return true;
  }
  void stop(const std::string&r){if(active()){incomplete_stage_=state_==State::Run;state_=State::Aborted;reason_=r;}}
  void tick(const Inputs&i){
    if(!active())return;
    if(i.clock<=last_clock_||i.steady<=last_steady_){stop("clock_rollback_or_freeze");return;}
    const double dt=i.steady-last_steady_;
    last_clock_=i.clock;last_steady_=i.steady;
    auto fault=health(i);if(!fault.empty()){stop(fault);return;}
    if(state_==State::Run){
      const double target=config_.speeds.at(stage_);
      request_speed_=std::min(target,request_speed_+config_.ramp_rate*dt);
      const bool at_target=std::abs(i.speed-target)<=config_.speed_tolerance&&request_speed_>=target;
      const bool unique_source=i.samples[0].source>last_wheel_source_;
      if(!at_target){settle_since_=-1;hold_since_=-1;hold_samples_=0;phase_="RAMP_OR_SETTLE";}
      else{
        if(settle_since_<0)settle_since_=i.steady;
        phase_="SETTLE";
        if(i.steady-settle_since_>=config_.settle_seconds){
          if(hold_since_<0){hold_since_=i.steady;hold_samples_=0;}
          phase_="HOLD";
          if(unique_source)++hold_samples_;
          if(i.steady-hold_since_>=config_.hold_seconds&&hold_samples_>=config_.minimum_hold_samples){
            completed_stages_.push_back({direction_,stage_});
            if(stage_+1<config_.speeds.size()){
              ++stage_;since_=i.steady;settle_since_=-1;hold_since_=-1;hold_samples_=0;phase_="RAMP_OR_SETTLE";
            }else{
              state_=State::Coast;since_=i.steady;stopped_since_=-1;phase_="COAST";reason_="coast_to_standstill";
            }
          }
        }
      }
      last_wheel_source_=i.samples[0].source;
      if(state_==State::Run&&i.steady-since_>=config_.stage_timeout)stop("stage_timeout");
    }else{
      if(std::abs(i.speed)>config_.standstill_speed){stopped_since_=-1;standstill_source_since_=-1;}
      else if(stopped_since_<0){stopped_since_=i.steady;standstill_source_since_=i.samples[0].source;}
      if(stopped_since_>=0&&i.steady-stopped_since_>=config_.stop_seconds&&i.samples[0].source-standstill_source_since_>=config_.stop_seconds){
        if(finished_left_&&direction_<0){state_=State::Complete;reason_="completed";return;}
        if(finished_left_){direction_=-1;stage_=0;}
        else finished_left_=true;
        state_=State::Run;since_=i.steady;request_speed_=0;settle_since_=-1;hold_since_=-1;hold_samples_=0;phase_="RAMP_OR_SETTLE";reason_="running";
      }else if(i.steady-since_>=config_.stop_timeout)stop("standstill_timeout");
    }
  }
  Request request()const{if(state_==State::Run)return {0,direction_*std::atan(config_.wheelbase/config_.radius),request_speed_,0};return {};}
  bool incomplete_stage()const{return incomplete_stage_;}
  const std::vector<std::pair<int,std::size_t>>& completed_stages()const{return completed_stages_;}
  const std::string& phase()const{return phase_;}
  State state()const{return state_;} const std::string& reason()const{return reason_;}
  std::size_t stage()const{return stage_;}int direction()const{return direction_;}
 private:
  std::string health(const Inputs&i)const{
    if(!std::isfinite(i.clock)||!std::isfinite(i.steady)||i.clock<=0)return "invalid_clock";
    if(!i.exclusive)return "publisher_conflict";
    if(!i.authority||!i.rc_ok)return "authority_lost";
    if(!i.telemetry_ok||!std::isfinite(i.speed))return "invalid_control_input";
    if(std::abs(i.speed)>config_.speed_limit)return "speed_limit_exceeded";
    if(!i.final_ok)return "final_command_mismatch";
    for(const auto&s:i.samples){
      double source_age=i.clock-s.source,receive_age=i.steady-s.received;
      if(!s.valid||!std::isfinite(source_age)||!std::isfinite(receive_age)||
         source_age<0||receive_age<0||source_age>config_.freshness||receive_age>config_.freshness)
        return "control_input_stale_or_invalid";
    }return {};
  }
  Config config_;State state_{State::Idle};bool used_{false},finished_left_{false};
  std::size_t stage_{0};int direction_{1};double since_{0},stopped_since_{-1},last_clock_{0},last_steady_{0};
  double request_speed_{0},settle_since_{-1},hold_since_{-1},last_wheel_source_{0},standstill_source_since_{-1};
  int hold_samples_{0};bool incomplete_stage_{false};
  std::vector<std::pair<int,std::size_t>> completed_stages_;
  std::string phase_{"IDLE"},reason_{"not_started"};
};
}

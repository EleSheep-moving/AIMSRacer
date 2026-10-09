// Offline audit probe. Compile the actual callbacks using their existing test
// access; no executor spins and all publications retain the shadow namespace.
// Exit 0 means the listed counterexamples reproduced, not runtime acceptance.
#define AIMS_MPCC_RT_TEST_ACCESS
#define main audit_runtime_entry_unused
#include "../src/node.cpp"
#undef main

namespace aims_mpcc_rt {
static void require(bool ok,const char* text){if(!ok)throw std::runtime_error(text);}
struct RuntimeClockProbe {
  static std::string source_history(RuntimeNode& node){
    std::lock_guard<std::mutex> lock(node.mutex_);const double now=steady();
    const auto r=node.bundle_.reference().at(0.);
    node.enabled_=false;node.snapshot_={{r.x,r.y,r.yaw,0.,0.,0.},{},{},now-.05,0.,now,true};
    node.mode_=true;node.mode_received_=now;node.history_->clear();node.history_->record(now,0.,0.,0.,0.);
    bool covered=true;try{node.history_->command_at(node.snapshot_.source);}catch(const std::exception&){covered=false;}
    require(node.fresh_locked(now)&&!covered,"source coverage counterexample did not reproduce");
    return "{\"fresh_gate\":true,\"history_covers_source\":false}";
  }
  static std::string clock_epoch(RuntimeNode& node){
    const auto r=node.bundle_.reference().at(0.);const double old=node.get_clock()->now().seconds()+100.;
    {
      std::lock_guard<std::mutex> lock(node.mutex_);const double now=steady();
      node.snapshot_={{r.x,r.y,r.yaw,0.,0.,0.},{},{},now,old,now,true};node.enabled_=true;
    }
    for(int i=0;i<2;++i){
      {std::lock_guard<std::mutex> lock(node.mutex_);node.history_->record(steady(),0.,0.,0.,0.);}
      nav_msgs::msg::Odometry m;m.header.frame_id="odom";m.child_frame_id="base_link";
      m.header.stamp=node.get_clock()->now();m.pose.pose.orientation.w=1.;
      m.pose.pose.position.x=r.x+node.cfg_.rear_offset*std::cos(r.yaw);
      m.pose.pose.position.y=r.y+node.cfg_.rear_offset*std::sin(r.yaw);
      m.pose.pose.orientation.z=std::sin(r.yaw/2);m.pose.pose.orientation.w=std::cos(r.yaw/2);
      node.odometry(m);
    }
    std::lock_guard<std::mutex> lock(node.mutex_);
    require(node.snapshot_.ros_source==old&&node.history_->newest()<0.,"clock reset counterexample did not reproduce");
    return "{\"old_source_retained\":true,\"history_cleared_twice\":true,\"stored_epoch_ahead_s\":100}";
  }
  static std::string delivery_clock(RuntimeNode& node){
    const auto r=node.bundle_.reference().at(0.);
    {std::lock_guard<std::mutex> lock(node.mutex_);const double now=steady();
      node.snapshot_={{r.x,r.y,r.yaw,0.,0.,0.},{},{},now,0.,now,true};
      node.history_->clear();node.history_->record(now,0.,0.,0.,0.);
      node.sampler_->reset(0.,0.,now);node.mode_=true;node.mode_received_=now;
      node.enabled_=true;node.stopping_=false;node.phase_="RUNNING";node.started_=now;node.generation_++;
    }
    node.cv_.notify_all();const double deadline=steady()+2.;
    std::unique_lock<std::mutex> lock(node.mutex_,std::defer_lock);
    while(steady()<deadline){
      lock.lock();if(node.requests_>0)break;
      const double now=steady();node.snapshot_.source=node.snapshot_.received=node.mode_received_=now;
      node.history_->record(now,0.,0.,0.,0.);lock.unlock();std::this_thread::yield();
    }
    require(lock.owns_lock()&&node.requests_==1,"worker did not submit first request");
    require(node.last_.sequence!=1&&!node.pending_,"worker delivered before the probe acquired the mutex; no forced-wait evidence");
    const double held=steady();
    // Let the independent worker finish while result delivery waits for state.
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    node.snapshot_.source=node.snapshot_.received=node.mode_received_=steady();
    node.history_->record(steady(),0.,0.,0.,0.);lock.unlock();
    while(steady()<deadline){lock.lock();if(node.last_.sequence==1)break;lock.unlock();std::this_thread::yield();}
    require(lock.owns_lock()&&node.last_.sequence==1,"worker did not deliver result");
    const double observed=steady()-held,reported=node.last_complete_;
    const bool accepted=node.pending_&&node.pending_->sequence==1;
    node.enabled_=false;node.pending_.reset();
    require(reported<node.budget_&&observed>.055&&accepted,"delivery deadline counterexample did not reproduce");
    std::ostringstream out;out<<std::setprecision(17)<<"{\"reported_complete_s\":"<<reported
      <<",\"observed_delivery_lower_bound_s\":"<<observed<<",\"budget_s\":"<<node.budget_
      <<",\"configured_frequency_hz\":"<<node.frequency_
      <<",\"pre_hold_delivery_absent\":true,\"pending_accepted\":true}";return out.str();
  }
};

static std::string execution_scope(const Bundle& b){
  const auto& cfg=b.config();Core core(b);const auto r=b.reference().at(0.);
  require(cfg.horizon==10&&cfg.dt==.1&&!cfg.envelope_soft_enabled,"execution probe requires strict N10/100 ms bundle");
  const double now=10.02;State initial{r.x,r.y,r.yaw,.79,0.,.4};Applied applied{.4,.4,0.};
  Plan source;source.success=true;source.status=0;source.dt=cfg.dt;
  source.source_epoch=now-.02;source.forecast_epoch=now;source.artifact_fingerprint=b.fingerprint();
  source.initial_applied=applied;source.states.push_back(initial);
  for(int k=0;k<cfg.horizon;++k){
    Control u{std::max(0.,.3-.1*k),.4,.79};source.controls.push_back(u);
    source.states.push_back(core.transition(source.states.back(),u,.4,cfg.dt));
  }
  auto candidate=core.reanchor(source,initial,applied,now,Alignment{});
  require(candidate.success&&!candidate.prefix_transported,"macro candidate counterexample is not feasible");
  OutputSampler sampler(cfg);Plan prime=source;prime.forecast_epoch=now-.42;prime.source_epoch=now-.44;
  for(auto& u:prime.controls)u[0]=.4;
  sampler.reset(.706,.4,now-.42);sampler.set_previous_endpoint(.4);
  OutputCommand command;
  for(int i=1;i<=20;++i)command=sampler.sample(&prime,now-.42+.02*i,initial,false,100.);
  require(std::abs(command.continuous_speed-.79)<1e-10,"sampler speed prefix priming mismatch");
  State physical=initial;double maximum=0.;std::ostringstream trace;trace<<std::setprecision(17)<<'[';
  for(int i=0;i<5*cfg.horizon;++i){
    command=sampler.sample(&candidate,now+.02*i,physical,false,100.);
    if(i)trace<<',';
    trace<<'['<<command.continuous_speed<<','<<command.steering<<','<<command.continuous_acceleration<<','<<command.steering_rate<<']';
    auto utilization=[&](const State& state){
      const double ay=state[3]*state[3]*std::tan(state[5])/(cfg.wheelbase*(1+cfg.understeer_coefficient*state[3]*state[3]));
      const double axis=command.continuous_acceleration>=0?cfg.envelope_accel:cfg.envelope_brake;
      return std::pow(ay/cfg.lateral_accel_limit,2)+std::pow(command.continuous_acceleration/axis,2);
    };
    maximum=std::max(maximum,utilization(physical));
    // Independent exact longitudinal/first-order steering integration; these
    // components determine envelope utilization without spatial approximation.
    physical[3]+=.02*command.continuous_acceleration;
    physical[5]=command.steering+(physical[5]-command.steering)*std::exp(-.02/cfg.steering_tau);
    maximum=std::max(maximum,utilization(physical));
  }
  trace<<']';require(maximum>1.0001,"executed envelope counterexample did not reproduce");
  std::ostringstream out;out<<std::setprecision(17)<<"{\"macro_candidate_accepted\":true,\"macro_violation\":"<<candidate.max_violation
    <<",\"maximum_executed_utilization\":"<<maximum<<",\"trace\":"<<trace.str()<<'}';return out.str();
}
}
int main(int argc,char** argv){
  if(argc!=3)throw std::invalid_argument("synthetic runtime bundle and measured sampler bundle required");
  std::vector<std::string> args{"audit","--ros-args","-p",std::string("artifact_directory:=")+argv[1],"-p","simulation:=true","-p","repeat_laps:=true","-p","shadow:=true","-p","solve_frequency:=10.0"};
  std::vector<char*> raw;for(auto& s:args)raw.push_back(s.data());rclcpp::init(raw.size(),raw.data());
  try{
    auto node=std::make_shared<aims_mpcc_rt::RuntimeNode>();
    const auto coverage=aims_mpcc_rt::RuntimeClockProbe::source_history(*node);
    const auto clock=aims_mpcc_rt::RuntimeClockProbe::clock_epoch(*node);
    const auto delivery=aims_mpcc_rt::RuntimeClockProbe::delivery_clock(*node);
    node.reset();
    const auto execution=aims_mpcc_rt::execution_scope(aims_mpcc_rt::Bundle::load(argv[2]));
    std::cout<<"AUDIT_JSON {\"history\":"<<coverage<<",\"clock\":"<<clock<<",\"delivery\":"<<delivery<<",\"execution\":"<<execution<<"}\n";
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}

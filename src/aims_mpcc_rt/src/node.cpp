#include "aims_mpcc_rt/core.hpp"
#include "aims_mpcc_rt/health.hpp"
#include "aims_mpcc_rt/history.hpp"
#include "aims_mpcc_rt/output.hpp"
#include "aims_mpcc_rt/execution.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>

namespace aims_mpcc_rt {
using Steady=std::chrono::steady_clock;
using Drive=ackermann_msgs::msg::AckermannDriveStamped;
static double steady(){return std::chrono::duration<double>(Steady::now().time_since_epoch()).count();}
static double wrap_angle(double a){return std::atan2(std::sin(a),std::cos(a));}
static std::string json_string(const std::string& text){
  std::ostringstream out;out<<'"';
  for(unsigned char c:text){
    if(c=='"'||c=='\\')out<<'\\'<<char(c);
    else if(c=='\n')out<<"\\n";else if(c=='\r')out<<"\\r";else if(c=='\t')out<<"\\t";
    else if(c<32)out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;
    else out<<char(c);
  }
  out<<'"';return out.str();
}
static double yaw(double x,double y,double z,double w){
  double norm=std::sqrt(x*x+y*y+z*z+w*w);
  if(!std::isfinite(norm)||norm<1e-12)throw std::invalid_argument("invalid quaternion");
  x/=norm;y/=norm;z/=norm;w/=norm;
  return std::atan2(2*(w*z+x*y),1-2*(y*y+z*z));
}
struct Snapshot {
  State state{};Applied applied{};Alignment alignment{};
  double source{},ros_source{},received{};bool present=false;
  std::string epoch;std::int64_t anchor_sequence=-1,anchor_stamp_ns=-1;
};
struct Pending {Plan plan;Applied applied{};HistoryCommand prefix{};std::uint64_t generation{},sequence{};double submitted{};};

class RuntimeNode final:public rclcpp::Node {
#ifdef AIMS_MPCC_RT_TEST_ACCESS
  friend struct RuntimeClockProbe;
  std::atomic<bool> test_callback_entered_{false},test_compute_complete_{false},test_block_worker_{false},test_worker_blocked_{false};
#endif
 public:
  RuntimeNode():Node("aims_mpcc_rt"){
    auto artifact=declare_parameter<std::string>("artifact_directory","");
    auto config=declare_parameter<std::string>("vehicle_config","");
    auto reference=declare_parameter<std::string>("path_directory","");
    if(artifact.empty())throw std::invalid_argument("artifact_directory must name an offline-built bundle");
    bundle_=Bundle::load(artifact,config,reference,declare_parameter<std::string>("source_root",""));
    cfg_=bundle_.config();
    if(cfg_.command_profile=="rate_bounded_v2"&&std::abs(cfg_.dt-.1)>1e-12)
      throw std::invalid_argument("rate_bounded_v2 requires the qualified 100 ms solver mesh; other meshes need held-output qualification");
    history_=std::make_unique<AppliedHistory>(cfg_);
    if(cfg_.envelope_soft_enabled)
      throw std::invalid_argument("experimental soft-envelope profile requires an independent recovery validator; use the strict vehicle profile");
    sampler_=std::make_unique<OutputSampler>(cfg_);
    activation_validator_=std::make_unique<Core>(bundle_);
    simulation_=declare_parameter<bool>("simulation",false);
    if(!simulation_&&cfg_.profile!="measured")throw std::invalid_argument("synthetic vehicle profile requires simulation:=true");
    if(!simulation_&&!bundle_.reference().recording_verified())throw std::invalid_argument("drive requires a closed reference with matching recorded vehicle geometry");
    shadow_=declare_parameter<bool>("shadow",true);
    repeat_laps_=declare_parameter<bool>("repeat_laps",false);
    frequency_=declare_parameter<double>("solve_frequency",20.);
    ttl_=declare_parameter<double>("plan_ttl",.8*cfg_.horizon*cfg_.dt);
    auto requested_horizon=declare_parameter<int>("horizon",cfg_.horizon);
    auto requested_rti=declare_parameter<int>("acados_rti_steps",cfg_.acados_rti_steps);
    if(requested_rti!=cfg_.acados_rti_steps)throw std::invalid_argument("acados_rti_steps conflicts with verified artifact");
    if(requested_horizon!=cfg_.horizon)throw std::invalid_argument("horizon conflicts with verified artifact");
    budget_=declare_parameter<double>("solver_timeout",.05);
    lead_=declare_parameter<double>("handover_delay",.02);
    auto directory=declare_parameter<std::string>("log_directory","");
    if(!std::isfinite(frequency_)||frequency_<=0||frequency_>50||!std::isfinite(ttl_)||ttl_<=0||
       !std::isfinite(budget_)||budget_<=0||!std::isfinite(lead_)||lead_<0||lead_>=ttl_)
      throw std::invalid_argument("invalid timing parameters");
    if(ttl_<=1./frequency_+std::max(budget_,lead_)+.02||ttl_>cfg_.horizon*cfg_.dt+lead_)
      throw std::invalid_argument("plan_ttl must cover scheduling and fit inside the prediction horizon");
    const std::string prefix=shadow_?"/mpcc_rt_shadow":"";
    pub_=create_publisher<Drive>(prefix+"/drive",10);
    status_pub_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>(prefix+"/mpcc/status",10);
    reference_pub_=create_publisher<nav_msgs::msg::Path>(prefix+"/mpcc/reference",rclcpp::QoS(1).transient_local());
    prediction_pub_=create_publisher<nav_msgs::msg::Path>(prefix+"/mpcc/prediction",10);
    receive_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    output_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;options.callback_group=receive_group_;
    odom_=create_subscription<nav_msgs::msg::Odometry>(declare_parameter<std::string>("odom_topic","/odometry/filtered"),
      rclcpp::SensorDataQoS().keep_last(1),[this](nav_msgs::msg::Odometry::ConstSharedPtr m){odometry(*m);},options);
    mode_sub_=create_subscription<std_msgs::msg::Bool>("/control/autonomy_speed_enabled",10,
      [this](std_msgs::msg::Bool::ConstSharedPtr m){std::lock_guard<std::mutex> lock(mutex_);
        if(mode_&&!m->data&&enabled_)fault_locked("Autonomy withdrawn; re-enable required");
        mode_=m->data;mode_received_=steady();},options);
    command_sub_=create_subscription<Drive>("/ackermann_cmd",10,
      [this](Drive::ConstSharedPtr m){forwarded(*m);},options);
    if(bundle_.reference().frame_id()=="map"){
      tf_=std::make_unique<tf2_ros::Buffer>(get_clock());
      listener_=std::make_shared<tf2_ros::TransformListener>(*tf_,this,false);
      identity_sub_=create_subscription<std_msgs::msg::String>("/localization/map_sha256",rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr m){std::lock_guard<std::mutex> lock(mutex_);identity_=m->data;
          if(enabled_&&identity_!=bundle_.reference().map_sha256())fault_locked("Map identity mismatch");},options);
      health_sub_=create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/localization/status",10,
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m){LocalizationHealth::Values v;int count=0;
          for(const auto& s:m->status)if(s.name=="aims_racer_system/localization"){
            ++count;for(const auto& kv:s.values)v[kv.key]=kv.value;}
          if(count!=1)v.clear();accept_health(v);},options);
    }
    enable_=create_service<std_srvs::srv::SetBool>(prefix+"/mpcc/enable",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr request,std_srvs::srv::SetBool::Response::SharedPtr response){
        enable_request(request,response);},rmw_qos_profile_services_default,receive_group_);
    output_timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{publish_command();},output_group_);
    status_timer_=create_wall_timer(std::chrono::milliseconds(100),[this]{publish_status();},output_group_);
    parameter_guard_=add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter>& parameters){
      rcl_interfaces::msg::SetParametersResult result;result.successful=true;
      for(const auto& parameter:parameters)if(parameter.get_name()!="use_sim_time"){
        result.successful=false;result.reason="runtime configuration is fixed; restart with a matching offline bundle";break;
      }
      return result;
    });
    publish_reference();
    if(!directory.empty()){
      std::filesystem::create_directories(directory);
      log_.open(std::filesystem::path(directory)/"runtime.csv",std::ios::out|std::ios::trunc);
      if(!log_)throw std::runtime_error("cannot open runtime log");
      log_<<"event,steady_s,sequence,source_epoch,forecast_epoch,submitted,complete_s,core_s,native_s,preparation_s,validation_s,accepted,status,passes,violation,observation_age_s,publish_gap_s,speed,steering,reason,compute_s,delivery_wait_s,delivery_s,disposition,requested_acceleration,emitted_acceleration,requested_steering_rate,emitted_steering_rate,limiter_active\n";
      logger_=std::thread([this]{log_loop();});
    }
    // Process-lifetime provenance, captured before any executor callback can
    // accept an explicit enable. Later status may first be discovered RUNNING.
    if(const char* instance=std::getenv("AIMS_MPCC_RT_SUPERVISOR_INSTANCE"))startup_instance_=instance;
    startup_disabled_=!enabled_&&!stopping_&&last_output_.speed==0.&&sampler_->continuous_speed()==0.;
    worker_=std::thread([this]{solve_loop();});
    RCLCPP_INFO(get_logger(),"acados runtime ready: %.1f Hz solve, 50 Hz output, shadow=%s",frequency_,shadow_?"true":"false");
  }
  ~RuntimeNode()override{
    {std::lock_guard<std::mutex> lock(mutex_);shutdown_=true;}cv_.notify_all();
    if(worker_.joinable())worker_.join();
    {std::lock_guard<std::mutex> lock(log_mutex_);
      // Reserve one final record even if the bounded normal queue is full.
      std::ostringstream summary;summary<<std::setprecision(17)<<"summary,"<<steady()<<','<<requests_
        <<",0,0,0,0,0,0,0,0,1,0,0,"<<log_dropped_.load()<<",0,0,0,0,\n";
      if(log_.is_open()){auto line=summary.str();line.pop_back();line+=",0,0,0,,0,0,0,0,0\n";log_queue_.push_back(std::move(line));}
      log_shutdown_=true;
    }log_cv_.notify_all();
    if(logger_.joinable())logger_.join();
  }
 private:
  void enable_request(const std_srvs::srv::SetBool::Request::SharedPtr request,
      const std_srvs::srv::SetBool::Response::SharedPtr response){
        std::lock_guard<std::mutex> lock(mutex_);const double now=steady();
        try{
          if(request->data){
            if(!solver_ready_)throw std::runtime_error("Solver not ready");
            if(!fresh_locked(now))throw std::runtime_error("fresh state, selector authority and actual input history required");
            if(!localization_locked(now))throw std::runtime_error(cfg_.command_profile=="rate_bounded_v2"&&!health_.qualified()&&bundle_.reference().frame_id()=="map"?
              "Incompatible localization protocol: qualified alignment payload required":"matching map, anchor snapshot and fresh trusted localization required");
            if(count_publishers(pub_->get_topic_name())!=1)throw std::runtime_error("controller must be sole command publisher");
            if(std::abs(snapshot_.state[3])>.1)throw std::runtime_error("start requires stationary vehicle");
            auto p=map_point(snapshot_.state,snapshot_.alignment);
            double theta=bundle_.reference().project(p[0],p[1]);auto r=bundle_.reference().at(theta);
            if(std::abs(wrap_angle(p[2]-r.yaw))>std::acos(-1.)/6)throw std::runtime_error("start heading outside 30 degrees");
            if(!footprint_locked(p,theta))throw std::runtime_error("start footprint outside corridor");
            enabled_=true;stopping_=false;phase_="RUNNING";reason_.clear();generation_++;
            start_progress_=unwrapped_progress_=last_progress_=theta;progress_ready_=true;
            started_=now;stationary_since_=-1.;recovery_good_=0;active_.reset();cancel_pending_locked("enable generation changed");sampler_->reset(0.,snapshot_.applied[1],now);
          }else if(enabled_){stopping_=true;phase_="STOPPING";reason_="Operator stop requested";generation_++;cancel_pending_locked("operator stop");}
          response->success=true;response->message=phase_;
          if(request->data)++explicit_enable_count_;
        }catch(const std::exception& e){response->success=false;response->message=e.what();}
        cv_.notify_all();  }
  static State map_point(State x,const Alignment& a){
    double c=std::cos(a[2]),s=std::sin(a[2]);double px=x[0],py=x[1];
    x[0]=a[0]+c*px-s*py;x[1]=a[1]+s*px+c*py;x[2]+=a[2];return x;
  }
  bool fresh_locked(double now)const{
    return snapshot_.present&&history_->covers(snapshot_.source)&&mode_&&now>=snapshot_.source&&now-snapshot_.source<=.1&&
      now>=snapshot_.received&&now-snapshot_.received<=.1&&
      now>=mode_received_&&now-mode_received_<=.1&&now>=history_->newest()&&now-history_->newest()<=.1;
  }
  bool localization_locked(double now){return bundle_.reference().frame_id()!="map"||
    (identity_==bundle_.reference().map_sha256()&&health_.usable(get_clock()->now().nanoseconds(),now)&&
     snapshot_.epoch==health_.epoch()&&snapshot_.anchor_sequence==health_.anchor_sequence()&&
     snapshot_.anchor_stamp_ns==health_.anchor_stamp_ns()&&
     (cfg_.command_profile!="rate_bounded_v2"||health_.qualified()));}
  void cancel_pending_locked(const std::string& reason){
    if(pending_){log_activation(*pending_,steady(),false,reason);++pending_cancelled_;pending_.reset();}
  }
  void fault_locked(const std::string& reason){
    if(phase_!="FAULT"||enabled_){reason_=reason;fault_steady_=steady();}
    enabled_=false;stopping_=false;phase_="FAULT";generation_++;active_.reset();cancel_pending_locked(reason);
  }
  void reset_clock_locked(double ros,double now){
    fault_locked("ROS clock moved backwards; fresh inputs and explicit re-enable required");
    snapshot_={};history_->clear();recent_.clear();health_.reset();identity_.clear();
    progress_ready_=false;start_progress_=unwrapped_progress_=last_progress_=0.;
    mode_=false;mode_received_=-1.;started_=last_publish_=0.;stationary_since_=-1.;recovery_good_=0;
    last_={};last_output_={};last_decision_state_={};last_decision_cap_=0.;sampler_->reset(0.,0.,now);last_ros_now_=ros;
  }
  void observe_clock_locked(double ros,double now){
    if(std::isfinite(last_ros_now_)&&ros<last_ros_now_-.001)reset_clock_locked(ros,now);
    last_ros_now_=ros;
  }
  void accept_health(const LocalizationHealth::Values& values){
    std::lock_guard<std::mutex> lock(mutex_);double now=steady();auto ros=get_clock()->now();
    observe_clock_locked(ros.seconds(),now);
    auto map=values.find("map_sha256");
    if(bundle_.reference().frame_id()=="map"&&cfg_.command_profile=="rate_bounded_v2"&&
       (map==values.end()||map->second!=bundle_.reference().map_sha256())){
      // Do not commit a transform under an incompatible map identity.
      health_.observe({},ros.nanoseconds(),now);
      snapshot_={};fault_locked("Localization alignment map identity mismatch");return;
    }
    bool changed=health_.observe(values,ros.nanoseconds(),now);
    if(changed){snapshot_={};fault_locked("Localization epoch changed; fresh anchor state and explicit re-enable required");}
    else {
      // Odom coordinates remain valid across a trusted anchor update within
      // one localization epoch. Adopt the atomic alignment record without
      // refreshing or relabeling the physical measurement/history clocks.
      if(snapshot_.present&&snapshot_.epoch==health_.epoch()&&health_.qualified()&&
         health_.usable(ros.nanoseconds(),now)&&now>=snapshot_.source&&now-snapshot_.source<=.1&&
         now>=snapshot_.received&&now-snapshot_.received<=.1&&history_->covers(snapshot_.source)){
        snapshot_.alignment=health_.alignment();snapshot_.anchor_sequence=health_.anchor_sequence();
        snapshot_.anchor_stamp_ns=health_.anchor_stamp_ns();
      }
      if(enabled_&&(!health_.usable(ros.nanoseconds(),now)||
      (cfg_.command_profile=="rate_bounded_v2"&&!health_.qualified())))fault_locked("Trusted localization unavailable");
    }
  }
  bool footprint_locked(const State& mapped,double theta)const{
    if(!cfg_.enforce_corridor)return true;
    const auto r=bundle_.reference().at(theta);const double nx=-std::sin(r.yaw),ny=std::cos(r.yaw);
    for(double longitudinal:{-cfg_.rear_extent,cfg_.front_extent})for(double lateral:{-cfg_.half_width,cfg_.half_width}){
      double px=mapped[0]+std::cos(mapped[2])*longitudinal-std::sin(mapped[2])*lateral;
      double py=mapped[1]+std::sin(mapped[2])*longitudinal+std::cos(mapped[2])*lateral;
      double cross=(px-r.x)*nx+(py-r.y)*ny;
      if(cross>bundle_.reference().left_width()||cross<-bundle_.reference().right_width())return false;
    }
    return true;
  }
  void odometry(const nav_msgs::msg::Odometry& m){
    const double now=steady(),ros=get_clock()->now().seconds();
    std::lock_guard<std::mutex> lock(mutex_);observe_clock_locked(ros,now);
    try{
      if(m.header.frame_id!="odom"||m.child_frame_id!="base_link")throw std::runtime_error("expected odom/base_link state");
      double stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,age=ros-stamp;
      if(!std::isfinite(age)||age<0||age>.1)throw std::runtime_error("stale or future odometry");
      auto q=m.pose.pose.orientation;
      const double norm=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
      if(!std::isfinite(norm)||std::abs(norm-1.)>.01)throw std::runtime_error("nonunit odometry quaternion");
      double heading=yaw(q.x,q.y,q.z,q.w);
      State x{m.pose.pose.position.x-cfg_.rear_offset*std::cos(heading),
              m.pose.pose.position.y-cfg_.rear_offset*std::sin(heading),heading,m.twist.twist.linear.x,0.,0.};
      if(!std::all_of(x.begin(),x.end(),[](double v){return std::isfinite(v);})||x[3]<-.05)
        throw std::runtime_error("invalid or reverse state");
      Alignment alignment{};
      if(tf_&&health_.qualified())alignment=health_.alignment();
      else if(tf_&&cfg_.command_profile=="rate_bounded_v2")throw std::runtime_error("Incompatible localization protocol: qualified alignment payload required");
      else if(tf_){auto t=tf_->lookupTransform("map","odom",tf2::TimePointZero);
        auto r=t.transform.rotation;alignment={t.transform.translation.x,t.transform.translation.y,yaw(r.x,r.y,r.z,r.w)};
        if(!std::all_of(alignment.begin(),alignment.end(),[](double v){return std::isfinite(v);}))throw std::runtime_error("nonfinite map alignment");}
      if(snapshot_.present&&stamp<snapshot_.ros_source)return; // reordered source message, not a ROS clock reset
      history_->command_at(now-age); // require actual source coverage before estimating steering

      if(enabled_&&snapshot_.present){double dt=std::max(0.,stamp-snapshot_.ros_source);
        if(std::hypot(x[0]-snapshot_.state[0],x[1]-snapshot_.state[1])>cfg_.max_speed*dt+.15||
           std::abs(wrap_angle(x[2]-snapshot_.state[2]))>.5)throw std::runtime_error("localization discontinuity");}
      auto applied=history_->at(now-age);x[3]=std::max(0.,x[3]);x[5]=applied.first;
      snapshot_={x,applied.second,alignment,now-age,stamp,now,true,health_.epoch(),health_.anchor_sequence(),health_.anchor_stamp_ns()};
    }catch(const std::exception& e){if(enabled_)fault_locked(e.what());}
  }
  void forwarded(const Drive& m){
    if(m.drive.jerk!=0.)return;
    double now=steady();std::lock_guard<std::mutex> lock(mutex_);
    observe_clock_locked(get_clock()->now().seconds(),now);
    try{
      double accel=0.,rate=0.;
      for(auto it=recent_.rbegin();it!=recent_.rend();++it)if(now>=it->first&&now-it->first<=.1&&
         std::abs(it->second.speed-m.drive.speed)<1e-6&&std::abs(it->second.steering-m.drive.steering_angle)<1e-6){
        accel=it->second.acceleration;rate=it->second.steering_rate;break;}
      history_->record(now,std::clamp(double(m.drive.steering_angle),-cfg_.steer_limit,cfg_.steer_limit),m.drive.speed,accel,rate);
    }catch(const std::exception& e){if(enabled_)fault_locked(e.what());}
  }
  void solve_loop(){
    try{
      Core core(bundle_);std::uint64_t seen_generation=0;double previous_submit=0.,next=steady();
      const char* capture_env=std::getenv("AIMS_MPCC_CAPTURE_REQUEST");
      const bool capture_requests=capture_env&&std::string(capture_env)=="1";
      {std::lock_guard<std::mutex> lock(mutex_);solver_ready_=true;}
      while(true){
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_until(lock,Steady::time_point(std::chrono::duration_cast<Steady::duration>(std::chrono::duration<double>(next))),[this]{return shutdown_;});
        if(shutdown_)return;
        double submitted=steady();next=submitted+1./frequency_;
        if(pending_){++skipped_pending_slots_;
          std::ostringstream skip;skip<<std::setprecision(17)<<"skipped_pending,"<<submitted<<','<<pending_->sequence
            <<",0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,pending owns takeover\n";queue_log(skip.str());continue;}
        if(!enabled_||stopping_||!fresh_locked(submitted)||!localization_locked(submitted))continue;
        Snapshot snapshot=snapshot_;auto history=*history_;auto sampler=*sampler_;
        auto previous=active_;auto generation=generation_;auto sequence=++requests_;worker_busy_=true;worker_submitted_=submitted;
        const bool recovering=phase_=="RECOVERING";
        std::ostringstream submission;submission<<std::setprecision(17)<<"submission,"<<submitted<<','<<sequence
          <<','<<snapshot.source<<','<<submitted+lead_<<','<<submitted<<",0,0,0,0,0,0,0,0,0,0,0,0,0,\n";
        queue_log(submission.str());
        double progress=unwrapped_progress_,last_progress=last_progress_,start_progress=start_progress_;lock.unlock();
        const bool reset_core=generation!=seen_generation;
        if(reset_core){core.reset();seen_generation=generation;previous_submit=0.;}
        double forecast=submitted+lead_;State initial{};
        double elapsed=previous_submit?submitted-previous_submit:1./frequency_,core_budget=0.;
        std::vector<double> targets;bool request_captured=false;
        Pending candidate;candidate.generation=generation;candidate.sequence=sequence;candidate.submitted=submitted;
        candidate.plan.source_epoch=snapshot.source;candidate.plan.forecast_epoch=forecast;
        try{
          initial=history.predict(snapshot.state,snapshot.source,forecast,submitted,
            [&](double t,const State& x){auto command=sampler.sample(previous?&previous->plan:nullptr,t,x,false,ttl_);
              return HistoryCommand{command.speed,command.steering,command.acceleration,command.steering_rate};},&candidate.prefix);
          candidate.applied={candidate.prefix.acceleration,candidate.prefix.steering,candidate.prefix.steering_rate};
          previous_submit=submitted;
          if(recovering)targets.assign(cfg_.horizon+1,0.);
          else if(!repeat_laps_){
            auto mapped=map_point(initial,snapshot.alignment);
            auto theta=core.reference().project(mapped[0],mapped[1]);
            targets=core.reference().speed_refs(theta,cfg_.horizon,cfg_.dt);
            double forecast_progress=progress+std::remainder(theta-last_progress,core.reference().length());
            double remaining=start_progress+core.reference().length()-forecast_progress;
            for(auto& target:targets){target=std::min(target,finish_cap(remaining));remaining-=target*cfg_.dt;}
          }
          core_budget=std::max(.000001,budget_-(steady()-submitted));
          if(capture_requests){log_request_snapshot(candidate,initial,snapshot.alignment,elapsed,targets,core_budget,reset_core,true,"");request_captured=true;}
#ifdef AIMS_MPCC_RT_TEST_ACCESS
          if(test_block_worker_.load()){
            test_worker_blocked_.store(true);
            // Model a native call that cannot be preempted by thread shutdown.
            // Only the process supervisor can bound termination in this test.
            while(test_block_worker_.load())std::this_thread::sleep_for(std::chrono::milliseconds(2));
            test_worker_blocked_.store(false);
          }
#endif
          candidate.plan=core.solve(initial,candidate.applied,snapshot.alignment,elapsed,
             core_budget,true,snapshot.source,forecast,targets);
        }catch(const std::exception& e){candidate.plan.reason=e.what();candidate.plan.source_epoch=snapshot.source;
          candidate.plan.forecast_epoch=forecast;candidate.plan.status=-1;
          if(capture_requests&&!request_captured)log_request_snapshot(candidate,initial,snapshot.alignment,elapsed,targets,core_budget,reset_core,false,e.what());}
        if(capture_requests)log_request_validation(candidate);
        const double computed=steady();
#ifdef AIMS_MPCC_RT_TEST_ACCESS
        test_compute_complete_.store(true);
#endif
        lock.lock();const double acquired=steady();
        bool accepted=candidate.plan.success&&enabled_&&!stopping_&&generation==generation_&&!pending_;
        std::string disposition=candidate.plan.success?"delivered_accepted":"solver_validation_failed";
        if(generation!=generation_||!enabled_||stopping_){accepted=false;disposition="generation_cancelled";candidate.plan.reason="request authority or generation changed";++cancelled_;}
        // Copy/allocate before the measured completion endpoint. Ownership and
        // deadline disposition are decided under the same state lock.
        auto owned=accepted?std::make_shared<Pending>(candidate):nullptr;
        last_=candidate;last_observation_age_=submitted-snapshot.source;
        if(accepted)pending_=std::move(owned);
        worker_busy_=false;
        double delivered=steady(),complete=delivered-submitted;
        if(complete>budget_){if(accepted)pending_.reset();accepted=false;disposition="deadline_expired";candidate.plan.reason="request_deadline";++late_;}
        if(accepted){++delivered_accepted_;}
        else {recovery_good_=0;++rejected_;if(!candidate.plan.success)++failed_;}
        last_=candidate;last_complete_=complete;last_compute_=computed-submitted;last_delivery_wait_=acquired-computed;
        delivered=steady();complete=delivered-submitted;
        if(accepted&&complete>budget_){pending_.reset();accepted=false;--delivered_accepted_;++rejected_;++late_;
          disposition="deadline_expired";candidate.plan.reason="request_deadline";last_=candidate;
          delivered=steady();complete=delivered-submitted;}
        last_complete_=complete;lock.unlock();
        log_request(candidate,delivered,complete,accepted,submitted-snapshot.source,computed-submitted,acquired-computed,disposition);

      }
    }catch(const std::exception& e){std::lock_guard<std::mutex> lock(mutex_);worker_busy_=false;solver_ready_=false;fault_locked(std::string("Solver unavailable: ")+e.what());}
  }
  bool activate_locked(double now,const State& observed,double speed_cap){
    if(!pending_||now<pending_->plan.forecast_epoch)return false;
    auto next=pending_;pending_.reset();
    const double begin=steady();
    auto reject=[&](const std::string& reason,bool reset_recovery=true){
      if(reset_recovery)recovery_good_=0;
      ++handover_rejected_;reason_=reason;log_activation(*next,begin,false,reason);return false;
    };
    try{
    if(next->generation!=generation_)return reject("Plan takeover generation cancelled");
    if(stopping_||!enabled_)return reject("Plan takeover authority cancelled");
    if(now-next->plan.source_epoch>ttl_)return reject("Plan takeover source TTL expired");
    if(!fresh_locked(now)||!localization_locked(now))return reject("Plan takeover input expired");
    // Until this callback, the new plan has never owned the actuator. Compare
    // the forecast with real old-plan commands, not unexecuted new controls.
    State expected=history_->predict(next->plan.states.front(),next->plan.forecast_epoch,now,now);
    if(std::hypot(expected[0]-observed[0],expected[1]-observed[1])>.3||
       std::abs(wrap_angle(expected[2]-observed[2]))>std::acos(-1.)/6||
       std::abs(expected[3]-observed[3])>.3||std::abs(expected[5]-observed[5])>std::acos(-1.)/9){
      return reject("Plan takeover state mismatch");}
    const auto actual=history_->command_at(now);
    if(std::abs(actual.speed-next->prefix.speed)>.3||
       std::abs(actual.steering-next->prefix.steering)>std::acos(-1.)/9){
      return reject("Plan takeover command mismatch");}
    auto applied=history_->at(now).second;
    // Opt-in forensic snapshots: record the exact immutable request and actual
    // prefix used for takeover. Formatting stays off in ordinary operation.
    static const bool capture_takeover=std::getenv("AIMS_MPCC_CAPTURE_TAKEOVER")!=nullptr;
    std::ostringstream capture;
    if(capture_takeover){
      capture<<std::setprecision(17)<<"{\"sequence\":"<<next->sequence
        <<",\"original_forecast_epoch\":"<<next->plan.forecast_epoch<<",\"now\":"<<now
        <<",\"source_epoch\":"<<next->plan.source_epoch<<",\"speed_cap\":"<<speed_cap<<",\"original_violation\":"<<next->plan.max_violation;
      auto array=[&](const auto& values){capture<<'[';bool first=true;for(auto v:values){if(!first)capture<<',';first=false;capture<<v;}capture<<']';};
      capture<<",\"source_state\":";array(next->plan.states.front());
      capture<<",\"source_applied\":";array(next->plan.initial_applied);
      capture<<",\"actual_state\":";array(observed);capture<<",\"actual_applied\":";array(applied);
      capture<<",\"alignment\":";array(snapshot_.alignment);capture<<",\"controls\":[";
      bool first=true;for(const auto& u:next->plan.controls){if(!first)capture<<',';first=false;array(u);}capture<<']';
    }
    next->plan=activation_validator_->reanchor(next->plan,observed,applied,now,snapshot_.alignment);
    if(capture_takeover){
      capture<<",\"anchored_violation\":";if(std::isfinite(next->plan.max_violation))capture<<next->plan.max_violation;else capture<<"null";
      capture<<",\"accepted\":"<<(next->plan.success?"true":"false")<<'}';
      std::string escaped;for(char ch:capture.str()){if(ch=='"')escaped+='"';escaped+=ch;}
      std::ostringstream row;row<<std::setprecision(17)<<"takeover_snapshot,"<<now<<','<<next->sequence
        <<",0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\""<<escaped<<"\"\n";queue_log(row.str());
    }
    if(!next->plan.success)return reject("Takeover candidate infeasible: "+next->plan.reason);
    auto execution=certify_execution(next->plan,*sampler_,next->plan.states.front(),snapshot_.alignment,cfg_,bundle_.reference(),now,ttl_,speed_cap);
    last_certificate_=execution.duration_s;
    if(!execution.success)return reject("Executed trace infeasible: "+execution.reason);
    if(phase_=="RECOVERING"){
      if(observed[3]<.05){recovery_good_=0;return reject("Recovery stopped; re-enable required");}
      if(++recovery_good_<2)return reject("Recovery awaits a second usable candidate",false);
    }
    next->applied=applied;active_=next;
    sampler_->set_previous_endpoint(applied[1]);++activated_;if(next->plan.prefix_transported)++transported_;phase_="RUNNING";reason_.clear();
    log_activation(*next,begin,true,next->plan.prefix_transported?next->plan.reason:"");return true;
    }catch(const std::exception& e){return reject(std::string("Plan takeover exception: ")+e.what());}
  }
  double stop_reserve()const{
    return cfg_.minimum_drive_speed*((cfg_.command_profile=="rate_bounded_v2"?0.:cfg_.brake_limit/cfg_.jerk_limit)+ttl_)+
      cfg_.minimum_drive_speed*cfg_.minimum_drive_speed/(2*cfg_.brake_limit);
  }
  double finish_cap(double remaining)const{
    double delay=(cfg_.command_profile=="rate_bounded_v2"?0.:cfg_.brake_limit/cfg_.jerk_limit)+ttl_;
    double cap=cfg_.brake_limit*(std::sqrt(delay*delay+2*std::max(0.,remaining)/cfg_.brake_limit)-delay);
    return remaining<=std::max(.05,stop_reserve())?0.:std::min(cfg_.cruise_speed,cap);
  }
  void publish_command(){
    double begin=steady();OutputCommand command;std::shared_ptr<Pending> visualization;
#ifdef AIMS_MPCC_RT_TEST_ACCESS
    test_callback_entered_.store(true);
#endif
    std::uint64_t seq=0,emission_generation=0;double source=0.,forecast=0.;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // Receive callbacks may have updated timestamps while this callback waited.
      // Sample the decision epoch after owning the same state mutex.
      const double decision=steady();
      observe_clock_locked(get_clock()->now().seconds(),decision);
      try{
        if(enabled_&&!fresh_locked(decision)){
          std::ostringstream reason;reason<<std::setprecision(9)<<"Input freshness expired: source_age="<<decision-snapshot_.source
            <<" receipt_age="<<decision-snapshot_.received<<" authority_age="<<decision-mode_received_
            <<" applied_age="<<decision-history_->newest();fault_locked(reason.str());
        }
        if(enabled_&&!localization_locked(decision))fault_locked("Localization freshness expired");
        if(enabled_&&count_publishers(pub_->get_topic_name())>1)fault_locked("Another command publisher appeared");
        if(enabled_&&last_publish_>0&&decision-last_publish_>.1)fault_locked("Command scheduling discontinuity");
        if(enabled_){
          auto measured_map=map_point(snapshot_.state,snapshot_.alignment);
          if(!footprint_locked(measured_map,bundle_.reference().project(measured_map[0],measured_map[1])))
            throw std::runtime_error("measured footprint outside corridor");
          auto decision_state=history_->predict(snapshot_.state,snapshot_.source,decision,decision);
          auto mapped=map_point(decision_state,snapshot_.alignment);
          double progress=bundle_.reference().project(mapped[0],mapped[1]);decision_state[4]=progress;
          if(!footprint_locked(mapped,progress))throw std::runtime_error("decision footprint outside corridor");
          if(progress_ready_)unwrapped_progress_+=std::remainder(progress-last_progress_,bundle_.reference().length());
          last_progress_=progress;
          double remaining=start_progress_+bundle_.reference().length()-unwrapped_progress_;
          double cap=repeat_laps_?cfg_.max_speed:finish_cap(remaining);
          last_decision_state_=decision_state;last_decision_cap_=cap;
          if(activate_locked(decision,decision_state,cap))visualization=active_;
          command=sampler_->sample(active_?&active_->plan:nullptr,decision,decision_state,stopping_,ttl_,cap);
          if(command.expired&&decision-started_>=ttl_){
            if(phase_!="RECOVERING")recovery_good_=0;
            phase_="RECOVERING";reason_="Plan expired";
          }
          if(stopping_)phase_="STOPPING";
          bool finish=!repeat_laps_&&std::abs(remaining)<=std::max(.2,stop_reserve()+.05);
          if((finish||stopping_||phase_=="RECOVERING")&&command.continuous_speed<1e-6&&decision_state[3]<.05){
            if(stationary_since_<0)stationary_since_=decision;
            if(decision-stationary_since_>=.5){enabled_=false;generation_++;cancel_pending_locked("stationary stop");active_.reset();
              reason_=finish?"One lap complete":phase_=="RECOVERING"?"Recovery stopped; re-enable required":"Stopped";
              phase_=finish?"COMPLETE":"READY";}
          }else stationary_since_=-1.;
          if(!repeat_laps_&&remaining<-.2){
            fault_locked("Finish overshoot");command={};command.steering=last_output_.steering;
          }
          if(active_){seq=active_->sequence;source=active_->plan.source_epoch;forecast=active_->plan.forecast_epoch;}
        }else command.steering=last_output_.steering;
      }catch(const std::exception& e){fault_locked(e.what());command={};command.steering=last_output_.steering;}
      emission_generation=generation_;
    }
    double published=0.,gap=0.;
    {
    std::lock_guard<std::mutex> lock(mutex_);
    // Serialize the actual publication with authority/epoch invalidation.
    if(generation_!=emission_generation){command={};command.steering=last_output_.steering;}
    Drive message;message.header.stamp=get_clock()->now();message.header.frame_id="base_link";
    message.drive.speed=command.speed;message.drive.steering_angle=command.steering;
    message.drive.acceleration=command.acceleration;message.drive.steering_angle_velocity=command.steering_rate;
    message.drive.jerk=0.;pub_->publish(message);
    published=steady();gap=last_publish_?published-last_publish_:0.;last_publish_=published;
    last_output_=command;recent_.push_back({published,command});if(recent_.size()>16)recent_.pop_front();
    }
    if(visualization)publish_prediction(visualization->plan);
    std::ostringstream line;line<<std::setprecision(17)<<"publish,"<<published<<','<<seq<<','<<source<<','<<forecast
      <<",0,"<<steady()-begin<<",0,0,0,0,1,0,0,0,0,"<<gap<<','<<command.speed<<','<<command.steering<<",,0,0,0,,"<<command.requested_acceleration<<','<<command.acceleration
      <<','<<command.requested_steering_rate<<','<<command.steering_rate<<','
      <<(command.acceleration_limited||command.steering_rate_limited)<<"\n";
    queue_log(line.str());
  }
  void publish_status(){
    diagnostic_msgs::msg::DiagnosticArray message;message.header.stamp=get_clock()->now();
    diagnostic_msgs::msg::DiagnosticStatus status;status.name="aims_mpcc";
    {std::lock_guard<std::mutex> lock(mutex_);status.message=reason_.empty()?phase_:reason_;status.level=phase_=="FAULT"?2:0;
      auto value=[&](const std::string& key,const auto& v){diagnostic_msgs::msg::KeyValue kv;kv.key=key;kv.value=std::isfinite(static_cast<double>(v))?std::to_string(v):"null";status.values.push_back(kv);};
      auto text=[&](const std::string& key,const std::string& v){diagnostic_msgs::msg::KeyValue kv;kv.key=key;kv.value=json_string(v);status.values.push_back(kv);};
      text("status",phase_);text("reason",reason_);text("backend","acados_cpp");
      text("execution_certificate_scope","activation at current speed cap; later dynamic caps use per-hold physical budget; ideal acceleration and steering lag");
      diagnostic_msgs::msg::KeyValue ready;ready.key="worker_ready";ready.value=solver_ready_?"true":"false";status.values.push_back(ready);
      text("startup_instance",startup_instance_);
      diagnostic_msgs::msg::KeyValue startup;startup.key="startup_disabled";startup.value=startup_disabled_?"true":"false";status.values.push_back(startup);
      value("explicit_enable_count",explicit_enable_count_);
      const double now=steady();
      value("enabled",enabled_);value("stopping",stopping_);value("authority",mode_);value("authority_age_s",now-mode_received_);
      value("source_age_s",snapshot_.present?now-snapshot_.source:std::numeric_limits<double>::quiet_NaN());
      value("receipt_age_s",snapshot_.present?now-snapshot_.received:std::numeric_limits<double>::quiet_NaN());
      value("applied_age_s",now-history_->newest());
      value("plan_source_age_s",active_?now-active_->plan.source_epoch:std::numeric_limits<double>::quiet_NaN());
      value("plan_phase_s",active_?now-active_->plan.forecast_epoch:std::numeric_limits<double>::quiet_NaN());
      auto mapped=map_point(snapshot_.state,snapshot_.alignment);auto point=bundle_.reference().at(bundle_.reference().project(mapped[0],mapped[1]));
      value("cross_track",-(mapped[0]-point.x)*std::sin(point.yaw)+(mapped[1]-point.y)*std::cos(point.yaw));
      value("enforce_corridor",cfg_.enforce_corridor);text("command_profile",cfg_.command_profile);
      text("anchor_epoch",health_.epoch());value("anchor_sequence",health_.anchor_sequence());value("anchor_stamp_ns",health_.anchor_stamp_ns());
      text("snapshot_epoch",snapshot_.epoch);value("snapshot_anchor_sequence",snapshot_.anchor_sequence);
      value("handover_delay",lead_);value("solver_timeout",budget_);value("solve_frequency",frequency_);
      value("fault_steady_s",fault_steady_);value("compute_s",last_compute_);value("delivery_wait_s",last_delivery_wait_);
      value("execution_certificate_s",last_certificate_);value("activation_s",last_activation_);value("worker_busy",worker_busy_);
      value("worker_elapsed_s",worker_busy_?now-worker_submitted_:0.);
#ifdef AIMS_MPCC_RT_TEST_ACCESS
      value("test_worker_blocked",test_worker_blocked_.load());
#endif
      value("skipped_pending_slots",skipped_pending_slots_);value("delivered_accepted",delivered_accepted_);value("generation_cancelled",cancelled_);value("pending_cancelled",pending_cancelled_);
      value("pending_sequence",pending_?pending_->sequence:0);value("horizon",cfg_.horizon);value("dt",cfg_.dt);value("solve_period",1./frequency_);value("plan_ttl",ttl_);
      value("progress",unwrapped_progress_);value("speed_command",last_output_.speed);
      value("model_speed_command",last_output_.continuous_speed);value("steering_command",last_output_.steering);
      value("decision_speed",last_decision_state_[3]);value("decision_steering",last_decision_state_[5]);value("decision_speed_cap",last_decision_cap_);
      value("requested_acceleration",last_output_.requested_acceleration);value("emitted_acceleration",last_output_.acceleration);
      value("requested_steering_rate",last_output_.requested_steering_rate);value("emitted_steering_rate",last_output_.steering_rate);
      value("acceleration_limited",last_output_.acceleration_limited);value("steering_rate_limited",last_output_.steering_rate_limited);
      value("speed",snapshot_.state[3]);value("solve_time",last_.plan.solve_time_s);
      value("deadline_misses",late_);value("solve_sequence",last_.sequence);
      value("requests",requests_);value("failed",failed_);value("rejected",rejected_);value("late",late_);
      value("activated",activated_);value("prefix_transport_activations",transported_);value("handover_rejected",handover_rejected_);value("complete_s",last_complete_);
      value("observation_age_s",last_observation_age_);value("native_status",last_.plan.status);
      value("active_envelope_transported",active_&&active_->plan.envelope_transported);
      value("active_original_max_violation",active_?active_->plan.original_max_violation:0.);
      value("geometry_refreshes",last_.plan.geometry_refreshes);value("geometry_refresh_time_s",last_.plan.geometry_refresh_time_s);
      value("max_geometry_progress_shift",last_.plan.max_geometry_progress_shift);value("acados_rti_steps",cfg_.acados_rti_steps);value("native_passes",last_.plan.native_passes);value("max_violation",last_.plan.max_violation);
      value("log_dropped",log_dropped_.load());
      const auto& b=sampler_->braking_budget();std::ostringstream budget;
      budget<<std::boolalpha<<"{\"available\":"<<b.available<<",\"feasible\":"<<b.feasible
        <<",\"lateral_utilization_bound\":";
      if(std::isfinite(b.lateral_utilization))budget<<b.lateral_utilization;else budget<<"null";
      budget<<",\"current_feasible\":"<<b.current_feasible<<",\"current_lateral_utilization\":";
      if(std::isfinite(b.current_lateral_utilization))budget<<b.current_lateral_utilization;else budget<<"null";
      budget<<",\"accel_capacity\":"<<b.accel_capacity<<",\"brake_capacity\":"<<b.brake_capacity<<'}';
      diagnostic_msgs::msg::KeyValue kv;kv.key="braking_budget";kv.value=budget.str();status.values.push_back(kv);}
    message.status.push_back(status);status_pub_->publish(message);
  }
  void publish_reference(){
    nav_msgs::msg::Path path;path.header.stamp=get_clock()->now();path.header.frame_id=bundle_.reference().frame_id();
    for(double s=0;s<bundle_.reference().length();s+=.1){auto r=bundle_.reference().at(s);
      geometry_msgs::msg::PoseStamped p;p.header=path.header;p.pose.position.x=r.x;p.pose.position.y=r.y;
      p.pose.orientation.z=std::sin(r.yaw/2);p.pose.orientation.w=std::cos(r.yaw/2);path.poses.push_back(p);}
    reference_pub_->publish(path);
  }
  void publish_prediction(const Plan& plan){
    nav_msgs::msg::Path path;path.header.stamp=get_clock()->now();path.header.frame_id="odom";
    for(const auto& x:plan.states){geometry_msgs::msg::PoseStamped p;p.header=path.header;
      p.pose.position.x=x[0];p.pose.position.y=x[1];p.pose.orientation.z=std::sin(x[2]/2);p.pose.orientation.w=std::cos(x[2]/2);path.poses.push_back(p);}
    prediction_pub_->publish(path);
  }
  void log_capture_row(const std::string& event,const Pending& p,const std::string& json){
    std::string escaped;escaped.reserve(json.size()+64);for(char ch:json){if(ch=='"')escaped+='"';escaped+=ch;}
    std::ostringstream row;row<<std::setprecision(17)<<event<<','<<steady()<<','<<p.sequence<<','
      <<p.plan.source_epoch<<','<<p.plan.forecast_epoch<<','<<p.submitted
      <<",0,0,0,0,0,0,0,0,0,0,0,0,0,\""<<escaped<<"\"\n";queue_log(row.str());
  }
  void log_request_snapshot(const Pending& p,const State& initial,const Alignment& alignment,double elapsed,
      const std::vector<double>& targets,double core_budget,bool core_reset,bool prepared,const std::string& error){
    if(!log_.is_open())return;
    std::ostringstream body;body<<std::setprecision(17);
    auto array=[&](const auto& values){body<<'[';bool first=true;for(double value:values){
      if(!first)body<<',';first=false;if(std::isfinite(value))body<<value;else body<<"null";}body<<']';};
    body<<"{\"sequence\":"<<p.sequence<<",\"generation\":"<<p.generation
      <<",\"initial\":";array(initial);body<<",\"applied\":";array(p.applied);
    body<<",\"alignment\":";array(alignment);body<<",\"targets\":";array(targets);
    body<<",\"elapsed\":"<<elapsed<<",\"request_budget_s\":"<<budget_<<",\"core_budget_s\":"<<core_budget
      <<",\"handover_delay\":"<<lead_<<",\"source_epoch\":"<<p.plan.source_epoch
      <<",\"forecast_epoch\":"<<p.plan.forecast_epoch<<",\"submitted\":"<<p.submitted
      <<",\"prefix_speed\":"<<p.prefix.speed<<",\"core_reset\":"<<(core_reset?"true":"false")
      <<",\"preparation_complete\":"<<(prepared?"true":"false")<<",\"preparation_error\":"<<json_string(error)
      <<",\"refresh_second_geometry\":true,\"artifact_fingerprint\":"<<json_string(bundle_.fingerprint())<<'}';
    log_capture_row("request_snapshot",p,body.str());
  }
  void log_request_validation(const Pending& p){
    if(!log_.is_open())return;
    std::ostringstream body;body<<std::setprecision(17)<<"{\"sequence\":"<<p.sequence
      <<",\"success\":"<<(p.plan.success?"true":"false")<<",\"status\":"<<p.plan.status
      <<",\"native_passes\":"<<p.plan.native_passes<<",\"reason\":"<<json_string(p.plan.reason)
      <<",\"geometry_refreshes\":"<<p.plan.geometry_refreshes<<",\"geometry_refresh_time_s\":";
    auto number=[&](double value){if(std::isfinite(value))body<<value;else body<<"null";};
    number(p.plan.geometry_refresh_time_s);body<<",\"max_geometry_progress_shift\":";number(p.plan.max_geometry_progress_shift);
    body<<",\"constraint_violations\":{";
    bool first=true;for(const auto& entry:p.plan.constraint_violations){if(!first)body<<',';first=false;
      body<<json_string(entry.first)<<':';if(std::isfinite(entry.second))body<<entry.second;else body<<"null";}
    body<<"}}";log_capture_row("request_validation",p,body.str());
  }
  void log_request(const Pending& p,double done,double complete,bool accepted,double age,double compute,double wait,const std::string& disposition){
    std::string reason=complete>budget_?"request_deadline":p.plan.reason;
    std::replace(reason.begin(),reason.end(),',',';');std::replace(reason.begin(),reason.end(),'\n',' ');
    std::ostringstream line;line<<std::setprecision(17)<<"request,"<<done<<','<<p.sequence<<','<<p.plan.source_epoch<<','
      <<p.plan.forecast_epoch<<','<<p.submitted<<','<<complete<<','<<p.plan.solve_time_s<<','<<p.plan.native_time_s<<','
      <<p.plan.preparation_time_s<<','<<p.plan.validation_time_s<<','
      <<accepted<<','<<p.plan.status<<','<<p.plan.native_passes<<','<<p.plan.max_violation<<','<<age<<",0,0,0,"<<reason<<','<<compute<<','<<wait<<','<<complete<<','<<disposition<<"\n";
    queue_log(line.str());
  }
  void log_activation(const Pending& p,double begin,bool accepted,std::string reason){
    last_activation_=steady()-begin;
    if(p.plan.prefix_transported||p.plan.original_max_violation>0.){
      std::ostringstream provenance;provenance<<std::setprecision(17)<<"; envelope_transported="<<p.plan.envelope_transported
        <<"; original_max_violation="<<p.plan.original_max_violation;
      for(const auto& entry:p.plan.original_constraint_violations)provenance<<"; original_"<<entry.first<<'='<<entry.second;
      reason+=provenance.str();
    }
    std::replace(reason.begin(),reason.end(),',',';');std::replace(reason.begin(),reason.end(),'\n',' ');
    std::ostringstream line;line<<std::setprecision(17)<<"activation,"<<steady()<<','<<p.sequence<<','
      <<p.plan.source_epoch<<','<<p.plan.forecast_epoch<<','<<p.submitted<<','<<steady()-begin
      <<",0,0,0,"<<p.plan.reanchor_time_s<<','<<accepted<<','<<p.plan.status
      <<",0,"<<p.plan.max_violation<<",0,0,0,0,"<<reason<<"\n";
    queue_log(line.str());
  }
  void queue_log(std::string line){if(!log_.is_open())return;
    if(line.rfind("publish,",0)!=0){if(!line.empty()&&line.back()=='\n')line.pop_back();
      if(line.rfind("request,",0)!=0)line+=",0,0,0,";
      line+=",0,0,0,0,0\n";}
std::lock_guard<std::mutex> lock(log_mutex_);
    if(log_queue_.size()>=4096){++log_dropped_;return;}log_queue_.push_back(std::move(line));log_cv_.notify_one();}
  void log_loop(){std::unique_lock<std::mutex> lock(log_mutex_);while(!log_shutdown_||!log_queue_.empty()){
    log_cv_.wait_for(lock,std::chrono::milliseconds(100),[this]{return log_shutdown_||!log_queue_.empty();});
    std::deque<std::string> batch;batch.swap(log_queue_);lock.unlock();for(const auto& line:batch)log_<<line;log_.flush();lock.lock();}}
  Bundle bundle_;Config cfg_;std::unique_ptr<AppliedHistory> history_;std::unique_ptr<OutputSampler> sampler_;
  std::unique_ptr<Core> activation_validator_;
  State last_decision_state_{};double last_decision_cap_=0.;
  Snapshot snapshot_;LocalizationHealth health_;std::string identity_,phase_="READY",reason_;
  bool enabled_=false,stopping_=false,shutdown_=false,solver_ready_=false,simulation_=false,shadow_=true,repeat_laps_=false,progress_ready_=false;
  std::string startup_instance_;bool startup_disabled_=false;std::uint64_t explicit_enable_count_=0;
  double last_ros_now_=std::numeric_limits<double>::quiet_NaN(),fault_steady_=0.,last_compute_=0.,last_delivery_wait_=0.,last_certificate_=0.,last_activation_=0.,worker_submitted_=0.;bool worker_busy_=false;
  double frequency_=20.,ttl_=.8,budget_=.05,lead_=.02,mode_received_=-1.,started_=0.,last_publish_=0.;
  double start_progress_=0.,unwrapped_progress_=0.,last_progress_=0.,last_complete_=0.,last_observation_age_=0.,stationary_since_=-1.;bool mode_=false;
  std::uint64_t generation_=0,requests_=0,failed_=0,rejected_=0,late_=0,activated_=0,handover_rejected_=0,transported_=0,delivered_accepted_=0,cancelled_=0,pending_cancelled_=0,skipped_pending_slots_=0;
  std::shared_ptr<Pending> active_,pending_;Pending last_;OutputCommand last_output_;
  std::deque<std::pair<double,OutputCommand>> recent_;
  std::mutex mutex_;std::condition_variable cv_;std::thread worker_;
  std::mutex log_mutex_;std::condition_variable log_cv_;std::thread logger_;std::ofstream log_;
  std::deque<std::string> log_queue_;bool log_shutdown_=false;std::atomic<std::uint64_t> log_dropped_{0};
  rclcpp::CallbackGroup::SharedPtr receive_group_,output_group_;
  rclcpp::Publisher<Drive>::SharedPtr pub_;rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_pub_,prediction_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub_;rclcpp::Subscription<Drive>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr identity_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_;
  rclcpp::TimerBase::SharedPtr output_timer_,status_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
  unsigned recovery_good_=0;
  std::unique_ptr<tf2_ros::Buffer> tf_;std::shared_ptr<tf2_ros::TransformListener> listener_;
};
} // namespace aims_mpcc_rt

int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  try{
    auto node=std::make_shared<aims_mpcc_rt::RuntimeNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),3);
    executor.add_node(node);executor.spin();executor.remove_node(node);node.reset();
  }catch(const std::exception& e){std::cerr<<"MPCC runtime startup/runtime error: "<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}

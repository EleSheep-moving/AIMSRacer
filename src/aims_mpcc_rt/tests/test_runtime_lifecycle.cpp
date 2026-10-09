#define AIMS_MPCC_RT_TEST_ACCESS
#define main runtime_entry_unused
#include "../src/node.cpp"
#undef main
namespace aims_mpcc_rt {
struct RuntimeClockProbe {
  static LocalizationHealth::Values health(const std::string& epoch,std::int64_t ns){
    LocalizationHealth::Values v{{"protocol_version","1"},{"epoch",epoch},{"health_sequence","1"},
      {"anchor_sequence","1"},{"last_anchor_stamp_ns",std::to_string(ns)},{"ready","true"},
      {"alignment_valid","true"},{"alignment_epoch",epoch},{"alignment_anchor_sequence","1"},
      {"alignment_stamp_ns",std::to_string(ns)}};
    for(auto key:{"x","y","z","qx","qy","qz"})v[std::string("map_odom_")+key]="0";
    v["map_odom_qw"]="1";return v;
  }
  static nav_msgs::msg::Odometry odom(RuntimeNode& node){
    auto r=node.bundle_.reference().at(0.);nav_msgs::msg::Odometry m;
    m.header.frame_id="odom";m.child_frame_id="base_link";m.header.stamp=node.get_clock()->now();
    m.pose.pose.position.x=r.x+node.cfg_.rear_offset*std::cos(r.yaw);
    m.pose.pose.position.y=r.y+node.cfg_.rear_offset*std::sin(r.yaw);
    m.pose.pose.orientation.z=std::sin(r.yaw/2);m.pose.pose.orientation.w=std::cos(r.yaw/2);return m;
  }
  static bool enable(RuntimeNode& node){
    auto req=std::make_shared<std_srvs::srv::SetBool::Request>();req->data=true;
    auto res=std::make_shared<std_srvs::srv::SetBool::Response>();node.enable_request(req,res);return res->success;
  }
  static void epoch(RuntimeNode& node){
    auto ns=node.get_clock()->now().nanoseconds();auto previous=health("old",ns);previous["map_sha256"]=node.bundle_.reference().map_sha256();node.accept_health(previous);
    {std::lock_guard<std::mutex> lock(node.mutex_);node.identity_=node.bundle_.reference().map_sha256();}
    {std::lock_guard<std::mutex> lock(node.mutex_);node.snapshot_.present=true;node.snapshot_.epoch="old";}
    auto next=health("new",ns);next["map_sha256"]=node.bundle_.reference().map_sha256();node.accept_health(next);
    if(node.snapshot_.present||enable(node))throw std::runtime_error("new ready epoch authorized old snapshot while disabled");
    Drive d;node.forwarded(d);node.odometry(odom(node));
    if(!node.snapshot_.present||node.snapshot_.epoch!="new")throw std::runtime_error("new odometry not bound to new accepted anchor");
    const double limit=steady()+2.;while(!node.solver_ready_&&steady()<limit)std::this_thread::yield();
    {std::lock_guard<std::mutex> lock(node.mutex_);node.mode_=true;node.mode_received_=steady();}
    if(!enable(node))throw std::runtime_error("fresh new epoch state and history did not permit stationary enable");
    if(node.bundle_.reference().frame_id()=="map"&&node.cfg_.command_profile=="rate_bounded_v2"){
      auto wrong=next;wrong["health_sequence"]="2";wrong["anchor_sequence"]="2";wrong["alignment_anchor_sequence"]="2";wrong["map_sha256"]="wrong-map";wrong["map_odom_x"]="123";
      node.accept_health(wrong);node.forwarded(d);node.odometry(odom(node));
      if(node.health_.qualified()||enable(node))throw std::runtime_error("wrong-map atomic anchor survived odometry and authorized enable");
      auto changed=next;changed["health_sequence"]="2";changed["map_odom_x"]="123";
      node.accept_health(changed);node.forwarded(d);node.odometry(odom(node));
      if(node.health_.qualified()||enable(node))throw std::runtime_error("invalid packet permitted replacing committed anchor without changing identity");
    }
  }
  static void clock(RuntimeNode& node){
    node.set_parameter(rclcpp::Parameter("use_sim_time",true));auto handle=node.get_clock()->get_clock_handle();
    if(rcl_enable_ros_time_override(handle)!=RCL_RET_OK||rcl_set_ros_time_override(handle,200000000000LL)!=RCL_RET_OK)
      throw std::runtime_error("could not establish initial ROS test clock");
    auto initial=health("before-reset",200000000000LL);initial["map_sha256"]=node.bundle_.reference().map_sha256();node.accept_health(initial);
    Drive d;node.forwarded(d);node.odometry(odom(node));
    {std::lock_guard<std::mutex> lock(node.mutex_);node.enabled_=true;node.recent_.push_back({steady(),{}});node.progress_ready_=true;}
    auto generation=node.generation_;
    if(rcl_set_ros_time_override(handle,100000000000LL)!=RCL_RET_OK)throw std::runtime_error("could not reset ROS test clock");
    node.odometry(odom(node));
    if(node.enabled_||node.snapshot_.present||node.history_->newest()!=-1.||!node.recent_.empty()||node.progress_ready_||node.generation_<=generation)
      throw std::runtime_error("clock reset did not clear complete runtime lifecycle");
    for(int i=0;i<3;++i){if(rcl_set_ros_time_override(handle,100000000000LL+i*1000000LL)!=RCL_RET_OK)
      throw std::runtime_error("could not advance reset ROS clock");
      auto fresh=health("after-reset",100000000000LL+i*1000000LL);
      fresh["health_sequence"]=fresh["anchor_sequence"]=fresh["alignment_anchor_sequence"]=std::to_string(i+1);
      fresh["map_sha256"]=node.bundle_.reference().map_sha256();node.accept_health(fresh);
      {std::lock_guard<std::mutex> lock(node.mutex_);node.identity_=node.bundle_.reference().map_sha256();}
      node.forwarded(d);node.odometry(odom(node));}
    if(!node.snapshot_.present||node.snapshot_.ros_source>101.||node.enabled_)throw std::runtime_error("fresh reset clock did not recover without old watermark");
  }
  static void anchor_order(RuntimeNode& node){
    if(node.bundle_.reference().frame_id()!="map")throw std::runtime_error("map bundle required for anchor-order regression");
    auto anchor=health("stable",node.get_clock()->now().nanoseconds());
    anchor["map_sha256"]=node.bundle_.reference().map_sha256();node.accept_health(anchor);
    {std::lock_guard<std::mutex> lock(node.mutex_);node.identity_=node.bundle_.reference().map_sha256();}
    Drive stopped;node.forwarded(stopped);node.odometry(odom(node));
    const double deadline=steady()+2.;
    while(steady()<deadline){bool ready=false;{std::lock_guard<std::mutex> lock(node.mutex_);ready=node.solver_ready_;}if(ready)break;std::this_thread::yield();}
    {std::lock_guard<std::mutex> lock(node.mutex_);node.mode_=true;node.mode_received_=steady();}
    if(!enable(node))throw std::runtime_error("anchor-order fixture could not enable stationary controller");
    for(int sequence:{2,3}){
      // Both legal arrival orders: health then output before odom; odom then
      // health then output. The odom-frame physical state remains unchanged.
      if(sequence==3){node.forwarded(stopped);node.odometry(odom(node));}
      Snapshot prior;double history_epoch=0.;{std::lock_guard<std::mutex> lock(node.mutex_);prior=node.snapshot_;history_epoch=node.history_->newest();}
      anchor["health_sequence"]=anchor["anchor_sequence"]=anchor["alignment_anchor_sequence"]=std::to_string(sequence);
      anchor["last_anchor_stamp_ns"]=anchor["alignment_stamp_ns"]=std::to_string(node.get_clock()->now().nanoseconds());
      anchor["map_odom_x"]=std::to_string((sequence-1)*.001);node.accept_health(anchor);node.publish_command();
      {std::lock_guard<std::mutex> lock(node.mutex_);
        if(!node.enabled_||node.phase_!="RUNNING")throw std::runtime_error("healthy same-epoch anchor before odometry faulted: "+node.reason_);
        if(node.snapshot_.epoch!="stable"||node.snapshot_.anchor_sequence!=sequence||
           node.snapshot_.alignment!=node.health_.alignment())throw std::runtime_error("same-epoch snapshot did not atomically adopt trusted alignment identity");
        if(node.snapshot_.state!=prior.state||node.snapshot_.source!=prior.source||node.snapshot_.received!=prior.received||
           node.history_->newest()!=history_epoch)throw std::runtime_error("anchor update relabeled or refreshed odometry/history epochs");
      }
      if(sequence==2){node.forwarded(stopped);node.odometry(odom(node));}
    }
    const auto alignment=node.snapshot_.alignment;auto duplicate=anchor;duplicate["map_odom_x"]=".5";
    node.accept_health(duplicate); // duplicate heartbeat cannot replace the committed record
    if(node.snapshot_.alignment!=alignment||!node.enabled_)throw std::runtime_error("duplicate anchor mutated healthy alignment");
    duplicate["health_sequence"]="4";node.accept_health(duplicate);
    if(node.enabled_||node.snapshot_.alignment!=alignment)throw std::runtime_error("same identity changed transform bypassed immutable anchor guard");
    auto next=health("new-epoch",node.get_clock()->now().nanoseconds());next["map_sha256"]=node.bundle_.reference().map_sha256();node.accept_health(next);
    if(node.snapshot_.present||enable(node))throw std::runtime_error("new epoch reused old odometry after same-epoch adoption repair");
  }
  static void stall(RuntimeNode& node){
    const char* domain=std::getenv("ROS_DOMAIN_ID");
    if(!domain||std::stoi(domain)<200)throw std::runtime_error("stall fixture requires a private ROS domain >=200");
    rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node.get_node_base_interface());
    auto odom_pub=node.create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered",rclcpp::SensorDataQoS());
    auto applied_pub=node.create_publisher<Drive>("/ackermann_cmd",10);
    auto authority_pub=node.create_publisher<std_msgs::msg::Bool>("/control/autonomy_speed_enabled",10);
    bool ready_seen=false;
    auto startup_status=node.create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/mpcc_rt_shadow/mpcc/status",10,
      [&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message){
        for(const auto& status:message->status){bool ready=false,disabled=false;
          for(const auto& value:status.values){if(value.key=="status")ready=value.value=="\"READY\"";
            if(value.key=="enabled")disabled=std::stod(value.value)==0.;}
          if(ready&&disabled)ready_seen=true;}});
    Drive actual;auto drive_sub=node.create_subscription<Drive>("/mpcc_rt_shadow/drive",10,
      [&](Drive::ConstSharedPtr command){actual=*command;});
    auto r=node.bundle_.reference().at(0.);State plant{r.x,r.y,r.yaw,0.,0.,0.};
    {std::lock_guard<std::mutex> lock(node.mutex_);node.history_->record(steady()-.05,0.,0.,0.,0.);}
    double last=steady(),startup=last,armed_at=0.;bool enabled=false,armed=false;
    while(rclcpp::ok()){
      const double now=steady(),dt=std::min(.05,std::max(0.,now-last));last=now;
      const double acceleration=std::clamp((double(actual.drive.speed)-plant[3])/std::max(dt,1e-6),-node.cfg_.brake_limit,node.cfg_.accel_limit);
      const double middle_speed=std::max(0.,plant[3]+acceleration*dt/2.);
      const double middle_delta=actual.drive.steering_angle+(plant[5]-actual.drive.steering_angle)*std::exp(-dt/(2*node.cfg_.steering_tau));
      const double yaw_rate=middle_speed*std::tan(middle_delta)/(node.cfg_.wheelbase*(1+node.cfg_.understeer_coefficient*middle_speed*middle_speed));
      plant[0]+=dt*middle_speed*std::cos(plant[2]+yaw_rate*dt/2.);plant[1]+=dt*middle_speed*std::sin(plant[2]+yaw_rate*dt/2.);
      plant[2]+=yaw_rate*dt;plant[3]=std::max(0.,plant[3]+acceleration*dt);
      plant[5]=actual.drive.steering_angle+(plant[5]-actual.drive.steering_angle)*std::exp(-dt/node.cfg_.steering_tau);
      auto m=odom(node);m.pose.pose.position.x=plant[0]+node.cfg_.rear_offset*std::cos(plant[2]);
      m.pose.pose.position.y=plant[1]+node.cfg_.rear_offset*std::sin(plant[2]);m.pose.pose.orientation.z=std::sin(plant[2]/2.);
      m.pose.pose.orientation.w=std::cos(plant[2]/2.);m.twist.twist.linear.x=plant[3];
      actual.header.stamp=node.get_clock()->now();actual.drive.jerk=0.;applied_pub->publish(actual);
      std_msgs::msg::Bool authority;authority.data=true;authority_pub->publish(authority);odom_pub->publish(m);
      executor.spin_some();
      if(!enabled&&ready_seen&&steady()-startup>=.25)enabled=enable(node);
      if(!enabled&&steady()-startup>3.)throw std::runtime_error("stall fixture never enabled fresh stationary controller after disabled READY");
      if(enabled&&!armed){std::lock_guard<std::mutex> lock(node.mutex_);
        if(node.activated_>0&&node.last_output_.speed>0.){node.test_block_worker_.store(true);armed=true;armed_at=steady();
          std::cout<<"TEST_WORKER_STALL_ARMED "<<std::setprecision(17)<<armed_at<<std::endl;}}
      if(!armed&&steady()-startup>4.)throw std::runtime_error("stall fixture never obtained positive activated output");
      if(armed&&steady()-armed_at>.3&&!node.test_worker_blocked_.load())
        throw std::runtime_error("worker stall injection did not hold a solve request");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    executor.remove_node(node.get_node_base_interface());
  }
  static void cap(RuntimeNode& node){
    if(node.cfg_.command_profile!="rate_bounded_v2")throw std::runtime_error("v2 bundle required for cap regression");
    const double t=steady();const auto r=node.bundle_.reference().at(0.);
    {std::lock_guard<std::mutex> lock(node.mutex_);
      node.repeat_laps_=false;node.history_->clear();
      node.history_->record(t-2.,.3,1.05,0.,0.);node.history_->record(t-.001,.3,1.05,0.,0.);
      State measured{r.x,r.y,r.yaw,1.,0.,.3};
      node.snapshot_={measured,{0.,.3,0.},{},t-.095,0.,t,true};node.mode_=true;node.mode_received_=t;
      node.enabled_=true;node.phase_="RUNNING";node.started_=t;node.progress_ready_=true;
      const auto predicted=node.history_->predict(measured,t-.095,t,t);
      auto mapped=node.map_point(predicted,{});const double theta=node.bundle_.reference().project(mapped[0],mapped[1]);
      const double remaining=.9*node.ttl_+.9*.9/(2*node.cfg_.brake_limit);
      node.start_progress_=remaining-node.bundle_.reference().length()+theta;node.last_progress_=node.unwrapped_progress_=0.;
      auto plan=std::make_shared<Pending>();plan->generation=node.generation_;plan->sequence=42;
      plan->plan.success=true;plan->plan.dt=node.cfg_.dt;plan->plan.source_epoch=t-.095;plan->plan.forecast_epoch=t-.02;
      plan->plan.initial_applied={0.,.3,0.};plan->plan.controls.assign(node.cfg_.horizon,Control{0.,.3,1.05});
      node.active_=plan;node.sampler_->reset(1.05,.3,t-.02);node.sampler_->set_previous_endpoint(.3);
    }
    node.publish_command();
    std::lock_guard<std::mutex> lock(node.mutex_);
    if(node.phase_!="RUNNING")throw std::runtime_error("initially safe capped execution faulted: "+node.reason_);
    const auto actual=node.history_->predict(node.snapshot_.state,node.snapshot_.source,node.last_publish_,node.last_publish_);
    const auto projected=node.map_point(actual,{});const double actual_progress=node.bundle_.reference().project(projected[0],projected[1]);
    if(std::abs(std::remainder(node.last_progress_-actual_progress,node.bundle_.reference().length()))>.01)
      throw std::runtime_error("finish/output decision used stale source state instead of actual-history prediction");
    const double lateral=actual[3]*actual[3]*std::tan(actual[5])/(node.cfg_.wheelbase*(1.+node.cfg_.understeer_coefficient*actual[3]*actual[3]));
    const double E=std::pow(node.last_output_.continuous_acceleration/node.cfg_.envelope_brake,2)+std::pow(lateral/node.cfg_.lateral_accel_limit,2);
    if(node.last_output_.continuous_acceleration>=0.||E>1.0001)throw std::runtime_error("finish cap violated retained envelope from initially safe decision state");
  }
  static void capture(RuntimeNode& node){
    // Use the actual worker and logger, then inspect the opt-in CSV records.
    run(node);std::size_t snapshots=0,submissions=0,validations=0;
    {std::lock_guard<std::mutex> lock(node.mutex_);node.shutdown_=true;}node.cv_.notify_all();
    if(node.worker_.joinable())node.worker_.join();
    {std::lock_guard<std::mutex> lock(node.log_mutex_);node.log_shutdown_=true;}node.log_cv_.notify_all();
    if(node.logger_.joinable())node.logger_.join();node.log_.close();
    std::ifstream input(std::filesystem::path(node.get_parameter("log_directory").as_string())/"runtime.csv");std::string line;
    while(std::getline(input,line)){
      if(line.rfind("submission,",0)==0)++submissions;
      if(line.rfind("request_validation,",0)==0)++validations;
      if(line.rfind("request_snapshot,",0)==0){
      ++snapshots;for(auto key:{"initial","applied","alignment","elapsed","targets","core_budget_s","core_reset"})
        if(line.find(std::string("\"\"")+key+"\"\"")==std::string::npos)throw std::runtime_error("request snapshot lacks replay field");
      }
    }
    if(snapshots==0||snapshots!=submissions||validations!=submissions)
      throw std::runtime_error("opt-in worker request capture does not account for every submitted request");
  }
  static void fault(RuntimeNode& node){
    std::lock_guard<std::mutex> lock(node.mutex_);node.fault_locked("first cause");const double first=node.fault_steady_;
    node.fault_locked("secondary symptom");
    if(node.reason_!="first cause"||node.fault_steady_!=first)throw std::runtime_error("secondary fault overwrote first cause/action timestamp");
  }
  static void delivery(RuntimeNode& node){
    std::unique_lock<std::mutex> lock(node.mutex_);
    const double t=steady();const auto r=node.bundle_.reference().at(0.);
    node.history_->record(t-.002,0.,0.,0.,0.);
    node.snapshot_={{r.x,r.y,r.yaw,0.,0.,0.},{},{},t-.001,0.,t,true};
    node.mode_=true;node.mode_received_=t;node.enabled_=true;node.started_=t;node.phase_="RUNNING";
    lock.unlock();
    const double limit=steady()+2.;
    while(steady()<limit){
      lock.lock();
      if(node.requests_>0&&node.last_.sequence==0)break;
      lock.unlock();std::this_thread::yield();
    }
    if(!lock.owns_lock())throw std::runtime_error("request already delivered before contention probe");
    while(!node.test_compute_complete_.load()&&steady()<limit)std::this_thread::yield();
    if(!node.test_compute_complete_.load()){lock.unlock();throw std::runtime_error("worker did not complete computation");}
    std::this_thread::sleep_for(std::chrono::milliseconds(60));lock.unlock();
    while(steady()<limit){lock.lock();if(node.last_.sequence>0)break;lock.unlock();std::this_thread::yield();}
    if(!lock.owns_lock())throw std::runtime_error("worker did not deliver disposition");
    if(node.late_==0||node.last_complete_<=node.budget_||node.pending_)
      throw std::runtime_error("60 ms delivery contention accepted under 50 ms budget");
  }
  static void run(RuntimeNode& node) {
    // Exercise the actual worker and publication callbacks with a 100 ms
    // forecast lead. A pending result must survive the 50 ms request slot.
    const double until=steady()+.65;
    while(steady()<until){
      {
        std::lock_guard<std::mutex> lock(node.mutex_);
        const double t=steady();const auto r=node.bundle_.reference().at(0.);
        node.history_->record(t-.002,0.,0.,0.,0.);
        node.snapshot_={{r.x,r.y,r.yaw,0.,0.,0.},{},{},t-.001,0.,t,true};
        node.mode_=true;node.mode_received_=t;
        if(!node.enabled_){node.enabled_=true;node.phase_="RUNNING";node.started_=t;node.sampler_->reset(0.,0.,t);}
      }
      node.publish_command();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::lock_guard<std::mutex> lock(node.mutex_);
    if(node.activated_==0)throw std::runtime_error("20 Hz / 100 ms lead never activated a pending plan");
  }
};
}
int main(int argc,char** argv){
  if(argc<2)throw std::invalid_argument("bundle argument required");
  std::vector<std::string> args{"lifecycle_test","--ros-args","-p",std::string("artifact_directory:=")+argv[1],
    "-p","simulation:=true","-p","repeat_laps:=true","-p","handover_delay:=0.1"};
  if(argc>2&&std::string(argv[2])=="capture"){
    setenv("AIMS_MPCC_CAPTURE_REQUEST","1",1);args.insert(args.end(),{"-p","log_directory:=/tmp/aims-runtime-request-capture"});}
  if(argc>2&&std::string(argv[2])=="stall")args.insert(args.end(),{"-p",std::string("log_directory:=")+(argc>3?argv[3]:"/tmp/aims-runtime-worker-stall"),
    "-r","/ackermann_cmd:=/mpcc_worker_stall/ackermann_cmd","-r","/odometry/filtered:=/mpcc_worker_stall/odometry",
    "-r","/control/autonomy_speed_enabled:=/mpcc_worker_stall/autonomy_speed_enabled"});
  std::vector<char*> raw;for(auto& s:args)raw.push_back(s.data());rclcpp::init(raw.size(),raw.data());
  const bool mesh=argc>2&&std::string(argv[2])=="mesh";
  int result=0;try{auto node=std::make_shared<aims_mpcc_rt::RuntimeNode>();
    if(mesh)throw std::runtime_error("unqualified v2 50 ms mesh started a controller");if(argc>2&&std::string(argv[2])=="delivery")aims_mpcc_rt::RuntimeClockProbe::delivery(*node);
    else if(argc>2&&std::string(argv[2])=="epoch")aims_mpcc_rt::RuntimeClockProbe::epoch(*node);
    else if(argc>2&&std::string(argv[2])=="anchor_order")aims_mpcc_rt::RuntimeClockProbe::anchor_order(*node);
    else if(argc>2&&std::string(argv[2])=="stall")aims_mpcc_rt::RuntimeClockProbe::stall(*node);
    else if(argc>2&&std::string(argv[2])=="cap")aims_mpcc_rt::RuntimeClockProbe::cap(*node);
    else if(argc>2&&std::string(argv[2])=="capture")aims_mpcc_rt::RuntimeClockProbe::capture(*node);
    else if(argc>2&&std::string(argv[2])=="fault")aims_mpcc_rt::RuntimeClockProbe::fault(*node);
    else if(argc>2&&std::string(argv[2])=="clock")aims_mpcc_rt::RuntimeClockProbe::clock(*node);
    else aims_mpcc_rt::RuntimeClockProbe::run(*node);}
  catch(const std::exception& e){std::cerr<<e.what()<<'\n';
    result=mesh&&std::string(e.what()).find("qualified 100 ms solver mesh")!=std::string::npos?0:1;}rclcpp::shutdown();return result;
}

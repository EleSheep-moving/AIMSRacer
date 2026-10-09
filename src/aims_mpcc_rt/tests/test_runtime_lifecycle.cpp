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
  std::vector<char*> raw;for(auto& s:args)raw.push_back(s.data());rclcpp::init(raw.size(),raw.data());
  const bool mesh=argc>2&&std::string(argv[2])=="mesh";
  int result=0;try{auto node=std::make_shared<aims_mpcc_rt::RuntimeNode>();
    if(mesh)throw std::runtime_error("unqualified v2 50 ms mesh started a controller");if(argc>2&&std::string(argv[2])=="delivery")aims_mpcc_rt::RuntimeClockProbe::delivery(*node);
    else if(argc>2&&std::string(argv[2])=="epoch")aims_mpcc_rt::RuntimeClockProbe::epoch(*node);
    else if(argc>2&&std::string(argv[2])=="capture")aims_mpcc_rt::RuntimeClockProbe::capture(*node);
    else if(argc>2&&std::string(argv[2])=="fault")aims_mpcc_rt::RuntimeClockProbe::fault(*node);
    else if(argc>2&&std::string(argv[2])=="clock")aims_mpcc_rt::RuntimeClockProbe::clock(*node);
    else aims_mpcc_rt::RuntimeClockProbe::run(*node);}
  catch(const std::exception& e){std::cerr<<e.what()<<'\n';
    result=mesh&&std::string(e.what()).find("qualified 100 ms solver mesh")!=std::string::npos?0:1;}rclcpp::shutdown();return result;
}

#define AIMS_MPCC_RT_TEST_ACCESS
#define main runtime_entry_unused
#include "../src/node.cpp"
#undef main

namespace aims_mpcc_rt {
struct RuntimeClockProbe {
  static void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
  static LocalizationHealth::Values health(RuntimeNode& node,const std::string& epoch){
    const auto stamp=std::to_string(node.get_clock()->now().nanoseconds());
    LocalizationHealth::Values v{{"protocol_version","1"},{"epoch",epoch},{"health_sequence","1"},
      {"anchor_sequence","1"},{"last_anchor_stamp_ns",stamp},{"ready","true"},{"alignment_valid","true"},
      {"alignment_epoch",epoch},{"alignment_anchor_sequence","1"},{"alignment_stamp_ns",stamp},
      {"map_sha256",node.bundle_.reference().map_sha256()}};
    for(auto key:{"x","y","z","qx","qy","qz"})v[std::string("map_odom_")+key]="0";
    v["map_odom_qw"]="1";return v;
  }
  static void fresh(RuntimeNode& node,bool authority=true){
    std::lock_guard<std::mutex> lock(node.mutex_);const double now=steady();
    const auto r=node.bundle_.reference().at(0.);
    node.history_->record(now-.005,0.,0.,0.,0.);
    node.snapshot_={{r.x,r.y,r.yaw,0.,0.,0.},{},{},now-.002,node.get_clock()->now().seconds(),now,true,
      node.health_.epoch(),node.health_.anchor_sequence(),node.health_.anchor_stamp_ns()};
    node.mode_=authority;node.mode_received_=now;
  }
  static void attempt(RuntimeNode& node,double now=steady()){
    std::lock_guard<std::mutex> lock(node.mutex_);node.try_auto_start_locked(now);
  }
  static void stop(RuntimeNode& node){
    auto request=std::make_shared<std_srvs::srv::SetBool::Request>();request->data=false;
    auto response=std::make_shared<std_srvs::srv::SetBool::Response>();node.enable_request(request,response);
    check(response->success,"stop while disabled was refused");
  }
  static void diagnostics(RuntimeNode& node){
    auto observer=std::make_shared<rclcpp::Node>("auto_start_diagnostics_observer");
    std::map<std::string,std::string> fields;
    auto sub=observer->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(node.status_pub_->get_topic_name(),10,
      [&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message){for(const auto& s:message->status)
        if(s.name=="aims_mpcc")for(const auto& kv:s.values)fields[kv.key]=kv.value;});
    rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(observer);
    const double deadline=steady()+2.;
    while(fields.empty()&&steady()<deadline){node.publish_status();executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));}
    check(fields["startup_instance"]==json_string(node.startup_instance_),"process identity diagnostic incorrect");
    check(fields["stop_request_count"]=="2","disabled stop counter diagnostic incorrect");
    check(fields["auto_start_pending"]=="0"&&fields["auto_start_count"]=="0","cancelled auto startup diagnostic incorrect");
    check(fields["explicit_enable_count"]=="0","disabled stop changed explicit enable diagnostic");
  }
  static void run(RuntimeNode& node,const std::string& scenario){
    check(node.has_parameter("auto_start"),"optional native auto_start parameter is missing");
    check(!node.enabled_,"startup must be disabled");
    check(!node.startup_instance_.empty(),"native process identity missing without supervisor");
    check(node.explicit_enable_count_==0,"startup cannot be an explicit enable");
    if(scenario=="default"){
      node.publish_command();check(!node.enabled_&&!node.auto_start_pending_,"default startup enabled controller");return;
    }
    check(node.auto_start_pending_,"optional startup did not enter pending state");
    if(scenario=="diagnostics"){
      stop(node);stop(node);diagnostics(node);return;
    }
    if(scenario=="stop"){
      stop(node);check(node.stop_request_count_==1&&!node.auto_start_pending_,"stop did not cancel pending startup");
      stop(node);check(node.stop_request_count_==2,"every disabled stop must count");
    }
    if(scenario=="timeout"){
      attempt(node,node.auto_start_deadline_+.001);
      check(!node.auto_start_pending_&&!node.enabled_,"wall deadline did not cancel pending startup");return;
    }
    const double deadline=steady()+3.;
    for(;;){bool ready=false;{std::lock_guard<std::mutex> lock(node.mutex_);ready=node.solver_ready_;}
      if(ready)break;check(steady()<deadline,"worker failed to become ready");std::this_thread::yield();}
    if(scenario=="anchor_rc_epoch"||scenario=="anchor_worker_epoch"){
      node.accept_health(health(node,"trusted"));
      {std::lock_guard<std::mutex> lock(node.mutex_);node.identity_=node.bundle_.reference().map_sha256();
        node.mode_=scenario!="anchor_rc_epoch";node.mode_received_=steady();
        if(scenario=="anchor_worker_epoch")node.solver_ready_=false;}
      Drive stopped;node.forwarded(stopped);
      const auto r=node.bundle_.reference().at(0.);nav_msgs::msg::Odometry odom;
      odom.header.frame_id="odom";odom.child_frame_id="base_link";odom.header.stamp=node.get_clock()->now();
      odom.pose.pose.position.x=r.x+node.cfg_.rear_offset*std::cos(r.yaw);
      odom.pose.pose.position.y=r.y+node.cfg_.rear_offset*std::sin(r.yaw);
      odom.pose.pose.orientation.z=std::sin(r.yaw/2);odom.pose.pose.orientation.w=std::cos(r.yaw/2);
      node.odometry(odom);
      {std::lock_guard<std::mutex> lock(node.mutex_);
        check(node.snapshot_.present&&node.localization_locked(steady()),"trusted snapshot fixture was not established");}
      attempt(node);check(!node.enabled_&&node.auto_start_pending_,"readiness prerequisite did not keep startup waiting");
      node.accept_health(health(node,"replacement"));
      check(!node.auto_start_pending_,"epoch fault after trusted anchor incorrectly preserved startup");return;
    }
    if(scenario=="worker"){
      {std::lock_guard<std::mutex> lock(node.mutex_);node.solver_ready_=false;}
      attempt(node);check(node.auto_start_pending_&&node.phase_=="READY","worker readiness caused fault or consumed startup");
      {std::lock_guard<std::mutex> lock(node.mutex_);node.solver_ready_=true;}
    }
    auto unready=health(node,"initializing");unready["ready"]="false";node.accept_health(unready);
    node.accept_health(health(node,"first"));
    if(scenario!="stop")check(node.auto_start_pending_,"normal initial epoch FAULT cancelled startup");
    if(scenario=="epoch"){
      node.accept_health(health(node,"second"));
      check(!node.auto_start_pending_,"later localization epoch fault preserved startup");return;
    }
    if(scenario=="clock"){
      {std::lock_guard<std::mutex> lock(node.mutex_);node.observe_clock_locked(100.,steady());node.observe_clock_locked(99.,steady());}
      check(!node.auto_start_pending_,"clock fault hidden by initial FAULT did not cancel startup");return;
    }
    if(scenario=="fault"){
      {std::lock_guard<std::mutex> lock(node.mutex_);node.fault_locked("Solver initialization failed");}
      check(!node.auto_start_pending_,"fatal fault hidden by initial FAULT did not cancel startup");return;
    }
    attempt(node);check(!node.enabled_,"startup accepted absent fresh observation");
    fresh(node,false);attempt(node);check(!node.enabled_,"startup accepted absent selector authority");
    fresh(node);
    if(node.bundle_.reference().frame_id()=="map"){
      attempt(node);check(!node.enabled_,"startup accepted wrong map identity");
      {std::lock_guard<std::mutex> lock(node.mutex_);node.identity_=node.bundle_.reference().map_sha256();}
      auto stale=health(node,"first");stale["health_sequence"]="2";stale["anchor_sequence"]="2";stale["alignment_anchor_sequence"]="2";
      const auto stamp=std::to_string(node.get_clock()->now().nanoseconds()-2000000000LL);
      stale["last_anchor_stamp_ns"]=stale["alignment_stamp_ns"]=stamp;node.accept_health(stale);
      fresh(node);attempt(node);check(!node.enabled_,"startup accepted stale trusted localization");
      auto trusted=health(node,"first");trusted["health_sequence"]="3";trusted["anchor_sequence"]="3";trusted["alignment_anchor_sequence"]="3";
      node.accept_health(trusted);
    }
    fresh(node);
    if(scenario=="heading"){
      {std::lock_guard<std::mutex> lock(node.mutex_);node.snapshot_.state[2]+=std::acos(-1.);}
      attempt(node);check(!node.auto_start_pending_&&!node.enabled_,"unsafe heading refusal did not cancel startup");return;
    }
    if(scenario=="manual"){
      auto request=std::make_shared<std_srvs::srv::SetBool::Request>();request->data=true;
      auto response=std::make_shared<std_srvs::srv::SetBool::Response>();node.enable_request(request,response);
      check(response->success&&node.explicit_enable_count_==1&&!node.auto_start_pending_,"manual enable did not consume pending auto startup");return;
    }
    if(scenario=="output")node.publish_command();else attempt(node);
    if(scenario=="stop"){
      check(!node.enabled_&&node.auto_start_count_==0,"stop before first enable did not prevent startup");return;
    }
    check(node.enabled_&&!node.auto_start_pending_&&node.auto_start_count_==1,"fresh trusted startup was not accepted exactly once");
    check(node.explicit_enable_count_==0,"automatic start incorrectly counted as explicit enable");
    {std::lock_guard<std::mutex> lock(node.mutex_);node.enabled_=false;node.phase_="COMPLETE";}
    attempt(node);check(!node.enabled_&&node.auto_start_count_==1,"completion re-enabled startup");
    {std::lock_guard<std::mutex> lock(node.mutex_);node.fault_locked("Solver timeout");}
    attempt(node);check(!node.enabled_&&node.auto_start_count_==1,"fault re-enabled startup");
  }
};
}
int main(int argc,char** argv){
  if(argc<3)throw std::invalid_argument("bundle and scenario required");
  const std::string scenario=argv[2];unsetenv("AIMS_MPCC_RT_SUPERVISOR_INSTANCE");
  std::vector<std::string> args{"auto_start_test","--ros-args","-p",std::string("artifact_directory:=")+argv[1],
    "-p","simulation:=true","-p","repeat_laps:=true"};
  if(scenario!="default")args.insert(args.end(),{"-p","auto_start:=true"});
  for(const std::string topic:{"/drive","/mpcc/status","/mpcc/reference","/mpcc/prediction","/mpcc/enable",
      "/ackermann_cmd","/odometry/filtered","/control/autonomy_speed_enabled","/localization/status","/localization/map_sha256"})
    args.insert(args.end(),{"-r",topic+":=/auto_start_test"+topic});
  std::vector<char*> raw;for(auto& arg:args)raw.push_back(arg.data());rclcpp::init(raw.size(),raw.data());
  int result=0;try{auto node=std::make_shared<aims_mpcc_rt::RuntimeNode>();
    aims_mpcc_rt::RuntimeClockProbe::run(*node,scenario);
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';result=1;}rclcpp::shutdown();return result;
}

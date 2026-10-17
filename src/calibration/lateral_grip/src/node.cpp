#include "aims_lateral_grip/core.hpp"
#include "aims_lateral_grip/evidence.hpp"
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <vesc_msgs/msg/vesc_command.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <crsf_receiver_msg/msg/crsf_channels16.hpp>
#include <openssl/sha.h>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <deque>
#include <chrono>
#include <cstdlib>
#include <unistd.h>
#include <iostream>
#include <map>

using json=nlohmann::json;
using namespace aims_lateral_grip;
using Command=vesc_msgs::msg::VescCommand;
static double steady(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static double stamp(const builtin_interfaces::msg::Time&t){return t.sec+t.nanosec*1e-9;}
static std::string hash(const std::string&s){unsigned char digest[SHA256_DIGEST_LENGTH];SHA256(reinterpret_cast<const unsigned char*>(s.data()),s.size(),digest);std::ostringstream out;for(auto b:digest)out<<std::hex<<std::setw(2)<<std::setfill('0')<<int(b);return out.str();}
static std::string file_hash(const std::string&p){std::ifstream f(p,std::ios::binary);if(!f)return "unavailable";std::ostringstream data;data<<f.rdbuf();return hash(data.str());}
static std::string csv(const std::string&s){std::string out="\"";for(char c:s){out+=c;if(c=='\"')out+='\"';}return out+'\"';}
class LateralNode:public rclcpp::Node {
 public:
  LateralNode():Node("lateral_grip_calib"),core_(Config{}){
    Config c;
    c.selected=declare_parameter("physical_configuration_selected",false);
    c.wheelbase=declare_parameter("wheelbase",0.);
    c.radius=declare_parameter("nominal_radius",0.);
    c.speeds=declare_parameter<std::vector<double>>("speed_steps",std::vector<double>{});
    c.speed_limit=declare_parameter("speed_limit",0.);
    c.steering_limit=declare_parameter("steering_limit",0.);
    c.ramp_rate=declare_parameter("ramp_rate",0.);
    c.speed_tolerance=declare_parameter("speed_tolerance",.1);
    c.settle_seconds=declare_parameter("settle_seconds",1.);
    c.hold_seconds=declare_parameter("hold_seconds",3.);
    c.stage_timeout=declare_parameter("stage_timeout",30.);
    c.minimum_hold_samples=declare_parameter("minimum_hold_samples",10);
    c.stop_seconds=declare_parameter("stop_seconds",1.);
    c.stop_timeout=declare_parameter("stop_timeout",15.);
    c.freshness=declare_parameter("freshness_seconds",.25);
    c.standstill_speed=declare_parameter("standstill_speed",.05);
    max_current_=declare_parameter("max_motor_current",0.);
    min_voltage_=declare_parameter("min_voltage",0.);
    max_temperature_=declare_parameter("max_temperature",0.);
    max_latency_=declare_parameter("max_command_latency",.1);
    profile_=declare_parameter("rc_profile",std::string("steering_ch1_throttle_ch3_aux_ch5_to_ch10"));
    output_root_=declare_parameter("output_root",std::string(""));
    if(output_root_.empty()){
      const char* user_home=std::getenv("HOME");
      if(user_home==nullptr||*user_home=='\0')
        throw std::runtime_error("HOME unavailable: set an explicit durable output_root");
      output_root_=(std::filesystem::path(user_home)/"aimsracer-data/sessions/lateral-grip").string();
    }
    source_revision_=declare_parameter("source_revision",std::string("unrecorded"));
    if(!has_parameter("use_sim_time"))declare_parameter("use_sim_time",false);
    sim_=get_parameter("use_sim_time").as_bool();
    limits_ok_=std::isfinite(max_current_)&&max_current_>0&&std::isfinite(min_voltage_)&&min_voltage_>0&&std::isfinite(max_temperature_)&&max_temperature_>0&&std::isfinite(max_latency_)&&max_latency_>0&&max_latency_<=c.freshness;
    cfg_=c;core_=Core(c);
    metadata_={{"schema",1},{"package","aims_lateral_grip"},{"source_revision",source_revision_},
      {"parameters",{{"physical_configuration_selected",c.selected},{"wheelbase",c.wheelbase},{"nominal_radius",c.radius},{"speed_steps",c.speeds},{"speed_limit",c.speed_limit},{"steering_limit",c.steering_limit},{"ramp_rate",c.ramp_rate},{"speed_tolerance",c.speed_tolerance},{"settle_seconds",c.settle_seconds},{"hold_seconds",c.hold_seconds},{"stage_timeout",c.stage_timeout},{"minimum_hold_samples",c.minimum_hold_samples},{"stop_seconds",c.stop_seconds},{"stop_timeout",c.stop_timeout},{"freshness_seconds",c.freshness},{"standstill_speed",c.standstill_speed},{"max_motor_current",max_current_},{"min_voltage",min_voltage_},{"max_temperature",max_temperature_},{"max_command_latency",max_latency_},{"rc_profile",profile_},{"use_sim_time",sim_}}},
      {"notes",{"steering radius is nominal atan(wheelbase/R)","zero current requests coast; no claim of standstill","source timestamps per channel; no source alignment of latest values","RC and calibration Bool authority have no source stamps; receipt ROS time marked explicitly","combined slip trials excluded; no mu_safe or planner update"}}};
    metadata_["output_root"]=output_root_;
    metadata_["config_sha256"]=hash(metadata_["parameters"].dump());
    char executable[4096];auto size=readlink("/proc/self/exe",executable,sizeof(executable)-1);
    if(size>0){executable[size]=0;metadata_["executable"]=executable;metadata_["executable_sha256"]=file_hash(executable);}
    auto token=std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count())+"-"+std::to_string(getpid());
    directory_=std::filesystem::path(output_root_)/token;std::filesystem::create_directories(directory_);
    std::ofstream metadata_file(directory_/"metadata.json");
    if(!write_checked(metadata_file,metadata_.dump(2)+"\n"))throw std::runtime_error("metadata write/flush failed");
    evidence_.open(directory_/"evidence.csv");evidence_<<"channel,source_time,receive_ros,receive_steady,state,phase,direction,stage,x,y,z,quality,detail\n";
    if(!evidence_)throw std::runtime_error("cannot open evidence");
    publisher_=create_publisher<Command>("/calib/vesc_command",rclcpp::QoS(1).reliable());
    status_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/calib/lateral/status",10);
    const auto sensor=rclcpp::SensorDataQoS();
    subscribe<nav_msgs::msg::Odometry>("/rear_axle/wheel_odom",sensor,[this](const auto&m){
      bool valid=m.child_frame_id=="base_link"&&std::isfinite(m.twist.twist.linear.x);
      input_.speed=valid?m.twist.twist.linear.x:NAN;update(0,stamp(m.header.stamp),valid);
      record("wheel",stamp(m.header.stamp),m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z,valid?"ok":"invalid",{{"frame",m.header.frame_id},{"child",m.child_frame_id}});
    });
    subscribe<vesc_msgs::msg::VescStateStamped>("/sensors/core",sensor,[this](const auto&m){
      const auto&s=m.state;input_.telemetry_ok=limits_ok_&&s.fault_code==0&&std::isfinite(s.current_motor)&&std::abs(s.current_motor)<=max_current_&&std::isfinite(s.voltage_input)&&s.voltage_input>=min_voltage_&&std::isfinite(s.speed)&&std::isfinite(s.temp_fet)&&s.temp_fet<=max_temperature_&&std::isfinite(s.temp_motor)&&s.temp_motor<=max_temperature_;
      update(1,stamp(m.header.stamp),input_.telemetry_ok);
      record("telemetry",stamp(m.header.stamp),s.current_motor,s.voltage_input,s.speed,input_.telemetry_ok?"ok":"invalid",{{"avg_iq",s.avg_iq},{"avg_id",s.avg_id},{"fault_code",s.fault_code},{"temp_fet",s.temp_fet},{"temp_motor",s.temp_motor},{"current_input",s.current_input},{"duty_cycle",s.duty_cycle}});
    });
    subscribe<crsf_receiver_msg::msg::CRSFChannels16>("/rc/channels",sensor,[this](const auto&m){
      std::array<int,16>v={m.ch1,m.ch2,m.ch3,m.ch4,m.ch5,m.ch6,m.ch7,m.ch8,m.ch9,m.ch10,m.ch11,m.ch12,m.ch13,m.ch14,m.ch15,m.ch16};
      bool standard=profile_=="steering_ch1_throttle_ch3_aux_ch5_to_ch10";
      input_.rc_ok=standard||profile_=="sequential_ch1_ch2";
      auto used=standard?std::vector<int>{1,3,5,6,7,8,10}:std::vector<int>{1,2,3,4,5,6,7};
      for(int k:used)input_.rc_ok=input_.rc_ok&&v[k-1]>0&&v[k-1]<=2047;
      input_.rc_ok=input_.rc_ok&&v[standard?4:2]>=992&&v[standard?7:6]>992;
      update(2,now().seconds(),input_.rc_ok,false);
      record("rc_receipt_time",now().seconds(),0,0,0,input_.rc_ok?"ok":"invalid",{{"channels",v},{"source_stamp_present",false}});
    });
    subscribe<std_msgs::msg::Bool>("/control/calibration_enabled",rclcpp::QoS(1).reliable(),[this](const auto&m){input_.authority=m.data;update(3,now().seconds(),m.data,false);record("authority_receipt_time",now().seconds(),m.data,0,0,m.data?"ok":"withdrawn",{{"source_stamp_present",false}});});
    subscribe<Command>("/vesc/command",rclcpp::QoS(1).reliable(),[this](const auto&m){
      const double t=steady(),age=now().seconds()-stamp(m.header.stamp);
      bool finite=std::isfinite(m.steering_angle)&&std::isfinite(m.target_speed_mps)&&std::isfinite(m.target_current)&&std::isfinite(m.target_duty)&&std::isfinite(m.target_brake_current)&&std::isfinite(m.target_acceleration_mps2);
      bool matched=false;
      for(const auto &sent:history_){const auto &r=sent.second;if(t-sent.first>max_latency_)continue;
        matched=matched||(m.control_mode==r.mode&&std::abs(m.steering_angle-r.steering)<1e-4&&std::abs(m.target_speed_mps-r.speed)<1e-4&&std::abs(m.target_current-r.current)<1e-4);}
      input_.final_ok=finite&&matched&&m.header.frame_id=="base_link"&&age>=0&&age<=max_latency_&&m.target_duty==0&&m.target_brake_current==0&&m.target_acceleration_mps2==0;
      update(4,stamp(m.header.stamp),input_.final_ok);
      record("final_command",stamp(m.header.stamp),m.steering_angle,m.target_speed_mps,m.target_current,input_.final_ok?"ok":"mismatch",{{"mode",m.control_mode},{"duty",m.target_duty},{"brake",m.target_brake_current},{"acceleration",m.target_acceleration_mps2}});
    });
    for(const auto& frame:{std::string("rear_axle"),std::string("center_of_mass")}){
      const std::string base="/state_estimation/"+frame;
      subscribe<geometry_msgs::msg::TwistWithCovarianceStamped>(base+"/velocity",sensor,[this,frame](const auto&m){const auto&v=m.twist.twist;observer(frame+"/velocity",stamp(m.header.stamp),v.linear.x,v.linear.y,v.angular.z,m.header.frame_id,{{"covariance",m.twist.covariance}});});
      subscribe<geometry_msgs::msg::AccelWithCovarianceStamped>(base+"/acceleration",sensor,[this,frame](const auto&m){const auto&a=m.accel.accel;observer(frame+"/acceleration",stamp(m.header.stamp),a.linear.x,a.linear.y,a.angular.z,m.header.frame_id,{{"covariance",m.accel.covariance}});});
      subscribe<sensor_msgs::msg::Imu>(base+"/yaw_rate",sensor,[this,frame](const auto&m){observer(frame+"/yaw_rate",stamp(m.header.stamp),m.angular_velocity.z,0,0,m.header.frame_id,{{"angular_velocity_covariance",m.angular_velocity_covariance}});});
      subscribe<diagnostic_msgs::msg::DiagnosticArray>(base+"/status",rclcpp::QoS(10),[this,frame](const auto&m){
        json detail=json::array();for(const auto&status:m.status){json values=json::object();for(const auto&v:status.values)values[v.key]=v.value;detail.push_back({{"level",status.level},{"name",status.name},{"message",status.message},{"values",values}});if(status.level>0)last_observer_warning_=steady();}
        record(frame+"/status",stamp(m.header.stamp),0,0,0,"reported",detail);
      });
    }
    for(const auto &topic:{std::string("/livox/imu"),std::string("/livox/imu_bias_corrected")}){
      subscribe<sensor_msgs::msg::Imu>(topic,sensor,[this,topic](const auto&m){
        record(topic,stamp(m.header.stamp),m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z,"raw",{{"frame",m.header.frame_id},{"angular_velocity",{m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z}},{"orientation",{m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w}},{"linear_acceleration_covariance",m.linear_acceleration_covariance},{"angular_velocity_covariance",m.angular_velocity_covariance}});
      });
    }
    driver_=create_client<rcl_interfaces::srv::GetParameters>("/vesc_driver_node/get_parameters");
    service_=create_service<std_srvs::srv::SetBool>("/calib/lateral/enable",[this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,std::shared_ptr<std_srvs::srv::SetBool::Response> response){
      refresh();if(request->data){response->success=io_ok_&&!sim_&&limits_ok_&&driver_ready()&&core_.enable(input_);response->message=response->success?"single-use session started; measured standstill required":"start refused: check configuration, use_sim_time=false, fresh authority/control/final/exclusive publishers, standstill and verified driver current_startup_enabled=false; session cannot restart";}
      else{core_.stop("operator_stop");response->success=true;response->message="zero-current coast requested; standstill not asserted";}
      summary();if(!io_ok_){response->success=false;response->message="evidence I/O failed; zero-current coast requested";}
    });
    parameter_guard_=add_on_set_parameters_callback([](const auto&){rcl_interfaces::msg::SetParametersResult result;result.successful=false;result.reason="session parameters immutable; restart node with explicit config";return result;});
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{tick();});
    RCLCPP_INFO(get_logger(),"Lateral calibration IDLE; output=%s; explicit /calib/lateral/enable required",directory_.c_str());summary();
  }
  ~LateralNode() override {shutdown_evidence();}
  void shutdown_evidence() noexcept {if(closed_)return;core_.stop("process_shutdown");summary();if(!write_checked(evidence_,""))io_fault("shutdown_flush_failed");evidence_.close();closed_=true;}
 private:
  template<class T,class F>void subscribe(const std::string&topic,const rclcpp::QoS&qos,F f){subscriptions_.push_back(create_subscription<T>(topic,qos,[f](const typename T::ConstSharedPtr m){f(*m);}));}
  void update(std::size_t k,double source,bool valid,bool stamped=true){auto&sample=input_.samples[k];if(stamped&&sample.source>0&&source<=sample.source)valid=false;sample={source,steady(),valid};if(!valid)core_.stop("invalid_or_nonmonotonic_control_sample");}
  void record(const std::string&channel,double source,double x,double y,double z,const std::string&quality,const json&detail=json::object()){
    std::ostringstream row;
    row<<std::setprecision(17)<<channel<<','<<source<<','<<now().seconds()<<','<<steady()<<','<<name(core_.state())<<','<<core_.phase()<<','<<core_.direction()<<','<<core_.stage()<<','<<x<<','<<y<<','<<z<<','<<quality<<','<<csv(detail.dump())<<'\n';
    if(!write_checked(evidence_,row.str()))io_fault("evidence_write_or_flush_failed");
  }
  void observer(const std::string&channel,double source,double x,double y,double z,const std::string&frame,json detail){
    const double age=now().seconds()-source;bool ok=std::isfinite(x)&&std::isfinite(y)&&std::isfinite(z)&&std::isfinite(age)&&age>=0&&age<=cfg_.freshness&&(frame=="base_link"||frame=="center_of_mass"||frame=="rear_axle");
    detail["frame"]=frame;record(channel,source,x,y,z,ok?"ok":"warn",detail);
    observer_receipt_[channel]=steady();if(!ok)last_observer_warning_=steady();
  }
  void io_fault(const std::string& reason) noexcept {
    io_ok_=false;core_.stop(reason);
    RCLCPP_ERROR(get_logger(),"Evidence failure: %s; zero-current coast requested",reason.c_str());
  }
  bool driver_ready()const{return driver_disabled_&&steady()-driver_received_<=1.;}
  void poll_driver(){
    const double t=steady();
    if(driver_pending_&&t-driver_requested_>.5){driver_->remove_pending_request(driver_request_id_);driver_pending_=false;driver_disabled_=false;core_.stop("driver_parameter_timeout");}
    if(driver_pending_||t-driver_requested_<.5||!driver_->service_is_ready())return;
    driver_pending_=true;driver_requested_=t;
    auto request=std::make_shared<rcl_interfaces::srv::GetParameters::Request>();request->names={"current_startup_enabled"};
    auto future=driver_->async_send_request(request,[this](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture response){
      driver_pending_=false;
      try{auto values=response.get()->values;driver_disabled_=values.size()==1&&values[0].type==1&&!values[0].bool_value;driver_received_=steady();record("driver_current_startup_enabled_receipt_time",now().seconds(),driver_disabled_?0:1,0,0,driver_disabled_?"ok":"invalid",{{"values",values.size()},{"verified_disabled",driver_disabled_}});if(!driver_disabled_)core_.stop("driver_current_startup_not_disabled");}
      catch(...){driver_disabled_=false;core_.stop("driver_parameter_error");}
    });driver_request_id_=future.request_id;
  }
  void refresh(){input_.clock=now().seconds();input_.steady=steady();input_.exclusive=count_publishers("/calib/vesc_command")==1&&count_publishers("/vesc/command")==1;}
  void tick(){
    poll_driver();refresh();if(core_.active()&&!driver_ready())core_.stop("driver_parameter_not_verified_or_stale");if(sim_&&core_.active())core_.stop("sim_time_driving_forbidden");
    core_.tick(input_);if(!io_ok_)core_.stop("evidence_io_failed");auto request=core_.request();
    record("request",now().seconds(),request.steering,request.speed,request.current,"ok",{{"mode",request.mode}});
    if(!io_ok_)request=Request{};
    Command m;m.header.stamp=now();m.header.frame_id="base_link";m.control_mode=request.mode;m.steering_angle=request.steering;m.target_speed_mps=request.speed;m.target_current=request.current;publisher_->publish(m);
    history_.push_back({steady(),request});while(history_.size()>100)history_.pop_front();
    bool warn=steady()-last_observer_warning_<1.;
    for(const auto&frame:{"rear_axle","center_of_mass"})for(const auto&channel:{"velocity","acceleration","yaw_rate"}){std::string key=std::string(frame)+"/"+channel;warn=warn||!observer_receipt_.count(key)||steady()-observer_receipt_[key]>cfg_.freshness;}
    diagnostic_msgs::msg::DiagnosticArray msg;msg.header.stamp=now();diagnostic_msgs::msg::DiagnosticStatus s;s.name="lateral_grip";s.hardware_id="calibration";s.level=core_.state()==State::Aborted||!io_ok_?2:warn?1:0;s.message=std::string(name(core_.state()))+": "+core_.reason();
    auto add=[&](std::string k,std::string v){diagnostic_msgs::msg::KeyValue kv;kv.key=k;kv.value=v;s.values.push_back(kv);};
    add("output_directory",directory_.string());add("configuration_valid",cfg_.valid()&&limits_ok_?"true":"false");add("driver_current_startup_disabled_verified",driver_ready()?"true":"false");add("exclusive_publishers",input_.exclusive?"true":"false");add("authority",input_.authority?"true":"false");add("rc_valid",input_.rc_ok?"true":"false");add("final_command_match",input_.final_ok?"true":"false");add("evidence_io_ok",io_ok_?"true":"false");add("phase",core_.phase());
    for(std::size_t k=0;k<input_.samples.size();++k){const auto&sample=input_.samples[k];add("control_"+std::to_string(k)+"_source_age",std::to_string(input_.clock-sample.source));add("control_"+std::to_string(k)+"_receive_age",std::to_string(input_.steady-sample.received));add("control_"+std::to_string(k)+"_valid",sample.valid?"true":"false");}add("observer_quality",warn?"WARN: missing, stale or invalid; recording continues":"available");add("config_sha256",metadata_["config_sha256"].get<std::string>());add("direction",std::to_string(core_.direction()));add("stage",std::to_string(core_.stage()));msg.status.push_back(s);status_->publish(msg);
    if(++ticks_%10==0)summary();
  }
  void summary() noexcept {
    try{
      json status={{"state",name(core_.state())},{"phase",core_.phase()},{"reason",core_.reason()},{"direction",core_.direction()},{"stage",core_.stage()},{"config_sha256",metadata_["config_sha256"]},{"coast_is_not_standstill",true},{"completed_stages",core_.completed_stages()},{"incomplete_stage",core_.incomplete_stage()},{"evidence_io_ok",io_ok_},{"ros_time",now().seconds()}};
      auto temp=directory_/"summary.json.tmp";std::ofstream file(temp);
      if(!write_checked(file,status.dump(2)+"\n")){io_fault("summary_write_or_flush_failed");return;}
      file.close();std::filesystem::rename(temp,directory_/"summary.json");
    }catch(const std::exception&error){io_fault(error.what());}
  }
  Config cfg_;Core core_;Inputs input_;json metadata_;std::filesystem::path directory_;std::ofstream evidence_;
  std::string profile_,output_root_,source_revision_;bool sim_{false},limits_ok_{false},closed_{false},io_ok_{true};double max_current_{0},min_voltage_{0},max_temperature_{0},max_latency_{.1},last_observer_warning_{-100};unsigned ticks_{0};
  std::deque<std::pair<double,Request>> history_;std::map<std::string,double>observer_receipt_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Publisher<Command>::SharedPtr publisher_;rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr driver_;
  bool driver_disabled_{false},driver_pending_{false};double driver_received_{-100},driver_requested_{-100};int64_t driver_request_id_{0};
  OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{auto node=std::make_shared<LateralNode>();rclcpp::spin(node);node->shutdown_evidence();node.reset();}catch(const std::exception&e){std::cerr<<"lateral_grip_calib: "<<e.what()<<'\n';if(rclcpp::ok())rclcpp::shutdown();return 1;}if(rclcpp::ok())rclcpp::shutdown();return 0;}

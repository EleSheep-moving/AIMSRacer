// Independent parallel observation: no control, TF, wheel or EKF consumers.
#include "aims_racer_system/velocity.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
namespace vx=aims_racer_system::velocity;
class VelocityNode final:public rclcpp::Node{
 public:
 VelocityNode():Node("vx_observer"){
  vx::Config c;
#define LOAD(k) c.k=parameter(#k,c.k)
  LOAD(gravity);LOAD(position_std);LOAD(bias_psd);LOAD(force_variance);LOAD(tilt_std);LOAD(initial_bias_std);
  LOAD(initialization_span);LOAD(position_gate);LOAD(angle_gate);LOAD(max_gap);LOAD(degraded_age);LOAD(max_age);LOAD(history);
#undef LOAD
  const int64_t n=parameter<int64_t>("max_events",4096);if(n<8||n>100000)throw std::invalid_argument("max_events outside8..100000");c.max_events=n;
  auto lever=parameter("livox_translation",std::vector<double>{.3,0,.03});
  auto q=parameter("livox_quaternion",std::vector<double>{0,0,0,1});
  if(lever.size()!=3||q.size()!=4)throw std::invalid_argument("geometry dimensions");
  c.lever={lever[0],lever[1],lever[2]};Eigen::Quaterniond rotation(q[3],q[0],q[1],q[2]);
  if(!rotation.coeffs().allFinite()||std::abs(rotation.norm()-1)>1e-9)throw std::invalid_argument("invalid mounting quaternion");
  c.mounting=rotation.toRotationMatrix();observer_=std::make_unique<vx::Observer>(c);lio_max_=c.max_age;
  max_age_=parameter("imu_max_age_s",.05);if(!std::isfinite(max_age_)||max_age_<=0||max_age_>.05)throw std::invalid_argument("IMU age must be <=50ms");
  imu_frame_=parameter<std::string>("imu_frame","livox_frame");world_frame_=parameter<std::string>("world_frame","odom");body_frame_=parameter<std::string>("body_frame","base_link");
  output_=parameter<std::string>("output_topic","/state_estimation/vx");
  pub_=create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(output_,100);
  status_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>(output_+"/status",10);
  debug_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>(output_+"/debug",10);
  // Both production publishers offer reliable QoS. A reliable depth256 input
  // also preserves receipt-order replay under offline publication bursts.
  imu_=create_subscription<sensor_msgs::msg::Imu>(parameter<std::string>("imu_topic","/livox/imu_bias_corrected"),rclcpp::QoS(256).reliable(),[this](sensor_msgs::msg::Imu::ConstSharedPtr m){timed([&]{receive(*m);});});
  lio_=create_subscription<nav_msgs::msg::Odometry>(parameter<std::string>("lio_topic","/fastlio2/lio_odom"),rclcpp::QoS(256).reliable(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m){timed([&]{receive(*m);});});
  timer_=create_wall_timer(std::chrono::milliseconds(100),[this]{diagnostics();});
 }
 private:
 template<class T>T parameter(const std::string&name,const T&value){rcl_interfaces::msg::ParameterDescriptor d;d.read_only=true;return declare_parameter<T>(name,value,d);}
 template<class F>void timed(F action){const auto b=std::chrono::steady_clock::now();action();if(cost_.size()==512)cost_.pop_front();cost_.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-b).count());}
 int64_t clock_ns(){int64_t t=now().nanoseconds();if(last_clock_&&t<*last_clock_){observer_->reset();published_.reset();imu_receipt_.reset();lio_receipt_.reset();++count_["clock_resets"];reason_="clock_reset";}last_clock_=t;return t;}
 void reject(const std::string&s){++count_[s];reason_=s;}
 std::optional<int64_t> stamp(const builtin_interfaces::msg::Time&h,int64_t t){
  if(h.sec<0||h.nanosec>=1000000000u){reject("invalid_stamp");return {};}
  const int64_t ns=int64_t(h.sec)*1000000000LL+h.nanosec;
  if(ns<=0){reject("invalid_stamp");return {};}if(ns>t){reject("future_stamp");return {};}return ns;
 }
 bool good(int64_t t)const{const auto e=observer_->latest();return e.valid&&std::isfinite(e.vx)&&std::isfinite(e.variance)&&e.variance>0&&e.stamp<=t&&(t-e.stamp)*1e-9<=max_age_&&(t-e.anchor)*1e-9<=lio_max_+1e-12;}
 void receive(const sensor_msgs::msg::Imu&m){
  const auto t=clock_ns();imu_receipt_=t;++count_["imu_callbacks"];
  if(m.header.frame_id!=imu_frame_){reject("wrong_frame");return;}
  if(m.angular_velocity_covariance[0]<0||m.linear_acceleration_covariance[0]<0){reject("unavailable");return;}
  for(const auto&cov:{m.angular_velocity_covariance,m.linear_acceleration_covariance})for(double v:cov)if(!std::isfinite(v)){reject("nonfinite");return;}
  auto ns=stamp(m.header.stamp,t);if(!ns)return;if((t-*ns)*1e-9>max_age_){reject("stale_imu");return;}
  if(!observer_->add_imu({*ns,{m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z},{m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z}})){reject("rejected_imu");return;}
  reason_.clear();auto e=observer_->latest();if(!good(t)||(published_&&e.stamp<=*published_))return;
  geometry_msgs::msg::TwistWithCovarianceStamped out;out.header.stamp=rclcpp::Time(e.stamp);out.header.frame_id=body_frame_;
  out.twist.twist.linear.x=e.vx;out.twist.covariance[0]=e.variance;
  for(unsigned k:{7u,14u,21u,28u,35u})out.twist.covariance[k]=1e6;
  pub_->publish(out);published_=e.stamp;++count_["published"];
 }
 void receive(const nav_msgs::msg::Odometry&m){
  const auto t=clock_ns();lio_receipt_=t;++count_["lio_callbacks"];
  if(m.header.frame_id!=world_frame_||m.child_frame_id!=imu_frame_){reject("wrong_frame");return;}
  for(double v:m.pose.covariance)if(!std::isfinite(v)){reject("nonfinite");return;}
  auto ns=stamp(m.header.stamp,t);if(!ns)return;
  const auto&p=m.pose.pose.position;const auto&q=m.pose.pose.orientation;
  auto before=observer_->latest().rejected;
  if(!observer_->add_lio({*ns,{p.x,p.y,p.z},Eigen::Quaterniond(q.w,q.x,q.y,q.z)})){reject("rejected_lio_input");return;}
  if(observer_->latest().rejected>before){reject("rejected_lio_innovation");return;}
  ++count_["lio_admitted"]; // A future IMU interval can still reject this anchor.
 }
 template<class T>static void field(diagnostic_msgs::msg::DiagnosticStatus&s,const std::string&k,const T&v){diagnostic_msgs::msg::KeyValue x;x.key=k;x.value=std::to_string(v);s.values.push_back(x);}
 static void field(diagnostic_msgs::msg::DiagnosticStatus&s,const std::string&k,const std::string&v){diagnostic_msgs::msg::KeyValue x;x.key=k;x.value=v;s.values.push_back(x);}
 void diagnostics(){
  const auto t=clock_ns();const auto e=observer_->latest();diagnostic_msgs::msg::DiagnosticArray a;a.header.stamp=now();a.header.frame_id=body_frame_;
  diagnostic_msgs::msg::DiagnosticStatus s;s.name="vx_observer";s.hardware_id="corrected_imu_raw_lio_shared_information";
  s.level=good(t)&&e.status=="valid"?s.OK:s.WARN;s.message=good(t)?e.status:(!e.valid?e.status:"input_stale");
  field(s,"valid",std::string(good(t)?"true":"false"));field(s,"last_rejection",reason_);field(s,"source_stamp_ns",e.stamp);field(s,"anchor_stamp_ns",e.anchor);
  field(s,"imu_source_age_s",e.stamp?(t-e.stamp)*1e-9:-1.);field(s,"lio_anchor_source_age_s",e.anchor?(t-e.anchor)*1e-9:-1.);
  field(s,"imu_receipt_age_s",imu_receipt_?(t-*imu_receipt_)*1e-9:-1.);field(s,"lio_receipt_age_s",lio_receipt_?(t-*lio_receipt_)*1e-9:-1.);
  field(s,"reference",std::string("base_link_rear_axle_approximate_IMU_lever"));field(s,"covariance",std::string("conditional_effective_uncalibrated_shared_LIO_IMU"));
  field(s,"other_velocity_components",std::string("unverified_placeholders_covariance_1e6"));field(s,"history_size",observer_->history_size());field(s,"replayed_imus",observer_->replayed_imus());field(s,"rejected_lio_innovations",e.rejected);field(s,"lio_reinitializations",e.reacquisitions);
  for(const auto&kv:count_)field(s,kv.first,kv.second);
  if(!cost_.empty()){std::vector<double>x(cost_.begin(),cost_.end());std::sort(x.begin(),x.end());field(s,"callback_window_n",x.size());field(s,"callback_window_p99_ms",x[std::size_t(std::ceil(.99*x.size()))-1]);field(s,"callback_window_max_ms",x.back());}
  a.status.push_back(s);status_->publish(a);
  s.values.clear();s.name="vx_observer_internal_diagnostics_only";
  for(int i=0;i<3;++i){const std::string k=std::to_string(i);field(s,"p_W_"+k,e.p[i]);field(s,"v_W_"+k,e.v[i]);field(s,"b_eff_I_"+k,e.b[i]);field(s,"position_innovation_W_"+k,e.innovation[i]);}
  field(s,"vx",e.vx);field(s,"variance",e.variance);a.status={s};debug_->publish(a);
 }
 std::unique_ptr<vx::Observer> observer_;double max_age_,lio_max_;std::string imu_frame_,world_frame_,body_frame_,output_,reason_="no_imu";
 std::optional<int64_t> last_clock_,imu_receipt_,lio_receipt_,published_;std::deque<double>cost_;std::map<std::string,uint64_t>count_;
 rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_;
 rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_;
 rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_,debug_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<VelocityNode>());}catch(const std::exception&e){RCLCPP_FATAL(rclcpp::get_logger("vx_observer"),"%s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}

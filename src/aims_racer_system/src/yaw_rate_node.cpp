// Independent gyro-led body-z rate/acceleration observer. No control consumers.
#include "aims_racer_system/yaw_rate.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
namespace yr=aims_racer_system::yaw_rate;
class YawRateNode final:public rclcpp::Node {
 public:
  YawRateNode():Node("yaw_rate") {
    yr::Config c;
    c.process_psd=parameter("process_psd",c.process_psd);
    c.measurement_variance=parameter("measurement_variance",c.measurement_variance);
    c.initial_alpha_variance=parameter("initial_alpha_variance",c.initial_alpha_variance);
    c.max_gap_s=parameter("max_gap_s",c.max_gap_s);
    auto mount=parameter("livox_quaternion",std::vector<double>{0.,0.,0.,1.});
    if(mount.size()!=4)throw std::invalid_argument("livox_quaternion requires xyzw");
    std::copy(mount.begin(),mount.end(),c.mount_xyzw.begin());
    observer_=std::make_unique<yr::Observer>(c);
    imu_frame_=parameter<std::string>("imu_frame","livox_frame");
    body_frame_=parameter<std::string>("body_frame","base_link");
    max_age_=parameter("imu_max_age_s",.05);
    if(!std::isfinite(max_age_)||max_age_<=0||max_age_>.5)throw std::invalid_argument("invalid imu_max_age_s");
    const auto output=parameter<std::string>("output_topic","/state_estimation/yaw_rate");
    rate_pub_=create_publisher<sensor_msgs::msg::Imu>(output,10);
    alpha_pub_=create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
        parameter<std::string>("acceleration_topic","/state_estimation/yaw_acceleration"),10);
    status_pub_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>(output+"/status",10);
    imu_sub_=create_subscription<sensor_msgs::msg::Imu>(
        parameter<std::string>("imu_topic","/livox/imu_bias_corrected"),rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr m){receive(*m);});
    for(const auto *k:{"callbacks","wrong_frame","unavailable","nonfinite","invalid_stamp","future_stamp","stale_stamp","clock_resets"})count_[k]=0;
    timer_=create_wall_timer(std::chrono::milliseconds(100),[this]{diagnostics();});
  }
 private:
  template<class T>T parameter(const std::string & name,const T & value) {
    rcl_interfaces::msg::ParameterDescriptor d;d.read_only=true;
    return declare_parameter<T>(name,value,d);
  }
  int64_t clock_ns() {
    const auto t=now().nanoseconds();
    if(last_clock_&&t<*last_clock_) {
      observer_->reset();last_receipt_.reset();published_.reset();reason_="clock_reset";++count_["clock_resets"];
    }
    last_clock_=t;return t;
  }
  bool good(int64_t t) const {
    auto e=observer_->latest();return e.valid&&reason_.empty()&&e.stamp_ns<=t&&(t-e.stamp_ns)*1e-9<=max_age_;
  }
  void reject(const std::string &why){++count_[why];reason_=why;}
  void receive(const sensor_msgs::msg::Imu &m) {
    const auto begin=std::chrono::steady_clock::now();
    receive_impl(m);
    if(cost_.size()==512)cost_.pop_front();
    cost_.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count());
  }
  void receive_impl(const sensor_msgs::msg::Imu &m) {
    const auto t=clock_ns();last_receipt_=t;++count_["callbacks"];
    if(m.header.frame_id!=imu_frame_){reject("wrong_frame");return;}
    if(m.angular_velocity_covariance[0]<0){reject("unavailable");return;}
    for(double v:m.angular_velocity_covariance)if(!std::isfinite(v)){reject("nonfinite");return;}
    for(double v:{m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z})
      if(!std::isfinite(v)){reject("nonfinite");return;}
    const auto &h=m.header.stamp;
    if(h.sec<0||h.nanosec>=1000000000u){reject("invalid_stamp");return;}
    const int64_t ns=static_cast<int64_t>(h.sec)*1000000000LL+h.nanosec;
    if(ns<=0){reject("invalid_stamp");return;}
    if(ns>t){reject("future_stamp");return;}
    if((t-ns)*1e-9>max_age_){reject("stale_stamp");return;}
    const auto before=observer_->counters();
    if(!observer_->add({ns,{m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z}})){
      auto c=observer_->counters();
      reason_=c.numerical_error>before.numerical_error?"numerical_error":
          (c.duplicate>before.duplicate?"duplicate":(c.out_of_order>before.out_of_order?"out_of_order":"invalid_input"));
      return;
    }
    reason_.clear();const auto e=observer_->latest();
    if(!good(t)||(published_&&e.stamp_ns<=*published_))return;
    sensor_msgs::msg::Imu rate;rate.header.stamp=m.header.stamp;rate.header.frame_id=body_frame_;
    rate.orientation_covariance[0]=-1.;rate.linear_acceleration_covariance[0]=-1.;
    rate.angular_velocity.z=e.rate;rate.angular_velocity_covariance[0]=1e6;
    rate.angular_velocity_covariance[4]=1e6;rate.angular_velocity_covariance[8]=e.rate_variance;
    geometry_msgs::msg::AccelWithCovarianceStamped alpha;alpha.header=rate.header;
    alpha.accel.accel.angular.z=e.alpha;
    for(unsigned k:{0u,7u,14u,21u,28u})alpha.accel.covariance[k]=1e6;
    alpha.accel.covariance[35]=e.alpha_variance;
    rate_pub_->publish(rate);alpha_pub_->publish(alpha);published_=e.stamp_ns;
  }
  template<class T>static void field(diagnostic_msgs::msg::DiagnosticStatus &s,const std::string &k,const T &value){
    diagnostic_msgs::msg::KeyValue v;v.key=k;v.value=std::to_string(value);s.values.push_back(v);
  }
  static void field(diagnostic_msgs::msg::DiagnosticStatus &s,const std::string &k,const std::string &value){
    diagnostic_msgs::msg::KeyValue v;v.key=k;v.value=value;s.values.push_back(v);
  }
  void diagnostics(){
    const auto t=clock_ns();auto e=observer_->latest();auto c=observer_->counters();
    diagnostic_msgs::msg::DiagnosticArray a;a.header.stamp=now();a.header.frame_id=body_frame_;
    diagnostic_msgs::msg::DiagnosticStatus s;s.name="yaw_rate";s.hardware_id="bias_corrected_livox_gyro";
    s.level=good(t)?s.OK:s.WARN;
    s.message=!reason_.empty()?reason_:(!e.stamp_ns?"no_imu":(!e.valid?"gyro_warmup":(good(t)?"valid":"imu_stale")));
    field(s,"valid",std::string(good(t)?"true":"false"));field(s,"covariance",std::string("conditional_model_uncalibrated"));
    field(s,"definition",std::string("body_omega_z_not_euler_yaw_derivative"));
    field(s,"online_bias_estimation",std::string("false"));field(s,"source_stamp_ns",e.stamp_ns);
    field(s,"imu_source_age_s",e.stamp_ns?(t-e.stamp_ns)*1e-9:-1.);
    field(s,"imu_receipt_age_s",last_receipt_?(t-*last_receipt_)*1e-9:-1.);
    field(s,"measured_rate",e.measured_rate);field(s,"rate",e.rate);field(s,"alpha",e.alpha);
    field(s,"rate_variance",e.rate_variance);field(s,"alpha_variance",e.alpha_variance);field(s,"cross_covariance",e.cross_covariance);
    field(s,"consecutive_samples",e.consecutive_samples);
    for(auto &kv:count_)field(s,kv.first,kv.second);
    field(s,"accepted",c.accepted);field(s,"duplicate",c.duplicate);field(s,"out_of_order",c.out_of_order);
    field(s,"invalid",c.invalid);field(s,"numerical_error",c.numerical_error);field(s,"gyro_gaps",c.gaps);
    if(!cost_.empty()){
      std::vector<double> x(cost_.begin(),cost_.end());std::sort(x.begin(),x.end());
      field(s,"callback_window_n",x.size());field(s,"callback_window_p99_ms",x[static_cast<std::size_t>(std::ceil(.99*x.size()))-1]);
      field(s,"callback_window_max_ms",x.back());
    }
    a.status.push_back(s);status_pub_->publish(a);
  }
  std::unique_ptr<yr::Observer> observer_;std::string imu_frame_,body_frame_,reason_="no_imu";double max_age_;
  std::map<std::string,uint64_t> count_;std::optional<int64_t> last_clock_,last_receipt_,published_;std::deque<double> cost_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr rate_pub_;
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr alpha_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<YawRateNode>());}
catch(const std::exception &e){RCLCPP_FATAL(rclcpp::get_logger("yaw_rate"),"%s",e.what());rclcpp::shutdown();return 1;}
rclcpp::shutdown();return 0;}

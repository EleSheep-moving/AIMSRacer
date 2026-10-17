// Diagnostic-only estimates from corrected raw IMU and physical-IMU LIO pose.
// No actuator, wheel, EKF or TF interface. Covariances are effective model values.
#include "aims_racer_system/velocity.hpp"
#include "aims_racer_system/body_acceleration.hpp"
#include "aims_racer_system/yaw_rate.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace vx=aims_racer_system::velocity;
namespace ba=aims_racer_system::body_acceleration;
namespace yr=aims_racer_system::yaw_rate;
using V3=Eigen::Vector3d;
using M3=Eigen::Matrix3d;
class StateObserverNode final : public rclcpp::Node {
 public:
  StateObserverNode():Node("state_observer") {
    imu_frame_=parameter<std::string>("imu_frame","livox_frame");
    world_frame_=parameter<std::string>("world_frame","odom");
    references_[0].frame=parameter<std::string>("rear_frame","base_link");
    references_[1].frame=parameter<std::string>("cg_frame","center_of_mass");
    for(const auto & frame:{imu_frame_,world_frame_,references_[0].frame,references_[1].frame})
      if(frame.empty())throw std::invalid_argument("frame names must not be empty");
    const V3 rear_to_imu=vector_parameter("livox_translation",{.30,0.,.03});
    references_[0].imu_to_point=-rear_to_imu;
    references_[1].imu_to_point=vector_parameter("imu_to_cg",{-.09,-.03,-.03});
    const auto quaternion=parameter("livox_quaternion",std::vector<double>{0.,0.,0.,1.});
    if(quaternion.size()!=4)throw std::invalid_argument("livox_quaternion requires xyzw");
    mounting_=Eigen::Quaterniond(quaternion[3],quaternion[0],quaternion[1],quaternion[2]);
    if(!mounting_.coeffs().allFinite()||std::abs(mounting_.norm()-1.)>1e-9)
      throw std::invalid_argument("livox_quaternion must be finite and unit length");
    mounting_.normalize();
    imu_max_age_=parameter("imu_max_age_s",.05);
    if(!std::isfinite(imu_max_age_)||imu_max_age_<=0.||imu_max_age_>.5)
      throw std::invalid_argument("imu_max_age_s must be within (0,.5]");
    cg_height_estimated_=parameter("cg_height_estimated",true);
    warn_only_quality_gates_=parameter("warn_only_quality_gates",true);
    geometry_calibrated_=parameter("geometry_calibrated",false);
    gyro_variance_=parameter("gyro_variance",1e-4);
    if(!std::isfinite(gyro_variance_)||gyro_variance_<=0.||gyro_variance_>1e12)
      throw std::invalid_argument("gyro_variance must be positive and finite");
    vx::Config velocity_config;velocity_config.lever=rear_to_imu;
    velocity_config.continue_on_lio_timeout=warn_only_quality_gates_;
    velocity_config.mounting=mounting_.toRotationMatrix();lio_max_age_=velocity_config.max_age;
    velocity_=std::make_unique<vx::Observer>(velocity_config);
    ba::Config acceleration_config;acceleration_config.mounting=mounting_;
    acceleration_config.lateral_enabled=true;acceleration_config.gyro_variance=gyro_variance_;
    acceleration_config.continue_on_lio_timeout=warn_only_quality_gates_;
    min_alpha_span_=parameter("min_alpha_span_s",.010);
    acceleration_config.min_alpha_span_s=min_alpha_span_;
    yr::Config yaw_config;yaw_config.measurement_variance=gyro_variance_;std::copy(quaternion.begin(),quaternion.end(),yaw_config.mount_xyzw.begin());
    yaw_=std::make_unique<yr::Observer>(yaw_config);
    for(std::size_t i=0;i<references_.size();++i) {
      auto & ref=references_[i];
      acceleration_config.lever=-ref.imu_to_point;
      ref.acceleration=std::make_unique<ba::Observer>(acceleration_config);
      const std::string prefix=std::string("/state_estimation/")+(i==0?"rear_axle":"center_of_mass");
      ref.velocity_pub=create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(prefix+"/velocity",100);
      ref.acceleration_pub=create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(prefix+"/acceleration",100);
      ref.yaw_pub=create_publisher<sensor_msgs::msg::Imu>(prefix+"/yaw_rate",100);
      ref.status_pub=create_publisher<diagnostic_msgs::msg::DiagnosticArray>(prefix+"/status",10);
    }
    imu_sub_=create_subscription<sensor_msgs::msg::Imu>(
      parameter<std::string>("imu_topic","/livox/imu_bias_corrected"),rclcpp::QoS(256).reliable(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m){receive(*m);});
    lio_sub_=create_subscription<nav_msgs::msg::Odometry>(
      parameter<std::string>("lio_topic","/fastlio2/lio_odom"),rclcpp::QoS(256).reliable(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m){receive(*m);});
    timer_=create_wall_timer(std::chrono::milliseconds(100),[this]{diagnostics();});
  }
 private:
  struct Reference {
    std::string frame;
    V3 imu_to_point=V3::Zero();
    std::unique_ptr<ba::Observer> acceleration;
    std::optional<int64_t> published_velocity,published_acceleration,published_yaw;
    std::string acceleration_error;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_pub;
    rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr acceleration_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr yaw_pub;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub;
  };
  template<class T>T parameter(const std::string & name,const T & value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;descriptor.read_only=true;
    return declare_parameter<T>(name,value,descriptor);
  }
  V3 vector_parameter(const std::string & name,const std::vector<double> & fallback) {
    const auto values=parameter(name,fallback);
    if(values.size()!=3)throw std::invalid_argument(name+" requires three values");
    const V3 result(values[0],values[1],values[2]);
    if(!result.allFinite()||!std::isfinite(result.squaredNorm()))throw std::invalid_argument(name+" must be finite");
    return result;
  }
  int64_t clock_ns() {
    const int64_t t=now().nanoseconds();
    if(last_clock_&&t<*last_clock_) {
      velocity_->reset();yaw_->reset();last_imu_.reset();last_lio_.reset();
      imu_receipt_.reset();lio_receipt_.reset();velocity_error_.clear();force_error_="no_imu";yaw_error_="no_imu";
      for(auto & ref:references_) {
        ref.acceleration->reset();ref.published_velocity.reset();ref.published_acceleration.reset();ref.published_yaw.reset();ref.acceleration_error.clear();
      }
      ++counts_["clock_resets"];
    }
    last_clock_=t;return t;
  }
  void reject(const std::string & reason){++counts_[reason];last_rejection_=reason;}
  std::optional<int64_t> stamp(const builtin_interfaces::msg::Time & source,int64_t t) {
    if(source.sec<0||source.nanosec>=1000000000u){reject("invalid_stamp");return {};}
    const int64_t ns=int64_t(source.sec)*1000000000LL+source.nanosec;
    if(ns<=0){reject("invalid_stamp");return {};}
    if(ns>t){reject("future_stamp");return {};}
    return ns;
  }
  static double age(int64_t t,int64_t source){return source>0?(t-source)*1e-9:-1.;}
  bool fresh(int64_t t,int64_t source,double limit)const{return source>0&&source<=t&&age(t,source)<=limit+1e-12;}
  template<std::size_t N>static bool covariance_ok(const std::array<double,N> & covariance,std::size_t dimension) {
    if(!std::all_of(covariance.begin(),covariance.end(),[](double v){return std::isfinite(v);}))return false;
    for(std::size_t i=0;i<dimension;++i)if(covariance[i*(dimension+1)]<0.)return false;
    return true;
  }
  static bool increasing(int64_t stamp,const std::optional<int64_t> & previous){return !previous||stamp>*previous;}
  M3 target_covariance(const vx::Estimate & e,const Reference & ref)const {
    const V3 & r=ref.imu_to_point;
    M3 covariance=e.velocity_covariance_body+gyro_variance_*(r.squaredNorm()*M3::Identity()-r*r.transpose());
    covariance=(.5*(covariance+covariance.transpose())).eval();
    for(int i=0;i<3;++i)covariance(i,i)=std::max(covariance(i,i),1e-4);
    return covariance;
  }
  bool velocity_good(int64_t t,const Reference & ref)const {
    const auto e=velocity_->latest();
    const V3 v=e.imu_velocity_body+e.omega_body.cross(ref.imu_to_point);
    return force_error_.empty()&&velocity_error_.empty()&&e.valid&&v.allFinite()&&
      target_covariance(e,ref).allFinite()&&(warn_only_quality_gates_||
        (fresh(t,e.stamp,imu_max_age_)&&fresh(t,e.anchor,lio_max_age_)));
  }
  bool acceleration_good(int64_t t,const Reference & ref)const {
    const auto e=ref.acceleration->latest();
    return force_error_.empty()&&ref.acceleration_error.empty()&&e.valid&&e.lateral_valid&&
      std::isfinite(e.ax)&&std::isfinite(e.ay)&&std::isfinite(e.acceleration_variance)&&
      std::isfinite(e.lateral_acceleration_variance)&&e.acceleration_variance>=0.&&e.lateral_acceleration_variance>=0.&&
      (warn_only_quality_gates_||(fresh(t,e.stamp_ns,imu_max_age_)&&fresh(t,e.anchor_stamp_ns,lio_max_age_)));
  }
  bool yaw_good(int64_t t)const {
    const auto e=yaw_->latest();return yaw_error_.empty()&&e.valid&&std::isfinite(e.rate)&&
      std::isfinite(e.rate_variance)&&e.rate_variance>=0.&&(warn_only_quality_gates_||fresh(t,e.stamp_ns,imu_max_age_));
  }
  void receive(const sensor_msgs::msg::Imu & message) {
    const int64_t t=clock_ns();++counts_["imu_callbacks"];
    const auto ns=stamp(message.header.stamp,t);if(!ns)return;
    if(message.header.frame_id!=imu_frame_){reject("imu_wrong_frame");return;}
    if(!fresh(t,*ns,imu_max_age_)) {
      if(!warn_only_quality_gates_){reject("imu_stale");return;}
      ++counts_["imu_age_warnings"];
    }
    if(last_imu_&&*ns<=*last_imu_){reject(*ns==*last_imu_?"imu_duplicate":"imu_out_of_order");return;}
    const V3 gyro(message.angular_velocity.x,message.angular_velocity.y,message.angular_velocity.z);
    if(!gyro.allFinite()||!covariance_ok(message.angular_velocity_covariance,3)) {
      yaw_error_="gyro_unavailable_or_nonfinite";force_error_=yaw_error_;reject(yaw_error_);return;
    }
    last_imu_=*ns;imu_receipt_=t;
    if(yaw_->add({*ns,{gyro.x(),gyro.y(),gyro.z()}}))yaw_error_.clear();
    else {yaw_error_="gyro_numerical_error";reject(yaw_error_);}
    const V3 force(message.linear_acceleration.x,message.linear_acceleration.y,message.linear_acceleration.z);
    if(!force.allFinite()||!covariance_ok(message.linear_acceleration_covariance,3)) {
      force_error_="force_unavailable_or_nonfinite";reject(force_error_);
    } else {
      force_error_.clear();
      if(velocity_->add_imu({*ns,force,gyro}))velocity_error_.clear();
      else {velocity_error_="velocity_numerical_error";reject(velocity_error_);}
      for(auto & ref:references_) {
        if(ref.acceleration->add_imu({*ns,force,gyro}))ref.acceleration_error.clear();
        else {ref.acceleration_error="acceleration_numerical_error";reject(ref.acceleration_error);}
      }
    }
    // Only a new current IMU may publish. Replayed LIO/IMU history never republishes.
    for(auto & ref:references_)publish(ref,*ns,t,message.header.stamp);
  }
  void receive(const nav_msgs::msg::Odometry & message) {
    const int64_t t=clock_ns();++counts_["lio_callbacks"];
    const auto ns=stamp(message.header.stamp,t);if(!ns)return;
    if(message.header.frame_id!=world_frame_||message.child_frame_id!=imu_frame_){reject("lio_wrong_frame");return;}
    if(!fresh(t,*ns,lio_max_age_)) {
      if(!warn_only_quality_gates_){reject("lio_stale");return;}
      ++counts_["lio_age_warnings"];
    }
    const auto & p=message.pose.pose.position;const auto & orientation=message.pose.pose.orientation;
    const V3 position(p.x,p.y,p.z);
    Eigen::Quaterniond q(orientation.w,orientation.x,orientation.y,orientation.z);
    if(!position.allFinite()||!q.coeffs().allFinite()||std::abs(q.norm()-1.)>1e-3||!covariance_ok(message.pose.covariance,6)) {
      reject("lio_unavailable_or_nonfinite");return;
    }
    q.normalize();
    const bool accepted=velocity_->add_lio({*ns,position,q});
    const Eigen::Quaterniond body_q=q*mounting_.conjugate();
    bool acceleration_accepted=false;
    for(auto & ref:references_)acceleration_accepted=ref.acceleration->add_lio({*ns,body_q})||acceleration_accepted;
    if(accepted||acceleration_accepted) {
      if(!last_lio_||*ns>*last_lio_)last_lio_=*ns;
      lio_receipt_=t;
    } else reject("lio_rejected_or_duplicate");
  }
  void publish(Reference & ref,int64_t source,int64_t t,const builtin_interfaces::msg::Time & stamp) {
    const auto v=velocity_->latest();const auto a=ref.acceleration->latest();const auto y=yaw_->latest();
    if(v.stamp==source&&velocity_good(t,ref)&&increasing(source,ref.published_velocity)) {
      geometry_msgs::msg::TwistWithCovarianceStamped out;out.header.stamp=stamp;out.header.frame_id=ref.frame;
      const V3 velocity=v.imu_velocity_body+v.omega_body.cross(ref.imu_to_point);const M3 covariance=target_covariance(v,ref);
      out.twist.twist.linear.x=velocity.x();out.twist.twist.linear.y=velocity.y();
      for(std::size_t i=0;i<6;++i)out.twist.covariance[i*7]=1e6;
      for(int i=0;i<2;++i)for(int j=0;j<2;++j)out.twist.covariance[i*6+j]=covariance(i,j);
      ref.velocity_pub->publish(out);ref.published_velocity=source;++counts_["velocity_published"];
    }
    if(a.stamp_ns==source&&acceleration_good(t,ref)&&increasing(source,ref.published_acceleration)) {
      geometry_msgs::msg::AccelWithCovarianceStamped out;out.header.stamp=stamp;out.header.frame_id=ref.frame;
      out.accel.accel.linear.x=a.ax;out.accel.accel.linear.y=a.ay;
      for(std::size_t i=0;i<6;++i)out.accel.covariance[i*7]=1e6;
      out.accel.covariance[0]=a.acceleration_variance;out.accel.covariance[7]=a.lateral_acceleration_variance;
      ref.acceleration_pub->publish(out);ref.published_acceleration=source;++counts_["acceleration_published"];
    }
    if(y.stamp_ns==source&&yaw_good(t)&&increasing(source,ref.published_yaw)) {
      sensor_msgs::msg::Imu out;out.header.stamp=stamp;out.header.frame_id=ref.frame;
      out.orientation_covariance[0]=-1.;out.linear_acceleration_covariance[0]=-1.;
      out.angular_velocity.z=y.rate;out.angular_velocity_covariance[0]=1e6;out.angular_velocity_covariance[4]=1e6;
      out.angular_velocity_covariance[8]=y.rate_variance;
      ref.yaw_pub->publish(out);ref.published_yaw=source;++counts_["yaw_published"];
    }
  }
  template<class T>static void field(diagnostic_msgs::msg::DiagnosticStatus & status,const std::string & key,const T & value) {
    diagnostic_msgs::msg::KeyValue entry;entry.key=key;std::ostringstream stream;stream<<std::setprecision(17)<<value;entry.value=stream.str();status.values.push_back(entry);
  }
  std::string velocity_reason(int64_t t,const Reference & ref)const {
    const auto e=velocity_->latest();
    if(!force_error_.empty())return force_error_;
    if(!velocity_error_.empty())return velocity_error_;
    if(!e.valid)return e.status;
    if(!fresh(t,e.stamp,imu_max_age_))return "imu_stale";
    if(!fresh(t,e.anchor,lio_max_age_))return "lio_stale";
    return velocity_good(t,ref)?e.status:"numerical_error";
  }
  std::string acceleration_reason(int64_t t,const Reference & ref)const {
    const auto e=ref.acceleration->latest();
    if(!force_error_.empty())return force_error_;
    if(!ref.acceleration_error.empty())return ref.acceleration_error;
    if(!e.valid)return ba::status_name(e.status);
    if(!fresh(t,e.stamp_ns,imu_max_age_))return "imu_stale";
    if(!fresh(t,e.anchor_stamp_ns,lio_max_age_))return "lio_stale";
    return acceleration_good(t,ref)?ba::status_name(e.status):"numerical_error";
  }
  std::string yaw_reason(int64_t t)const {
    const auto e=yaw_->latest();if(!yaw_error_.empty())return yaw_error_;
    if(!e.valid)return e.stamp_ns?"gyro_warmup":"no_imu";
    return fresh(t,e.stamp_ns,imu_max_age_)?"valid":"imu_stale";
  }
  void diagnostics() {
    const int64_t t=clock_ns();const auto v=velocity_->latest();const auto y=yaw_->latest();
    for(const auto & ref:references_) {
      const auto a=ref.acceleration->latest();
      // Publication availability is separate from freshness and quality.
      const bool vp=velocity_good(t,ref),ap=acceleration_good(t,ref),yp=yaw_good(t);
      const bool vg=vp&&fresh(t,v.stamp,imu_max_age_),ag=ap&&fresh(t,a.stamp_ns,imu_max_age_),yg=yp&&fresh(t,y.stamp_ns,imu_max_age_);
      const bool quality_warning=velocity_reason(t,ref)!="valid"||acceleration_reason(t,ref)!="valid"||yaw_reason(t)!="valid";
      diagnostic_msgs::msg::DiagnosticArray out;out.header.stamp=rclcpp::Time(t);out.header.frame_id=ref.frame;
      diagnostic_msgs::msg::DiagnosticStatus status;status.name=std::string(get_name())+"/"+ref.frame;
      status.hardware_id="corrected_raw_imu_raw_lio_diagnostic";status.level=vg&&ag&&yg&&!quality_warning?status.OK:status.WARN;
      status.message=vg&&ag&&yg?(quality_warning?"degraded_continuing":"valid"):"partial_or_invalid";
      for(const auto & key:{"vx_valid","vy_valid"})field(status,key,vg?"true":"false");
      for(const auto & key:{"ax_valid","ay_valid"})field(status,key,ag?"true":"false");
      field(status,"yaw_rate_valid",yg?"true":"false");
      field(status,"warn_only_quality_gates",warn_only_quality_gates_?"true":"false");
      field(status,"quality_degraded",quality_warning?"true":"false");
      field(status,"velocity_computable",vp?"true":"false");field(status,"acceleration_computable",ap?"true":"false");field(status,"yaw_rate_computable",yp?"true":"false");
      field(status,"velocity_reason",velocity_reason(t,ref));field(status,"acceleration_reason",acceleration_reason(t,ref));field(status,"yaw_rate_reason",yaw_reason(t));
      field(status,"velocity_source_stamp_ns",v.stamp);field(status,"acceleration_source_stamp_ns",a.stamp_ns);field(status,"yaw_rate_source_stamp_ns",y.stamp_ns);
      field(status,"velocity_source_age_s",age(t,v.stamp));field(status,"acceleration_source_age_s",age(t,a.stamp_ns));field(status,"yaw_rate_source_age_s",age(t,y.stamp_ns));
      field(status,"imu_source_age_s",age(t,last_imu_.value_or(0)));field(status,"lio_source_age_s",age(t,last_lio_.value_or(0)));
      field(status,"imu_receipt_age_s",age(t,imu_receipt_.value_or(0)));field(status,"lio_receipt_age_s",age(t,lio_receipt_.value_or(0)));
      field(status,"velocity_lio_anchor_stamp_ns",v.anchor);field(status,"acceleration_lio_anchor_stamp_ns",a.anchor_stamp_ns);
      field(status,"velocity_lio_anchor_age_s",age(t,v.anchor));field(status,"acceleration_lio_anchor_age_s",age(t,a.anchor_stamp_ns));
      field(status,"alpha_window_span_s",a.alpha_window_span_s);field(status,"alpha_sample_center_age_s",a.alpha_sample_center_age_s);field(status,"min_alpha_span_s",min_alpha_span_);
      field(status,"cg_height_estimated",cg_height_estimated_?"true":"false");field(status,"geometry_calibrated",geometry_calibrated_?"true":"false");field(status,"dynamic_accuracy_validated","false");
      field(status,"covariance","conditional_effective_uncalibrated_shared_LIO_IMU");
      field(status,"acceleration_xy_cross_covariance","unidentified_diagonal_effective_model");field(status,"yaw_definition","body_omega_z_not_euler_yaw_derivative");
      field(status,"last_rejection",last_rejection_);field(status,"velocity_history_size",velocity_->history_size());field(status,"velocity_replayed_imus",velocity_->replayed_imus());
      field(status,"velocity_rejected_lio_innovations",v.rejected);field(status,"velocity_reacquisitions",v.reacquisitions);
      field(status,"acceleration_history_size",ref.acceleration->history_size());field(status,"acceleration_replayed_imus",ref.acceleration->counters().replayed_imu);
      for(const auto & count:counts_)field(status,"input."+count.first,count.second);
      out.status.push_back(status);ref.status_pub->publish(out);
      if(status.level==status.WARN)RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,
        "%s: velocity=%s acceleration=%s yaw_rate=%s; numeric publication follows computability, source stamps preserved",
        ref.frame.c_str(),velocity_reason(t,ref).c_str(),acceleration_reason(t,ref).c_str(),yaw_reason(t).c_str());
    }
  }
  std::array<Reference,2> references_;
  std::unique_ptr<vx::Observer> velocity_;std::unique_ptr<yr::Observer> yaw_;
  Eigen::Quaterniond mounting_=Eigen::Quaterniond::Identity();
  std::string imu_frame_,world_frame_,force_error_="no_imu",yaw_error_="no_imu",velocity_error_,last_rejection_;
  double imu_max_age_,lio_max_age_,min_alpha_span_,gyro_variance_;
  bool cg_height_estimated_,geometry_calibrated_,warn_only_quality_gates_;
  std::optional<int64_t> last_clock_,last_imu_,last_lio_,imu_receipt_,lio_receipt_;
  std::map<std::string,uint64_t> counts_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<StateObserverNode>());}
  catch(const std::exception & error) {RCLCPP_FATAL(rclcpp::get_logger("state_observer"),"%s",error.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}

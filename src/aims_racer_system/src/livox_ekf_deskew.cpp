#include "aims_racer_system/deskew.hpp"
#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace aims_racer_system {
using Cloud = livox_ros_driver2::msg::CustomMsg;
using Steady = std::chrono::steady_clock;
class LivoxEkfDeskew : public rclcpp::Node {
public:
  LivoxEkfDeskew() : Node("livox_ekf_deskew") {
    const auto t = declare_parameter<std::vector<double>>("livox_translation", {.3, 0., .03});
    const auto q = declare_parameter<std::vector<double>>("livox_quaternion", {0.,0.,0.,1.});
    const double max_gap=declare_parameter("max_odometry_gap_sec",.02);
    if(!std::isfinite(max_gap) || max_gap<=0.) throw std::invalid_argument("Invalid max_odometry_gap_sec");
    max_gap_ns_=static_cast<int64_t>(max_gap*1e9);
    if (t.size() != 3 || q.size() != 4) throw std::invalid_argument("Invalid mount dimensions");
    Eigen::Quaterniond rotation(q[3],q[0],q[1],q[2]);
    if (!rotation.coeffs().allFinite() || std::abs(rotation.norm()-1.)>.01) throw std::invalid_argument("Invalid mount quaternion");
    mount_ = Eigen::Isometry3d::Identity();
    mount_.translation() = Eigen::Vector3d(t[0],t[1],t[2]);
    if (!mount_.translation().allFinite()) throw std::invalid_argument("Invalid mount translation");
    mount_.linear() = rotation.normalized().toRotationMatrix();
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/localization/deskewed_cloud", rclcpp::SensorDataQoS().keep_last(1));
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/localization/deskew_status",10);
    cloud_sub_ = create_subscription<Cloud>("/livox/lidar",rclcpp::SensorDataQoS().keep_last(2),
      [this](Cloud::ConstSharedPtr cloud) {
        std::lock_guard<std::mutex> lock(mutex_);
        const int64_t stamp = rclcpp::Time(cloud->header.stamp).nanoseconds();
        if (last_cloud_ >= 0 && stamp < last_cloud_) reset_locked();
        last_cloud_ = stamp;
        ++received_;
        if (pending_.size() == 2) {pending_.pop_front(); ++queue_dropped_;}
        pending_.push_back({cloud,Steady::now(),generation_});
        cv_.notify_one();
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered",rclcpp::QoS(500),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        if (msg->header.frame_id!="odom" || msg->child_frame_id!="base_link") return;
        const auto &p=msg->pose.pose.position; const auto &q=msg->pose.pose.orientation;
        std::lock_guard<std::mutex> lock(mutex_);
        const int64_t stamp=rclcpp::Time(msg->header.stamp).nanoseconds();
        if (history_.latest_stamp()>=0 && stamp < history_.latest_stamp()) reset_locked();
        history_.add({stamp,{p.x,p.y,p.z},Eigen::Quaterniond(q.w,q.x,q.y,q.z)});
        cv_.notify_one();
      });
    timer_=create_wall_timer(std::chrono::milliseconds(100),[this] {diagnostics();});
    worker_=std::thread([this] {work();});
  }
  ~LivoxEkfDeskew() override {
    {std::lock_guard<std::mutex> lock(mutex_); stopping_=true;}
    cv_.notify_all(); if(worker_.joinable()) worker_.join();
  }
private:
  struct Pending {Cloud::ConstSharedPtr cloud; Steady::time_point received; uint64_t generation;};
  void reset_locked() {
    history_.clear(); pending_.clear(); ++generation_; ++resets_;
    last_cloud_=-1; last_output_=-1;
  }
  void work() {
    while(true) {
      Pending scan;
      PoseHistory history;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock,[this]{return stopping_ || !pending_.empty();});
        if(stopping_) return;
        scan=pending_.front(); pending_.pop_front();
        active_=true;
        active_started_=Steady::now();
      }
      const auto started=Steady::now();
      int64_t begin=rclcpp::Time(scan.cloud->header.stamp).nanoseconds(), end=begin;
      bool valid=scan.cloud->header.frame_id=="livox_frame" && !scan.cloud->points.empty() &&
        scan.cloud->point_num == scan.cloud->points.size();
      // Livox header is scan start; offset_time is nanoseconds, not seconds.
      for(const auto &p:scan.cloud->points) {
        end=std::max(end,begin+static_cast<int64_t>(p.offset_time));
        valid=valid && std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
      }
      valid=valid && end-begin<=200000000LL;
      bool covered=false;
      if(valid) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_until(lock,scan.received+std::chrono::milliseconds(100),[&] {
          return stopping_ || generation_!=scan.generation ||
            history_.covers(begin,end,max_gap_ns_);
        });
        if(stopping_) return;
        covered=generation_==scan.generation && history_.covers(begin,end,max_gap_ns_);
        if(covered) history=history_;
      }
      if(!valid || !covered) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(!valid) ++invalid_dropped_; else ++coverage_dropped_;
        active_=false; continue;
      }
      sensor_msgs::msg::PointCloud2 output;
      output.header=scan.cloud->header; output.header.stamp=rclcpp::Time(end);
      sensor_msgs::PointCloud2Modifier modifier(output);
      modifier.setPointCloud2Fields(4,"x",1,sensor_msgs::msg::PointField::FLOAT32,
        "y",1,sensor_msgs::msg::PointField::FLOAT32,"z",1,sensor_msgs::msg::PointField::FLOAT32,
        "intensity",1,sensor_msgs::msg::PointField::FLOAT32);
      modifier.resize(scan.cloud->points.size()); output.is_dense=true;
      sensor_msgs::PointCloud2Iterator<float> x(output,"x"),y(output,"y"),z(output,"z"),intensity(output,"intensity");
      const auto target=*history.interpolate(end,max_gap_ns_);
      const auto target_inverse=(pose_matrix(target)*mount_).inverse();
      for(const auto &p:scan.cloud->points) {
        const auto pose=history.interpolate(begin+p.offset_time,max_gap_ns_);
        if(!pose) {valid=false;break;}
        const Eigen::Vector3d corrected=target_inverse*(pose_matrix(*pose)*mount_)*Eigen::Vector3d(p.x,p.y,p.z);
        *x=corrected.x(); *y=corrected.y(); *z=corrected.z(); *intensity=p.reflectivity;
        ++x; ++y; ++z; ++intensity;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if(valid && generation_==scan.generation) {
          cloud_pub_->publish(output); ++published_; last_output_=end;
          queue_age_ms_=std::chrono::duration<double,std::milli>(started-scan.received).count();
          stage_ms_=std::chrono::duration<double,std::milli>(Steady::now()-started).count();
        } else if(!valid) ++coverage_dropped_;
        active_=false;
      }
    }
  }
  void diagnostics() {
    const auto now_stamp=now();
    diagnostic_msgs::msg::DiagnosticArray array; array.header.stamp=now_stamp;
    diagnostic_msgs::msg::DiagnosticStatus status; status.name="aims_racer_system/deskew";
    std::lock_guard<std::mutex> lock(mutex_);
    if(last_clock_>=0 && now_stamp.nanoseconds()<last_clock_) reset_locked();
    last_clock_=now_stamp.nanoseconds();
    const double age=last_output_<0 ? -1. : (now_stamp.nanoseconds()-last_output_)*1e-9;
    status.level=last_output_<0 || age>.5 ? 1 : 0;
    status.message=last_output_<0 ? "waiting_for_full_coverage" : (age>.5 ? "stale" : "deskewed");
    const auto add=[&](const std::string &key,auto value) {
      diagnostic_msgs::msg::KeyValue kv; kv.key=key;kv.value=std::to_string(value);status.values.push_back(kv);
    };
    add("received",received_); add("published",published_); add("queue_dropped",queue_dropped_);
    add("coverage_dropped",coverage_dropped_); add("invalid_dropped",invalid_dropped_); add("resets",resets_);
    add("pending",pending_.size());add("processing",active_);add("history_samples",history_.size());
    add("queue_age_ms",queue_age_ms_);add("stage_ms",stage_ms_);add("source_age_sec",age);
    add("oldest_pending_age_ms",pending_.empty() ? 0. :
      std::chrono::duration<double,std::milli>(Steady::now()-pending_.front().received).count());
    add("active_stage_age_ms",active_ ?
      std::chrono::duration<double,std::milli>(Steady::now()-active_started_).count() : 0.);
    array.status.push_back(status);diagnostics_pub_->publish(array);
  }
  Eigen::Isometry3d mount_;
  PoseHistory history_;
  std::mutex mutex_; std::condition_variable cv_; std::thread worker_;
  std::deque<Pending> pending_; bool stopping_=false,active_=false;
  uint64_t generation_=0,received_=0,published_=0,queue_dropped_=0,coverage_dropped_=0,invalid_dropped_=0,resets_=0;
  int64_t last_cloud_=-1,last_clock_=-1,last_output_=-1,max_gap_ns_=20000000LL;
  double queue_age_ms_=0.,stage_ms_=0.;
  Steady::time_point active_started_;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<aims_racer_system::LivoxEkfDeskew>());}
  catch(const std::exception &e) {RCLCPP_FATAL(rclcpp::get_logger("deskew"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}

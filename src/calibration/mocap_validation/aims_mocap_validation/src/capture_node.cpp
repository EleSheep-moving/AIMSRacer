// Passive timing observer: never publishes vehicle commands or transforms.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <vesc_msgs/msg/vesc_command.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <openssl/sha.h>
#include <atomic>
#include <cmath>
#include <iostream>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>

using I64=int64_t;
using Serialized=rclcpp::SerializedMessage;
using Info=rclcpp::MessageInfo;
static I64 system_ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();}
static I64 steady_ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
template<class T> static std::optional<I64> header_ns(const Serialized& serialized){
  T message;rclcpp::Serialization<T> codec;codec.deserialize_message(&serialized,&message);
  if(message.header.stamp.sec<0||message.header.stamp.nanosec>=1000000000u)return {};
  const I64 stamp=I64(message.header.stamp.sec)*1000000000+message.header.stamp.nanosec;
  return stamp>0?std::optional<I64>(stamp):std::nullopt;
}
static std::optional<I64> source_ns(const Serialized& s,const std::string& type){
#define SOURCE(name,klass) if(type==name)return header_ns<klass>(s)
  SOURCE("sensor_msgs/msg/Imu",sensor_msgs::msg::Imu);
  SOURCE("nav_msgs/msg/Odometry",nav_msgs::msg::Odometry);
  SOURCE("geometry_msgs/msg/PoseStamped",geometry_msgs::msg::PoseStamped);
  SOURCE("geometry_msgs/msg/PoseWithCovarianceStamped",geometry_msgs::msg::PoseWithCovarianceStamped);
  SOURCE("geometry_msgs/msg/TransformStamped",geometry_msgs::msg::TransformStamped);
  SOURCE("geometry_msgs/msg/TwistWithCovarianceStamped",geometry_msgs::msg::TwistWithCovarianceStamped);
  SOURCE("geometry_msgs/msg/AccelWithCovarianceStamped",geometry_msgs::msg::AccelWithCovarianceStamped);
  SOURCE("diagnostic_msgs/msg/DiagnosticArray",diagnostic_msgs::msg::DiagnosticArray);
  SOURCE("vesc_msgs/msg/VescCommand",vesc_msgs::msg::VescCommand);
  SOURCE("vesc_msgs/msg/VescStateStamped",vesc_msgs::msg::VescStateStamped);
  SOURCE("ackermann_msgs/msg/AckermannDriveStamped",ackermann_msgs::msg::AckermannDriveStamped);
#undef SOURCE
  return {};
}
// Humble's GenericSubscription callback omits MessageInfo. Preserve the
// executor-provided metadata by overriding its public dispatch hooks.
class TimedSubscription:public rclcpp::GenericSubscription {
 public:
  using Callback=std::function<void(std::shared_ptr<Serialized>,const Info&)>;
  TimedSubscription(rclcpp::Node& node,const std::string& topic,const std::string& type,Callback callback)
  :GenericSubscription(node.get_node_base_interface().get(),rclcpp::get_typesupport_library(type,"rosidl_typesupport_cpp"),
    topic,type,rclcpp::SensorDataQoS().keep_last(200),[](std::shared_ptr<Serialized>){},rclcpp::SubscriptionOptions{}),callback_(std::move(callback)){}
  void handle_message(std::shared_ptr<void>& message,const Info& info)override{callback_(std::static_pointer_cast<Serialized>(message),info);}
  void handle_serialized_message(const std::shared_ptr<Serialized>& message,const Info& info)override{callback_(message,info);}
 private:Callback callback_;
};
struct Row{uint64_t sequence;std::string topic,type,digest;std::optional<I64> source;I64 pub,dds_receive,ros,system,steady;};
struct Stats{uint64_t count{},source_duplicates{},source_backwards{},source_gaps{},receive_gaps{};I64 last_source{},last_steady{};};
class Capture:public rclcpp::Node{
 public:
  Capture():Node("mocap_capture"){
    path_=declare_parameter<std::string>("output_file","");
    auto topics=declare_parameter<std::vector<std::string>>("topics",std::vector<std::string>{});
    auto types=declare_parameter<std::vector<std::string>>("types",std::vector<std::string>{});
    auto gaps=declare_parameter<std::vector<double>>("max_gaps_s",std::vector<double>(topics.size(),.1));
    const auto capacity=declare_parameter<int>("queue_capacity",4096);
    if(path_.empty()||topics.empty()||topics.size()!=types.size()||gaps.size()!=topics.size()||capacity<=0)throw std::invalid_argument("output/topics/types/gaps/capacity required");
    capacity_=static_cast<size_t>(capacity);
    if(std::filesystem::exists(path_))throw std::invalid_argument("timing output already exists");
    output_.exceptions(std::ios::failbit|std::ios::badbit);output_.open(path_);
    output_<<"sequence,topic,type,source_ns,dds_pub_ns,dds_receive_ns,receive_ros_ns,receive_system_ns,receive_steady_ns,cdr_sha256\n";
    for(size_t i=0;i<topics.size();++i){
      if(!std::isfinite(gaps[i])||gaps[i]<=0||stats_.count(topics[i]))throw std::invalid_argument("invalid gap or duplicate topic");
      stats_[topics[i]]=Stats{};gaps_[topics[i]]=static_cast<I64>(gaps[i]*1e9);
      auto subscription=std::make_shared<TimedSubscription>(*this,topics[i],types[i],
        [this,topic=topics[i],type=types[i]](std::shared_ptr<Serialized> s,const Info& info){receive(topic,type,*s,info);});
      get_node_topics_interface()->add_subscription(subscription,nullptr);subscriptions_.push_back(subscription);
    }
    publisher_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/mocap_validation/capture_status",10);
    timer_=create_wall_timer(std::chrono::seconds(1),[this]{publish_health();});
    writer_=std::thread([this]{write_loop();});
  }
  ~Capture()override{finish();}
  bool finish(){
    if(finalized_)return !writer_failed_&&!health_failed_;
    finalized_=true;
    {std::lock_guard<std::mutex> lock(mutex_);stopping_=true;}condition_.notify_all();
    if(writer_.joinable())writer_.join();
    try{output_.flush();}catch(const std::exception& e){writer_failed_=true;RCLCPP_ERROR(get_logger(),"Final timing flush failed: %s",e.what());}
    try{write_health();}catch(const std::exception& e){health_failed_=true;RCLCPP_ERROR(get_logger(),"Health write failed: %s",e.what());}
    return !writer_failed_&&!health_failed_;
  }
 private:
  void receive(const std::string& topic,const std::string& type,const Serialized& serialized,const Info& info){
    Row r{};r.topic=topic;r.type=type;r.ros=now().nanoseconds();r.system=system_ns();r.steady=steady_ns();
    const auto& rmw=info.get_rmw_message_info();r.pub=rmw.source_timestamp;r.dds_receive=rmw.received_timestamp;
    try{r.source=source_ns(serialized,type);}catch(const std::exception&){++decode_errors_;}
    unsigned char digest[SHA256_DIGEST_LENGTH];const auto& raw=serialized.get_rcl_serialized_message();SHA256(raw.buffer,raw.buffer_length,digest);
    std::ostringstream hex;hex<<std::hex<<std::setfill('0');for(auto c:digest)hex<<std::setw(2)<<unsigned(c);r.digest=hex.str();
    {std::lock_guard<std::mutex> lock(mutex_);r.sequence=++sequence_;auto& stats=stats_.at(topic);++stats.count;
      if(stats.last_steady&&r.steady-stats.last_steady>gaps_[topic]){++stats.receive_gaps;}
      stats.last_steady=r.steady;
      if(r.source){if(stats.last_source){if(*r.source==stats.last_source)++stats.source_duplicates;else if(*r.source<stats.last_source)++stats.source_backwards;else if(*r.source-stats.last_source>gaps_[topic])++stats.source_gaps;}stats.last_source=*r.source;}
      if(last_system_&&std::abs((r.system-last_system_)-(r.steady-last_steady_))>100000000)++clock_jumps_;
      if(last_ros_&&std::abs((r.ros-last_ros_)-(r.steady-last_steady_))>100000000)++ros_clock_jumps_;
      last_system_=r.system;last_ros_=r.ros;last_steady_=r.steady;
      if(queue_.size()>=capacity_){++drops_;return;}queue_.push_back(std::move(r));
    }condition_.notify_one();
  }
  void write_loop(){
    try{for(;;){Row row;
      {std::unique_lock<std::mutex> lock(mutex_);condition_.wait(lock,[this]{return stopping_||!queue_.empty();});if(queue_.empty()&&stopping_)break;row=std::move(queue_.front());queue_.pop_front();}
      output_<<row.sequence<<','<<row.topic<<','<<row.type<<',';if(row.source)output_<<*row.source;output_<<',';
      if(row.pub>0){output_<<row.pub;}
      output_<<',';if(row.dds_receive>0){output_<<row.dds_receive;}
      output_<<','<<row.ros<<','<<row.system<<','<<row.steady<<','<<row.digest<<'\n';
    }output_.flush();}catch(const std::exception& e){writer_failed_=true;RCLCPP_ERROR(get_logger(),"Timing writer failed: %s",e.what());rclcpp::shutdown();}
  }
  void publish_health(){
    diagnostic_msgs::msg::DiagnosticArray array;array.header.stamp=now();diagnostic_msgs::msg::DiagnosticStatus s;s.name="mocap_capture";
    s.level=drops_||writer_failed_?1:0;s.message=writer_failed_?"writer_failed":drops_?"queue_drops":"recording";
    for(const auto& item:std::vector<std::pair<std::string,uint64_t>>{{"messages",sequence_},{"queue_drops",drops_},{"decode_errors",decode_errors_},{"system_clock_jumps",clock_jumps_},{"ros_clock_jumps",ros_clock_jumps_}}){diagnostic_msgs::msg::KeyValue kv;kv.key=item.first;kv.value=std::to_string(item.second);s.values.push_back(kv);}array.status.push_back(s);publisher_->publish(array);
  }
  void write_health(){
    std::filesystem::path health(path_);health.replace_extension(".health.json");std::ofstream stream;stream.exceptions(std::ios::failbit|std::ios::badbit);stream.open(health);
    stream<<"{\"messages\":"<<sequence_<<",\"queue_drops\":"<<drops_<<",\"decode_errors\":"<<decode_errors_<<",\"system_clock_jumps\":"<<clock_jumps_<<",\"ros_clock_jumps\":"<<ros_clock_jumps_<<",\"writer_failed\":"<<(writer_failed_?"true":"false")<<",\"topics\":{";
    bool first=true;for(const auto& item:stats_){if(!first)stream<<',';first=false;const auto& s=item.second;stream<<std::quoted(item.first)<<":{\"count\":"<<s.count<<",\"source_duplicates\":"<<s.source_duplicates<<",\"source_backwards\":"<<s.source_backwards<<",\"source_gaps\":"<<s.source_gaps<<",\"receive_gaps\":"<<s.receive_gaps<<'}';}stream<<"}}\n";stream.flush();
  }
  bool finalized_{},health_failed_{};std::string path_;std::ofstream output_;size_t capacity_{};std::mutex mutex_;std::condition_variable condition_;std::deque<Row> queue_;bool stopping_{};std::thread writer_;
  std::map<std::string,Stats> stats_;std::map<std::string,I64> gaps_;I64 last_system_{},last_ros_{},last_steady_{};
  std::atomic<uint64_t> sequence_{},drops_{},decode_errors_{},clock_jumps_{},ros_clock_jumps_{};std::atomic<bool> writer_failed_{};
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{auto node=std::make_shared<Capture>();rclcpp::spin(node);const bool ok=node->finish();rclcpp::shutdown();return ok?0:1;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}}

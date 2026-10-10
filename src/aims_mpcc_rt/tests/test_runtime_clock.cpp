// Compile the actual callback with a test-only entry marker. The production
// binary has neither the marker nor access to its private state.
#define AIMS_MPCC_RT_TEST_ACCESS
#define main runtime_entry_unused
#include "../src/node.cpp"
#undef main

namespace aims_mpcc_rt {
struct RuntimeClockProbe {
  static void run(RuntimeNode& node) {
    std::unique_lock<std::mutex> lock(node.mutex_);
    const double epoch=steady();
    node.snapshot_.present=true;node.snapshot_.source=epoch-.05;node.snapshot_.received=epoch;
    node.mode_=true;node.mode_received_=epoch;
    node.history_->clear();node.history_->record(epoch,0.,0.,0.,0.);
    if(node.fresh_locked(epoch))throw std::runtime_error("history beginning after measurement incorrectly authorized state");

    std::thread publisher([&]{node.publish_command();});
    const auto deadline=Steady::now()+std::chrono::seconds(2);
    while(!node.test_callback_entered_.load()&&Steady::now()<deadline)std::this_thread::yield();
    if(!node.test_callback_entered_.load()){
      lock.unlock();publisher.join();throw std::runtime_error("publisher did not enter callback");
    }
    // Model a receive/enable callback completing while output waits for mutex.
    // Its timestamps and sampler reset are newer than callback entry, but
    // older than the valid decision epoch after acquiring this mutex.
    const double received=steady();const auto ref=node.bundle_.reference().at(0.);
    node.snapshot_={{ref.x,ref.y,ref.yaw,0.,0.,0.},{},{},received,0.,received,true};
    node.history_->clear();node.history_->record(received,0.,0.,0.,0.);
    node.mode_=true;node.mode_received_=received;node.started_=received;
    node.sampler_->reset(0.,0.,received);node.enabled_=true;node.phase_="RUNNING";
    lock.unlock();publisher.join();lock.lock();
    if(node.phase_!="RUNNING"||!node.enabled_)
      throw std::runtime_error("fresh receive interleaving incorrectly faulted: "+node.reason_);
    if(node.last_output_.expired&&steady()-node.started_>=node.ttl_)
      throw std::runtime_error("clock test exceeded source TTL");

    // An odometry callback waits while another callback observes a newer ROS
    // time. The source stamp stays valid; callback entry time must not be
    // mistaken for a backwards clock after acquiring the shared mutex.
    nav_msgs::msg::Odometry message;
    message.header.frame_id="odom";message.child_frame_id="base_link";
    message.header.stamp=node.get_clock()->now();
    message.pose.pose.position.x=ref.x+node.cfg_.rear_offset*std::cos(ref.yaw);
    message.pose.pose.position.y=ref.y+node.cfg_.rear_offset*std::sin(ref.yaw);
    message.pose.pose.orientation.z=std::sin(ref.yaw/2.);
    message.pose.pose.orientation.w=std::cos(ref.yaw/2.);
    std::thread receiver([&]{node.odometry(message);});
    const auto receiver_deadline=Steady::now()+std::chrono::seconds(2);
    while(!node.test_odometry_entered_.load()&&Steady::now()<receiver_deadline)std::this_thread::yield();
    if(!node.test_odometry_entered_.load()){
      lock.unlock();receiver.join();throw std::runtime_error("odometry did not enter callback");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    node.observe_clock_locked(node.get_clock()->now().seconds(),steady());
    lock.unlock();receiver.join();lock.lock();
    if(!node.enabled_||node.phase_!="RUNNING"||!node.snapshot_.present)
      throw std::runtime_error("queued odometry incorrectly reset ROS clock: "+node.reason_);
    const double source=message.header.stamp.sec+message.header.stamp.nanosec*1e-9;
    if(node.snapshot_.ros_source!=source)
      throw std::runtime_error("queued odometry was not accepted");

    // Real rollback protection, including invalidation, remains unchanged.
    const double ros=node.get_clock()->now().seconds();
    node.observe_clock_locked(ros,steady());
    node.observe_clock_locked(ros-.002,steady());
    if(node.enabled_||node.phase_!="FAULT"||node.snapshot_.present||node.mode_||!node.identity_.empty())
      throw std::runtime_error("genuine ROS clock rollback did not invalidate control");
    std::cout<<"clock ordering regressions and genuine rollback protection passed\n";
  }
};
}
int main(int argc,char** argv) {
  if(argc!=2)throw std::invalid_argument("synthetic bundle argument required");
  std::vector<std::string> args{"clock_test","--ros-args","-p",std::string("artifact_directory:=")+argv[1],
    "-p","simulation:=true","-p","repeat_laps:=true"};
  for(const std::string topic:{"/drive","/mpcc/status","/mpcc/reference","/mpcc/prediction","/mpcc/enable"})
    args.insert(args.end(),{"-r",topic+":=/mpcc_rt_test"+topic});
  std::vector<char*> raw;for(auto& s:args)raw.push_back(s.data());
  rclcpp::init(raw.size(),raw.data());
  int result=0;
  try{auto node=std::make_shared<aims_mpcc_rt::RuntimeNode>();aims_mpcc_rt::RuntimeClockProbe::run(*node);}
  catch(const std::exception& e){std::cerr<<e.what()<<'\n';result=1;}
  rclcpp::shutdown();return result;
}

#pragma once
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>

namespace aims_mpcc_rt {
// Protocol-v1 authorization uses the trusted anchor, not refreshed TF stamps.
// Call under the ROS adapter's state mutex; no callbacks run inside this class.
class LocalizationHealth {
 public:
  using Values = std::map<std::string,std::string>;
  bool observe(const Values& v, std::int64_t ros_now_ns, double steady_now) {
    bool changed=false;
    try {
      if(v.at("protocol_version")!="1") throw std::invalid_argument("protocol");
      auto epoch=v.at("epoch");
      auto hs=integer(v.at("health_sequence"));
      auto as=integer(v.at("anchor_sequence"));
      auto stamp=integer(v.at("last_anchor_stamp_ns"));
      auto ready=v.at("ready");
      if(epoch.empty()||hs<0||as<0||stamp<0||(ready!="true"&&ready!="false")||
         !std::isfinite(steady_now)) throw std::invalid_argument("health");
      if(retired_.count(epoch)) return false;
      if(!epoch_.empty()&&epoch!=epoch_) {retired_.insert(epoch_);changed=true;}
      if(epoch!=epoch_) {
        epoch_=epoch;sequence_=health_sequence_=loss_sequence_=-1;
        stamp_ns_=-1;anchor_received_=nan();
      }
      if(hs<=health_sequence_) return changed;
      health_sequence_=hs;
      if(as<sequence_) return changed;
      if(as==sequence_&&stamp_ns_>=0&&stamp!=stamp_ns_)
        throw std::invalid_argument("anchor changed without sequence");
      if(as>sequence_)
        anchor_received_=steady_now-std::max(0.,(ros_now_ns-stamp)*1e-9);
      sequence_=as;stamp_ns_=stamp;received_=steady_now;
      if(ready=="false") loss_sequence_=std::max(loss_sequence_,as);
      ready_=ready=="true"&&as>loss_sequence_;
      auto it=v.find("state");reason_=it==v.end()?"Localization not ready":it->second;
    } catch(const std::exception&) {
      ready_=false;reason_="Malformed localization health";
    }
    return changed;
  }
  bool usable(std::int64_t ros_now_ns,double steady_now) const {
    if(!ready_||stamp_ns_<0) return false;
    double ros_age=(ros_now_ns-stamp_ns_)*1e-9;
    double anchor_age=steady_now-anchor_received_,heartbeat_age=steady_now-received_;
    return std::isfinite(ros_age)&&std::isfinite(anchor_age)&&std::isfinite(heartbeat_age)&&
      ros_age>=0&&ros_age<=1.&&anchor_age>=0&&anchor_age<=1.&&
      heartbeat_age>=0&&heartbeat_age<=.3;
  }
  const std::string& epoch() const {return epoch_;}
  const std::string& reason() const {return reason_;}
 private:
  static double nan(){return std::numeric_limits<double>::quiet_NaN();}
  static std::int64_t integer(const std::string& value) {
    std::size_t end=0;auto result=std::stoll(value,&end);
    if(value.find_first_not_of(" \t\r\n",end)!=std::string::npos)
      throw std::invalid_argument("invalid integer");
    return result;
  }
  std::string epoch_,reason_="Localization health unavailable";
  std::set<std::string> retired_;
  std::int64_t sequence_=-1,health_sequence_=-1,loss_sequence_=-1,stamp_ns_=-1;
  double received_=nan(),anchor_received_=nan();
  bool ready_=false;
};
}  // namespace aims_mpcc_rt

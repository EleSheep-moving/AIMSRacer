#pragma once
#include <cmath>
#include <array>
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
  void reset(){*this=LocalizationHealth{};}
  std::int64_t anchor_sequence()const{return sequence_;}
  std::int64_t anchor_stamp_ns()const{return stamp_ns_;}
  bool qualified()const{return qualified_;}
  const std::array<double,3>& alignment()const{return alignment_;}
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
        stamp_ns_=-1;anchor_received_=nan();qualified_=false;committed_=false;ready_=false;
      }
      if(hs<=health_sequence_) return changed;
      health_sequence_=hs;
      if(as<sequence_) return changed;
      if(as==sequence_&&stamp_ns_>=0&&stamp!=stamp_ns_)
        throw std::invalid_argument("anchor changed without sequence");
      std::array<double,7> transform{};bool qualified=false;
      auto valid=v.find("alignment_valid");
      if(valid!=v.end()&&valid->second=="true"){
        if(v.at("alignment_epoch")!=epoch||integer(v.at("alignment_anchor_sequence"))!=as||
           integer(v.at("alignment_stamp_ns"))!=stamp)throw std::invalid_argument("alignment identity");
        const char* fields[]={"x","y","z","qx","qy","qz","qw"};
        for(int i=0;i<7;++i){auto text=v.at(std::string("map_odom_")+fields[i]);std::size_t end=0;
          transform[i]=std::stod(text,&end);
          if(end!=text.size()||!std::isfinite(transform[i]))throw std::invalid_argument("alignment finite");}
        double norm=0.;for(int i=3;i<7;++i)norm+=transform[i]*transform[i];
        if(std::abs(std::sqrt(norm)-1.)>.01)throw std::invalid_argument("alignment quaternion");
        if(as==sequence_&&committed_&&transform!=transform_)throw std::invalid_argument("alignment changed without sequence");
        qualified=true;
      }
      if(as>sequence_)committed_=false;
      qualified_=qualified;
      if(qualified){committed_=true;transform_=transform;auto qx=transform[3],qy=transform[4],qz=transform[5],qw=transform[6];
        const double norm=std::sqrt(qx*qx+qy*qy+qz*qz+qw*qw);qx/=norm;qy/=norm;qz/=norm;qw/=norm;
        alignment_={transform[0],transform[1],std::atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))};}
      if(as>sequence_)
        anchor_received_=steady_now-std::max(0.,(ros_now_ns-stamp)*1e-9);
      sequence_=as;stamp_ns_=stamp;received_=steady_now;
      if(ready=="false") loss_sequence_=std::max(loss_sequence_,as);
      ready_=ready=="true"&&as>loss_sequence_;
      auto it=v.find("state");reason_=it==v.end()?"Localization not ready":it->second;
    } catch(const std::exception&) {
      ready_=false;qualified_=false;reason_="Malformed localization health";
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
  bool ready_=false,qualified_=false,committed_=false;
  std::array<double,7> transform_{};std::array<double,3> alignment_{};
};
}  // namespace aims_mpcc_rt

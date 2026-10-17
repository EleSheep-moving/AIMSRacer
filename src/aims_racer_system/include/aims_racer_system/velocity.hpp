#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <string>
#include <vector>
namespace aims_racer_system::velocity {
using V3=Eigen::Vector3d;using M3=Eigen::Matrix3d;using M9=Eigen::Matrix<double,9,9>;
struct Config {
 double gravity=9.80665,position_std=.03,bias_psd=1e-4,force_variance=.01,
 tilt_std=.00872664626,initial_bias_std=.2,initialization_span=.2,
 position_gate=16.266,angle_gate=.35,max_gap=.03,degraded_age=.3,max_age=.5,history=2.;
 std::size_t max_events=4096;V3 lever{.30,0,.03};M3 mounting=M3::Identity();
 bool continue_on_lio_timeout=false; // opt-in inertial coasting; status/covariance retain degradation
};
struct Imu {int64_t stamp;V3 f,w;};
struct Lio {int64_t stamp;V3 p;Eigen::Quaterniond q;};
struct Estimate {
 int64_t stamp=0,anchor=0;bool valid=false;std::string status="no_imu";
 double vx=0,variance=1.;V3 p=V3::Zero(),v=V3::Zero(),b=V3::Zero(),innovation=V3::Zero();
 // Body-basis IMU velocity covariance is conditional/effective, not calibrated.
 V3 imu_velocity_body=V3::Zero(),velocity_body=V3::Zero(),omega_body=V3::Zero();
 M3 velocity_covariance_body=M3::Identity();
 uint64_t rejected=0,reacquisitions=0;
};
// Orientation is a gyro-propagated, historical LIO-constrained nuisance state.
// Covariance is conditional/effective: shared LIO/IMU correlation is not calibrated.
class Observer {
 public:
 explicit Observer(Config config={});void reset();
 bool add_imu(const Imu&);bool add_lio(const Lio&);
 Estimate latest() const;
 std::size_t history_size() const{return imus_.size()+anchors_.size();}
 uint64_t replayed_imus() const{return replayed_;}
 private:
 struct State {
  std::optional<Imu> previous;M3 R=M3::Identity();V3 p=V3::Zero(),v=V3::Zero(),b=V3::Zero();
  M9 P=M9::Identity();bool oriented=false,initialized=false;int64_t anchor=0;
  std::vector<std::pair<int64_t,V3>> init;V3 innovation=V3::Zero();uint64_t rejected=0,reacquisitions=0;
  std::string status="no_imu";
 };
 Config c_;std::deque<Imu> imus_;std::deque<State> states_;std::map<int64_t,Lio> anchors_;
 State base_,s_;uint64_t replayed_=0;
 bool old(int64_t) const;void propagate(State&,double,const V3&,const V3&) const;
 void lio(State&,const Lio&) const;State advance(const State&,const Imu&) const;
 void replay(std::size_t);void trim();
};
}

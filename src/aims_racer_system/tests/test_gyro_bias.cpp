#include "aims_racer_system/gyro_bias.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using aims_racer_system::GyroBiasCalibrator;
using aims_racer_system::GyroBiasParameters;
const Eigen::Vector3d bias(.001, -.002, .02);
const Eigen::Vector3d accel(.3, .4, std::sqrt(.75));

void feed(GyroBiasCalibrator & c, double start=0., double end=10.1,
  double speed=0., double turning=0.)
{
  for (int i=0; start+i*.005 < end-1e-10; ++i) {
    const double t=start+i*.005;
    c.add_wheel(t,speed);
    c.add_lio(t,{speed*t,0.,0.},Eigen::Quaterniond(Eigen::AngleAxisd(turning*t,Eigen::Vector3d::UnitZ())),{speed,0.,0.});
    c.process(t,bias+Eigen::Vector3d(0.,0.,turning),accel);
  }
}

template<class F> void invalid(F f) {
  bool caught=false;
  try {f();} catch(const std::invalid_argument &) {caught=true;}
  assert(caught);
}

int main() {
  // Default ten-second calibration, inclined raw-g gravity, frozen all-axis bias.
  GyroBiasCalibrator c;
  feed(c,0.,10.);
  assert(!c.ready());
  feed(c,10.,10.1);
  assert(c.ready());
  assert((c.bias()-bias).norm()<1e-12);
  auto corrected=c.process(10.2,bias+Eigen::Vector3d(.1,.2,.5),accel);
  assert(corrected && (*corrected-Eigen::Vector3d(.1,.2,.5)).norm()<1e-12);
  assert((c.bias()-bias).norm()<1e-12);
  // Repeated or backward source timestamps invalidate even a frozen calibration.
  assert(!c.process(1.,bias,accel)); assert(!c.ready());
  assert(c.bias().isZero()); assert(!c.process(1.1,bias,accel));
  assert(c.reason()=="missing or stale source-time wheel/LIO observations");
  feed(c,1.2,11.4); assert(c.ready());
  assert(!c.process(11.395,bias,accel)); assert(!c.ready());

  GyroBiasCalibrator no_observation;
  for (int i=0;i<2200;++i) assert(!no_observation.process(i*.005,bias,accel));
  no_observation.add_wheel(100.,0.);
  no_observation.add_lio(100.,Eigen::Vector3d::Zero(),Eigen::Quaterniond::Identity(),Eigen::Vector3d::Zero());
  for (int i=0;i<2200;++i) assert(!no_observation.process(12.+i*.005,bias,accel));
  assert(!no_observation.ready());
  GyroBiasCalibrator moving; feed(moving,0.,10.1,.1); assert(!moving.ready());
  GyroBiasCalibrator turning; feed(turning,0.,10.1,0.,.003); assert(!turning.ready());
  GyroBiasCalibrator reset; feed(reset,0.,6.); feed(reset,6.,7.,.1);
  feed(reset,7.,16.); assert(!reset.ready()); feed(reset,16.,17.2); assert(reset.ready());
  // Raw acceleration is in g; SI-unit input cannot silently calibrate.
  GyroBiasCalibrator si;
  si.add_wheel(0.,0.);si.add_lio(0.,Eigen::Vector3d::Zero(),Eigen::Quaterniond::Identity(),Eigen::Vector3d::Zero());
  assert(!si.process(0.,bias,{0.,0.,9.81})); assert(si.sample_count()==0);
  assert(!si.process(.005,{0.,0.,std::numeric_limits<double>::quiet_NaN()},accel));
  invalid([]{GyroBiasParameters p;p.duration=0.;GyroBiasCalibrator bad(p);});
  invalid([]{GyroBiasParameters p;p.min_samples=2;GyroBiasCalibrator bad(p);});
  invalid([&]{si.add_wheel(1.,std::numeric_limits<double>::infinity());});
  invalid([&]{si.add_lio(1.,Eigen::Vector3d::Zero(),Eigen::Quaterniond(2.,0.,0.,0.),Eigen::Vector3d::Zero());});

  // Duration and minimum sample count are independent requirements.
  GyroBiasParameters short_params;short_params.duration=.01;short_params.min_samples=10;
  GyroBiasCalibrator short_window(short_params);
  feed(short_window,0.,.045);assert(!short_window.ready());
  feed(short_window,.045,.05);assert(short_window.ready());
  GyroBiasCalibrator gap(short_params);feed(gap,0.,.025);
  feed(gap,1.,1.025);assert(!gap.ready());assert(gap.sample_count()==5);
  feed(gap,1.025,1.05);assert(gap.ready());
  GyroBiasParameters noisy_params;noisy_params.duration=.1;noisy_params.min_samples=10;
  GyroBiasCalibrator noisy(noisy_params);
  for (int i=0;i<=20;++i) {
    double t=i*.005;noisy.add_wheel(t,0.);
    noisy.add_lio(t,Eigen::Vector3d::Zero(),Eigen::Quaterniond::Identity(),Eigen::Vector3d::Zero());
    noisy.process(t,bias+Eigen::Vector3d(i%2 ? .01 : -.01,0.,0.),accel);
  }
  assert(!noisy.ready());assert(noisy.sample_count()==0);
  assert(noisy.reason()=="sensor variation too high; restarting stationary window");
  GyroBiasCalibrator accel_noisy(noisy_params);
  for (int i=0;i<=20;++i) {
    double t=i*.005;accel_noisy.add_wheel(t,0.);
    accel_noisy.add_lio(t,Eigen::Vector3d::Zero(),Eigen::Quaterniond::Identity(),Eigen::Vector3d::Zero());
    accel_noisy.process(t,bias,accel+Eigen::Vector3d(i%2 ? .03 : -.03,0.,0.));
  }
  assert(!accel_noisy.ready());assert(accel_noisy.sample_count()==0);
  // Bounded observations and bounded calibration sample storage.
  GyroBiasCalibrator bounded;
  bounded.add_wheel(0.,0.);bounded.add_lio(0.,Eigen::Vector3d::Zero(),Eigen::Quaterniond::Identity(),Eigen::Vector3d::Zero());
  for(int i=1;i<=200;++i) {
    bounded.add_wheel(100.+i,0.);
    bounded.add_lio(100.+i,Eigen::Vector3d::Zero(),Eigen::Quaterniond::Identity(),Eigen::Vector3d::Zero());
  }
  assert(!bounded.process(0.,bias,accel));assert(bounded.sample_count()==0);
  GyroBiasParameters budget_params;budget_params.duration=1000.;budget_params.min_samples=3;
  GyroBiasCalibrator budget(budget_params);feed(budget,0.,50.005);
  assert(budget.reason()=="calibration sample budget exceeded");assert(budget.sample_count()==0);

  sensor_msgs::msg::Imu msg;
  msg.header.frame_id="livox_frame";msg.header.stamp.sec=123;
  msg.angular_velocity.x=bias.x();msg.angular_velocity.y=bias.y();msg.angular_velocity.z=bias.z();
  msg.linear_acceleration.x=.3;msg.linear_acceleration.y=.4;msg.linear_acceleration.z=.866;
  auto output=aims_racer_system::correct_imu(msg,bias);
  assert(output.header==msg.header && output.linear_acceleration==msg.linear_acceleration);
  assert(output.angular_velocity_covariance==msg.angular_velocity_covariance);
  assert(msg.angular_velocity.z==.02 && output.angular_velocity.z==0.);
  msg.angular_velocity_covariance={.01,.002,0.,.002,.01,0.,0.,0.,.01};
  output=aims_racer_system::correct_imu(msg,bias);
  assert(std::abs(output.angular_velocity_covariance[8]-.010001)<1e-15);
  assert(output.angular_velocity_covariance[1]==.002);
  assert(msg.angular_velocity_covariance[8]==.01);
  msg.angular_velocity_covariance[0]=-1.;
  invalid([&]{aims_racer_system::correct_imu(msg,bias);});
  std::cout<<"gyro bias native assertions passed\n";
}

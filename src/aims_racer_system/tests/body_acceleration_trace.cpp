// Source-time trace driver. stdin:
// IMU stamp_ns fx_g fy_g fz_g wx_rad_s wy_rad_s wz_rad_s
// LIO stamp_ns qw qx qy qz
// RESET
// Every input event returns a CSV row; only new accepted IMU + valid rows are
// publication candidates. LIO rows show revised internal state at latest IMU.
#include "aims_racer_system/body_acceleration.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
using namespace aims_racer_system::body_acceleration;
int main(int argc,char **argv) {
  try {
    Config c;
    auto number=[&](int & i) {if(++i>=argc)throw std::invalid_argument("missing option value");return std::stod(argv[i]);};
    for(int i=1;i<argc;++i) {
      const std::string arg=argv[i];
      if(arg=="--no-filter")c.filter_enabled=false;
      else if(arg=="--lateral")c.lateral_enabled=true;
      else if(arg=="--jerk-psd")c.jerk_psd=number(i);
      else if(arg=="--max-lio-age")c.max_lio_age_s=number(i);
      else if(arg=="--mount") {
        const double w=number(i),x=number(i),y=number(i),z=number(i);
        c.mounting=Eigen::Quaterniond(w,x,y,z);
      } else if(arg=="--lever"||arg=="--bias") {
        const double x=number(i),y=number(i),z=number(i);
        if(arg=="--lever")c.lever=Eigen::Vector3d(x,y,z);else c.accel_bias_body=Eigen::Vector3d(x,y,z);
      } else throw std::invalid_argument("unknown option: "+arg);
    }
    Observer observer(c);
    std::cout<<std::setprecision(17);
    std::cout<<"event,accepted,stamp_ns,valid,status,anchor_stamp_ns,ax,jerk,raw_ax,raw_ay,raw_az,variance,measurement_variance,force_x,force_y,force_z,gravity_x,gravity_y,gravity_z,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z,tangential_x,tangential_y,tangential_z,centripetal_x,centripetal_y,centripetal_z,event_cost_us,replayed_imu_delta,alpha_window_span_s,alpha_sample_center_age_s,ay,lateral_jerk,lateral_variance,lateral_valid\n";
    std::string line;
    while(std::getline(std::cin,line)) {
      std::istringstream input(line);std::string op;input>>op;
      if(op.empty()||op[0]=='#')continue;
      bool accepted=false;
      double event_cost_us=0.0;
      const auto replayed_before=observer.counters().replayed_imu;
      auto timed=[&](auto && call) {
        const auto start=std::chrono::steady_clock::now();
        const bool result=call();
        const auto finish=std::chrono::steady_clock::now();
        event_cost_us=std::chrono::duration<double,std::micro>(finish-start).count();
        return result;
      };
      if(op=="IMU") {
        ImuSample m;
        if(!(input>>m.stamp_ns>>m.specific_force_g.x()>>m.specific_force_g.y()>>m.specific_force_g.z()
          >>m.angular_velocity_rad_s.x()>>m.angular_velocity_rad_s.y()>>m.angular_velocity_rad_s.z()))
          throw std::invalid_argument("invalid IMU line");
        accepted=timed([&]{return observer.add_imu(m);});
      } else if(op=="LIO") {
        int64_t stamp;double w,x,y,z;
        if(!(input>>stamp>>w>>x>>y>>z))throw std::invalid_argument("invalid LIO line");
        accepted=timed([&]{return observer.add_lio({stamp,Eigen::Quaterniond(w,x,y,z)});});
      } else if(op=="RESET") {accepted=timed([&]{observer.reset();return true;});}
      else throw std::invalid_argument("unknown event: "+op);
      const auto e=observer.latest();
      std::cout<<op<<','<<accepted<<','<<e.stamp_ns<<','<<e.valid<<','<<status_name(e.status)<<','
        <<e.anchor_stamp_ns<<','<<e.ax<<','<<e.jerk;
      auto vector=[&](const Eigen::Vector3d & v) {std::cout<<','<<v.x()<<','<<v.y()<<','<<v.z();};
      vector(e.acceleration_body);std::cout<<','<<e.acceleration_variance<<','<<e.measurement_variance;
      vector(e.force_body);vector(e.gravity_body);vector(e.omega_body);vector(e.alpha_body);vector(e.tangential);vector(e.centripetal);
      const uint64_t replayed_delta=op=="RESET" ? 0 : observer.counters().replayed_imu-replayed_before;
      std::cout<<','<<event_cost_us<<','<<replayed_delta<<','<<e.alpha_window_span_s
        <<','<<e.alpha_sample_center_age_s<<','<<e.ay<<','<<e.lateral_jerk<<','<<e.lateral_acceleration_variance<<','<<e.lateral_valid<<'\n';
    }
  } catch(const std::exception & e) {std::cerr<<e.what()<<'\n';return 2;}
}

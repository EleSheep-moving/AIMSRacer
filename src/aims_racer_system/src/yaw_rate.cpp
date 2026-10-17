#include "aims_racer_system/yaw_rate.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace aims_racer_system::yaw_rate {
Observer::Observer(const Config & c):config_(c) {
  for(double v:{c.process_psd,c.measurement_variance,c.initial_alpha_variance,c.max_gap_s})
    if(!std::isfinite(v)||v<=0||v>1e12)throw std::invalid_argument("invalid yaw-rate model parameter");
  if(c.max_gap_s>.5)throw std::invalid_argument("max_gap_s must be <=0.5");
  double norm=0.;for(double v:c.mount_xyzw){if(!std::isfinite(v))throw std::invalid_argument("invalid mounting quaternion");norm+=v*v;}
  if(!std::isfinite(norm)||std::abs(norm-1.)>1e-6)throw std::invalid_argument("mounting quaternion must be unit length");
  const double n=std::sqrt(norm),x=c.mount_xyzw[0]/n,y=c.mount_xyzw[1]/n,z=c.mount_xyzw[2]/n,w=c.mount_xyzw[3]/n;
  project_={2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)}; // row z of R_body_imu
}
void Observer::reset(){estimate_=Estimate{};}
bool Observer::add(const Sample & s) {
  if(s.stamp_ns<=0){++counters_.invalid;return false;}
  for(double v:s.gyro_rad_s)if(!std::isfinite(v)){++counters_.invalid;return false;}
  if(s.stamp_ns==estimate_.stamp_ns){++counters_.duplicate;return false;}
  if(s.stamp_ns<estimate_.stamp_ns){++counters_.out_of_order;return false;}
  double z=0.;for(unsigned i=0;i<3;++i)z+=project_[i]*s.gyro_rad_s[i];
  // Arithmetic guard, not a physical sensor saturation range or spike filter.
  if(!std::isfinite(z)||std::abs(z)>1e100){++counters_.invalid;++counters_.numerical_error;return false;}
  auto e=estimate_;const double dt=(s.stamp_ns-e.stamp_ns)*1e-9;
  const bool gap=e.stamp_ns>0&&dt>config_.max_gap_s+1e-12;
  const double r=config_.measurement_variance;
  if(!e.stamp_ns||gap) {
    e=Estimate{};e.rate=z;e.rate_variance=r;e.alpha_variance=config_.initial_alpha_variance;
  } else {
    const double p00=e.rate_variance+2*dt*e.cross_covariance+dt*dt*e.alpha_variance+config_.process_psd*dt*dt*dt/3;
    const double p01=e.cross_covariance+dt*e.alpha_variance+config_.process_psd*dt*dt/2;
    const double p11=e.alpha_variance+config_.process_psd*dt;
    const double k0=p00/(p00+r),k1=p01/(p00+r),innovation=z-(e.rate+dt*e.alpha);
    e.rate+=dt*e.alpha+k0*innovation;e.alpha+=k1*innovation;
    e.rate_variance=(1-k0)*(1-k0)*p00+k0*k0*r;
    e.cross_covariance=(1-k0)*(p01-k1*p00)+k0*k1*r;
    e.alpha_variance=p11-2*k1*p01+k1*k1*p00+k1*k1*r;
  }
  for(double v:{e.rate,e.alpha,e.rate_variance,e.alpha_variance,e.cross_covariance})
    if(!std::isfinite(v)){++counters_.invalid;++counters_.numerical_error;return false;}
  if(e.rate_variance<0||e.alpha_variance<0||
      e.rate_variance*e.alpha_variance-e.cross_covariance*e.cross_covariance < -1e-12) {
    ++counters_.invalid;++counters_.numerical_error;return false;
  }
  e.stamp_ns=s.stamp_ns;e.measured_rate=z;e.consecutive_samples=std::min(3u,e.consecutive_samples+1);
  e.valid=e.consecutive_samples>=3;estimate_=e;
  ++counters_.accepted;if(gap)++counters_.gaps;return true;
}
}

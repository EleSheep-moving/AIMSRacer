#include "aims_racer_system/body_acceleration.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <stdexcept>
#include <vector>

namespace aims_racer_system::body_acceleration {
namespace {
double seconds(int64_t a, int64_t b) { return static_cast<double>(a-b)*1e-9; }
bool quaternion_ok(const Eigen::Quaterniond & q) {
  return q.coeffs().allFinite() && std::abs(q.norm()-1.0)<=1e-3;
}
Eigen::Quaterniond delta(const Eigen::Vector3d & rotation) {
  const double angle=rotation.norm();
  return angle<1e-14 ? Eigen::Quaterniond::Identity() : Eigen::Quaterniond(Eigen::AngleAxisd(angle,rotation/angle));
}
struct Gyro { int64_t stamp; Eigen::Vector3d omega; };
struct State {
  bool has_imu=false, oriented=false, filter_initialized=false;
  ImuSample previous;
  int64_t anchor=0;
  Eigen::Quaterniond q=Eigen::Quaterniond::Identity();
  std::deque<Gyro> gyro;
  Eigen::Vector2d x=Eigen::Vector2d::Zero();
  Eigen::Vector2d y=Eigen::Vector2d::Zero(); // separate mean, shared matched-response gain/covariance
  Eigen::Matrix2d p=Eigen::Matrix2d::Identity();
  int64_t filter_stamp=0;
  Estimate estimate;
};
}
const char * status_name(Status s) {
  switch(s) {
    case Status::no_imu:return "no_imu"; case Status::no_orientation:return "no_orientation";
    case Status::gyro_warmup:return "gyro_warmup";
    case Status::gyro_time_support:return "gyro_time_support"; case Status::gyro_gap:return "gyro_gap";
    case Status::lio_stale:return "lio_stale"; case Status::numerical_error:return "numerical_error";
    case Status::valid:return "valid";
  } return "unknown";
}
struct Observer::Impl {
  Config c; Counters count;
  struct Entry { ImuSample sample; State after; };
  std::deque<Entry> imus;
  std::map<int64_t,Eigen::Quaterniond> anchors;
  State baseline, current;
  explicit Impl(const Config & config):c(config) {
    if (!quaternion_ok(c.mounting) || !c.lever.allFinite() || !c.accel_bias_body.allFinite())
      throw std::invalid_argument("invalid mounting, lever or acceleration bias");
    for(double v:{c.gravity,c.alpha_window_s,c.max_gyro_gap_s,c.max_lio_age_s,c.history_s,c.jerk_psd,c.force_variance,c.gyro_variance,c.tilt_std_rad})
      if(!std::isfinite(v)||v<=0) throw std::invalid_argument("positive finite model parameters required");
    if(!std::isfinite(c.min_alpha_span_s)||c.min_alpha_span_s<0.||c.min_alpha_span_s>c.alpha_window_s)
      throw std::invalid_argument("min_alpha_span_s must be within the angular regression window");
    if(c.gravity<9.0||c.gravity>10.0||c.alpha_window_s>0.2||c.max_gyro_gap_s>0.2||c.history_s<c.max_lio_age_s||c.max_events<8)
      throw std::invalid_argument("invalid physical scale or history bounds");
    if (!std::isfinite(c.lever.squaredNorm()) ||
      !std::isfinite(c.accel_bias_body.squaredNorm()) ||
      !std::isfinite(std::pow(c.gravity*c.tilt_std_rad, 2)))
      throw std::invalid_argument("calibration/model arithmetic overflows");
    c.mounting.normalize();
  }
  bool advance(State & s,const ImuSample & m) {
    Estimate e; e.stamp_ns=m.stamp_ns;
    e.force_body=c.mounting*m.specific_force_g*c.gravity;
    e.corrected_force_body=e.force_body-c.accel_bias_body;
    e.omega_body=c.mounting*m.angular_velocity_rad_s;
    const bool gap=s.has_imu && seconds(m.stamp_ns,s.previous.stamp_ns)>c.max_gyro_gap_s+1e-12;
    if(gap) { s.oriented=false; s.gyro.clear(); s.filter_initialized=false; }
    const auto start=s.has_imu ? anchors.upper_bound(s.previous.stamp_ns):anchors.lower_bound(m.stamp_ns);
    const auto end=anchors.upper_bound(m.stamp_ns);
    int64_t cursor=s.has_imu?s.previous.stamp_ns:m.stamp_ns;
    const Eigen::Vector3d prev_omega=s.has_imu?c.mounting*s.previous.angular_velocity_rad_s:e.omega_body;
    const double dt=s.has_imu?seconds(m.stamp_ns,s.previous.stamp_ns):0.0;
    auto omega_at=[&](int64_t stamp)->Eigen::Vector3d {
      if(dt<=0) return e.omega_body;
      const double r=seconds(stamp,s.previous.stamp_ns)/dt;
      return prev_omega+(e.omega_body-prev_omega)*r;
    };
    auto propagate=[&](int64_t until) {
      if(s.oriented && until>cursor)
        s.q=(s.q*delta(0.5*(omega_at(cursor)+omega_at(until))*seconds(until,cursor))).normalized();
      cursor=until;
    };
    for(auto it=start;it!=end;++it) {
      // A source anchor cannot bridge a missing IMU interval. Exact current anchors
      // restore coverage; interior anchors require a continuously sampled bracket.
      if(!gap || it->first==m.stamp_ns) {
        propagate(it->first); s.q=it->second; s.anchor=it->first; s.oriented=true; cursor=it->first;
      }
    }
    propagate(m.stamp_ns);
    s.gyro.push_back({m.stamp_ns,e.omega_body});
    while(!s.gyro.empty() && (s.gyro.size()>std::min<std::size_t>(512,c.max_events) ||
      seconds(m.stamp_ns,s.gyro.front().stamp)>c.alpha_window_s+1e-12)) s.gyro.pop_front();
    double denominator=0, mean=0, alpha_variance=0;
    const bool derivative=s.gyro.size()>=3;
    for(const auto & g:s.gyro) mean+=seconds(g.stamp,m.stamp_ns);
    if(!s.gyro.empty()) {
      mean/=s.gyro.size();
      e.alpha_window_span_s=seconds(s.gyro.back().stamp,s.gyro.front().stamp);
      e.alpha_sample_center_age_s=-mean;
    }
    const bool time_support=c.min_alpha_span_s==0. || e.alpha_window_span_s+1e-12>=c.min_alpha_span_s;
    if(derivative && time_support) {
      for(const auto & g:s.gyro) { const double t=seconds(g.stamp,m.stamp_ns)-mean; denominator+=t*t; }
      if(denominator>1e-16) for(const auto & g:s.gyro) {
        const double w=(seconds(g.stamp,m.stamp_ns)-mean)/denominator;
        e.alpha_body+=w*g.omega; alpha_variance+=w*w*c.gyro_variance;
      }
    }
    e.q_world_body=s.q; e.anchor_stamp_ns=s.anchor;
    e.gravity_body=s.q.conjugate()*Eigen::Vector3d(0,0,-c.gravity);
    e.tangential=e.alpha_body.cross(c.lever);
    e.centripetal=e.omega_body.cross(e.omega_body.cross(c.lever));
    e.acceleration_body=e.corrected_force_body+e.gravity_body-e.tangential-e.centripetal;
    // Correlated LIO/IMU information is not counted as independent measurements.
    // Inflation and a measurement floor prevent KF covariance claiming calibrated accuracy.
    const double lever2=c.lever.squaredNorm();
    e.measurement_variance=c.force_variance+2.0*lever2*alpha_variance+
      8.0*lever2*e.omega_body.squaredNorm()*c.gyro_variance+
      std::pow(c.gravity*c.tilt_std_rad,2);
    e.status=gap?Status::gyro_gap:(!s.oriented?Status::no_orientation:
      (!time_support?Status::gyro_time_support:(!derivative?Status::gyro_warmup:
      (denominator<=1e-16?Status::gyro_warmup:
      (seconds(m.stamp_ns,s.anchor)>c.max_lio_age_s+1e-12?Status::lio_stale:Status::valid)))));
    if(!e.acceleration_body.allFinite() || !e.alpha_body.allFinite() ||
      !e.omega_body.allFinite() || !e.q_world_body.coeffs().allFinite() ||
      !std::isfinite(e.measurement_variance)) {
      e.status=Status::numerical_error; s.oriented=false;
    }
    e.valid=e.status==Status::valid || (c.continue_on_lio_timeout && e.status==Status::lio_stale);
    if(c.continue_on_lio_timeout && e.status==Status::lio_stale) {
      // Uncalibrated conservative growth model, not measured attitude accuracy.
      const double coast=seconds(m.stamp_ns,s.anchor)-c.max_lio_age_s;
      e.measurement_variance+=c.gravity*c.gravity*c.gyro_variance*coast*coast;
    }
    e.ax=e.acceleration_body.x();
    e.ay=e.acceleration_body.y();
    e.acceleration_variance=e.measurement_variance;
    if(e.valid && c.filter_enabled) {
      if(!s.filter_initialized) {
        s.x<<e.ax,0.0; s.p=Eigen::Matrix2d::Zero(); s.p(0,0)=e.measurement_variance; s.p(1,1)=c.jerk_psd;
        if(c.lateral_enabled) s.y<<e.ay,0.0;
        s.filter_initialized=true;
      } else {
        const double h=seconds(m.stamp_ns,s.filter_stamp);
        Eigen::Matrix2d f; f<<1,h,0,1;
        Eigen::Matrix2d q; q<<h*h*h/3,h*h/2,h*h/2,h; q*=c.jerk_psd;
        s.x=f*s.x; s.p=f*s.p*f.transpose()+q;
        const Eigen::Vector2d k=s.p.col(0)/(s.p(0,0)+e.measurement_variance);
        s.x+=k*(e.ax-s.x[0]);
        if(c.lateral_enabled) { s.y=f*s.y; s.y+=k*(e.ay-s.y[0]); }
        Eigen::Matrix2d a=Eigen::Matrix2d::Identity(); a.col(0)-=k;
        s.p=a*s.p*a.transpose()+k*e.measurement_variance*k.transpose();
        s.p=0.5*(s.p+s.p.transpose()).eval();
      }
      s.filter_stamp=m.stamp_ns; e.ax=s.x[0]; e.jerk=s.x[1];
      e.acceleration_variance=std::max(e.measurement_variance,s.p(0,0));
      if(c.lateral_enabled) { e.ay=s.y[0]; e.lateral_jerk=s.y[1]; }
    } else if(!e.valid) s.filter_initialized=false;
    e.lateral_acceleration_variance=e.acceleration_variance;
    e.lateral_valid=e.valid && c.lateral_enabled;
    if(e.valid && (!std::isfinite(e.ax) || !std::isfinite(e.jerk) ||
      !std::isfinite(e.acceleration_variance))) {
      e.valid=false; e.status=Status::numerical_error; s.filter_initialized=false;
    }
    // Admission is transactional: callers discard this candidate on any arithmetic
    // failure, including failures that arise while replaying later filter states.
    if (e.status==Status::numerical_error || !s.q.coeffs().allFinite() ||
      !s.x.allFinite() || !s.p.allFinite() || (c.lateral_enabled && !s.y.allFinite()) ||
      s.p(0,0)<0.0 || s.p(1,1)<0.0 ||
      !std::isfinite(s.p.determinant()) || s.p.determinant()<-1e-9) return false;
    s.has_imu=true; s.previous=m; s.estimate=e;
    return true;
  }
  bool replay(std::size_t first) {
    State candidate=first==0 ? baseline : imus[first-1].after;
    std::vector<State> revised;
    revised.reserve(imus.size()-first);
    for (std::size_t i=first; i<imus.size(); ++i) {
      if (!advance(candidate, imus[i].sample)) return false;
      revised.push_back(candidate);
    }
    // No live history/checkpoint is modified until the entire suffix succeeds.
    for (std::size_t i=0; i<revised.size(); ++i)
      imus[first+i].after=std::move(revised[i]);
    current=imus.empty() ? baseline : imus.back().after;
    ++count.replays; count.replayed_imu+=revised.size();
    return true;
  }
  void trim() {
    if(imus.empty()) {
      if(!anchors.empty()) {
        const int64_t newest=anchors.rbegin()->first;
        while(!anchors.empty() && (anchors.size()>c.max_events ||
          seconds(newest,anchors.begin()->first)>c.history_s)) {
          anchors.erase(anchors.begin()); ++count.evicted;
        }
      }
      return;
    }
    const int64_t newest=imus.back().sample.stamp_ns;
    while(imus.size()>1 && (imus.size()+anchors.size()>c.max_events || seconds(newest,imus.front().sample.stamp_ns)>c.history_s)) {
      baseline=std::move(imus.front().after); imus.pop_front(); ++count.evicted;
      anchors.erase(anchors.begin(),anchors.upper_bound(baseline.previous.stamp_ns));
    }
    // Future anchors count toward the same bound, and cannot force unbounded storage.
    while(imus.size()+anchors.size()>c.max_events) { anchors.erase(std::prev(anchors.end())); ++count.evicted; }
  }
  bool old(int64_t stamp) const {
    return (baseline.has_imu&&stamp<=baseline.previous.stamp_ns)||
      (!imus.empty()&&seconds(imus.back().sample.stamp_ns,stamp)>c.history_s);
  }
};
Observer::Observer(const Config & c):impl_(new Impl(c)){}
Observer::~Observer()=default;
Observer::Observer(Observer &&) noexcept=default;
Observer & Observer::operator=(Observer &&) noexcept=default;
bool Observer::add_imu(const ImuSample & m) {
  auto & p=*impl_;
  if(m.stamp_ns<=0||!m.specific_force_g.allFinite()||!m.angular_velocity_rad_s.allFinite()) {
    ++p.count.invalid; return false;
  }
  if(p.old(m.stamp_ns)) { ++p.count.too_old; return false; }
  auto it=std::lower_bound(p.imus.begin(),p.imus.end(),m.stamp_ns,
    [](const auto & a,int64_t t){return a.sample.stamp_ns<t;});
  if(it!=p.imus.end()&&it->sample.stamp_ns==m.stamp_ns) {
    ++p.count.duplicate; return false;
  }
  const bool ordered=it==p.imus.end();
  if(ordered) {
    State candidate=p.current;
    if(!p.advance(candidate,m)) { ++p.count.invalid; ++p.count.numerical_error; return false; }
    if(p.current.has_imu && seconds(m.stamp_ns,p.current.previous.stamp_ns)>p.c.max_gyro_gap_s+1e-12)
      ++p.count.gyro_gaps;
    p.current=std::move(candidate);
    p.imus.push_back({m,p.current});
  } else {
    const std::size_t index=static_cast<std::size_t>(it-p.imus.begin());
    p.imus.insert(it,{m,{}});
    if(!p.replay(index)) {
      p.imus.erase(p.imus.begin()+index);
      ++p.count.invalid; ++p.count.numerical_error; return false;
    }
  }
  ++p.count.accepted_imu;
  p.trim(); return true;
}
bool Observer::add_lio(const LioSample & l) {
  auto & p=*impl_;
  if(l.stamp_ns<=0||!quaternion_ok(l.q_world_body)) { ++p.count.invalid; return false; }
  if(p.old(l.stamp_ns)) { ++p.count.too_old; return false; }
  if(p.anchors.count(l.stamp_ns)) { ++p.count.duplicate; return false; }
  p.anchors.emplace(l.stamp_ns,l.q_world_body.normalized());
  if(!p.imus.empty()&&l.stamp_ns<=p.imus.back().sample.stamp_ns) {
    const auto first=std::lower_bound(p.imus.begin(),p.imus.end(),l.stamp_ns,
      [](const auto & a,int64_t t){return a.sample.stamp_ns<t;});
    if(!p.replay(static_cast<std::size_t>(first-p.imus.begin()))) {
      p.anchors.erase(l.stamp_ns); ++p.count.invalid; ++p.count.numerical_error; return false;
    }
  }
  ++p.count.accepted_lio;
  p.trim(); return true;
}
Estimate Observer::latest() const {return impl_->current.estimate;}
Counters Observer::counters() const {return impl_->count;}
std::size_t Observer::history_size() const {return impl_->imus.size()+impl_->anchors.size();}
void Observer::reset() {const Config c=impl_->c; impl_.reset(new Impl(c));}
} // namespace aims_racer_system::body_acceleration

#include "aims_mpcc_rt/core.hpp"
#include <yaml-cpp/yaml.h>
#include <openssl/evp.h>
#include <dlfcn.h>
#include <sys/utsname.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace aims_mpcc_rt {
namespace {
constexpr double pi=3.14159265358979323846;
using Clock=std::chrono::steady_clock;
double duration(Clock::time_point t) {return std::chrono::duration<double>(Clock::now()-t).count();}
double wrap(double x,double period) {return x-period*std::floor(x/period);}
template<class C> bool finite(const C &v) {return std::all_of(v.begin(),v.end(),[](double x){return std::isfinite(x);});}
std::string hash(const std::filesystem::path &path) {
  std::ifstream in(path,std::ios::binary);
  if(!in)throw std::runtime_error("artifact file missing: "+path.string());
  auto ctx=EVP_MD_CTX_new();
  if(!ctx)throw std::runtime_error("SHA256 allocation failed");
  EVP_DigestInit_ex(ctx,EVP_sha256(),nullptr);
  std::array<char,16384> data{};
  while(in){in.read(data.data(),data.size());EVP_DigestUpdate(ctx,data.data(),in.gcount());}
  std::array<unsigned char,EVP_MAX_MD_SIZE> digest{};unsigned int count{};
  EVP_DigestFinal_ex(ctx,digest.data(),&count);EVP_MD_CTX_free(ctx);
  std::ostringstream out;for(unsigned int i=0;i<count;++i)out<<std::hex<<std::setw(2)<<std::setfill('0')<<int(digest[i]);
  return out.str();
}
void verify(const std::filesystem::path &path,const YAML::Node &expected) {
  if(hash(path)!=expected.as<std::string>())throw std::runtime_error("artifact hash mismatch: "+path.string());
}
std::vector<double> values(const YAML::Node &node) {std::vector<double> out;for(auto v:node)out.push_back(v.as<double>());return out;}
double stopping_rate(double distance,double acceleration,double dt) {
  distance=std::max(0.,distance);double change=acceleration*dt;
  double n=std::floor((std::sqrt(1.+8.*distance/(dt*change))-1.)/2.)+1.;
  return distance/(n*dt)+change*(n-1.)/2.;
}
double utilization(const State &s,double acceleration,const Config &c) {
  auto axis=acceleration>=0?c.envelope_accel:c.envelope_brake;
  auto ay=s[3]*s[3]*std::tan(s[5])/(c.wheelbase*(1.+c.understeer_coefficient*s[3]*s[3]));
  return std::pow(acceleration/axis,2)+std::pow(ay/c.lateral_accel_limit,2);
}
InternalState internal(const State &s,const Applied &a) {
  InternalState x{};std::copy(s.begin(),s.end(),x.begin());std::copy(a.begin(),a.end(),x.begin()+6);return x;
}
State physical(const InternalState &x) {State s{};std::copy_n(x.begin(),6,s.begin());return s;}
}

struct Bundle::Impl {
  Config cfg;Reference reference;std::string fingerprint;
  std::vector<double> lower,upper,terminal_lower,terminal_upper,input_lower,input_upper;
  std::vector<double> cost_scaling;
  std::vector<std::string> constraint_groups;
  int nu{},nh{},nh_e{},sample_count{};void *library{};
  void *(*create)(){};void (*free)(void*){};int (*reset)(void*){};
  void (*set)(void*,int,const char*,const double*){};
  void (*initial)(void*,const double*){};int (*parameters)(void*,int,double*){};
  int (*solve)(void*){};void (*get)(void*,int,const char*,double*){};
  double (*cost)(void*){};
  int (*eval)(int,const double*,const double*,const double*,double*){};
  ~Impl(){if(library)dlclose(library);}
};

Bundle Bundle::load(const std::string &directory,const std::string &config_path,
                    const std::string &reference_dir,const std::string &source_root) {
  auto root=std::filesystem::absolute(directory);auto manifest=YAML::LoadFile((root/"manifest.json").string());
  auto native=YAML::LoadFile((root/"native_manifest.json").string());
  verify(root/"manifest.json",native["source_manifest_sha256"]);
  if(manifest["schema_version"].as<int>()!=1||manifest["capsule_abi_version"].as<int>()!=1||native["schema_version"].as<int>()!=1||
     manifest["backend"].as<std::string>()!="acados_sqp_rti_hpipm"||manifest["nx"].as<int>()!=9||
     manifest["np"].as<int>()!=10||manifest["acados_commit"].as<std::string>()!="59d93e17d2985fdd73fc58b8a83ed8f83a024171")
    throw std::runtime_error("unsupported MPCC artifact version/model");
  utsname platform{};if(uname(&platform))throw std::runtime_error("cannot inspect target platform");
  if(native["machine"].as<std::string>()!=platform.machine||native["system"].as<std::string>()!=platform.sysname||
     native["acados_commit"].as<std::string>()!=manifest["acados_commit"].as<std::string>()||
     native["source_fingerprint"].as<std::string>()!=manifest["fingerprint"].as<std::string>())
    throw std::runtime_error("artifact target/fingerprint mismatch");
  for(auto entry:manifest["files"])verify(root/entry.first.as<std::string>(),entry.second);
  for(auto entry:native["libraries"])verify(root/entry.first.as<std::string>(),entry.second);
  for(auto entry:native["dependencies"])verify(entry.first.as<std::string>(),entry.second);
  if(!config_path.empty())verify(config_path,manifest["input_config_sha256"]);
  if(!reference_dir.empty())for(auto entry:manifest["input_reference_hashes"])
    verify(std::filesystem::path(reference_dir)/entry.first.as<std::string>(),entry.second);
  if(!source_root.empty())for(auto entry:manifest["source_hashes"])
    verify(std::filesystem::path(source_root)/entry.first.as<std::string>(),entry.second);
  Bundle bundle;bundle.impl_=std::make_shared<Impl>();auto &i=*bundle.impl_;
  i.fingerprint=manifest["fingerprint"].as<std::string>();
  i.lower=values(manifest["lower"]);i.upper=values(manifest["physical_upper"]);
  i.terminal_lower=values(manifest["terminal_lower"]);i.terminal_upper=values(manifest["terminal_upper"]);
  i.input_lower=values(manifest["input_lower"]);i.input_upper=values(manifest["input_upper"]);
  i.cost_scaling=values(manifest["cost_scaling"]);
  i.nu=manifest["nu"].as<int>();i.nh=manifest["nh"].as<int>();i.nh_e=manifest["nh_e"].as<int>();
  i.sample_count=manifest["candidate_sample_count"].as<int>();
  if(manifest["constraint_groups"])for(auto group:manifest["constraint_groups"])
    i.constraint_groups.push_back(group.as<std::string>());
  else for(int row=0;row<i.nh;++row)i.constraint_groups.push_back("nonlinear_row_"+std::to_string(row));
  if(i.constraint_groups.size()!=size_t(i.nh))throw std::runtime_error("constraint group count mismatch");
  auto data=YAML::LoadFile((root/"config.json").string());auto &c=i.cfg;
  c.profile=data["profile"].as<std::string>();
  c.command_profile=data["command_profile"]?data["command_profile"].as<std::string>():"legacy_bounded_v1";
  std::string exported_profile=manifest["command_profile"]?manifest["command_profile"].as<std::string>():"legacy_bounded_v1";
  if((c.command_profile!="legacy_bounded_v1"&&c.command_profile!="rate_bounded_v2")||
     c.command_profile!=exported_profile)throw std::runtime_error("artifact command profile mismatch");
  if(c.command_profile=="rate_bounded_v2"){
    if(!manifest["constraint_groups"])throw std::runtime_error("v2 constraint provenance missing");
    for(auto group:manifest["constraint_groups"])
      if(group.as<std::string>()=="jerk"||group.as<std::string>()=="steering_acceleration")
        throw std::runtime_error("v2 bundle retains removed hard constraint");
  }
#define FIELD(name) c.name=data[#name].as<double>()
  FIELD(wheelbase);FIELD(rear_offset);FIELD(half_width);FIELD(cruise_speed);FIELD(max_speed);
  FIELD(minimum_drive_speed);FIELD(steer_limit);FIELD(steer_rate);FIELD(steer_acceleration);
  FIELD(accel_limit);FIELD(brake_limit);FIELD(jerk_limit);FIELD(steering_tau);FIELD(understeer_coefficient);
  FIELD(lateral_accel_limit);FIELD(recovery_jerk_limit);FIELD(envelope_recovery_time);FIELD(envelope_slack_limit);
#undef FIELD
  c.steering_acceleration_scale=data["steering_acceleration_scale"]&&!data["steering_acceleration_scale"].IsNull()?
    data["steering_acceleration_scale"].as<double>():c.steer_acceleration;
  if(!std::isfinite(c.steering_acceleration_scale)||c.steering_acceleration_scale<=0.)
    throw std::runtime_error("invalid steering rate change objective scale");
  c.front_extent=data["front_extent"].IsNull()?c.rear_offset+data["half_length"].as<double>():data["front_extent"].as<double>();
  c.rear_extent=data["rear_extent"].IsNull()?data["half_length"].as<double>()-c.rear_offset:data["rear_extent"].as<double>();
  c.envelope_accel=data["longitudinal_envelope_accel"].IsNull()?c.accel_limit:data["longitudinal_envelope_accel"].as<double>();
  c.envelope_brake=data["longitudinal_envelope_brake"].IsNull()?c.brake_limit:data["longitudinal_envelope_brake"].as<double>();
  c.enforce_corridor=data["enforce_corridor"].as<bool>();c.envelope_soft_enabled=data["envelope_soft_enabled"].as<bool>();
  c.recovery_jerk_enabled=data["recovery_jerk_enabled"].as<bool>();
  c.acados_rti_steps=data["acados_rti_steps"]?data["acados_rti_steps"].as<int>():1;
  if(c.acados_rti_steps!=1&&c.acados_rti_steps!=2)throw std::runtime_error("native RTI maximum must be one or two");
  c.horizon=manifest["horizon"].as<int>();c.dt=manifest["dt"].as<double>();
  bool hundred_ms=std::abs(c.dt-.1)<1e-12,fifty_ms=std::abs(c.dt-.05)<1e-12;
  // Old 100 ms bundles may describe a near-multiple duration while their
  // generated model retains exactly five 20 ms steps. Use that physical mesh.
  if(hundred_ms)c.dt=.1;else if(fifty_ms)c.dt=.05;
  if(c.horizon<1||c.dt<=0.||!(hundred_ms||fifty_ms)||
     i.cost_scaling.size()!=size_t(c.horizon+1)||!finite(i.cost_scaling)||
     i.sample_count!=(hundred_ms?6:4)||
     i.nu!=(c.envelope_soft_enabled?4:3)||i.lower.size()!=size_t(i.nh)||i.upper.size()!=size_t(i.nh))
    throw std::runtime_error("artifact dimensions mismatch");
  auto reference=YAML::LoadFile((root/"reference.json").string());auto &r=i.reference;
  r.length_=reference["length"].as<double>();r.left_width_=reference["left_width"].as<double>();
  r.right_width_=reference["right_width"].as<double>();r.frame_=reference["frame_id"].as<std::string>();
  if(reference["metadata"]["map_sha256"])r.map_hash_=reference["metadata"]["map_sha256"].as<std::string>();
  r.closed_lap_=reference["metadata"]["closed_lap"]&&reference["metadata"]["closed_lap"].as<bool>();
  r.recording_verified_=r.closed_lap_&&reference["metadata"]["vehicle_geometry"].IsMap();
  r.knots_=values(reference["knots"]);r.coefficients_.resize(r.knots_.size()-1);
  if(!reference["speed_profile"]||reference["speed_profile"]["schema_version"].as<int>()!=1)
    throw std::runtime_error("frozen speed profile missing or mismatched");
  r.speed_positions_=values(reference["speed_profile"]["positions"]);r.speeds_=values(reference["speed_profile"]["speeds"]);
  if(r.speed_positions_.empty()||r.speed_positions_.size()!=r.speeds_.size()||
     r.speed_positions_.front()!=0.||r.speed_positions_.back()>=r.length_||
     !finite(r.speed_positions_)||!finite(r.speeds_)||
     std::any_of(r.speeds_.begin(),r.speeds_.end(),[&](double v){return v<0.||v>c.max_speed;}))
    throw std::runtime_error("invalid frozen speed profile");
  for(size_t j=0;j<r.coefficients_.size();++j)for(int k=0;k<6;++k)for(int q=0;q<2;++q)
    r.coefficients_[j][k][q]=reference["coefficients"][k][j][q].as<double>();
  i.library=dlopen((root/"libaims_mpcc_bundle.so").c_str(),RTLD_NOW|RTLD_LOCAL);
  if(!i.library)throw std::runtime_error(std::string("cannot load verified solver: ")+dlerror());
  auto abi=reinterpret_cast<int(*)()>(dlsym(i.library,"aims_rt_abi_version"));
  if(!abi||abi()!=1)throw std::runtime_error("unsupported native capsule ABI version");
#define SYMBOL(member,name) i.member=reinterpret_cast<decltype(i.member)>(dlsym(i.library,name));if(!i.member)throw std::runtime_error("missing capsule ABI symbol: " name)
  SYMBOL(create,"aims_rt_create");SYMBOL(free,"aims_rt_free");SYMBOL(reset,"aims_rt_reset");
  SYMBOL(set,"aims_rt_set");SYMBOL(initial,"aims_rt_initial");SYMBOL(parameters,"aims_rt_parameters");
  SYMBOL(solve,"aims_rt_solve");SYMBOL(get,"aims_rt_get");SYMBOL(eval,"aims_rt_eval");
  SYMBOL(cost,"aims_rt_cost");
#undef SYMBOL
  // dlopen may otherwise reuse a library with the same SONAME from a prior
  // bundle or LD_LIBRARY_PATH. Verify the files that actually supply symbols.
  const std::array<std::pair<const char*,const char*>,3> dependencies{{
    {"ocp_nlp_solve","libacados.so"},{"d_ocp_qp_ipm_solve","libhpipm.so"},{"blasfeo_dgemm_nn","libblasfeo.so"}}};
  for(const auto &entry:dependencies) {
    auto symbol=dlsym(i.library,entry.first);Dl_info location{};
    if(!symbol||!dladdr(symbol,&location))throw std::runtime_error("cannot verify loaded numerical dependency");
    YAML::Node expected;
    for(auto value:native["dependencies"])if(std::filesystem::path(value.first.as<std::string>()).filename()==entry.second)expected=value.second;
    if(!expected)throw std::runtime_error("dependency absent from native manifest");
    verify(location.dli_fname,expected);
  }
  Dl_info solver_location{};auto solver_symbol=dlsym(i.library,"aims_runtime_acados_create");
  if(!solver_symbol||!dladdr(solver_symbol,&solver_location))throw std::runtime_error("cannot verify loaded OCP solver");
  verify(solver_location.dli_fname,native["libraries"]["generated/libacados_ocp_solver_aims_runtime.so"]);
  return bundle;
}
const Config &Bundle::config() const{return impl_->cfg;}
const Reference &Bundle::reference() const{return impl_->reference;}
const std::string &Bundle::fingerprint() const{return impl_->fingerprint;}
std::vector<double> Bundle::evaluate(int kind,const InternalState &x,const std::vector<double> &u,const Parameters &p) const {
  if(kind<0||kind>6||u.size()!=size_t(impl_->nu))throw std::invalid_argument("generated model argument dimensions");
  std::vector<double> out(kind==0?9:kind==1?impl_->nh:kind==2?impl_->nh_e:kind==5?10+impl_->nh+6*impl_->sample_count:kind==6?1+impl_->nh_e:1);
  if(impl_->eval(kind,x.data(),u.data(),p.data(),out.data()))throw std::runtime_error("generated function evaluation failed");
  return out;
}

ReferencePoint Reference::at(double theta) const {
  if(!std::isfinite(theta))throw std::invalid_argument("finite reference progress required");
  double sw=wrap(theta,length_);
  size_t index=std::min(size_t(std::upper_bound(knots_.begin(),knots_.end(),sw)-knots_.begin()-1),coefficients_.size()-1);
  double h=sw-knots_[index];std::array<std::array<double,2>,3> result{};
  for(int derivative=0;derivative<3;++derivative)for(int q=0;q<2;++q) {
    double v=0.;for(int k=0;k<6-derivative;++k){int power=5-k;double factor=1.;for(int j=0;j<derivative;++j)factor*=power-j;v=v*h+factor*coefficients_[index][k][q];}
    result[derivative][q]=v;
  }
  double norm=std::hypot(result[1][0],result[1][1]);
  if(norm<1e-6)throw std::runtime_error("reference has zero tangent");
  return {result[0][0],result[0][1],std::atan2(result[1][1],result[1][0]),
    (result[1][0]*result[2][1]-result[1][1]*result[2][0])/std::pow(norm,3),norm};
}
double Reference::project(double x,double y) const {
  if(!std::isfinite(x)||!std::isfinite(y))throw std::invalid_argument("finite projection position required");
  auto distance=[&](double t){auto r=at(t);return std::pow(x-r.x,2)+std::pow(y-r.y,2);};
  int count=std::max(100,int(length_/.05));double step=length_/count,guess=0.,best=std::numeric_limits<double>::infinity();
  for(int j=0;j<count;++j){double d=distance(j*step);if(d<best){best=d;guess=j*step;}}
  double a=guess-step,b=guess+step;
  // Bounded golden search uses the exact polynomial, never a fitted spline.
  constexpr double ratio=.6180339887498948482;
  double c=b-ratio*(b-a),d=a+ratio*(b-a),fc=distance(c),fd=distance(d);
  for(int j=0;j<100&&b-a>1e-11;++j) {
    if(fc<fd){b=d;d=c;fd=fc;c=b-ratio*(b-a);fc=distance(c);}
    else{a=c;c=d;fc=fd;d=a+ratio*(b-a);fd=distance(d);}
  }
  return wrap((a+b)/2.,length_);
}
double Reference::speed_at(double theta) const {
  if(!std::isfinite(theta))throw std::invalid_argument("finite speed profile progress required");
  double s=wrap(theta,length_);
  size_t k=std::min(size_t(std::upper_bound(speed_positions_.begin(),speed_positions_.end(),s)-speed_positions_.begin()-1),speeds_.size()-1);
  double next=k+1==speeds_.size()?length_:speed_positions_[k+1];
  double next_speed=k+1==speeds_.size()?speeds_.front():speeds_[k+1];
  double fraction=(s-speed_positions_[k])/(next-speed_positions_[k]);
  return speeds_[k]+fraction*(next_speed-speeds_[k]);
}
std::vector<double> Reference::speed_refs(double theta,int horizon,double dt) const {
  if(horizon<1||!std::isfinite(dt)||dt<=0.)throw std::invalid_argument("invalid speed preview horizon/dt");
  std::vector<double> out;out.reserve(horizon+1);
  for(int k=0;k<=horizon;++k) {
    double speed=speed_at(theta);out.push_back(speed);
    double tangent=std::max(1e-9,at(theta).tangent_norm);
    double midpoint=theta+.5*speed*dt/tangent;
    theta+=speed_at(midpoint)*dt/std::max(1e-9,at(midpoint).tangent_norm);
  }
  return out;
}

Core::Core(Bundle bundle):bundle_(std::move(bundle)) {
  capsule_=bundle_.impl_->create();if(!capsule_)throw std::runtime_error("acados capsule creation failed");
}
Core::~Core(){bundle_.impl_->free(capsule_);}
void Core::reset(){bundle_.impl_->reset(capsule_);previous_={};previous_elapsed_=0.;have_progress_=false;}
Control Core::interpolate(const std::vector<Control> &controls,double dt,double elapsed) {
  if(controls.empty()||!std::isfinite(dt)||dt<=0.||!std::isfinite(elapsed)||elapsed<0.)throw std::invalid_argument("invalid warm interpolation");
  // Check the finite horizon before casting a potentially enormous quotient.
  if(elapsed>=dt*controls.size())return controls.back();
  double index=elapsed/dt;size_t k=std::min(size_t(std::floor(index)),controls.size()-1);
  double fraction=k==controls.size()-1?0.:index-k;Control out{};
  for(size_t j=0;j<out.size();++j)out[j]=controls[k][j]*(1.-fraction)+controls[std::min(k+1,controls.size()-1)][j]*fraction;
  return out;
}
State Core::transition(State x,const Control &u,double previous_endpoint,double span) const {
  if(!finite(x)||!finite(u)||!std::isfinite(previous_endpoint)||!std::isfinite(span)||span<0.||span>config().dt+1e-12)
    throw std::invalid_argument("invalid forecasting transition");
  if(span==0.)return x;
  const auto &c=config();auto rhs=[&](const State &s,double command){return State{
    s[3]*std::cos(s[2]),s[3]*std::sin(s[2]),s[3]*std::tan(s[5])/(c.wheelbase*(1+c.understeer_coefficient*s[3]*s[3])),
    u[0],u[2],(command-s[5])/c.steering_tau};};
  for(double t=0.;t<span-1e-12;t+=.02) {
    double h=std::min(.02,span-t),command=previous_endpoint+(u[1]-previous_endpoint)*(t+h)/span;
    auto k1=rhs(x,command);State v{};
    for(int j=0;j<6;++j)v[j]=x[j]+.5*h*k1[j];
    auto k2=rhs(v,command);
    for(int j=0;j<6;++j)v[j]=x[j]+.5*h*k2[j];
    auto k3=rhs(v,command);
    for(int j=0;j<6;++j)v[j]=x[j]+h*k3[j];
    auto k4=rhs(v,command);
    for(int j=0;j<6;++j)x[j]+=h*(k1[j]+2*k2[j]+2*k3[j]+k4[j])/6.;
  }
  return x;
}

Plan Core::solve(State initial,const Applied &applied,const Alignment &alignment,double elapsed,
                 double budget,bool second,double source_epoch,double forecast_epoch,const std::vector<double> &targets,bool refresh_second_geometry) {
  auto started=Clock::now();Plan plan;plan.dt=config().dt;plan.source_epoch=source_epoch;plan.forecast_epoch=forecast_epoch;
  plan.artifact_fingerprint=bundle_.fingerprint();plan.speed_targets=targets;plan.map_alignment=alignment;plan.initial_applied=applied;
  if(!finite(initial)||!finite(applied)||!finite(alignment)||!std::isfinite(elapsed)||elapsed<=0.||
     !std::isfinite(budget)||budget<=0.||!std::isfinite(source_epoch)||!std::isfinite(forecast_epoch))
    throw std::invalid_argument("finite solve inputs and positive elapsed/budget required");
  const auto &cfg=config();const auto &ref=reference();auto &native=*bundle_.impl_;
  if(ref.frame_id()=="odom"&&alignment!=Alignment{})throw std::invalid_argument("odom reference must not receive alignment");
  double cosine=std::cos(alignment[2]),sine=std::sin(alignment[2]);
  double mapx=cosine*initial[0]-sine*initial[1]+alignment[0];
  double mapy=sine*initial[0]+cosine*initial[1]+alignment[1];
  initial[4]=ref.project(mapx,mapy);initial[3]=std::max(0.,initial[3]);
  if(have_progress_) {initial[4]=previous_theta_+std::remainder(initial[4]-previous_theta_,ref.length());initial[2]=previous_yaw_+std::remainder(initial[2]-previous_yaw_,2*pi);}
  auto speed_refs=ref.speed_refs(initial[4],cfg.horizon,cfg.dt);
  if(!targets.empty()) {
    if(targets.size()!=size_t(cfg.horizon+1)||!finite(targets)||
       std::any_of(targets.begin(),targets.end(),[&](double v){return v<0.||v>cfg.max_speed;}))
      throw std::invalid_argument("speed targets must be N+1 finite values within speed bounds");
    for(size_t k=0;k<targets.size();++k)speed_refs[k]=std::min(speed_refs[k],targets[k]);
  }
  if(previous_.success)previous_elapsed_+=elapsed;
  auto x0=internal(initial,applied);std::vector<InternalState> seed{x0};std::vector<std::vector<double>> inputs;
  const bool v2=cfg.command_profile=="rate_bounded_v2";
  double jerk=cfg.jerk_limit;
  if(!v2&&cfg.envelope_soft_enabled&&cfg.recovery_jerk_enabled) {
    double lo=std::max(-cfg.brake_limit,applied[0]-cfg.jerk_limit*cfg.dt);
    double hi=std::min(cfg.accel_limit,applied[0]+cfg.jerk_limit*cfg.dt);
    if(lo<=hi&&utilization(initial,std::clamp(0.,lo,hi),cfg)>1.+1e-8)jerk=cfg.recovery_jerk_limit;
  }
  for(int k=0;k<cfg.horizon;++k) {
    auto x=seed.back();auto r=ref.at(x[4]);double a=(speed_refs[k+1]-x[3])/cfg.dt;
    double steer=std::atan(cfg.wheelbase*(1.+cfg.understeer_coefficient*x[3]*x[3])*r.curvature);
    if(previous_.success&&previous_elapsed_<cfg.horizon*cfg.dt) {
      auto shifted=interpolate(previous_.controls,cfg.dt,previous_elapsed_+k*cfg.dt);a=shifted[0];steer=shifted[1];
    }
    double bound=k*cfg.dt<cfg.envelope_recovery_time-1e-10?jerk:cfg.jerk_limit;
    double acceleration_lower=v2?-cfg.brake_limit:std::max(-cfg.brake_limit,x[6]-bound*cfg.dt);
    double acceleration_upper=v2?cfg.accel_limit:std::min(cfg.accel_limit,x[6]+bound*cfg.dt);
    if(acceleration_lower>acceleration_upper) {
      plan.reason="applied acceleration has no bounded continuation";plan.solve_time_s=duration(started);return plan;
    }
    a=std::clamp(a,acceleration_lower,acceleration_upper);
    double left=cfg.steer_limit+x[7],right=cfg.steer_limit-x[7];
    double lower=v2?std::max(-cfg.steer_rate,-left/cfg.dt):std::max({-cfg.steer_rate,x[8]-cfg.steer_acceleration*cfg.dt,-left/cfg.dt,-stopping_rate(left,cfg.steer_acceleration,cfg.dt)});
    double upper=v2?std::min(cfg.steer_rate,right/cfg.dt):std::min({cfg.steer_rate,x[8]+cfg.steer_acceleration*cfg.dt,right/cfg.dt,stopping_rate(right,cfg.steer_acceleration,cfg.dt)});
    if(lower>upper+1e-12){plan.reason="applied steering has no bounded continuation";plan.solve_time_s=duration(started);return plan;}
    // Roundoff can invert a zero-width rate interval by a few ulps. Keep the
    // hard endpoint bound, collapsing only inversions within the tolerance
    // already checked above, so std::clamp always receives lower<=upper.
    if(lower>upper)lower=upper;
    double rate=std::clamp((std::clamp(steer,-cfg.steer_limit,cfg.steer_limit)-x[7])/cfg.dt,lower,upper);
    std::vector<double> u{a,x[7]+rate*cfg.dt,std::clamp((x[3]+.5*a*cfg.dt)/r.tangent_norm,0.,cfg.max_speed)};
    if(cfg.envelope_soft_enabled)u.push_back(0.);
    auto next=bundle_.evaluate(0,x,u,{});InternalState nextx{};std::copy(next.begin(),next.end(),nextx.begin());
    seed.push_back(nextx);inputs.push_back(u);
  }
  std::vector<Parameters> parameters;
  for(int k=0;k<=cfg.horizon;++k) {
    auto r=ref.at(seed[k][4]);double dx=r.x-alignment[0],dy=r.y-alignment[1];
    double cap=cfg.envelope_soft_enabled?std::max(0.,cfg.envelope_recovery_time-k*cfg.dt):0.;
    Parameters p{cosine*dx+sine*dy,-sine*dx+cosine*dy,r.yaw-alignment[2],r.curvature,
                 seed[k][4],r.tangent_norm,speed_refs[k],k*cfg.dt<cfg.envelope_recovery_time-1e-10?jerk:cfg.jerk_limit,cap,k==0?1.:0.};
    parameters.push_back(p);native.parameters(capsule_,k,p.data());native.set(capsule_,k,"x",seed[k].data());
    if(k<cfg.horizon)native.set(capsule_,k,"u",inputs[k].data());
  }
  native.initial(capsule_,x0.data());
  plan.preparation_time_s=duration(started);
  // Each candidate is propagated exactly once through the generated nonlinear
  // model; validation consumes the resulting samples and unchanged controls.
  for(int pass=0;pass<(second?cfg.acados_rti_steps:1);++pass) {
    if(duration(started)>=budget){plan.reason="solve budget exhausted before RTI";break;}
    auto optimized=Clock::now();plan.status=native.solve(capsule_);
    double pass_native_duration=duration(optimized);plan.native_time_s+=pass_native_duration;++plan.native_passes;
    auto validation_started=Clock::now();
    plan.raw_optimizer_cost=0.;plan.native_cost=native.cost(capsule_);
    std::vector<std::vector<double>> native_controls;
    std::vector<InternalState> native_states;
    for(int k=0;k<cfg.horizon;++k) {
      std::vector<double> u(native.nu);native.get(capsule_,k,"u",u.data());
      InternalState raw{};native.get(capsule_,k,"x",raw.data());native_states.push_back(raw);
      plan.max_geometry_progress_shift=std::max(plan.max_geometry_progress_shift,std::abs(raw[4]-parameters[k][4]));
      plan.raw_optimizer_cost+=native.cost_scaling[k]*bundle_.evaluate(3,raw,u,parameters[k])[0];
      native_controls.push_back(std::move(u));
    }
    InternalState raw_terminal{};native.get(capsule_,cfg.horizon,"x",raw_terminal.data());
    native_states.push_back(raw_terminal);
    plan.max_geometry_progress_shift=std::max(plan.max_geometry_progress_shift,std::abs(raw_terminal[4]-parameters.back()[4]));
    std::vector<double> terminal_u(native.nu,0.);
    plan.raw_optimizer_cost+=native.cost_scaling.back()*bundle_.evaluate(4,raw_terminal,terminal_u,parameters.back())[0];
    plan=validate_candidate(initial,applied,alignment,native_controls,parameters,std::move(plan));
    bool all_finite=std::isfinite(plan.max_violation);
    if(duration(started)>=budget){plan.success=false;plan.reason="solve budget exhausted";}
    plan.validation_time_s+=duration(validation_started);
    if(plan.success||plan.status!=0||!all_finite||duration(started)>=budget)break;
    double estimate=pass_native_duration*1.25+duration(validation_started);
    if(pass==0&&second&&budget-duration(started)<estimate) {
      plan.reason="remaining budget cannot cover estimated second RTI";break;
    }
    if(pass==0&&second&&cfg.acados_rti_steps==2&&refresh_second_geometry){
      // Geometry remains frozen WITHIN each RTI pass. A corrective pass uses
      // the first native iterate's progress, retaining its states and controls.
      // The same parameter vector is consumed by the second optimizer and
      // final exact nonlinear candidate validation/objective evaluation.
      auto refresh_started=Clock::now();
      if(!std::all_of(native_states.begin(),native_states.end(),[](const auto& x){return finite(x);})){
        plan.success=false;plan.reason="nonfinite native geometry iterate";break;
      }
      for(int k=0;k<=cfg.horizon;++k){
        const double theta=k?native_states[k][4]:initial[4];
        auto r=ref.at(theta);double dx=r.x-alignment[0],dy=r.y-alignment[1];
        auto &p=parameters[k];
        p[0]=cosine*dx+sine*dy;p[1]=-sine*dx+cosine*dy;
        p[2]=r.yaw-alignment[2];p[3]=r.curvature;p[4]=theta;p[5]=r.tangent_norm;
        native.parameters(capsule_,k,p.data());
      }
      native.initial(capsule_,x0.data()); // preserve the immutable measurement
      ++plan.geometry_refreshes;plan.geometry_refresh_time_s+=duration(refresh_started);
    }
  }
  plan.solve_time_s=duration(started);
  if(plan.success){previous_=plan;previous_elapsed_=0.;have_progress_=true;previous_theta_=initial[4];previous_yaw_=initial[2];}
  return plan;
}

Plan Core::validate_candidate(const State &initial,const Applied &applied,const Alignment &alignment,
                              const std::vector<std::vector<double>> &controls,
                              const std::vector<Parameters> &parameters,Plan plan) const {
  const auto &cfg=config();const auto &ref=reference();const auto &native=*bundle_.impl_;
  double cosine=std::cos(alignment[2]),sine=std::sin(alignment[2]);
  plan.map_alignment=alignment;plan.initial_applied=applied;
  plan.states.clear();plan.controls.clear();plan.states.push_back(initial);plan.max_violation=0.;plan.constraint_violations.clear();plan.cost=0.;InternalState x=internal(initial,applied);
  bool all_finite=true;
  auto violation=[&](double value,double lower,double upper,const std::string &group){
    if(!std::isfinite(value)){all_finite=false;plan.constraint_violations[group]=std::numeric_limits<double>::infinity();return;}
    const double error=std::max({0.,lower-value,value-upper});
    plan.constraint_violations[group]=std::max(plan.constraint_violations[group],error);
    plan.max_violation=std::max(plan.max_violation,error);
  };
  for(int k=0;k<cfg.horizon;++k) {
    const auto &u=controls[k];
    all_finite=all_finite&&finite(u);plan.controls.push_back({u[0],u[1],u[2]});
    for(int j=0;j<native.nu;++j)violation(u[j],native.input_lower[j],native.input_upper[j],"input_bounds");
    violation(x[3],0.,cfg.max_speed,"speed_bounds");violation(x[5],-cfg.steer_limit,cfg.steer_limit,"steering_bounds");
    auto candidate=bundle_.evaluate(5,x,u,parameters[k]);
    for(int j=0;j<native.nh;++j)violation(candidate[9+j],native.lower[j],native.upper[j],native.constraint_groups[j]);
    plan.cost+=native.cost_scaling[k]*candidate[9+native.nh];
    for(int j=0;j<native.sample_count;++j) {
      State sample{};std::copy_n(candidate.begin()+10+native.nh+6*j,6,sample.begin());
      if(!finite(sample)){all_finite=false;continue;}
      violation(sample[3],0.,cfg.max_speed,"speed_bounds");violation(sample[5],-cfg.steer_limit,cfg.steer_limit,"steering_bounds");
      if(cfg.enforce_corridor) {
        auto r=ref.at(sample[4]);double yaw=sample[2]+alignment[2];
        double px=cosine*sample[0]-sine*sample[1]+alignment[0];
        double py=sine*sample[0]+cosine*sample[1]+alignment[1];
        for(double along:{cfg.front_extent,-cfg.rear_extent})for(double across:{-cfg.half_width,cfg.half_width}) {
          double dx=px+along*std::cos(yaw)-across*std::sin(yaw)-r.x;
          double dy=py+along*std::sin(yaw)+across*std::cos(yaw)-r.y;
          violation(-std::sin(r.yaw)*dx+std::cos(r.yaw)*dy,-ref.right_width(),ref.left_width(),"corridor");
        }
      }
    }
    std::copy_n(candidate.begin(),9,x.begin());
    all_finite=all_finite&&finite(x);plan.states.push_back(physical(x));
  }
  std::vector<double> terminal_u(native.nu,0.);
  auto terminal=bundle_.evaluate(6,x,terminal_u,parameters.back());
  for(int j=0;j<native.nh_e;++j)violation(terminal[j],native.terminal_lower[j],native.terminal_upper[j],j?"terminal_corridor":"terminal_operating_envelope");
  violation(x[3],0.,cfg.max_speed,"speed_bounds");violation(x[5],-cfg.steer_limit,cfg.steer_limit,"steering_bounds");
  plan.cost+=native.cost_scaling.back()*terminal.back();
  all_finite=all_finite&&std::isfinite(plan.cost)&&std::isfinite(plan.native_cost)&&std::isfinite(plan.raw_optimizer_cost);
  if(!all_finite)plan.max_violation=std::numeric_limits<double>::infinity();
  plan.success=plan.status==0&&all_finite&&plan.max_violation<1e-4;
  plan.reason=plan.success?"accepted":"nonlinear candidate rejected";
  return plan;
}
Plan Core::reanchor(const Plan &source,State actual,const Applied &applied,double epoch,
                    const std::optional<Alignment> &current_alignment) const {
  auto started=Clock::now();Plan plan=source;plan.success=false;plan.reanchor_time_s=0.;plan.prefix_transported=false;
  const auto &cfg=config();const auto &ref=reference();Alignment alignment=current_alignment.value_or(source.map_alignment);
  if(!source.success||source.status!=0||source.controls.size()!=size_t(cfg.horizon)||
     source.states.empty()||source.dt!=cfg.dt||source.artifact_fingerprint!=bundle_.fingerprint()) {
    plan.reason="source plan is not eligible for reanchor";return plan;
  }
  if(cfg.envelope_soft_enabled){plan.reason="soft recovery requires independent comparator";return plan;}
  if(!finite(actual)||!finite(applied)||!finite(alignment)||!std::isfinite(epoch)||epoch<source.source_epoch)
    throw std::invalid_argument("finite reanchor inputs and actual epoch required");
  if(ref.frame_id()=="odom"&&alignment!=Alignment{})throw std::invalid_argument("odom reanchor must not receive alignment");
  double cosine=std::cos(alignment[2]),sine=std::sin(alignment[2]);
  double theta=ref.project(cosine*actual[0]-sine*actual[1]+alignment[0],sine*actual[0]+cosine*actual[1]+alignment[1]);
  actual[4]=source.states.front()[4]+std::remainder(theta-source.states.front()[4],ref.length());
  actual[2]=source.states.front()[2]+std::remainder(actual[2]-source.states.front()[2],2*pi);
  auto speeds=ref.speed_refs(actual[4],cfg.horizon,cfg.dt);
  if(!source.speed_targets.empty()) {
    if(source.speed_targets.size()!=speeds.size()||!finite(source.speed_targets))throw std::invalid_argument("invalid source speed targets");
    for(size_t k=0;k<speeds.size();++k)speeds[k]=std::min(speeds[k],source.speed_targets[k]);
  }
  std::vector<std::vector<double>> controls;
  for(const auto &u:source.controls)controls.emplace_back(u.begin(),u.end());
  std::vector<Parameters> parameters;theta=actual[4];
  for(int k=0;k<=cfg.horizon;++k) {
    auto r=ref.at(theta);double dx=r.x-alignment[0],dy=r.y-alignment[1];
    parameters.push_back({cosine*dx+sine*dy,-sine*dx+cosine*dy,r.yaw-alignment[2],r.curvature,
                          theta,r.tangent_norm,speeds[k],cfg.jerk_limit,0.,k==0?1.:0.});
    if(k<cfg.horizon)theta+=controls[k][2]*cfg.dt;
  }
  plan.forecast_epoch=epoch;
  plan=validate_candidate(actual,applied,alignment,controls,parameters,std::move(plan));
  if(!plan.success) {
    // The original endpoints were optimized relative to the forecast applied
    // prefix. A delayed old output can change the first rate and, consequently,
    // the second interval's rate change. Try transporting that endpoint
    // schedule only for a small, physically bounded change of prefix. The
    // extra slot accounts for one held 50 Hz output command at the forecast.
    const double delay=epoch-source.forecast_epoch,span=delay+.02;
    const double da=applied[0]-source.initial_applied[0];
    const double ds=applied[1]-source.initial_applied[1];
    const double dr=applied[2]-source.initial_applied[2];
    const bool v2=cfg.command_profile=="rate_bounded_v2";
    bool eligible=std::isfinite(delay)&&delay>=0.&&delay<=.05&&finite(source.initial_applied)&&
      std::abs(ds)<=cfg.steer_rate*span&&
      (v2||(std::abs(da)<=cfg.jerk_limit*span&&std::abs(dr)<=cfg.steer_acceleration*span));
    for(const auto &u:controls)eligible=eligible&&finite(u);
    if(!eligible)plan.reason+="; prefix transport ineligible";
    else {
      auto transported=controls;
      double endpoint=applied[1],rate=applied[2];bool continuation=true;
      for(auto &u:transported) {
        u[0]=std::clamp(u[0]+da,-cfg.brake_limit,cfg.accel_limit);
        double left=cfg.steer_limit+endpoint,right=cfg.steer_limit-endpoint;
        double lower=v2?std::max(-cfg.steer_rate,-left/cfg.dt):std::max({-cfg.steer_rate,rate-cfg.steer_acceleration*cfg.dt,
                              -left/cfg.dt,-stopping_rate(left,cfg.steer_acceleration,cfg.dt)});
        double upper=v2?std::min(cfg.steer_rate,right/cfg.dt):std::min({cfg.steer_rate,rate+cfg.steer_acceleration*cfg.dt,
                              right/cfg.dt,stopping_rate(right,cfg.steer_acceleration,cfg.dt)});
        if(lower>upper+1e-12){continuation=false;break;}
        if(lower>upper)lower=upper;
        rate=std::clamp((std::clamp(u[1]+ds,-cfg.steer_limit,cfg.steer_limit)-endpoint)/cfg.dt,lower,upper);
        endpoint+=rate*cfg.dt;u[1]=endpoint;
      }
      if(!continuation)plan.reason+="; prefix transport has no bounded steering continuation";
      else {
        // Progress controls are unchanged, so the refreshed geometric stage
        // parameters also apply to this distinct nonlinear candidate.
        auto candidate=validate_candidate(actual,applied,alignment,transported,parameters,plan);
        if(candidate.success) {
          plan=std::move(candidate);plan.prefix_transported=true;
          plan.reason="accepted with bounded prefix transport";
        } else {
          // Keep the original failed certificate and its maximum violation.
          // An alternate candidate never hides why the original was rejected.
          plan.reason+="; prefix transport rejected: "+candidate.reason;
        }
      }
    }
  }
  plan.reanchor_time_s=duration(started);
  return plan;
}
}  // namespace aims_mpcc_rt

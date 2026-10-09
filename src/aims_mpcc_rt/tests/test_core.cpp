#include "aims_mpcc_rt/core.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <limits>
#include <iomanip>

using namespace aims_mpcc_rt;
void check(bool value, const char *message) {if(!value) throw std::runtime_error(message);}
void near(double a,double b,double tolerance,const char *message) {
  if(!std::isfinite(a)||!std::isfinite(b)||std::abs(a-b)>tolerance) {std::cerr<<a<<" != "<<b<<"\n";throw std::runtime_error(message);}
}
State independent_plant(State x,const Control &u,double previous,double span,const Config &cfg) {
  // Independent 2 ms midpoint integrator with 10% slower steering lag.
  for(double t=0.;t<span-1e-12;t+=.002) {
    double h=std::min(.002,span-t);
    double hold=std::min(span,(std::floor((t+1e-10)/.02)+1)*.02);
    double command=previous+(u[1]-previous)*hold/span;
    auto rhs=[&](const State &s){return State{s[3]*std::cos(s[2]),s[3]*std::sin(s[2]),
      s[3]*std::tan(s[5])/(cfg.wheelbase*(1+cfg.understeer_coefficient*s[3]*s[3])),u[0],u[2],
      (command-s[5])/(1.1*cfg.steering_tau)};};
    auto k=rhs(x);State mid{};for(int j=0;j<6;++j)mid[j]=x[j]+h*.5*k[j];
    k=rhs(mid);for(int j=0;j<6;++j)x[j]+=h*k[j];
  }
  return x;
}
int main(int argc,char **argv) {
  bool rejected_nan=false;
  try {near(std::numeric_limits<double>::quiet_NaN(),0.,1e-9,"NaN oracle");}
  catch(const std::runtime_error&){rejected_nan=true;}
  check(rejected_nan,"numerical oracle must reject NaN");
  if(argc==2&&std::string(argv[1])=="--oracle-only")return 0;
  check(argc==2||argc==3,"bundle argument required");
  const auto b=Bundle::load(argv[1]);
  auto fixtures=YAML::LoadFile(std::string(argv[1])+"/parity.json");
  for(auto row:fixtures["reference"]) {
    check(bool(row["speed"]),"frozen SpeedPlanner parity fixtures missing");
    auto actual=b.reference().at(row["theta"].as<double>());
    near(b.reference().speed_at(row["theta"].as<double>()),row["speed"].as<double>(),1e-12,"periodic SpeedPlanner parity");
    auto speeds=b.reference().speed_refs(row["theta"].as<double>(),b.config().horizon,b.config().dt);
    for(size_t k=0;k<speeds.size();++k)near(speeds[k],row["speed_refs"][k].as<double>(),1e-12,"physical tangent midpoint speed preview parity");
    near(actual.x,row["x"].as<double>(),2e-12,"quintic x parity");
    near(actual.y,row["y"].as<double>(),2e-12,"quintic y parity");
    near(actual.yaw,row["yaw"].as<double>(),2e-12,"quintic yaw parity");
    near(actual.curvature,row["curvature"].as<double>(),2e-12,"quintic curvature parity");
    auto projected=b.reference().project(actual.x,actual.y);
    double wrapped=std::fmod(row["theta"].as<double>(),b.reference().length());
    if(wrapped<0)wrapped+=b.reference().length();
    auto error=std::remainder(projected-wrapped,b.reference().length());
    near(error,0.,2e-7,"projection parity");
  }
  Core core(b);
  for(auto row:fixtures["model"]) {
    InternalState x{};Parameters p{};std::vector<double> u;
    for(size_t i=0;i<x.size();++i)x[i]=row["state"][i].as<double>();
    for(size_t i=0;i<p.size();++i)p[i]=row["parameters"][i].as<double>();
    for(auto v:row["control"])u.push_back(v.as<double>());
    auto transition=b.evaluate(0,x,u,p);
    for(size_t i=0;i<x.size();++i)near(transition[i],row["transition"][i].as<double>(),1e-12,"generated transition parity");
    auto constraints=b.evaluate(1,x,u,p);
    for(size_t i=0;i<constraints.size();++i)near(constraints[i],row["constraints"][i].as<double>(),1e-12,"constraint parity");
    near(b.evaluate(3,x,u,p)[0],row["cost"].as<double>(),1e-10,"nonlinear cost parity");
    near(b.evaluate(3,x,u,p)[0],row["analytic_stage_cost"].as<double>(),1e-8,"legacy frozen stage objective equation parity");
    check(bool(row["terminal_constraints"]),"terminal constraint fixtures missing");
    auto terminal=b.evaluate(2,x,u,p);
    for(size_t i=0;i<terminal.size();++i)near(terminal[i],row["terminal_constraints"][i].as<double>(),1e-12,"terminal constraint parity");
    near(b.evaluate(4,x,u,p)[0],row["terminal_cost"].as<double>(),1e-10,"terminal cost parity");
    near(b.evaluate(4,x,u,p)[0],row["analytic_terminal_cost"].as<double>(),1e-8,"legacy frozen terminal objective equation parity");
    auto candidate=b.evaluate(5,x,u,p);
    check(candidate.size()==10+constraints.size()+6*(int(std::ceil(b.config().dt/.02-1e-12))+1),
          "single forward candidate must expose all integration samples");
    for(size_t i=0;i<x.size();++i)near(candidate[i],transition[i],1e-12,"single-pass candidate dynamics parity");
    for(size_t i=0;i<constraints.size();++i)near(candidate[9+i],constraints[i],1e-12,"single-pass candidate constraints parity");
    near(candidate[9+constraints.size()],row["cost"].as<double>(),1e-10,"single-pass candidate cost parity");
    for(size_t i=0;i<row["samples"].size();++i)for(int j=0;j<6;++j)
      near(candidate[10+constraints.size()+i*6+j],row["samples"][i][j].as<double>(),1e-12,"shared nonlinear integration sample parity");
    State s{};for(size_t i=0;i<s.size();++i)s[i]=x[i];
    Control c{u[0],u[1],u[2]};auto forecast=core.transition(s,c,x[7],b.config().dt);
    for(size_t i=0;i<s.size();++i)near(forecast[i],transition[i],1e-12,"forecast RK4 parity");
  }
  std::vector<Control> c{{0.,0.,0.},{1.,.2,.5},{2.,.4,1.}};
  near(Core::interpolate(c,.1,.025)[0],.25,1e-12,"40 Hz fractional shift");
  near(Core::interpolate(c,.1,.05)[0],.5,1e-12,"20 Hz fractional shift");
  near(Core::interpolate(c,.1,.15)[0],1.5,1e-12,"elapsed shift accumulation");
  near(Core::interpolate(c,.1,.4)[0],2.,1e-12,"tail held");
  near(Core::interpolate(c,.1,1e300)[0],2.,1e-12,"huge finite elapsed holds tail without integer overflow");
  if(argc==3&&std::string(argv[2])=="--parity-only") {
    std::cout<<"PASS: model/objective/constraint/quintic/periodic speed-profile and midpoint preview parity\n";
    return 0;
  }
  auto r0=b.reference().at(0.);State stopped{r0.x,r0.y,r0.yaw,0.,0.,0.};
  auto near_bound=stopped;near_bound[5]=b.config().steer_limit;
  auto rounded_prefix=core.solve(near_bound,{0.,b.config().steer_limit,b.config().steer_acceleration*b.config().dt+1.1e-16}, {},.05);
  check(rounded_prefix.reason!="applied steering has no bounded continuation", "roundoff steering interval canonicalized before clamp");
  core.reset();
  auto impossible=core.solve(stopped,{10.,0.,0.}, {},.05);
  if(b.config().command_profile=="legacy_bounded_v1")
    check(!impossible.success&&impossible.native_passes==0,"impossible applied acceleration rejected before optimizer");
  else check(impossible.native_passes>0,"v2 seed does not constrain the removed acceleration prefix jerk");
  core.reset();
  auto stationary=core.solve(stopped,{}, {},.05,.05,false,0.,0.,std::vector<double>(b.config().horizon+1,0.));
  check(stationary.success,"zero speed targets candidate feasible");
  near(stationary.states.back()[3],0.,1e-4,"zero speed targets preserve standstill within solver feasibility tolerance");
  core.reset();
  auto ref=b.reference().at(0.);
  State initial{ref.x,ref.y,ref.yaw,0.,0.,0.};Applied applied{};
  double execution_step=.5*b.config().dt;
  std::vector<double> timings;double sum_error=0.,peak_error=0.,progress=0.,last_theta=0.;int passes=0;
  int count=int(std::ceil((b.reference().length()/b.config().cruise_speed+5.)/execution_step));
  for(int i=0;i<count;++i) {
    auto plan=core.solve(initial,applied,{},execution_step,.05,true);
    if(!plan.success){
      std::cerr<<std::setprecision(17)<<"cycle="<<i<<" status="<<plan.status<<" passes="<<plan.native_passes
        <<" violation="<<plan.max_violation<<" reason="<<plan.reason<<"\nstate:";
      for(double v:initial)std::cerr<<" "<<v;
      std::cerr<<"\nprefix:";for(double v:applied)std::cerr<<" "<<v;std::cerr<<"\n";
    }
    check(plan.success,"synthetic circle candidate rejected");
    near(plan.native_cost,plan.raw_optimizer_cost,1e-7,"actual native objective scaling parity");
    check(plan.native_passes<=b.config().acados_rti_steps,"declared RTI pass maximum");
    check(plan.max_violation<1e-4,"nonlinear candidate feasible");
    auto u=plan.controls.front();
    double previous_endpoint=applied[1];
    // Execute half an OCP stage with the exact prefix steering endpoint.
    u[1]=previous_endpoint+(u[1]-previous_endpoint)*.5;
    initial=independent_plant(initial,u,previous_endpoint,execution_step,b.config());
    applied={u[0],u[1],(u[1]-previous_endpoint)/execution_step};
    timings.push_back(plan.solve_time_s);passes+=plan.native_passes;
    auto theta=b.reference().project(initial[0],initial[1]);
    progress+=std::remainder(theta-last_theta,b.reference().length());last_theta=theta;
    auto closest=b.reference().at(theta);
    double error=std::hypot(initial[0]-closest.x,initial[1]-closest.y);
    sum_error+=error*error;peak_error=std::max(peak_error,error);
  }
  check(initial[3]>.4,"closed loop accelerates toward cruise");
  check(progress>b.reference().length(),"independent plant completes lap");
  check(std::sqrt(sum_error/count)<.1,"independent plant tracks circle");
  std::sort(timings.begin(),timings.end());
  std::cout<<"PASS: exact quintic/model/cost/constraint parity, fractional warm shift, independent lagged plant\n"
           <<"N="<<b.config().horizon<<" dt="<<b.config().dt<<" execution_step="<<execution_step
           <<" cruise="<<b.config().cruise_speed<<" final_speed="<<initial[3]<<" rms_error="<<std::sqrt(sum_error/count)
           <<" peak_error="<<peak_error<<" lap_progress="<<progress/b.reference().length()<<" RTI_passes="<<passes
           <<" p50_ms="<<timings[count/2]*1000<<" p95_ms="<<timings[int(count*.95)]*1000
           <<" max_ms="<<timings.back()*1000<<"\n";
}

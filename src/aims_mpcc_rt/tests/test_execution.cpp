#include "aims_mpcc_rt/execution.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace aims_mpcc_rt;
static void require(bool value,const char* reason){if(!value)throw std::runtime_error(reason);}
int main(int argc,char** argv){
  require(argc==2,"bundle required");auto bundle=Bundle::load(argv[1]);const auto& c=bundle.config();
  Core core(bundle);const auto r=bundle.reference().at(0.);const double now=10.02;
  State initial{r.x,r.y,r.yaw,.79,0.,.4};Applied applied{.4,.4,0.};
  Plan source;source.success=true;source.status=0;source.dt=c.dt;source.source_epoch=now-.02;
  source.forecast_epoch=now;source.artifact_fingerprint=bundle.fingerprint();source.initial_applied=applied;
  source.states.push_back(initial);
  for(int k=0;k<c.horizon;++k){Control u{std::max(0.,.3-.1*k),.4,.79};source.controls.push_back(u);
    source.states.push_back(core.transition(source.states.back(),u,.4,c.dt));}
  auto candidate=core.reanchor(source,initial,applied,now,Alignment{});
  require(candidate.success,"counterexample macro candidate must be accepted");
  OutputSampler sampler(c);Plan prime=source;prime.forecast_epoch=now-.42;prime.source_epoch=now-.44;
  for(auto& u:prime.controls)u[0]=.4;
  sampler.reset(.706,.4,now-.42);sampler.set_previous_endpoint(.4);
  for(int i=1;i<=20;++i)sampler.sample(&prime,now-.42+.02*i,initial,false,100.);
  if(c.command_profile=="rate_bounded_v2")sampler.reset(.79,.4,now-.02,.4,0.);
  const double before=sampler.continuous_speed();
  auto certificate=certify_execution(candidate,sampler,initial,Alignment{},c,bundle.reference(),now,100.);
  if(c.command_profile=="rate_bounded_v2")
    require(certificate.success&&certificate.max_utilization<=1.+1e-4,
      "v2 immediate retained-bound continuation must certify this counterexample");
  else{
    require(!certificate.success,"macro-feasible but executed-infeasible plan incorrectly certified");
    require(certificate.max_utilization>1.,"counterexample must expose physical utilization");
  }
  require(sampler.continuous_speed()==before,"certificate mutated live output sampler");
  if(c.command_profile=="rate_bounded_v2"){
    // A later binding cap must not convert a safe, certified continuation into
    // full braking when only a conservative future proxy is infeasible.
    State moving{r.x,r.y,r.yaw,1.05,0.,.3};
    Plan cap_plan=source;cap_plan.source_epoch=.92;cap_plan.forecast_epoch=.92;
    cap_plan.initial_applied={0.,.25,0.};
    cap_plan.controls.assign(c.horizon,Control{0.,.15,1.05});cap_plan.controls[0][1]=.35;
    OutputSampler cap_sampler(c);cap_sampler.reset(1.05,.33,.98);
    auto nominal=certify_execution(cap_plan,cap_sampler,moving,Alignment{},c,bundle.reference(),1.,.8);
    auto capped=certify_execution(cap_plan,cap_sampler,moving,Alignment{},c,bundle.reference(),1.,.8,.9);
    require(nominal.success&&capped.success,"later finish cap violated a safe certified continuation");
    auto first=cap_sampler.sample(&cap_plan,1.,moving,false,.8,.9);
    require(cap_sampler.braking_budget().current_feasible&&!cap_sampler.braking_budget().feasible&&
      std::abs(first.continuous_acceleration)<1e-10,"infeasible future proxy must not bypass current-state limits");
    cap_plan.controls[0][1]=.4;cap_sampler.reset(1.05,.33,.98);
    require(!certify_execution(cap_plan,cap_sampler,moving,Alignment{},c,bundle.reference(),1.,.8).success&&
      !certify_execution(cap_plan,cap_sampler,moving,Alignment{},c,bundle.reference(),1.,.8,.9).success,
      "genuinely unsafe held steering must remain rejected with and without a cap");
  }
  auto safe=source;safe.controls.assign(c.horizon,Control{0.,0.,.2});safe.initial_applied={0.,0.,0.};
  State safe_initial{r.x,r.y,r.yaw,.2,0.,0.};OutputSampler safe_sampler(c);safe_sampler.reset(.2,0.,now-.02);
  safe.states.assign(c.horizon+1,safe_initial);safe.forecast_epoch=now;
  auto ok=certify_execution(safe,safe_sampler,safe_initial,Alignment{},c,bundle.reference(),now,.8);
  require(ok.success&&ok.checks>0,"safe held execution must certify");
  for(const auto& ticks:std::vector<std::vector<double>>{{.01},{.03},{.04},{.01,.03,.02,.04}}){
    auto irregular=certify_execution(safe,safe_sampler,safe_initial,Alignment{},c,bundle.reference(),now,.8,
      std::numeric_limits<double>::infinity(),ticks);
    require(irregular.success,"safe irregular output schedule must certify");
  }
  require(!certify_execution(safe,safe_sampler,safe_initial,Alignment{},c,bundle.reference(),now,.8,
    std::numeric_limits<double>::infinity(),{.06}).success,"unsupported output interval must fail certificate");
  safe.controls[0][0]=std::numeric_limits<double>::quiet_NaN();
  require(!certify_execution(safe,safe_sampler,safe_initial,Alignment{},c,bundle.reference(),now,.8).success,
          "nonfinite control must fail execution certification");
  Config corridor=c;corridor.enforce_corridor=true;
  const double theta=bundle.reference().length()/4.;const auto away=bundle.reference().at(theta);
  State parked{away.x,away.y,away.yaw,0.,theta,0.};
  Plan stationary=source;stationary.controls.assign(c.horizon,Control{0.,0.,0.});
  stationary.states.assign(c.horizon+1,parked);stationary.initial_applied={0.,0.,0.};
  OutputSampler parked_sampler(corridor);parked_sampler.reset(0.,0.,now-.02);
  require(certify_execution(stationary,parked_sampler,stationary.states.front(),Alignment{},corridor,
    bundle.reference(),now,.8).success,"nonzero projected progress corridor certificate rejected parked vehicle");
  parked[4]=0.;
  require(!certify_execution(stationary,parked_sampler,parked,Alignment{},corridor,
    bundle.reference(),now,.8).success,"corridor oracle must distinguish incorrectly zeroed progress");
  corridor.half_width=std::numeric_limits<double>::quiet_NaN();
  require(!certify_execution(stationary,parked_sampler,stationary.states.front(),Alignment{},corridor,
    bundle.reference(),now,.8).success,"nonfinite footprint cannot bypass executed corridor check");
  auto variable=safe;variable.controls.assign(c.horizon,Control{0.,0.,0.});
  variable.controls[0][2]=1.5;variable.source_epoch=now;variable.forecast_epoch=now;
  State variable_initial=safe_initial;variable_initial[3]=0.;OutputSampler variable_sampler(c);variable_sampler.reset(0.,0.,now-.02);
  auto crossed=certify_execution(variable,variable_sampler,variable_initial,Alignment{},c,bundle.reference(),now,.105,
    std::numeric_limits<double>::infinity(),{.03});
  require(crossed.success&&std::abs(crossed.final_state[4]-.15)<1e-10,
    "virtual progress integration crossed stage with wrong virtual-speed control");
  if(c.command_profile=="rate_bounded_v2"){
    Plan boundary=safe;boundary.success=true;boundary.source_epoch=20.;boundary.forecast_epoch=20.12;
    boundary.controls.assign(c.horizon,Control{0.,.178,.5});boundary.initial_applied={0.,.178,0.};
    State physical{r.x,r.y,r.yaw,.5,0.,.178};OutputSampler live(c);live.reset(.5,.178,20.7799);
    auto covered=certify_execution(boundary,live,physical,Alignment{},c,bundle.reference(),20.7999,.8);
    require(covered.success&&covered.checks>=2,"valid packet crossing original-source TTL must certify recovery braking");
    require(covered.max_utilization<=1.+1e-4,"TTL boundary retains the strict physical ellipse");
    auto unsafe=physical;unsafe[3]=c.max_speed;unsafe[5]=c.steer_limit;
    require(!certify_execution(boundary,live,unsafe,Alignment{},c,bundle.reference(),20.7999,.8).success,
      "TTL recovery budget must not admit a genuinely infeasible physical prefix");
  }
  std::cout<<"independent executed trace certificate passed; counterexample utilization="<<certificate.max_utilization<<'\n';
}

#include "aims_mpcc_rt/history.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace aims_mpcc_rt;
int main(){
  Config c{};c.steering_tau=.08;c.wheelbase=.36;c.max_speed=1.5;
  c.accel_limit=c.brake_limit=.5;
  AppliedHistory h(c);
  bool missing=false;try{h.command_at(10.);}catch(const std::exception&){missing=true;}assert(missing);
  h.record(10.,.1,1.,.2,.3);
  assert(!h.covers(9.9)&&h.covers(10.));
  auto before=h.at(9.9);assert(before.first==0.);
  auto estimate=h.at(10.08);assert(std::abs(estimate.first-.1*(1-std::exp(-1.)))<1e-12);
  assert(estimate.second[0]==.2&&estimate.second[1]==.1&&estimate.second[2]==.3);
  bool backwards=false;try{h.record(9.9,.1,1.,0.,0.);}catch(const std::exception&){backwards=true;}assert(backwards);
  h.record(10.1,-.1,0.,-.2,0.);
  State x{};auto y=h.predict(x,10.,10.2,10.2);
  assert(y[0]>0.&&y[3]<.06);
  assert(y[5]<estimate.first);
  auto actual=h.command_at(10.05);assert(actual.speed==1.);
  auto held=h.predict(x,10.,10.2,10.1);
  assert(held[3]<.06); // command exactly at known boundary must be held forward
  auto z=h.predict(x,10.,10.1,10.05,[](double,const State&){return HistoryCommand{0.,0.,0.,0.};});
  assert(z[3]<.03);
  assert(h.command_at(10.05).speed==1.); // forecasts never become actual history
  h.clear();assert(!h.covers(10.)&&h.newest()==-1.);
  std::cout<<"actual command history and prediction passed\n";
}

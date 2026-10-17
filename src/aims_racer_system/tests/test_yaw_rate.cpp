#include "aims_racer_system/yaw_rate.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace aims_racer_system::yaw_rate;
static Sample sample(int64_t t,double z) {return {t,{0.,0.,z}};}
static bool near(double a,double b,double tol=1e-8){return std::abs(a-b)<tol;}
int main() {
  {Observer o; for(int i=0;i<200;++i)assert(o.add(sample(1000000000LL+i*5000000LL,.7)));
   auto e=o.latest();assert(e.valid&&near(e.rate,.7)&&near(e.alpha,0));}
  {Observer o;int64_t t=1000000000LL;double elapsed=0;
   for(int i=0;i<800;++i){const int64_t dt=i%2?4000000:6000000;t+=dt;elapsed+=dt*1e-9;assert(o.add(sample(t,.2+1.5*elapsed)));}
   auto e=o.latest();assert(near(e.rate,.2+1.5*elapsed,1e-6)&&near(e.alpha,1.5,1e-5));}
  {Config c;c.mount_xyzw={0.,std::sqrt(.5),0.,std::sqrt(.5)};Observer o(c);
   for(int i=0;i<10;++i)assert(o.add({1000000000LL+i*5000000LL,{1.,0.,0.}}));
   assert(near(o.latest().rate,-1.));}
  {Observer o;for(int i=0;i<5;++i)o.add(sample(1000000000LL+i*5000000LL,.3));
   auto before=o.latest();assert(!o.add(sample(before.stamp_ns,9.)));assert(!o.add(sample(before.stamp_ns-1,9.)));
   assert(!o.add(sample(before.stamp_ns+1,std::numeric_limits<double>::quiet_NaN())));
   assert(near(before.rate,o.latest().rate)&&before.stamp_ns==o.latest().stamp_ns);
   assert(o.counters().duplicate==1&&o.counters().out_of_order==1&&o.counters().invalid==1);
   assert(o.add(sample(before.stamp_ns+100000000LL,-.5)));assert(!o.latest().valid);
   o.add(sample(before.stamp_ns+105000000LL,-.5));assert(!o.latest().valid);
   o.add(sample(before.stamp_ns+110000000LL,-.5));assert(o.latest().valid&&near(o.latest().rate,-.5));assert(o.counters().gaps==1);
   o.reset();assert(!o.latest().valid);assert(o.add(sample(1,.1)));}
  {Observer o;for(int i=0;i<1000;++i){o.add(sample(1000000000LL+i*5000000LL,std::sin(i*.02)));auto e=o.latest();
   assert(e.rate_variance>=0&&e.alpha_variance>=0&&e.rate_variance*e.alpha_variance-e.cross_covariance*e.cross_covariance>=-1e-12);}
   auto before=o.latest();assert(!o.add(sample(before.stamp_ns+1,1e308)));assert(before.stamp_ns==o.latest().stamp_ns);
   assert(o.add(sample(before.stamp_ns+5000000LL,.2)));assert(o.latest().valid);}
  {Config c;c.measurement_variance=0;bool bad=false;try{Observer o(c);}catch(const std::invalid_argument &){bad=true;}assert(bad);}
  std::cout<<"yaw-rate core: six physical/fault/covariance scenarios passed\n";
}

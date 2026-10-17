#include "aims_lateral_grip/core.hpp"
#include <cassert>
#include <iostream>
using namespace aims_lateral_grip;
Config config(){Config c;c.selected=true;c.wheelbase=.3;c.radius=3.;c.speeds={.5,1.};c.speed_limit=1.5;c.steering_limit=.2;c.stop_seconds=.1;c.stop_timeout=1.;c.ramp_rate=1.;c.speed_tolerance=.05;c.settle_seconds=.1;c.hold_seconds=.2;c.stage_timeout=3.;c.minimum_hold_samples=3;return c;}
Inputs healthy(double now,double speed=0.){Inputs i;i.clock=now;i.steady=now;i.authority=true;i.exclusive=true;i.speed=speed;i.rc_ok=true;i.telemetry_ok=true;i.final_ok=true;for(auto&s:i.samples)s={now,now,true};return i;}
void start(Core&c){assert(c.enable(healthy(1)));c.tick(healthy(1.11));assert(c.state()==State::Run);}
void hold(Core&c,double base,double speed){for(int n=1;n<=12;++n)c.tick(healthy(base+n*.05,speed));}
int main(){
 {Core c(Config{});assert(!c.enable(healthy(1)));assert(c.request().mode==1&&c.request().speed==0);}
 {auto cfg=config();cfg.radius=0;Core c(cfg);assert(!c.enable(healthy(1)));}
 {Core c(config());start(c);assert(c.request().speed==0);c.tick(healthy(1.21));assert(std::abs(c.request().speed-.1)<1e-9);c.tick(healthy(1.31));assert(std::abs(c.request().speed-.2)<1e-9);}
 {Core c(config());start(c);for(int n=1;n<=31;++n)c.tick(healthy(1.11+n*.1,0.));assert(c.state()==State::Aborted);assert(c.reason()=="stage_timeout");assert(c.stage()==0);assert(c.incomplete_stage());assert(c.completed_stages().empty());}
 {Core c(config());start(c);for(int n=1;n<=6;++n)c.tick(healthy(1.11+n*.1,.5));assert(c.stage()==0);c.tick(healthy(1.76,0.));c.tick(healthy(1.81,.5));c.tick(healthy(1.91,.5));c.tick(healthy(2.01,.5));assert(c.stage()==0);c.tick(healthy(2.12,.5));assert(c.stage()==0);c.tick(healthy(2.23,.5));assert(c.stage()==1);}
 {Core c(config());start(c);for(int n=1;n<=20;++n){auto i=healthy(1.11+n*.05,.5);i.samples[0].source=1.11;c.tick(i);if(c.state()==State::Aborted)break;}assert(c.stage()==0);}
 {Core c(config());start(c);hold(c,1.11,.5);hold(c,1.71,.5);assert(c.stage()==1);hold(c,2.31,1.);hold(c,2.91,1.);assert(c.state()==State::Coast);c.tick(healthy(3.6,.5));assert(c.direction()==1);c.tick(healthy(3.7));c.tick(healthy(3.81));assert(c.direction()==-1);assert(c.request().steering<0);c.stop("operator_stop");assert(!c.enable(healthy(4)));}
 {Core c(config());assert(c.enable(healthy(1)));auto i=healthy(1.02);i.authority=false;c.tick(i);assert(c.reason()=="authority_lost");assert(c.request().mode==1&&c.request().current==0.);}
 {Core c(config());assert(c.enable(healthy(1)));auto i=healthy(1.02);i.samples[0].source=.5;c.tick(i);assert(c.state()==State::Aborted);}
 {Core c(config());assert(c.enable(healthy(1)));auto i=healthy(1.02);i.samples[0].received=.1;c.tick(i);assert(c.state()==State::Aborted);}
 {Core c(config());assert(c.enable(healthy(1)));c.tick(healthy(.9));assert(c.reason()=="clock_rollback_or_freeze");}
 {Core c(config());assert(c.enable(healthy(1)));auto i=healthy(1.02);i.final_ok=false;c.tick(i);assert(c.state()==State::Aborted);}
 {Core c(config());assert(c.enable(healthy(1)));auto i=healthy(1.02);i.exclusive=false;c.tick(i);assert(c.reason()=="publisher_conflict");}
 {Core c(config());auto i=healthy(1);i.speed=.2;assert(!c.enable(i));}
 {Core c(config());assert(c.enable(healthy(1)));c.tick(healthy(2.01,.3));assert(c.reason()=="standstill_timeout");}
 {Core c(config());assert(c.enable(healthy(1)));c.tick(healthy(1.02,2.));assert(c.reason()=="speed_limit_exceeded");}
 {Core c(config());start(c);bool right=false;for(int n=1;n<200&&c.active();++n){right=right||c.direction()<0;double v=c.request().speed;c.tick(healthy(1.11+n*.05,v));}assert(right);assert(c.state()==State::Complete);assert(c.completed_stages().size()==4);assert(!c.enable(healthy(20)));}
 {Core c(config());assert(c.enable(healthy(1)));auto i=healthy(1.11);i.samples[0].source=1.;c.tick(i);assert(c.state()==State::Coast);}
 std::cout<<"lateral core contracts passed\n";
}

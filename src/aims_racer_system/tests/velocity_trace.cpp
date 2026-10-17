#include "aims_racer_system/velocity.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <chrono>
#include <cmath>
using namespace aims_racer_system::velocity;
int main(int argc,char**argv){
 if(argc<2)return 2;Config config;int64_t shift=0;
 for(int i=2;i+1<argc;i+=2){const std::string key=argv[i];const double value=std::stod(argv[i+1]);
 if(key=="--lever-y")config.lever.y()=value;else if(key=="--lever-z")config.lever.z()=value;
 else if(key=="--mount-roll-deg")config.mounting=Eigen::AngleAxisd(value*std::acos(-1.)/180,V3::UnitX()).toRotationMatrix();
 else if(key=="--lio-shift-ns")shift=std::stoll(argv[i+1]);else return 2;}
 Observer o(config);std::ifstream input(argv[1]);std::string name;int64_t t;std::cout<<std::setprecision(17);
 std::cout<<"event,input_source_ns,accepted,stamp_ns,valid,status,anchor_stamp_ns,vx,variance,px,py,pz,vwx,vwy,vwz,bix,biy,biz,rejected_lio,history_size,ix,iy,iz,lio_reinitializations,cost_us\n";
 while(input>>name>>t){bool accepted=false;const auto begin=std::chrono::steady_clock::now();
 if(name=="IMU"){Imu m;m.stamp=t;input>>m.f.x()>>m.f.y()>>m.f.z()>>m.w.x()>>m.w.y()>>m.w.z();accepted=o.add_imu(m);}
 else if(name=="LIO"){Lio m;m.stamp=t+shift;input>>m.p.x()>>m.p.y()>>m.p.z()>>m.q.x()>>m.q.y()>>m.q.z()>>m.q.w();accepted=o.add_lio(m);}else return 3;
 const auto e=o.latest();const double cost=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count();
 std::cout<<name<<','<<t<<','<<accepted<<','<<e.stamp<<','<<e.valid<<','<<e.status<<','<<e.anchor<<','<<e.vx<<','<<e.variance;
 for(const auto&v:{e.p,e.v,e.b})for(int k=0;k<3;++k)std::cout<<','<<v[k];std::cout<<','<<e.rejected<<','<<o.history_size()<<','<<e.innovation.x()<<','<<e.innovation.y()<<','<<e.innovation.z()<<','<<e.reacquisitions<<','<<cost<<'\n';
 }return input.eof()?0:4;
}

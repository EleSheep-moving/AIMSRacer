#include "aims_racer_system/yaw_rate.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <stdexcept>
using namespace aims_racer_system::yaw_rate;
int main(int argc,char **argv){try{
 Config c;if(argc==2)c.process_psd=std::stod(argv[1]);else if(argc!=1)throw std::invalid_argument("usage: yaw_rate_trace [process_psd]");
 Observer o(c);Sample s;std::cout<<std::setprecision(17)<<"stamp_ns,accepted,valid,measured_rate,rate,alpha,rate_variance,alpha_variance,cross_covariance,cost_us\n";
 while(std::cin>>s.stamp_ns>>s.gyro_rad_s[0]>>s.gyro_rad_s[1]>>s.gyro_rad_s[2]){
  const auto start=std::chrono::steady_clock::now();const bool ok=o.add(s);
  const double us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();const auto e=o.latest();
  std::cout<<s.stamp_ns<<','<<ok<<','<<e.valid<<','<<e.measured_rate<<','<<e.rate<<','<<e.alpha<<','<<e.rate_variance<<','<<e.alpha_variance<<','<<e.cross_covariance<<','<<us<<'\n';
 }if(!std::cin.eof())throw std::invalid_argument("malformed trace input");
 }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}}

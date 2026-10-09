// Offline fixed-input evidence. This executable never creates a ROS node.
#include "aims_mpcc_rt/core.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <limits>
using namespace aims_mpcc_rt;
namespace {
std::string json_string(const std::string& value) {
  std::ostringstream out;out << '\"';
  for(unsigned char ch:value) {
    if(ch=='\"'||ch=='\\')out << '\\' << char(ch);
    else if(ch<0x20)out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(ch) << std::dec;
    else out << char(ch);
  }
  out << '\"';return out.str();
}
std::string json_number(double value) {
  if(!std::isfinite(value))return "null";
  std::ostringstream out;out << std::setprecision(17) << value;return out.str();
}
}
int main(int argc,char** argv){
  if(argc!=4)throw std::invalid_argument("bundle, recorded fixture and output JSON required");
  auto bundle=Bundle::load(argv[1]);auto cfg=YAML::LoadFile(std::string(argv[1])+"/config.json");
  if(cfg["envelope_soft_enabled"].as<bool>())throw std::invalid_argument("historical strict-envelope evidence requires a strict bundle");
  std::ofstream output(argv[3]);if(!output)throw std::invalid_argument("cannot open output JSON");
  auto rows=YAML::LoadFile(argv[2]);output<<std::setprecision(17)<<"[\n";bool first=true;int matched=0;
  for(auto row:rows){
    bool matches=row["horizon"].as<int>()==bundle.config().horizon&&row["dt"].as<double>()==bundle.config().dt;
    for(auto item:row["config"]){auto a=item.second,b=cfg[item.first.as<std::string>()];
      matches=matches&&a.Type()==b.Type()&&(a.IsNull()||a.as<std::string>()==b.as<std::string>());}
    if(!matches)continue;
    ++matched;
    auto request=row["request"],s=request["state"],a=request["previous"];
    State state{s["x"].as<double>(),s["y"].as<double>(),s["yaw"].as<double>(),s["speed"].as<double>(),0.,s["steering"].as<double>()};
    Applied applied{a["acceleration"].as<double>(),a["steering"].as<double>(),a["steering_rate"].as<double>()};
    Alignment alignment{};for(int j=0;j<3;++j)alignment[j]=request["map_alignment"][j].as<double>();
    std::vector<double> refs;for(auto v:request["speed_refs"])refs.push_back(v.as<double>());
    Core solver(bundle);auto result=solver.solve(state,applied,alignment,request["elapsed"].as<double>(),.05,true,0.,.02,refs);
    const auto& c=bundle.config();double ay=state[3]*state[3]*std::tan(state[5])/(c.wheelbase*(1+c.understeer_coefficient*state[3]*state[3]));
    double lo=std::max(-c.brake_limit,applied[0]-c.jerk_limit*c.dt),hi=std::min(c.accel_limit,applied[0]+c.jerk_limit*c.dt);
    double best=lo<=hi?std::clamp(0.,lo,hi):0.;double axis=best>=0?c.envelope_accel:c.envelope_brake;
    double minimum=lo<=hi?std::pow(ay/c.lateral_accel_limit,2)+std::pow(best/axis,2):std::numeric_limits<double>::quiet_NaN();
    if(!first)output<<",\n";first=false;
    output<<"{\"id\":"<<json_string(row["id"].as<std::string>())<<",\"success\":"<<(result.success?"true":"false")
      <<",\"status\":"<<result.status<<",\"passes\":"<<result.native_passes<<",\"complete_s\":"<<json_number(result.solve_time_s)
      <<",\"native_s\":"<<json_number(result.native_time_s)<<",\"max_violation\":"<<json_number(result.max_violation)
      <<",\"initial_acceleration_continuation_feasible\":"<<(lo<=hi?"true":"false")
      <<",\"minimum_initial_utilization\":"<<json_number(minimum)<<",\"reason\":"<<json_string(result.reason)<<"}";
  }
  output<<"\n]\n";
  return matched?0:2;
}

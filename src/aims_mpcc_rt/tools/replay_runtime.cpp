// Exact ordered runtime-request replay. No ROS node or actuator interfaces.
#include "aims_mpcc_rt/core.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <cmath>
using namespace aims_mpcc_rt;
static std::string quote(const std::string& text){std::string out="\"";for(char c:text){
  if(c=='\"'||c=='\\')out+='\\';if(c=='\n'){out+="\\n";continue;}out+=c;}return out+'\"';}
static std::string number(double value){if(!std::isfinite(value))return "null";
  std::ostringstream out;out<<std::setprecision(17)<<value;return out.str();}
template<std::size_t N>static std::array<double,N> array(const YAML::Node& value){
  if(!value.IsSequence()||value.size()!=N)throw std::invalid_argument("request array dimension mismatch");
  std::array<double,N> answer;for(std::size_t j=0;j<N;++j)answer[j]=value[j].as<double>();return answer;}
int main(int argc,char** argv){
  if(argc<4||argc>5)throw std::invalid_argument("bundle, ordered request JSON, output JSON, optional frozen|refresh required");
  const bool refresh=argc==4||std::string(argv[4])=="refresh";
  if(argc==5&&!refresh&&std::string(argv[4])!="frozen")throw std::invalid_argument("geometry mode must be frozen or refresh");
  Bundle bundle=Bundle::load(argv[1]);Core core(bundle);auto rows=YAML::LoadFile(argv[2]);
  std::ofstream out(argv[3]);if(!out)throw std::invalid_argument("cannot write replay output");
  out<<"[\n";bool first=true;
  for(auto row:rows){
    if(row["core_reset"]&&row["core_reset"].as<bool>())core.reset();
    if(row["preparation_complete"]&&!row["preparation_complete"].as<bool>())continue;
    State state=array<6>(row["initial"]);Applied applied=array<3>(row["applied"]);Alignment alignment=array<3>(row["alignment"]);
    std::vector<double> targets;for(auto value:row["targets"])targets.push_back(value.as<double>());
    auto result=core.solve(state,applied,alignment,row["elapsed"].as<double>(),row["core_budget_s"].as<double>(),true,
      row["source_epoch"].as<double>(),row["forecast_epoch"].as<double>(),targets,refresh);
    if(!first)out<<",\n";first=false;
    out<<"{\"sequence\":"<<row["sequence"].as<unsigned long long>()<<",\"success\":"<<(result.success?"true":"false")
       <<",\"status\":"<<result.status<<",\"passes\":"<<result.native_passes<<",\"max_violation\":"<<number(result.max_violation)
       <<",\"geometry_refreshes\":"<<result.geometry_refreshes<<",\"geometry_shift\":"<<number(result.max_geometry_progress_shift)
       <<",\"solve_s\":"<<number(result.solve_time_s)<<",\"reason\":"<<quote(result.reason)<<",\"constraint_violations\":{";
    bool first_group=true;for(auto item:result.constraint_violations){if(!first_group)out<<',';first_group=false;
      out<<quote(item.first)<<':'<<number(item.second);}out<<"},\"states\":[";
    bool first_state=true;for(auto state:result.states){if(!first_state)out<<',';first_state=false;out<<'[';
      for(std::size_t j=0;j<state.size();++j){if(j)out<<',';out<<number(state[j]);}out<<']';}out<<"],\"controls\":[";
    bool first_control=true;for(auto control:result.controls){if(!first_control)out<<',';first_control=false;out<<'[';
      for(std::size_t j=0;j<control.size();++j){if(j)out<<',';out<<number(control[j]);}out<<']';}out<<"]}";
  }
  out<<"\n]\n";
}

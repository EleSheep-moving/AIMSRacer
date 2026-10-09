#include "aims_mpcc_rt/core.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <openssl/sha.h>
#include <iomanip>
#include <sstream>
#include <cstdlib>
#include <vector>
using aims_mpcc_rt::Bundle;
namespace fs=std::filesystem;
std::string file_sha(const fs::path &path) {
  std::ifstream file(path,std::ios::binary);std::ostringstream data;data<<file.rdbuf();
  auto bytes=data.str();unsigned char hash[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(bytes.data()),bytes.size(),hash);
  std::ostringstream out;for(auto byte:hash)out<<std::hex<<std::setw(2)<<std::setfill('0')<<int(byte);
  return out.str();
}
template<class F> void rejected(F f,const char *message) {
  bool threw=false;try {f();}catch(const std::exception&){threw=true;}
  if(!threw)throw std::runtime_error(message);
}
template<class F> void rejected_with(F f,const std::string& expected) {
  try{f();}catch(const std::exception& error){
    if(std::string(error.what()).find(expected)!=std::string::npos)return;
    throw std::runtime_error("wrong artifact rejection: "+std::string(error.what()));
  }
  throw std::runtime_error("artifact should reject: "+expected);
}
std::string shell_quote(const std::string& value){
  std::string answer="'";for(char c:value)answer+=c=='\''?"'\\''":std::string(1,c);return answer+"'";
}
void run(const std::vector<std::string>& args){
  std::string command;for(const auto& arg:args)command+=shell_quote(arg)+" ";
  if(std::system(command.c_str())!=0)throw std::runtime_error("artifact provider fixture build failed");
}
int main(int argc,char **argv) {
  if(argc!=2)throw std::runtime_error("bundle argument required");
  fs::path root=argv[1];
  auto manifest=YAML::LoadFile((root/"manifest.json").string());
  if(!manifest["capsule_abi_version"]||manifest["capsule_abi_version"].as<int>()!=1)
    throw std::runtime_error("capsule ABI version absent from exported artifact");
  if(!manifest["cost_scaling"])throw std::runtime_error("explicit objective scaling absent from exported artifact");
  Bundle::load(root.string(),(root/"input_config.yaml").string(),(root/"input_reference").string(),(root/"sources").string());
  auto temp=fs::temp_directory_path()/("aims-mpcc-artifact-test-"+std::to_string(getpid()));
  fs::create_directories(temp);
  fs::copy(root,temp/"copy",fs::copy_options::recursive);
  auto copy=temp/"copy";
  auto near_manifest=YAML::LoadFile((copy/"manifest.json").string());
  double canonical=near_manifest["dt"].as<double>();
  near_manifest["dt"]=canonical+5e-13;
  {std::ofstream changed(copy/"manifest.json");changed<<YAML::Dump(near_manifest);}
  auto near_native=YAML::LoadFile((copy/"native_manifest.json").string());
  near_native["source_manifest_sha256"]=file_sha(copy/"manifest.json");
  {std::ofstream changed(copy/"native_manifest.json");changed<<YAML::Dump(near_native);}
  {
    auto near_bundle=Bundle::load(copy.string());
    if(near_bundle.config().dt!=canonical)throw std::runtime_error("near stage duration must canonicalize consistently with legacy mesh");
  }  // Unload the copied library before subsequent corruption tests overwrite it.
  fs::copy_file(root/"manifest.json",copy/"manifest.json",fs::copy_options::overwrite_existing);
  fs::copy_file(root/"native_manifest.json",copy/"native_manifest.json",fs::copy_options::overwrite_existing);
  if(manifest["command_profile"]&&manifest["command_profile"].as<std::string>()=="rate_bounded_v2"){
    auto missing_marker=YAML::LoadFile((copy/"manifest.json").string());missing_marker.remove("stage_zero_envelope_bounds");
    {std::ofstream changed(copy/"manifest.json");changed<<YAML::Dump(missing_marker);}
    auto missing_native=YAML::LoadFile((copy/"native_manifest.json").string());
    missing_native["source_manifest_sha256"]=file_sha(copy/"manifest.json");
    {std::ofstream changed(copy/"native_manifest.json");changed<<YAML::Dump(missing_native);}
    rejected_with([&]{Bundle::load(copy.string());},"regenerated with exact stage-zero");
    fs::copy_file(root/"manifest.json",copy/"manifest.json",fs::copy_options::overwrite_existing);
    fs::copy_file(root/"native_manifest.json",copy/"native_manifest.json",fs::copy_options::overwrite_existing);
    auto original_native=YAML::LoadFile((root/"native_manifest.json").string());fs::path install;
    for(auto entry:original_native["dependencies"]){
      fs::path dependency=entry.first.as<std::string>();
      if(dependency.filename()=="libacados.so")install=dependency.parent_path().parent_path();
    }
    if(install.empty())throw std::runtime_error("native dependency location unavailable for provider regression");
    std::ifstream input(root/"bridge.c");std::ostringstream bytes;bytes<<input.rdbuf();std::string bridge=bytes.str();
    auto begin=bridge.find("void aims_rt_input_bounds(");auto end=bridge.find("int aims_rt_parameters(",begin);
    if(begin==std::string::npos||end==std::string::npos)throw std::runtime_error("exported input bounds bridge unavailable");
    bridge.erase(begin,end-begin);
    {std::ofstream source(copy/"bridge_missing.c");source<<bridge;}
    auto command=std::vector<std::string>{"gcc","-shared","-fPIC","-O2",(copy/"bridge_missing.c").string(),
      (copy/"model_eval.c").string(),"-I"+(install/"include").string(),"-I"+(install/"include/acados").string(),
      "-I"+(install/"include/blasfeo/include").string(),"-I"+(install/"include/hpipm/include").string(),
      "-L"+(copy/"generated").string(),"-L"+(install/"lib").string(),"-lacados_ocp_solver_aims_runtime",
      "-lacados","-lhpipm","-lblasfeo","-lm","-Wl,--disable-new-dtags,-rpath,$ORIGIN/generated:"+(install/"lib").string(),
      "-o",(copy/"libaims_mpcc_bundle.so").string()};
    run(command);
    auto certify_wrapper=[&]{auto n=original_native;n["libraries"]["libaims_mpcc_bundle.so"]=file_sha(copy/"libaims_mpcc_bundle.so");
      std::ofstream changed(copy/"native_manifest.json");changed<<YAML::Dump(n);};
    certify_wrapper();
    rejected_with([&]{Bundle::load(copy.string());},"missing capsule ABI symbol: aims_rt_input_bounds");
    {std::ofstream source(copy/"foreign_setter.c");source<<"void aims_rt_input_bounds(void*c,int stage,const double*l,const double*u){}\n";}
    run({"gcc","-shared","-fPIC",(copy/"foreign_setter.c").string(),"-o",(copy/"libforeign_setter.so").string()});
    command.insert(command.begin()+2,{"-Wl,--no-as-needed",(copy/"libforeign_setter.so").string(),"-Wl,--as-needed"});
    run(command);certify_wrapper();
    rejected_with([&]{Bundle::load(copy.string());},"stage-zero bounds setter provider mismatch");
    fs::copy_file(root/"libaims_mpcc_bundle.so",copy/"libaims_mpcc_bundle.so",fs::copy_options::overwrite_existing);
    fs::copy_file(root/"native_manifest.json",copy/"native_manifest.json",fs::copy_options::overwrite_existing);
  }
  auto profile_manifest=YAML::LoadFile((copy/"manifest.json").string());
  profile_manifest["command_profile"]=manifest["command_profile"]&&manifest["command_profile"].as<std::string>()=="rate_bounded_v2"?
    "legacy_bounded_v1":"rate_bounded_v2";
  {std::ofstream changed(copy/"manifest.json");changed<<YAML::Dump(profile_manifest);}
  auto profile_native=YAML::LoadFile((copy/"native_manifest.json").string());
  profile_native["source_manifest_sha256"]=file_sha(copy/"manifest.json");
  {std::ofstream changed(copy/"native_manifest.json");changed<<YAML::Dump(profile_native);}
  rejected([&]{Bundle::load(copy.string());},"bundle profile disagrees with frozen configuration must fail startup");
  fs::copy_file(root/"manifest.json",copy/"manifest.json",fs::copy_options::overwrite_existing);
  fs::copy_file(root/"native_manifest.json",copy/"native_manifest.json",fs::copy_options::overwrite_existing);
  auto native=YAML::LoadFile((copy/"native_manifest.json").string());
  native["machine"]="incorrect_target";
  {std::ofstream changed(copy/"native_manifest.json");changed<<YAML::Dump(native);}
  rejected([&]{Bundle::load(copy.string());},"wrong target architecture must fail startup");
  fs::copy_file(root/"native_manifest.json",copy/"native_manifest.json",fs::copy_options::overwrite_existing);
  native=YAML::LoadFile((copy/"native_manifest.json").string());native["acados_commit"]="wrong_commit";
  {std::ofstream changed(copy/"native_manifest.json");changed<<YAML::Dump(native);}
  rejected([&]{Bundle::load(copy.string());},"wrong dependency version must fail startup");
  fs::copy_file(root/"native_manifest.json",copy/"native_manifest.json",fs::copy_options::overwrite_existing);
  {std::ofstream changed(temp/"different.yaml");changed<<"wheelbase: 999\n";}
  rejected([&]{Bundle::load(root.string(),(temp/"different.yaml").string());},"wrong config must fail startup");
  {std::ofstream changed(copy/"sources/config.py",std::ios::app);changed<<"\n# corruption\n";}
  rejected([&]{Bundle::load(copy.string());},"source corruption must fail startup");
  fs::copy_file(root/"sources/config.py",copy/"sources/config.py",fs::copy_options::overwrite_existing);
  {std::ofstream changed(copy/"libaims_mpcc_bundle.so",std::ios::app);changed<<"corruption";}
  rejected([&]{Bundle::load(copy.string());},"native corruption must fail startup");
  fs::copy_file(root/"libaims_mpcc_bundle.so",copy/"libaims_mpcc_bundle.so",fs::copy_options::overwrite_existing);
  {std::ofstream changed(copy/"reference.json",std::ios::app);changed<<" ";}
  rejected([&]{Bundle::load(copy.string());},"reference corruption must fail startup");
  fs::remove_all(temp);
  std::cout<<"PASS: matching artifacts load; architecture/version/config/reference/source/native mismatch rejected\n";
}

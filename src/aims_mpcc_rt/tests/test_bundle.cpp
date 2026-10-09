#include "aims_mpcc_rt/core.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
using aims_mpcc_rt::Bundle;
namespace fs=std::filesystem;
template<class F> void rejected(F f,const char *message) {
  bool threw=false;try {f();}catch(const std::exception&){threw=true;}
  if(!threw)throw std::runtime_error(message);
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

// Test the exact qualification function used by Bundle::load, without a solver.
#include "../src/core.cpp"
#include <iostream>
int main() {
  auto config=YAML::Load(R"({wheelbase: 0.36, rear_offset: 0, half_width: 0.16, half_length: null, front_extent: 0.52, rear_extent: 0.1})");
  auto metadata=YAML::Load(R"({closed_lap: true, source: gui, source_frame: base_link, coordinate_reference: rear_axle, vehicle_geometry: {wheelbase: 0.36, rear_offset: 0, half_width: 0.16, half_length: null, front_extent: 0.52, rear_extent: 0.1}})");
  if(!aims_mpcc_rt::reference_geometry_matches(metadata,config))throw std::runtime_error("GUI complete geometry rejected");
  metadata.remove("source");metadata.remove("source_frame");metadata.remove("coordinate_reference");
  if(!aims_mpcc_rt::reference_geometry_matches(metadata,config))throw std::runtime_error("legacy complete geometry rejected");
  for(const char* field:{"wheelbase","rear_offset","half_width","front_extent","rear_extent"}) {
    auto changed=YAML::Clone(metadata);changed["vehicle_geometry"][field]=9.;
    if(aims_mpcc_rt::reference_geometry_matches(changed,config))throw std::runtime_error("geometry mismatch accepted");
  }
  auto missing=YAML::Clone(metadata);missing["vehicle_geometry"].remove("rear_extent");
  if(aims_mpcc_rt::reference_geometry_matches(missing,config))throw std::runtime_error("missing geometry accepted");
  auto missing_null=YAML::Clone(metadata);missing_null["vehicle_geometry"].remove("half_length");
  if(aims_mpcc_rt::reference_geometry_matches(missing_null,config))throw std::runtime_error("missing nullable half_length accepted");
  auto nonfinite=YAML::Clone(metadata);nonfinite["vehicle_geometry"]["wheelbase"]=".nan";
  if(aims_mpcc_rt::reference_geometry_matches(nonfinite,config))throw std::runtime_error("nonfinite geometry accepted");
  metadata["source_frame"]="front_axle";
  if(aims_mpcc_rt::reference_geometry_matches(metadata,config))throw std::runtime_error("incorrect source frame accepted");
  metadata.remove("source_frame");metadata["closed_lap"]=false;
  if(aims_mpcc_rt::reference_geometry_matches(metadata,config))throw std::runtime_error("open reference accepted");
  std::cout<<"PASS: GUI and legacy qualification, complete exact geometry, finite values, closed lap and source frame\n";
}

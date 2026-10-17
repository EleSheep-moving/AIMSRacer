#pragma once
#include <ostream>
#include <string>
namespace aims_lateral_grip {
inline bool write_checked(std::ostream& stream,const std::string& bytes) noexcept {
  try{stream<<bytes;stream.flush();return stream.good();}catch(...){return false;}
}
}

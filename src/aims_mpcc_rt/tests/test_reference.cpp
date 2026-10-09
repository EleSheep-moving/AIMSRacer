#include "aims_mpcc_rt/core.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <limits>
using namespace aims_mpcc_rt;
int main(int argc,char** argv) {
  if(argc!=2)throw std::runtime_error("bundle required");
  auto bundle=Bundle::load(argv[1]);const auto& ref=bundle.reference();
  const double length=ref.length();
  for(double fraction:{-.01,0.,.001,.23,.51,.99,1.,1.01}) {
    auto p=ref.at(fraction*length),wrapped=ref.at((fraction+3.)*length);
    if(std::hypot(p.x-wrapped.x,p.y-wrapped.y)>1e-9 ||
       std::abs(std::remainder(p.yaw-wrapped.yaw,2.*std::acos(-1.)))>1e-9)
      throw std::runtime_error("reference periodic geometry mismatch");
    // Compare off-path global projection with an independent dense search.
    // At nearby branches, require the closest geometry rather than assuming
    // that progress must remain on the branch used to construct the query.
    for(double offset:{-.2,-.05,0.,.05,.2}) {
      const double x=p.x-offset*std::sin(p.yaw),y=p.y+offset*std::cos(p.yaw);
      auto q=ref.at(ref.project(x,y));double selected=std::hypot(q.x-x,q.y-y);
      double dense=std::numeric_limits<double>::infinity();
      for(int j=0;j<20000;++j){auto candidate=ref.at(length*j/20000.);
        dense=std::min(dense,std::hypot(candidate.x-x,candidate.y-y));}
      if(!std::isfinite(selected)||selected>dense+1e-4)
        throw std::runtime_error("global projection missed a closer reference branch");
    }
  }
  std::cout<<"PASS: wrap and off-path global projection against dense independent search\n";
}

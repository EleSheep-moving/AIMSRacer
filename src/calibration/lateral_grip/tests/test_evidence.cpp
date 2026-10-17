#include "aims_lateral_grip/evidence.hpp"
#include <fstream>
#include <sstream>
#include <cassert>
int main(){std::ostringstream good;assert(aims_lateral_grip::write_checked(good,"sample\n"));assert(good.str()=="sample\n");std::ofstream full("/dev/full");assert(!aims_lateral_grip::write_checked(full,"sample\n"));}

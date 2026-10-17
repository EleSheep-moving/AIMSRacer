// Loopback test fixture only. Not installed; never sends vehicle commands.
#include <vrpn_Tracker.h>
#include <vrpn_Connection.h>
#include <vrpn_Shared.h>
#include <chrono>
#include <thread>
#include <cstdlib>
int main(int argc,char** argv){
  const int port=argc>1?std::atoi(argv[1]):43883;
  auto* connection=vrpn_create_server_connection(port);
  if(!connection)return 1;
  {vrpn_Tracker_Server tracker("synthetic_car",connection,1);
   for(int i=0;i<4000;++i){struct timeval stamp;vrpn_gettimeofday(&stamp,nullptr);stamp.tv_sec-=2;
    const vrpn_float64 p[3]={1.25,0.1,0.0},q[4]={0.,0.,0.,1.};
    tracker.report_pose(0,stamp,p,q);tracker.mainloop();connection->mainloop();std::this_thread::sleep_for(std::chrono::milliseconds(10));}}
  connection->removeReference();return 0;
}

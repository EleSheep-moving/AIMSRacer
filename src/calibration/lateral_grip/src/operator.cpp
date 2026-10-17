#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <chrono>
#include <iostream>
#include <thread>
#ifndef LATERAL_OPERATION
#define LATERAL_OPERATION "inspect"
#endif
int main(int argc,char**argv){rclcpp::init(argc,argv);auto node=rclcpp::Node::make_shared(std::string("lateral_")+LATERAL_OPERATION);int result=1;
  if(std::string(LATERAL_OPERATION)=="inspect"){
    bool received=false;auto sub=node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/calib/lateral/status",10,[&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m){for(const auto&s:m->status){std::cout<<s.message<<'\n';for(const auto&v:s.values)std::cout<<v.key<<": "<<v.value<<'\n';}received=true;});
    auto end=std::chrono::steady_clock::now()+std::chrono::seconds(3);while(rclcpp::ok()&&!received&&std::chrono::steady_clock::now()<end){rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(20));}result=received?0:1;
  }else{auto client=node->create_client<std_srvs::srv::SetBool>("/calib/lateral/enable");if(client->wait_for_service(std::chrono::seconds(3))){auto req=std::make_shared<std_srvs::srv::SetBool::Request>();req->data=std::string(LATERAL_OPERATION)=="start";auto f=client->async_send_request(req);if(rclcpp::spin_until_future_complete(node,f,std::chrono::seconds(5))==rclcpp::FutureReturnCode::SUCCESS){auto response=f.get();std::cout<<response->message<<'\n';result=response->success?0:1;}}}
  if(result)std::cerr<<"lateral operation failed or timed out\n";rclcpp::shutdown();return result;
}

#include "aims_racer_system/velocity.hpp"
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace aims_racer_system::velocity {
namespace {
M3 exp(const V3 &v){const double a=v.norm();return a<1e-15?M3::Identity():Eigen::AngleAxisd(a,v/a).toRotationMatrix();}
bool finite(const V3&v){return v.allFinite();}
}
Observer::Observer(Config c):c_(std::move(c)){
 for(double x:{c_.gravity,c_.position_std,c_.bias_psd,c_.force_variance,c_.tilt_std,c_.initial_bias_std,c_.initialization_span,c_.position_gate,c_.angle_gate,c_.max_gap,c_.degraded_age,c_.max_age,c_.history})
 if(!std::isfinite(x)||x<=0)throw std::invalid_argument("positive finite velocity configuration required");
 if(c_.history<c_.max_age||c_.degraded_age>c_.max_age||c_.max_events<8||!finite(c_.lever)||!c_.mounting.allFinite()||
 (c_.mounting.transpose()*c_.mounting-M3::Identity()).norm()>1e-9||std::abs(c_.mounting.determinant()-1)>1e-9)
 throw std::invalid_argument("invalid velocity geometry/history limits");
}
void Observer::reset(){imus_.clear();states_.clear();anchors_.clear();base_=State{};s_=State{};replayed_=0;}
bool Observer::old(int64_t t) const{
 return (base_.previous&&t<=base_.previous->stamp)||(!imus_.empty()&&(imus_.back().stamp-t)*1e-9>c_.history)||
 (imus_.empty()&&!anchors_.empty()&&(anchors_.rbegin()->first-t)*1e-9>c_.history);
}
void Observer::propagate(State&s,double dt,const V3&f,const V3&w) const{
 if(dt<=0||!s.oriented)return;
 const M3 R=s.R*exp(w*(dt*.5));const V3 a=R*(f*c_.gravity-s.b)+V3(0,0,-c_.gravity);
 s.p+=s.v*dt+.5*a*dt*dt;s.v+=a*dt;s.R=s.R*exp(w*dt);
 if(!s.initialized)return;
 M9 F=M9::Identity();F.block<3,3>(0,3)=M3::Identity()*dt;F.block<3,3>(0,6)=-.5*dt*dt*R;F.block<3,3>(3,6)=-dt*R;
 Eigen::Matrix<double,9,3> G=Eigen::Matrix<double,9,3>::Zero();G.topRows<3>()=.5*dt*dt*R;G.middleRows<3>(3)=dt*R;
 M9 Q=(c_.force_variance+std::pow(c_.gravity*c_.tilt_std,2))*(G*G.transpose());const double bq=c_.bias_psd;
 Q.block<3,3>(0,0)+=M3::Identity()*bq*std::pow(dt,5)/20;Q.block<3,3>(0,3)+=M3::Identity()*bq*std::pow(dt,4)/8;
 Q.block<3,3>(3,0)=Q.block<3,3>(0,3).transpose();Q.block<3,3>(3,3)+=M3::Identity()*bq*std::pow(dt,3)/3;
 Q.block<3,3>(0,6)=-R*bq*std::pow(dt,3)/6;Q.block<3,3>(6,0)=Q.block<3,3>(0,6).transpose();
 Q.block<3,3>(3,6)=-R*bq*dt*dt/2;Q.block<3,3>(6,3)=Q.block<3,3>(3,6).transpose();Q.block<3,3>(6,6)=M3::Identity()*bq*dt;
 s.P=(F*s.P*F.transpose()+Q).eval();s.P=((s.P+s.P.transpose())*.5).eval();
}
void Observer::lio(State&s,const Lio&m) const{
 const M3 R=m.q.normalized().toRotationMatrix();
 if(!c_.continue_on_lio_timeout&&s.initialized&&(m.stamp-s.anchor)*1e-9>c_.max_age+1e-12){
  const auto previous=s.previous;const auto rejected=s.rejected;const auto count=s.reacquisitions+1;
  s=State{};s.previous=previous;s.rejected=rejected;s.reacquisitions=count;
 }
 if(!s.oriented){s.R=R;s.p=m.p;s.v.setZero();s.b.setZero();s.oriented=true;}
 if(!s.initialized){
  s.init.emplace_back(m.stamp,(m.p-s.p).eval());
  while(s.init.size()>3&&(m.stamp-s.init.front().first)*1e-9>1.)s.init.erase(s.init.begin());
  while(s.init.size()>64)s.init.erase(s.init.begin()+1);
  if(s.init.size()>=3&&(m.stamp-s.init.front().first)*1e-9+1e-12>=c_.initialization_span){
   double mean=0,den=0;V3 ymean=V3::Zero(),slope=V3::Zero();
   for(const auto&k:s.init){mean+=(k.first-s.init.front().first)*1e-9;ymean+=k.second;}
   mean/=s.init.size();ymean/=s.init.size();
   for(const auto&k:s.init){double tc=(k.first-s.init.front().first)*1e-9-mean;den+=tc*tc;slope+=tc*k.second;}
   slope/=den;double residual=0;
   for(const auto&k:s.init){double tc=(k.first-s.init.front().first)*1e-9-mean;residual=std::max(residual,(k.second-ymean-tc*slope).norm());}
   if(residual<=std::max(.10,5*c_.position_std)){
    const double last=(m.stamp-s.init.front().first)*1e-9-mean;
    s.v+=slope;s.p=m.p;s.initialized=true;s.init.clear();const double var=c_.position_std*c_.position_std;s.P.setZero();
    s.P.block<3,3>(0,0)=M3::Identity()*var;s.P.block<3,3>(3,3)=M3::Identity()*var/den;
    s.P.block<3,3>(0,3)=M3::Identity()*var*last/den;s.P.block<3,3>(3,0)=s.P.block<3,3>(0,3).transpose();
    s.P.block<3,3>(6,6)=M3::Identity()*c_.initial_bias_std*c_.initial_bias_std;
   }
  }
  s.R=R;s.anchor=m.stamp;return;
 }
 const V3 innovation=m.p-s.p;const M3 S=s.P.block<3,3>(0,0)+M3::Identity()*c_.position_std*c_.position_std;
 const auto solve=S.ldlt();const double angle=Eigen::AngleAxisd(s.R.transpose()*R).angle();
 if(innovation.dot(solve.solve(innovation))>c_.position_gate||angle>c_.angle_gate){++s.rejected;return;}
 s.innovation=innovation;const Eigen::Matrix<double,9,3> K=solve.solve(s.P.topRows<3>()).transpose();
 const Eigen::Matrix<double,9,1> dx=K*innovation;s.p+=dx.head<3>();s.v+=dx.segment<3>(3);s.b+=dx.tail<3>();
 M9 A=M9::Identity();A.leftCols<3>()-=K;
 s.P=(A*s.P*A.transpose()+c_.position_std*c_.position_std*(K*K.transpose())).eval();s.P=((s.P+s.P.transpose())*.5).eval();
 s.R=R;s.anchor=m.stamp;
}
Observer::State Observer::advance(const State&before,const Imu&m) const{
 State s=before;const auto previous=s.previous;
 if(previous&&(m.stamp-previous->stamp)*1e-9>c_.max_gap+1e-12){const auto rejected=s.rejected;const auto count=s.reacquisitions;s=State{};s.rejected=rejected;s.reacquisitions=count;s.previous=m;s.status="imu_gap";return s;}
 const int64_t start=previous?previous->stamp:m.stamp-1;int64_t cursor=previous?previous->stamp:m.stamp;
 auto values=[&](int64_t at){if(!previous)return std::pair<V3,V3>{m.f,m.w};const double a=double(at-previous->stamp)/double(m.stamp-previous->stamp);
 return std::pair<V3,V3>{previous->f+a*(m.f-previous->f),previous->w+a*(m.w-previous->w)};};
 auto step=[&](int64_t until){const auto a=values(cursor),b=values(until);propagate(s,(until-cursor)*1e-9,.5*(a.first+b.first),.5*(a.second+b.second));cursor=until;};
 for(auto it=anchors_.upper_bound(start);it!=anchors_.end()&&it->first<=m.stamp;++it){step(it->first);lio(s,it->second);}
 step(m.stamp);s.previous=m;const double age=(m.stamp-s.anchor)*1e-9;
 s.status=!s.initialized?"initializing":(age>c_.max_age+1e-12?"lio_stale":(age>c_.degraded_age+1e-12?"degraded":"valid"));
 const V3 projected=c_.mounting*s.R.transpose()*s.v-V3(c_.mounting*m.w).cross(c_.lever);
 const Eigen::RowVector3d row=c_.mounting.row(0)*s.R.transpose();
 const double variance=(row*s.P.block<3,3>(3,3)*row.transpose())(0);
 if(!s.p.allFinite()||!s.v.allFinite()||!s.b.allFinite()||!s.P.allFinite()||!s.R.allFinite()||!projected.allFinite()||!std::isfinite(variance)||variance<0)throw std::runtime_error("nonfinite velocity filter/output");
 return s;
}
void Observer::replay(std::size_t index){
 State s=index==0?base_:states_[index-1];std::vector<State> changed;changed.reserve(imus_.size()-index);
 for(std::size_t j=index;j<imus_.size();++j){s=advance(s,imus_[j]);changed.push_back(s);}
 while(states_.size()>index)states_.pop_back();
 for(auto&v:changed)states_.push_back(std::move(v));
 if(!states_.empty())s_=states_.back();
 replayed_+=changed.size();
}
void Observer::trim(){
 if(imus_.empty()&&!anchors_.empty()){
  const auto newest=anchors_.rbegin()->first;while(!anchors_.empty()&&(anchors_.size()>c_.max_events||(newest-anchors_.begin()->first)*1e-9>c_.history))anchors_.erase(anchors_.begin());return;
 }
 while(imus_.size()>1&&(history_size()>c_.max_events||(imus_.back().stamp-imus_.front().stamp)*1e-9>c_.history)){
  imus_.pop_front();base_=std::move(states_.front());states_.pop_front();anchors_.erase(anchors_.begin(),anchors_.upper_bound(base_.previous->stamp));
 }
 if(imus_.size()==1&&!anchors_.empty()){
  const auto newest=anchors_.rbegin()->first;
  if((newest-imus_.front().stamp)*1e-9>c_.history)
   while(!anchors_.empty()&&(newest-anchors_.begin()->first)*1e-9>c_.history)anchors_.erase(anchors_.begin());
 }
 while(history_size()>c_.max_events)anchors_.erase(anchors_.begin());
}
bool Observer::add_imu(const Imu&m){
 if(m.stamp<=0||!finite(m.f)||!finite(m.w)||old(m.stamp))return false;
 const auto it=std::lower_bound(imus_.begin(),imus_.end(),m.stamp,[](const Imu&a,int64_t t){return a.stamp<t;});
 if(it!=imus_.end()&&it->stamp==m.stamp)return false;
 const std::size_t index=it-imus_.begin();
 try{if(index==imus_.size()){auto state=advance(s_,m);imus_.push_back(m);states_.push_back(state);s_=state;}
 else{imus_.insert(it,m);try{replay(index);}catch(...){imus_.erase(imus_.begin()+index);throw;}}}
 catch(const std::exception&){return false;}trim();return true;
}
bool Observer::add_lio(const Lio&m){
 if(m.stamp<=0||!finite(m.p)||!m.q.coeffs().allFinite()||std::abs(m.q.norm()-1)>1e-3||old(m.stamp)||anchors_.count(m.stamp))return false;
 anchors_.emplace(m.stamp,m);try{if(!imus_.empty()&&m.stamp<=imus_.back().stamp){
 const auto it=std::lower_bound(imus_.begin(),imus_.end(),m.stamp,[](const Imu&a,int64_t t){return a.stamp<t;});replay(it-imus_.begin());}}
 catch(const std::exception&){anchors_.erase(m.stamp);return false;}trim();return true;
}
Estimate Observer::latest() const{
 Estimate e;e.stamp=s_.previous?s_.previous->stamp:0;e.anchor=s_.anchor;e.status=s_.status;e.valid=s_.initialized&&(s_.status=="valid"||s_.status=="degraded"||(c_.continue_on_lio_timeout&&s_.status=="lio_stale"));
 const V3 omega=s_.previous?V3(c_.mounting*s_.previous->w):V3::Zero();
 const V3 velocity=c_.mounting*s_.R.transpose()*s_.v-omega.cross(c_.lever);const Eigen::RowVector3d row=c_.mounting.row(0)*s_.R.transpose();
 const M3 projection=c_.mounting*s_.R.transpose();
 e.imu_velocity_body=projection*s_.v;e.velocity_body=velocity;e.omega_body=omega;
 e.velocity_covariance_body=projection*s_.P.block<3,3>(3,3)*projection.transpose();
 e.vx=velocity.x();e.variance=std::max((row*s_.P.block<3,3>(3,3)*row.transpose())(0),1e-4);
 e.p=s_.p;e.v=s_.v;e.b=s_.b;e.innovation=s_.innovation;e.rejected=s_.rejected;e.reacquisitions=s_.reacquisitions;return e;
}
}

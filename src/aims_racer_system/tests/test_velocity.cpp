#include "aims_racer_system/velocity.hpp"
#include <gtest/gtest.h>
using namespace aims_racer_system::velocity;
void drive(Observer& o,int n=600,int lio_step=20){
 for(int i=0;i<=n;++i){const int64_t t=1000000000LL+i*5000000LL;
  if(i%lio_step==0)ASSERT_TRUE(o.add_lio({t,{i*.005*2,i*.005*.3,0},Eigen::Quaterniond::Identity()}));
  ASSERT_TRUE(o.add_imu({t,{0,0,1},{0,0,0}}));}}
TEST(Velocity,MovingThreeDimensionalInitialization){Observer o;drive(o);auto e=o.latest();EXPECT_TRUE(e.valid);EXPECT_NEAR(e.vx,2,1e-9);EXPECT_NEAR(e.v.y(),.3,1e-9);EXPECT_LE(o.history_size(),4096u);}
TEST(Velocity,HighFrequencyInitialization){Observer o;drive(o,200,4);EXPECT_TRUE(o.latest().valid);EXPECT_NEAR(o.latest().vx,2,1e-9);}
TEST(Velocity,GapDuplicateAndNumericalFault){Observer o;drive(o,200);auto e=o.latest();EXPECT_FALSE(o.add_imu({e.stamp,{0,0,1},{0,0,0}}));EXPECT_FALSE(o.add_imu({e.stamp+5000000,{1e308,0,1},{0,0,0}}));EXPECT_EQ(o.latest().stamp,e.stamp);ASSERT_TRUE(o.add_imu({e.stamp+40000000,{0,0,1},{0,0,0}}));EXPECT_FALSE(o.latest().valid);EXPECT_EQ(o.latest().status,"imu_gap");}
TEST(Velocity,OutageAndBound){Config c;c.max_events=100;Observer o(c);drive(o,200);for(int i=1;i<=102;++i)ASSERT_TRUE(o.add_imu({2000000000LL+i*5000000LL,{0,0,1},{0,0,0}}));EXPECT_FALSE(o.latest().valid);EXPECT_EQ(o.latest().status,"lio_stale");EXPECT_LE(o.history_size(),100u);}
TEST(Velocity,CoastingWarnsGrowsCovarianceAndResumesWithoutReinitialization){
 Config c;c.continue_on_lio_timeout=true;Observer o(c);drive(o,200);
 const double variance=o.latest().variance;
 for(int i=1;i<=200;++i)ASSERT_TRUE(o.add_imu({2000000000LL+i*5000000LL,{0,0,1},{0,0,0}}));
 auto e=o.latest();ASSERT_TRUE(e.valid);EXPECT_EQ(e.status,"lio_stale");EXPECT_NEAR(e.vx,2,1e-8);EXPECT_GT(e.variance,variance);
 ASSERT_TRUE(o.add_lio({3000000000LL,{4,.6,0},Eigen::Quaterniond::Identity()}));
 e=o.latest();EXPECT_TRUE(e.valid);EXPECT_EQ(e.status,"valid");EXPECT_EQ(e.reacquisitions,0u);
 ASSERT_TRUE(o.add_lio({3000000001LL,{100,0,0},Eigen::Quaterniond::Identity()}));
 ASSERT_TRUE(o.add_imu({3005000000LL,{0,0,1},{0,0,0}}));
 EXPECT_GE(o.latest().rejected,1u);EXPECT_NEAR(o.latest().vx,2,1e-8);
}
TEST(Velocity,AnchorOnlyBound){Config c;c.max_events=20;Observer o(c);o.add_imu({1000000000,{0,0,1},{0,0,0}});for(int i=0;i<100;++i)EXPECT_TRUE(o.add_lio({1000000000LL+i*100000000LL,{0,0,0},Eigen::Quaterniond::Identity()}));EXPECT_LE(o.history_size(),20u);}
TEST(Velocity,RejectInvalidRotation){Config c;c.mounting(0,0)=2;EXPECT_THROW(Observer o(c),std::invalid_argument);}
TEST(Velocity,DelayedUpdateAndJump){Observer a,b;for(int i=0;i<=400;++i){int64_t t=1000000000LL+i*5000000LL;
 if(i%20==0)a.add_lio({t,{i*.01,0,0},Eigen::Quaterniond::Identity()});
 if(i>=30&&(i-30)%20==0)b.add_lio({t-150000000,{(i-30)*.01,0,0},Eigen::Quaterniond::Identity()});
 ASSERT_TRUE(a.add_imu({t,{0,0,1},{0,0,0}}));ASSERT_TRUE(b.add_imu({t,{0,0,1},{0,0,0}}));}
 EXPECT_NEAR(a.latest().vx,b.latest().vx,1e-8);EXPECT_GT(b.replayed_imus(),0u);
 EXPECT_TRUE(b.add_lio({2999999999LL,{100,0,0},Eigen::Quaterniond::Identity()}));EXPECT_GE(b.latest().rejected,1u);EXPECT_NEAR(b.latest().vx,2,1e-8);
 EXPECT_FALSE(b.add_lio({500000000,{0,0,0},Eigen::Quaterniond::Identity()}));
}
TEST(Velocity,ProjectedOutputMustBeFinite){Config c;c.mounting=Eigen::AngleAxisd(std::acos(-1.)/4,V3::UnitZ()).toRotationMatrix();c.lever.setZero();Observer o(c);const V3 v(1.5*std::pow(2.,1023),-1.5*std::pow(2.,1023),0);
for(int i=0;i<=100;++i){int64_t t=1000000000LL+i*5000000LL;if(i%50==0)o.add_lio({t,v*(i*.005),Eigen::Quaterniond::Identity()});o.add_imu({t,{0,0,1},{0,0,0}});}const auto e=o.latest();EXPECT_TRUE(!e.valid||std::isfinite(e.vx));}
TEST(Velocity,LioOnlyAfterImuOutageRetainsTwoSeconds){Observer o;o.add_imu({1000000000,{0,0,1},{0,0,0}});for(int i=0;i<100;++i)o.add_lio({1000000000LL+i*100000000LL,{0,0,0},Eigen::Quaterniond::Identity()});EXPECT_LE(o.history_size(),22u);}
TEST(Velocity,DenseSourceStampsAndGapBoundary){Observer o;drive(o,200);for(int i=1;i<=100;++i)ASSERT_TRUE(o.add_imu({2000000000LL+i,{0,0,1},{0,0,0}}));EXPECT_TRUE(o.latest().valid);EXPECT_NEAR(o.latest().vx,2,1e-8);ASSERT_TRUE(o.add_imu({2030000100LL,{0,0,1},{0,0,0}}));EXPECT_TRUE(o.latest().valid);ASSERT_TRUE(o.add_imu({2060000101LL,{0,0,1},{0,0,0}}));EXPECT_FALSE(o.latest().valid);}
TEST(Velocity,ReacquireAfterExpiredAnchorRequiresThreePoses){Observer o;drive(o,200);for(int i=1;i<=140;++i)ASSERT_TRUE(o.add_imu({2000000000LL+i*5000000LL,{0,0,1},{0,0,0}}));ASSERT_FALSE(o.latest().valid);
for(int i=2;i<=42;++i){int64_t t=2700000000LL+i*5000000LL;if(i==2||i==22||i==42)o.add_lio({t,{100.+(t-2710000000LL)*2e-9,0,0},Eigen::Quaterniond::Identity()});o.add_imu({t,{0,0,1},{0,0,0}});if(i<42)EXPECT_FALSE(o.latest().valid);}EXPECT_TRUE(o.latest().valid);EXPECT_NEAR(o.latest().vx,2,1e-8);EXPECT_EQ(o.latest().reacquisitions,1u);}

TEST(Velocity, BodyVectorProjectionCovarianceAndPointLeverSigns) {
  Config c;
  c.mounting=Eigen::AngleAxisd(.31,V3(1,2,3).normalized()).toRotationMatrix();
  c.lever=V3(.30,-.02,.03);
  const Eigen::Quaterniond q(Eigen::AngleAxisd(.67,V3(2,-1,3).normalized()));
  const V3 world_velocity(2.,-.7,.3),raw_force=q.conjugate()*V3(0,0,1);
  Observer observer(c);
  for(int i=0;i<=40;++i) {
    const int64_t stamp=1000000000LL+i*5000000LL;
    if(i%20==0) ASSERT_TRUE(observer.add_lio({stamp,world_velocity*(i*.005),q}));
    const V3 gyro=i==40?V3(.2,-.1,.8):V3::Zero();
    ASSERT_TRUE(observer.add_imu({stamp,raw_force,gyro}));
  }
  const auto e=observer.latest();ASSERT_TRUE(e.valid);
  const M3 projection=c.mounting*q.toRotationMatrix().transpose();
  const V3 omega=c.mounting*V3(.2,-.1,.8),imu_velocity=projection*e.v;
  EXPECT_LT((e.imu_velocity_body-imu_velocity).norm(),1e-10);
  EXPECT_LT((e.omega_body-omega).norm(),1e-12);
  EXPECT_LT((e.velocity_body-(imu_velocity-omega.cross(c.lever))).norm(),1e-10);
  EXPECT_GT(std::abs(e.velocity_body.y()),.1);
  EXPECT_DOUBLE_EQ(e.vx,e.velocity_body.x());
  EXPECT_NEAR(e.variance,std::max(e.velocity_covariance_body(0,0),1e-4),1e-12);
  EXPECT_LT((e.velocity_covariance_body-e.velocity_covariance_body.transpose()).norm(),1e-12);
  EXPECT_GT(e.velocity_covariance_body.diagonal().minCoeff(),0.);
  EXPECT_LT((e.velocity_covariance_body-M3::Identity()*(c.position_std*c.position_std/.02)).norm(),1e-10);
  const V3 imu_to_cg(-.09,-.03,-.03);
  const V3 cg=e.imu_velocity_body+e.omega_body.cross(imu_to_cg);
  EXPECT_LT((cg-e.velocity_body-e.omega_body.cross(imu_to_cg+c.lever)).norm(),1e-12);
}

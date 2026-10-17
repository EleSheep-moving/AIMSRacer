#include "aims_racer_system/body_acceleration.hpp"
#include <gtest/gtest.h>
#include <cmath>
using namespace aims_racer_system::body_acceleration;
using V = Eigen::Vector3d;
TEST(BodyAcceleration,CoastingKeepsCompensationAndInflatesModelUncertainty){
 Config c;c.continue_on_lio_timeout=true;c.lateral_enabled=true;Observer o(c);
 const int64_t t0=1000000000LL;o.add_lio({t0,Eigen::Quaterniond::Identity()});
 for(int i=0;i<=60;++i)ASSERT_TRUE(o.add_imu({t0+i*5000000LL,V(.1,.2,1),V::Zero()}));
 const auto before=o.latest();ASSERT_TRUE(before.valid);
 for(int i=61;i<=200;++i)ASSERT_TRUE(o.add_imu({t0+i*5000000LL,V(.1,.2,1),V::Zero()}));
 const auto after=o.latest();EXPECT_TRUE(after.valid);EXPECT_TRUE(after.lateral_valid);EXPECT_EQ(after.status,Status::lio_stale);
 EXPECT_NEAR(after.ax,.1*c.gravity,1e-10);EXPECT_NEAR(after.ay,.2*c.gravity,1e-10);
 EXPECT_GT(after.measurement_variance,before.measurement_variance);
 ASSERT_TRUE(o.add_lio({t0+1000000000LL,Eigen::Quaterniond::Identity()}));EXPECT_EQ(o.latest().status,Status::valid);
}
TEST(BodyAcceleration, StaticTiltAndTranslation) {
  Config c; c.filter_enabled = false;
  Observer o(c);
  Eigen::Quaterniond q(Eigen::AngleAxisd(0.3,V::UnitY()));
  o.add_lio({1000000000,q});
  V gravity = q.conjugate()*V(0,0,-c.gravity);
  for (int i=0;i<6;++i) ASSERT_TRUE(o.add_imu({1000000000+i*5000000LL,-gravity/c.gravity,V::Zero()}));
  ASSERT_TRUE(o.latest().valid);
  EXPECT_LT(o.latest().acceleration_body.norm(),1e-10);
  o.add_imu({1030000000,(-gravity+V(2,0,0))/c.gravity,V::Zero()});
  EXPECT_NEAR(o.latest().ax,2,1e-10);
}
TEST(BodyAcceleration, RotationLeverAndPitchDynamics) {
  Config c; c.filter_enabled=false; Observer o(c);
  const int64_t t0=2000000000;
  o.add_lio({t0,Eigen::Quaterniond::Identity()});
  const V alpha(0,1.2,0);
  for(int i=0;i<30;++i) {
    const double t=i*.005; V omega=alpha*t;
    Eigen::Quaterniond q(Eigen::AngleAxisd(.5*alpha.y()*t*t,V::UnitY()));
    V gravity=q.conjugate()*V(0,0,-c.gravity);
    V force=-gravity+alpha.cross(c.lever)+omega.cross(omega.cross(c.lever));
    o.add_imu({t0+i*5000000LL,force/c.gravity,omega});
  }
  ASSERT_TRUE(o.latest().valid);
  EXPECT_LT(o.latest().acceleration_body.norm(),1e-8);
  EXPECT_LT((o.latest().alpha_body-alpha).norm(),1e-9);
  Observer spin(c); spin.add_lio({t0,Eigen::Quaterniond::Identity()});
  const V omega(0,0,3),centripetal=omega.cross(omega.cross(c.lever));
  for(int i=0;i<10;++i) spin.add_imu({t0+i*5000000LL,(V(0,0,c.gravity)+centripetal)/c.gravity,omega});
  EXPECT_LT(spin.latest().acceleration_body.norm(),1e-9);
  EXPECT_LT((spin.latest().centripetal-centripetal).norm(),1e-9);
}
TEST(BodyAcceleration, DelayedInteriorLioReplaysPhysicalAndFilterState) {
  Config c; c.max_lio_age_s=1; Observer ordered(c), delayed(c);
  const int64_t t0=3000000000;
  ordered.add_lio({t0,Eigen::Quaterniond::Identity()}); delayed.add_lio({t0,Eigen::Quaterniond::Identity()});
  const int64_t anchor=t0+82500000;
  Eigen::Quaterniond correction(Eigen::AngleAxisd(.13,V::UnitY()));
  ordered.add_lio({anchor,correction});
  for(int i=0;i<60;++i) {
    ImuSample m{t0+i*5000000LL,V(.03*std::sin(i*.1),0,1),V(0,.1,0)};
    ordered.add_imu(m); delayed.add_imu(m);
  }
  const auto immutable=delayed.latest();
  ASSERT_TRUE(delayed.add_lio({anchor,correction}));
  const auto a=ordered.latest(),b=delayed.latest();
  ASSERT_TRUE(a.valid); ASSERT_TRUE(b.valid);
  EXPECT_EQ(a.stamp_ns,b.stamp_ns);
  EXPECT_NEAR(a.ax,b.ax,1e-12); EXPECT_NEAR(a.jerk,b.jerk,1e-12);
  EXPECT_LT((a.gravity_body-b.gravity_body).norm(),1e-12);
  EXPECT_GT(std::abs(immutable.ax-b.ax),.5);
  EXPECT_EQ(delayed.counters().replays,1u);
}
TEST(BodyAcceleration, DelayedImuMatchesOrderedAndFutureAnchorDoesNotLeak) {
  Observer a,b; const int64_t t0=4000000000;
  a.add_lio({t0,Eigen::Quaterniond::Identity()}); b.add_lio({t0,Eigen::Quaterniond::Identity()});
  ImuSample skipped;
  for(int i=0;i<20;++i) {
    ImuSample m{t0+i*5000000LL,V(i*.01,0,1),V(0,0,.1*i)};
    a.add_imu(m); if(i==8)skipped=m;else b.add_imu(m);
  }
  b.add_imu(skipped);
  EXPECT_NEAR(a.latest().ax,b.latest().ax,1e-12);
  const auto previous=b.latest();
  b.add_lio({t0+200000000,Eigen::Quaterniond(Eigen::AngleAxisd(.4,V::UnitY()))});
  EXPECT_DOUBLE_EQ(previous.ax,b.latest().ax);
  EXPECT_EQ(previous.stamp_ns,b.latest().stamp_ns);
}
TEST(BodyAcceleration, GapRequiresFreshCoveredAnchorAndWarmup) {
  Config c; c.filter_enabled=false; Observer o(c); const int64_t t0=5000000000;
  o.add_lio({t0,Eigen::Quaterniond::Identity()});
  for(int i=0;i<4;++i)o.add_imu({t0+i*5000000LL,V(0,0,1),V::Zero()});
  ASSERT_TRUE(o.latest().valid);
  o.add_imu({t0+100000000,V(0,0,1),V::Zero()});
  EXPECT_EQ(o.latest().status,Status::gyro_gap);
  for(int i=21;i<25;++i)o.add_imu({t0+i*5000000LL,V(0,0,1),V::Zero()});
  EXPECT_FALSE(o.latest().valid);
  o.add_lio({t0+115000000,Eigen::Quaterniond::Identity()});
  EXPECT_TRUE(o.latest().valid);
  o.add_imu({t0+700000000,V(0,0,1),V::Zero()});
  EXPECT_FALSE(o.latest().valid);
}
TEST(BodyAcceleration, StaleAndInvalidInputsAndBiasUnitsMounting) {
  Config c; c.filter_enabled=false; c.max_lio_age_s=.02;
  c.mounting=Eigen::Quaterniond(Eigen::AngleAxisd(.2,V::UnitX()));
  c.accel_bias_body=V(.3,.1,0); Observer o(c);
  const int64_t t0=6000000000; o.add_lio({t0,Eigen::Quaterniond::Identity()});
  const V raw=c.mounting.conjugate()*(V(2,0,c.gravity)+c.accel_bias_body)/c.gravity;
  for(int i=0;i<5;++i)o.add_imu({t0+i*5000000LL,raw,V::Zero()});
  EXPECT_NEAR(o.latest().ax,2,1e-10);
  EXPECT_GE(o.latest().acceleration_variance,c.force_variance);
  EXPECT_FALSE(o.add_imu({t0+20000000,raw,V::Zero()}));
  o.add_imu({t0+25000000,raw,V::Zero()}); EXPECT_EQ(o.latest().status,Status::lio_stale);
  const auto previous=o.latest();
  EXPECT_FALSE(o.add_imu({t0+30000000,V(NAN,0,0),V::Zero()}));
  EXPECT_FALSE(o.add_lio({t0+30000000,Eigen::Quaterniond(0,0,0,0)}));
  EXPECT_EQ(previous.stamp_ns,o.latest().stamp_ns);
  EXPECT_EQ(o.counters().invalid,2u); EXPECT_EQ(o.counters().duplicate,1u);
  c.gravity=1; EXPECT_THROW(Observer bad(c),std::invalid_argument);
  c.gravity=9.80665; c.mounting=Eigen::Quaterniond(2,0,0,0);
  EXPECT_THROW(Observer bad(c),std::invalid_argument);
}
TEST(BodyAcceleration, BoundedHistoryTooOldResetAndIncrementalWork) {
  Config c; c.max_events=32; Observer o(c); const int64_t t0=7000000000;
  o.add_lio({t0,Eigen::Quaterniond::Identity()});
  for(int i=0;i<2000;++i) {
    if(i%20==0)o.add_lio({t0+i*5000000LL,Eigen::Quaterniond::Identity()});
    o.add_imu({t0+i*5000000LL,V(0,0,1),V::Zero()});
    ASSERT_LE(o.history_size(),c.max_events);
  }
  EXPECT_TRUE(o.latest().valid); EXPECT_EQ(o.counters().replayed_imu,0u);
  EXPECT_FALSE(o.add_lio({t0,Eigen::Quaterniond::Identity()}));
  EXPECT_FALSE(o.add_imu({t0,V(0,0,1),V::Zero()}));
  EXPECT_EQ(o.counters().too_old,2u);
  o.reset(); EXPECT_EQ(o.latest().status,Status::no_imu); EXPECT_EQ(o.history_size(),0u);
  for(int i=0;i<100;++i)o.add_lio({t0+i*5000000LL,Eigen::Quaterniond::Identity()});
  EXPECT_LE(o.history_size(),c.max_events);
}
TEST(BodyAcceleration, FilterSyntheticGainAndPhase) {
  constexpr double pi=3.14159265358979323846,dt=.005;
  for(double frequency:{.2,.5,1.,2.,3.}) {
    Observer o; double ss=0,cc=0,ys=0,yc=0;
    const int n=static_cast<int>(std::max(12.0,8.0/frequency)/dt);
    for(int i=0;i<n;++i) {
      int64_t stamp=8000000000LL+i*5000000LL;
      if(i%20==0)o.add_lio({stamp,Eigen::Quaterniond::Identity()});
      const double phase=2*pi*frequency*i*dt;
      o.add_imu({stamp,V(std::sin(phase)/9.80665,0,1),V::Zero()});
      if(i>n/2&&o.latest().valid) {
        const double s=std::sin(phase),co=std::cos(phase);
        ss+=s*s; cc+=co*co; ys+=o.latest().ax*s; yc+=o.latest().ax*co;
      }
    }
    const double gain=std::hypot(ys/ss,yc/cc);
    const double delay=-std::atan2(yc/cc,ys/ss)/(2*pi*frequency);
    std::cout<<"FILTER f="<<frequency<<" gain="<<gain<<" delay_s="<<delay<<'\n';
    EXPECT_LE(std::abs(gain-1),.1); EXPECT_LE(std::abs(delay),.020);
  }
}
TEST(BodyAcceleration, NumericalOverflowCannotProduceValidEstimate) {
  Observer o; const int64_t t0=10000000000;
  o.add_lio({t0,Eigen::Quaterniond::Identity()});
  for(int i=0;i<4;++i)o.add_imu({t0+i*5000000LL,V(0,0,1),V::Zero()});
  ASSERT_TRUE(o.latest().valid);
  const auto previous=o.latest(); const auto size=o.history_size();
  EXPECT_FALSE(o.add_imu({t0+20000000,V(1e308,0,1),V::Zero()}));
  EXPECT_TRUE(o.latest().valid); EXPECT_EQ(o.latest().stamp_ns,previous.stamp_ns);
  EXPECT_DOUBLE_EQ(o.latest().ax,previous.ax); EXPECT_EQ(o.history_size(),size);
  EXPECT_FALSE(o.add_imu({t0+20000000,V(0,0,1),V(1e200,0,0)}));
  EXPECT_EQ(o.latest().stamp_ns,previous.stamp_ns);
  EXPECT_EQ(o.counters().numerical_error,2u);
  Config bad; bad.lever=V(1e308,0,0); EXPECT_THROW(Observer invalid(bad),std::invalid_argument);
}

TEST(BodyAcceleration, LateAnchorReplaysOnlyAffectedSuffix) {
  Config c; c.max_lio_age_s=2; Observer o(c);
  const int64_t t0=11000000000;
  o.add_lio({t0,Eigen::Quaterniond::Identity()});
  for(int i=0;i<300;++i)o.add_imu({t0+i*5000000LL,V(0,0,1),V::Zero()});
  const auto before=o.counters();
  o.add_lio({t0+1402500000,Eigen::Quaterniond(Eigen::AngleAxisd(.1,V::UnitY()))});
  EXPECT_EQ(o.counters().replayed_imu-before.replayed_imu,19u);
  EXPECT_TRUE(o.latest().valid);
}

TEST(BodyAcceleration, FilterReducesSyntheticWhiteForceNoise) {
  Observer filtered; Config raw_config; raw_config.filter_enabled=false; Observer raw(raw_config);
  uint32_t seed=42; double filtered_sum=0,raw_sum=0; int count=0;
  const int64_t t0=13000000000;
  for(int i=0;i<12000;++i) {
    const int64_t stamp=t0+i*5000000LL;
    if(i%20==0) {
      filtered.add_lio({stamp,Eigen::Quaterniond::Identity()});
      raw.add_lio({stamp,Eigen::Quaterniond::Identity()});
    }
    seed=1664525u*seed+1013904223u;
    const double noise=(static_cast<double>(seed)/4294967296.0-.5)*.2;
    const ImuSample m{stamp,V(noise/9.80665,0,1),V::Zero()};
    filtered.add_imu(m); raw.add_imu(m);
    if(i>1000) { filtered_sum+=filtered.latest().ax*filtered.latest().ax;
      raw_sum+=raw.latest().ax*raw.latest().ax; ++count; }
  }
  const double ratio=std::sqrt(filtered_sum/raw_sum);
  std::cout<<"FILTER synthetic_white_noise_std_ratio="<<ratio<<" samples="<<count<<'\n';
  EXPECT_LT(ratio,.8);
}
TEST(BodyAcceleration, AllAxesRotationNonidentityMountAndPose) {
  Config c; c.filter_enabled=false; c.lever=V(.3,-.08,.03);
  c.mounting=Eigen::Quaterniond(Eigen::AngleAxisd(.32,V(1,2,3).normalized()));
  c.accel_bias_body=V(.03,-.02,.01);
  const Eigen::Quaterniond q0(Eigen::AngleAxisd(.41,V(3,1,-2).normalized()));
  const V omega(1.2,-.7,2.1); Observer observer(c);
  const int64_t t0=14000000000;
  observer.add_lio({t0,q0});
  for(int i=0;i<80;++i) {
    const double t=i*.005;
    const Eigen::Quaterniond q=q0*Eigen::Quaterniond(Eigen::AngleAxisd(omega.norm()*t,omega.normalized()));
    const V gravity=q.conjugate()*V(0,0,-c.gravity);
    const V force=-gravity+omega.cross(omega.cross(c.lever))+c.accel_bias_body;
    observer.add_imu({t0+i*5000000LL,c.mounting.conjugate()*force/c.gravity,c.mounting.conjugate()*omega});
  }
  ASSERT_TRUE(observer.latest().valid);
  EXPECT_LT(observer.latest().acceleration_body.norm(),1e-10);
  EXPECT_LT((observer.latest().omega_body-omega).norm(),1e-12);
}
TEST(BodyAcceleration, AngularRegressionExposesSampleSupport) {
  Config c; c.filter_enabled=false; Observer observer(c);
  const int64_t t0=15000000000;
  observer.add_lio({t0,Eigen::Quaterniond::Identity()});
  EXPECT_DOUBLE_EQ(observer.latest().alpha_window_span_s,0.0);
  EXPECT_DOUBLE_EQ(observer.latest().alpha_sample_center_age_s,0.0);
  for(int i=0;i<6;++i)
    observer.add_imu({t0+i*5000000LL,V(0,0,1),V(0,0,i*.01)});
  ASSERT_TRUE(observer.latest().valid);
  EXPECT_NEAR(observer.latest().alpha_window_span_s,.025,1e-12);
  EXPECT_NEAR(observer.latest().alpha_sample_center_age_s,.0125,1e-12);
}

TEST(PlanarAcceleration, RollRestAndCircularMotionHavePhysicalLateralMeaning) {
  Config c; c.lateral_enabled=true; const int64_t t0=15000000000LL;
  Observer rest(c);Eigen::Quaterniond q(Eigen::AngleAxisd(.18,V::UnitX()));
  rest.add_lio({t0,q});const V force=q.conjugate()*V(0,0,1);
  for(int i=0;i<30;++i)ASSERT_TRUE(rest.add_imu({t0+i*5000000LL,force,V::Zero()}));
  ASSERT_TRUE(rest.latest().lateral_valid);EXPECT_NEAR(rest.latest().ay,0,1e-10);
  Observer circle(c);circle.add_lio({t0,Eigen::Quaterniond::Identity()});
  for(int i=0;i<80;++i)ASSERT_TRUE(circle.add_imu({t0+i*5000000LL,V(-.3,4,c.gravity)/c.gravity,V(0,0,1)}));
  EXPECT_NEAR(circle.latest().ay,4,1e-10);EXPECT_NEAR(circle.latest().ax,0,1e-10);
}
TEST(PlanarAcceleration, TangentialLeverCorrectionDoesNotBecomeLateralG) {
  Config c;c.lateral_enabled=true;Observer o(c);const int64_t t0=16000000000LL;
  o.add_lio({t0,Eigen::Quaterniond::Identity()});
  for(int i=0;i<70;++i){const double r=.2+2*i*.005;
    ASSERT_TRUE(o.add_imu({t0+i*5000000LL,V(-r*r*.3,.6,c.gravity)/c.gravity,V(0,0,r)}));}
  EXPECT_NEAR(o.latest().tangential.y(),.6,1e-10);EXPECT_NEAR(o.latest().ay,0,1e-10);
}
TEST(PlanarAcceleration, MatchedAxesAndLegacyAxAreIdentical) {
  Config c;c.max_lio_age_s=2.;Observer legacy(c);c.lateral_enabled=true;Observer planar(c);
  const int64_t t0=17000000000LL;legacy.add_lio({t0,Eigen::Quaterniond::Identity()});planar.add_lio({t0,Eigen::Quaterniond::Identity()});
  for(int i=0;i<300;++i){const double f=std::sin(i*.04);ImuSample m{t0+i*5000000LL,V(f,f,1),V::Zero()};
    ASSERT_TRUE(legacy.add_imu(m));ASSERT_TRUE(planar.add_imu(m));auto a=legacy.latest(),b=planar.latest();
    EXPECT_DOUBLE_EQ(a.ax,b.ax);EXPECT_DOUBLE_EQ(a.jerk,b.jerk);EXPECT_DOUBLE_EQ(a.acceleration_variance,b.acceleration_variance);
    EXPECT_FALSE(a.lateral_valid);
    if(b.valid){EXPECT_TRUE(b.lateral_valid);EXPECT_DOUBLE_EQ(b.ax,b.ay);EXPECT_DOUBLE_EQ(b.jerk,b.lateral_jerk);}
  }
}
TEST(PlanarAcceleration, LateRollReplaysLateralFilterAndOldSnapshotsStayImmutable) {
  Config c;c.lateral_enabled=true;c.max_lio_age_s=1;Observer ordered(c),late(c);const int64_t t0=19000000000LL;
  const LioSample first{t0,Eigen::Quaterniond::Identity()},anchor{t0+82500000,Eigen::Quaterniond(Eigen::AngleAxisd(.15,V::UnitX()))};
  ordered.add_lio(first);late.add_lio(first);ordered.add_lio(anchor);
  for(int i=0;i<60;++i){ImuSample m{t0+i*5000000LL,V(0,.02*std::sin(i*.1),1),V(.08,0,0)};ordered.add_imu(m);late.add_imu(m);}
  auto before=late.latest();ASSERT_TRUE(late.add_lio(anchor));auto a=ordered.latest(),b=late.latest();
  ASSERT_TRUE(b.lateral_valid);EXPECT_NEAR(a.ay,b.ay,1e-12);EXPECT_NEAR(a.lateral_jerk,b.lateral_jerk,1e-12);
  EXPECT_GT(std::abs(before.ay-b.ay),1.);EXPECT_EQ(before.stamp_ns,b.stamp_ns);
}

TEST(BodyAcceleration, TinyGyroTimeSupportIsInvalidAndRecoversWithoutTerminalHistory) {
  Config c;c.lateral_enabled=true;c.min_alpha_span_s=.010;
  Observer observer(c);const int64_t t0=22000000000LL;
  ASSERT_TRUE(observer.add_lio({t0,Eigen::Quaterniond::Identity()}));
  for(int i=0;i<3;++i)ASSERT_TRUE(observer.add_imu({t0+i*1000,V(0,0,1),V(0,0,i*.01)}));
  const auto unsupported=observer.latest();
  EXPECT_FALSE(unsupported.valid);EXPECT_FALSE(unsupported.lateral_valid);
  EXPECT_EQ(unsupported.status,Status::gyro_time_support);
  EXPECT_STREQ(status_name(unsupported.status),"gyro_time_support");
  EXPECT_DOUBLE_EQ(unsupported.alpha_body.norm(),0.);
  // New normal-rate samples provide support, and the first valid acceleration
  // initializes the terminal filter directly rather than retaining a huge derivative.
  for(int i=1;i<=2;++i)ASSERT_TRUE(observer.add_imu({t0+i*5000000LL,V(0,0,1),V::Zero()}));
  const auto recovered=observer.latest();ASSERT_TRUE(recovered.valid);
  EXPECT_GE(recovered.alpha_window_span_s,.010-1e-12);
  EXPECT_DOUBLE_EQ(recovered.ax,recovered.acceleration_body.x());
  EXPECT_DOUBLE_EQ(recovered.ay,recovered.acceleration_body.y());
  EXPECT_DOUBLE_EQ(recovered.jerk,0.);EXPECT_DOUBLE_EQ(recovered.lateral_jerk,0.);
  for(int i=3;i<=6;++i)ASSERT_TRUE(observer.add_imu({t0+i*5000000LL,V(0,0,1),V::Zero()}));
  for(int i=1;i<=3;++i)ASSERT_TRUE(observer.add_imu({t0+60000000LL+i*1000,V(0,0,1),V(0,0,i*.01)}));
  EXPECT_FALSE(observer.latest().valid);
}
TEST(BodyAcceleration, PointReferenceCgLeverSignIncludesTangentialAndCentripetalTerms) {
  Config rear;rear.filter_enabled=false;rear.lateral_enabled=true;
  Config cg=rear;const V imu_to_cg(-.09,-.03,-.03);cg.lever=-imu_to_cg;
  Observer a(rear),b(cg);const int64_t t0=23000000000LL;
  a.add_lio({t0,Eigen::Quaterniond::Identity()});b.add_lio({t0,Eigen::Quaterniond::Identity()});
  const V alpha(0,0,2.);
  for(int i=0;i<8;++i) {
    const V omega(0,0,.5+i*.01);const ImuSample sample{t0+i*5000000LL,V(0,0,1),omega};
    ASSERT_TRUE(a.add_imu(sample));ASSERT_TRUE(b.add_imu(sample));
  }
  const auto r=a.latest(),g=b.latest();ASSERT_TRUE(r.valid);ASSERT_TRUE(g.lateral_valid);
  const V rear_to_cg=rear.lever+imu_to_cg;
  const V expected=alpha.cross(rear_to_cg)+r.omega_body.cross(r.omega_body.cross(rear_to_cg));
  EXPECT_LT((g.acceleration_body-r.acceleration_body-expected).norm(),1e-10);
  EXPECT_NEAR(g.ax,g.acceleration_body.x(),1e-12);EXPECT_NEAR(g.ay,g.acceleration_body.y(),1e-12);
}
TEST(BodyAcceleration, RejectInvalidMinimumAngularSupportConfiguration) {
  Config c;c.min_alpha_span_s=-.001;EXPECT_THROW(Observer o(c),std::invalid_argument);
  c.min_alpha_span_s=std::nan("");EXPECT_THROW(Observer o(c),std::invalid_argument);
  c.min_alpha_span_s=c.alpha_window_s+.001;EXPECT_THROW(Observer o(c),std::invalid_argument);
}

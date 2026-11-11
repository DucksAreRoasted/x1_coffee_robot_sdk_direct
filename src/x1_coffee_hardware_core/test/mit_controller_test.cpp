#include <gtest/gtest.h>
#include "x1_coffee_hardware_core/mit_controller.hpp"
using namespace x1_coffee_hardware_core;
namespace {
MITLimits limits(){return {-.2,.2,2,10,100,100,20,1,2,100,100,100};}
MITConfig config(){MITConfig c; c.transition_verified=true;c.max_dt=.002;c.lease_sec=.1;c.mode_timeout=.2;c.quiet_velocity=.02;c.quiet_sec=.05; for(size_t i=0;i<14;++i){c.limits[i]=limits();c.conversion[i]={.01,1000,true};c.initial[i]={0,0,0,0,0};}return c;}
MITFeedback feedback(){MITFeedback f;f.bus_op=f.all_enabled=f.all_valid=f.waist_csp=true;for(size_t i=0;i<14;++i){f.mode[i]=8;f.joints[i]={0,0,0,true};}return f;}
TEST(MITController, FormulaAndRateLimit){MITController c;auto l=limits();ASSERT_TRUE(c.configure(l));ASSERT_TRUE(c.seed({0,0,0,true}));MITCommand d{.1,0,2,1,0};auto r=c.compute(d,{0,0,0,true},.001);EXPECT_EQ(r.fault,MITFault::none);EXPECT_LE(std::abs(r.torque),.1+1e-9);EXPECT_GT(r.torque,0);}
TEST(MITController, RejectsBadInput){MITController c;ASSERT_TRUE(c.configure(limits()));ASSERT_TRUE(c.seed({0,0,0,true}));auto r=c.compute({.3,0,1,1,0},{0,0,0,true},.001);EXPECT_EQ(r.fault,MITFault::input);}
TEST(MITController, ConversionMustBeVerified){MITConversion x{.01,1000,false};std::int16_t out=0;EXPECT_FALSE(x.toPdo(1,&out));x.verified=true;EXPECT_TRUE(x.toPdo(1,&out));EXPECT_EQ(out,100);}
TEST(MITController, GainsAreOnlyInitialEstimate){double kp=0,kd=0;EXPECT_TRUE(MITController::initialGains(2,.2,3,.8,&kp,&kd));EXPECT_DOUBLE_EQ(kp,18);EXPECT_DOUBLE_EQ(kd,9.4);EXPECT_FALSE(MITController::initialGains(2,10,3,.8,&kp,&kd));}
TEST(MITSession, FeedbackFaultLatches){auto c=config();c.lease_sec=.5;MITSession s;ASSERT_TRUE(s.configure(c));auto f=feedback();ASSERT_TRUE(s.start(0x3fff,f,1));for(auto &m:f.mode)m=10;auto o=s.step(f,1.001);EXPECT_FALSE(o.shutdown);EXPECT_EQ(s.status().phase,MITPhase::active);f.all_valid=false;o=s.step(f,1.002);EXPECT_TRUE(o.shutdown);EXPECT_EQ(s.status().phase,MITPhase::fault);EXPECT_TRUE(s.status().fault!=MITFault::none);}
TEST(MITSession, ModeAndWaistAreGated){auto c=config();MITSession s;ASSERT_TRUE(s.configure(c));auto f=feedback();f.waist_csp=false;EXPECT_FALSE(s.start(1,f,1));}
}

/**
 * @brief       Implementation for testing Lift drag model. Canonical values obtained from Conrad McGreal's MATLAB hand calcs. Values for CL, CD, Lift, Drag, correspond to the first entries in their respective arrays output from the script. vInf and alpha values are derived from sample runs of gzserver with the lift drag enhanced plugin active. Note: lookupCL and lookupCD functions will not be tested because the only line of code executed is a call to avionics_sim::Bilinear_interp::interpolate, which is tested elsewhere in Test_Bilinear_interp.cpp. The matching getCL and getCD methods will also be untested as they are simple etter functions. calculateLiftDragModelValues also not tested because it is an aggregate function that executes calculateDynamicPressure(), lookupCL(), lookupCD(), calculateLift() and calculateDrag(). If these functions pass tests, so too will it. Edge cases omitted for now: will need to test edge cases in MATLAB first to obtain canonical values before testing here.
 * @file        Test_Lift_drag_model.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright(C) 2019, Swift Engineering Inc. All rights reserved.
 */

/*
Methodology for unit testing:
1) All public methods (set/gets should only be tested if something more than setting/getting of values is done. For example, here, set and get alpha is tested because the alpha can be multiplied by a constant to get its degree value, which is eventually stored.)
2) All branches of code execution (All acceptable input variations; All methods must be called by the end of the tests).
3) Unexpected inputs (Esp for those that have direct user exposure, such as through a controller in an MVC framework).
4) Edge cases
*/
#include <gtest/gtest.h>
#include <boost/array.hpp>
#include "avionics_sim/Lift_drag_model.hpp"

//Accurate within a thousandth. Decreasing this value increases rates of test failure.
float acceptable_error=10E-3;
const float rho=1.225;
const float canonicalCl=0.7542;
const float canonicalCd=0.0598;
const std::vector<float> LUT_alpha{-180.0000,-175.0000,-170.0000,-165.0000,-160.0000,-155.0000,-150.0000,-145.0000,-140.0000,-135.0000,-130.0000,-125.0000,-120.0000,-115.0000,-110.0000,-105.0000,-100.0000,-95.0000,-90.0000,-85.0000,-80.0000,-75.0000,-70.0000,-65.0000,-60.0000,-55.0000,-50.0000,-45.0000,-40.0000,-35.0000,-30.0000,-27.0000,-26.0000,-25.0000,-24.0000,-23.0000,-22.0000,-21.0000,-20.0000,-19.0000,-18.0000,-17.0000,-16.0000,-15.0000,-14.0000,-13.0000,-12.0000,-11.0000,-10.0000,-9.0000,-8.0000,-7.0000,-6.0000,-5.0000,-4.0000,-3.0000,-2.0000,-1.0000,-0.0000,0.0000,1.0000,2.0000,3.0000,4.0000,5.0000,6.0000,7.0000,8.0000,9.0000,10.0000,11.0000,12.0000,13.0000,14.0000,15.0000,16.0000,17.0000,18.0000,19.0000,20.0000,21.0000,22.0000,23.0000,24.0000,25.0000,26.0000,27.0000,30.0000,35.0000,40.0000,45.0000,50.0000,55.0000,60.0000,65.0000,70.0000,75.0000,80.0000,85.0000,90.0000,95.0000,100.0000,105.0000,110.0000,115.0000,120.0000,125.0000,130.0000,135.0000,140.0000,145.0000,150.0000,155.0000,160.0000,165.0000,170.0000,175.0000,180.0000 }; 

const std::vector<float> LUT_CL{ -0.0000,0.6900,0.8500,0.6750,0.6600,0.7400,0.8500,0.9100,0.9450,0.9450,0.9100,0.8400,0.7350,0.6250,0.5100,0.3700,0.2200,0.0700,-0.0700,-0.2200,-0.3700,-0.5150,-0.6500,-0.7650,-0.8750,-0.9650,-1.0400,-1.0850,-1.0750,-1.0200,-0.9150,-0.9646,-0.9109,-0.8572,-0.8034,-0.7497,-0.6956,-0.6414,-0.5870,-0.5322,-0.4768,-0.4200,-0.3620,-0.3082,-0.2546,-0.2030,-0.1533,-0.1095,-0.1325,-0.8527,-0.8274,-0.7460,-0.6600,-0.5500,-0.4400,-0.3300,-0.2200,-0.1100,-0.0000,0.0000,0.1100,0.2200,0.3300,0.4400,0.5500,0.6600,0.7460,0.8274,0.8527,0.1325,0.1095,0.1533,0.2030,0.2546,0.3082,0.3620,0.4200,0.4768,0.5322,0.5870,0.6414,0.6956,0.7497,0.8034,0.8572,0.9109,0.9646,0.9150,1.0200,1.0750,1.0850,1.0400,0.9650,0.8750,0.7650,0.6500,0.5150,0.3700,0.2200,0.0700,-0.0700,-0.2200,-0.3700,-0.5100,-0.6250,-0.7350,-0.8400,-0.9100,-0.9450,-0.9450,-0.9100,-0.8500,-0.7400,-0.6600,-0.6750,-0.8500,-0.6900,0.0000 }; 

const std::vector<float> LUT_CD{ 0.0250,0.0550,0.1400,0.2300,0.3200,0.4200,0.5700,0.7550,0.9250,1.0850,1.2250,1.3500,1.4650,1.5550,1.6350,1.7000,1.7500,1.7800,1.8000,1.8000,1.7800,1.7350,1.6650,1.5750,1.4700,1.3450,1.2150,1.0750,0.9200,0.7450,0.5700,0.4730,0.4460,0.4200,0.3940,0.3690,0.3440,0.3200,0.2970,0.2740,0.2520,0.2310,0.2100,0.1900,0.1710,0.1520,0.1340,0.0760,0.0188,0.0203,0.0185,0.0170,0.0152,0.0140,0.0124,0.0114,0.0108,0.0104,0.0103,0.0103,0.0104,0.0108,0.0114,0.0124,0.0140,0.0152,0.0170,0.0185,0.0203,0.0188,0.0760,0.1340,0.1520,0.1710,0.1900,0.2100,0.2310,0.2520,0.2740,0.2970,0.3200,0.3440,0.3690,0.3940,0.4200,0.4460,0.4730,0.5700,0.7450,0.9200,1.0750,1.2150,1.3450,1.4700,1.5750,1.6650,1.7350,1.7800,1.8000,1.8000,1.7800,1.7500,1.7000,1.6350,1.5550,1.4650,1.3500,1.2250,1.0850,0.9250,0.7550,0.5700,0.4200,0.3200,0.2300,0.1400,0.0550,0.0250 }; 

//When testing for control surfaces especially, it is important to set the area.
const double area=0.40970;

/// \brief Test setting and getting alpha angle in radians.
TEST(Test_Lift_drag_model_UnitTest, set_get_alpha_as_radian) {
	avionics_sim::Lift_drag_model ldm;
	float testAngle=7.6329e-06;
	float valueToTestAgainst=testAngle*57.2958;
	ldm.setAlpha(testAngle,true);
	ASSERT_FLOAT_EQ(ldm.getAlpha(), valueToTestAgainst);
}

/// \brief Test setting and getting alpha angle in degrees.
TEST(Test_Lift_drag_model_UnitTest, set_get_alpha_as_degree) {
	avionics_sim::Lift_drag_model ldm;
	float testAngle=45.0;
	ldm.setAlpha(testAngle);
	ASSERT_FLOAT_EQ(ldm.getAlpha(), testAngle);
}

/// \brief Test setting and getting alpha aerodynamic force.
TEST(Test_Lift_drag_model_UnitTest, calculate_get_dynamic_pressure) {
	avionics_sim::Lift_drag_model ldm;
	double testSpeed=45.0;
        double testRho=rho;
	double valueToTestAgainst=0.5*testRho*testSpeed*testSpeed;
	ldm.setSpeed(testSpeed);
	ldm.setAirDensity(testRho);
	ldm.calculateDynamicPressure();
	ASSERT_FLOAT_EQ(ldm.getDynamicPressure(), valueToTestAgainst);
}

/*Lift Tests*/

//vInf=0.0
/// \brief Test getting lift when alpha and vInf are zero.
TEST(Test_Lift_drag_model_UnitTest, edge_case_negative_ninety_degrees_alpha_zero_vInf_lift) {
	float alpha=-90.0;
	float vInf=0.0;	
	double lift;
	avionics_sim::Lift_drag_model ldm;
	double canonicalLift=0;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
        ldm.calculateDynamicPressure();
	ldm.lookupCL();
	ldm.calculateLift();
        lift=ldm.getLift();
	ASSERT_NEAR(lift,canonicalLift, acceptable_error);
}

TEST(Test_Lift_drag_model_UnitTest, edge_case_negative_forty_five_degrees_alpha_zero_vInf_lift) {
	float alpha=-45.0;
	float vInf=0.0;	
	double lift;
	avionics_sim::Lift_drag_model ldm;
	double canonicalLift=0;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
        ldm.calculateDynamicPressure();
	ldm.lookupCL();
	ldm.calculateLift();
        lift=ldm.getLift();
	ASSERT_NEAR(lift,canonicalLift, acceptable_error);
}

TEST(Test_Lift_drag_model_UnitTest, zero_alpha_zero_vInf_lift) {
	float alpha=0.0;
	float vInf=0.0;	
	double lift;
	avionics_sim::Lift_drag_model ldm;
	double canonicalLift=0;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.calculateDynamicPressure();
	ldm.lookupCL();
	ldm.calculateLift();
        lift=ldm.getLift();
	ASSERT_NEAR(lift,canonicalLift, acceptable_error);
}

TEST(Test_Lift_drag_model_UnitTest, forty_five_degree_alpha_zero_vInf_lift) {
	float alpha=45.0;
	float vInf=0.0;	
	double lift;
	avionics_sim::Lift_drag_model ldm;
	double canonicalLift=0;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.calculateDynamicPressure();
	ldm.lookupCL();
	ldm.calculateLift();
        lift=ldm.getLift();
	ASSERT_NEAR(lift,canonicalLift, acceptable_error);
}

TEST(Test_Lift_drag_model_UnitTest, ninety_degree_alpha_zero_vInf_lift) {
	float alpha=90.0;
	float vInf=0.0;	
	double lift;
	avionics_sim::Lift_drag_model ldm;
	double canonicalLift=0;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.calculateDynamicPressure();
	ldm.lookupCL();
	ldm.calculateLift();
        lift=ldm.getLift();
	ASSERT_NEAR(lift,canonicalLift, acceptable_error);
}

//vInf=15.0
/// \brief Test getting lift when alpha and vInf are zero.
TEST(Test_Lift_drag_model_UnitTest, edge_case_negative_ninety_degrees_alpha_fifteen_vInf_lift) {
	float alpha=-90.0;
	float vInf=15.0;	
	double lift;
	avionics_sim::Lift_drag_model ldm;
	double canonicalLift=-3.9523;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.lookupCL();	
	ldm.calculateDynamicPressure();	
	ldm.calculateLift();
        lift=ldm.getLift();
	ASSERT_NEAR(lift,canonicalLift, acceptable_error);
}

TEST(Test_Lift_drag_model_UnitTest, edge_case_negative_forty_five_degrees_alpha_fifteen_vInf_lift) {
	float alpha=-45.0;
	float vInf=15.0;	
	double lift;
	avionics_sim::Lift_drag_model ldm;
	double canonicalLift=-61.261;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.calculateDynamicPressure();
	ldm.lookupCL();
	ldm.calculateLift();
        lift=ldm.getLift();
	ASSERT_NEAR(lift,canonicalLift, acceptable_error);
}

TEST(Test_Lift_drag_model_UnitTest, zero_alpha_fifteen_vInf_lift) {
	float alpha=0.0;
	float vInf=15.0;	
	double lift;
	avionics_sim::Lift_drag_model ldm;
	double canonicalLift=0;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.calculateDynamicPressure();
	ldm.lookupCL();
	ldm.calculateLift();
        lift=ldm.getLift();
	ASSERT_NEAR(lift,canonicalLift, acceptable_error);
}

TEST(Test_Lift_drag_model_UnitTest, forty_five_degree_alpha_fifteen_vInf_lift) {
	float alpha=45.0;
	float vInf=15.0;	
	double lift;
	avionics_sim::Lift_drag_model ldm;
	double canonicalLift=61.261;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.calculateDynamicPressure();
	ldm.lookupCL();
	ldm.calculateLift();
        lift=ldm.getLift();
	ASSERT_NEAR(lift,canonicalLift, acceptable_error);
}

TEST(Test_Lift_drag_model_UnitTest, ninety_degree_alpha_fifteen_vInf_lift) {
	float alpha=90.0;
	float vInf=15.0;	
	double lift;
	avionics_sim::Lift_drag_model ldm;
	double canonicalLift=3.9523;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.calculateDynamicPressure();
	ldm.lookupCL();
	ldm.calculateLift();
        lift=ldm.getLift();
	ASSERT_NEAR(lift,canonicalLift, acceptable_error);
}

//vInf=30.0
/// \brief Test getting lift when alpha and vInf are zero.
TEST(Test_Lift_drag_model_UnitTest, edge_case_negative_ninety_degrees_alpha_thirty_vInf_lift) {
	float alpha=-90.0;
	float vInf=30.0;	
	double lift;
	avionics_sim::Lift_drag_model ldm;
	double canonicalLift=-15.8093;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.lookupCL();	
	ldm.calculateDynamicPressure();	
	ldm.calculateLift();
        lift=ldm.getLift();
	ASSERT_NEAR(lift,canonicalLift, acceptable_error);
}

TEST(Test_Lift_drag_model_UnitTest, edge_case_negative_forty_five_degrees_alpha_thirty_vInf_lift) {
	float alpha=-45.0;
	float vInf=30.0;	
	double lift;
	avionics_sim::Lift_drag_model ldm;
	double canonicalLift=-245.0441;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.calculateDynamicPressure();
	ldm.lookupCL();
	ldm.calculateLift();
        lift=ldm.getLift();
	ASSERT_NEAR(lift,canonicalLift, acceptable_error);
}

TEST(Test_Lift_drag_model_UnitTest, zero_alpha_thirty_vInf_lift) {
	float alpha=0.0;
	float vInf=30.0;	
	double lift;
	avionics_sim::Lift_drag_model ldm;
	double canonicalLift=0;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.calculateDynamicPressure();
	ldm.lookupCL();
	ldm.calculateLift();
        lift=ldm.getLift();
	ASSERT_NEAR(lift,canonicalLift, acceptable_error);
}

TEST(Test_Lift_drag_model_UnitTest, forty_five_degree_alpha_thirty_vInf_lift) {
	float alpha=45.0;
	float vInf=30.0;	
	double lift;
	avionics_sim::Lift_drag_model ldm;
	double canonicalLift=245.0441;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.calculateDynamicPressure();
	ldm.lookupCL();
	ldm.calculateLift();
        lift=ldm.getLift();
	ASSERT_NEAR(lift,canonicalLift, acceptable_error);
}

TEST(Test_Lift_drag_model_UnitTest, ninety_degree_alpha_thirty_vInf_lift) {
	float alpha=90.0;
	float vInf=30.0;	
	double lift;
	avionics_sim::Lift_drag_model ldm;
	double canonicalLift=15.8093;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.calculateDynamicPressure();
	ldm.lookupCL();
	ldm.calculateLift();
        lift=ldm.getLift();
	ASSERT_NEAR(lift,canonicalLift, acceptable_error);
}


/*Drag Tests*/
TEST(Test_Lift_drag_model_UnitTest, edge_case_negative_ninety_degrees_alpha_zero_vInf_drag) {
	float alpha=-90.0;
	float vInf=0.0;	
	double drag;
	avionics_sim::Lift_drag_model ldm;
	double canonicalDrag=0;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
        ldm.calculateDynamicPressure();
	ldm.lookupCL();
	ldm.calculateDrag();
        drag=ldm.getLift();
	ASSERT_NEAR(drag,canonicalDrag, acceptable_error);
}

TEST(Test_Lift_drag_model_UnitTest, edge_case_negative_forty_five_degrees_alpha_zero_vInf_drag) {
	float alpha=-45.0;
	float vInf=0.0;	
	double drag;
	avionics_sim::Lift_drag_model ldm;
	double canonicalDrag=0;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
        ldm.calculateDynamicPressure();
	ldm.lookupCD();
	ldm.calculateDrag();
        drag=ldm.getDrag();
	ASSERT_NEAR(drag,canonicalDrag, acceptable_error);
}

TEST(Test_Lift_drag_model_UnitTest, zero_alpha_zero_vInf_drag) {
	float alpha=0.0;
	float vInf=0.0;	
	double drag;
	avionics_sim::Lift_drag_model ldm;
	double canonicalDrag=0;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.calculateDynamicPressure();
	ldm.lookupCD();
	ldm.calculateDrag();
        drag=ldm.getDrag();
	ASSERT_NEAR(drag,canonicalDrag, acceptable_error);
}

TEST(Test_Lift_drag_model_UnitTest, forty_five_degree_alpha_zero_vInf_drag) {
	float alpha=45.0;
	float vInf=0.0;	
	double drag;
	avionics_sim::Lift_drag_model ldm;
	double canonicalDrag=0;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.calculateDynamicPressure();
	ldm.lookupCD();
	ldm.calculateDrag();
        drag=ldm.getDrag();
	ASSERT_NEAR(drag,canonicalDrag, acceptable_error);
}

TEST(Test_Lift_drag_model_UnitTest, ninety_degree_alpha_zero_vInf_drag) {
	float alpha=90.0;
	float vInf=0.0;	
	double drag;
	avionics_sim::Lift_drag_model ldm;
	double canonicalDrag=0;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.calculateDynamicPressure();
	ldm.lookupCD();
	ldm.calculateDrag();
        drag=ldm.getDrag();
	ASSERT_NEAR(drag,canonicalDrag, acceptable_error);
}

//vInf=15.0
/// \brief Test getting drag when alpha and vInf are zero.
TEST(Test_Lift_drag_model_UnitTest, edge_case_negative_ninety_degrees_alpha_fifteen_vInf_drag) {
	float alpha=-90.0;
	float vInf=15.0;	
	double drag;
	avionics_sim::Lift_drag_model ldm;
	double canonicalDrag=101.6312;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.lookupCD();	
	ldm.calculateDynamicPressure();	
	ldm.calculateDrag();
        drag=ldm.getDrag();
	ASSERT_NEAR(drag,canonicalDrag, acceptable_error);
}

TEST(Test_Lift_drag_model_UnitTest, edge_case_negative_forty_five_degrees_alpha_fifteen_vInf_drag) {
	float alpha=-45.0;
	float vInf=15.0;	
	double drag;
	avionics_sim::Lift_drag_model ldm;
	double canonicalDrag=60.6964;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.calculateDynamicPressure();
	ldm.lookupCD();
	ldm.calculateDrag();
        drag=ldm.getDrag();
	ASSERT_NEAR(drag,canonicalDrag, acceptable_error);
}

TEST(Test_Lift_drag_model_UnitTest, zero_alpha_fifteen_vInf_drag) {
	float alpha=0.0;
	float vInf=15.0;	
	double drag;
	avionics_sim::Lift_drag_model ldm;
	double canonicalDrag=0.58156;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.calculateDynamicPressure();
	ldm.lookupCD();
	ldm.calculateDrag();
        drag=ldm.getDrag();
	ASSERT_NEAR(drag,canonicalDrag, acceptable_error);
}

TEST(Test_Lift_drag_model_UnitTest, forty_five_degree_alpha_fifteen_vInf_drag) {
	float alpha=45.0;
	float vInf=15.0;	
	double drag;
	avionics_sim::Lift_drag_model ldm;
	double canonicalDrag=60.6964;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.calculateDynamicPressure();
	ldm.lookupCD();
	ldm.calculateDrag();
        drag=ldm.getDrag();
	ASSERT_NEAR(drag,canonicalDrag, acceptable_error);
}

TEST(Test_Lift_drag_model_UnitTest, ninety_degree_alpha_fifteen_vInf_drag) {
	float alpha=90.0;
	float vInf=15.0;	
	double drag;
	avionics_sim::Lift_drag_model ldm;
	double canonicalDrag=101.6312;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.calculateDynamicPressure();
	ldm.lookupCD();
	ldm.calculateDrag();
        drag=ldm.getDrag();
	ASSERT_NEAR(drag,canonicalDrag, acceptable_error);
}

//vInf=30.0
/// \brief Test getting drag when alpha and vInf are zero.
TEST(Test_Lift_drag_model_UnitTest, edge_case_negative_ninety_degrees_alpha_thirty_vInf_drag) {
	float alpha=-90.0;
	float vInf=30.0;	
	double drag;
	avionics_sim::Lift_drag_model ldm;
	double canonicalDrag=406.5248;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.lookupCD();	
	ldm.calculateDynamicPressure();	
	ldm.calculateDrag();
        drag=ldm.getDrag();
	ASSERT_NEAR(drag,canonicalDrag, acceptable_error);
}

TEST(Test_Lift_drag_model_UnitTest, edge_case_negative_forty_five_degrees_alpha_thirty_vInf_drag) {
	float alpha=-45.0;
	float vInf=30.0;	
	double drag;
	avionics_sim::Lift_drag_model ldm;
	double canonicalDrag=242.7857;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.calculateDynamicPressure();
	ldm.lookupCD();
	ldm.calculateDrag();
        drag=ldm.getDrag();
	ASSERT_NEAR(drag,canonicalDrag, acceptable_error);
}

TEST(Test_Lift_drag_model_UnitTest, zero_alpha_thirty_vInf_drag) {
	float alpha=0.0;
	float vInf=30.0;	
	double drag;
	avionics_sim::Lift_drag_model ldm;
	double canonicalDrag=2.3262;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.calculateDynamicPressure();
	ldm.lookupCD();
	ldm.calculateDrag();
        drag=ldm.getDrag();
	ASSERT_NEAR(drag,canonicalDrag, acceptable_error);
}

TEST(Test_Lift_drag_model_UnitTest, forty_five_degree_alpha_thirty_vInf_drag) {
	float alpha=45.0;
	float vInf=30.0;	
	double drag;
	avionics_sim::Lift_drag_model ldm;
	double canonicalDrag=242.7857;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.calculateDynamicPressure();
	ldm.lookupCD();
	ldm.calculateDrag();
        drag=ldm.getDrag();
	ASSERT_NEAR(drag,canonicalDrag, acceptable_error);
}

TEST(Test_Lift_drag_model_UnitTest, ninety_degree_alpha_thirty_vInf_drag) {
	float alpha=90.0;
	float vInf=30.0;	
	double drag;
	avionics_sim::Lift_drag_model ldm;
	double canonicalDrag=406.5248;
	ldm.setAlpha(alpha);
	ldm.setSpeed(vInf);
        ldm.setAirDensity(rho);
	ldm.setArea(area);
	ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
	ldm.calculateDynamicPressure();
	ldm.lookupCD();
	ldm.calculateDrag();
        drag=ldm.getDrag();
	ASSERT_NEAR(drag,canonicalDrag, acceptable_error);
}


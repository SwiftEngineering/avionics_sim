/**
 * @brief       Avionics Simulator Library Test Runner Implementation
 * @file        main.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include <string>
#include <tuple>
#include <vector>
#include <random>
#include <iostream>
#include <float.h>
#include <limits>
#include <cmath>

#include "TestRunnerEnvironment.h"
#include "TestEventListener.h"
#include "xmlparser.h"
#include "utilities.h"
#include "CombinatoricalLiftDragTest.h"
#include "CoordUtilsTest.h"
#include "LiftDragSetAlphaParameterized.h"
#include "LiftDragSetSpeedParameterized.h"
#include "LiftDragSetAirDensityParameterized.h"
#include "LiftDragLookupCLParameterized.h"
#include "LiftDragLookupCDParameterized.h"
#include "LiftDragSetAreaParameterized.h"
#include "LiftDragCalculateLiftParameterized.h"
#include "LiftDragCalculateDragParameterized.h"
#include "LiftDragCalculateDynamicPressureParameterized.h"
#include "MotorModelSetThrustParameterized.h"
#include "MotorModelSetExitVelocityParameterized.h"


//Uncomment this to use the smaller hand calc tests.
//#define HANDTEST 1
#define NO_TESTS_EXECUTED 0

const std::vector<float> LUT_alpha{-180.0000,-175.0000,-170.0000,-165.0000,-160.0000,-155.0000,-150.0000,-145.0000,-140.0000,-135.0000,-130.0000,-125.0000,-120.0000,-115.0000,-110.0000,-105.0000,-100.0000,-95.0000,-90.0000,-85.0000,-80.0000,-75.0000,-70.0000,-65.0000,-60.0000,-55.0000,-50.0000,-45.0000,-40.0000,-35.0000,-30.0000,-27.0000,-26.0000,-25.0000,-24.0000,-23.0000,-22.0000,-21.0000,-20.0000,-19.0000,-18.0000,-17.0000,-16.0000,-15.0000,-14.0000,-13.0000,-12.0000,-11.0000,-10.0000,-9.0000,-8.0000,-7.0000,-6.0000,-5.0000,-4.0000,-3.0000,-2.0000,-1.0000,-0.0000,0.0000,1.0000,2.0000,3.0000,4.0000,5.0000,6.0000,7.0000,8.0000,9.0000,10.0000,11.0000,12.0000,13.0000,14.0000,15.0000,16.0000,17.0000,18.0000,19.0000,20.0000,21.0000,22.0000,23.0000,24.0000,25.0000,26.0000,27.0000,30.0000,35.0000,40.0000,45.0000,50.0000,55.0000,60.0000,65.0000,70.0000,75.0000,80.0000,85.0000,90.0000,95.0000,100.0000,105.0000,110.0000,115.0000,120.0000,125.0000,130.0000,135.0000,140.0000,145.0000,150.0000,155.0000,160.0000,165.0000,170.0000,175.0000,180.0000 }; 

const std::vector<float> LUT_CL{ -0.0000,0.6900,0.8500,0.6750,0.6600,0.7400,0.8500,0.9100,0.9450,0.9450,0.9100,0.8400,0.7350,0.6250,0.5100,0.3700,0.2200,0.0700,-0.0700,-0.2200,-0.3700,-0.5150,-0.6500,-0.7650,-0.8750,-0.9650,-1.0400,-1.0850,-1.0750,-1.0200,-0.9150,-0.9646,-0.9109,-0.8572,-0.8034,-0.7497,-0.6956,-0.6414,-0.5870,-0.5322,-0.4768,-0.4200,-0.3620,-0.3082,-0.2546,-0.2030,-0.1533,-0.1095,-0.1325,-0.8527,-0.8274,-0.7460,-0.6600,-0.5500,-0.4400,-0.3300,-0.2200,-0.1100,-0.0000,0.0000,0.1100,0.2200,0.3300,0.4400,0.5500,0.6600,0.7460,0.8274,0.8527,0.1325,0.1095,0.1533,0.2030,0.2546,0.3082,0.3620,0.4200,0.4768,0.5322,0.5870,0.6414,0.6956,0.7497,0.8034,0.8572,0.9109,0.9646,0.9150,1.0200,1.0750,1.0850,1.0400,0.9650,0.8750,0.7650,0.6500,0.5150,0.3700,0.2200,0.0700,-0.0700,-0.2200,-0.3700,-0.5100,-0.6250,-0.7350,-0.8400,-0.9100,-0.9450,-0.9450,-0.9100,-0.8500,-0.7400,-0.6600,-0.6750,-0.8500,-0.6900,0.0000 }; 

const std::vector<float> LUT_CD{ 0.0250,0.0550,0.1400,0.2300,0.3200,0.4200,0.5700,0.7550,0.9250,1.0850,1.2250,1.3500,1.4650,1.5550,1.6350,1.7000,1.7500,1.7800,1.8000,1.8000,1.7800,1.7350,1.6650,1.5750,1.4700,1.3450,1.2150,1.0750,0.9200,0.7450,0.5700,0.4730,0.4460,0.4200,0.3940,0.3690,0.3440,0.3200,0.2970,0.2740,0.2520,0.2310,0.2100,0.1900,0.1710,0.1520,0.1340,0.0760,0.0188,0.0203,0.0185,0.0170,0.0152,0.0140,0.0124,0.0114,0.0108,0.0104,0.0103,0.0103,0.0104,0.0108,0.0114,0.0124,0.0140,0.0152,0.0170,0.0185,0.0203,0.0188,0.0760,0.1340,0.1520,0.1710,0.1900,0.2100,0.2310,0.2520,0.2740,0.2970,0.3200,0.3440,0.3690,0.3940,0.4200,0.4460,0.4730,0.5700,0.7450,0.9200,1.0750,1.2150,1.3450,1.4700,1.5750,1.6650,1.7350,1.7800,1.8000,1.8000,1.7800,1.7500,1.7000,1.6350,1.5550,1.4650,1.3500,1.2250,1.0850,0.9250,0.7550,0.5700,0.4200,0.3200,0.2300,0.1400,0.0550,0.0250 };

const std::vector<double> alphaAngles{-180.0000,-175.0000,-170.0000,-165.0000,-160.0000,-155.0000,-150.0000,-145.0000,-140.0000,-135.0000,-130.0000,-125.0000,-120.0000,-115.0000,-110.0000,-105.0000,-100.0000,-95.0000,-90.0000,-85.0000,-80.0000,-75.0000,-70.0000,-65.0000,-60.0000,-55.0000,-50.0000,-45.0000,-40.0000,-35.0000,-30.0000,-27.0000,-26.0000,-25.0000,-24.0000,-23.0000,-22.0000,-21.0000,-20.0000,-19.0000,-18.0000,-17.0000,-16.0000,-15.0000,-14.0000,-13.0000,-12.0000,-11.0000,-10.0000,-9.0000,-8.0000,-7.0000,-6.0000,-5.0000,-4.0000,-3.0000,-2.0000,-1.0000,-0.0000,0.0000,1.0000,2.0000,3.0000,4.0000,5.0000,6.0000,7.0000,8.0000,9.0000,10.0000,11.0000,12.0000,13.0000,14.0000,15.0000,16.0000,17.0000,18.0000,19.0000,20.0000,21.0000,22.0000,23.0000,24.0000,25.0000,26.0000,27.0000,30.0000,35.0000,40.0000,45.0000,50.0000,55.0000,60.0000,65.0000,70.0000,75.0000,80.0000,85.0000,90.0000,95.0000,100.0000,105.0000,110.0000,115.0000,120.0000,125.0000,130.0000,135.0000,140.0000,145.0000,150.0000,155.0000,160.0000,165.0000,170.0000,175.0000,180.0000};

LiftDragSetAlphaParameterCollections setAlphaValues;

LiftDragSetSpeedParameterCollections setSpeedValues;

LiftDragSetAirDensityParameterCollections setAirDensityValues;

LiftDragLookupCLParameterCollections lookupCLValues;

LiftDragLookupCDParameterCollections lookupCDValues;

LiftDragSetAreaParameterCollections setAreaValues;

LiftDragCalculateLiftParameterCollections calculateLiftValues;

LiftDragCalculateDragParameterCollections calculateDragValues;

LiftDragCalculateDynamicPressureParameterCollections calculateDynamicPressureValues;

MotorModelSetExitVelocityParameterCollections setExitVelocityValues;

MotorModelSetThrustParameterCollections setThrustValues;

CoordUtilsDataCollections coordUtilData;

//Parameter collections for each of the four combinations for prop wash and control surface conditions.
LiftDragParameterCollections noPropWashNoControlSurfaceParamCollections;
LiftDragParameterCollections noPropWashControlSurfaceParamCollections;
LiftDragParameterCollections propWashNoControlSurfaceParamCollections;
LiftDragParameterCollections propWashControlSurfaceParamCollections;

//NB: Disable test by prefacing first parameter with "DISABLED_"
//Instantiation of parameterized integration test for non prop wash, non control surface.
INSTANTIATE_TEST_CASE_P(NoPropWashNoControlSurface,
                        CombinatoricalLiftDragTest,
                        ::testing::ValuesIn(noPropWashNoControlSurfaceParamCollections));

//Instantiation of parameterized integration test for non prop wash, control surface.
INSTANTIATE_TEST_CASE_P(NoPropWashControlSurface,
CombinatoricalLiftDragTest,
::testing::ValuesIn(noPropWashControlSurfaceParamCollections));

//Instantiation of parameterized integration test for prop wash, non control surface.
INSTANTIATE_TEST_CASE_P(PropWashNoControlSurface,
CombinatoricalLiftDragTest,
::testing::ValuesIn(propWashNoControlSurfaceParamCollections));

//Instantiation of parameterized integration test for prop wash, control surface.
INSTANTIATE_TEST_CASE_P(PropWashControlSurface,
CombinatoricalLiftDragTest,
::testing::ValuesIn(propWashControlSurfaceParamCollections));

//Instantiation of parameterized CoordUtils test.
INSTANTIATE_TEST_CASE_P(CoordUtilsTestProjectVectorGlobal,
                        CoordUtilsTest,
                        ::testing::ValuesIn(coordUtilData));

//Instantiation of LiftDrag::setAlpha method test
INSTANTIATE_TEST_CASE_P(LiftDragUnitTestSetGetAlpha,
                        LiftDragSetAlphaParameterized,
                        ::testing::ValuesIn(setAlphaValues));

//Instantiation of LiftDrag::setAlpha method test
INSTANTIATE_TEST_CASE_P(LiftDragUnitTestSetGetSpeed,
                        LiftDragSetSpeedParameterized,
                        ::testing::ValuesIn(setSpeedValues));

//Instantiation of LiftDrag::setAirDensity method test
INSTANTIATE_TEST_CASE_P(LiftDragUnitTestSetGetAirDensity,
                        LiftDragSetAirDensityParameterized,
                        ::testing::ValuesIn(setAirDensityValues));

//Instantiation of LiftDrag::lookupCL method test
INSTANTIATE_TEST_CASE_P(LiftDragUnitTestLookupCL,
                        LiftDragLookupCLParameterized,
                        ::testing::ValuesIn(lookupCLValues));

//Instantiation of LiftDrag::lookupCL method test
INSTANTIATE_TEST_CASE_P(LiftDragUnitTestLookupCD,
                        LiftDragLookupCDParameterized,
                        ::testing::ValuesIn(lookupCDValues));

//Instantiation of LiftDrag::setArea method test
INSTANTIATE_TEST_CASE_P(LiftDragUnitTestSetGetArea,
                        LiftDragSetAreaParameterized,
                        ::testing::ValuesIn(setAreaValues));

//Instantiation of LiftDrag::setArea method test
INSTANTIATE_TEST_CASE_P(LiftDragUnitTestCalculateLift,
                        LiftDragCalculateLiftParameterized,
                        ::testing::ValuesIn(calculateLiftValues));

//Instantiation of LiftDrag::setArea method test
INSTANTIATE_TEST_CASE_P(LiftDragUnitTestCalculateDrag,
                        LiftDragCalculateDragParameterized,
                        ::testing::ValuesIn(calculateDragValues));

//Instantiation of LiftDrag::setArea method test
INSTANTIATE_TEST_CASE_P(LiftDragUnitTestCalculateDynamicPressure,
                        LiftDragCalculateDynamicPressureParameterized,
                        ::testing::ValuesIn(calculateDynamicPressureValues));

//Instantiation of upcoming MotorModel::setThrust method test
INSTANTIATE_TEST_CASE_P(MotorModelUnitTestSetThrust,
                        MotorModelSetThrustParameterized,
                        ::testing::ValuesIn(setThrustValues));

//Instantiation of upcoming MotorModel::setExitVelocity method test
INSTANTIATE_TEST_CASE_P(MotorModelUnitTestSetExitVelocity,
                        MotorModelSetExitVelocityParameterized,
                        ::testing::ValuesIn(setExitVelocityValues));

bool runUnitTests=false;
bool runIntegrationTests=false;
bool runSystemTests=false;
const int minNumArguments=2;
const int maxNumArguments=3;

//Function to generate random numbers of type double.
static double getRandomDouble(double lowerBound, double upperBound)
{
	std::random_device dev;
    std::mt19937 rng(dev());
	std::uniform_real_distribution<double> dist(lowerBound, upperBound);
    return dist(rng);
}

//Function to generate random numbers of type double.
static int getRandomInt(int lowerBound, int upperBound)
{
	std::random_device dev;
    std::mt19937 rng(dev());
	std::uniform_int_distribution<int> dist(lowerBound, upperBound);
    return dist(rng);
}

//Function to set up parameterized coord utils test
static void setupCoordUtilsTestData()
{
	XMLParser inputs;

	//Read in coordinate utils test data.
	std::string coordUtilsTestDataFile="../unit_tests/TestData/Test_Coordinate_Utils.txt";
	inputs.openXMLFile(coordUtilsTestDataFile.c_str());
	
	std::vector<std::string> worldLinearVelocitiesData;
	std::vector<std::string> worldOrientationsData;
	std::vector<std::string> canonicalLinearVels;

	//Store each node data for input into a vector of strings.
	inputs.getXMLVector("worldLinearVels", worldLinearVelocitiesData);
	inputs.getXMLVector("poseVectors", worldOrientationsData);
	inputs.getXMLVector("canonicalLinearVels", canonicalLinearVels);

	unsigned int numParams=worldOrientationsData.size();
	for (unsigned int j=0; j<numParams; j++)
	{
		CoordUtilsParams params;
		getDataVector(j, worldLinearVelocitiesData, params.worldLinearVelocities);
		getDataVector(j, worldOrientationsData, params.worldOrientations);
		getDataVector(j, canonicalLinearVels, params.globalVelocities);
		coordUtilData.push_back(params);
	}
}

static void setupLiftDragSetAlphaTestData()
{
	//TODO: Refactor s.t. 57.2958 (DEGREES_IN_RADIANS) in 
		// Lift_drag_model is available where needed globally.

		LiftDragSetAlphaParams randLowerBoundAlphaRadians;
		LiftDragSetAlphaParams randUpperBoundAlphaRadians;
		LiftDragSetAlphaParams zeroAlphaRadians;
		LiftDragSetAlphaParams lowerBoundAlphaRadians;
		LiftDragSetAlphaParams upperBoundAlphaRadians;
		LiftDragSetAlphaParams lowestBoundAlphaRadians;
		LiftDragSetAlphaParams highestBoundAlphaRadians;

		LiftDragSetAlphaParams randLowerBoundAlphaDegrees;
		LiftDragSetAlphaParams randUpperBoundAlphaDegrees;
		LiftDragSetAlphaParams zeroAlphaDegrees;
		LiftDragSetAlphaParams lowestBoundAlphaDegrees;
		LiftDragSetAlphaParams highestBoundAlphaDegrees;

		LiftDragSetAlphaParams randExceptionLowerBoundAlpha;
		LiftDragSetAlphaParams randExceptionUpperBoundAlpha;
		LiftDragSetAlphaParams exceptionLowestBoundAlpha;
		LiftDragSetAlphaParams exceptionHighestBoundAlpha;
		LiftDragSetAlphaParams nanAlpha;

		LiftDragSetAlphaParams vInfLessThanMin;

		double minRadians=avionics_sim::Lift_drag_model::MIN_AOA/57.2958;
		double maxRadians=avionics_sim::Lift_drag_model::MAX_AOA/57.2958;
		double minRadiansBound=(avionics_sim::Lift_drag_model::MIN_AOA+1)/57.2958;
		double maxRadiansBound=(avionics_sim::Lift_drag_model::MAX_AOA-1)/57.2958;
		double minDegreesBound=(avionics_sim::Lift_drag_model::MIN_AOA+1);
		double maxDegreesBound=(avionics_sim::Lift_drag_model::MAX_AOA-1);
		randLowerBoundAlphaRadians.isRadians=true;
		randLowerBoundAlphaRadians.vInf=avionics_sim::Lift_drag_model::MIN_VINF_ALPHA;
		randLowerBoundAlphaRadians.value=getRandomDouble(minRadiansBound, -1/57.2958);
		randUpperBoundAlphaRadians.isRadians=true;
		randUpperBoundAlphaRadians.vInf=avionics_sim::Lift_drag_model::MIN_VINF_ALPHA;
		randUpperBoundAlphaRadians.value=getRandomDouble(1/57.2958, maxRadiansBound);
		zeroAlphaRadians.isRadians=true;
		zeroAlphaRadians.vInf=avionics_sim::Lift_drag_model::MIN_VINF_ALPHA;
		zeroAlphaRadians.value=0/57.2958;
		lowestBoundAlphaRadians.isRadians=true;
		lowestBoundAlphaRadians.vInf=avionics_sim::Lift_drag_model::MIN_VINF_ALPHA;
		lowestBoundAlphaRadians.value=minRadians;
		highestBoundAlphaRadians.isRadians=true;
		highestBoundAlphaRadians.vInf=avionics_sim::Lift_drag_model::MIN_VINF_ALPHA;
		highestBoundAlphaRadians.value=maxRadians;
		lowerBoundAlphaRadians.isRadians=true;
		lowerBoundAlphaRadians.vInf=avionics_sim::Lift_drag_model::MIN_VINF_ALPHA;
		lowerBoundAlphaRadians.value=minRadians;
		upperBoundAlphaRadians.isRadians=true;
		upperBoundAlphaRadians.vInf=avionics_sim::Lift_drag_model::MIN_VINF_ALPHA;
		upperBoundAlphaRadians.value=maxRadians;
		
		setAlphaValues.push_back(randLowerBoundAlphaRadians);
		setAlphaValues.push_back(randUpperBoundAlphaRadians);
		setAlphaValues.push_back(zeroAlphaRadians);
		setAlphaValues.push_back(lowestBoundAlphaRadians);
		setAlphaValues.push_back(highestBoundAlphaRadians);
		setAlphaValues.push_back(lowerBoundAlphaRadians);
		setAlphaValues.push_back(upperBoundAlphaRadians);

		randLowerBoundAlphaDegrees.isRadians=false;
		randLowerBoundAlphaDegrees.vInf=avionics_sim::Lift_drag_model::MIN_VINF_ALPHA;
		randLowerBoundAlphaDegrees.value=getRandomDouble(minDegreesBound, -1);
		randUpperBoundAlphaDegrees.isRadians=false;
		randUpperBoundAlphaDegrees.vInf=avionics_sim::Lift_drag_model::MIN_VINF_ALPHA;
		randUpperBoundAlphaDegrees.value=getRandomDouble(1, maxDegreesBound);
		zeroAlphaDegrees.isRadians=false;
		zeroAlphaDegrees.vInf=avionics_sim::Lift_drag_model::MIN_VINF_ALPHA;
		zeroAlphaDegrees.value=0;
		lowestBoundAlphaDegrees.isRadians=false;
		lowestBoundAlphaDegrees.vInf=avionics_sim::Lift_drag_model::MIN_VINF_ALPHA;
		lowestBoundAlphaDegrees.value=avionics_sim::Lift_drag_model::MIN_AOA;
		highestBoundAlphaDegrees.isRadians=false;
		highestBoundAlphaDegrees.vInf=avionics_sim::Lift_drag_model::MIN_VINF_ALPHA;
		highestBoundAlphaDegrees.value=avionics_sim::Lift_drag_model::MAX_AOA;


		setAlphaValues.push_back(randLowerBoundAlphaDegrees);
		setAlphaValues.push_back(randUpperBoundAlphaDegrees);
		setAlphaValues.push_back(zeroAlphaDegrees);
		setAlphaValues.push_back(lowestBoundAlphaDegrees);
		setAlphaValues.push_back(highestBoundAlphaDegrees);

		randExceptionLowerBoundAlpha.isRadians=false;
		randExceptionLowerBoundAlpha.vInf=avionics_sim::Lift_drag_model::MIN_VINF_ALPHA;
		randExceptionLowerBoundAlpha.value=getRandomDouble(std::numeric_limits<double>::lowest()+1, avionics_sim::Lift_drag_model::MIN_AOA-1);

		randExceptionUpperBoundAlpha.isRadians=false;
		randExceptionUpperBoundAlpha.vInf=avionics_sim::Lift_drag_model::MIN_VINF_ALPHA;
		randExceptionUpperBoundAlpha.value=getRandomDouble(avionics_sim::Lift_drag_model::MAX_AOA+1, std::numeric_limits<double>::max()-1);

		exceptionLowestBoundAlpha.isRadians=false;
		exceptionLowestBoundAlpha.vInf=avionics_sim::Lift_drag_model::MIN_VINF_ALPHA;
		exceptionLowestBoundAlpha.value=std::numeric_limits<double>::lowest();

		exceptionHighestBoundAlpha.isRadians=false;
		exceptionHighestBoundAlpha.vInf=avionics_sim::Lift_drag_model::MIN_VINF_ALPHA;
		exceptionHighestBoundAlpha.value=std::numeric_limits<double>::max();
		
		nanAlpha.isRadians=false;
		nanAlpha.vInf=avionics_sim::Lift_drag_model::MIN_VINF_ALPHA;
		nanAlpha.value=std::numeric_limits<double>::quiet_NaN();

		vInfLessThanMin.isRadians=false;
		vInfLessThanMin.value=0;
		vInfLessThanMin.vInf=0.0;

		setAlphaValues.push_back(randExceptionLowerBoundAlpha);
		setAlphaValues.push_back(randExceptionUpperBoundAlpha);
		setAlphaValues.push_back(exceptionLowestBoundAlpha);
		setAlphaValues.push_back(exceptionHighestBoundAlpha);
		setAlphaValues.push_back(nanAlpha);

		setAlphaValues.push_back(vInfLessThanMin);
}

static void setupLiftDragSetSpeedTestData()
{
	LiftDragSetSpeedParams exceptionLowestBoundSpeed;
	LiftDragSetSpeedParams randExceptionLowerBoundSpeed;
	LiftDragSetSpeedParams zeroSpeed;
	LiftDragSetSpeedParams minSpeed;
	LiftDragSetSpeedParams maxSpeed;
	LiftDragSetSpeedParams randExceptionUpperBoundSpeed;
	LiftDragSetSpeedParams exceptionHighestBoundSpeed;
	LiftDragSetSpeedParams nanSpeed;

	exceptionLowestBoundSpeed.value=std::numeric_limits<double>::lowest();

	randExceptionLowerBoundSpeed.value=getRandomDouble(std::numeric_limits<double>::lowest()+1, avionics_sim::Lift_drag_model::MIN_VINF-1);

	zeroSpeed.value=0;

	minSpeed.value=avionics_sim::Lift_drag_model::MIN_VINF;

	maxSpeed.value=avionics_sim::Lift_drag_model::MAX_VINF;

	randExceptionUpperBoundSpeed.value=getRandomDouble(avionics_sim::Lift_drag_model::MAX_VINF+1, std::numeric_limits<double>::max()-1);

	exceptionHighestBoundSpeed.value=std::numeric_limits<double>::max();

	nanSpeed.value=std::numeric_limits<double>::quiet_NaN();

	setSpeedValues.push_back(exceptionLowestBoundSpeed);
	setSpeedValues.push_back(randExceptionLowerBoundSpeed);
	setSpeedValues.push_back(zeroSpeed);
	setSpeedValues.push_back(minSpeed);
	setSpeedValues.push_back(maxSpeed);
	setSpeedValues.push_back(randExceptionUpperBoundSpeed);
	setSpeedValues.push_back(exceptionHighestBoundSpeed);
	setSpeedValues.push_back(nanSpeed);

}

static void setupLiftDragSetAirDensityTestData()
{
	LiftDragSetAirDensityParams exceptionLowestBoundAirDensity;
	LiftDragSetAirDensityParams randExceptionLowerBoundAirDensity;
	LiftDragSetAirDensityParams zeroAirDensity;
	LiftDragSetAirDensityParams minAirDensity;
	LiftDragSetAirDensityParams maxAirDensity;
	LiftDragSetAirDensityParams randExceptionUpperBoundAirDensity;
	LiftDragSetAirDensityParams exceptionHighestBoundAirDensity;
	LiftDragSetAirDensityParams nanAirDensity;

	exceptionLowestBoundAirDensity.value=std::numeric_limits<double>::lowest();

	randExceptionLowerBoundAirDensity.value=getRandomDouble(std::numeric_limits<double>::lowest()+1, avionics_sim::Lift_drag_model::MIN_AIR_DENSITY-1);

	zeroAirDensity.value=0;

	minAirDensity.value=avionics_sim::Lift_drag_model::MIN_AIR_DENSITY;

	maxAirDensity.value=avionics_sim::Lift_drag_model::MAX_AIR_DENSITY;

	randExceptionUpperBoundAirDensity.value=getRandomDouble(avionics_sim::Lift_drag_model::MAX_AIR_DENSITY+1, std::numeric_limits<double>::max()-1);

	exceptionHighestBoundAirDensity.value=std::numeric_limits<double>::max();

	nanAirDensity.value=std::numeric_limits<double>::quiet_NaN();

	setAirDensityValues.push_back(exceptionLowestBoundAirDensity);
	setAirDensityValues.push_back(randExceptionLowerBoundAirDensity);
	setAirDensityValues.push_back(zeroAirDensity);
	setAirDensityValues.push_back(minAirDensity);
	setAirDensityValues.push_back(maxAirDensity);
	setAirDensityValues.push_back(randExceptionUpperBoundAirDensity);
	setAirDensityValues.push_back(exceptionHighestBoundAirDensity);
	setAirDensityValues.push_back(nanAirDensity);
}

static void setupLiftDragLookupCLTestData()
{
	/*Only need to test for these four cases as non-exception cases,
	as the other values would be caught as exceptions when setAlpha is called. None of these four should yield an exception. Also, no need to test radians, as angle is stored internally in degree form.*/
	LiftDragLookupCLParams zeroAlpha;
	LiftDragLookupCLParams minAlpha;
	LiftDragLookupCLParams maxAlpha;
	LiftDragLookupCLParams randAlphaBtwMinAndMax;

	zeroAlpha.value=0;
    copy(LUT_alpha.begin(), LUT_alpha.end(), back_inserter(zeroAlpha.LUT_alpha)); 
    copy(LUT_CL.begin(), LUT_CL.end(), back_inserter(zeroAlpha.LUT_CL)); 
    copy(LUT_CD.begin(), LUT_CD.end(), back_inserter(zeroAlpha.LUT_CD)); 

	minAlpha.value=avionics_sim::Lift_drag_model::MIN_AOA;
	copy(LUT_alpha.begin(), LUT_alpha.end(), back_inserter(minAlpha.LUT_alpha)); 
	// Copying vector by copy function 
    copy(LUT_CL.begin(), LUT_CL.end(), back_inserter(minAlpha.LUT_CL)); 
	// Copying vector by copy function 
    copy(LUT_CD.begin(), LUT_CD.end(), back_inserter(minAlpha.LUT_CD)); 

	maxAlpha.value=avionics_sim::Lift_drag_model::MAX_AOA;
	copy(LUT_alpha.begin(), LUT_alpha.end(), back_inserter(maxAlpha.LUT_alpha)); 
	// Copying vector by copy function 
    copy(LUT_CL.begin(), LUT_CL.end(), back_inserter(maxAlpha.LUT_CL)); 
	// Copying vector by copy function 
    copy(LUT_CD.begin(), LUT_CD.end(), back_inserter(maxAlpha.LUT_CD));

	//Choose a non-zero random number between MIN_AOA and MAX_AOA
	double randAlpha=0;
	while (randAlpha==0)
	{
		randAlpha=alphaAngles.at(getRandomInt(0, alphaAngles.size()-1));
	}
	randAlphaBtwMinAndMax.value=randAlpha;
	copy(LUT_alpha.begin(), LUT_alpha.end(), back_inserter(randAlphaBtwMinAndMax.LUT_alpha)); 
	// Copying vector by copy function 
    copy(LUT_CL.begin(), LUT_CL.end(), back_inserter(randAlphaBtwMinAndMax.LUT_CL)); 
	// Copying vector by copy function 
    copy(LUT_CD.begin(), LUT_CD.end(), back_inserter(randAlphaBtwMinAndMax.LUT_CD));

	lookupCLValues.push_back(zeroAlpha);
	lookupCLValues.push_back(minAlpha);
	lookupCLValues.push_back(maxAlpha);
	lookupCLValues.push_back(randAlphaBtwMinAndMax);
}

static void setupLiftDragLookupCDTestData()
{
	/*Only need to test for these four cases as non-exception cases,
	as the other values would be caught as exceptions when setAlpha is called. None of these four should yield an exception. Also, no need to test radians, as angle is stored internally in degree form.*/
	LiftDragLookupCDParams zeroAlpha;
	LiftDragLookupCDParams minAlpha;
	LiftDragLookupCDParams maxAlpha;
	LiftDragLookupCDParams randAlphaBtwMinAndMax;
	
	zeroAlpha.value=0;
	copy(LUT_alpha.begin(), LUT_alpha.end(), back_inserter(zeroAlpha.LUT_alpha)); 
    copy(LUT_CL.begin(), LUT_CL.end(), back_inserter(zeroAlpha.LUT_CL)); 
    copy(LUT_CD.begin(), LUT_CD.end(), back_inserter(zeroAlpha.LUT_CD)); 

	minAlpha.value=avionics_sim::Lift_drag_model::MIN_AOA;
	copy(LUT_alpha.begin(), LUT_alpha.end(), back_inserter(minAlpha.LUT_alpha)); 
    copy(LUT_CL.begin(), LUT_CL.end(), back_inserter(minAlpha.LUT_CL)); 
    copy(LUT_CD.begin(), LUT_CD.end(), back_inserter(minAlpha.LUT_CD)); 

	maxAlpha.value=avionics_sim::Lift_drag_model::MAX_AOA;
	copy(LUT_alpha.begin(), LUT_alpha.end(), back_inserter(maxAlpha.LUT_alpha)); 
    copy(LUT_CL.begin(), LUT_CL.end(), back_inserter(maxAlpha.LUT_CL)); 
    copy(LUT_CD.begin(), LUT_CD.end(), back_inserter(maxAlpha.LUT_CD)); 

	//Choose a random, non-zero angle from the LUT.
	double randAlpha=0;
	while (randAlpha==0)
	{
		randAlpha=alphaAngles.at(getRandomInt(0, alphaAngles.size()-1));
	}
	
	randAlphaBtwMinAndMax.value=randAlpha;
	copy(LUT_alpha.begin(), LUT_alpha.end(), back_inserter(randAlphaBtwMinAndMax.LUT_alpha)); 
    copy(LUT_CL.begin(), LUT_CL.end(), back_inserter(randAlphaBtwMinAndMax.LUT_CL)); 
    copy(LUT_CD.begin(), LUT_CD.end(), back_inserter(randAlphaBtwMinAndMax.LUT_CD)); 

	lookupCDValues.push_back(zeroAlpha);
	lookupCDValues.push_back(minAlpha);
	lookupCDValues.push_back(maxAlpha);
	lookupCDValues.push_back(randAlphaBtwMinAndMax);
}

static void setupLiftDragAreaTestData()
{
	LiftDragSetAreaParams exceptionLowestBoundArea;
	LiftDragSetAreaParams randExceptionLowerBoundArea;
	LiftDragSetAreaParams zeroArea;
	LiftDragSetAreaParams minArea;
	LiftDragSetAreaParams maxArea;
	LiftDragSetAreaParams randExceptionUpperBoundArea;
	LiftDragSetAreaParams exceptionHighestBoundArea;
	LiftDragSetAreaParams nanArea;

	exceptionLowestBoundArea.value=std::numeric_limits<double>::lowest();

	randExceptionLowerBoundArea.value=getRandomDouble(std::numeric_limits<double>::lowest()+1, avionics_sim::Lift_drag_model::MIN_AREA-1);

	zeroArea.value=0;

	minArea.value=avionics_sim::Lift_drag_model::MIN_AREA;

	maxArea.value=avionics_sim::Lift_drag_model::MAX_AREA;

	randExceptionUpperBoundArea.value=getRandomDouble(avionics_sim::Lift_drag_model::MAX_AREA+1, std::numeric_limits<double>::max()-1);

	exceptionHighestBoundArea.value=std::numeric_limits<double>::max();

	nanArea.value=std::numeric_limits<double>::quiet_NaN();

	setAreaValues.push_back(exceptionLowestBoundArea);
	setAreaValues.push_back(randExceptionLowerBoundArea);
	setAreaValues.push_back(zeroArea);
	setAreaValues.push_back(minArea);
	setAreaValues.push_back(maxArea);
	setAreaValues.push_back(randExceptionUpperBoundArea);
	setAreaValues.push_back(exceptionHighestBoundArea);
	setAreaValues.push_back(nanArea);
}

static void setupLiftDragCalculateLiftTestData()
{
	LiftDragCalculateLiftParams randomBoundedSet;
	
	randomBoundedSet.angle=alphaAngles.at(getRandomInt(0, alphaAngles.size()-1));
	randomBoundedSet.rho=getRandomDouble(avionics_sim::Lift_drag_model::MIN_AIR_DENSITY,avionics_sim::Lift_drag_model::MAX_AIR_DENSITY);
	randomBoundedSet.vInf=getRandomDouble(avionics_sim::Lift_drag_model::MIN_VINF,avionics_sim::Lift_drag_model::MAX_VINF);

	copy(LUT_alpha.begin(), LUT_alpha.end(), back_inserter(randomBoundedSet.LUT_alpha)); 
    copy(LUT_CL.begin(), LUT_CL.end(), back_inserter(randomBoundedSet.LUT_CL)); 
    copy(LUT_CD.begin(), LUT_CD.end(), back_inserter(randomBoundedSet.LUT_CD));

	calculateLiftValues.push_back(randomBoundedSet);
}

static void setupLiftDragCalculateDragTestData()
{
	LiftDragCalculateDragParams randomBoundedSet;
	
	randomBoundedSet.angle=alphaAngles.at(getRandomInt(0, alphaAngles.size()-1));
	randomBoundedSet.rho=getRandomDouble(avionics_sim::Lift_drag_model::MIN_AIR_DENSITY,avionics_sim::Lift_drag_model::MAX_AIR_DENSITY);
	randomBoundedSet.vInf=getRandomDouble(avionics_sim::Lift_drag_model::MIN_VINF,avionics_sim::Lift_drag_model::MAX_VINF);

	copy(LUT_alpha.begin(), LUT_alpha.end(), back_inserter(randomBoundedSet.LUT_alpha)); 
    copy(LUT_CL.begin(), LUT_CL.end(), back_inserter(randomBoundedSet.LUT_CL)); 
    copy(LUT_CD.begin(), LUT_CD.end(), back_inserter(randomBoundedSet.LUT_CD));

	calculateDragValues.push_back(randomBoundedSet);
}

static void setupLiftDragCalculateDynamicPressureData()
{
	LiftDragCalculateDynamicPressureParams randomBoundedSet;
	
	randomBoundedSet.rho=getRandomDouble(avionics_sim::Lift_drag_model::MIN_AIR_DENSITY,avionics_sim::Lift_drag_model::MAX_AIR_DENSITY);
	randomBoundedSet.vInf=getRandomDouble(avionics_sim::Lift_drag_model::MIN_VINF,avionics_sim::Lift_drag_model::MAX_VINF);

	calculateDynamicPressureValues.push_back(randomBoundedSet);
}


static void setupMotorModelSetThrustData()
{
	MotorModelSetThrustParams exceptionLowestBoundThrust;
	MotorModelSetThrustParams randExceptionLowerBoundThrust;
	MotorModelSetThrustParams zeroThrust;
	MotorModelSetThrustParams minThrustTest;
	MotorModelSetThrustParams maxThrustTest;
	MotorModelSetThrustParams randExceptionUpperBoundThrust;
	MotorModelSetThrustParams exceptionHighestBoundThrust;
	MotorModelSetThrustParams nanThrust;

	double minThrust=0;
	double maxThrust=138;

	exceptionLowestBoundThrust.thrust=std::numeric_limits<double>::lowest();
	exceptionLowestBoundThrust.minThrust=minThrust;
	exceptionLowestBoundThrust.maxThrust=maxThrust;


	randExceptionLowerBoundThrust.thrust=getRandomDouble(std::numeric_limits<double>::lowest()+1, minThrust-1);
	randExceptionLowerBoundThrust.minThrust=minThrust;
	randExceptionLowerBoundThrust.maxThrust=maxThrust;

	zeroThrust.thrust=0;
	zeroThrust.minThrust=minThrust;
	zeroThrust.maxThrust=maxThrust;

	minThrustTest.thrust=minThrust;
	minThrustTest.minThrust=minThrust;
	minThrustTest.maxThrust=maxThrust;

	maxThrustTest.thrust=maxThrust;
	maxThrustTest.minThrust=minThrust;
	maxThrustTest.maxThrust=maxThrust;

	randExceptionUpperBoundThrust.thrust=getRandomDouble(maxThrust+1, std::numeric_limits<double>::max()-1);
	randExceptionUpperBoundThrust.minThrust=getRandomDouble(maxThrust+1, std::numeric_limits<double>::max()-1);
	randExceptionUpperBoundThrust.maxThrust=getRandomDouble(maxThrust+1, std::numeric_limits<double>::max()-1);

	exceptionHighestBoundThrust.thrust=std::numeric_limits<double>::max();
	exceptionHighestBoundThrust.minThrust=minThrust;
	exceptionHighestBoundThrust.maxThrust=maxThrust;

	nanThrust.thrust=std::numeric_limits<double>::quiet_NaN();
	nanThrust.minThrust=minThrust;
	nanThrust.maxThrust=maxThrust;

	setThrustValues.push_back(exceptionLowestBoundThrust);
	setThrustValues.push_back(randExceptionLowerBoundThrust);
	setThrustValues.push_back(zeroThrust);
	setThrustValues.push_back(minThrustTest);
	setThrustValues.push_back(maxThrustTest);
	setThrustValues.push_back(randExceptionUpperBoundThrust);
	setThrustValues.push_back(exceptionHighestBoundThrust);
	setThrustValues.push_back(nanThrust);
}

static void setupMotorModelSetExitVelocityData()
{
	MotorModelSetExitVelocityParams exceptionLowestBoundExitVelocity;
	MotorModelSetExitVelocityParams randExceptionLowerBoundExitVelocity;
	MotorModelSetExitVelocityParams zeroExitVelocity;
	MotorModelSetExitVelocityParams minExitVelocityTest;
	MotorModelSetExitVelocityParams maxExitVelocityTest;
	MotorModelSetExitVelocityParams randExceptionUpperBoundExitVelocity;
	MotorModelSetExitVelocityParams exceptionHighestBoundExitVelocity;
	MotorModelSetExitVelocityParams nanExitVelocity;

	double minExitVelocity=0;
	double maxExitVelocity=138;

	exceptionLowestBoundExitVelocity.exitVelocity=std::numeric_limits<double>::lowest();
	exceptionLowestBoundExitVelocity.minExitVelocity=minExitVelocity;
	exceptionLowestBoundExitVelocity.maxExitVelocity=maxExitVelocity;


	randExceptionLowerBoundExitVelocity.exitVelocity=getRandomDouble(std::numeric_limits<double>::lowest()+1, minExitVelocity-1);
	randExceptionLowerBoundExitVelocity.minExitVelocity=minExitVelocity;
	randExceptionLowerBoundExitVelocity.maxExitVelocity=maxExitVelocity;

	zeroExitVelocity.exitVelocity=0;
	zeroExitVelocity.minExitVelocity=minExitVelocity;
	zeroExitVelocity.maxExitVelocity=maxExitVelocity;

	minExitVelocityTest.exitVelocity=minExitVelocity;
	minExitVelocityTest.minExitVelocity=minExitVelocity;
	minExitVelocityTest.maxExitVelocity=maxExitVelocity;

	maxExitVelocityTest.exitVelocity=maxExitVelocity;
	maxExitVelocityTest.minExitVelocity=minExitVelocity;
	maxExitVelocityTest.maxExitVelocity=maxExitVelocity;

	randExceptionUpperBoundExitVelocity.exitVelocity=getRandomDouble(maxExitVelocity+1, std::numeric_limits<double>::max()-1);
	randExceptionUpperBoundExitVelocity.minExitVelocity=getRandomDouble(maxExitVelocity+1, std::numeric_limits<double>::max()-1);
	randExceptionUpperBoundExitVelocity.maxExitVelocity=getRandomDouble(maxExitVelocity+1, std::numeric_limits<double>::max()-1);

	exceptionHighestBoundExitVelocity.exitVelocity=std::numeric_limits<double>::max();
	exceptionHighestBoundExitVelocity.minExitVelocity=minExitVelocity;
	exceptionHighestBoundExitVelocity.maxExitVelocity=maxExitVelocity;

	nanExitVelocity.exitVelocity=std::numeric_limits<double>::quiet_NaN();
	nanExitVelocity.minExitVelocity=minExitVelocity;
	nanExitVelocity.maxExitVelocity=maxExitVelocity;

	setExitVelocityValues.push_back(exceptionLowestBoundExitVelocity);
	setExitVelocityValues.push_back(randExceptionLowerBoundExitVelocity);
	setExitVelocityValues.push_back(zeroExitVelocity);
	setExitVelocityValues.push_back(minExitVelocityTest);
	setExitVelocityValues.push_back(maxExitVelocityTest);
	setExitVelocityValues.push_back(randExceptionUpperBoundExitVelocity);
	setExitVelocityValues.push_back(exceptionHighestBoundExitVelocity);
	setExitVelocityValues.push_back(nanExitVelocity);
}

int main(int argc, char **argv)
{
	std::string envFilters="";
	const std::string integrationFilter="*Integration*";
	const std::string unitTestFilter="*Unit*";
	const std::string systemTestFilter="*System*";

	//If no flags have been specified (or there has been unexpected number of arguments), display an usage message.
	if ( (argc<minNumArguments) || (argc>maxNumArguments) )
	{
		std::cout<<"Usage: avionics_sim_test_runner {-UnitTests | -IntegrationTests | -SystemTests} [html log filename]"<<std::endl<<std::endl;
		exit(NO_TESTS_EXECUTED);
	}
	else
	{
		std::string first_arg=argv[1];
		if (first_arg.find("-IntegrationTests")!=std::string::npos)
		{
			runIntegrationTests=true;
		}
		else if (first_arg.find("-UnitTests")!=std::string::npos)
		{
			runUnitTests=true;
		}
		else if (first_arg.find("-SystemTests")!=std::string::npos)
		{
			runSystemTests=true;
		}
		else
		{
			std::cout<<"Unrecognized option."<<std::endl<<std::endl;
			exit(NO_TESTS_EXECUTED);
		}
	}

	if (runSystemTests)
	{
		std::cout<<"System tests not yet supported."<<std::endl<<std::endl;
		envFilters=envFilters+systemTestFilter;

		//Exit immediately for now.
		exit(NO_TESTS_EXECUTED);
	}

	if (runUnitTests)
	{
		if (envFilters.length()!=0)
		{
			envFilters=envFilters+":"+unitTestFilter;
		}
		else
		{
			envFilters=unitTestFilter;
		}

		setupCoordUtilsTestData();

		setupLiftDragSetAlphaTestData();

		setupLiftDragSetSpeedTestData();

		setupLiftDragSetAirDensityTestData();

		setupLiftDragLookupCLTestData();

		setupLiftDragLookupCDTestData();

		setupLiftDragAreaTestData();

		setupLiftDragCalculateLiftTestData();

		setupLiftDragCalculateDragTestData();

		setupLiftDragCalculateDynamicPressureData();

		setupMotorModelSetThrustData();

		setupMotorModelSetExitVelocityData();
	}

	if (runIntegrationTests)
	{
		if (envFilters.length()!=0)
		{
			envFilters=envFilters+":"+integrationFilter;
		}
		else
		{
			envFilters=integrationFilter;
		}

		//Read in data for lift drag integration testing.
		XMLParser inputs;
		XMLParser truths;
	
		#ifndef HANDTEST
		std::vector<std::string> inputFiles={ "../integration_tests/TestData/Test_LiftDrag_Enhanced_Plugin_Single_Instance_Inputs_combinations_noprop_controlsurface.txt", 
		"../integration_tests/TestData/Test_LiftDrag_Enhanced_Plugin_Single_Instance_Inputs_combinations_noprop_nocontrolsurface.txt", 
		"../integration_tests/TestData/Test_LiftDrag_Enhanced_Plugin_Single_Instance_Inputs_combinations_prop_controlsurface.txt",
		"../integration_tests/TestData/Test_LiftDrag_Enhanced_Plugin_Single_Instance_Inputs_combinations_prop_nocontrolsurface.txt" };
		std::vector<std::string> truthFiles={ "../integration_tests/TestData/Test_LiftDrag_Enhanced_Plugin_Single_Instance_Truths_combinations_noprop_controlsurface.txt", 
		"../integration_tests/TestData/Test_LiftDrag_Enhanced_Plugin_Single_Instance_Truths_combinations_noprop_nocontrolsurface.txt", 
		"../integration_tests/TestData/Test_LiftDrag_Enhanced_Plugin_Single_Instance_Truths_combinations_prop_controlsurface.txt",
		"../integration_tests/TestData/Test_LiftDrag_Enhanced_Plugin_Single_Instance_Truths_combinations_prop_nocontrolsurface.txt" };
		#endif

		#ifdef HANDTEST
		std::vector<std::string> inputFiles={ "../integration_tests/TestData/Test_LiftDrag_Enhanced_Plugin_Single_Instance_Inputs_handcalc_nopropwash_controlsurface.txt", 
		"../integration_tests/TestData/Test_LiftDrag_Enhanced_Plugin_Single_Instance_Inputs_handcalc_nopropwash_nocontrolsurface.txt", 
		"../integration_tests/TestData/Test_LiftDrag_Enhanced_Plugin_Single_Instance_Inputs_handcalc_propwash_controlsurface.txt",
		"../integration_tests/TestData/Test_LiftDrag_Enhanced_Plugin_Single_Instance_Inputs_handcalc_propwash_nocontrolsurface.txt" };
		std::vector<std::string> truthFiles={ "../integration_tests/TestData/Test_LiftDrag_Enhanced_Plugin_Single_Instance_Truths_handcalc_nopropwash_controlsurface", 
		"../integration_tests/TestData/Test_LiftDrag_Enhanced_Plugin_Single_Instance_Truths_handcalc_nopropwash_nocontrolsurface.txt", 
		"../integration_tests/TestData/Test_LiftDrag_Enhanced_Plugin_Single_Instance_Truths_handcalc_propwash_controlsurface.txt",
		"../integration_tests/TestData/Test_LiftDrag_Enhanced_Plugin_Single_Instance_Truths_handcalc_propwash_nocontrolsurface.txt" };
		#endif

		/*
		Set the number of integration lift drag types. Because there is one ground truth file to each test input data,
		and each test input data represents a different test type, this quantity is equal to the number of input files.
		*/
		int numIntegrationLiftDragTestTypes=inputFiles.size();

		//For each of the combinations, read data input and truth data from their corresponding XML files.
		for (int i=0; i<numIntegrationLiftDragTestTypes;i++)
		{
			inputs.openXMLFile(inputFiles.at(i).c_str());
			truths.openXMLFile(truthFiles.at(i).c_str());
			
			std::vector<std::string> worldLinearVelocitiesData;
			std::vector<std::string> exitVelocitiesData;
			std::vector<std::string> worldOrientationsData;
			std::vector<std::string> controlSurfaceDeflectionsData;
			std::vector<std::string> plantModelsData;
			std::vector<std::string> vInfsData;
			std::vector<std::string> alphaDegData;
			std::vector<std::string> forceTruthData;

			//Store each node data for input into a vector of strings.
			inputs.getXMLVector("worldLinearVels", worldLinearVelocitiesData);
			inputs.getXMLVector("exitVelocities", exitVelocitiesData);
			inputs.getXMLVector("poseVectors", worldOrientationsData);
			inputs.getXMLVector("controlSurfaceDeflections", controlSurfaceDeflectionsData);
			inputs.getXMLVector("plantModels", plantModelsData);

			//Store each node data for truths into a vector of strings.
			truths.getXMLVector("vInfTruth", vInfsData);
			truths.getXMLVector("alphaDeg", alphaDegData);
			truths.getXMLVector("forceTruth", forceTruthData);

			/*
			There will be a one to one relationship, meaning, each param object will have one entry from each vector, 
			so ok to use any given vector as length reference.
			*/
			unsigned int numParams=worldOrientationsData.size();

			if (i==0)
			{
				for (unsigned int j=0; j<numParams; j++)
				{
					LiftDragParams paramsNoPropWashControlSurface;
					getDataVector(j, worldLinearVelocitiesData, paramsNoPropWashControlSurface.worldLinearVelocities);
					getDataVector(j, exitVelocitiesData, paramsNoPropWashControlSurface.exitVelocities);
					getDataVector(j, worldOrientationsData, paramsNoPropWashControlSurface.worldOrientations);
					getDataVector(j, controlSurfaceDeflectionsData, paramsNoPropWashControlSurface.controlSurfaceDeflections);
					getDataVector(j, plantModelsData, paramsNoPropWashControlSurface.plantModels);

					//Alpha and vInfs are scalar, not vector, hence must use getDataFloat instead.
					getDataFloat(j, vInfsData, paramsNoPropWashControlSurface.vInfTruths);
					getDataFloat(j, alphaDegData, paramsNoPropWashControlSurface.alphaDegTruths);
					getDataVector(j, forceTruthData, paramsNoPropWashControlSurface.liftDragForceVectorTruths);
					noPropWashControlSurfaceParamCollections.push_back(paramsNoPropWashControlSurface);
				}
			}

			else if (i==1)
			{
				for (unsigned int j=0; j<numParams; j++)
				{
					LiftDragParams paramsNoPropWashNoControlSurface;
					getDataVector(j, worldLinearVelocitiesData, paramsNoPropWashNoControlSurface.worldLinearVelocities);
					getDataVector(j, exitVelocitiesData, paramsNoPropWashNoControlSurface.exitVelocities);
					getDataVector(j, worldOrientationsData, paramsNoPropWashNoControlSurface.worldOrientations);
					getDataVector(j, controlSurfaceDeflectionsData, paramsNoPropWashNoControlSurface.controlSurfaceDeflections);
					getDataVector(j, plantModelsData, paramsNoPropWashNoControlSurface.plantModels);
					getDataFloat(j, vInfsData, paramsNoPropWashNoControlSurface.vInfTruths);
					getDataFloat(j, alphaDegData, paramsNoPropWashNoControlSurface.alphaDegTruths);
					getDataVector(j, forceTruthData, paramsNoPropWashNoControlSurface.liftDragForceVectorTruths);
					noPropWashNoControlSurfaceParamCollections.push_back(paramsNoPropWashNoControlSurface);
				}
			}

			else if (i==2)
			{
				for (unsigned int j=0; j<numParams; j++)
				{
					LiftDragParams paramsPropWashControlSurface;
					getDataVector(j, worldLinearVelocitiesData, paramsPropWashControlSurface.worldLinearVelocities);
					getDataVector(j, exitVelocitiesData, paramsPropWashControlSurface.exitVelocities);
					getDataVector(j, worldOrientationsData, paramsPropWashControlSurface.worldOrientations);
					getDataVector(j, controlSurfaceDeflectionsData, paramsPropWashControlSurface.controlSurfaceDeflections);
					getDataVector(j, plantModelsData, paramsPropWashControlSurface.plantModels);
					getDataFloat(j, vInfsData, paramsPropWashControlSurface.vInfTruths);
					getDataFloat(j, alphaDegData, paramsPropWashControlSurface.alphaDegTruths);
					getDataVector(j, forceTruthData, paramsPropWashControlSurface.liftDragForceVectorTruths);
					propWashControlSurfaceParamCollections.push_back(paramsPropWashControlSurface);
				}
			}

			else if (i==3)
			{
				for (unsigned int j=0; j<numParams; j++)
				{
					LiftDragParams paramsPropWashNoControlSurface;
					getDataVector(j, worldLinearVelocitiesData, paramsPropWashNoControlSurface.worldLinearVelocities);
					getDataVector(j, exitVelocitiesData, paramsPropWashNoControlSurface.exitVelocities);
					getDataVector(j, worldOrientationsData, paramsPropWashNoControlSurface.worldOrientations);
					getDataVector(j, controlSurfaceDeflectionsData, paramsPropWashNoControlSurface.controlSurfaceDeflections);
					getDataVector(j, plantModelsData, paramsPropWashNoControlSurface.plantModels);
					getDataFloat(j, vInfsData, paramsPropWashNoControlSurface.vInfTruths);
					getDataFloat(j, alphaDegData, paramsPropWashNoControlSurface.alphaDegTruths);
					getDataVector(j, forceTruthData, paramsPropWashNoControlSurface.liftDragForceVectorTruths);
					propWashNoControlSurfaceParamCollections.push_back(paramsPropWashNoControlSurface);
				}
			}
		}
	}

	//Establish test filters.
	::testing::GTEST_FLAG(filter)=envFilters;

	//Add Integration Environment. This contains global tear down tasks, such as report generation, etc.	
	::testing::AddGlobalTestEnvironment(new TestRunnerEnvironment);

	//setLogFilename(std::string filename) {logFileName=filename;}
	TestEventListener* testEventListener;

	//Init Google Test Framework.
	::testing::InitGoogleTest(&argc, argv);

	// Obtain the reference to the active listeners.
  	 ::testing::TestEventListeners& listeners =
      ::testing::UnitTest::GetInstance()->listeners();
	
	// Add custom test event listeners for each test type.
	if (runUnitTests)
	{
		testEventListener=new TestEventListener("unit_test");
	}

	if (runIntegrationTests)
	{
		//Create TestEventListener, populate descriptions for each test.
        testEventListener=new TestEventListener("integration_test");

        std::string inputParams="pitch: {-2pi, 2pi}, increments of pi.<br/> roll: {-2pi, 2pi}, increments of pi.<br/> yaw: \
        {-2pi, 2pi}, increments of pi.<br/> velocity, x component: {0,20}, increments of 20.<br/> \
        velocity, y component: {0,20}, increments of 20.<br/> velocity, z component: {0,20}, increments of 20.<br/> \
        exit velocity, x component: {0,5}, increments of 5.<br/> \
        exit velocity, y component: {0,5}, increments of 5.<br/> \
        exit velocity, z component: {0,5}, increments of 5.<br/> \
        exit velocity, w component: {0,5}, increments of 5.<br/> \
        Control surface deflection values: {-45, -15, 0, 15, 45}<br/> \
        Number of possible control surface slots: 4<br/> \
        Number of slots per used for control surface slot variation: 1 (Each slot tested with one of the five possible \
        deflection values.)";

        testEventListener->addTestClassNameAndDescription("NoPropWashNoControlSurface/CombinatoricalLiftDragTest",
        "No prop wash, no control surface. \n\nInput parameter bounds:\n "+inputParams);

        testEventListener->addTestClassNameAndDescription("NoPropWashControlSurface/CombinatoricalLiftDragTest",
        "No prop wash, control surface. \n\nInput parameter bounds:\n "+inputParams);

        testEventListener->addTestClassNameAndDescription("PropWashNoControlSurface/CombinatoricalLiftDragTest",
        "Prop wash, no control surface. \n\nInput parameter bounds:\n "+inputParams);

        testEventListener->addTestClassNameAndDescription("PropWashControlSurface/CombinatoricalLiftDragTest",
        "Prop wash, control surface. \n\nInput parameter bounds:\n "+inputParams);
	}

	//If the user added specific log filename, use that instead of the one automatically generated.
	if (argc==maxNumArguments)
	{
		std::string userSpecifiedLogFilename=argv[2];
		testEventListener->initLogFile(userSpecifiedLogFilename);
	}
	else
	{
		testEventListener->initLogFile();
	}
	
	
	// Add the custom test event listener.
	listeners.Append(testEventListener);

	//Run all tests
	return RUN_ALL_TESTS();
}

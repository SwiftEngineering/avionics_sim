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
#include "LiftDragSetVelocityParameterized.h"
#include "LiftDragSetAirDensityParameterized.h"
#include "LiftDragLookupCLParameterized.h"
#include "LiftDragLookupCDParameterized.h"
#include "LiftDragSetAreaParameterized.h"
#include "LiftDragCalculateLiftParameterized.h"
#include "LiftDragCalculateDragParameterized.h"
#include "LiftDragCalculateDynamicPressureParameterized.h"
#include "MotorModelSetThrustParameterized.h"
#include "MotorModelSetExitVelocityParameterized.h"
#include "LiftDragCalculateLateralForceParameterized.h"
#include "LiftDragSetBetaParameterized.h"
#include "Atan2Parameterized.h"
#include "LiftDragSetLateralAreaParameterized.h"
#include "LiftDragCalculateLocalVelocitiesParameterized.h"
#include "LiftDragCalculateWindAnglesParameterized.h"
#include "LiftDragCalculateForcesParamsStruct.h"
#include "LiftDragCalculateForcesParameterized.h"
#include "LiftDragCalculateForcesMultiElementParameterized.h"


//Uncomment this to use the smaller hand calc tests.
//#define HANDTEST 1
#define NO_TESTS_EXECUTED 0

std::string alphaLUTcontrolSurfaceDeflectionsString="-180.0000,-175.0000,-170.0000,-165.0000,-160.0000,-155.0000,-150.0000,-145.0000,-140.0000,-135.0000,-130.0000,-125.0000,-120.0000,-115.0000,-110.0000,-105.0000,-100.0000,-95.0000,-90.0000,-85.0000,-80.0000,-75.0000,-70.0000,-65.0000,-60.0000,-55.0000,-50.0000,-45.0000,-40.0000,-35.0000,-30.0000,-27.0000,-26.0000,-25.0000,-24.0000,-23.0000,-22.0000,-21.0000,-20.0000,-19.0000,-18.0000,-17.0000,-16.0000,-15.0000,-14.0000,-13.0000,-12.0000,-11.0000,-10.0000,-9.0000,-8.0000,-7.0000,-6.0000,-5.0000,-4.0000,-3.0000,-2.0000,-1.0000,-0.0000,0.0000,1.0000,2.0000,3.0000,4.0000,5.0000,6.0000,7.0000,8.0000,9.0000,10.0000,11.0000,12.0000,13.0000,14.0000,15.0000,16.0000,17.0000,18.0000,19.0000,20.0000,21.0000,22.0000,23.0000,24.0000,25.0000,26.0000,27.0000,30.0000,35.0000,40.0000,45.0000,50.0000,55.0000,60.0000,65.0000,70.0000,75.0000,80.0000,85.0000,90.0000,95.0000,100.0000,105.0000,110.0000,115.0000,120.0000,125.0000,130.0000,135.0000,140.0000,145.0000,150.0000,155.0000,160.0000,165.0000,170.0000,175.0000,180.0000";

std::string alphaLUTcontrolSurfaceDeflectionsCL="0.0000,0.6900,0.8500,0.6750,0.6600,0.7400,0.8500,0.9100,0.9450,0.9450,0.9100,0.8400,0.7350,0.6250,0.5100,0.3700,0.2200,0.0700,-0.0700,-0.2200,-0.3700,-0.5150,-0.6500,-0.7650,-0.8750,-0.9650,-1.0400,-1.0850,-1.0750,-1.0200,-0.9150,-0.9646,-0.9109,-0.8572,-0.8034,-0.7497,-0.6956,-0.6414,-0.5870,-0.5322,-0.4768,-0.4200,-0.3620,-0.3082,-0.2546,-0.2030,-0.1533,-0.1095,-0.1325,-0.8527,-0.8274,-0.7460,-0.6600,-0.5500,-0.4400,-0.3300,-0.2200,-0.1100,-0.0000,0.0000,0.1100,0.2200,0.3300,0.4400,0.5500,0.6600,0.7460,0.8274,0.8527,0.1325,0.1095,0.1533,0.2030,0.2546,0.3082,0.3620,0.4200,0.4768,0.5322,0.5870,0.6414,0.6956,0.7497,0.8034,0.8572,0.9109,0.9646,0.9150,1.0200,1.0750,1.0850,1.0400,0.9650,0.8750,0.7650,0.6500,0.5150,0.3700,0.2200,0.0700,-0.0700,-0.2200,-0.3700,-0.5100,-0.6250,-0.7350,-0.8400,-0.9100,-0.9450,-0.9450,-0.9100,-0.8500,-0.7400,-0.6600,-0.6750,-0.8500,-0.6900,0.0000";

std::string alphaLUTcontrolSurfaceDeflectionsCD="0.0250,0.0550,0.1400,0.2300,0.3200,0.4200,0.5700,0.7550,0.9250,1.0850,1.2250,1.3500,1.4650,1.5550,1.6350,1.7000,1.7500,1.7800,1.8000,1.8000,1.7800,1.7350,1.6650,1.5750,1.4700,1.3450,1.2150,1.0750,0.9200,0.7450,0.5700,0.4730,0.4460,0.4200,0.3940,0.3690,0.3440,0.3200,0.2970,0.2740,0.2520,0.2310,0.2100,0.1900,0.1710,0.1520,0.1340,0.0760,0.0188,0.0203,0.0185,0.0170,0.0152,0.0140,0.0124,0.0114,0.0108,0.0104,0.0103,0.0103,0.0104,0.0108,0.0114,0.0124,0.0140,0.0152,0.0170,0.0185,0.0203,0.0188,0.0760,0.1340,0.1520,0.1710,0.1900,0.2100,0.2310,0.2520,0.2740,0.2970,0.3200,0.3440,0.3690,0.3940,0.4200,0.4460,0.4730,0.5700,0.7450,0.9200,1.0750,1.2150,1.3450,1.4700,1.5750,1.6650,1.7350,1.7800,1.8000,1.8000,1.7800,1.7500,1.7000,1.6350,1.5550,1.4650,1.3500,1.2250,1.0850,0.9250,0.7550,0.5700,0.4200,0.3200,0.2300,0.1400,0.0550,0.0250";

std::string alphaLUTString="-187.0000,-182.0000,-177.0000,-172.0000,-167.0000,-162.0000,-157.0000,-152.0000,-147.0000,-142.0000,-137.0000,-132.0000,-127.0000,-122.0000,-117.0000,-112.0000,-107.0000,-102.0000,-97.0000,-92.0000,-87.0000,-82.0000,-77.0000,-72.0000,-67.0000,-62.0000,-57.0000,-52.0000,-47.0000,-42.0000,-37.0000,-34.0000,-33.0000,-32.0000,-31.0000,-30.0000,-29.0000,-28.0000,-27.0000,-26.0000,-25.0000,-24.0000,-23.0000,-22.0000,-21.0000,-20.0000,-19.0000,-18.0000,-17.0000,-16.0000,-15.0000,-14.0000,-13.0000,-12.0000,-11.0000,-10.0000,-9.0000,-8.0000,-7.0000,0.0000,1.5000,3.0000,4.0000,5.0000,6.0000,7.0000,8.0000,9.0000,10.0000,11.0000,12.0000,16.0000,20.0000,30.0000,45.0000,60.0000,90.0000,95.0000,98.0000,103.0000,108.0000,113.0000,118.0000,123.0000,128.0000,133.0000,138.0000,143.0000,148.0000,153.0000,158.0000,163.0000,168.0000,173.0000,188.0000";

std::string alphaLUTCL="0.0000,0.6900,0.8500,0.6750,0.6600,0.7400,0.8500,0.9100,0.9450,0.9450,0.9100,0.8400,0.7350,0.6250,0.5100,0.3700,0.2200,0.0700,-0.0700,-0.2200,-0.3700,-0.5150,-0.6500,-0.7650,-0.8750,-0.9650,-1.0400,-1.0850,-1.0750,-1.0200,-0.9150,-0.9646,-0.9109,-0.8572,-0.8034,-0.7497,-0.6956,-0.6414,-0.5870,-0.5322,-0.4768,-0.4200,-0.3620,-0.3082,-0.2546,-0.2030,-0.1533,-0.1095,-0.1325,-0.8527,-0.8274,-0.7460,-0.6600,-0.5500,-0.4400,-0.3300,-0.2200,-0.1100,0.0620,0.7542,0.8987,1.0497,1.1363,1.1897,1.2348,1.2626,1.2554,1.2238,1.1889,1.1587,1.1214,0.9689,0.9104,0.9300,0.8654,0.9227,-0.1147,-0.2602,-0.3700,-0.5100,-0.6250,-0.7350,-0.8400,-0.9100,-0.9450,-0.9450,-0.9100,-0.8500,-0.7400,-0.6600,-0.6750,-0.8500,-0.6900,0.0000,0.6600";

std::string alphaLUTCD="0.0250,0.0550,0.1400,0.2300,0.3200,0.4200,0.5700,0.7550,0.9250,1.0850,1.2250,1.3500,1.4650,1.5550,1.6350,1.7000,1.7500,1.7800,1.8000,1.8000,1.7800,1.7350,1.6650,1.5750,1.4700,1.3450,1.2150,1.0750,0.9200,0.7450,0.5700,0.4730,0.4460,0.4200,0.3940,0.3690,0.3440,0.3200,0.2970,0.2740,0.2520,0.2310,0.2100,0.1900,0.1710,0.1520,0.1340,0.0760,0.0188,0.0203,0.0185,0.0170,0.0152,0.0140,0.0124,0.0114,0.0108,0.0104,0.0456,0.0598,0.0682,0.0794,0.0877,0.1010,0.1161,0.1336,0.1540,0.1814,0.2084,0.2313,0.2578,0.3645,0.4681,0.7118,1.0732,0.7157,1.6811,1.6610,1.7000,1.6350,1.5550,1.4650,1.3500,1.2250,1.0850,0.9250,0.7550,0.5700,0.4200,0.3200,0.2300,0.1400,0.0550,0.0250,0.3200";

std::vector<double> LUT_Alpha_Control_Surface; 

std::vector<double> LUT_CL_Control_Surface; 

std::vector<double> LUT_CD_Control_Surface;

std::vector<double> LUT_Alpha; 

std::vector<double> LUT_CL; 

std::vector<double> LUT_CD;

const int lastNegativeAlphaIdx=58;

LiftDragSetAlphaParameterCollections setAlphaValues;

LiftDragSetVelocityParameterCollections setFreestreamVelocityValues;

LiftDragSetVelocityParameterCollections setPlanarVelocityValues;

LiftDragSetVelocityParameterCollections setLateralVelocityValues;

LiftDragSetAirDensityParameterCollections setAirDensityValues;

LiftDragLookupCLParameterCollections lookupCLValues;

LiftDragLookupCDParameterCollections lookupCDValues;

LiftDragSetAreaParameterCollections setAreaValues;

LiftDragSetLateralAreaParameterCollections setLateralAreaValues;

LiftDragCalculateDynamicPressureParameterCollections calculateDynamicPressureValues;

MotorModelSetExitVelocityParameterCollections setExitVelocityValues;

MotorModelSetThrustParameterCollections setThrustValues;

CoordUtilsDataCollections coordUtilData;

//Parameter collections for each of the four combinations for prop wash and control surface conditions.
LiftDragParameterCollections noPropWashNoControlSurfaceParamCollections;
LiftDragParameterCollections noPropWashControlSurfaceParamCollections;
LiftDragParameterCollections propWashNoControlSurfaceParamCollections;
LiftDragParameterCollections propWashControlSurfaceParamCollections;

LiftDragCalculateLateralForceParameterCollections lateralForceParamCollections;

LiftDragSetBetaParameterCollections setBetaValues;

Atan2ParameterCollections setAtan2Values;

LiftDragCalculateLocalVelocitiesParameterCollections setCalculateLocalVelocitiesValues;

LiftDragCalculateWindAnglesParameterCollections setCalculateWindAnglesValues;

LiftDragCalculateForcesParameterCollections setCalculateForcesValues;

LiftDragCalculateMultiElementForcesParameterCollections threeElementTwoCSValues;

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

//Instantiation of LiftDrag::setFreestreamVelocity method test
INSTANTIATE_TEST_CASE_P(LiftDragUnitTestSetGetFreestreamVelocity,
                        LiftDragSetVelocityParameterized,
                        ::testing::ValuesIn(setFreestreamVelocityValues));

//Instantiation of LiftDrag::setPlanarVelocity method test
INSTANTIATE_TEST_CASE_P(LiftDragUnitTestSetGetPlanarVelocity,
                        LiftDragSetVelocityParameterized,
                        ::testing::ValuesIn(setPlanarVelocityValues));

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

//Instantiation of LiftDrag::setLateralArea method test
INSTANTIATE_TEST_CASE_P(LiftDragUnitTestSetGetLateralArea,
                        LiftDragSetLateralAreaParameterized,
                        ::testing::ValuesIn(setLateralAreaValues));

//Instantiation of LiftDrag::calculateDynamicPressure method test
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

//Instantiation of LiftDrag::calculateLateralForce method test
INSTANTIATE_TEST_CASE_P(LiftDragCalculateLateralForce,
                        LiftDragCalculateLateralForceParameterized,
                        ::testing::ValuesIn(lateralForceParamCollections));

//Instantiation of LiftDrag::setBeta method test
INSTANTIATE_TEST_CASE_P(LiftDragUnitTestSetGetBeta,
                        LiftDragSetBetaParameterized,
                        ::testing::ValuesIn(setBetaValues));

//Instantiation of Atan2 method test
INSTANTIATE_TEST_CASE_P(Atan2Test,
                        Atan2Parameterized,
                        ::testing::ValuesIn(setAtan2Values));

//Instantiation of LiftDrag::calculateLocalVelocities test
INSTANTIATE_TEST_CASE_P(LiftDragCalculateLocalVelocities,
                        LiftDragCalculateLocalVelocitiesParameterized,
                        ::testing::ValuesIn(setCalculateLocalVelocitiesValues));

//Instantiation of LiftDrag::calculateWindAngles test
INSTANTIATE_TEST_CASE_P(LiftDragCalculateWindAngles,
                        LiftDragCalculateWindAnglesParameterized,
                        ::testing::ValuesIn(setCalculateWindAnglesValues));

//Instantiation of LiftDrag::calculateLiftDragModelValues test
INSTANTIATE_TEST_CASE_P(LiftDragCalculateForces,
                        LiftDragCalculateForcesParameterized,
                        ::testing::ValuesIn(setCalculateForcesValues));

//Instantiation of LiftDrag::calculateLiftDragModelValues test
INSTANTIATE_TEST_CASE_P(LiftDragCalculateMultiElementForces,
                        LiftDragCalculateMultiElementForcesParameterized,
                        ::testing::ValuesIn(threeElementTwoCSValues));

bool runUnitTests=false;
bool runIntegrationTests=false;
bool runSystemTests=false;
const int minNumArguments=2;
const int maxNumArguments=3;

static void initLUTTables()
{
	avionics_sim::Bilinear_interp::get1DLUTelementsFromString(alphaLUTString, &LUT_Alpha);
	avionics_sim::Bilinear_interp::get1DLUTelementsFromString(alphaLUTCL, &LUT_CL);
	avionics_sim::Bilinear_interp::get1DLUTelementsFromString(alphaLUTCD, &LUT_CD);
	avionics_sim::Bilinear_interp::get1DLUTelementsFromString(alphaLUTcontrolSurfaceDeflectionsString, &LUT_Alpha_Control_Surface);
	avionics_sim::Bilinear_interp::get1DLUTelementsFromString(alphaLUTcontrolSurfaceDeflectionsCL, &LUT_CL_Control_Surface);
	avionics_sim::Bilinear_interp::get1DLUTelementsFromString(alphaLUTcontrolSurfaceDeflectionsCD, &LUT_CD_Control_Surface);
}

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

		//Until test data is updated to have forward and upward vectors, used a fixed set.
		params.vecFwd=ignition::math::Vector3d(1.0,0.0,0.0);
		params.vecUpward=ignition::math::Vector3d(0.0,0.0,1.0);
		coordUtilData.push_back(params);
	}
}

static void setupLiftDragSetAlphaTestData()
{
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
		LiftDragSetAlphaParams alphaBelowNeg180Degrees;
		LiftDragSetAlphaParams alphaBelowNeg180Radians;
		LiftDragSetAlphaParams alphaAbove180Degrees;
		LiftDragSetAlphaParams alphaAbove180Radians;
		LiftDragSetAlphaParams alphaAt180Degrees;
		LiftDragSetAlphaParams alphaAt180Radians;
		LiftDragSetAlphaParams alphaAtNeg180Degrees;
		LiftDragSetAlphaParams alphaAtNeg180Radians;

		double minRadians=avionics_sim::Lift_drag_model::MIN_AOA/57.2958;
		double maxRadians=avionics_sim::Lift_drag_model::MAX_AOA/57.2958;
		double minRadiansBound=(avionics_sim::Lift_drag_model::MIN_AOA+1)/57.2958;
		double maxRadiansBound=(avionics_sim::Lift_drag_model::MAX_AOA-1)/57.2958;
		double minDegreesBound=(avionics_sim::Lift_drag_model::MIN_AOA+1);
		double maxDegreesBound=(avionics_sim::Lift_drag_model::MAX_AOA-1);
		double oneEightyDegreesBound=180.0;
		double oneEightyRadiansBound=180.0/57.2958;
		double negOneEightyDegreesBound=-180.0;
		double negOneEightyRadiansBound=-180.0/57.2958;
		randLowerBoundAlphaRadians.isRadians=true;
		randLowerBoundAlphaRadians.value=getRandomDouble(minRadiansBound, -1/57.2958);
		randUpperBoundAlphaRadians.isRadians=true;
		randUpperBoundAlphaRadians.value=getRandomDouble(1/57.2958, maxRadiansBound);
		zeroAlphaRadians.isRadians=true;
		zeroAlphaRadians.value=0/57.2958;
		lowestBoundAlphaRadians.isRadians=true;
		lowestBoundAlphaRadians.value=minRadians;
		highestBoundAlphaRadians.isRadians=true;
		highestBoundAlphaRadians.value=maxRadians;
		lowerBoundAlphaRadians.isRadians=true;
		lowerBoundAlphaRadians.value=minRadians;
		upperBoundAlphaRadians.isRadians=true;
		upperBoundAlphaRadians.value=maxRadians;
		
		setAlphaValues.push_back(randLowerBoundAlphaRadians);
		setAlphaValues.push_back(randUpperBoundAlphaRadians);
		setAlphaValues.push_back(zeroAlphaRadians);
		setAlphaValues.push_back(lowestBoundAlphaRadians);
		setAlphaValues.push_back(highestBoundAlphaRadians);
		setAlphaValues.push_back(lowerBoundAlphaRadians);
		setAlphaValues.push_back(upperBoundAlphaRadians);

		randLowerBoundAlphaDegrees.isRadians=false;
		randLowerBoundAlphaDegrees.value=getRandomDouble(minDegreesBound, -1);
		randUpperBoundAlphaDegrees.isRadians=false;
		randUpperBoundAlphaDegrees.value=getRandomDouble(1, maxDegreesBound);
		zeroAlphaDegrees.isRadians=false;
		zeroAlphaDegrees.value=0;
		lowestBoundAlphaDegrees.isRadians=false;
		lowestBoundAlphaDegrees.value=avionics_sim::Lift_drag_model::MIN_AOA;
		highestBoundAlphaDegrees.isRadians=false;
		highestBoundAlphaDegrees.value=avionics_sim::Lift_drag_model::MAX_AOA;

		setAlphaValues.push_back(randLowerBoundAlphaDegrees);
		setAlphaValues.push_back(randUpperBoundAlphaDegrees);
		setAlphaValues.push_back(zeroAlphaDegrees);
		setAlphaValues.push_back(lowestBoundAlphaDegrees);
		setAlphaValues.push_back(highestBoundAlphaDegrees);

		randExceptionLowerBoundAlpha.isRadians=false;
		randExceptionLowerBoundAlpha.value=getRandomDouble(-180, avionics_sim::Lift_drag_model::MIN_AOA-1);

		randExceptionUpperBoundAlpha.isRadians=false;
		randExceptionUpperBoundAlpha.value=getRandomDouble(avionics_sim::Lift_drag_model::MAX_AOA+1, 180);

		exceptionLowestBoundAlpha.isRadians=false;
		exceptionLowestBoundAlpha.value=-180;

		exceptionHighestBoundAlpha.isRadians=false;
		exceptionHighestBoundAlpha.value=180;
		
		nanAlpha.isRadians=false;
		nanAlpha.value=std::numeric_limits<double>::quiet_NaN();

		vInfLessThanMin.isRadians=false;
		vInfLessThanMin.value=0;

		setAlphaValues.push_back(randExceptionLowerBoundAlpha);
		setAlphaValues.push_back(randExceptionUpperBoundAlpha);
		setAlphaValues.push_back(exceptionLowestBoundAlpha);
		setAlphaValues.push_back(exceptionHighestBoundAlpha);
		setAlphaValues.push_back(nanAlpha);
		setAlphaValues.push_back(vInfLessThanMin);

		alphaBelowNeg180Degrees.isRadians=false;
		alphaBelowNeg180Degrees.value=getRandomDouble(-1080, -180);

		alphaBelowNeg180Radians.isRadians=true;
		alphaBelowNeg180Radians.value=getRandomDouble(-18.84956, -3.14159);

		alphaAbove180Degrees.isRadians=false;
		alphaAbove180Degrees.value=getRandomDouble(180, 1080);

		alphaAbove180Radians.isRadians=true;
		alphaAbove180Radians.value=getRandomDouble(3.14159, 18.84956);

		setAlphaValues.push_back(alphaBelowNeg180Degrees);
		setAlphaValues.push_back(alphaBelowNeg180Radians);
		setAlphaValues.push_back(alphaAbove180Degrees);
		setAlphaValues.push_back(alphaAbove180Radians);

		alphaAt180Degrees.isRadians=false;
		alphaAt180Degrees.value=oneEightyDegreesBound;
		alphaAt180Radians.isRadians=true;
		alphaAt180Radians.value=oneEightyRadiansBound;

		setAlphaValues.push_back(alphaAt180Degrees);
		setAlphaValues.push_back(alphaAt180Radians);

		alphaAtNeg180Degrees.isRadians=false;
		alphaAtNeg180Degrees.value=negOneEightyDegreesBound;
		alphaAtNeg180Radians.isRadians=true;
		alphaAtNeg180Radians.value=negOneEightyRadiansBound;

		setAlphaValues.push_back(alphaAtNeg180Degrees);
		setAlphaValues.push_back(alphaAtNeg180Radians);
}

static void setupLiftDragSetFreestreamVelocityTestData()
{
	LiftDragSetVelocityParams exceptionLowestBoundVelocity;
	LiftDragSetVelocityParams randExceptionLowerBoundVelocity;
	LiftDragSetVelocityParams zeroVelocity;
	LiftDragSetVelocityParams minVelocity;
	LiftDragSetVelocityParams maxVelocity;
	LiftDragSetVelocityParams randExceptionUpperBoundVelocity;
	LiftDragSetVelocityParams exceptionHighestBoundVelocity;
	LiftDragSetVelocityParams nanVelocity;
	LiftDragSetVelocityParams randomRealisticVelocity;

	exceptionLowestBoundVelocity.value=std::numeric_limits<double>::lowest();

	randExceptionLowerBoundVelocity.value=getRandomDouble(std::numeric_limits<double>::lowest()+1, avionics_sim::Lift_drag_model::MIN_VINF-1);

	zeroVelocity.value=0;

	minVelocity.value=avionics_sim::Lift_drag_model::MIN_VINF;

	maxVelocity.value=avionics_sim::Lift_drag_model::MAX_VINF;

	randExceptionUpperBoundVelocity.value=getRandomDouble(avionics_sim::Lift_drag_model::MAX_VINF+1, std::numeric_limits<double>::max()-1);

	exceptionHighestBoundVelocity.value=std::numeric_limits<double>::max();

	nanVelocity.value=std::numeric_limits<double>::quiet_NaN();

	randomRealisticVelocity.value=getRandomDouble(avionics_sim::Lift_drag_model::MIN_VINF, avionics_sim::Lift_drag_model::MAX_VINF);

	setFreestreamVelocityValues.push_back(exceptionLowestBoundVelocity);
	setFreestreamVelocityValues.push_back(randExceptionLowerBoundVelocity);
	setFreestreamVelocityValues.push_back(zeroVelocity);
	setFreestreamVelocityValues.push_back(minVelocity);
	setFreestreamVelocityValues.push_back(maxVelocity);
	setFreestreamVelocityValues.push_back(randExceptionUpperBoundVelocity);
	setFreestreamVelocityValues.push_back(exceptionHighestBoundVelocity);
	setFreestreamVelocityValues.push_back(nanVelocity);
	setFreestreamVelocityValues.push_back(randomRealisticVelocity);
}

static void setupLiftDragSetPlanarVelocityTestData()
{
	LiftDragSetVelocityParams exceptionLowestBoundVelocity;
	LiftDragSetVelocityParams randExceptionLowerBoundVelocity;
	LiftDragSetVelocityParams zeroVelocity;
	LiftDragSetVelocityParams minVelocity;
	LiftDragSetVelocityParams maxVelocity;
	LiftDragSetVelocityParams randExceptionUpperBoundVelocity;
	LiftDragSetVelocityParams exceptionHighestBoundVelocity;
	LiftDragSetVelocityParams nanVelocity;
	LiftDragSetVelocityParams randomRealisticVelocity;

	exceptionLowestBoundVelocity.value=std::numeric_limits<double>::lowest();

	randExceptionLowerBoundVelocity.value=getRandomDouble(std::numeric_limits<double>::lowest()+1, avionics_sim::Lift_drag_model::MIN_VINF-1);

	zeroVelocity.value=0;

	minVelocity.value=avionics_sim::Lift_drag_model::MIN_VINF;

	maxVelocity.value=avionics_sim::Lift_drag_model::MAX_VINF;

	randExceptionUpperBoundVelocity.value=getRandomDouble(avionics_sim::Lift_drag_model::MAX_VINF+1, std::numeric_limits<double>::max()-1);

	exceptionHighestBoundVelocity.value=std::numeric_limits<double>::max();

	nanVelocity.value=std::numeric_limits<double>::quiet_NaN();

	randomRealisticVelocity.value=getRandomDouble(avionics_sim::Lift_drag_model::MIN_VINF-1, avionics_sim::Lift_drag_model::MAX_VINF+1);

	setPlanarVelocityValues.push_back(exceptionLowestBoundVelocity);
	setPlanarVelocityValues.push_back(randExceptionLowerBoundVelocity);
	setPlanarVelocityValues.push_back(zeroVelocity);
	setPlanarVelocityValues.push_back(minVelocity);
	setPlanarVelocityValues.push_back(maxVelocity);
	setPlanarVelocityValues.push_back(randExceptionUpperBoundVelocity);
	setPlanarVelocityValues.push_back(exceptionHighestBoundVelocity);
	setPlanarVelocityValues.push_back(nanVelocity);
	setPlanarVelocityValues.push_back(randomRealisticVelocity);
}

static void setupLiftDragSetLateralVelocityTestData()
{
	LiftDragSetVelocityParams exceptionLowestBoundVelocity;
	LiftDragSetVelocityParams randExceptionLowerBoundVelocity;
	LiftDragSetVelocityParams zeroVelocity;
	LiftDragSetVelocityParams minVelocity;
	LiftDragSetVelocityParams maxVelocity;
	LiftDragSetVelocityParams randExceptionUpperBoundVelocity;
	LiftDragSetVelocityParams exceptionHighestBoundVelocity;
	LiftDragSetVelocityParams nanVelocity;
	LiftDragSetVelocityParams randomRealisticVelocity;

	exceptionLowestBoundVelocity.value=std::numeric_limits<double>::lowest();

	randExceptionLowerBoundVelocity.value=getRandomDouble(std::numeric_limits<double>::lowest()+1, avionics_sim::Lift_drag_model::MIN_LATERAL_VELOCITY-1);

	zeroVelocity.value=0;

	minVelocity.value=avionics_sim::Lift_drag_model::MIN_LATERAL_VELOCITY;

	maxVelocity.value=avionics_sim::Lift_drag_model::MAX_LATERAL_VELOCITY;

	randExceptionUpperBoundVelocity.value=getRandomDouble(avionics_sim::Lift_drag_model::MAX_LATERAL_VELOCITY+1, std::numeric_limits<double>::max()-1);

	exceptionHighestBoundVelocity.value=std::numeric_limits<double>::max();

	nanVelocity.value=std::numeric_limits<double>::quiet_NaN();

	randomRealisticVelocity.value=getRandomDouble(avionics_sim::Lift_drag_model::MIN_LATERAL_VELOCITY+1, avionics_sim::Lift_drag_model::MAX_LATERAL_VELOCITY-1);

	setLateralVelocityValues.push_back(exceptionLowestBoundVelocity);
	setLateralVelocityValues.push_back(randExceptionLowerBoundVelocity);
	setLateralVelocityValues.push_back(zeroVelocity);
	setLateralVelocityValues.push_back(minVelocity);
	setLateralVelocityValues.push_back(maxVelocity);
	setLateralVelocityValues.push_back(randExceptionUpperBoundVelocity);
	setLateralVelocityValues.push_back(exceptionHighestBoundVelocity);
	setLateralVelocityValues.push_back(nanVelocity);
	setLateralVelocityValues.push_back(randomRealisticVelocity);
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
    copy(LUT_Alpha_Control_Surface.begin(), LUT_Alpha_Control_Surface.end(), back_inserter(zeroAlpha.LUT_alpha)); 
    copy(LUT_CL_Control_Surface.begin(), LUT_CL_Control_Surface.end(), back_inserter(zeroAlpha.LUT_CL)); 
    copy(LUT_CD_Control_Surface.begin(), LUT_CD_Control_Surface.end(), back_inserter(zeroAlpha.LUT_CD)); 

	minAlpha.value=avionics_sim::Lift_drag_model::MIN_AOA;
	copy(LUT_Alpha_Control_Surface.begin(), LUT_Alpha_Control_Surface.end(), back_inserter(minAlpha.LUT_alpha)); 
	// Copying vector by copy function 
    copy(LUT_CL_Control_Surface.begin(), LUT_CL_Control_Surface.end(), back_inserter(minAlpha.LUT_CL)); 
	// Copying vector by copy function 
    copy(LUT_CD_Control_Surface.begin(), LUT_CD_Control_Surface.end(), back_inserter(minAlpha.LUT_CD)); 

	maxAlpha.value=avionics_sim::Lift_drag_model::MAX_AOA;
	copy(LUT_Alpha_Control_Surface.begin(), LUT_Alpha_Control_Surface.end(), back_inserter(maxAlpha.LUT_alpha)); 
	// Copying vector by copy function 
    copy(LUT_CL_Control_Surface.begin(), LUT_CL_Control_Surface.end(), back_inserter(maxAlpha.LUT_CL)); 
	// Copying vector by copy function 
    copy(LUT_CD_Control_Surface.begin(), LUT_CD_Control_Surface.end(), back_inserter(maxAlpha.LUT_CD));

	//Choose a non-zero random number between MIN_AOA and MAX_AOA
	double randAlpha=0;
	while (randAlpha==0)
	{
		randAlpha=LUT_Alpha_Control_Surface.at(getRandomInt(0, LUT_Alpha_Control_Surface.size()-1));
	}
	randAlphaBtwMinAndMax.value=randAlpha;
	copy(LUT_Alpha_Control_Surface.begin(), LUT_Alpha_Control_Surface.end(), back_inserter(randAlphaBtwMinAndMax.LUT_alpha)); 
	// Copying vector by copy function 
    copy(LUT_CL_Control_Surface.begin(), LUT_CL_Control_Surface.end(), back_inserter(randAlphaBtwMinAndMax.LUT_CL)); 
	// Copying vector by copy function 
    copy(LUT_CD_Control_Surface.begin(), LUT_CD_Control_Surface.end(), back_inserter(randAlphaBtwMinAndMax.LUT_CD));

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
	copy(LUT_Alpha_Control_Surface.begin(), LUT_Alpha_Control_Surface.end(), back_inserter(zeroAlpha.LUT_alpha)); 
    copy(LUT_CL_Control_Surface.begin(), LUT_CL_Control_Surface.end(), back_inserter(zeroAlpha.LUT_CL)); 
    copy(LUT_CD_Control_Surface.begin(), LUT_CD_Control_Surface.end(), back_inserter(zeroAlpha.LUT_CD)); 

	minAlpha.value=avionics_sim::Lift_drag_model::MIN_AOA;
	copy(LUT_Alpha_Control_Surface.begin(), LUT_Alpha_Control_Surface.end(), back_inserter(minAlpha.LUT_alpha)); 
    copy(LUT_CL_Control_Surface.begin(), LUT_CL_Control_Surface.end(), back_inserter(minAlpha.LUT_CL)); 
    copy(LUT_CD_Control_Surface.begin(), LUT_CD_Control_Surface.end(), back_inserter(minAlpha.LUT_CD)); 

	maxAlpha.value=avionics_sim::Lift_drag_model::MAX_AOA;
	copy(LUT_Alpha_Control_Surface.begin(), LUT_Alpha_Control_Surface.end(), back_inserter(maxAlpha.LUT_alpha)); 
    copy(LUT_CL_Control_Surface.begin(), LUT_CL_Control_Surface.end(), back_inserter(maxAlpha.LUT_CL)); 
    copy(LUT_CD_Control_Surface.begin(), LUT_CD_Control_Surface.end(), back_inserter(maxAlpha.LUT_CD)); 

	//Choose a random, non-zero angle from the LUT.
	double randAlpha=0;
	while (randAlpha==0)
	{
		randAlpha=LUT_Alpha_Control_Surface.at(getRandomInt(0, LUT_Alpha_Control_Surface.size()-1));
	}
	
	randAlphaBtwMinAndMax.value=randAlpha;
	copy(LUT_Alpha_Control_Surface.begin(), LUT_Alpha_Control_Surface.end(), back_inserter(randAlphaBtwMinAndMax.LUT_alpha)); 
    copy(LUT_CL_Control_Surface.begin(), LUT_CL_Control_Surface.end(), back_inserter(randAlphaBtwMinAndMax.LUT_CL)); 
    copy(LUT_CD_Control_Surface.begin(), LUT_CD_Control_Surface.end(), back_inserter(randAlphaBtwMinAndMax.LUT_CD)); 

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

static void setupLiftDragLateralAreaTestData()
{
	LiftDragSetLateralAreaParams exceptionLowestBoundArea;
	LiftDragSetLateralAreaParams randExceptionLowerBoundArea;
	LiftDragSetLateralAreaParams zeroArea;
	LiftDragSetLateralAreaParams minArea;
	LiftDragSetLateralAreaParams maxArea;
	LiftDragSetLateralAreaParams randExceptionUpperBoundArea;
	LiftDragSetLateralAreaParams exceptionHighestBoundArea;
	LiftDragSetLateralAreaParams nanArea;

	exceptionLowestBoundArea.value=std::numeric_limits<double>::lowest();

	randExceptionLowerBoundArea.value=getRandomDouble(std::numeric_limits<double>::lowest()+1, avionics_sim::Lift_drag_model::MIN_AREA-1);

	zeroArea.value=0;

	minArea.value=avionics_sim::Lift_drag_model::MIN_AREA;

	maxArea.value=avionics_sim::Lift_drag_model::MAX_AREA;

	randExceptionUpperBoundArea.value=getRandomDouble(avionics_sim::Lift_drag_model::MAX_AREA+1, std::numeric_limits<double>::max()-1);

	exceptionHighestBoundArea.value=std::numeric_limits<double>::max();

	nanArea.value=std::numeric_limits<double>::quiet_NaN();

	setLateralAreaValues.push_back(exceptionLowestBoundArea);
	setLateralAreaValues.push_back(randExceptionLowerBoundArea);
	setLateralAreaValues.push_back(zeroArea);
	setLateralAreaValues.push_back(minArea);
	setLateralAreaValues.push_back(maxArea);
	setLateralAreaValues.push_back(randExceptionUpperBoundArea);
	setLateralAreaValues.push_back(exceptionHighestBoundArea);
	setLateralAreaValues.push_back(nanArea);
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

static void setupLiftDragSetBetaTestData()
{
		LiftDragSetBetaParams randLowerBoundBetaRadians;
		LiftDragSetBetaParams randUpperBoundBetaRadians;
		LiftDragSetBetaParams zeroBetaRadians;
		LiftDragSetBetaParams lowerBoundBetaRadians;
		LiftDragSetBetaParams upperBoundBetaRadians;
		LiftDragSetBetaParams lowestBoundBetaRadians;
		LiftDragSetBetaParams highestBoundBetaRadians;

		LiftDragSetBetaParams randLowerBoundBetaDegrees;
		LiftDragSetBetaParams randUpperBoundBetaDegrees;
		LiftDragSetBetaParams zeroBetaDegrees;
		LiftDragSetBetaParams lowestBoundBetaDegrees;
		LiftDragSetBetaParams highestBoundBetaDegrees;

		LiftDragSetBetaParams randExceptionLowerBoundBeta;
		LiftDragSetBetaParams randExceptionUpperBoundBeta;
		LiftDragSetBetaParams exceptionLowestBoundBeta;
		LiftDragSetBetaParams exceptionHighestBoundBeta;
		LiftDragSetBetaParams nanBeta;

		LiftDragSetBetaParams betaBelowNeg180Degrees;
		LiftDragSetBetaParams betaBelowNeg180Radians;
		LiftDragSetBetaParams betaAbove180Degrees;
		LiftDragSetBetaParams betaAbove180Radians;
		LiftDragSetBetaParams betaAt180Degrees;
		LiftDragSetBetaParams betaAt180Radians;
		LiftDragSetBetaParams betaAtNeg180Degrees;
		LiftDragSetBetaParams betaAtNeg180Radians;
		

		double minRadians=avionics_sim::Lift_drag_model::MIN_AOA/57.2958;
		double maxRadians=avionics_sim::Lift_drag_model::MAX_AOA/57.2958;
		double minRadiansBound=(avionics_sim::Lift_drag_model::MIN_AOA+1)/57.2958;
		double maxRadiansBound=(avionics_sim::Lift_drag_model::MAX_AOA-1)/57.2958;
		double minDegreesBound=(avionics_sim::Lift_drag_model::MIN_AOA+1);
		double maxDegreesBound=(avionics_sim::Lift_drag_model::MAX_AOA-1);
		double oneEightyDegreesBound=180.0;
		double oneEightyRadiansBound=180.0/57.2958;
		double negOneEightyDegreesBound=-180.0;
		double negOneEightyRadiansBound=-180.0/57.2958;

		betaAt180Degrees.isRadians=false;
		betaAt180Degrees.value=oneEightyDegreesBound;
		betaAt180Radians.isRadians=true;
		betaAt180Radians.value=oneEightyRadiansBound;

		randLowerBoundBetaRadians.isRadians=true;
		randLowerBoundBetaRadians.value=getRandomDouble(minRadiansBound, -1/57.2958);

		randUpperBoundBetaRadians.isRadians=true;
		randUpperBoundBetaRadians.value=getRandomDouble(1/57.2958, maxRadiansBound);
		
		zeroBetaRadians.isRadians=true;
		zeroBetaRadians.value=0/57.2958;

		lowestBoundBetaRadians.isRadians=true;
		lowestBoundBetaRadians.value=minRadians;

		highestBoundBetaRadians.isRadians=true;
		highestBoundBetaRadians.value=maxRadians;
		
		lowerBoundBetaRadians.isRadians=true;
		lowerBoundBetaRadians.value=minRadians;
		
		upperBoundBetaRadians.isRadians=true;
		upperBoundBetaRadians.value=maxRadians;
		
		setBetaValues.push_back(randLowerBoundBetaRadians);
		setBetaValues.push_back(randUpperBoundBetaRadians);
		setBetaValues.push_back(zeroBetaRadians);
		setBetaValues.push_back(lowestBoundBetaRadians);
		setBetaValues.push_back(highestBoundBetaRadians);
		setBetaValues.push_back(lowerBoundBetaRadians);
		setBetaValues.push_back(upperBoundBetaRadians);

		randLowerBoundBetaDegrees.isRadians=false;
		randLowerBoundBetaDegrees.value=getRandomDouble(minDegreesBound, -1);

		randUpperBoundBetaDegrees.isRadians=false;
		randUpperBoundBetaDegrees.value=getRandomDouble(1, maxDegreesBound);

		zeroBetaDegrees.isRadians=false;
		zeroBetaDegrees.value=0;

		lowestBoundBetaDegrees.isRadians=false;
		lowestBoundBetaDegrees.value=avionics_sim::Lift_drag_model::MIN_AOA;

		highestBoundBetaDegrees.isRadians=false;
		highestBoundBetaDegrees.value=avionics_sim::Lift_drag_model::MAX_AOA;

		setBetaValues.push_back(randLowerBoundBetaDegrees);
		setBetaValues.push_back(randUpperBoundBetaDegrees);
		setBetaValues.push_back(zeroBetaDegrees);
		setBetaValues.push_back(lowestBoundBetaDegrees);
		setBetaValues.push_back(highestBoundBetaDegrees);

		randExceptionLowerBoundBeta.isRadians=false;
		randExceptionLowerBoundBeta.value=getRandomDouble(-1080, avionics_sim::Lift_drag_model::MIN_AOA-1);

		randExceptionUpperBoundBeta.isRadians=false;
		randExceptionUpperBoundBeta.value=getRandomDouble(avionics_sim::Lift_drag_model::MAX_AOA+1, 1080);

		exceptionLowestBoundBeta.isRadians=false;
		exceptionLowestBoundBeta.value=-1080;

		exceptionHighestBoundBeta.isRadians=false;
		exceptionHighestBoundBeta.value=1080;
		
		nanBeta.isRadians=false;
		nanBeta.value=std::numeric_limits<double>::quiet_NaN();

		setBetaValues.push_back(randExceptionLowerBoundBeta);
		setBetaValues.push_back(randExceptionUpperBoundBeta);
		setBetaValues.push_back(exceptionLowestBoundBeta);
		setBetaValues.push_back(exceptionHighestBoundBeta);
		setBetaValues.push_back(nanBeta);

		//Don't want to go too low or high in choosing a random number...
		betaBelowNeg180Degrees.isRadians=false;
		betaBelowNeg180Degrees.value=getRandomDouble(-1080, -180);

		betaBelowNeg180Radians.isRadians=true;
		betaBelowNeg180Radians.value=getRandomDouble(-18.84956, -3.14159);

		betaAbove180Degrees.isRadians=false;
		betaAbove180Degrees.value=getRandomDouble(180, 1080);

		betaAbove180Radians.isRadians=true;
		betaAbove180Radians.value=getRandomDouble(3.14159, 18.84956);

		setBetaValues.push_back(betaBelowNeg180Degrees);
		setBetaValues.push_back(betaBelowNeg180Radians);
		setBetaValues.push_back(betaAbove180Degrees);
		setBetaValues.push_back(betaAbove180Radians);
		setBetaValues.push_back(betaAt180Degrees);
		setBetaValues.push_back(betaAt180Radians);

		betaAtNeg180Degrees.isRadians=false;
		betaAtNeg180Degrees.value=negOneEightyDegreesBound;
		betaAtNeg180Radians.isRadians=true;
		betaAtNeg180Radians.value=negOneEightyRadiansBound;

		setBetaValues.push_back(betaAtNeg180Degrees);
		setBetaValues.push_back(betaAtNeg180Radians);
}

static void setupAtan2TestData()
{
		Atan2Params xyZero;
		Atan2Params xZero;
		Atan2Params yZero;

		xyZero.x=0;
		xyZero.y=0;

		xZero.x=0;
		xZero.y=getRandomDouble(1, std::numeric_limits<double>::max());

		yZero.x=getRandomDouble(1, std::numeric_limits<double>::max());
		yZero.y=0;

		setAtan2Values.push_back(xyZero);
		setAtan2Values.push_back(xZero);
		setAtan2Values.push_back(yZero);
}

static void setupCalculateLocalVelocitiesData()
{
	LiftDragCalculateLocalVelocitiesParams markdownInput;
	markdownInput.inputWorldVel=ignition::math::Vector3d(10.0,1.0,0.1);
	markdownInput.inputPose=ignition::math::Pose3d(0.0,0.0,0.0,-0.09,1.48,0.1);
	markdownInput.correctVPlanar=10;
	markdownInput.correctVBody=ignition::math::Vector3d(0.9,-0.9,9.9);
	markdownInput.correctVLateral=markdownInput.correctVBody.Y();
	markdownInput.correctVinf=markdownInput.correctVBody.Length();
	setCalculateLocalVelocitiesValues.push_back(markdownInput);
}

static void setupCalculateWindAnglesData()
{
	LiftDragCalculateWindAnglesParams markdownInput;
	markdownInput.inputWorldVel=ignition::math::Vector3d(10.0,1.0,0.1);
	markdownInput.inputPose=ignition::math::Pose3d(0.0,0.0,0.0,-0.09,1.48,0.1);
	markdownInput.vFwd=ignition::math::Vector3d(0.0,0.0,1.0);
	markdownInput.vUpwd=ignition::math::Vector3d(-1.0,0.0,0.0);

	//Lift drag model stores degrees in angles, as lookup table requires input in degrees. Doing conversion to radians in test.
	markdownInput.correctAlpha= 0.0812;
	markdownInput.correctBeta=0.0900;
	setCalculateWindAnglesValues.push_back(markdownInput);
}

static void setupCalculateForcesData()
{
	LiftDragCalculateForcesParams unspecificCase;
	unspecificCase.inputWorldVel=ignition::math::Vector3d(10.0,1.0,0.1);
	unspecificCase.inputPose=ignition::math::Pose3d(0.0,0.0,0.0,-0.09,1.48,0.1);
	unspecificCase.vFwd=ignition::math::Vector3d(0.0,0.0,1.0);
	unspecificCase.vUpwd=ignition::math::Vector3d(-1.0,0.0,0.0);
	unspecificCase.area=1.0;
	unspecificCase.rho=1.225;
	unspecificCase.motorExitVelocity=0.0;
	unspecificCase.controlAlpha=0.0;
	
	copy(LUT_Alpha_Control_Surface.begin(), LUT_Alpha_Control_Surface.end(), back_inserter(unspecificCase.LUT_alpha)); 
    copy(LUT_CL_Control_Surface.begin(), LUT_CL_Control_Surface.end(), back_inserter(unspecificCase.LUT_CL)); 
    copy(LUT_CD_Control_Surface.begin(), LUT_CD_Control_Surface.end(), back_inserter(unspecificCase.LUT_CD));
	
	unspecificCase.trueLift=ignition::math::Vector3d(-31.3604,0.0,0.0);
	unspecificCase.trueDrag=ignition::math::Vector3d(0.0,0.0,1.7237);
	unspecificCase.trueLateralForce=ignition::math::Vector3d(0.0,0.0,0.0);
	
    unspecificCase.hasMotorExitVelocity=false;
    unspecificCase.isControlSurface=false;
	unspecificCase.isNonSpecific=true;
	setCalculateForcesValues.push_back(unspecificCase);
}

static void setupCalculateMultiElementForcesDataTwoCSPlusMainWing()
{
	LiftDragCalculateMultiElementForcesParams threeElemTwoCS;

	LiftDragCalculateForcesParams mainWing;
	mainWing.inputWorldVel=ignition::math::Vector3d(10.0,1.0,0.1);
	mainWing.inputPose=ignition::math::Pose3d(0.0,0.0,0.0,-0.09,1.48,0.1);
	mainWing.vFwd=ignition::math::Vector3d(0.0,0.0,1.0);
	mainWing.vUpwd=ignition::math::Vector3d(-1.0,0.0,0.0);
	mainWing.area=1.3290;
	mainWing.rho=1.225;
	mainWing.motorExitVelocity=0.0;
	mainWing.controlAlpha=4.6510;
	copy(LUT_Alpha.begin(), LUT_Alpha.end(), back_inserter(mainWing.LUT_alpha)); 
    copy(LUT_CL.begin(), LUT_CL.end(), back_inserter(mainWing.LUT_CL)); 
    copy(LUT_CD.begin(), LUT_CD.end(), back_inserter(mainWing.LUT_CD));
	mainWing.trueLift=ignition::math::Vector3d(95.5105,0.0,0.0);
	mainWing.trueDrag=ignition::math::Vector3d(0.0,0.0,7.8589);
	mainWing.trueLateralForce=ignition::math::Vector3d(0.0,0.0,0.0);
    mainWing.hasMotorExitVelocity=false;
	mainWing.motorExitVelocity=0;
    mainWing.isControlSurface=false;
	mainWing.isNonSpecific=false;
	mainWing.linkName="Main Wing";
	mainWing.vInf=10.0097;

	LiftDragCalculateForcesParams firstCSWing;
	firstCSWing.inputWorldVel=ignition::math::Vector3d(10.0,1.0,0.1);
	firstCSWing.inputPose=ignition::math::Pose3d(0.0,0.0,0.0,-0.09,1.48,0.1);
	firstCSWing.vFwd=ignition::math::Vector3d(0.0,0.0,1.0);
	firstCSWing.vUpwd=ignition::math::Vector3d(-1.0,0.0,0.0);
	firstCSWing.area=0.0492;
	firstCSWing.rho=1.225;
	firstCSWing.motorExitVelocity=0.0;
	firstCSWing.controlAlpha=11.25;
	copy(LUT_Alpha_Control_Surface.begin(), LUT_Alpha_Control_Surface.end(), back_inserter(firstCSWing.LUT_alpha)); 
    copy(LUT_CL_Control_Surface.begin(), LUT_CL_Control_Surface.end(), back_inserter(firstCSWing.LUT_CL)); 
    copy(LUT_CD_Control_Surface.begin(), LUT_CD_Control_Surface.end(), back_inserter(firstCSWing.LUT_CD));
	firstCSWing.trueLift=ignition::math::Vector3d(1.4554,0.0,0.0);
	firstCSWing.trueDrag=ignition::math::Vector3d(0.0,0.0,1.0935);
	firstCSWing.trueLateralForce=ignition::math::Vector3d(0.0,0.0,0.0);
    firstCSWing.hasMotorExitVelocity=true;
	firstCSWing.motorExitVelocity=0;
    firstCSWing.isControlSurface=true;
	firstCSWing.isNonSpecific=false;
	firstCSWing.linkName="Port CS";
	firstCSWing.vInf=20.0242;

	LiftDragCalculateForcesParams secondCSWing;
	secondCSWing.inputWorldVel=ignition::math::Vector3d(10.0,1.0,0.1);
	secondCSWing.inputPose=ignition::math::Pose3d(0.0,0.0,0.0,-0.09,1.48,0.1);
	secondCSWing.vFwd=ignition::math::Vector3d(0.0,0.0,1.0);
	secondCSWing.vUpwd=ignition::math::Vector3d(-1.0,0.0,0.0);
	secondCSWing.area=0.0492;
	secondCSWing.rho=1.225;
	secondCSWing.motorExitVelocity=0.0;
	secondCSWing.controlAlpha=11.25;
	copy(LUT_Alpha_Control_Surface.begin(), LUT_Alpha_Control_Surface.end(), back_inserter(secondCSWing.LUT_alpha)); 
    copy(LUT_CL_Control_Surface.begin(), LUT_CL_Control_Surface.end(), back_inserter(secondCSWing.LUT_CL)); 
    copy(LUT_CD_Control_Surface.begin(), LUT_CD_Control_Surface.end(), back_inserter(secondCSWing.LUT_CD));
	secondCSWing.trueLift=ignition::math::Vector3d(1.4554,0.0,0.0);
	secondCSWing.trueDrag=ignition::math::Vector3d(0.0,0.0,1.0935);
	secondCSWing.trueLateralForce=ignition::math::Vector3d(0.0,0.0,0.0);
    secondCSWing.hasMotorExitVelocity=true;
	secondCSWing.motorExitVelocity=0;
    secondCSWing.isControlSurface=true;
	secondCSWing.isNonSpecific=false;
	secondCSWing.linkName="Starboard CS";
	secondCSWing.vInf=20.0242;

	threeElemTwoCS.elementData.push_back(mainWing);
	threeElemTwoCS.elementData.push_back(firstCSWing);
	threeElemTwoCS.elementData.push_back(secondCSWing);
	threeElemTwoCS.summedTrueForce=ignition::math::Vector3d(-98.7441,0.0,-2.2754);
	threeElementTwoCSValues.push_back(threeElemTwoCS);
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

		initLUTTables();

		setupCoordUtilsTestData();

		setupLiftDragSetAlphaTestData();

		setupLiftDragSetFreestreamVelocityTestData();

		setupLiftDragSetPlanarVelocityTestData();

		setupLiftDragSetLateralVelocityTestData();

		setupLiftDragSetAirDensityTestData();

		setupLiftDragLookupCLTestData();

		setupLiftDragLookupCDTestData();

		setupLiftDragAreaTestData();

		setupLiftDragLateralAreaTestData();

		setupLiftDragCalculateDynamicPressureData();

		setupMotorModelSetThrustData();

		setupMotorModelSetExitVelocityData();

		setupLiftDragSetBetaTestData();

		setupAtan2TestData();

		setupCalculateLocalVelocitiesData();

		setupCalculateWindAnglesData();

		setupCalculateForcesData();

		setupCalculateMultiElementForcesDataTwoCSPlusMainWing();
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

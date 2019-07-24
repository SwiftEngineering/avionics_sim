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
#include "TestRunnerEnvironment.h"
#include "TestEventListener.h"
#include "xmlparser.h"
#include "utilities.h"
#include "CombinatoricalLiftDragTest.h"
#include "CoordUtilsTest.h"

//Uncomment this to use the smaller hand calc tests.
//#define HANDTEST 1
#define NO_TESTS_EXECUTED 0

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

bool runUnitTests=false;
bool runIntegrationTests=false;
bool runSystemTests=false;

int main(int argc, char **argv)
{
	std::string envFilters="";
	const std::string integrationFilter="*Integration*";
	const std::string unitTestFilter="*Unit*";
	const std::string systemTestFilter="*System*";

	//If no flags have been specified (or there has been unexpected number of arguments), display an usage message.
	if (argc!=2)
	{
		std::cout<<"Usage: avionics_sim_test_runner {-UnitTests | -IntegrationTests | -SystemTests}"<<std::endl<<std::endl;
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

	//Init Google Test Framework.
	::testing::InitGoogleTest(&argc, argv);

	// Obtain the reference to the active listeners.
  	 ::testing::TestEventListeners& listeners =
      ::testing::UnitTest::GetInstance()->listeners();
	
	// Add custom test event listeners for each test type.
	if (runUnitTests)
	{
		listeners.Append(new TestEventListener("unit_test"));
	}

	if (runIntegrationTests)
	{
		//Create TestEventListener, populate descriptions for each test.
        TestEventListener* testEventListener=new TestEventListener("integration_test");

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

		// Add the custom test event listener.
        listeners.Append(testEventListener);
	}

	//Run all tests
	return RUN_ALL_TESTS();
}

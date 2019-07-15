/**
 * @brief       Lift Drag Plugin (Single Instance) Test Implementation
 * @file        Test_LiftDrag_Enhanced_Plugin_Single_Instance.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include "CombinatoricalLiftDragTest.h"

static const float acceptable_error=10E-3;
static const int linkIDIdxInPlantModels=0;
static const int areaIdxInPlantModels=1;
static const int controlSurfaceBoolIdx=2;
static const int controlSurfaceIDIdx=3;
static const int propWashBoolIdx=4;
static const int propWashValIdx=5;
static const std::string linkIDPrefix="link_";

// Parameterized test for lift drag plugin.   
///
/// \brief      Tests lift drag plugin input data to ensure correctness of lift drag plugin calculation.
///
/// \details    N/A
/// \param[in]  N/A
/// \return     N/A
///  
TEST_P(CombinatoricalLiftDragTest, SingleInstanceLiftDragPluginIntegrationTest) {

	LiftDragParams param=GetParam();

	double cl, cd, lift, drag, rho, speed, alpha;
	int controlSurfaceIdx, propWashIdx;
	bool isControlSurface, isPropWash;

	controlSurfaceIdx=0; 
	propWashIdx=0;

	//Extract values from object into relevant variables.
	double exitVelocityX=std::stof(param.exitVelocities.at(0));
	double exitVelocityY=std::stof(param.exitVelocities.at(1));
	double exitVelocityZ=std::stof(param.exitVelocities.at(2));
	double exitVelocityW=std::stof(param.exitVelocities.at(3));
	double exitVelocities[4]={exitVelocityX, exitVelocityY, exitVelocityW, exitVelocityZ};
	double motorExitVelocity=0.0;
	double area=std::stof(param.plantModels.at(areaIdxInPlantModels));
	double wingPoseX=std::stof(param.worldOrientations.at(0));
	double wingPoseY=std::stof(param.worldOrientations.at(1));
	double wingPoseZ=std::stof(param.worldOrientations.at(2));
	double worldVelocityX=std::stof(param.worldLinearVelocities.at(0));
	double worldVelocityY=std::stof(param.worldLinearVelocities.at(1));
	double worldVelocityZ=std::stof(param.worldLinearVelocities.at(2));

	std::string alphaLUTcontrolSurfaceDeflectionsString="-180,-175,-170,-165,-160,-155,-150,-145,-140,-135,-130,-125,-120,-115,-110,-105,-100,-95,-90,-85,-80,-75,-70,-65,-60,-55,-50,-45,-40,-35,-30,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100,105,110,115,120,125,130,135,140,145,150,155,160,165,170,175,180";
	std::string alphaLUTcontrolSurfaceDeflectionsCL="0.0000,0.6900,0.8500,0.6750,0.6600,0.7400,0.8500,0.9100,0.9450,0.9450,0.9100,0.8400,0.7350,0.6250,0.5100,0.3700,0.2200,0.0700,-0.0700,-0.2200,-0.3700,-0.5150,-0.6500,-0.7650,-0.8750,-0.9650,-1.0400,-1.0850,-1.0750,-1.0200,-0.9150,-0.9646,-0.9109,-0.8572,-0.8034,-0.7497,-0.6956,-0.6414,-0.5870,-0.5322,-0.4768,-0.4200,-0.3620,-0.3082,-0.2546,-0.2030,-0.1533,-0.1095,-0.1325,-0.8527,-0.8274,-0.7460,-0.6600,-0.5500,-0.4400,-0.3300,-0.2200,-0.1100,0.0000,0.1100,0.2200,0.3300,0.4400,0.5500,0.6600,0.7460,0.8274,0.8527,0.1325,0.1095,0.1533,0.2030,0.2546,0.3082,0.3620,0.4200,0.4768,0.5322,0.5870,0.6414,0.6956,0.7497,0.8034,0.8572,0.9109,0.9646,0.9150,1.0200,1.0750,1.0850,1.0400,0.9650,0.8750,0.7650,0.6500,0.5150,0.3700,0.2200,0.0700,-0.0700,-0.2200,-0.3700,-0.5100,-0.6250,-0.7350,-0.8400,-0.9100,-0.9450,-0.9450,-0.9100,-0.8500,-0.7400,-0.6600,-0.6750,-0.8500,-0.6900,0.0000";
	std::string alphaLUTcontrolSurfaceDeflectionsCD="0.0250,0.0550,0.1400,0.2300,0.3200,0.4200,0.5700,0.7550,0.9250,1.0850,1.2250,1.3500,1.4650,1.5550,1.6350,1.7000,1.7500,1.7800,1.8000,1.8000,1.7800,1.7350,1.6650,1.5750,1.4700,1.3450,1.2150,1.0750,0.9200,0.7450,0.5700,0.4730,0.4460,0.4200,0.3940,0.3690,0.3440,0.3200,0.2970,0.2740,0.2520,0.2310,0.2100,0.1900,0.1710,0.1520,0.1340,0.0760,0.0188,0.0203,0.0185,0.0170,0.0152,0.0140,0.0124,0.0114,0.0108,0.0104,0.0103,0.0104,0.0108,0.0114,0.0124,0.0140,0.0152,0.0170,0.0185,0.0203,0.0188,0.0760,0.1340,0.1520,0.1710,0.1900,0.2100,0.2310,0.2520,0.2740,0.2970,0.3200,0.3440,0.3690,0.3940,0.4200,0.4460,0.4730,0.5700,0.7450,0.9200,1.0750,1.2150,1.3450,1.4700,1.5750,1.6650,1.7350,1.7800,1.8000,1.8000,1.7800,1.7500,1.7000,1.6350,1.5550,1.4650,1.3500,1.2250,1.0850,0.9250,0.7550,0.5700,0.4200,0.3200,0.2300,0.1400,0.0550,0.0250";
	std::string alphaLUTString="-180,-175,-170,-165,-160,-155,-150,-145,-140,-135,-130,-125,-120,-115,-110,-105,-100,-95,-90,-85,-80,-75,-70,-65,-60,-55,-50,-45,-40,-35,-30,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-7,0,1.5,3,4,5,6,7,8,9,10,11,12,16,20,30,45,60,90,95,100,105,110,115,120,125,130,135,140,145,150,155,160,165,170,175,180";
	std::string alphaLUTCL="0.0000,0.6900,0.8500,0.6750,0.6600,0.7400,0.8500,0.9100,0.9450,0.9450,0.9100,0.8400,0.7350,0.6250,0.5100,0.3700,0.2200,0.0700,-0.0700,-0.2200,-0.3700,-0.5150,-0.6500,-0.7650,-0.8750,-0.9650,-1.0400,-1.0850,-1.0750,-1.0200,-0.9150,-0.9646,-0.9109,-0.8572,-0.8034,-0.7497,-0.6956,-0.6414,-0.5870,-0.5322,-0.4768,-0.4200,-0.3620,-0.3082,-0.2546,-0.2030,-0.1533,-0.1095,-0.1325,0.0620,0.7542,0.8987,1.0497,1.1363,1.1897,1.2348,1.2626,1.2554,1.2238,1.1889,1.1587,1.1214,0.9689,0.9104,0.9300,0.8654,0.6452,-0.1147,-0.2602,-0.2200,-0.3700,-0.5100,-0.6250,-0.7350,-0.8400,-0.9100,-0.9450,-0.9450,-0.9100,-0.8500,-0.7400,-0.6600,-0.6750,-0.8500,-0.6900,0.0000";
	std::string alphaLUTCD="0.0250,0.0550,0.1400,0.2300,0.3200,0.4200,0.5700,0.7550,0.9250,1.0850,1.2250,1.3500,1.4650,1.5550,1.6350,1.7000,1.7500,1.7800,1.8000,1.8000,1.7800,1.7350,1.6650,1.5750,1.4700,1.3450,1.2150,1.0750,0.9200,0.7450,0.5700,0.4730,0.4460,0.4200,0.3940,0.3690,0.3440,0.3200,0.2970,0.2740,0.2520,0.2310,0.2100,0.1900,0.1710,0.1520,0.1340,0.0760,0.0188,0.0456,0.0598,0.0682,0.0794,0.0877,0.1010,0.1161,0.1336,0.1540,0.1814,0.2084,0.2313,0.2578,0.3645,0.4681,0.7118,1.0732,1.3840,1.6811,1.6610,1.7500,1.7000,1.6350,1.5550,1.4650,1.3500,1.2250,1.0850,0.9250,0.7550,0.5700,0.4200,0.3200,0.2300,0.1400,0.0550,0.0250";
	std::vector<float> LUTalpha;
	std::vector<float> LUTCL;
	std::vector<float> LUTCD;
	std::vector<float> LUTcontrolSurfaceDeflectionsalpha;
	std::vector<float> LUTcontrolSurfaceDeflectionsCL;
	std::vector<float> LUTcontrolSurfaceDeflectionsCD;
	avionics_sim::Bilinear_interp::get1DLUTelementsFromString(alphaLUTString, &LUTalpha);
	avionics_sim::Bilinear_interp::get1DLUTelementsFromString(alphaLUTCL, &LUTCL);
	avionics_sim::Bilinear_interp::get1DLUTelementsFromString(alphaLUTCD, &LUTCD);
	avionics_sim::Bilinear_interp::get1DLUTelementsFromString(alphaLUTcontrolSurfaceDeflectionsString, &LUTcontrolSurfaceDeflectionsalpha);
	avionics_sim::Bilinear_interp::get1DLUTelementsFromString(alphaLUTcontrolSurfaceDeflectionsCL, &LUTcontrolSurfaceDeflectionsCL);
	avionics_sim::Bilinear_interp::get1DLUTelementsFromString(alphaLUTcontrolSurfaceDeflectionsCD, &LUTcontrolSurfaceDeflectionsCD);
	avionics_sim::Lift_drag_model LiftDragModel;
	
	ignition::math::Vector3d vel = ignition::math::Vector3d(worldVelocityX, worldVelocityY, worldVelocityZ);
	ignition::math::Pose3d pose = ignition::math::Pose3d(0.0,0.0,0.0,wingPoseX, wingPoseY, wingPoseZ);

	int controlSurfaceBool=std::stoi(param.plantModels.at(controlSurfaceBoolIdx));
	if (controlSurfaceBool==1)
	{
		isControlSurface=true;
		controlSurfaceIdx=std::stoi(param.plantModels.at(controlSurfaceIDIdx));
	}
	else
	{
		isControlSurface=false;
	}

	int propWashBool=std::stoi(param.plantModels.at(propWashBoolIdx));
	if (propWashBool==1)
	{
		isPropWash=true;
		propWashIdx=std::stoi(param.plantModels.at(propWashValIdx));
	}
	else
	{
		isPropWash=false;
	}

	LiftDragModel.setLUTs(LUTalpha, LUTCL, LUTCD);
	if(isControlSurface) // Control surface
	{
		LiftDragModel.setLUTs(LUTcontrolSurfaceDeflectionsalpha, LUTcontrolSurfaceDeflectionsCL, LUTcontrolSurfaceDeflectionsCD);
		// Get the control surface deflection angle for use as alpha
		double controlAngle; // if controlJoint does not exist, set controlAngle to zero.
		controlAngle = std::stoi(param.controlSurfaceDeflections.at(controlSurfaceIdx)); // if controlJoint does exist, set controlAngle to control joint value.
		if(isPropWash) // Propwash over control surface
		{
			// Set alpha equal to control surface angle.
			motorExitVelocity=exitVelocities[propWashIdx-1]; 
			LiftDragModel.setAlpha(controlAngle,true);
			LiftDragModel.setSpeed(motorExitVelocity); // Set velocity from motor V_e
		} 
		else
		{ 	// No propwash on control surface
			LiftDragModel.calculateAlpha(pose, vel, &alpha);
			//std::cout<<"controlAngle is "<<std::to_string(controlAngle)<<", in radians: "<<std::to_string(LiftDragModel.convertDegreesToRadians(controlAngle))<<", alpha is "<<std::to_string(alpha)<<std::endl;

			//Alpha returned is in radians, but controlAngle is not in radians. Convert to radians first before adding.
			alpha += controlAngle; // Alpha offset by control surface angle. 
			//std::cout<<"alpha is now "<<std::to_string(alpha)<<std::endl;
		}  
	} 
	else if(!isPropWash && !isControlSurface) // No propwash non-control surface
	{
		//Calculate and set alpha / set_v_infinity 
		//ignition::math::Vector3d tmp3d = ignition::math::Vector3d(0,-1,10.0); 
		ignition::math::Vector3d vInf_transformed; 
		LiftDragModel.calculateAlpha(pose, vel, &alpha, &vInf_transformed);
	} 
	else if(isPropWash && !isControlSurface) //PropWashNoControlSurface
	{ // Propwash on non-control surface
		motorExitVelocity=exitVelocities[propWashIdx-1];
		LiftDragModel.setSpeed(motorExitVelocity); // Set velocity from motor V_e
		LiftDragModel.setAlpha(0); // Assume alpha = 0 (aligned with motor)  ,true
	}

	rho = 1.225; 
	LiftDragModel.setAirDensity(rho); 
	LiftDragModel.setArea(area);
	LiftDragModel.calculateLiftDragModelValues(); // Calculate values
	speed = LiftDragModel.getSpeed();
	cl = LiftDragModel.getCL();
	cd = LiftDragModel.getCD();
	lift = LiftDragModel.getLift();
	drag = LiftDragModel.getDrag(); 
	ignition::math::Vector3d force = ignition::math::Vector3d(-lift, 0, -drag);

	std::string failStr="Values of interest:\n";
	failStr.append("Wing Pose=("+std::to_string(wingPoseX)+", "+std::to_string(wingPoseY)+", "+std::to_string(wingPoseZ)+")\n");
	failStr.append("World velocity=("+std::to_string(worldVelocityX)+", "+std::to_string(worldVelocityY)+", "+std::to_string(worldVelocityZ)+")\n");
	failStr.append("Exit velocity=(");
	for (int i=0; i<3; i++)
	{
		failStr.append(std::to_string(exitVelocities[i])+", ");
	}
	failStr.append(std::to_string(exitVelocities[3])+")\n");
	failStr.append("Control surfaces=(0,0,0,0)\n");
	failStr.append("Alpha (in degrees, as stored in LiftDragModel)=");
	failStr.append(std::to_string(LiftDragModel.getAlpha())+"\n");
	failStr.append("Alpha (local, in radians)=");
	failStr.append(std::to_string(alpha)+"\n");
	failStr.append("vInf=");
	failStr.append(std::to_string(speed)+"\n");
	failStr.append("Area=");
	failStr.append(std::to_string(LiftDragModel.getArea())+"\n");
	failStr.append("Rho=");
	failStr.append(std::to_string(LiftDragModel.getAirDensity())+"\n");
	failStr.append("CL=");
	failStr.append(std::to_string(cl)+"\n");
	failStr.append("CD=");
	failStr.append(std::to_string(cd)+"\n");
	failStr.append("lift=");
	failStr.append(std::to_string(lift)+"\n");
	failStr.append("drag=");
	failStr.append(std::to_string(drag)+"\n");
	if (controlSurfaceBool==1)
	{
		failStr.append("isControlSurface=true\n");
	}
	else
	{
		failStr.append("isControlSurface=false\n");
	}

	if (propWashBool==1)
	{
		failStr.append("isPropWash=true\n");
	}
	else
	{
		failStr.append("isPropWash=false\n");
	}
	failStr.append("Control surface ID=");
	failStr.append(std::to_string(controlSurfaceIdx)+"\n");
	failStr.append("Propwash ID=");
	failStr.append(std::to_string(propWashIdx)+"\n");
	
	float trueLift=std::stof(param.liftDragForceVectorTruths.at(0));
	float trueDrag=std::stof(param.liftDragForceVectorTruths.at(2));

	//If there is a failure in assertion, print out the data for the case.
   	ASSERT_NEAR(force[0],trueLift,acceptable_error)<< failStr;
	ASSERT_NEAR(force[2],trueDrag,acceptable_error)<< failStr;
}
/**
 * @brief       Motor_model::setThrust Test Class
 * @file        Test_Lift_Drag_Set_Get_Alpha.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "MotorModelSetExitVelocityParameterized.h"
#include "avionics_sim/Lift_drag_model.hpp"
#include "avionics_sim/Motor_model_exception.hpp"
#include "avionics_sim/Math_util.hpp"

/*Motor model as an Avionics Sim class does not yet exist. So, just testing passed in values directly. TODO: Turn this into the style of LiftDrag::SetAlpha once the Motor Model has been established in avionics_sim.*/
TEST_P(MotorModelSetExitVelocityParameterized, MotorModelSetGetExitVelocity_UnitTest) {
	//double valueToTestAgainst;
	MotorModelSetExitVelocityParams param=GetParam();
	double inputExitVelocity=param.exitVelocity;
	double minExitVelocity=param.minExitVelocity;
	double maxExitVelocity=param.maxExitVelocity;
	
	//Math utility object (used for floating point comparison, display of digits to a certain precision)
    avionics_sim::Math_util mu;

	//Set vInf to minimum for consideration of alpha.
	try
	{
		if (isnan(inputExitVelocity))
        {
            std::string errMsg="Motor_model::setExitVelocity: input thrust is NaN";
            Motor_model_exception e(errMsg);
			throw e;
        }
        else
        {
			double tolerance=0.00001;

            //Check if input thrust is within bounds. Throw an exception if it is not.
            if ( mu.rough_lt(inputExitVelocity, minExitVelocity, tolerance) || mu.rough_gt(inputExitVelocity, maxExitVelocity, tolerance) )
            {
                std::string errMsg="Exit velocity is not within bounds of ["+std::to_string(minExitVelocity)+", "+std::to_string(maxExitVelocity)+"]. rho= "+std::to_string(inputExitVelocity);
				Motor_model_exception e(errMsg);
                throw e;
            }
        } 
	}
	catch(const Motor_model_exception& e)
	{
		std::cerr<<"Exception successfully caught for setExitVelocity."<<std::endl;
		std::cerr << e.what() <<std::endl;
	}
}
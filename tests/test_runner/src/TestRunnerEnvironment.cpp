/**
 * @brief       Integration Test Environment class
 * @file        TestRunnerEnvironment.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "TestRunnerEnvironment.h"
#include <iostream>
#include <fstream>

TestRunnerEnvironment::~TestRunnerEnvironment()
{
	//All destructor work goes here.
}

void TestRunnerEnvironment::SetUp()
{
	//Any pre-test tasks can be placed here.
}

void TestRunnerEnvironment::TearDown()
{
	//Any tear down tasks can be placed here.
}

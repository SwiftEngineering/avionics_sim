/**
 * @brief       Integration Test Environment class
 * @file        IntegrationTestEnvironment.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "../include/IntegrationTestEnvironment.h"
#include <iostream>
#include <fstream>

IntegrationTestEnvironment::~IntegrationTestEnvironment()
{
	//All destructor work goes here.
}

void IntegrationTestEnvironment::SetUp()
{
	//Any pre-test tasks can be placed here.
}

void IntegrationTestEnvironment::TearDown()
{
	//Generate test summary.
	/*NB: More documentation can be found here:
	http://docs.ros.org/melodic/api/self_test/html/classtesting_1_1TestCase.html#ab61929942a202f03903182866bd0e086*/
	::testing::UnitTest* unit_test = ::testing::UnitTest::GetInstance();
	generateReportOverview(unit_test->total_test_count(), unit_test->successful_test_count(), unit_test->failed_test_count());
}

void IntegrationTestEnvironment::generateReportOverview(int totalTests, int testsPassed, int testsFailed)
{
	std::ofstream summaryFile;
	summaryFile.open ("TestSummary.html");
	summaryFile << "<html><head><title> Test Execution Summary Report </title>"<<std::endl;
	summaryFile << "<script type='text/javascript' src='http://www.google.com/jsapi'></script>"<<std::endl;
	summaryFile << "<script type='text/javascript'>"<<std::endl;
	summaryFile << "google.load('visualization', '1', {packages:['gauge']});"<<std::endl;
	summaryFile << "google.setOnLoadCallback(drawChart);"<<std::endl;
	summaryFile << "function drawChart() { var passedData = new google.visualization.DataTable();"<<std::endl;
	summaryFile << "passedData.addColumn('string', 'Label'); passedData.addColumn('number', 'Value');"<<std::endl;
	summaryFile << "passedData.addRows(1); passedData.setValue(0, 0, 'Tests Passed');"<<std::endl;
	summaryFile << "passedData.setValue(0, 1, " << testsPassed << ");"<<std::endl;
	summaryFile << "var passedChart = new google.visualization.Gauge(document.getElementById('chart_passed'));"<<std::endl;
	summaryFile << "var passedOptions = {width: 400, height: 120, redFrom: 0, redTo: 30, yellowFrom: 31, yellowTo: 60, greenFrom: 61, greenTo: 100, minorTicks: 5};"<<std::endl;
	summaryFile << "passedChart.draw(passedData, passedOptions);"<<std::endl;
	summaryFile << "var failedData = new google.visualization.DataTable();"<<std::endl;
	summaryFile << "failedData.addColumn('string', 'Label');"<<std::endl;
	summaryFile << "failedData.addColumn('number', 'Value');"<<std::endl;
	summaryFile << "failedData.addRows(1);"<<std::endl;
	summaryFile << "failedData.setValue(0, 0, 'Tests Failed');"<<std::endl;
	summaryFile << "failedData.setValue(0, 1, " << testsFailed << ");"<<std::endl;
	summaryFile << "var failedChart = new google.visualization.Gauge(document.getElementById('chart_failed'));"<<std::endl;
	summaryFile << "var failedOptions = {width: 400, height: 120, redFrom: 31, redTo: 100, yellowFrom: 11, yellowTo: 30, greenFrom: 0, greenTo: 10, minorTicks: 5};"<<std::endl;
	summaryFile << "failedChart.draw(failedData, failedOptions);}"<<std::endl;
	summaryFile << "</script></head>"<<std::endl;
	summaryFile << "<body style='font-weight: bold; font-size: 10pt; color: black; font-family: Tahoma'><br />Test Execution Summary"<<std::endl;

	/*
	Test Suite Description HTML. Integrate next round.
	<!--Test Suite Description-->
Test Suite Description
<br/>Lift drag plugin integration test, main wing. 320,000 tests each for propwash and non-propwash cases, leading to a total of 640,000.<br/><br/>Parameter detail:<br/> {-2pi, -pi, 0, pi, 2pi} as possible values for pitch, roll, yaw (in radians). <br/> {0,20} as possible values for the three velocity components. <br/> {0,5} as possible values for each of the four exit velocity components.  <br/> {-45, -15, 0, 15, 45} as possible values for each of the four control surface deflection surface angles (in degrees).
<!--End Test Suite Description-->
	 */
	summaryFile << "<br /><br />Test Suite Summary<table><tr><td style='width: 200px; height: 56px; background-color: #669918'>Total Tests Executed</td><td style='height: 56px; background-color: #669918'>Passed</td><td style='height: 56px; background-color: #669918'>Failed</td></tr>"<<std::endl;
	summaryFile << "<tr><td style='width: 200px; background-color: #D0F0A2'>"<<totalTests<<"</td><td style='background-color: #D0F0A2'>"<<testsPassed<<"</td><td style='background-color: #D0F0A2'>"<<testsFailed<<"</td></tr>"<<std::endl;
	summaryFile << "<tr><td><div id='chart_passed'></div></td><td><div id='chart_failed'></div></td></tr></table></body></html>"<<std::endl;
	summaryFile.close();
}

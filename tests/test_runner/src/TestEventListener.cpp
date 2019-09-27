#include "TestEventListener.h"
#include <boost/algorithm/string.hpp>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>

TestEventListener::TestEventListener(std::string testType)
{
    resetStats();
    logFileName="./test_"+testType+"_"+GetCurrentDateAndTime()+".html";

    /* 
    Open file. Done here in constructor so that the file handle is kept open for writing during the course of tests.
    Opening and closing in OnTestStart and OnTestEnd would result in filesystem thrashing, especially in large number of
    tests.*/
    testReporter.open(logFileName);

    //Write header HTML header information.
    std::string testDateAndTime=GetCurrentDateAndTime(false);
	testReporter.writeHTML("<html><head><title> Test Execution Summary Report, "+testDateAndTime+"</title>");
    testReporter.writeHTML("</head>");
	testReporter.writeHTML("<body style='font-weight: bold; font-size: 10pt; color: black; font-family: Tahoma'> \
    <br />Test Execution Summary Report, "+testDateAndTime+"<br/><br/>");
}

TestEventListener::~TestEventListener()
{
    //Output the last test case.
    generateTestCaseOverview();

    //write closing tags
    testReporter.writeHTML("</body></html>");

    //Close logger
    testReporter.close();

    //Clear the hash table
    testClassInfo.clear();

    //Clear the error list.
    errorList.clear();

    //Tell user where the output file is.
    std::cout<<std::endl<<"Results of this test run are written here: "+logFileName<<std::endl;
}

void TestEventListener::OnTestStart(const ::testing::TestInfo& test_info)
{
    std::string curTestCaseName=test_info.test_case_name();

    //If we have a new test class, handle class description output.
    if (currentClassName.compare(curTestCaseName)!=0)
    {
        generateTestCaseOverview();

        currentClassName=curTestCaseName;
    }
}

// Called when an assert fails or SUCCESS gets called.
void TestEventListener::OnTestPartResult(const ::testing::TestPartResult& test_part_result)
{
    /*If there is a failure, save it in the error list. Push at end so can maintain order.
    Summary=msg in (Assert(cond)<<msg) in case of failure.*/
    if (test_part_result.failed())
    {
        //errDatum err{test_part_result.summary()};
        std::string errData=test_part_result.summary();

        //Replace the newlines with HTML breaks
        boost::replace_all(errData, "\n", "<br/>");

        errDatum *err = new errDatum{errData};
        errorList.push_back(*err);
        testsFailed++;
    }
}

// Called after a test finishes.
void TestEventListener::OnTestEnd(const ::testing::TestInfo& test_info)
{
    totalTests++;
    //std::cout<<"totalTests: "<<std::to_string(totalTests)<<", passed: "<<std::to_string(testsPassed)<<", failed: "<<std::to_string(testsFailed)<<std::endl;
}

void TestEventListener::addTestClassNameAndDescription(std::string testClassName, std::string testClassDescription)
{
    std::string descriptionInHTML=testClassDescription;
    boost::replace_all(descriptionInHTML, "\n", "<br/>");
    std::pair<std::string,std::string> newDescriptionPair (testClassName, descriptionInHTML);
    testClassInfo.insert(newDescriptionPair);
    if (currentClassName.compare("")==0)
    {
        currentClassName=testClassName;
    }
}

std::string TestEventListener::retrieveClassDescription(std::string testClassName)
{
    std::unordered_map<std::string,std::string>::const_iterator got = testClassInfo.find (testClassName);
    if (got != testClassInfo.end()) {
        return got->second;
    } else {
        return "N/A";
    }
}

void TestEventListener::generateTestCaseOverview()
{
    std::string testDescription=retrieveClassDescription(currentClassName);
    std::string totalTestsStr=std::to_string(totalTests);
    std::string testsFailedStr=std::to_string(testsFailed);

    //For now, if we have any disabled tests (totalTests=0), don't generate anything.
    if (totalTests==0)
    {
        return;
    }

    /*Tests that are passing are not being incremented in OnTestPartResult. 
    Instead, deduct by subtracting failed from total to get the number of passed tests.
    */
    testsPassed=totalTests-testsFailed;
    std::string testsPassedStr=std::to_string(testsPassed);

    std::string currentClassNameWithoutSlash=currentClassName;
    boost::replace_all(currentClassNameWithoutSlash, "/", "_");

    std::string testsPassedDiv=currentClassNameWithoutSlash+"_chart_passed";
    std::string testsFailedDiv=currentClassNameWithoutSlash+"_chart_failed";
    std::string drawChartInstanceName="drawChart_"+currentClassNameWithoutSlash;

    //Generate javascript for chart. Cannot parameterize this, as there must be a callback for each fn in order to execute.
    testReporter.writeHTML("<script type='text/javascript' src='https://www.gstatic.com/charts/loader.js'></script>");
	testReporter.writeHTML("<script type='text/javascript'>");
    testReporter.writeHTML("google.charts.load('visualization', '1', {packages:['gauge']});");
    testReporter.writeHTML("google.charts.setOnLoadCallback("+drawChartInstanceName+");");
    testReporter.writeHTML("function "+drawChartInstanceName+"() { var passedData = new google.visualization.DataTable();");
    testReporter.writeHTML("var chartWidth = 500;");
    testReporter.writeHTML("var chartHeight = 220;");
    testReporter.writeHTML("var testsPassed ="+testsPassedStr+";");
    testReporter.writeHTML("var testsFailed = "+testsFailedStr+";");
    testReporter.writeHTML("var totalTests = "+totalTestsStr+";");
    testReporter.writeHTML("var unit = totalTests/3;");
    testReporter.writeHTML("var unhealthyThresholdMin =0;");
    testReporter.writeHTML("var unhealthyThresholdMax = unit;");
    testReporter.writeHTML("var unstableThresholdMin = unhealthyThresholdMax;");
    testReporter.writeHTML("var unstableThresholdMax = unstableThresholdMin+unit;");
    testReporter.writeHTML("var stableThresholdMin= unstableThresholdMax;");
    testReporter.writeHTML("var ticks = 5;");
	testReporter.writeHTML("passedData.addColumn('string', 'Label'); passedData.addColumn('number', 'Value');");
	testReporter.writeHTML("passedData.addRows(1); passedData.setValue(0, 0, 'Tests Passed');");
	testReporter.writeHTML("passedData.setValue(0, 1, testsPassed);");
	testReporter.writeHTML("var passedChart = new google.visualization.Gauge(document.getElementById('"+testsPassedDiv+"'));");

    //NB: numbers for red, green, yellow are percentages of total, not actual numbers if no min/max set.
	testReporter.writeHTML("var passedOptions = {width: chartWidth, height: chartHeight, redFrom: unhealthyThresholdMin, redTo: unhealthyThresholdMax, yellowFrom: unstableThresholdMin, yellowTo: unstableThresholdMax, greenFrom: stableThresholdMin, greenTo: totalTests, minorTicks: 5, max: totalTests, min: 0};");
	testReporter.writeHTML("passedChart.draw(passedData, passedOptions);");
	testReporter.writeHTML("var failedData = new google.visualization.DataTable();");
	testReporter.writeHTML("failedData.addColumn('string', 'Label');");
	testReporter.writeHTML("failedData.addColumn('number', 'Value');");
	testReporter.writeHTML("failedData.addRows(1);");
	testReporter.writeHTML("failedData.setValue(0, 0, 'Tests Failed');");
	testReporter.writeHTML("failedData.setValue(0, 1, testsFailed );");
	testReporter.writeHTML("var failedChart = new google.visualization.Gauge(document.getElementById('"+testsFailedDiv+"'));");
	testReporter.writeHTML("var failedOptions = {width: chartWidth, height: chartHeight, redFrom: stableThresholdMin, redTo: totalTests, yellowFrom: unstableThresholdMin, yellowTo: unstableThresholdMax, greenFrom: 0, greenTo: unhealthyThresholdMax, minorTicks: 5, max: totalTests, min: 0};");
	testReporter.writeHTML("failedChart.draw(failedData, failedOptions);}");
    testReporter.writeHTML("</script>");

	testReporter.writeHTML("<br /><br /><table>");
    testReporter.writeHTML("<tr><td style='width: 200px; height: 56px; background-color: #669918'> \
    Test Suite Name</td></tr>");
    testReporter.writeHTML("<tr><td style='width: 200px; height: 56px'>"
    +currentClassName+"</td></tr>");
    testReporter.writeHTML("<tr><td style='width: 200px; height: 56px; background-color: #669918'> \
    Test Suite Description</td></tr>");
    testReporter.writeHTML("<tr><td style='width: 200px; height: 56px'>"
    +testDescription+"</td></tr>");
    testReporter.writeHTML("<tr><td style='width: 200px; height: 56px; background-color: #669918'> \
    Total Tests Executed</td><td style='height: 56px; background-color: #669918'>Passed</td> \
    <td style='height: 56px; background-color: #669918'>Failed</td></tr>");
	testReporter.writeHTML("<tr><td style='width: 200px; background-color: #D0F0A2'>"+totalTestsStr
    +"</td><td style='background-color: #D0F0A2'>"+testsPassedStr
    +"</td><td style='background-color: #D0F0A2'>"+testsFailedStr+"</td></tr>");
	testReporter.writeHTML("<tr><td><div id='"+testsPassedDiv+"'></div></td><td><div id='"+
    testsFailedDiv+"'></div></td></tr>");

    //Output an error data if there were failures.
    if (!errorList.empty())
    {
        testReporter.writeHTML("<tr><td style='width: 200px; height: 56px; background-color: #669918'>Error Data</td></tr>");

        testReporter.writeHTML("<tr><td style='width: 200px;'>");

        for (const errDatum &errMsg : errorList)
            testReporter.write(errMsg.errInfo+"<br/><br/>");

        testReporter.writeHTML("</td></tr>");
    }
    
    testReporter.writeHTML("</table>");

    errorList.clear();
        
    resetStats();
}

std::string TestEventListener::GetCurrentDateAndTime(bool isForFilename)
{
    auto time = std::time(nullptr);
    std::stringstream ss;
    if (isForFilename)
    {
        ss << std::put_time(std::localtime(&time), "%F_%T"); // ISO 8601 without timezone information.
    }
    else
    {
        ss << std::put_time(std::localtime(&time), "%d-%m-%Y %H-%M-%S") << std::endl;
    }
    auto s = ss.str();
    if (isForFilename)
    {
        std::replace(s.begin(), s.end(), ':', '-');
    }
    return s;
}

void TestEventListener::resetStats()
{
    totalTests=0;
    testsPassed=0;
    testsFailed=0;
    currentClassName="";
}

#include "TestEventListener.h"

TestEventListener::TestEventListener(std::string _logFileName)
{
    logFileName=_logFileName;

    /* 
    Open file. Done here in constructor so that the file handle is kept open for writing during the course of tests.
    Opening and closing in OnTestStart and OnTestEnd would result in thrashing, especially in large number of
    tests.*/
    errorReporter.open(logFileName);
}

TestEventListener::~TestEventListener()
{
    //Close logger
    errorReporter.close();
}

void TestEventListener::OnTestStart(const ::testing::TestInfo& test_info)
{
    //If there are any other tasks to perform at test start, do them here.
}

// Called when an assert fails or SUCCESS gets called.
void TestEventListener::OnTestPartResult(const ::testing::TestPartResult& test_part_result)
{
    //If there is a failure, write it to the log file. 
    //Summary=msg in (Assert(cond)<<msg) in case of failure.
    if (test_part_result.failed())
    {
        errorReporter.writeLog(test_part_result.summary());
        
    }
}

// Called after a test finishes.
void TestEventListener::OnTestEnd(const ::testing::TestInfo& test_info)
{
    
}
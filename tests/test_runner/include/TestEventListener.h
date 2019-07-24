#ifndef TEST_EVENT_LISTENER_H_
#define TEST_EVENT_LISTENER_H_

#include "gtest/gtest.h"
#include "Logger.h"

class TestEventListener : public ::testing::EmptyTestEventListener {

  public:

    TestEventListener(std::string _logFileName="./test_result.log");

    ~TestEventListener();

    // Called before a test starts.
    virtual void OnTestStart(const ::testing::TestInfo& test_info);

    // Called when an assert fails or SUCCESS gets called.
    virtual void OnTestPartResult(const ::testing::TestPartResult& test_part_result);

    // Called after a test finishes.
    virtual void OnTestEnd(const ::testing::TestInfo& test_info);
private:
    Logger errorReporter;

    std::string logFileName;
};
#endif
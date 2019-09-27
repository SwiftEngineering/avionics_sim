#ifndef TEST_EVENT_LISTENER_H_
#define TEST_EVENT_LISTENER_H_

#include "gtest/gtest.h"
#include "Logger.h"
#include "TestRunnerEnvironment.h"
#include <unordered_map>
#include <boost/intrusive/list.hpp>

class TestEventListener : public ::testing::EmptyTestEventListener {

  public:

    // Constructor
    ///
    /// \brief      N/A 
    ///
    /// \details    N/A
    /// \param[in]  testType     Test type (Integration, Unit, etc.)
    /// \return     Instance of TestEventListener
    ///
    TestEventListener(std::string tstType);

    // Destructor.
    ///
    /// \brief      N/A 
    ///
    /// \details    N/A
    /// \param[in]  N/A
    /// \return     N/A
    ///
    ~TestEventListener();

    // Fired before the test starts.
    ///
    /// \brief      N/A 
    ///
    /// \details    N/A
    /// \param[in]  test_info     Object of type TestInfo
    /// \return     N/A
    ///
    virtual void OnTestStart(const ::testing::TestInfo& test_info);

    // Fired after a failed assertion or a SUCCEED() invocation.
    ///
    /// \brief      N/A 
    ///
    /// \details    N/A
    /// \param[in]  test_part_result     Object of type TestPartResult (contains test result)
    /// \return     N/A
    ///
    virtual void OnTestPartResult(const ::testing::TestPartResult& test_part_result);

    // Fired after the test ends.
    ///
    /// \brief      N/A 
    ///
    /// \details    N/A
    /// \param[in]  test_info     Object of type TestInfo
    /// \return     N/A
    ///
    virtual void OnTestEnd(const ::testing::TestInfo& test_info);

    // Function to add test description and test class name. Used in report output.  
    ///
    /// \brief      N/A 
    ///
    /// \details    N/A
    /// \param[in]  testClassName     Test Class Name
    /// \param[in]  testClassDescription     Test class description
    /// \return     N/A
    ///
    void addTestClassNameAndDescription(std::string testClassName, std::string testClassDescription);

    // Function to initialize log file after filename has been set. 
    ///
    /// \brief      N/A 
    ///
    /// \details    N/A
    /// \param[in]  testClassName     Test Class Name
    /// \param[in]  testClassDescription     Test class description
    /// \return     N/A
    ///
    void initLogFile(std::string filename="");

    
private:
    Logger testReporter;

    std::string logFileName;

    std::string currentClassName;

    std::string testType;

    struct errDatum : public boost::intrusive::list_base_hook<>
    {
      std::string errInfo;
      errDatum(std::string errStr) : errInfo{std::move(errStr)} {}
    };

    /* 
    Doubly linked list to hold error data. 
    Why a double linked list? To assure O(1) speed in insertion regardless of structure size (insertion at tail ptr).
    Head ptr will be used to retrieve all data in O(N) time. 
    */
    boost::intrusive::list<errDatum> errorList;

    // Function to retrieve class description from class name.
    ///
    /// \brief      N/A 
    ///
    /// \details    N/A
    /// \param[in]  testClassName     Test Class Name
    /// \return     String containing class description.
    ///
    std::string retrieveClassDescription(std::string testClassName);

    //Hash table storing class descriptions and their name as key.
    std::unordered_map<std::string,std::string> testClassInfo;

    // Function to generate HTML report for each test case (suite)   
    ///
    /// \brief      N/A 
    ///
    /// \details    adapted from https://unmesh.me/2012/05/15/using-google-charts-api-for-test-results-dashboard/
    /// \param[in]  N/A
    /// \return     N/A
    ///
    void generateTestCaseOverview();

    // Function to generate current date and time as String.   
    ///
    /// \brief      N/A 
    ///
    /// \details    N/A
    /// \param[in]  isForFilename     Boolean indicating whether the string is intended for use in a filename.
    /// \return     N/A
    ///
    std::string GetCurrentDateAndTime(bool isForFilename=true);

    //Stat counters per test
    int totalTests;
    int testsPassed;
    int testsFailed;

    // Function to reset stat counters.
    ///
    /// \brief      N/A 
    ///
    /// \details    N/A
    /// \param[in]  N/A
    /// \return     N/A
    ///
    void resetStats();
};

#endif

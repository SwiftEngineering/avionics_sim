#ifndef TEST_RUNNER_ENVIRONMENT_H_
#define TEST_RUNNER_ENVIRONMENT_H_

/**
 * @brief       Integration Test Environment class
 * @file        TestRunnerEnvironment.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "gtest/gtest.h"

class TestRunnerEnvironment : public ::testing::Environment {
 public:

  virtual ~TestRunnerEnvironment();

  // Function to prepare the objects for the test environment.   
  ///
  /// \brief      N/A 
  ///
  /// \details    N/A
  /// \param[in]  N/A
  /// \return     N/A
  ///
  virtual void SetUp();

  // Function to release any resources allocated, as well as perform other cleanup tasks (report generation, etc.)   
  ///
  /// \brief      N/A 
  ///
  /// \details    N/A
  /// \param[in]  N/A
  /// \return     N/A
  ///
  virtual void TearDown();
  
 private:

  // Function to generate HTML report upon completion of all tests.   
  ///
  /// \brief      N/A 
  ///
  /// \details    adapted from https://unmesh.me/2012/05/15/using-google-charts-api-for-test-results-dashboard/
  /// \param[in]  totalTests (number of total tests)
  /// \param[in]  testsPassed (number of tests passed)
  /// \param[in]  testsFailed (number of tests failed)
  /// \return     N/A
  ///
  void generateReportOverview(int totalTests, int testsPassed, int testsFailed);
};
#endif
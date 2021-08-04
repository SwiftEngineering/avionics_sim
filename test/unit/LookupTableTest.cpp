/**
 * @copyright   Copyright (c) 2021, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include <gtest/gtest.h>

#include <string>
#include <sstream>
#include <boost/array.hpp>
#include <stdio.h>

#include "LookupTable.hpp"


class LookupTableTest : public ::testing::Test {
  protected:
    std::vector<std::string> colNames = {"Angle Of Attack", "Lift Coefficient"};
    std::vector<std::vector<double>> cols = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    avionics_sim::LookupTable lookupTable_ =
        avionics_sim::LookupTable(colNames, cols);

    // You can define per-test set-up logic as usual.
    virtual void SetUp() {
    }
};

TEST_F(LookupTableTest, TestConstruction) {
    // Given a value within the bounds of the first data vector
    double refValue = 1.5;

    // When value is lookup in table
    double result = lookupTable_.lookup(1.5, "Angle Of Attack", "Lift Coefficient");

    // Then
    ASSERT_EQ(result, 4.5);
}

#pragma once

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <random>

#define EXPECT_V3_NEAR(a, b, abs_error) {\
  EXPECT_NEAR(a[0], b[0], abs_error);\
  EXPECT_NEAR(a[1], b[1], abs_error);\
  EXPECT_NEAR(a[2], b[2], abs_error);\
  }

namespace {

  double epsilon = 1E-5;

}


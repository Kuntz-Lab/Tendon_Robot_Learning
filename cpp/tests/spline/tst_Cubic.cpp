/**
 * author:        Michael Bentley
 * email:         mikebentley15@gmail.com
 * date-created:  25 February 2020
 */

#include "spline/Cubic.h"

#include <gtest/gtest.h>

using spline::Cubic;
using spline::operator<<;

TEST(CubicTests, empty_constructor) {
  Cubic<double> expected{0.0, 0.0, 0.0, 0.0};
  Cubic<double> actual;
  ASSERT_EQ(actual, expected);
}

TEST(CubicTests, constant_function) {
  Cubic<double> c{1.5};
  ASSERT_EQ(c(0.0), 1.5);
  ASSERT_EQ(c(1.0), 1.5);
  ASSERT_EQ(c(2.0), 1.5);
  ASSERT_EQ(c(15.0), 1.5);
  ASSERT_EQ(c(6.324e23), 1.5);
}

TEST(CubicTests, linear_function) {
  Cubic<int> c{2, 3};
  ASSERT_EQ(c( 0),  2);
  ASSERT_EQ(c( 1),  5);
  ASSERT_EQ(c( 2),  8);
  ASSERT_EQ(c(-1), -1);
}

TEST(CubicTests, quadratic_function) {
  Cubic<int> c{2, -1, 2};
  ASSERT_EQ(c(-1), 5);
  ASSERT_EQ(c( 0), 2);
  ASSERT_EQ(c( 1), 3);
  ASSERT_EQ(c( 2), 8);
}

TEST(CubicTests, cubic_function) {
  Cubic<int> c{4, 3, 2, 1};
  ASSERT_EQ(c(-1),  2);
  ASSERT_EQ(c( 0),  4);
  ASSERT_EQ(c( 1), 10);
  ASSERT_EQ(c( 2), 26);
}

TEST(CubicTests, zero_deriv) {
  Cubic<int> c;
  ASSERT_EQ(c.deriv(), c);
}

TEST(CubicTests, constant_deriv) {
  Cubic<int> c{2};
  Cubic<int> zeros{0};
  ASSERT_EQ(c.deriv(), zeros);
}

TEST(CubicTests, linear_deriv) {
  Cubic<int> c{2, 1};
  Cubic<int> expected{1};
  ASSERT_EQ(c.deriv(), expected);
}

TEST(CubicTests, quadratic_deriv) {
  Cubic<int> c{3, 2, 1};
  Cubic<int> expected{2, 2};
  ASSERT_EQ(c.deriv(), expected);
}

TEST(CubicTests, cubic_deriv) {
  Cubic<int> c    {4, 3, 2, 1};
  Cubic<int> cp   {3, 4, 3};
  Cubic<int> cpp  {4, 6};
  Cubic<int> cppp {6};
  Cubic<int> cpppp{0};
  ASSERT_EQ(c.deriv(), cp);
  ASSERT_EQ(c.deriv().deriv(), cpp);
  ASSERT_EQ(c.deriv().deriv().deriv(), cppp);
  ASSERT_EQ(c.deriv().deriv().deriv().deriv(), cpppp);
}


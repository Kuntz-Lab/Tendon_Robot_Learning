/**
 * author:        Michael Bentley
 * email:         mikebentley15@gmail.com
 * date-created:  25 February 2020
 */

#include "spline/CubicSpline.h"

#include <gtest/gtest.h>

using spline::Cubic;
using spline::CubicSpline;
using spline::operator<<;

TEST(CubicSplineTests, test_ints) {
                  // x, y, yp
  CubicSpline spline(0, 2, 1,
                     1, 3, 1); // a straight line
  ASSERT_EQ(spline(-1), 1);
  ASSERT_EQ(spline( 0), 2);
  ASSERT_EQ(spline( 1), 3);
  ASSERT_EQ(spline( 2), 4);
  ASSERT_EQ(spline( 3), 5);
}

TEST(CubicSplineTests, test_x_double_y_ints) {
  CubicSpline<double, int> spline(0.0, 2, 2,
                                  1.5, 5, 2); // a straight line
  ASSERT_EQ(spline(-1.00),  0.0);
  ASSERT_EQ(spline(-0.50),  1.0);
  ASSERT_EQ(spline( 0.00),  2.0);
  ASSERT_EQ(spline( 1.50),  5.0);
  ASSERT_EQ(spline( 3.50),  9.0);
  ASSERT_EQ(spline( 4.25), 10.5);
}

TEST(CubicSplineTests, reconstruct_correct_spline) {
  Cubic<double> c {1.2, 2.3, 3.4, 4.5};
  auto cp = c.deriv();

  // fit the spline to my exact cubic function
  CubicSpline<double, double> spline( 1.3, c( 1.3), cp( 1.3),
                                     -2.1, c(-2.1), cp(-2.1));

  double tol = 1e-10; // tolerance
  ASSERT_NEAR(spline( 3.2), c( 3.2), tol);
  ASSERT_NEAR(spline(-9.1), c(-9.1), tol);
  ASSERT_NEAR(spline(-1.2), c(-1.2), tol);
  ASSERT_NEAR(spline( 0.0), c( 0.0), tol);
  ASSERT_NEAR(spline( 1.1), c( 1.1), tol);
}

TEST(CubicSplineTests, reconstruct_correct_spline_deriv) {
  Cubic<double> c {1.2, 2.3, 3.4, 4.5};
  auto cp = c.deriv();

  // fit the spline to my exact cubic function
  CubicSpline<double, double> spline( 1.3, c( 1.3), cp( 1.3),
                                     -2.1, c(-2.1), cp(-2.1));

  auto spline_p = spline.deriv();

  double tol = 1e-10; // tolerance
  ASSERT_NEAR(spline_p( 3.2), cp( 3.2), tol);
  ASSERT_NEAR(spline_p(-9.1), cp(-9.1), tol);
  ASSERT_NEAR(spline_p(-1.2), cp(-1.2), tol);
  ASSERT_NEAR(spline_p( 0.0), cp( 0.0), tol);
  ASSERT_NEAR(spline_p( 1.1), cp( 1.1), tol);
}

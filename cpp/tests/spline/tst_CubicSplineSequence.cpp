/**
 * author:        Michael Bentley
 * email:         mikebentley15@gmail.com
 * date-created:  25 February 2020
 */

#include "spline/CubicSplineSequence.h"

#include <gtest/gtest.h>

#include <algorithm>

using spline::CubicSpline;
using spline::CubicSplineSequence;
using spline::operator<<;

TEST(CubicSplineSequenceTests, test_empty_throws) {
  std::vector<double> empty;
  ASSERT_THROW(CubicSplineSequence(empty, empty, empty), std::out_of_range);
}

TEST(CubicSplineSequenceTests, test_different_sizes_throws) {
  std::vector<double> a {1.1, 2.2, 3.3, 4.4};
  std::vector<double> b {1.1, 2.2, 3.3, 4.4, 5.5};
  std::vector<double> c {1.1, 2.2, 3.3};
  ASSERT_THROW(CubicSplineSequence(a, a, b), std::out_of_range);
  ASSERT_THROW(CubicSplineSequence(b, a, b), std::out_of_range);
  ASSERT_THROW(CubicSplineSequence(b, a, a), std::out_of_range);
  ASSERT_THROW(CubicSplineSequence(c, c, a), std::out_of_range);
}

TEST(CubicSplineSequenceTests, test_one_point_throws) {
  std::vector<double> a {1.1};
  ASSERT_THROW(CubicSplineSequence(a, a, a), std::out_of_range);
}

TEST(CubicSplineSequenceTests, test_x0_and_xn) {
  std::vector<double> x  {1.1, 3.3,  5.5,  7.7};
  std::vector<double> y  {3.2, 8.5, -2.1,  2.4};
  std::vector<double> yp {1.2, 5.3, -9.1, -7.0};
  CubicSplineSequence seq(x, y, yp);
  ASSERT_DOUBLE_EQ(seq.x0(), x.front());
  ASSERT_DOUBLE_EQ(seq.xn(), x.back());
}

TEST(CubicSplineSequenceTests, test_two_points_equal_to_single_spline) {
  std::vector<double> a {1.1, 2.2};
  CubicSplineSequence seq(a, a, a);
  CubicSpline spline(a[0], a[0], a[0], a[1], a[1], a[1]);

  ASSERT_DOUBLE_EQ(seq(1.1), spline(1.1));
  ASSERT_DOUBLE_EQ(seq(1.2), spline(1.2));
  ASSERT_DOUBLE_EQ(seq(1.3), spline(1.3));
  ASSERT_DOUBLE_EQ(seq(1.4), spline(1.4));
  ASSERT_DOUBLE_EQ(seq(1.5), spline(1.5));
  ASSERT_DOUBLE_EQ(seq(1.6), spline(1.6));
  ASSERT_DOUBLE_EQ(seq(1.7), spline(1.7));
  ASSERT_DOUBLE_EQ(seq(1.8), spline(1.8));
  ASSERT_DOUBLE_EQ(seq(1.9), spline(1.9));
  ASSERT_DOUBLE_EQ(seq(2.0), spline(2.0));
  ASSERT_DOUBLE_EQ(seq(2.1), spline(2.1));
  ASSERT_DOUBLE_EQ(seq(2.2), spline(2.2));
}

TEST(CubicSplineSequenceTests, test_two_splines) {
  std::vector<double> x  {1.1, 3.3,  5.5,  7.7};
  std::vector<double> y  {3.2, 8.5, -2.1,  2.4};
  std::vector<double> yp {1.2, 5.3, -9.1, -7.0};

  CubicSpline s1(x[0], y[0], yp[0],
                 x[1], y[1], yp[1]);
  CubicSpline s2(x[1], y[1], yp[1],
                 x[2], y[2], yp[2]);
  CubicSpline s3(x[2], y[2], yp[2],
                 x[3], y[3], yp[3]);

  CubicSplineSequence seq(x, y, yp);

  const double threshold = 1e-11;
  ASSERT_NEAR(seq(0.3), s1(0.3), threshold);
  ASSERT_NEAR(seq(0.4), s1(0.4), threshold);
  ASSERT_NEAR(seq(0.5), s1(0.5), threshold);
  ASSERT_NEAR(seq(0.6), s1(0.6), threshold);
  ASSERT_NEAR(seq(0.7), s1(0.7), threshold);
  ASSERT_NEAR(seq(0.8), s1(0.8), threshold);
  ASSERT_NEAR(seq(0.9), s1(0.9), threshold);
  ASSERT_NEAR(seq(1.0), s1(1.0), threshold);
  ASSERT_NEAR(seq(1.1), s1(1.1), threshold);
  ASSERT_NEAR(seq(1.2), s1(1.2), threshold);
  ASSERT_NEAR(seq(1.3), s1(1.3), threshold);
  ASSERT_NEAR(seq(1.4), s1(1.4), threshold);
  ASSERT_NEAR(seq(1.5), s1(1.5), threshold);
  ASSERT_NEAR(seq(1.6), s1(1.6), threshold);
  ASSERT_NEAR(seq(1.7), s1(1.7), threshold);
  ASSERT_NEAR(seq(1.8), s1(1.8), threshold);
  ASSERT_NEAR(seq(1.9), s1(1.9), threshold);
  ASSERT_NEAR(seq(2.0), s1(2.0), threshold);
  ASSERT_NEAR(seq(2.1), s1(2.1), threshold);
  ASSERT_NEAR(seq(2.2), s1(2.2), threshold);
  ASSERT_NEAR(seq(2.3), s1(2.3), threshold);
  ASSERT_NEAR(seq(2.4), s1(2.4), threshold);
  ASSERT_NEAR(seq(2.5), s1(2.5), threshold);
  ASSERT_NEAR(seq(2.6), s1(2.6), threshold);
  ASSERT_NEAR(seq(2.7), s1(2.7), threshold);
  ASSERT_NEAR(seq(2.8), s1(2.8), threshold);
  ASSERT_NEAR(seq(2.9), s1(2.9), threshold);
  ASSERT_NEAR(seq(3.0), s1(3.0), threshold);
  ASSERT_NEAR(seq(3.1), s1(3.1), threshold);
  ASSERT_NEAR(seq(3.2), s1(3.2), threshold);
  ASSERT_NEAR(seq(3.3), s2(3.3), threshold);
  ASSERT_NEAR(seq(3.4), s2(3.4), threshold);
  ASSERT_NEAR(seq(3.5), s2(3.5), threshold);
  ASSERT_NEAR(seq(3.6), s2(3.6), threshold);
  ASSERT_NEAR(seq(3.7), s2(3.7), threshold);
  ASSERT_NEAR(seq(3.8), s2(3.8), threshold);
  ASSERT_NEAR(seq(3.9), s2(3.9), threshold);
  ASSERT_NEAR(seq(4.0), s2(4.0), threshold);
  ASSERT_NEAR(seq(4.1), s2(4.1), threshold);
  ASSERT_NEAR(seq(4.2), s2(4.2), threshold);
  ASSERT_NEAR(seq(4.3), s2(4.3), threshold);
  ASSERT_NEAR(seq(4.4), s2(4.4), threshold);
  ASSERT_NEAR(seq(4.5), s2(4.5), threshold);
  ASSERT_NEAR(seq(4.6), s2(4.6), threshold);
  ASSERT_NEAR(seq(4.7), s2(4.7), threshold);
  ASSERT_NEAR(seq(4.8), s2(4.8), threshold);
  ASSERT_NEAR(seq(4.9), s2(4.9), threshold);
  ASSERT_NEAR(seq(5.0), s2(5.0), threshold);
  ASSERT_NEAR(seq(5.1), s2(5.1), threshold);
  ASSERT_NEAR(seq(5.2), s2(5.2), threshold);
  ASSERT_NEAR(seq(5.3), s2(5.3), threshold);
  ASSERT_NEAR(seq(5.4), s2(5.4), threshold);
  ASSERT_NEAR(seq(5.5), s3(5.5), threshold);
  ASSERT_NEAR(seq(5.6), s3(5.6), threshold);
  ASSERT_NEAR(seq(5.7), s3(5.7), threshold);
  ASSERT_NEAR(seq(5.8), s3(5.8), threshold);
  ASSERT_NEAR(seq(5.9), s3(5.9), threshold);
  ASSERT_NEAR(seq(6.0), s3(6.0), threshold);
  ASSERT_NEAR(seq(6.1), s3(6.1), threshold);
  ASSERT_NEAR(seq(6.2), s3(6.2), threshold);
  ASSERT_NEAR(seq(6.3), s3(6.3), threshold);
  ASSERT_NEAR(seq(6.4), s3(6.4), threshold);
  ASSERT_NEAR(seq(6.5), s3(6.5), threshold);
  ASSERT_NEAR(seq(6.6), s3(6.6), threshold);
  ASSERT_NEAR(seq(6.7), s3(6.7), threshold);
  ASSERT_NEAR(seq(6.8), s3(6.8), threshold);
  ASSERT_NEAR(seq(6.9), s3(6.9), threshold);
  ASSERT_NEAR(seq(7.0), s3(7.0), threshold);
  ASSERT_NEAR(seq(7.1), s3(7.1), threshold);
  ASSERT_NEAR(seq(7.2), s3(7.2), threshold);
  ASSERT_NEAR(seq(7.3), s3(7.3), threshold);
  ASSERT_NEAR(seq(7.4), s3(7.4), threshold);
  ASSERT_NEAR(seq(7.5), s3(7.5), threshold);
  ASSERT_NEAR(seq(7.6), s3(7.6), threshold);
  ASSERT_NEAR(seq(7.7), s3(7.7), threshold);
  ASSERT_NEAR(seq(7.8), s3(7.8), threshold);
  ASSERT_NEAR(seq(7.9), s3(7.9), threshold);
  ASSERT_NEAR(seq(8.0), s3(8.0), threshold);
  ASSERT_NEAR(seq(8.1), s3(8.1), threshold);
  ASSERT_NEAR(seq(8.2), s3(8.2), threshold);
  ASSERT_NEAR(seq(8.3), s3(8.3), threshold);
  ASSERT_NEAR(seq(8.4), s3(8.4), threshold);
  ASSERT_NEAR(seq(8.5), s3(8.5), threshold);
  ASSERT_NEAR(seq(8.6), s3(8.6), threshold);
  ASSERT_NEAR(seq(8.7), s3(8.7), threshold);
  ASSERT_NEAR(seq(8.8), s3(8.8), threshold);
  ASSERT_NEAR(seq(8.9), s3(8.9), threshold);
}

TEST(CubicSplineSequenceTests, test_two_splines_deriv) {
  std::vector<double> x  {1.1, 3.3,  5.5,  7.7};
  std::vector<double> y  {3.2, 8.5, -2.1,  2.4};
  std::vector<double> yp {1.2, 5.3, -9.1, -7.0};

  CubicSpline s1(x[0], y[0], yp[0],
                 x[1], y[1], yp[1]);
  CubicSpline s2(x[1], y[1], yp[1],
                 x[2], y[2], yp[2]);
  CubicSpline s3(x[2], y[2], yp[2],
                 x[3], y[3], yp[3]);
  auto s1p = s1.deriv();
  auto s2p = s2.deriv();
  auto s3p = s3.deriv();

  CubicSplineSequence seq(x, y, yp);
  auto seq_p = seq.deriv();

  double tol = 1e-10; // absolute tolerance
  ASSERT_NEAR(seq_p(0.3), s1p(0.3), tol);
  ASSERT_NEAR(seq_p(0.4), s1p(0.4), tol);
  ASSERT_NEAR(seq_p(0.5), s1p(0.5), tol);
  ASSERT_NEAR(seq_p(0.6), s1p(0.6), tol);
  ASSERT_NEAR(seq_p(0.7), s1p(0.7), tol);
  ASSERT_NEAR(seq_p(0.8), s1p(0.8), tol);
  ASSERT_NEAR(seq_p(0.9), s1p(0.9), tol);
  ASSERT_NEAR(seq_p(1.0), s1p(1.0), tol);
  ASSERT_NEAR(seq_p(1.1), s1p(1.1), tol);
  ASSERT_NEAR(seq_p(1.2), s1p(1.2), tol);
  ASSERT_NEAR(seq_p(1.3), s1p(1.3), tol);
  ASSERT_NEAR(seq_p(1.4), s1p(1.4), tol);
  ASSERT_NEAR(seq_p(1.5), s1p(1.5), tol);
  ASSERT_NEAR(seq_p(1.6), s1p(1.6), tol);
  ASSERT_NEAR(seq_p(1.7), s1p(1.7), tol);
  ASSERT_NEAR(seq_p(1.8), s1p(1.8), tol);
  ASSERT_NEAR(seq_p(1.9), s1p(1.9), tol);
  ASSERT_NEAR(seq_p(2.0), s1p(2.0), tol);
  ASSERT_NEAR(seq_p(2.1), s1p(2.1), tol);
  ASSERT_NEAR(seq_p(2.2), s1p(2.2), tol);
  ASSERT_NEAR(seq_p(2.3), s1p(2.3), tol);
  ASSERT_NEAR(seq_p(2.4), s1p(2.4), tol);
  ASSERT_NEAR(seq_p(2.5), s1p(2.5), tol);
  ASSERT_NEAR(seq_p(2.6), s1p(2.6), tol);
  ASSERT_NEAR(seq_p(2.7), s1p(2.7), tol);
  ASSERT_NEAR(seq_p(2.8), s1p(2.8), tol);
  ASSERT_NEAR(seq_p(2.9), s1p(2.9), tol);
  ASSERT_NEAR(seq_p(3.0), s1p(3.0), tol);
  ASSERT_NEAR(seq_p(3.1), s1p(3.1), tol);
  ASSERT_NEAR(seq_p(3.2), s1p(3.2), tol);
  ASSERT_NEAR(seq_p(3.3), s2p(3.3), tol);
  ASSERT_NEAR(seq_p(3.4), s2p(3.4), tol);
  ASSERT_NEAR(seq_p(3.5), s2p(3.5), tol);
  ASSERT_NEAR(seq_p(3.6), s2p(3.6), tol);
  ASSERT_NEAR(seq_p(3.7), s2p(3.7), tol);
  ASSERT_NEAR(seq_p(3.8), s2p(3.8), tol);
  ASSERT_NEAR(seq_p(3.9), s2p(3.9), tol);
  ASSERT_NEAR(seq_p(4.0), s2p(4.0), tol);
  ASSERT_NEAR(seq_p(4.1), s2p(4.1), tol);
  ASSERT_NEAR(seq_p(4.2), s2p(4.2), tol);
  ASSERT_NEAR(seq_p(4.3), s2p(4.3), tol);
  ASSERT_NEAR(seq_p(4.4), s2p(4.4), tol);
  ASSERT_NEAR(seq_p(4.5), s2p(4.5), tol);
  ASSERT_NEAR(seq_p(4.6), s2p(4.6), tol);
  ASSERT_NEAR(seq_p(4.7), s2p(4.7), tol);
  ASSERT_NEAR(seq_p(4.8), s2p(4.8), tol);
  ASSERT_NEAR(seq_p(4.9), s2p(4.9), tol);
  ASSERT_NEAR(seq_p(5.0), s2p(5.0), tol);
  ASSERT_NEAR(seq_p(5.1), s2p(5.1), tol);
  ASSERT_NEAR(seq_p(5.2), s2p(5.2), tol);
  ASSERT_NEAR(seq_p(5.3), s2p(5.3), tol);
  ASSERT_NEAR(seq_p(5.4), s2p(5.4), tol);
  ASSERT_NEAR(seq_p(5.5), s3p(5.5), tol);
  ASSERT_NEAR(seq_p(5.6), s3p(5.6), tol);
  ASSERT_NEAR(seq_p(5.7), s3p(5.7), tol);
  ASSERT_NEAR(seq_p(5.8), s3p(5.8), tol);
  ASSERT_NEAR(seq_p(5.9), s3p(5.9), tol);
  ASSERT_NEAR(seq_p(6.0), s3p(6.0), tol);
  ASSERT_NEAR(seq_p(6.1), s3p(6.1), tol);
  ASSERT_NEAR(seq_p(6.2), s3p(6.2), tol);
  ASSERT_NEAR(seq_p(6.3), s3p(6.3), tol);
  ASSERT_NEAR(seq_p(6.4), s3p(6.4), tol);
  ASSERT_NEAR(seq_p(6.5), s3p(6.5), tol);
  ASSERT_NEAR(seq_p(6.6), s3p(6.6), tol);
  ASSERT_NEAR(seq_p(6.7), s3p(6.7), tol);
  ASSERT_NEAR(seq_p(6.8), s3p(6.8), tol);
  ASSERT_NEAR(seq_p(6.9), s3p(6.9), tol);
  ASSERT_NEAR(seq_p(7.0), s3p(7.0), tol);
  ASSERT_NEAR(seq_p(7.1), s3p(7.1), tol);
  ASSERT_NEAR(seq_p(7.2), s3p(7.2), tol);
  ASSERT_NEAR(seq_p(7.3), s3p(7.3), tol);
  ASSERT_NEAR(seq_p(7.4), s3p(7.4), tol);
  ASSERT_NEAR(seq_p(7.5), s3p(7.5), tol);
  ASSERT_NEAR(seq_p(7.6), s3p(7.6), tol);
  ASSERT_NEAR(seq_p(7.7), s3p(7.7), tol);
  ASSERT_NEAR(seq_p(7.8), s3p(7.8), tol);
  ASSERT_NEAR(seq_p(7.9), s3p(7.9), tol);
  ASSERT_NEAR(seq_p(8.0), s3p(8.0), tol);
  ASSERT_NEAR(seq_p(8.1), s3p(8.1), tol);
  ASSERT_NEAR(seq_p(8.2), s3p(8.2), tol);
  ASSERT_NEAR(seq_p(8.3), s3p(8.3), tol);
  ASSERT_NEAR(seq_p(8.4), s3p(8.4), tol);
  ASSERT_NEAR(seq_p(8.5), s3p(8.5), tol);
  ASSERT_NEAR(seq_p(8.6), s3p(8.6), tol);
  ASSERT_NEAR(seq_p(8.7), s3p(8.7), tol);
  ASSERT_NEAR(seq_p(8.8), s3p(8.8), tol);
  ASSERT_NEAR(seq_p(8.9), s3p(8.9), tol);
}


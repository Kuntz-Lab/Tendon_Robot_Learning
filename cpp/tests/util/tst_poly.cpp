/**
 * author:        Michael Bentley
 * email:         mikebentley15@gmail.com
 * date-created:  06 August 2020
 */

#include "util/poly.h"

#include <Eigen/Core>

#include <gtest/gtest.h>

using Vec = Eigen::VectorXd;
using util::poly_at;
using util::poly_der;

std::ostream& operator<< (std::ostream& out, const Vec &vec) {
  out << "[";
  bool first = true;
  for (int i = 0; i < vec.size(); i++) {
    if (!first) { out << ", "; }
    first = false;
    out << vec[i];
  }
  return out << "]";
}

TEST(UtilPolyTests, poly_at_empty) {
  Vec poly;
  ASSERT_DOUBLE_EQ(poly_at(poly, -2.0), 0.0);
  ASSERT_DOUBLE_EQ(poly_at(poly, -1.0), 0.0);
  ASSERT_DOUBLE_EQ(poly_at(poly,  0.0), 0.0);
  ASSERT_DOUBLE_EQ(poly_at(poly,  1.0), 0.0);
  ASSERT_DOUBLE_EQ(poly_at(poly,  2.0), 0.0);
}

TEST(UtilPolyTests, poly_at_constant) {
  Vec poly(1);
  poly << 3.2;
  ASSERT_DOUBLE_EQ(poly_at(poly, -2.0), 3.2);
  ASSERT_DOUBLE_EQ(poly_at(poly, -1.0), 3.2);
  ASSERT_DOUBLE_EQ(poly_at(poly,  0.0), 3.2);
  ASSERT_DOUBLE_EQ(poly_at(poly,  1.0), 3.2);
  ASSERT_DOUBLE_EQ(poly_at(poly,  2.0), 3.2);
}

TEST(UtilPolyTests, poly_at_linear) {
  Vec poly(2);
  poly << 1.2, 2.0;
  ASSERT_DOUBLE_EQ(poly_at(poly, -2.0), -2.8);
  ASSERT_DOUBLE_EQ(poly_at(poly, -1.0), -0.8);
  ASSERT_DOUBLE_EQ(poly_at(poly,  0.0),  1.2);
  ASSERT_DOUBLE_EQ(poly_at(poly,  1.0),  3.2);
  ASSERT_DOUBLE_EQ(poly_at(poly,  2.0),  5.2);
}

TEST(UtilPolyTests, poly_at_quadratic) {
  Vec poly(3);
  poly << 1.2, 2.0, 1.0;
  ASSERT_DOUBLE_EQ(poly_at(poly, -2.0),  1.2);
  ASSERT_DOUBLE_EQ(poly_at(poly, -1.0),  0.2);
  ASSERT_DOUBLE_EQ(poly_at(poly,  0.0),  1.2);
  ASSERT_DOUBLE_EQ(poly_at(poly,  1.0),  4.2);
  ASSERT_DOUBLE_EQ(poly_at(poly,  2.0),  9.2);
}

TEST(UtilPolyTests, poly_at_cubic) {
  Vec poly(4);
  poly << 1.2, 1.0, 2.0, 1.0;
  ASSERT_DOUBLE_EQ(poly_at(poly, -2.0), -0.8);
  ASSERT_DOUBLE_EQ(poly_at(poly, -1.0),  1.2);
  ASSERT_DOUBLE_EQ(poly_at(poly,  0.0),  1.2);
  ASSERT_DOUBLE_EQ(poly_at(poly,  1.0),  5.2);
  ASSERT_DOUBLE_EQ(poly_at(poly,  2.0), 19.2);
}

TEST(UtilPolyTests, poly_at_many) {
  Vec poly(7);
  poly << 1.2, 1.0, 2.0, 0, 0, 1, 0;
  ASSERT_DOUBLE_EQ(poly_at(poly, -2.0), -24.8);
  ASSERT_DOUBLE_EQ(poly_at(poly, -1.0),   1.2);
  ASSERT_DOUBLE_EQ(poly_at(poly,  0.0),   1.2);
  ASSERT_DOUBLE_EQ(poly_at(poly,  1.0),   5.2);
  ASSERT_DOUBLE_EQ(poly_at(poly,  2.0),  43.2);
}

TEST(UtilPolyTests, poly_der_empty) {
  Vec poly;
  Vec expected_der;
  Vec actual_der = poly_der(poly);
  ASSERT_EQ(expected_der, actual_der)
    << "  expected: " << expected_der << "\n"
    << "  actual:   " << actual_der   << "\n";
}

TEST(UtilPolyTests, poly_der_constant) {
  Vec poly(1);
  poly << 2.3;
  Vec expected_der(1);
  expected_der << 0;
  Vec actual_der = poly_der(poly);
  ASSERT_EQ(expected_der, actual_der)
    << "  expected: " << expected_der << "\n"
    << "  actual:   " << actual_der   << "\n";
}

TEST(UtilPolyTests, poly_der_linear) {
  Vec poly(2);
  poly << 2.3, 1.3;
  Vec expected_der(2);
  expected_der << 1.3, 0.0;
  Vec actual_der = poly_der(poly);
  ASSERT_EQ(expected_der, actual_der)
    << "  expected: " << expected_der << "\n"
    << "  actual:   " << actual_der   << "\n";
}

TEST(UtilPolyTests, poly_der_quadratic) {
  Vec poly(3);
  poly << 2.3, 1.3, 0.25;
  Vec expected_der(3);
  expected_der << 1.3, 0.5, 0.0;
  Vec actual_der = poly_der(poly);
  ASSERT_EQ(expected_der, actual_der)
    << "  expected: " << expected_der << "\n"
    << "  actual:   " << actual_der   << "\n";
}

TEST(UtilPolyTests, poly_der_cubic) {
  Vec poly(4);
  poly << 2.3, 1.3, 0.25, 1.0;
  Vec expected_der(4);
  expected_der << 1.3, 0.5, 3.0, 0.0;
  Vec actual_der = poly_der(poly);
  ASSERT_EQ(expected_der, actual_der)
    << "  expected: " << expected_der << "\n"
    << "  actual:   " << actual_der   << "\n";
}

TEST(UtilPolyTests, poly_der_many) {
  Vec poly(10);
  poly << 2.3, 1.3, 0.25, 1, 2, 3, 4, 5, 6, 7;
  Vec expected_der(10);
  expected_der << 1.3, 0.5, 3, 8, 15, 24, 35, 48, 63, 0;
  Vec actual_der = poly_der(poly);
  ASSERT_EQ(expected_der, actual_der)
    << "  expected: " << expected_der << "\n"
    << "  actual:   " << actual_der   << "\n";
}

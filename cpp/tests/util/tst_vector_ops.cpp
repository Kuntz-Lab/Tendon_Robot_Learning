/**
 * author:          Michael Bentley
 * email:           mikebentley15@gmail.com
 * date-created:    11 November 2021
 */

#include "util/vector_ops.h"

#include <gtest/gtest.h>

#include <vector>

TEST(UtilVectorOpsTests, vector_range_same_vector) {
  std::vector<double> a = {1,2,3,4};
  auto b = a;
  double dL = 0.1;
  std::vector<std::vector<double>> expected = {a};
  auto actual = util::range(a, b, dL);
  ASSERT_EQ(expected, actual);
}

TEST(UtilVectorOpsTests, range_same_value) {
  double a = 1;
  auto b = a;
  double dL = 0.1;
  std::vector<double> expected = {a};
  auto actual = util::range(a, b, dL);
  ASSERT_EQ(expected, actual);
}

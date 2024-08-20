/**
 * author:          Michael Bentley
 * email:           mikebentley15@gmail.com
 * date-created:    26 March 2022
 */

#include "util/angles.h"

#include <gtest/gtest.h>

#include <vector>

TEST(UtilAnglesTests, fmod_floor) {
  ASSERT_NEAR(util::fmod_floor( 1.1, 1.9),  1.1, 1e-7);
  ASSERT_NEAR(util::fmod_floor(-1.1, 1.9),  0.8, 1e-7);
  ASSERT_NEAR(util::fmod_floor( 2.1, 1.9),  0.2, 1e-7);
  ASSERT_NEAR(util::fmod_floor(-2.1, 1.9),  1.7, 1e-7);
  ASSERT_NEAR(util::fmod_floor( 4.1, 1.1),  0.8, 1e-7);
  ASSERT_NEAR(util::fmod_floor(-4.1, 1.1),  0.3, 1e-7);

  // show comparatively what std::fmod() does
  ASSERT_NEAR(std::fmod( 1.1, 1.9),  1.1, 1e-7);
  ASSERT_NEAR(std::fmod(-1.1, 1.9), -1.1, 1e-7);
  ASSERT_NEAR(std::fmod( 2.1, 1.9),  0.2, 1e-7);
  ASSERT_NEAR(std::fmod(-2.1, 1.9), -0.2, 1e-7);
  ASSERT_NEAR(std::fmod( 4.1, 1.1),  0.8, 1e-7);
  ASSERT_NEAR(std::fmod(-4.1, 1.1), -0.8, 1e-7);
}

TEST(UtilAnglesTests, wrap_range) {
  // from 0 to x
  ASSERT_NEAR(util::wrap_range( 1.1, 0.0, 1.9),  1.1, 1e-7);
  ASSERT_NEAR(util::wrap_range(-1.1, 0.0, 1.9),  0.8, 1e-7);
  ASSERT_NEAR(util::wrap_range( 2.1, 0.0, 1.9),  0.2, 1e-7);
  ASSERT_NEAR(util::wrap_range(-2.1, 0.0, 1.9),  1.7, 1e-7);
  ASSERT_NEAR(util::wrap_range( 4.1, 0.0, 1.1),  0.8, 1e-7);
  ASSERT_NEAR(util::wrap_range(-4.1, 0.0, 1.1),  0.3, 1e-7);

  // from -x to 0
  ASSERT_NEAR(util::wrap_range( 1.1, -1.9, 0.0), -0.8, 1e-7);
  ASSERT_NEAR(util::wrap_range(-1.1, -1.9, 0.0), -1.1, 1e-7);
  ASSERT_NEAR(util::wrap_range( 2.1, -1.9, 0.0), -1.7, 1e-7);
  ASSERT_NEAR(util::wrap_range(-2.1, -1.9, 0.0), -0.2, 1e-7);
  ASSERT_NEAR(util::wrap_range( 4.1, -1.1, 0.0), -0.3, 1e-7);
  ASSERT_NEAR(util::wrap_range(-4.1, -1.1, 0.0), -0.8, 1e-7);

  // from -x to x
  ASSERT_NEAR(util::wrap_range( 1.1, -1.9, 1.9),  1.1, 1e-7);
  ASSERT_NEAR(util::wrap_range(-1.1, -1.9, 1.9), -1.1, 1e-7);
  ASSERT_NEAR(util::wrap_range( 2.1, -1.9, 1.9), -1.7, 1e-7);
  ASSERT_NEAR(util::wrap_range(-2.1, -1.9, 1.9),  1.7, 1e-7);
  ASSERT_NEAR(util::wrap_range( 4.1, -1.1, 1.1), -0.3, 1e-7);
  ASSERT_NEAR(util::wrap_range(-4.1, -1.1, 1.1),  0.3, 1e-7);

  // from a to b
  ASSERT_NEAR(util::wrap_range( 1.1,  1.9,  2.9),  2.1, 1e-7);
  ASSERT_NEAR(util::wrap_range(-1.1,  1.9,  2.9),  1.9, 1e-7);
  ASSERT_NEAR(util::wrap_range( 2.1,  1.9,  2.9),  2.1, 1e-7);
  ASSERT_NEAR(util::wrap_range(-2.1,  1.9,  2.9),  1.9, 1e-7);
  ASSERT_NEAR(util::wrap_range( 4.1,  1.1,  2.1),  2.1, 1e-7);
  ASSERT_NEAR(util::wrap_range(-4.1,  1.1,  2.1),  1.9, 1e-7);
}

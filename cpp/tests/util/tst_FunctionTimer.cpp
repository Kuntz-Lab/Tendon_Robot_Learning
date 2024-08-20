/**
 * author:        Michael Bentley
 * email:         mikebentley15@gmail.com
 * date-created:  02 December 2020
 */

#include "util/FunctionTimer.h"

#include <gtest/gtest.h>

#include <unistd.h>  // for usleep()

TEST(FunctionTimerTests, void_return_compiles) {
  util::FunctionTimer timer;

  ASSERT_EQ(timer.get_times().size(), size_t(0));
  timer.time([]() { usleep(20000); });
  ASSERT_EQ(timer.get_times().size(), size_t(1));
  ASSERT_GE(timer.get_times()[0], 0.02f);
}

TEST(FunctionTimerTests, nonvoid_return_compiles) {
  util::FunctionTimer timer;
  
  ASSERT_EQ(timer.get_times().size(), size_t(0));
  auto success = timer.time([]() { return usleep(20000); });
  ASSERT_EQ(timer.get_times().size(), size_t(1));
  ASSERT_GE(timer.get_times()[0], 0.02f);
  ASSERT_EQ(success, 0);
}

TEST(FunctionTimerTests, time_function_call_with_void_func) {
  std::vector<int> args;
  auto f = [&args](int x, int y, int z) {
    args.emplace_back(x);
    args.emplace_back(y);
    args.emplace_back(z);
  };
  util::time_function_call(f, 5, 4, 3);
  std::vector<int> expected {5, 4, 3};
  ASSERT_EQ(expected, args);
}

TEST(FunctionTimerTests, time_function_call_with_return_func) {
  std::vector<int> args;
  auto f = [&args](int x, int y, int z) {
    args.emplace_back(x);
    args.emplace_back(y);
    args.emplace_back(z);
    return args;
  };
  [[maybe_unused]] float timing;
  auto returned = util::time_function_call(f, timing, 5, 4, 3);
  std::vector<int> expected {5, 4, 3};
  ASSERT_EQ(expected, args);
  ASSERT_EQ(returned, args);
}

TEST(FunctionTimerTests, time_function_call_can_return_reference) {
  int a = 3;
  auto f = [&a](int) -> int& {
    return a;
  };
  float timing;
  int b = util::time_function_call(f, timing, 5);
  int &c = util::time_function_call(f, timing, 4);
  ASSERT_EQ(a, b);
  ASSERT_EQ(&a, &c);
}

TEST(FunctionTimerTests, wrap_can_return_reference) {
  int a = 3;
  auto f = [&a]() -> int& { return a; };
  util::FunctionTimer timer;
  auto wrapped = timer.wrap(f);

  int &b = f();
  ASSERT_EQ(&a, &b);

  int c = wrapped();
  ASSERT_EQ(a, c);

  int &d = wrapped();
  ASSERT_EQ(&a, &d);
}

TEST(FunctionTimerTests, time_can_return_reference) {
  int a = 3;
  auto f = [&a]() -> int& { return a; };
  util::FunctionTimer timer;

  int &b = f();
  ASSERT_EQ(&a, &b);

  int c = timer.time(f);
  ASSERT_EQ(a, c);

  int &d = timer.time(f);
  ASSERT_EQ(&a, &d);
}

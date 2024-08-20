/**
 * author:        Michael Bentley
 * email:         mikebentley15@gmail.com
 * date-created:  18 March 2020
 */

#include "cpptoml/toml_conversions.h"
#include <cpptoml/cpptoml.h>

#include <collision/Point.h>
#include <collision/Sphere.h>
#include <collision/Capsule.h>
#include <collision/CapsuleSequence.h>

#include <gtest/gtest.h>

#include <string>

using collision::operator<<;

TEST(toml_conversion_tests, to_stream_array) {
  auto arr = cpptoml::make_array();
  arr->push_back("hi");
  arr->push_back("hello");
  arr->push_back("world!");
  std::ostringstream out;
  cpptoml::to_stream(out, arr);
  ASSERT_EQ("[\"hi\", \"hello\", \"world!\"]", out.str());
}

TEST(toml_conversion_tests, to_stream_array_nullptr) {
  std::shared_ptr<cpptoml::array> arr;
  std::ostringstream out;
  ASSERT_THROW(cpptoml::to_stream(out, arr), std::invalid_argument);
}

TEST(toml_conversion_tests, to_stream_table) {
  auto tbl = cpptoml::make_table();
  tbl->insert("name", "mike");
  tbl->insert("age", 30);
  std::string expected =
    "age = 30\n"
    "name = \"mike\"\n";
  std::ostringstream out;
  cpptoml::to_stream(out, tbl);
  ASSERT_EQ(expected, out.str());
}

TEST(toml_conversion_tests, to_stream_table_nullptr) {
  std::shared_ptr<cpptoml::table> tbl;
  std::ostringstream out;
  ASSERT_THROW(cpptoml::to_stream(out, tbl), std::invalid_argument);
}

TEST(toml_conversion_tests, to_stream_table_array) {
  auto tbl_1 = cpptoml::make_table();
  tbl_1->insert("name", "mike");
  tbl_1->insert("age", 30);
  auto tbl_2 = cpptoml::make_table();
  tbl_2->insert("name", "sam");
  tbl_2->insert("age", 29);
  auto tbl_arr = cpptoml::make_table_array();
  tbl_arr->push_back(tbl_1);
  tbl_arr->push_back(tbl_2);
  auto tbl_container = cpptoml::make_table();
  tbl_container->insert("tbl_array", tbl_arr);
  std::string expected =
    "[[tbl_array]]\n"
    "  age = 30\n"
    "  name = \"mike\"\n"
    "[[tbl_array]]\n"
    "  age = 29\n"
    "  name = \"sam\"\n";
  std::ostringstream out;
  cpptoml::to_stream(out, tbl_container);
  ASSERT_EQ(expected, out.str());
}

TEST(toml_conversion_tests, to_stream_table_array_nullptr) {
  std::shared_ptr<cpptoml::table_array> tbl_arr;
  std::ostringstream out;
  ASSERT_THROW(cpptoml::to_stream(out, tbl_arr), std::invalid_argument);
}

TEST(toml_conversion_tests, to_stream_nested_array) {
  auto arr_1 = cpptoml::make_array();
  arr_1->push_back(1);
  arr_1->push_back(2);
  arr_1->push_back(3);
  auto arr_2 = cpptoml::make_array();
  arr_2->push_back("mike");
  arr_2->push_back("sam");
  auto arr_3 = cpptoml::make_array();
  arr_3->push_back(true);
  arr_3->push_back(false);
  arr_3->push_back(false);
  auto arr = cpptoml::make_array();
  arr->push_back(arr_1);
  arr->push_back(arr_2);
  arr->push_back(arr_3);
  std::string expected =
    "[[1, 2, 3], [\"mike\", \"sam\"], [true, false, false]]";
  std::ostringstream out;
  cpptoml::to_stream(out, arr);
  ASSERT_EQ(expected, out.str());
}

TEST(toml_conversion_tests, to_stream_nested_tables) {
  auto tbl_1 = cpptoml::make_table();
  tbl_1->insert("name", "mike");
  tbl_1->insert("age", 30);
  auto tbl_2 = cpptoml::make_table();
  tbl_2->insert("name", "sam");
  tbl_2->insert("age", 29);
  auto tbl = cpptoml::make_table();
  tbl->insert("husband", tbl_1);
  tbl->insert("wife", tbl_2);
  std::string expected =
    "[husband]\n"
    "  age = 30\n"
    "  name = \"mike\"\n"
    "[wife]\n"
    "  age = 29\n"
    "  name = \"sam\"\n";
  std::ostringstream out;
  cpptoml::to_stream(out, tbl);
  ASSERT_EQ(expected, out.str());
}

TEST(toml_conversion_tests, to_string_array) {
  auto arr = cpptoml::make_array();
  arr->push_back("hi");
  arr->push_back("hello");
  arr->push_back("world!");
  ASSERT_EQ("[\"hi\", \"hello\", \"world!\"]", cpptoml::to_string(arr));
}

TEST(toml_conversion_tests, to_string_array_nullptr) {
  std::shared_ptr<cpptoml::array> arr;
  ASSERT_THROW(cpptoml::to_string(arr), std::invalid_argument);
}

TEST(toml_conversion_tests, to_string_table) {
  auto tbl = cpptoml::make_table();
  tbl->insert("name", "mike");
  tbl->insert("age", 30);
  std::string expected =
    "age = 30\n"
    "name = \"mike\"\n";
  ASSERT_EQ(expected, cpptoml::to_string(tbl));
}

TEST(toml_conversion_tests, to_string_table_nullptr) {
  std::shared_ptr<cpptoml::table> tbl;
  ASSERT_THROW(cpptoml::to_string(tbl), std::invalid_argument);
}

TEST(toml_conversion_tests, to_string_table_array) {
  auto tbl_1 = cpptoml::make_table();
  tbl_1->insert("name", "mike");
  tbl_1->insert("age", 30);
  auto tbl_2 = cpptoml::make_table();
  tbl_2->insert("name", "sam");
  tbl_2->insert("age", 29);
  auto tbl_arr = cpptoml::make_table_array();
  tbl_arr->push_back(tbl_1);
  tbl_arr->push_back(tbl_2);
  auto tbl_container = cpptoml::make_table();
  tbl_container->insert("tbl_array", tbl_arr);
  std::string expected =
    "[[tbl_array]]\n"
    "  age = 30\n"
    "  name = \"mike\"\n"
    "[[tbl_array]]\n"
    "  age = 29\n"
    "  name = \"sam\"\n";
  ASSERT_EQ(expected, cpptoml::to_string(tbl_container));
}

TEST(toml_conversion_tests, to_string_table_array_nullptr) {
  std::shared_ptr<cpptoml::table_array> tbl_arr;
  ASSERT_THROW(cpptoml::to_string(tbl_arr), std::invalid_argument);
}

TEST(toml_conversion_tests, to_string_nested_array) {
  auto arr_1 = cpptoml::make_array();
  arr_1->push_back(1);
  arr_1->push_back(2);
  arr_1->push_back(3);
  auto arr_2 = cpptoml::make_array();
  arr_2->push_back("mike");
  arr_2->push_back("sam");
  auto arr_3 = cpptoml::make_array();
  arr_3->push_back(true);
  arr_3->push_back(false);
  arr_3->push_back(false);
  auto arr = cpptoml::make_array();
  arr->push_back(arr_1);
  arr->push_back(arr_2);
  arr->push_back(arr_3);
  std::string expected =
    "[[1, 2, 3], [\"mike\", \"sam\"], [true, false, false]]";
  ASSERT_EQ(expected, cpptoml::to_string(arr));
}

TEST(toml_conversion_tests, to_string_nested_tables) {
  auto tbl_1 = cpptoml::make_table();
  tbl_1->insert("name", "mike");
  tbl_1->insert("age", 30);
  auto tbl_2 = cpptoml::make_table();
  tbl_2->insert("name", "sam");
  tbl_2->insert("age", 29);
  auto tbl = cpptoml::make_table();
  tbl->insert("husband", tbl_1);
  tbl->insert("wife", tbl_2);
  std::string expected =
    "[husband]\n"
    "  age = 30\n"
    "  name = \"mike\"\n"
    "[wife]\n"
    "  age = 29\n"
    "  name = \"sam\"\n";
  ASSERT_EQ(expected, cpptoml::to_string(tbl));
}

TEST(toml_conversion_tests, to_toml_vector) {
  Eigen::VectorXd vec(5);
  vec << 1, 2, 3, 4, 5;
  ASSERT_EQ(vec, cpptoml::to_vector(cpptoml::to_toml(vec)));
}

TEST(toml_conversion_tests, to_toml_vector3) {
  Eigen::Vector3d v{1.1, 2.2, 3.3};
  ASSERT_EQ(v, cpptoml::to_point(cpptoml::to_toml(v)));
}

TEST(toml_conversion_tests, to_toml_point) {
  collision::Point p{1.1, 2.2, 3.3};
  ASSERT_EQ(p, cpptoml::to_point(cpptoml::to_toml(p)));
}

TEST(toml_conversion_tests, to_toml_matrix) {
  Eigen::Matrix3d mat;
  mat << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;
  ASSERT_EQ(mat, cpptoml::to_matrix(cpptoml::to_toml(mat)));
}

TEST(toml_conversion_tests, to_toml_sphere) {
  collision::Sphere s{{3.3, 2.2, 1.1}, 2.5};
  auto actual = collision::Sphere::from_toml(cpptoml::to_toml(s));
  ASSERT_EQ(s.c, actual.c);
  ASSERT_EQ(s.r, actual.r);
}

TEST(toml_conversion_tests, to_toml_capsule) {
  collision::Capsule c{{1, 2, 3}, {4, 5, 6}, 7};
  auto actual = collision::Capsule::from_toml(cpptoml::to_toml(c));
  ASSERT_EQ(c.a, actual.a);
  ASSERT_EQ(c.b, actual.b);
  ASSERT_EQ(c.r, actual.r);
}

TEST(toml_conversion_tests, to_toml_capsule_sequence_empty) {
  collision::CapsuleSequence seq{{}, 10};
  auto actual = collision::CapsuleSequence::from_toml(cpptoml::to_toml(seq));
  ASSERT_EQ(seq.points, actual.points);
  ASSERT_EQ(seq.r, actual.r);
}

TEST(toml_conversion_tests, to_toml_capsule_sequence) {
  collision::CapsuleSequence seq{{{1,2,3}, {4,5,6}, {7,8,9}}, 10};
  auto actual = collision::CapsuleSequence::from_toml(cpptoml::to_toml(seq));
  ASSERT_EQ(seq.points, actual.points);
  ASSERT_EQ(seq.r, actual.r);
}

TEST(toml_conversion_tests, to_toml_stdvec_double) {
  std::vector<double> vec{1, 2, 3, 4};
  ASSERT_EQ(vec, cpptoml::to_stdvec<double>(cpptoml::to_toml(vec)));
}

TEST(toml_conversion_tests, to_toml_stdvec_string) {
  std::vector<std::string> vec{"hi"};
  ASSERT_EQ(vec, cpptoml::to_stdvec<std::string>(cpptoml::to_toml(vec)));
}

TEST(toml_conversion_tests, to_toml_stdvec_int_empty) {
  std::vector<int64_t> vec{};
  ASSERT_EQ(vec, cpptoml::to_stdvec<int64_t>(cpptoml::to_toml(vec)));
}

TEST(toml_conversion_tests, to_toml_stdvec_sphere) {
  std::vector<collision::Sphere> vec{{{1,2,3}, 1}, {{4,5,6}, 2}};
  ASSERT_EQ(vec, cpptoml::to_stdvec<collision::Sphere>(cpptoml::to_toml(vec)));
}

TEST(toml_conversion_tests, to_toml_stdvec_capsule) {
  std::vector<collision::Capsule> vec{
    {{1,2,3}, {2,3,4}, 1},
    {{3,4,5}, {4,5,6}, 2},
    {{4,5,6}, {7,8,9}, 3},
  };
  ASSERT_EQ(vec, cpptoml::to_stdvec<collision::Capsule>(cpptoml::to_toml(vec)));
}

TEST(toml_conversion_tests, to_toml_stdvec_capsule_sequence) {
  std::vector<collision::CapsuleSequence> vec{
    {{{1,2,3}, {2,3,4}, {3,4,5}, {4,5,6}}, 1},
    {{{1,2,3}, {3,2,1}}, 2},
    {{{3,2,1}, {4,3,2}, {5,4,3}, {6,5,4}, {7,6,5}, {8,7,6}}, 3},
  };
  ASSERT_EQ(vec, cpptoml::to_stdvec<collision::CapsuleSequence>(
                     cpptoml::to_toml(vec)));
}

TEST(toml_conversion_tests, to_vector_nullptr) {
  ASSERT_THROW(cpptoml::to_vector(nullptr), std::invalid_argument);
}

TEST(toml_conversion_tests, to_vector_empty) {
  auto arr = cpptoml::make_array();
  Eigen::VectorXd expected;
  ASSERT_EQ(expected, cpptoml::to_vector(arr));
}

TEST(toml_conversion_tests, to_vector_wrong_type) {
  auto arr = cpptoml::make_array();
  arr->push_back("bob");
  arr->push_back("name");
  ASSERT_THROW(cpptoml::to_vector(arr), cpptoml::parse_exception);
}

TEST(toml_conversion_tests, to_vector_double_array) {
  auto arr = cpptoml::make_array();
  auto subarr = cpptoml::make_array();
  subarr->push_back(1.1);
  arr->push_back(subarr);
  ASSERT_THROW(cpptoml::to_vector(arr), cpptoml::parse_exception);
}

TEST(toml_conversion_tests, to_point_nullptr) {
  ASSERT_THROW(cpptoml::to_point(nullptr), std::invalid_argument);
}

TEST(toml_conversion_tests, to_point_empty) {
  auto arr = cpptoml::make_array();
  ASSERT_THROW(cpptoml::to_point(arr), std::out_of_range);
}

TEST(toml_conversion_tests, to_point_wrong_type) {
  auto arr = cpptoml::make_array();
  arr->push_back("amy");
  arr->push_back("name");
  arr->push_back("bob");
  ASSERT_THROW(cpptoml::to_point(arr), cpptoml::parse_exception);
}

TEST(toml_conversion_tests, to_point_too_few) {
  auto arr = cpptoml::make_array();
  arr->push_back(1.1);
  arr->push_back(2.2);
  ASSERT_THROW(cpptoml::to_point(arr), std::out_of_range);
}

TEST(toml_conversion_tests, to_point_too_many) {
  auto arr = cpptoml::make_array();
  arr->push_back(1.1);
  arr->push_back(2.1);
  arr->push_back(3.1);
  arr->push_back(4.1);
  ASSERT_THROW(cpptoml::to_point(arr), cpptoml::parse_exception);
}

TEST(toml_conversion_tests, to_matrix_nullptr) {
  ASSERT_THROW(cpptoml::to_matrix(nullptr), std::invalid_argument);
}

TEST(toml_conversion_tests, to_matrix_empty) {
  ASSERT_THROW(cpptoml::to_matrix(cpptoml::make_table()), std::out_of_range);
}

TEST(toml_conversion_tests, to_matrix_wrong_type) {
  Eigen::Matrix3d mat;
  mat << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;

  auto tbl = cpptoml::to_toml(mat);
  tbl->insert("row_1", cpptoml::to_toml(
                           std::vector<std::string>{"hi", "there", "friend"}));
  ASSERT_THROW(cpptoml::to_matrix(tbl), cpptoml::parse_exception);

  tbl = cpptoml::to_toml(mat);
  tbl->insert("row_2", "name");
  ASSERT_THROW(cpptoml::to_matrix(tbl), cpptoml::parse_exception);

  tbl = cpptoml::to_toml(mat);
  tbl->insert("row_3", 3.14);
  ASSERT_THROW(cpptoml::to_matrix(tbl), cpptoml::parse_exception);
}

TEST(toml_conversion_tests, to_matrix_missing_row_1) {
  Eigen::Matrix3d mat;
  mat << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;
  auto tbl = cpptoml::to_toml(mat);
  tbl->erase("row_1");
  ASSERT_THROW(cpptoml::to_matrix(tbl), std::out_of_range);
}

TEST(toml_conversion_tests, to_matrix_missing_row_2) {
  Eigen::Matrix3d mat;
  mat << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;
  auto tbl = cpptoml::to_toml(mat);
  tbl->erase("row_2");
  ASSERT_THROW(cpptoml::to_matrix(tbl), std::out_of_range);
}

TEST(toml_conversion_tests, to_matrix_missing_row_3) {
  Eigen::Matrix3d mat;
  mat << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;
  auto tbl = cpptoml::to_toml(mat);
  tbl->erase("row_3");
  ASSERT_THROW(cpptoml::to_matrix(tbl), std::out_of_range);
}

TEST(toml_conversion_tests, to_matrix_too_few_vals) {
  Eigen::Matrix3d mat;
  mat << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;
  auto tbl = cpptoml::to_toml(mat);
  tbl->insert("row_1", cpptoml::to_toml(std::vector<double>{1, 2}));
  ASSERT_THROW(cpptoml::to_matrix(tbl), std::out_of_range);
}

TEST(toml_conversion_tests, to_matrix_too_many_rows) {
  Eigen::Matrix3d mat;
  mat << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;
  auto tbl = cpptoml::to_toml(mat);
  tbl->insert("row_4", cpptoml::to_toml(Eigen::Vector3d{10, 11, 12}));
  ASSERT_NO_THROW(cpptoml::to_matrix(tbl));
}

TEST(toml_conversion_tests, to_matrix_too_many_vals) {
  Eigen::Matrix3d mat;
  mat << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;
  auto tbl = cpptoml::to_toml(mat);
  tbl->insert("row_3", cpptoml::to_toml(std::vector<double>{1, 2, 3, 4}));
  ASSERT_THROW(cpptoml::to_matrix(tbl), cpptoml::parse_exception);
}

TEST(toml_conversion_tests, to_stdvec_array_nullptr) {
  cpptoml::array_ptr arr = nullptr;
  ASSERT_THROW(cpptoml::to_stdvec<double>(arr), std::invalid_argument);
}

TEST(toml_conversion_tests, to_stdvec_array_empty) {
  ASSERT_EQ(std::vector<double>{},
            cpptoml::to_stdvec<double>(cpptoml::make_array()));
}

TEST(toml_conversion_tests, to_stdvec_array_wrong_type) {
  auto arr = cpptoml::to_toml(std::vector<std::string>{"3", "4"});
  ASSERT_THROW(cpptoml::to_stdvec<double>(arr), cpptoml::parse_exception);
}

TEST(toml_conversion_tests, to_stdvec_table_array_nullptr) {
  cpptoml::table_array_ptr tbl_arr = nullptr;
  ASSERT_THROW(cpptoml::to_stdvec<collision::Sphere>(tbl_arr),
               std::invalid_argument);
}

TEST(toml_conversion_tests, to_stdvec_table_array_empty) {
  ASSERT_EQ(cpptoml::to_stdvec<collision::Sphere>(cpptoml::make_table_array()),
            std::vector<collision::Sphere>{});
}

TEST(toml_conversion_tests, to_stdvec_table_array_wrong_type) {
  std::vector<collision::Sphere> spheres {
    {{1,2,3}, 1},
    {{2,3,4}, 2},
    {{3,4,5}, 3},
  };
  auto tbl_arr = cpptoml::to_toml(spheres);
  ASSERT_THROW(cpptoml::to_stdvec<collision::Capsule>(tbl_arr),
               std::out_of_range);
}

/**
 * author:        Michael Bentley
 * email:         mikebentley15@gmail.com
 * date-created:  28 February 2020
 */

#include "collision/collision.h"

#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>

#include <gtest/gtest.h>

using collision::operator<<;
using collision::Mesh;
using collision::Capsule;
using collision::CapsuleSequence;
using collision::Point;
using collision::Sphere;
using collision::collides;

using CollisionManager = fcl::DynamicAABBTreeCollisionManager;

//
// Point
//

TEST(CollisionTests, point_against_point) {
  Point p1{1, 2, 3};
  Point p2{1.01, 2, 3};
  ASSERT_TRUE(collides(p1, p1));
  ASSERT_TRUE(collides(p2, p2));
  ASSERT_FALSE(collides(p1, p2));
  ASSERT_FALSE(collides(p2, p1));
}


//
// Sphere
//

TEST(CollisionTests, point_against_sphere) {
  Point p{1.0, 2.0, 3.0};
  Point outside{1.0, 2.0, 3.2};
  Sphere s{p, 0.1};
  ASSERT_TRUE(collides(p, s));
  ASSERT_FALSE(collides(outside, s));
}

TEST(CollisionTests, sphere_against_point) {
  Point p{1.0, 2.0, 3.0};
  Point outside{1.0, 2.0, 3.2};
  Sphere s{p, 0.1};
  ASSERT_TRUE(collides(s, p));
  ASSERT_FALSE(collides(s, outside));
}

TEST(CollisionTests, sphere_against_sphere) {
  Sphere a{{1, 2, 3}, 1};
  Sphere b{{0, 1, 2}, 2};
  Sphere c{{0, 0, 0}, 1};
  ASSERT_TRUE(collides(a, a));
  ASSERT_TRUE(collides(b, b));
  ASSERT_TRUE(collides(c, c));
  ASSERT_TRUE(collides(a, b));
  ASSERT_TRUE(collides(b, a));
  ASSERT_TRUE(collides(b, c));
  ASSERT_TRUE(collides(c, b));
  ASSERT_FALSE(collides(a, c));
  ASSERT_FALSE(collides(c, a));
}


//
// Capsule
//

TEST(CollisionTests, capsule_against_point) {
  Capsule c{{1, 1, 0}, {1, 1, 2}, 2};
  Point in1 {1, -0.8,   1};  // cylinder section
  Point in2 {1,    0,  -1};  // bottom hemisphere section
  Point in3 {1,    2,   3};  // top hemisphere section
  Point out1{1,  3.1,   1};  // out the side
  Point out2{1,    2,  -2};  // out the bottom
  Point out3{1,    1, 4.2};  // out the top
  ASSERT_TRUE(collides(c, in1));
  ASSERT_TRUE(collides(c, in2));
  ASSERT_TRUE(collides(c, in3));
  ASSERT_FALSE(collides(c, out1));
  ASSERT_FALSE(collides(c, out2));
  ASSERT_FALSE(collides(c, out3));
}

TEST(CollisionTests, point_against_capsule) {
  Capsule c{{1, 1, 0}, {1, 1, 2}, 2};
  Point in1 {1, -0.8,   1};  // cylinder section
  Point in2 {1,    0,  -1};  // bottom hemisphere section
  Point in3 {1,    2,   3};  // top hemisphere section
  Point out1{1,  3.1,   1};  // out the side
  Point out2{1,    2,  -2};  // out the bottom
  Point out3{1,    1, 4.2};  // out the top
  ASSERT_TRUE(collides(in1, c));
  ASSERT_TRUE(collides(in2, c));
  ASSERT_TRUE(collides(in3, c));
  ASSERT_FALSE(collides(out1, c));
  ASSERT_FALSE(collides(out2, c));
  ASSERT_FALSE(collides(out3, c));
}

TEST(CollisionTests, capsule_against_sphere) {
  Capsule c{{1, 1, 0}, {1, 1, 2}, 1};
  Sphere in1 {{1, -0.8,   1}, 1};  // cylinder section
  Sphere in2 {{1,    0,  -1}, 1};  // bottom hemisphere section
  Sphere in3 {{1,    2,   3}, 1};  // top hemisphere section
  Sphere out1{{1,  3.1,   1}, 1};  // out the side
  Sphere out2{{1,    2,  -2}, 1};  // out the bottom
  Sphere out3{{1,    1, 4.2}, 1};  // out the top
  ASSERT_TRUE(collides(c, in1));
  ASSERT_TRUE(collides(c, in2));
  ASSERT_TRUE(collides(c, in3));
  ASSERT_FALSE(collides(c, out1));
  ASSERT_FALSE(collides(c, out2));
  ASSERT_FALSE(collides(c, out3));
}

TEST(CollisionTests, sphere_against_capsule) {
  Capsule c{{1, 1, 0}, {1, 1, 2}, 1};
  Sphere in1 {{1, -0.8,   1}, 1};  // cylinder section
  Sphere in2 {{1,    0,  -1}, 1};  // bottom hemisphere section
  Sphere in3 {{1,    2,   3}, 1};  // top hemisphere section
  Sphere out1{{1,  3.1,   1}, 1};  // out the side
  Sphere out2{{1,    2,  -2}, 1};  // out the bottom
  Sphere out3{{1,    1, 4.2}, 1};  // out the top
  ASSERT_TRUE(collides(in1, c));
  ASSERT_TRUE(collides(in2, c));
  ASSERT_TRUE(collides(in3, c));
  ASSERT_FALSE(collides(out1, c));
  ASSERT_FALSE(collides(out2, c));
  ASSERT_FALSE(collides(out3, c));
}

TEST(CollisionTests, capsule_against_capsule_too_close) {
  Point A{1, 0, 0}, B{2, 0, 0}, C{3, 0, 0};
  Capsule a{A, A, 0.1};
  Capsule b{B, B, 1.1};
  Capsule bc{B, C, 0.8};
  ASSERT_TRUE (collides( a,  a));
  ASSERT_TRUE (collides( a,  b));
  ASSERT_FALSE(collides( a, bc));
  ASSERT_TRUE (collides( b,  a));
  ASSERT_TRUE (collides( b,  b));
  ASSERT_TRUE (collides( b, bc));
  ASSERT_FALSE(collides(bc,  a));
  ASSERT_TRUE (collides(bc,  b));
  ASSERT_TRUE (collides(bc, bc));
}

TEST(CollisionTests, capsule_against_capsule_parallel) {
  Capsule a{{1, 0, 0}, {4, 0, 0}, 2};
  Capsule in_y{{2, 3, 4}, {3, 3, 4}, 4};
  Capsule in_n{{2, 3, 4}, {3, 3, 4}, 2.8};
  Capsule over_y{{3, 3, 4}, {5, 3, 4}, 4};
  Capsule over_n{{3, 3, 4}, {5, 3, 4}, 2.8};
  Capsule out_y{{7, 0, 4}, {9, 0, 4}, 4};
  Capsule out_n{{7, 0, 4}, {9, 0, 4}, 2.8};
  ASSERT_TRUE (collides(a, a     ));
  ASSERT_TRUE (collides(a, in_y  ));
  ASSERT_FALSE(collides(a, in_n  ));
  ASSERT_TRUE (collides(a, over_y));
  ASSERT_FALSE(collides(a, over_n));
  ASSERT_TRUE (collides(a, out_y ));
  ASSERT_FALSE(collides(a, out_n ));
  ASSERT_TRUE (collides(in_y  , a));
  ASSERT_FALSE(collides(in_n  , a));
  ASSERT_TRUE (collides(over_y, a));
  ASSERT_FALSE(collides(over_n, a));
  ASSERT_TRUE (collides(out_y , a));
  ASSERT_FALSE(collides(out_n , a));
}

TEST(CollisionTests, capsule_against_capsule) {
  Capsule a{{1, 0, 0}, {5, 0, 0}, 2};
  Capsule b{{2, 1, 1}, {2, 1.1, 1}, .1};
  Capsule c{{0.9, 0, 0}, {0, 0, 0}, 4};
  Capsule d{{-0.1, 0.1, 0}, {-0.1, 1, 0}, 0.2};
  ASSERT_TRUE (collides(a, b));
  ASSERT_TRUE (collides(a, c));
  ASSERT_TRUE (collides(a, d));
  ASSERT_TRUE (collides(b, a));
  ASSERT_TRUE (collides(b, c));
  ASSERT_FALSE(collides(b, d));
  ASSERT_TRUE (collides(c, a));
  ASSERT_TRUE (collides(c, b));
  ASSERT_TRUE (collides(c, d));
  ASSERT_TRUE (collides(d, a));
  ASSERT_FALSE(collides(d, b));
  ASSERT_TRUE (collides(d, c));
}


//
// CapsuleSequence
//

TEST(CollisionTests, capsuleseq_against_point) {
  CapsuleSequence c{{{1, 1, 0}, {1, 1, 1}, {1, 1, 2}}, 2};
  Point in1 {1, -0.8,   1};  // cylinder section
  Point in2 {1,    0,  -1};  // bottom hemisphere section
  Point in3 {1,    2,   3};  // top hemisphere section
  Point out1{1,  3.1,   1};  // out the side
  Point out2{1,    2,  -2};  // out the bottom
  Point out3{1,    1, 4.2};  // out the top
  ASSERT_TRUE (collides(c, in1));
  ASSERT_TRUE (collides(c, in2));
  ASSERT_TRUE (collides(c, in3));
  ASSERT_FALSE(collides(c, out1));
  ASSERT_FALSE(collides(c, out2));
  ASSERT_FALSE(collides(c, out3));
}

TEST(CollisionTests, point_against_capsuleseq) {
  CapsuleSequence c{{{1, 1, 0}, {1, 1, 1}, {1, 1, 2}}, 2};
  Point in1 {1, -0.8,   1};  // cylinder section
  Point in2 {1,    0,  -1};  // bottom hemisphere section
  Point in3 {1,    2,   3};  // top hemisphere section
  Point out1{1,  3.1,   1};  // out the side
  Point out2{1,    2,  -2};  // out the bottom
  Point out3{1,    1, 4.2};  // out the top
  ASSERT_TRUE(collides(in1, c));
  ASSERT_TRUE(collides(in2, c));
  ASSERT_TRUE(collides(in3, c));
  ASSERT_FALSE(collides(out1, c));
  ASSERT_FALSE(collides(out2, c));
  ASSERT_FALSE(collides(out3, c));
}

TEST(CollisionTests, capsuleseq_against_sphere) {
  CapsuleSequence c{{{1, 1, 0}, {1, 1, 1}, {1, 1, 2}}, 1};
  Sphere in1 {{1, -0.8,   1}, 1};  // cylinder section
  Sphere in2 {{1,    0,  -1}, 1};  // bottom hemisphere section
  Sphere in3 {{1,    2,   3}, 1};  // top hemisphere section
  Sphere out1{{1,  3.1,   1}, 1};  // out the side
  Sphere out2{{1,    2,  -2}, 1};  // out the bottom
  Sphere out3{{1,    1, 4.2}, 1};  // out the top
  ASSERT_TRUE(collides(c, in1));
  ASSERT_TRUE(collides(c, in2));
  ASSERT_TRUE(collides(c, in3));
  ASSERT_FALSE(collides(c, out1));
  ASSERT_FALSE(collides(c, out2));
  ASSERT_FALSE(collides(c, out3));
}

TEST(CollisionTests, sphere_against_capsuleseq) {
  CapsuleSequence c{{{1, 1, 0}, {1, 1, 1}, {1, 1, 2}}, 1};
  Sphere in1 {{1, -0.8,   1}, 1};  // cylinder section
  Sphere in2 {{1,    0,  -1}, 1};  // bottom hemisphere section
  Sphere in3 {{1,    2,   3}, 1};  // top hemisphere section
  Sphere out1{{1,  3.1,   1}, 1};  // out the side
  Sphere out2{{1,    2,  -2}, 1};  // out the bottom
  Sphere out3{{1,    1, 4.2}, 1};  // out the top
  ASSERT_TRUE(collides(in1, c));
  ASSERT_TRUE(collides(in2, c));
  ASSERT_TRUE(collides(in3, c));
  ASSERT_FALSE(collides(out1, c));
  ASSERT_FALSE(collides(out2, c));
  ASSERT_FALSE(collides(out3, c));
}

TEST(CollisionTests, capsuleseq_against_capsule_too_close) {
  Point A{1, 0, 0}, B{2, 0, 0}, C{3, 0, 0};
  CapsuleSequence a{{A, A}, 0.1};
  CapsuleSequence b{{B, B, B}, 1.1};
  CapsuleSequence bc{{B, B, C, B}, 0.8};
  ASSERT_TRUE (collides( a,  a));
  ASSERT_TRUE (collides( a,  b));
  ASSERT_FALSE(collides( a, bc));
  ASSERT_TRUE (collides( b,  a));
  ASSERT_TRUE (collides( b,  b));
  ASSERT_TRUE (collides( b, bc));
  ASSERT_FALSE(collides(bc,  a));
  ASSERT_TRUE (collides(bc,  b));
  ASSERT_TRUE (collides(bc, bc));
}

TEST(CollisionTests, capsuleseq_against_capsule_parallel) {
  CapsuleSequence a{{{1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}}, 2};
  Capsule in_y{{2, 3, 4}, {3, 3, 4}, 4};
  Capsule in_n{{2, 3, 4}, {3, 3, 4}, 2.8};
  Capsule over_y{{3, 3, 4}, {5, 3, 4}, 4};
  Capsule over_n{{3, 3, 4}, {5, 3, 4}, 2.8};
  Capsule out_y{{7, 0, 4}, {9, 0, 4}, 4};
  Capsule out_n{{7, 0, 4}, {9, 0, 4}, 2.8};
  ASSERT_TRUE (collides(a, a     ));
  ASSERT_TRUE (collides(a, in_y  ));
  ASSERT_FALSE(collides(a, in_n  ));
  ASSERT_TRUE (collides(a, over_y));
  ASSERT_FALSE(collides(a, over_n));
  ASSERT_TRUE (collides(a, out_y ));
  ASSERT_FALSE(collides(a, out_n ));
  ASSERT_TRUE (collides(in_y  , a));
  ASSERT_FALSE(collides(in_n  , a));
  ASSERT_TRUE (collides(over_y, a));
  ASSERT_FALSE(collides(over_n, a));
  ASSERT_TRUE (collides(out_y , a));
  ASSERT_FALSE(collides(out_n , a));
}

TEST(CollisionTests, capsuleseq_against_capsule) {
  CapsuleSequence sa{{{1, 0, 0}, {3, 0, 0}, {5, 0, 0}}, 2};
  CapsuleSequence sb{{{2, 1, 1}, {2, 1.1, 1}}, .1};
  CapsuleSequence sc{{{0.9, 0, 0}, {0.8, 0, 0}, {0, 0, 0}}, 4};
  CapsuleSequence sd{{{-0.1, 0.1, 0}, {-0.1, 1, 0}}, 0.2};
  Capsule a{{1, 0, 0}, {5, 0, 0}, 2};
  Capsule b{{2, 1, 1}, {2, 1.1, 1}, .1};
  Capsule c{{0.9, 0, 0}, {0, 0, 0}, 4};
  Capsule d{{-0.1, 0.1, 0}, {-0.1, 1, 0}, 0.2};
  ASSERT_TRUE (collides(sa, b));
  ASSERT_TRUE (collides(sa, c));
  ASSERT_TRUE (collides(sa, d));
  ASSERT_TRUE (collides(sb, a));
  ASSERT_TRUE (collides(sb, c));
  ASSERT_FALSE(collides(sb, d));
  ASSERT_TRUE (collides(sc, a));
  ASSERT_TRUE (collides(sc, b));
  ASSERT_TRUE (collides(sc, d));
  ASSERT_TRUE (collides(sd, a));
  ASSERT_FALSE(collides(sd, b));
  ASSERT_TRUE (collides(sd, c));
  ASSERT_TRUE (collides(a, sb));
  ASSERT_TRUE (collides(a, sc));
  ASSERT_TRUE (collides(a, sd));
  ASSERT_TRUE (collides(b, sa));
  ASSERT_TRUE (collides(b, sc));
  ASSERT_FALSE(collides(b, sd));
  ASSERT_TRUE (collides(c, sa));
  ASSERT_TRUE (collides(c, sb));
  ASSERT_TRUE (collides(c, sd));
  ASSERT_TRUE (collides(d, sa));
  ASSERT_FALSE(collides(d, sb));
  ASSERT_TRUE (collides(d, sc));
}

TEST(CollisionTests, capsuleseq_with_no_points) {
  Point p{0, 0, 0};
  Sphere s{p, 5000};
  Capsule c{p, {1, 2, 3}, 5000};
  CapsuleSequence empty{{}, 5000};
  ASSERT_FALSE(collides(empty, p));
  ASSERT_FALSE(collides(empty, s));
  ASSERT_FALSE(collides(empty, c));
  ASSERT_FALSE(collides(p, empty));
  ASSERT_FALSE(collides(s, empty));
  ASSERT_FALSE(collides(c, empty));
}

TEST(CollisionTests, capsuleseq_with_one_point) {
  Point p{0, 0, 0};
  Sphere s{p, 5000};
  Capsule c{p, {1, 2, 3}, 5000};
  CapsuleSequence one{{p}, 5000};
  ASSERT_FALSE(collides(one, p));
  ASSERT_FALSE(collides(one, s));
  ASSERT_FALSE(collides(one, c));
  ASSERT_FALSE(collides(p, one));
  ASSERT_FALSE(collides(s, one));
  ASSERT_FALSE(collides(c, one));
}

TEST(CollisionTests, capsuleseq_against_capsuleseq) {
  CapsuleSequence a{{{1, 0, 0}, {3, 0, 0}, {5, 0, 0}}, 2};
  CapsuleSequence b{{{2, 1, 1}, {2, 1.1, 1}}, .1};
  CapsuleSequence c{{{0.9, 0, 0}, {0.8, 0, 0}, {0, 0, 0}}, 4};
  CapsuleSequence d{{{-0.1, 0.1, 0}, {-0.1, 1, 0}}, 0.2};
  ASSERT_TRUE (collides(a, b));
  ASSERT_TRUE (collides(a, c));
  ASSERT_TRUE (collides(a, d));
  ASSERT_TRUE (collides(b, a));
  ASSERT_TRUE (collides(b, c));
  ASSERT_FALSE(collides(b, d));
  ASSERT_TRUE (collides(c, a));
  ASSERT_TRUE (collides(c, b));
  ASSERT_TRUE (collides(c, d));
  ASSERT_TRUE (collides(d, a));
  ASSERT_FALSE(collides(d, b));
  ASSERT_TRUE (collides(d, c));
}

//
// CapsuleSequence self-collision
//

TEST(SelfCollisionTests, capsuleseq_empty) {
  CapsuleSequence a{};
  ASSERT_FALSE(collides_self(a));
}

TEST(SelfCollisionTests, capsuleseq_short) {
  CapsuleSequence a{{{1, 0, 0}, {3, 0, 0}, {5, 0, 0}}, 2};
  CapsuleSequence b{{{2, 1, 1}, {2, 1.1, 1}}, .1};
  CapsuleSequence c{{{0.9, 0, 0}, {0.8, 0, 0}, {0, 0, 0}}, 4};
  CapsuleSequence d{{{-0.1, 0.1, 0}, {-0.1, 1, 0}}, 0.2};

  ASSERT_FALSE(collides_self(a));
  ASSERT_FALSE(collides_self(b));
  ASSERT_FALSE(collides_self(c));
  ASSERT_FALSE(collides_self(d));
}

TEST(SelfCollisionTests, capsuleseq_straight) {
  CapsuleSequence a{{{1, 0, 0}, {3, 0, 0}, {5, 0, 0}, {7, 0, 0}, {9, 0, 0}}, 2};
  CapsuleSequence b{{{2, 1, 1}, {2, 1.3, 1}, {2, 1.5, 1}, {2, 1.6, 1}, {2, 1.7, 1}}, .1};
  CapsuleSequence c{{{0.9, 0, 0}, {0.8, 0, 0}, {0, 0, 0}, {-5, 0, 0}, {-20, 0, 0}}, 4};
  CapsuleSequence d{{{-0.1, 0.1, 0}, {-0.1, 1, 0}, {-0.1, 8, 0}, {-0.1, 8.1, 0}, {-0.1, 9, 0}}, 0.2};

  ASSERT_FALSE(collides_self(a));
  ASSERT_FALSE(collides_self(b));
  ASSERT_FALSE(collides_self(c));
  ASSERT_FALSE(collides_self(d));
}

TEST(SelfCollisionTests, capsuleseq_double_backs) {
  CapsuleSequence a{{{1, 0, 0}, {3, 0, 0}, {7, 0, 0}, {4, 0, 0}, {1, 0, 0}}, 2};
  CapsuleSequence b{{{2, 1, 1}, {2, 1.3, 1}, {2, 1.5, 1}, {2, 1.3, 1}, {2, 1.1, 1}}, .1};
  CapsuleSequence c{{{0.9, 0, 0}, {0.8, 0, 0}, {0, 0, 0}, {0.5, 0, 0}, {0.6, 0, 0}}, .4};
  CapsuleSequence d{{{-0.1, 0.1, 0}, {-0.1, 1, 0}, {-0.1, .8, 0}, {-0.1, .81, 0}, {-0.1, .9, 0}}, 0.02};

  ASSERT_TRUE(collides_self(a));
  ASSERT_TRUE(collides_self(b));
  ASSERT_TRUE(collides_self(c));
  ASSERT_TRUE(collides_self(d));
}


//
// Mesh
//

// not implemented
//TEST(CollisionTests, mesh_against_point) {
//  ASSERT_TRUE(false);
//}

TEST(CollisionTests, mesh_against_sphere) {
  Sphere a{{1, 2, 3}, 1};
  Sphere b{{0, 1, 2}, 2};
  Sphere c{{0, 0, 0}, 1};
  auto ma = Mesh::from_fcl(a.to_fcl_mesh_model());
  auto mb = Mesh::from_fcl(b.to_fcl_mesh_model());
  auto mc = Mesh::from_fcl(c.to_fcl_mesh_model());
  ASSERT_TRUE (collides(ma, a));
  ASSERT_TRUE (collides(mb, b));
  ASSERT_TRUE (collides(mc, c));
  ASSERT_TRUE (collides(ma, b));
  ASSERT_TRUE (collides(mb, a));
  ASSERT_TRUE (collides(mb, c));
  ASSERT_TRUE (collides(mc, b));
  ASSERT_FALSE(collides(ma, c));
  ASSERT_FALSE(collides(mc, a));
  ASSERT_TRUE (collides(a, ma));
  ASSERT_TRUE (collides(b, mb));
  ASSERT_TRUE (collides(c, mc));
  ASSERT_TRUE (collides(a, mb));
  ASSERT_TRUE (collides(b, ma));
  ASSERT_TRUE (collides(b, mc));
  ASSERT_TRUE (collides(c, mb));
  ASSERT_FALSE(collides(a, mc));
  ASSERT_FALSE(collides(c, ma));
}

TEST(CollisionTests, mesh_against_capsule) {
  Capsule c{{1, 1, 0}, {1, 1, 2}, 1};
  Sphere in1 {{1, -0.8,   1}, 1};  // cylinder section
  Sphere in2 {{1,    0,  -1}, 1};  // bottom hemisphere section
  Sphere in3 {{1,    2,   3}, 1};  // top hemisphere section
  Sphere out1{{1,  3.1,   1}, 1};  // out the side
  Sphere out2{{1,    2,  -2}, 1};  // out the bottom
  Sphere out3{{1,    1, 4.2}, 1};  // out the top
  auto m_in1  = Mesh::from_fcl(in1 .to_fcl_mesh_model());
  auto m_in2  = Mesh::from_fcl(in2 .to_fcl_mesh_model());
  auto m_in3  = Mesh::from_fcl(in3 .to_fcl_mesh_model());
  auto m_out1 = Mesh::from_fcl(out1.to_fcl_mesh_model());
  auto m_out2 = Mesh::from_fcl(out2.to_fcl_mesh_model());
  auto m_out3 = Mesh::from_fcl(out3.to_fcl_mesh_model());
  ASSERT_TRUE (collides(m_in1 , c));
  ASSERT_TRUE (collides(m_in2 , c));
  ASSERT_TRUE (collides(m_in3 , c));
  ASSERT_FALSE(collides(m_out1, c));
  ASSERT_FALSE(collides(m_out2, c));
  ASSERT_FALSE(collides(m_out3, c));
  ASSERT_TRUE (collides(c, m_in1 ));
  ASSERT_TRUE (collides(c, m_in2 ));
  ASSERT_TRUE (collides(c, m_in3 ));
  ASSERT_FALSE(collides(c, m_out1));
  ASSERT_FALSE(collides(c, m_out2));
  ASSERT_FALSE(collides(c, m_out3));
}

// not implemented yet
//TEST(CollisionTests, mesh_against_capsulesequence) {
//  CapsuleSequence c{{{1, 1, 0}, {1, 1, 1}, {1, 1, 2}}, 1};
//  Sphere in1 {{1, -0.8,   1}, 1};  // cylinder section
//  Sphere in2 {{1,    0,  -1}, 1};  // bottom hemisphere section
//  Sphere in3 {{1,    2,   3}, 1};  // top hemisphere section
//  Sphere out1{{1,  3.1,   1}, 1};  // out the side
//  Sphere out2{{1,    2,  -2}, 1};  // out the bottom
//  Sphere out3{{1,    1, 4.2}, 1};  // out the top
//  auto m_in1  = Mesh::from_fcl(in1 .to_fcl_mesh_model());
//  auto m_in2  = Mesh::from_fcl(in2 .to_fcl_mesh_model());
//  auto m_in3  = Mesh::from_fcl(in3 .to_fcl_mesh_model());
//  auto m_out1 = Mesh::from_fcl(out1.to_fcl_mesh_model());
//  auto m_out2 = Mesh::from_fcl(out2.to_fcl_mesh_model());
//  auto m_out3 = Mesh::from_fcl(out3.to_fcl_mesh_model());
//  ASSERT_TRUE (collides(m_in1 , c));
//  ASSERT_TRUE (collides(m_in2 , c));
//  ASSERT_TRUE (collides(m_in3 , c));
//  ASSERT_FALSE(collides(m_out1, c));
//  ASSERT_FALSE(collides(m_out2, c));
//  ASSERT_FALSE(collides(m_out3, c));
//  ASSERT_TRUE (collides(c, m_in1 ));
//  ASSERT_TRUE (collides(c, m_in2 ));
//  ASSERT_TRUE (collides(c, m_in3 ));
//  ASSERT_FALSE(collides(c, m_out1));
//  ASSERT_FALSE(collides(c, m_out2));
//  ASSERT_FALSE(collides(c, m_out3));
//}

TEST(CollisionTests, mesh_against_mesh) {
  Sphere a{{1, 2, 3}, 1};
  Sphere b{{0, 1, 2}, 2};
  Sphere c{{0, 0, 0}, 1};
  auto ma = Mesh::from_fcl(a.to_fcl_mesh_model());
  auto mb = Mesh::from_fcl(b.to_fcl_mesh_model());
  auto mc = Mesh::from_fcl(c.to_fcl_mesh_model());
  ASSERT_TRUE (collides(ma, ma));
  ASSERT_TRUE (collides(mb, mb));
  ASSERT_TRUE (collides(mc, mc));
  ASSERT_TRUE (collides(ma, mb));
  ASSERT_TRUE (collides(mb, ma));
  ASSERT_TRUE (collides(mb, mc));
  ASSERT_TRUE (collides(mc, mb));
  ASSERT_FALSE(collides(ma, mc));
  ASSERT_FALSE(collides(mc, ma));
}

TEST(CollisionTests, mesh_empty) {
  Point p{0, 0, 0};
  Sphere s{p, 5000};
  Capsule c{p, {1, 2, 3}, 5000};
  CapsuleSequence seq{{p, {1, 2, 3}, {3, 4, 5}}, 5000};
  auto m = Mesh::from_fcl(Sphere{p, 1}.to_fcl_mesh_model());
  Mesh empty;
  //ASSERT_FALSE(collides(empty, p  )); // TODO: not yet implemented
  ASSERT_FALSE(collides(empty, s  ));
  ASSERT_FALSE(collides(empty, c  ));
  //ASSERT_FALSE(collides(empty, seq)); // TODO: not yet implemented
  ASSERT_FALSE(collides(empty, m  ));
  //ASSERT_FALSE(collides(p,   empty)); // TODO: not yet implemented
  ASSERT_FALSE(collides(s,   empty));
  ASSERT_FALSE(collides(c,   empty));
  //ASSERT_FALSE(collides(seq, empty)); // TODO: not yet implemented
  ASSERT_FALSE(collides(m,   empty));
}

// not implemented yet
//TEST(CollisionTests, mesh_triangles_out_of_bounds) {
//  ASSERT_TRUE(false);
//}

// // not implemented yet
// TEST(CollisionTests, mesh_only_points) {
//   Capsule c{{1, 1, 0}, {1, 1, 2}, 2};
//   Point in1 {1, -0.8,   1};  // cylinder section
//   Point in2 {1,    0,  -1};  // bottom hemisphere section
//   Point in3 {1,    2,   3};  // top hemisphere section
//   Point out1{1,  3.1,   1};  // out the side
//   Point out2{1,    2,  -2};  // out the bottom
//   Point out3{1,    1, 4.2};  // out the top
//   auto add_vertex = [](Mesh &m, const Point &p) {
//     m.vertices.emplace_back(p[0], p[1], p[2]);
//   };
//   Mesh m1; add_vertex(m1, in1);
//   Mesh m2; add_vertex(m2, in2);
//   Mesh m3; add_vertex(m3, in3);
//   Mesh m4; add_vertex(m4, out1);
//   Mesh m5; add_vertex(m5, out2);
//   Mesh m6; add_vertex(m6, out3);
//   ASSERT_TRUE (collides(m1, m1));
//   ASSERT_TRUE (collides(m2, m2));
//   ASSERT_TRUE (collides(m3, m3));
//   ASSERT_TRUE (collides(m4, m4));
//   ASSERT_TRUE (collides(m5, m5));
//   ASSERT_TRUE (collides(m6, m6));
//   ASSERT_TRUE (collides(m1, c));
//   ASSERT_TRUE (collides(m2, c));
//   ASSERT_TRUE (collides(m3, c));
//   ASSERT_FALSE(collides(m4, c));
//   ASSERT_FALSE(collides(m5, c));
//   ASSERT_FALSE(collides(m6, c));
//   ASSERT_TRUE (collides(c, m1));
//   ASSERT_TRUE (collides(c, m2));
//   ASSERT_TRUE (collides(c, m3));
//   ASSERT_FALSE(collides(c, m4));
//   ASSERT_FALSE(collides(c, m5));
//   ASSERT_FALSE(collides(c, m6));
//
//   Mesh m7;
//   add_vertex(m7, in1);
//   add_vertex(m7, in2);
//   add_vertex(m7, in3);
//   ASSERT_TRUE (collides(m7, m7));
//   ASSERT_TRUE (collides(m7, c));
//   ASSERT_TRUE (collides(c, m7));
//   ASSERT_TRUE (collides(m7, m1));
//   ASSERT_TRUE (collides(m7, m2));
//   ASSERT_TRUE (collides(m7, m3));
//   ASSERT_FALSE(collides(m7, m4));
//   ASSERT_FALSE(collides(m7, m5));
//   ASSERT_FALSE(collides(m7, m6));
//   ASSERT_TRUE (collides(m1, m7));
//   ASSERT_TRUE (collides(m2, m7));
//   ASSERT_TRUE (collides(m3, m7));
//   ASSERT_FALSE(collides(m4, m7));
//   ASSERT_FALSE(collides(m5, m7));
//   ASSERT_FALSE(collides(m6, m7));
//
//   Mesh m8;
//   add_vertex(m8, out1);
//   add_vertex(m8, out2);
//   add_vertex(m8, out3);
//   ASSERT_TRUE (collides(m8, m8));
//   ASSERT_FALSE(collides(m8, c));
//   ASSERT_FALSE(collides(c, m8));
//   ASSERT_FALSE(collides(m8, m1));
//   ASSERT_FALSE(collides(m8, m2));
//   ASSERT_FALSE(collides(m8, m3));
//   ASSERT_TRUE (collides(m8, m4));
//   ASSERT_TRUE (collides(m8, m5));
//   ASSERT_TRUE (collides(m8, m6));
//   ASSERT_FALSE(collides(m8, m7));
//   ASSERT_FALSE(collides(m1, m8));
//   ASSERT_FALSE(collides(m2, m8));
//   ASSERT_FALSE(collides(m3, m8));
//   ASSERT_TRUE (collides(m4, m8));
//   ASSERT_TRUE (collides(m5, m8));
//   ASSERT_TRUE (collides(m6, m8));
//   ASSERT_FALSE(collides(m7, m8));
//
//   Mesh m9;
//   add_vertex(m9, in1);
//   add_vertex(m9, in2);
//   add_vertex(m9, in3);
//   add_vertex(m9, out1);
//   add_vertex(m9, out2);
//   add_vertex(m9, out3);
//   ASSERT_TRUE (collides(m9, m9));
//   ASSERT_TRUE (collides(m9, c));
//   ASSERT_TRUE (collides(c, m9));
//   ASSERT_TRUE (collides(m9, m1));
//   ASSERT_TRUE (collides(m9, m2));
//   ASSERT_TRUE (collides(m9, m3));
//   ASSERT_TRUE (collides(m9, m4));
//   ASSERT_TRUE (collides(m9, m5));
//   ASSERT_TRUE (collides(m9, m6));
//   ASSERT_TRUE (collides(m9, m7));
//   ASSERT_TRUE (collides(m9, m8));
//   ASSERT_TRUE (collides(m1, m9));
//   ASSERT_TRUE (collides(m2, m9));
//   ASSERT_TRUE (collides(m3, m9));
//   ASSERT_TRUE (collides(m4, m9));
//   ASSERT_TRUE (collides(m5, m9));
//   ASSERT_TRUE (collides(m6, m9));
//   ASSERT_TRUE (collides(m7, m9));
//   ASSERT_TRUE (collides(m8, m9));
// }


//
// FCL
//

TEST(CollisionTests, collisionobj_against_collisionobj) {
  Sphere a{{1, 2, 3}, 1};
  Sphere b{{0, 1, 2}, 2};
  Sphere c{{0, 0, 0}, 1};
  auto aobj = a.to_fcl_object();
  auto bobj = b.to_fcl_object();
  auto cobj = c.to_fcl_object();
  ASSERT_TRUE (collides(aobj, aobj));
  ASSERT_TRUE (collides(bobj, bobj));
  ASSERT_TRUE (collides(cobj, cobj));
  ASSERT_TRUE (collides(aobj, bobj));
  ASSERT_TRUE (collides(bobj, aobj));
  ASSERT_TRUE (collides(bobj, cobj));
  ASSERT_TRUE (collides(cobj, bobj));
  ASSERT_FALSE(collides(aobj, cobj));
  ASSERT_FALSE(collides(cobj, aobj));
}

TEST(CollisionTests, collisionmanager_against_collisionobj) {
  Sphere a{{1, 2, 3}, 1};
  Sphere b{{0, 1, 2}, 2};
  Sphere c{{0, 0, 0}, 1};
  auto aobj = a.to_fcl_object();
  auto bobj = b.to_fcl_object();
  auto cobj = c.to_fcl_object();

  auto ma = std::make_shared<CollisionManager>();
  ma->registerObject(aobj.get());
  ma->setup();
  auto mb = std::make_shared<CollisionManager>();
  mb->registerObject(bobj.get());
  mb->setup();
  auto mc = std::make_shared<CollisionManager>();
  mc->registerObject(cobj.get());
  mc->setup();

  ASSERT_TRUE (collides(ma, aobj));
  ASSERT_TRUE (collides(mb, bobj));
  ASSERT_TRUE (collides(mc, cobj));
  ASSERT_TRUE (collides(ma, bobj));
  ASSERT_TRUE (collides(mb, aobj));
  ASSERT_TRUE (collides(mb, cobj));
  ASSERT_TRUE (collides(mc, bobj));
  ASSERT_FALSE(collides(ma, cobj));
  ASSERT_FALSE(collides(mc, aobj));
  ASSERT_TRUE (collides(aobj, ma));
  ASSERT_TRUE (collides(bobj, mb));
  ASSERT_TRUE (collides(cobj, mc));
  ASSERT_TRUE (collides(bobj, ma));
  ASSERT_TRUE (collides(aobj, mb));
  ASSERT_TRUE (collides(cobj, mb));
  ASSERT_TRUE (collides(bobj, mc));
  ASSERT_FALSE(collides(cobj, ma));
  ASSERT_FALSE(collides(aobj, mc));

  auto mab = std::make_shared<CollisionManager>();
  mab->registerObject(aobj.get());
  mab->registerObject(bobj.get());
  mab->setup();

  ASSERT_TRUE (collides(mab, aobj));
  ASSERT_TRUE (collides(mab, bobj));
  ASSERT_TRUE (collides(mab, cobj));
  ASSERT_TRUE (collides(aobj, mab));
  ASSERT_TRUE (collides(bobj, mab));
  ASSERT_TRUE (collides(cobj, mab));

  auto mac = std::make_shared<CollisionManager>();
  mac->registerObject(aobj.get());
  mac->registerObject(cobj.get());
  mac->setup();

  ASSERT_TRUE (collides(mac, aobj));
  ASSERT_TRUE (collides(mac, bobj));
  ASSERT_TRUE (collides(mac, cobj));
  ASSERT_TRUE (collides(aobj, mac));
  ASSERT_TRUE (collides(bobj, mac));
  ASSERT_TRUE (collides(cobj, mac));

  auto mabc = std::make_shared<CollisionManager>();
  mabc->registerObject(aobj.get());
  mabc->registerObject(bobj.get());
  mabc->registerObject(cobj.get());
  mabc->setup();

  ASSERT_TRUE (collides(mabc, aobj));
  ASSERT_TRUE (collides(mabc, bobj));
  ASSERT_TRUE (collides(mabc, cobj));
  ASSERT_TRUE (collides(aobj, mabc));
  ASSERT_TRUE (collides(bobj, mabc));
  ASSERT_TRUE (collides(cobj, mabc));
}

TEST(CollisionTests, collisionmanager_against_collisionmanager) {
  Sphere a{{1, 2, 3}, 1};
  Sphere b{{0, 1, 2}, 2};
  Sphere c{{0, 0, 0}, 1};
  auto aobj = a.to_fcl_object();
  auto bobj = b.to_fcl_object();
  auto cobj = c.to_fcl_object();
  // all collide except for a and c

  auto ma = std::make_shared<CollisionManager>();
  ma->registerObject(aobj.get());
  ma->setup();
  auto mb = std::make_shared<CollisionManager>();
  mb->registerObject(bobj.get());
  mb->setup();
  auto mc = std::make_shared<CollisionManager>();
  mc->registerObject(cobj.get());
  mc->setup();
  auto mab = std::make_shared<CollisionManager>();
  mab->registerObject(aobj.get());
  mab->registerObject(bobj.get());
  mab->setup();
  auto mac = std::make_shared<CollisionManager>();
  mac->registerObject(aobj.get());
  mac->registerObject(cobj.get());
  mac->setup();
  auto mbc = std::make_shared<CollisionManager>();
  mbc->registerObject(bobj.get());
  mbc->registerObject(cobj.get());
  mbc->setup();
  auto mabc = std::make_shared<CollisionManager>();
  mabc->registerObject(aobj.get());
  mabc->registerObject(bobj.get());
  mabc->registerObject(cobj.get());
  mabc->setup();

  ASSERT_TRUE (collides(ma  , ma  ));
  ASSERT_TRUE (collides(ma  , mb  ));
  ASSERT_FALSE(collides(ma  , mc  ));
  ASSERT_TRUE (collides(ma  , mab ));
  ASSERT_TRUE (collides(ma  , mac ));
  ASSERT_TRUE (collides(ma  , mbc ));
  ASSERT_TRUE (collides(ma  , mabc));

  ASSERT_TRUE (collides(mb  , ma  ));
  ASSERT_TRUE (collides(mb  , mb  ));
  ASSERT_TRUE (collides(mb  , mc  ));
  ASSERT_TRUE (collides(mb  , mab ));
  ASSERT_TRUE (collides(mb  , mac ));
  ASSERT_TRUE (collides(mb  , mbc ));
  ASSERT_TRUE (collides(mb  , mabc));

  ASSERT_FALSE(collides(mc  , ma  ));
  ASSERT_TRUE (collides(mc  , mb  ));
  ASSERT_TRUE (collides(mc  , mc  ));
  ASSERT_TRUE (collides(mc  , mab ));
  ASSERT_TRUE (collides(mc  , mac ));
  ASSERT_TRUE (collides(mc  , mbc ));
  ASSERT_TRUE (collides(mc  , mabc));

  ASSERT_TRUE (collides(mab , ma  ));
  ASSERT_TRUE (collides(mab , mb  ));
  ASSERT_TRUE (collides(mab , mc  ));
  ASSERT_TRUE (collides(mab , mab ));
  ASSERT_TRUE (collides(mab , mac ));
  ASSERT_TRUE (collides(mab , mbc ));
  ASSERT_TRUE (collides(mab , mabc));

  ASSERT_TRUE (collides(mac , ma  ));
  ASSERT_TRUE (collides(mac , mb  ));
  ASSERT_TRUE (collides(mac , mc  ));
  ASSERT_TRUE (collides(mac , mab ));
  ASSERT_TRUE (collides(mac , mac ));
  ASSERT_TRUE (collides(mac , mbc ));
  ASSERT_TRUE (collides(mac , mabc));

  ASSERT_TRUE (collides(mbc , ma  ));
  ASSERT_TRUE (collides(mbc , mb  ));
  ASSERT_TRUE (collides(mbc , mc  ));
  ASSERT_TRUE (collides(mbc , mab ));
  ASSERT_TRUE (collides(mbc , mac ));
  ASSERT_TRUE (collides(mbc , mbc ));
  ASSERT_TRUE (collides(mbc , mabc));

  ASSERT_TRUE (collides(mabc, ma  ));
  ASSERT_TRUE (collides(mabc, mb  ));
  ASSERT_TRUE (collides(mabc, mc  ));
  ASSERT_TRUE (collides(mabc, mab ));
  ASSERT_TRUE (collides(mabc, mac ));
  ASSERT_TRUE (collides(mabc, mbc ));
  ASSERT_TRUE (collides(mabc, mabc));
}

TEST(CollisionTests, collisionmanager_empty) {
  Sphere a{{1, 2, 3}, 1};
  Sphere b{{0, 1, 2}, 2};
  Sphere c{{0, 0, 0}, 1};
  auto aobj = a.to_fcl_object();
  auto bobj = b.to_fcl_object();
  auto cobj = c.to_fcl_object();

  auto empty = std::make_shared<CollisionManager>();

  ASSERT_FALSE(collides(aobj, empty));
  ASSERT_FALSE(collides(bobj, empty));
  ASSERT_FALSE(collides(cobj, empty));
  ASSERT_FALSE(collides(empty, aobj));
  ASSERT_FALSE(collides(empty, bobj));
  ASSERT_FALSE(collides(empty, cobj));
}

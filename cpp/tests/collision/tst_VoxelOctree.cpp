/**
 * author:         Michael Bentley
 * email:          mikebentley15@gmail.com
 * date-created:   28 September 2020
 */

#include "collision/VoxelOctree.h"

#include "collision/Sphere.h"

#include <itkImage.h>

#include <3rdparty/nlohmann/json.hpp>

#include <gtest/gtest.h>

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

using collision::Sphere;
using collision::VoxelOctree;

namespace {

std::ostream &print_slice(std::ostream &out, const VoxelOctree &v, size_t ix) {
  for (size_t iy = 0; iy < v.Ny(); iy++) {
    for (size_t iz = 0; iz < v.Nz(); iz++) {
      out << (v.cell(ix, iy, iz) ? '1' : '.');
    }
    out << std::endl;
  }
  return out;
}

std::string slice_to_string (const VoxelOctree &v, size_t ix) {
  std::ostringstream ss;
  print_slice(ss, v, ix);
  return ss.str();
};

std::ostream& operator<< (std::ostream &out, const VoxelOctree &v) {
  out << "VoxelOctree(" << v.Nx() << ")\n"
      << "  xlim:     [" << v.xlim().first << ", " << v.xlim().second << "]\n"
      << "  ylim:     [" << v.ylim().first << ", " << v.ylim().second << "]\n"
      << "  zlim:     [" << v.zlim().first << ", " << v.zlim().second << "]\n"
      << "  spacing:  [" << v.dx() << ", " << v.dy() << ", " << v.dz() << "]\n"
      << "\n";
  for (size_t ix = 0; ix < v.Nx(); ix++) {
     out << "Slice " << ix << "\n";
     print_slice(out, v, ix);
     out << "\n";
  }
  return out;
}

class FileRemover {
  const std::string _fname;
public:
  FileRemover(const std::string &fname) : _fname(fname) {}
  ~FileRemover() { std::remove(_fname.c_str()); }
  std::string fname() const { return _fname; }
};

bool file_exists(const std::string &fname) {
    std::ifstream infile(fname);
    return infile.good();
}

} // end of unnamed namespace

TEST(VoxelOctreeTests, constructor_defaults) {
  const size_t N = 32;
  VoxelOctree v(N);
  ASSERT_EQ(v.Nx(), N);
  ASSERT_EQ(v.Ny(), N);
  ASSERT_EQ(v.Nz(), N);
  ASSERT_EQ(v.N(), N*N*N);
  ASSERT_EQ(v.Nbx(), N/4);
  ASSERT_EQ(v.Nby(), N/4);
  ASSERT_EQ(v.Nbz(), N/4);
  ASSERT_EQ(v.Nb(), N*N*N/64);
  ASSERT_DOUBLE_EQ(v.xlim().first,  0.0);
  ASSERT_DOUBLE_EQ(v.xlim().second, 1.0);
  ASSERT_DOUBLE_EQ(v.ylim().first,  0.0);
  ASSERT_DOUBLE_EQ(v.ylim().second, 1.0);
  ASSERT_DOUBLE_EQ(v.zlim().first,  0.0);
  ASSERT_DOUBLE_EQ(v.zlim().second, 1.0);
  ASSERT_DOUBLE_EQ(v.dx(), 1.0/N);
  ASSERT_DOUBLE_EQ(v.dy(), 1.0/N);
  ASSERT_DOUBLE_EQ(v.dz(), 1.0/N);

  ASSERT_EQ(v.nblocks(), size_t(0));
  for (size_t x = 0; x < v.Nx(); x++) {
    for (size_t y = 0; y < v.Ny(); y++) {
      for (size_t z = 0; z < v.Nz(); z++) {
        ASSERT_FALSE(v.cell(x, y, z));
      }
    }
  }
  for (size_t x = 0; x < v.Nbx(); x++) {
    for (size_t y = 0; y < v.Nby(); y++) {
      for (size_t z = 0; z < v.Nbz(); z++) {
        ASSERT_EQ(v.block(x, y, z), size_t(0));
      }
    }
  }
  ASSERT_FALSE(v.collides(v));
}

TEST(VoxelOctreeTests, xlim_good) {
  const size_t N = 32;
  VoxelOctree v(N);
  v.set_xlim(0.1, 5.2);
  ASSERT_DOUBLE_EQ(v.xlim().first, 0.1);
  ASSERT_DOUBLE_EQ(v.xlim().second, 5.2);
  ASSERT_DOUBLE_EQ(v.dx(), (5.2 - 0.1) / N);
  ASSERT_DOUBLE_EQ(v.dbx(), 4 * (5.2 - 0.1) / N);
  ASSERT_DOUBLE_EQ(v.ylim().first, 0.0);
  ASSERT_DOUBLE_EQ(v.ylim().second, 1.0);
  ASSERT_DOUBLE_EQ(v.dy(), 1.0 / N);
  ASSERT_DOUBLE_EQ(v.dby(), 4.0 / N);
  ASSERT_DOUBLE_EQ(v.zlim().first, 0.0);
  ASSERT_DOUBLE_EQ(v.zlim().second, 1.0);
  ASSERT_DOUBLE_EQ(v.dz(), 1.0 / N);
  ASSERT_DOUBLE_EQ(v.dbz(), 4.0 / N);
}

TEST(VoxelOctreeTests, xlim_inf) {
  const size_t N = 32;
  VoxelOctree v(N);
  // don't use infinity in real situations
  v.set_xlim(0.0, std::numeric_limits<double>::infinity());
  ASSERT_DOUBLE_EQ(v.xlim().first, 0.0);
  ASSERT_DOUBLE_EQ(v.xlim().second, std::numeric_limits<double>::infinity());
  ASSERT_DOUBLE_EQ(v.dx(), std::numeric_limits<double>::infinity());
  ASSERT_DOUBLE_EQ(v.dbx(), std::numeric_limits<double>::infinity());
}

TEST(VoxelOctreeTests, xlim_negative_range) {
  const size_t N = 32;
  VoxelOctree v(N);
  ASSERT_THROW(v.set_xlim(1.0, 0.0), std::length_error);
}

TEST(VoxelOctreeTests, xlim_equal) {
  const size_t N = 32;
  VoxelOctree v(N);
  ASSERT_THROW(v.set_xlim(0.1, 0.1), std::length_error);
}

TEST(VoxelOctreeTests, ylim_good) {
  const size_t N = 32;
  VoxelOctree v(N);
  v.set_ylim(0.1, 5.2);
  ASSERT_DOUBLE_EQ(v.ylim().first, 0.1);
  ASSERT_DOUBLE_EQ(v.ylim().second, 5.2);
  ASSERT_DOUBLE_EQ(v.dy(), (5.2 - 0.1) / N);
  ASSERT_DOUBLE_EQ(v.dby(), 4 * (5.2 - 0.1) / N);
  ASSERT_DOUBLE_EQ(v.xlim().first, 0.0);
  ASSERT_DOUBLE_EQ(v.xlim().second, 1.0);
  ASSERT_DOUBLE_EQ(v.dx(), 1.0 / N);
  ASSERT_DOUBLE_EQ(v.dbx(), 4.0 / N);
  ASSERT_DOUBLE_EQ(v.zlim().first, 0.0);
  ASSERT_DOUBLE_EQ(v.zlim().second, 1.0);
  ASSERT_DOUBLE_EQ(v.dz(), 1.0 / N);
  ASSERT_DOUBLE_EQ(v.dbz(), 4.0 / N);
}

TEST(VoxelOctreeTests, ylim_inf) {
  const size_t N = 32;
  VoxelOctree v(N);
  // don't use infinity in real situations
  v.set_ylim(0.0, std::numeric_limits<double>::infinity());
  ASSERT_DOUBLE_EQ(v.ylim().first, 0.0);
  ASSERT_DOUBLE_EQ(v.ylim().second, std::numeric_limits<double>::infinity());
  ASSERT_DOUBLE_EQ(v.dy(), std::numeric_limits<double>::infinity());
  ASSERT_DOUBLE_EQ(v.dby(), std::numeric_limits<double>::infinity());
}

TEST(VoxelOctreeTests, ylim_negative_range) {
  const size_t N = 32;
  VoxelOctree v(N);
  ASSERT_THROW(v.set_ylim(1.0, 0.0), std::length_error);
}

TEST(VoxelOctreeTests, ylim_equal) {
  const size_t N = 32;
  VoxelOctree v(N);
  ASSERT_THROW(v.set_ylim(0.1, 0.1), std::length_error);
}

TEST(VoxelOctreeTests, zlim_good) {
  const size_t N = 32;
  VoxelOctree v(N);
  v.set_zlim(0.1, 5.2);
  ASSERT_DOUBLE_EQ(v.zlim().first, 0.1);
  ASSERT_DOUBLE_EQ(v.zlim().second, 5.2);
  ASSERT_DOUBLE_EQ(v.dz(), (5.2 - 0.1) / N);
  ASSERT_DOUBLE_EQ(v.dbz(), 4 * (5.2 - 0.1) / N);
  ASSERT_DOUBLE_EQ(v.ylim().first, 0.0);
  ASSERT_DOUBLE_EQ(v.ylim().second, 1.0);
  ASSERT_DOUBLE_EQ(v.dy(), 1.0 / N);
  ASSERT_DOUBLE_EQ(v.dby(), 4.0 / N);
  ASSERT_DOUBLE_EQ(v.xlim().first, 0.0);
  ASSERT_DOUBLE_EQ(v.xlim().second, 1.0);
  ASSERT_DOUBLE_EQ(v.dx(), 1.0 / N);
  ASSERT_DOUBLE_EQ(v.dbx(), 4.0 / N);
}

TEST(VoxelOctreeTests, zlim_inf) {
  const size_t N = 32;
  VoxelOctree v(N);
  // don't use infinity in real situations
  v.set_zlim(0.0, std::numeric_limits<double>::infinity());
  ASSERT_DOUBLE_EQ(v.zlim().first, 0.0);
  ASSERT_DOUBLE_EQ(v.zlim().second, std::numeric_limits<double>::infinity());
  ASSERT_DOUBLE_EQ(v.dz(), std::numeric_limits<double>::infinity());
  ASSERT_DOUBLE_EQ(v.dbz(), std::numeric_limits<double>::infinity());
}

TEST(VoxelOctreeTests, zlim_negative_range) {
  const size_t N = 32;
  VoxelOctree v(N);
  ASSERT_THROW(v.set_zlim(1.0, 0.0), std::length_error);
}

TEST(VoxelOctreeTests, zlim_equal) {
  const size_t N = 32;
  VoxelOctree v(N);
  ASSERT_THROW(v.set_zlim(0.1, 0.1), std::length_error);
}

TEST(VoxelOctreeTests, set_block_returns_correctly) {
  const size_t N = 32;
  VoxelOctree v(N);

  uint64_t b = size_t(0xf33ff33feaeaeaea);
  v.set_block(1, 2, 3, b);
  ASSERT_EQ(b , v.block(1, 2, 3));
  ASSERT_EQ(size_t(1), v.nblocks());

  uint64_t b2 = size_t(154);
  v.set_block(4, 5, 6, b2);
  ASSERT_EQ(b , v.block(1, 2, 3));
  ASSERT_EQ(b2, v.block(4, 5, 6));
  ASSERT_EQ(size_t(2), v.nblocks());

  for (size_t x = 0; x < v.Nbx(); x++) {
    for (size_t y = 0; y < v.Nby(); y++) {
      for (size_t z = 0; z < v.Nbz(); z++) {
        if (x == 1 && y == 2 && z == 3) {
          ASSERT_EQ(b , v.block(x, y, z));
        } else if (x == 4 && y == 5 && z == 6) {
          ASSERT_EQ(b2, v.block(x, y, z));
        } else {
          ASSERT_EQ(size_t(0), v.block(x, y, z));
        }
      }
    }
  }
}

TEST(VoxelOctreeTests, set_block_can_retrieve) {
  const size_t N = 32;
  VoxelOctree v(N);
  size_t b(0x5445'facb'3333'1234);
  v.set_block(5, 6, 7, b);
  ASSERT_EQ(b, v.block(5, 6, 7));
  ASSERT_EQ(size_t(1), v.nblocks());
}

TEST(VoxelOctreeTests, set_block_set_zero) {
  const size_t N = 32;
  VoxelOctree v(N);
  size_t b(0x5445'facb'3333'1234);
  v.set_block(5, 6, 7, b);
  ASSERT_EQ(b, v.block(5, 6, 7));
  ASSERT_EQ(size_t(1), v.nblocks());
  v.set_block(5, 6, 7, 0);
  ASSERT_EQ(size_t(0), v.block(5, 6, 7));
  ASSERT_EQ(size_t(0), v.nblocks());
}

TEST(VoxelOctreeTests, union_block_same_value) {
  const size_t N = 32;
  VoxelOctree v(N);
  size_t b(0x5445'facb'3333'1234);
  v.set_block(5, 6, 7, b);
  ASSERT_EQ(b, v.block(5, 6, 7));
  ASSERT_EQ(size_t(1), v.nblocks());
  ASSERT_EQ(b, v.union_block(5, 6, 7, b));
  ASSERT_EQ(b, v.block(5, 6, 7));
  ASSERT_EQ(size_t(1), v.nblocks());
}

TEST(VoxelOctreeTests, union_block_against_zero) {
  const size_t N = 32;
  VoxelOctree v(N);
  size_t b(0x5445'facb'3333'1234);
  ASSERT_EQ(size_t(0), v.union_block(5, 6, 7, b));
  ASSERT_EQ(b, v.block(5, 6, 7));
  ASSERT_EQ(size_t(1), v.nblocks());
}

TEST(VoxelOctreeTests, union_block_with_zero) {
  const size_t N = 32;
  VoxelOctree v(N);
  size_t b(0x5445'facb'3333'1234);
  v.set_block(5, 6, 7, b);
  ASSERT_EQ(b, v.block(5, 6, 7));
  ASSERT_EQ(size_t(1), v.nblocks());
  ASSERT_EQ(b, v.union_block(5, 6, 7, 0));
  ASSERT_EQ(b, v.block(5, 6, 7));
  ASSERT_EQ(size_t(1), v.nblocks());
}

TEST(VoxelOctreeTests, union_block_with_and_against_zero) {
  const size_t N = 32;
  VoxelOctree v(N);
  ASSERT_EQ(size_t(0), v.union_block(5, 6, 7, 0));
  ASSERT_EQ(size_t(0), v.block(5, 6, 7));
  ASSERT_EQ(size_t(0), v.nblocks());
}

TEST(VoxelOctreeTests, union_block_without_overlap) {
  const size_t N = 32;
  VoxelOctree v(N);
  size_t b1(0x5445'facb'3333'1234);
  size_t b2(0x2112'0000'4840'2141);
  size_t expected_union = b1 | b2;
  v.set_block(5, 6, 7, b1);
  ASSERT_EQ(b1, v.block(5, 6, 7));
  ASSERT_EQ(size_t(1), v.nblocks());
  ASSERT_EQ(b1, v.union_block(5, 6, 7, b2));
  ASSERT_EQ(expected_union, v.block(5, 6, 7));
  ASSERT_EQ(size_t(1), v.nblocks());
}

TEST(VoxelOctreeTests, union_block_with_overlap) {
  const size_t N = 32;
  VoxelOctree v(N);
  size_t b1(0x5445'facb'3333'1234);
  size_t b2(0x7112'ff00'484f'2141);
  size_t expected_union = b1 | b2;
  v.set_block(5, 6, 7, b1);
  ASSERT_EQ(b1, v.block(5, 6, 7));
  ASSERT_EQ(size_t(1), v.nblocks());
  ASSERT_EQ(b1, v.union_block(5, 6, 7, b2));
  ASSERT_EQ(expected_union, v.block(5, 6, 7));
  ASSERT_EQ(size_t(1), v.nblocks());
}

TEST(VoxelOctreeTests, intersect_block_same_value) {
  const size_t N = 32;
  VoxelOctree v(N);
  size_t b(0x5445'facb'3333'1234);
  v.set_block(5, 6, 7, b);
  ASSERT_EQ(b, v.block(5, 6, 7));
  ASSERT_EQ(size_t(1), v.nblocks());
  ASSERT_EQ(b, v.intersect_block(5, 6, 7, b));
  ASSERT_EQ(b, v.block(5, 6, 7));
  ASSERT_EQ(size_t(1), v.nblocks());
}

TEST(VoxelOctreeTests, intersect_block_against_zero) {
  const size_t N = 32;
  VoxelOctree v(N);
  size_t b(0x5445'facb'3333'1234);
  ASSERT_EQ(size_t(0), v.intersect_block(5, 6, 7, b));
  ASSERT_EQ(size_t(0), v.block(5, 6, 7));
  ASSERT_EQ(size_t(0), v.nblocks());
}

TEST(VoxelOctreeTests, intersect_block_with_zero) {
  const size_t N = 32;
  VoxelOctree v(N);
  size_t b(0x5445'facb'3333'1234);
  v.set_block(5, 6, 7, b);
  ASSERT_EQ(b, v.block(5, 6, 7));
  ASSERT_EQ(size_t(1), v.nblocks());
  ASSERT_EQ(b, v.intersect_block(5, 6, 7, 0));
  ASSERT_EQ(size_t(0), v.block(5, 6, 7));
  ASSERT_EQ(size_t(0), v.nblocks());
}

TEST(VoxelOctreeTests, intersect_block_with_and_against_zero) {
  const size_t N = 32;
  VoxelOctree v(N);
  ASSERT_EQ(size_t(0), v.intersect_block(5, 6, 7, 0));
  ASSERT_EQ(size_t(0), v.block(5, 6, 7));
  ASSERT_EQ(size_t(0), v.nblocks());
}

TEST(VoxelOctreeTests, intersect_block_without_overlap) {
  const size_t N = 32;
  VoxelOctree v(N);
  size_t b1(0x5445'facb'3333'1234);
  size_t b2(0x2112'0000'4840'2141);
  v.set_block(5, 6, 7, b1);
  ASSERT_EQ(b1, v.block(5, 6, 7));
  ASSERT_EQ(size_t(1), v.nblocks());
  ASSERT_EQ(b1, v.intersect_block(5, 6, 7, b2));
  ASSERT_EQ(size_t(0), v.block(5, 6, 7));
  ASSERT_EQ(size_t(0), v.nblocks());
}

TEST(VoxelOctreeTests, intersect_block_with_overlap) {
  const size_t N = 32;
  VoxelOctree v(N);
  size_t b1(0x5445'facb'3333'1234);
  size_t b2(0x7112'ff00'484f'2141);
  size_t expected_intersect = b1 & b2;
  v.set_block(5, 6, 7, b1);
  ASSERT_EQ(b1, v.block(5, 6, 7));
  ASSERT_EQ(size_t(1), v.nblocks());
  ASSERT_EQ(b1, v.intersect_block(5, 6, 7, b2));
  ASSERT_EQ(expected_intersect, v.block(5, 6, 7));
  ASSERT_EQ(size_t(1), v.nblocks());
}

TEST(VoxelOctreeTests, DISABLED_set_cell_more_tests) {
  ASSERT_TRUE(false) << "Need more tests";
}

TEST(VoxelOctreeTests, DISABLED_find_block_more_tests) {
  ASSERT_TRUE(false) << "Need more tests";
}

TEST(VoxelOctreeTests, DISABLED_nearest_block_idx_more_tests) {
  ASSERT_TRUE(false) << "Need more tests";
}

TEST(VoxelOctreeTests, DISABLED_find_block_idx_more_tests) {
  ASSERT_TRUE(false) << "Need more tests";
}

TEST(VoxelOctreeTests, DISABLED_nearest_cell_more_tests) {
  ASSERT_TRUE(false) << "Need more tests";
}

TEST(VoxelOctreeTests, DISABLED_find_cell_more_tests) {
  ASSERT_TRUE(false) << "Need more tests";
}

TEST(VoxelOctreeTests, DISABLED_add_point_more_tests) {
  ASSERT_TRUE(false) << "Need more tests";
}

TEST(VoxelOctreeTests, add_sphere_small) {
  const size_t N = 4;
  VoxelOctree v(N);
  v.add_sphere(Sphere{{0.5, 0.5, 0.5}, 0.5});

  ASSERT_EQ(slice_to_string(v, 0),
      "....\n"
      ".11.\n"
      ".11.\n"
      "....\n");
  ASSERT_EQ(slice_to_string(v, 1),
      ".11.\n"
      "1111\n"
      "1111\n"
      ".11.\n");
  ASSERT_EQ(slice_to_string(v, 2),
      ".11.\n"
      "1111\n"
      "1111\n"
      ".11.\n");
  ASSERT_EQ(slice_to_string(v, 3),
      "....\n"
      ".11.\n"
      ".11.\n"
      "....\n");
}

TEST(VoxelOctreeTests, add_sphere_outside_limits) {
  const size_t N = 16;
  VoxelOctree v(N);
  v.set_xlim(3, 6);
  v.set_ylim(2, 10);
  v.set_zlim(-7, -4);
  // with the old limits, this would have fully covered the space
  // now, it shouldn't even color one voxel.
  v.add_sphere(Sphere{{.5, .5, .5}, 1});
  ASSERT_EQ(v.nblocks(), size_t(0));
  for (size_t bx = 0; bx < v.Nbx(); bx++) {
    for (size_t by = 0; by < v.Nby(); by++) {
      for (size_t bz = 0; bz < v.Nbz(); bz++) {
        ASSERT_EQ(v.block(bx, by, bz), size_t(0));
      }
    }
  }
}

TEST(VoxelOctreeTests, add_sphere_huge) {
  const size_t N = 8;
  VoxelOctree v(N);
  v.add_sphere(Sphere{{-6, 2, -4}, 23}); // easily covers space of [0, 1]^3

  for (size_t ix = 0; ix < v.Nx(); ix++) {
    ASSERT_EQ(slice_to_string(v, ix),
        "11111111\n"
        "11111111\n"
        "11111111\n"
        "11111111\n"
        "11111111\n"
        "11111111\n"
        "11111111\n"
        "11111111\n");
  }
}

TEST(VoxelOctreeTests, DISABLED_add_sphere_more_tests) {
  ASSERT_TRUE(false) << "Need more tests";
}

TEST(VoxelOctreeTests, add_voxels_empty) {
  const size_t N = 32;
  VoxelOctree v1(N), v2(N);
  ASSERT_EQ(v1.nblocks(), size_t(0));
  for (size_t bx = 0; bx < v1.Nbx(); bx++) {
    for (size_t by = 0; by < v1.Nby(); by++) {
      for (size_t bz = 0; bz < v1.Nbz(); bz++) {
        ASSERT_EQ(v1.block(bx, by, bz), size_t(0));
      }
    }
  }
  v1.add_voxels(v2);
  ASSERT_EQ(v1.nblocks(), size_t(0));
  for (size_t bx = 0; bx < v1.Nbx(); bx++) {
    for (size_t by = 0; by < v1.Nby(); by++) {
      for (size_t bz = 0; bz < v1.Nbz(); bz++) {
        ASSERT_EQ(v1.block(bx, by, bz), size_t(0));
      }
    }
  }

  size_t block_val(0xfeefabcabc012345);
  v1.set_block(1, 2, 3, block_val);
  v1.add_voxels(v2);
  ASSERT_EQ(v1.nblocks(), size_t(1));
  for (size_t bx = 0; bx < v1.Nbx(); bx++) {
    for (size_t by = 0; by < v1.Nby(); by++) {
      for (size_t bz = 0; bz < v1.Nbz(); bz++) {
        if (bx == 1 && by == 2 && bz == 3) {
          ASSERT_EQ(v1.block(bx, by, bz), block_val);
        } else {
          ASSERT_EQ(v1.block(bx, by, bz), size_t(0));
        }
      }
    }
  }
}

TEST(VoxelOctreeTests, add_voxels_self) {
  const size_t N = 32;
  VoxelOctree v1(N);
  ASSERT_EQ(v1.nblocks(), size_t(0));
  for (size_t bx = 0; bx < v1.Nbx(); bx++) {
    for (size_t by = 0; by < v1.Nby(); by++) {
      for (size_t bz = 0; bz < v1.Nbz(); bz++) {
        ASSERT_EQ(v1.block(bx, by, bz), size_t(0));
      }
    }
  }
  v1.add_voxels(v1);
  ASSERT_EQ(v1.nblocks(), size_t(0));
  for (size_t bx = 0; bx < v1.Nbx(); bx++) {
    for (size_t by = 0; by < v1.Nby(); by++) {
      for (size_t bz = 0; bz < v1.Nbz(); bz++) {
        ASSERT_EQ(v1.block(bx, by, bz), size_t(0));
      }
    }
  }

  size_t block_val(0xfeefabcabc012345);
  v1.set_block(1, 2, 3, block_val);
  v1.add_voxels(v1);
  ASSERT_EQ(v1.nblocks(), size_t(1));
  for (size_t bx = 0; bx < v1.Nbx(); bx++) {
    for (size_t by = 0; by < v1.Nby(); by++) {
      for (size_t bz = 0; bz < v1.Nbz(); bz++) {
        if (bx == 1 && by == 2 && bz == 3) {
          ASSERT_EQ(v1.block(bx, by, bz), block_val);
        } else {
          ASSERT_EQ(v1.block(bx, by, bz), size_t(0));
        }
      }
    }
  }
}

TEST(VoxelOctreeTests, add_voxels_nonoverlapping) {
  const size_t N = 4;
  VoxelOctree v1(N), v2(N);
  v1.add_sphere(Sphere{{0, 0, 0}, 1});
  v2.add_sphere(Sphere{{1, 1, 1}, .7});

  ASSERT_EQ(slice_to_string(v1, 0),
      "1111\n"
      "1111\n"
      "111.\n"
      "11..\n");
  ASSERT_EQ(slice_to_string(v1, 1),
      "1111\n"
      "111.\n"
      "111.\n"
      "1...\n");
  ASSERT_EQ(slice_to_string(v1, 2),
      "111.\n"
      "111.\n"
      "11..\n"
      "....\n");
  ASSERT_EQ(slice_to_string(v1, 3),
      "11..\n"
      "1...\n"
      "....\n"
      "....\n");

  ASSERT_EQ(slice_to_string(v2, 0),
      "....\n"
      "....\n"
      "....\n"
      "....\n");
  ASSERT_EQ(slice_to_string(v2, 1),
      "....\n"
      "....\n"
      "....\n"
      "...1\n");
  ASSERT_EQ(slice_to_string(v2, 2),
      "....\n"
      "....\n"
      "..11\n"
      "..11\n");
  ASSERT_EQ(slice_to_string(v2, 3),
      "....\n"
      "...1\n"
      "..11\n"
      ".111\n");

  v1.add_voxels(v2);

  ASSERT_EQ(slice_to_string(v1, 0),
      "1111\n"
      "1111\n"
      "111.\n"
      "11..\n");
  ASSERT_EQ(slice_to_string(v1, 1),
      "1111\n"
      "111.\n"
      "111.\n"
      "1..1\n");
  ASSERT_EQ(slice_to_string(v1, 2),
      "111.\n"
      "111.\n"
      "1111\n"
      "..11\n");
  ASSERT_EQ(slice_to_string(v1, 3),
      "11..\n"
      "1..1\n"
      "..11\n"
      ".111\n");

  ASSERT_EQ(slice_to_string(v2, 0),
      "....\n"
      "....\n"
      "....\n"
      "....\n");
  ASSERT_EQ(slice_to_string(v2, 1),
      "....\n"
      "....\n"
      "....\n"
      "...1\n");
  ASSERT_EQ(slice_to_string(v2, 2),
      "....\n"
      "....\n"
      "..11\n"
      "..11\n");
  ASSERT_EQ(slice_to_string(v2, 3),
      "....\n"
      "...1\n"
      "..11\n"
      ".111\n");
}

TEST(VoxelOctreeTests, add_voxels_overlapping) {
  const size_t N = 4;
  VoxelOctree v1(N), v2(N);
  v1.add_sphere(Sphere{{0, 0, 0}, 1});
  v2.add_sphere(Sphere{{0, 1, 1}, .7});

  ASSERT_EQ(slice_to_string(v1, 0),
      "1111\n"
      "1111\n"
      "111.\n"
      "11..\n");
  ASSERT_EQ(slice_to_string(v1, 1),
      "1111\n"
      "111.\n"
      "111.\n"
      "1...\n");
  ASSERT_EQ(slice_to_string(v1, 2),
      "111.\n"
      "111.\n"
      "11..\n"
      "....\n");
  ASSERT_EQ(slice_to_string(v1, 3),
      "11..\n"
      "1...\n"
      "....\n"
      "....\n");

  ASSERT_EQ(slice_to_string(v2, 0),
      "....\n"
      "...1\n"
      "..11\n"
      ".111\n");
  ASSERT_EQ(slice_to_string(v2, 1),
      "....\n"
      "....\n"
      "..11\n"
      "..11\n");
  ASSERT_EQ(slice_to_string(v2, 2),
      "....\n"
      "....\n"
      "....\n"
      "...1\n");
  ASSERT_EQ(slice_to_string(v2, 3),
      "....\n"
      "....\n"
      "....\n"
      "....\n");

  v1.add_voxels(v2);

  ASSERT_EQ(slice_to_string(v1, 0),
      "1111\n"
      "1111\n"
      "1111\n"
      "1111\n");
  ASSERT_EQ(slice_to_string(v1, 1),
      "1111\n"
      "111.\n"
      "1111\n"
      "1.11\n");
  ASSERT_EQ(slice_to_string(v1, 2),
      "111.\n"
      "111.\n"
      "11..\n"
      "...1\n");
  ASSERT_EQ(slice_to_string(v1, 3),
      "11..\n"
      "1...\n"
      "....\n"
      "....\n");

  ASSERT_EQ(slice_to_string(v2, 0),
      "....\n"
      "...1\n"
      "..11\n"
      ".111\n");
  ASSERT_EQ(slice_to_string(v2, 1),
      "....\n"
      "....\n"
      "..11\n"
      "..11\n");
  ASSERT_EQ(slice_to_string(v2, 2),
      "....\n"
      "....\n"
      "....\n"
      "...1\n");
  ASSERT_EQ(slice_to_string(v2, 3),
      "....\n"
      "....\n"
      "....\n"
      "....\n");
}

TEST(VoxelOctreeTests, DISABLED_remove_interior_more_tests) {
  ASSERT_TRUE(false) << "Need more tests";
}

TEST(VoxelOctreeTests, DISABLED_collides_more_tests) {
  ASSERT_TRUE(false) << "Need more tests";
}

TEST(VoxelOctreeTests, to_and_from_itk_image) {
  const size_t N = 32;
  VoxelOctree v1(N);
  v1.add_sphere(Sphere{{0.5, 0.75, 1.0}, 0.5});
  auto image = v1.to_itk_image();
  auto v2 = VoxelOctree::from_itk_image(image);

  // check limits
  ASSERT_EQ(v1.xlim().first , v2.xlim().first );
  ASSERT_EQ(v1.ylim().first , v2.ylim().first );
  ASSERT_EQ(v1.zlim().first , v2.zlim().first );
  ASSERT_EQ(v1.xlim().second, v2.xlim().second);
  ASSERT_EQ(v1.ylim().second, v2.ylim().second);
  ASSERT_EQ(v1.zlim().second, v2.zlim().second);
  ASSERT_EQ(v1.dx(), v2.dx());
  ASSERT_EQ(v1.dy(), v2.dy());
  ASSERT_EQ(v1.dz(), v2.dz());

  // check each slice
  ASSERT_EQ(slice_to_string(v1, 0), slice_to_string(v2, 0));
  ASSERT_EQ(slice_to_string(v1, 1), slice_to_string(v2, 1));
  ASSERT_EQ(slice_to_string(v1, 2), slice_to_string(v2, 2));
  ASSERT_EQ(slice_to_string(v1, 3), slice_to_string(v2, 3));
  ASSERT_EQ(slice_to_string(v1, 4), slice_to_string(v2, 4));
  ASSERT_EQ(slice_to_string(v1, 5), slice_to_string(v2, 5));
  ASSERT_EQ(slice_to_string(v1, 6), slice_to_string(v2, 6));
  ASSERT_EQ(slice_to_string(v1, 7), slice_to_string(v2, 7));

  // check the VoxelObject::operator==()
  ASSERT_EQ(v1, v2);
}

TEST(VoxelOctreeTests, to_and_from_itk_image_nonstandard_size) {
  const size_t N = 32;
  VoxelOctree v1(N);
  v1.set_xlim(32.21, 152.1);
  v1.set_ylim(-21, 22.22);
  v1.set_zlim(0.5, 1.2);

  v1.add_sphere(Sphere{{0.5, 0.75, 1.0}, 0.5});
  auto image = v1.to_itk_image();
  auto v2 = VoxelOctree::from_itk_image(image);

  // check limits
  ASSERT_EQ(v1.xlim().first , v2.xlim().first );
  ASSERT_EQ(v1.ylim().first , v2.ylim().first );
  ASSERT_EQ(v1.zlim().first , v2.zlim().first );
  ASSERT_EQ(v1.xlim().second, v2.xlim().second);
  ASSERT_EQ(v1.ylim().second, v2.ylim().second);
  ASSERT_EQ(v1.zlim().second, v2.zlim().second);
  ASSERT_EQ(v1.dx(), v2.dx());
  ASSERT_EQ(v1.dy(), v2.dy());
  ASSERT_EQ(v1.dz(), v2.dz());

  // check each slice
  ASSERT_EQ(slice_to_string(v1, 0), slice_to_string(v2, 0));
  ASSERT_EQ(slice_to_string(v1, 1), slice_to_string(v2, 1));
  ASSERT_EQ(slice_to_string(v1, 2), slice_to_string(v2, 2));
  ASSERT_EQ(slice_to_string(v1, 3), slice_to_string(v2, 3));
  ASSERT_EQ(slice_to_string(v1, 4), slice_to_string(v2, 4));
  ASSERT_EQ(slice_to_string(v1, 5), slice_to_string(v2, 5));
  ASSERT_EQ(slice_to_string(v1, 6), slice_to_string(v2, 6));
  ASSERT_EQ(slice_to_string(v1, 7), slice_to_string(v2, 7));

  // check the VoxelObject::operator==()
  ASSERT_EQ(v1, v2);
}

TEST(VoxelOctreeTests, to_nrrd_creates_file) {
  const size_t N = 8;
  VoxelOctree v(N);
  v.add_sphere(Sphere{{0.5, 0.75, 1.0}, 0.5});
  FileRemover remover("/tmp/VoxelOctreeTests-to-nrrd-creates-file.nrrd");
  v.to_nrrd(remover.fname());
  ASSERT_TRUE(file_exists(remover.fname()));
}

TEST(VoxelOctreeTests, to_and_from_nrrd) {
  const size_t N = 32;
  VoxelOctree v1(N);
  v1.add_sphere(Sphere{{0.5, 0.75, 1.0}, 0.5});
  FileRemover remover("/tmp/VoxelOctreeTests-to-and-from-nrrd.nrrd");
  v1.to_nrrd(remover.fname());
  ASSERT_TRUE(file_exists(remover.fname()));

  auto v2 = VoxelOctree::from_nrrd(remover.fname());

  // check limits
  ASSERT_EQ(v1.xlim().first , v2.xlim().first );
  ASSERT_EQ(v1.ylim().first , v2.ylim().first );
  ASSERT_EQ(v1.zlim().first , v2.zlim().first );
  ASSERT_EQ(v1.xlim().second, v2.xlim().second);
  ASSERT_EQ(v1.ylim().second, v2.ylim().second);
  ASSERT_EQ(v1.zlim().second, v2.zlim().second);
  ASSERT_EQ(v1.dx(), v2.dx());
  ASSERT_EQ(v1.dy(), v2.dy());
  ASSERT_EQ(v1.dz(), v2.dz());

  // check each slice
  ASSERT_EQ(slice_to_string(v1, 0), slice_to_string(v2, 0));
  ASSERT_EQ(slice_to_string(v1, 1), slice_to_string(v2, 1));
  ASSERT_EQ(slice_to_string(v1, 2), slice_to_string(v2, 2));
  ASSERT_EQ(slice_to_string(v1, 3), slice_to_string(v2, 3));
  ASSERT_EQ(slice_to_string(v1, 4), slice_to_string(v2, 4));
  ASSERT_EQ(slice_to_string(v1, 5), slice_to_string(v2, 5));
  ASSERT_EQ(slice_to_string(v1, 6), slice_to_string(v2, 6));
  ASSERT_EQ(slice_to_string(v1, 7), slice_to_string(v2, 7));

  // check the VoxelObject::operator==()
  ASSERT_EQ(v1, v2);
}

TEST(VoxelOctreeTests, to_and_from_nrrd_nonstandard_size) {
  const size_t N = 32;
  VoxelOctree v1(N);
  v1.set_xlim(32.21, 152.1);
  v1.set_ylim(-21, 22.22);
  v1.set_zlim(0.5, 1.2);

  v1.add_sphere(Sphere{{0.5, 0.75, 1.0}, 0.5});
  FileRemover remover("/tmp/VoxelOctreeTests-to-and-from-nrrd-nonstandard-size.nrrd");
  v1.to_nrrd(remover.fname());
  ASSERT_TRUE(file_exists(remover.fname()));

  auto v2 = VoxelOctree::from_nrrd(remover.fname());

  // check limits
  ASSERT_EQ(v1.xlim().first , v2.xlim().first );
  ASSERT_EQ(v1.ylim().first , v2.ylim().first );
  ASSERT_EQ(v1.zlim().first , v2.zlim().first );
  ASSERT_EQ(v1.xlim().second, v2.xlim().second);
  ASSERT_EQ(v1.ylim().second, v2.ylim().second);
  ASSERT_EQ(v1.zlim().second, v2.zlim().second);
  ASSERT_EQ(v1.dx(), v2.dx());
  ASSERT_EQ(v1.dy(), v2.dy());
  ASSERT_EQ(v1.dz(), v2.dz());

  // check each slice
  ASSERT_EQ(slice_to_string(v1, 0), slice_to_string(v2, 0));
  ASSERT_EQ(slice_to_string(v1, 1), slice_to_string(v2, 1));
  ASSERT_EQ(slice_to_string(v1, 2), slice_to_string(v2, 2));
  ASSERT_EQ(slice_to_string(v1, 3), slice_to_string(v2, 3));
  ASSERT_EQ(slice_to_string(v1, 4), slice_to_string(v2, 4));
  ASSERT_EQ(slice_to_string(v1, 5), slice_to_string(v2, 5));
  ASSERT_EQ(slice_to_string(v1, 6), slice_to_string(v2, 6));
  ASSERT_EQ(slice_to_string(v1, 7), slice_to_string(v2, 7));

  // check the VoxelObject::operator==()
  ASSERT_EQ(v1, v2);
}

TEST(VoxelOctreeTests, to_json_creates_file) {
  const size_t N = 8;
  VoxelOctree v(N);
  v.add_sphere(Sphere{{0.5, 0.75, 1.0}, 0.5});
  FileRemover remover("/tmp/VoxelOctreeTests-to-json-creates-file.json");
  {
    std::ofstream out(remover.fname());
    out << v.to_json();
  }
  ASSERT_TRUE(file_exists(remover.fname()));
}

TEST(VoxelOctreeTests, to_and_from_json) {
  const size_t N = 32;
  VoxelOctree v1(N);
  v1.add_sphere(Sphere{{0.5, 0.75, 1.0}, 0.5});
  FileRemover remover("/tmp/VoxelOctreeTests-to-and-from-json.json");
  {
    std::ofstream out(remover.fname());
    out << v1.to_json();
  }
  ASSERT_TRUE(file_exists(remover.fname()));

  std::ifstream in(remover.fname());
  nlohmann::json json;
  in >> json;
  auto v2 = VoxelOctree::from_json(json);

  // check limits
  ASSERT_EQ(v1.xlim().first , v2.xlim().first );
  ASSERT_EQ(v1.ylim().first , v2.ylim().first );
  ASSERT_EQ(v1.zlim().first , v2.zlim().first );
  ASSERT_EQ(v1.xlim().second, v2.xlim().second);
  ASSERT_EQ(v1.ylim().second, v2.ylim().second);
  ASSERT_EQ(v1.zlim().second, v2.zlim().second);
  ASSERT_EQ(v1.dx(), v2.dx());
  ASSERT_EQ(v1.dy(), v2.dy());
  ASSERT_EQ(v1.dz(), v2.dz());

  // check each slice
  ASSERT_EQ(slice_to_string(v1, 0), slice_to_string(v2, 0));
  ASSERT_EQ(slice_to_string(v1, 1), slice_to_string(v2, 1));
  ASSERT_EQ(slice_to_string(v1, 2), slice_to_string(v2, 2));
  ASSERT_EQ(slice_to_string(v1, 3), slice_to_string(v2, 3));
  ASSERT_EQ(slice_to_string(v1, 4), slice_to_string(v2, 4));
  ASSERT_EQ(slice_to_string(v1, 5), slice_to_string(v2, 5));
  ASSERT_EQ(slice_to_string(v1, 6), slice_to_string(v2, 6));
  ASSERT_EQ(slice_to_string(v1, 7), slice_to_string(v2, 7));

  // check the VoxelObject::operator==()
  ASSERT_EQ(v1, v2);
}

TEST(VoxelOctreeTests, to_and_from_json_nonstandard_size) {
  const size_t N = 32;
  VoxelOctree v1(N);
  v1.set_xlim(32.21, 152.1);
  v1.set_ylim(-21, 22.22);
  v1.set_zlim(0.5, 1.2);

  v1.add_sphere(Sphere{{0.5, 0.75, 1.0}, 0.5});
  FileRemover remover("/tmp/VoxelOctreeTests-to-and-from-json-nonstandard-size.json");
  {
    std::ofstream out(remover.fname());
    out << v1.to_json();
  }
  ASSERT_TRUE(file_exists(remover.fname()));

  std::ifstream in(remover.fname());
  nlohmann::json json;
  in >> json;
  auto v2 = VoxelOctree::from_json(json);

  // check limits
  ASSERT_EQ(v1.xlim().first , v2.xlim().first );
  ASSERT_EQ(v1.ylim().first , v2.ylim().first );
  ASSERT_EQ(v1.zlim().first , v2.zlim().first );
  ASSERT_EQ(v1.xlim().second, v2.xlim().second);
  ASSERT_EQ(v1.ylim().second, v2.ylim().second);
  ASSERT_EQ(v1.zlim().second, v2.zlim().second);
  ASSERT_EQ(v1.dx(), v2.dx());
  ASSERT_EQ(v1.dy(), v2.dy());
  ASSERT_EQ(v1.dz(), v2.dz());

  // check each slice
  ASSERT_EQ(slice_to_string(v1, 0), slice_to_string(v2, 0));
  ASSERT_EQ(slice_to_string(v1, 1), slice_to_string(v2, 1));
  ASSERT_EQ(slice_to_string(v1, 2), slice_to_string(v2, 2));
  ASSERT_EQ(slice_to_string(v1, 3), slice_to_string(v2, 3));
  ASSERT_EQ(slice_to_string(v1, 4), slice_to_string(v2, 4));
  ASSERT_EQ(slice_to_string(v1, 5), slice_to_string(v2, 5));
  ASSERT_EQ(slice_to_string(v1, 6), slice_to_string(v2, 6));
  ASSERT_EQ(slice_to_string(v1, 7), slice_to_string(v2, 7));

  // check the VoxelObject::operator==()
  ASSERT_EQ(v1, v2);
}


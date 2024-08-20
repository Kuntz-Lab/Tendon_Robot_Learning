#include <cliparser/CliParser.h>
#include <collision/Sphere.h>
#include <collision/VoxelOctree.h>
#include <util/json_io.h>
#include <3rdparty/nlohmann/json.hpp>

#include <iostream>

using collision::Sphere;
using collision::VoxelOctree;

namespace {

void populate_parser(CliParser &parser) {
  parser.set_program_description(
      "Demonstrate the ability of VoxelOctree to voxelize a sphere and to\n"
      "  convert a voxel object to a boxy mesh for various resolutions.");
}

// write a circle from the center of the voxel space in a [0, 1]^3 space
void write_sphere(const std::string &filebase, double r = 0.5,
                  size_t voxel_dim = 256)
{
  VoxelOctree v(voxel_dim);
  //v.set_zlim(0, 5);
  v.add_sphere(Sphere{{.5, .5, .5}, r});
  std::cout << "Voxel(" << v.Nx() << ") blocks: " << v.nblocks() << std::endl;
  std::cout << "  Writing " << filebase + ".msgpack" << std::endl;
  auto j = v.to_json();
  util::write_json(filebase + ".msgpack", j);
  std::cout << "  Writing " << filebase + ".stl" << std::endl;
  auto m = v.to_mesh();
  m.to_stl(filebase + ".stl");
}

} // end of unnamed namespace

int main(int arg_count, char *arg_list[]) {
  CliParser parser;
  populate_parser(parser);
  parser.parse(arg_count, arg_list);

  write_sphere("sphere-004", 0.5,   4);
  write_sphere("sphere-008", 0.5,   8);
  write_sphere("sphere-016", 0.5,  16);
  write_sphere("sphere-032", 0.5,  32);
  write_sphere("sphere-064", 0.5,  64);
  write_sphere("sphere-128", 0.5, 128);
  write_sphere("sphere-256", 0.5, 256);
  write_sphere("sphere-512", 0.5, 512);

  return 0;
}

#include "cliparser/CliParser.h"
#include "collision/stl_io.h"

#include <string>
#include <iostream>

void populate_parser(CliParser &parser) {
  parser.set_program_description(
      "Convert STL to binary or ASCII and fix normal vectors");
  parser.add_flag("-b", "--binary");
  parser.add_argflag("-s", "--scale");
  parser.add_positional("input");
  parser.add_positional("output");
  parser.set_required("input");
  parser.set_required("output");
  parser.set_description("--binary",
      "output STL in binary format (ASCII format by default)");
  parser.set_description("--scale", "scale STL by this factor");
  parser.set_description("input", "input STL mesh file");
  parser.set_description("output", "output STL mesh file");
}

int main(int argCount, char* argList[]) {
  CliParser parser;
  populate_parser(parser);
  parser.parse(argCount, argList);
  std::cout << "input:  " << parser["input"]  << "\n"
               "output: " << parser["output"] << "\n";

  // read mesh
  std::vector<float> vertices, normals;
  std::vector<size_t> triangles, solid_ranges;
  collision::read_stl_file(parser["input"].c_str(), vertices, normals,
                           triangles, solid_ranges);

  // scale mesh
  if (parser.has("--scale")) {
    float scale = parser.get("--scale", 1.0f);
    for (auto &v : vertices) {
      v *= scale;
    }
  }

  // output mesh
  if (parser.has("--binary")) {
    collision::write_stl_file_binary(parser["output"], vertices, triangles);
  } else {
    collision::write_stl_file_ascii(parser["output"], vertices, triangles);
  }
  return 0;
}

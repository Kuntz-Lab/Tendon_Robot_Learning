#include <cliparser/CliParser.h>
#include <util/openfile_check.h>
#include <util/json_io.h>

#include <3rdparty/nlohmann/json.hpp>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

using nlohmann::json;

namespace {

namespace defaults {
  int indent = 4;
} // end of namespace defaults

void populate_parser(CliParser &parser) {
  parser.set_program_description("Pretty print a json file to the console.");

  parser.add_positional("jsonfile");
  parser.set_description("jsonfile", "file to pretty print, default to stdout");

  parser.add_argflag("-i", "--indent");
  parser.set_description("--indent", "indent to use (default is "
                         + std::to_string(defaults::indent) + ")");

  parser.add_argflag("-f", "--format");
  parser.set_description("--format",
                      "json format.  If given a jsonfile, will guess based\n"
      "                on the file's extension.  If reading from stdin, will\n"
      "                by default assume JSON text format.  This flag\n"
      "                overrides that default behavior to use a specific\n"
      "                format.  Choices are json, bson, cbor, msgpack, and\n"
      "                ubjson.");
}

} // end of unnamed namespace

int main(int arg_count, char *arg_list[]) {
  CliParser parser;
  populate_parser(parser);
  parser.parse(arg_count, arg_list);

  auto indent = parser.get("--indent", defaults::indent);

  // figure out the format
  auto format = util::JsonFormat::JSON;
  if (parser.has("--format")) {
    format = util::json_format(parser["--format"]);
  } else if (parser.has("jsonfile")) {
    format = util::json_format_from_path(parser["jsonfile"]);
  }

  // read it in
  json data;
  if (parser.has("jsonfile")) {
    data = util::read_json(parser["jsonfile"], format);
  } else {
    data = util::read_json(std::cin, format);
  }

  // print it out
  std::cout << std::setw(indent) << data << std::endl;

  return 0;
}

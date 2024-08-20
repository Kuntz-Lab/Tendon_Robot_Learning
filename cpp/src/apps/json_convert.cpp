#include <cliparser/CliParser.h>
#include <util/json_io.h>
#include <util/openfile_check.h>
#include <util/time_function_call.h>

#include <3rdparty/nlohmann/json.hpp>

#include <algorithm>
#include <cctype>  // for std::tolower()
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using nlohmann::json;

namespace {

void populate_parser(CliParser &parser) {
  parser.set_program_description("Convert to and from different json file formats");

  parser.add_positional("infile");
  parser.set_description("infile", "file to convert, default to stdin");

  parser.add_positional("outfile");
  parser.set_description("outfile", "file to write, default to stdout");

  parser.add_argflag("-i", "--input-format");
  parser.set_description("--input-format",
                      "Format for input.  Valid types are json, bson, cbor,\n"
      "                msgpack, and ubjson.  Typically this is inferred by\n"
      "                the file extension, which actually matches the\n"
      "                available types.  Use this option if you are either\n"
      "                using stdin for your input or the file extension does\n"
      "                not match one of the valid types (case-insensitive).");

  parser.add_argflag("-o", "--output-format");
  parser.set_description("--output-format",
                      "Format for output.  Valid types are json, bson, cbor,\n"
      "                msgpack, and ubjson.  Typically this is inferred by\n"
      "                the file extension, which actually matches the\n"
      "                available types.  Use this option if you are either\n"
      "                using stdout for your output or the file extension does\n"
      "                not match one of the valid types (case-insensitive).");

  parser.add_flag("-q", "--quiet");
  parser.set_description("--quiet", "Suppress the timing message at the end");
}

} // end of unnamed namespace

int main(int arg_count, char* arg_list[]) {
  CliParser parser;
  populate_parser(parser);
  parser.parse(arg_count, arg_list);

  // figure out the input format
  auto in_format = util::JsonFormat::JSON;
  if (parser.has("-i")) {
    in_format = util::json_format(parser["-i"]);
  } else if (parser.has("infile")) {
    in_format = util::json_format_from_path(parser["infile"]);
  }

  // figure out the output format
  auto out_format = util::JsonFormat::JSON;
  if (parser.has("-o")) {
    out_format = util::json_format(parser["-o"]);
  } else if (parser.has("outfile")) {
    out_format = util::json_format_from_path(parser["outfile"]);
  }

  // read function
  std::function<json()> read;
  if (parser.has("infile")) {
    read = [&parser, in_format]()
        { return util::read_json(parser["infile"], in_format); };
  } else {
    read = [in_format]()
        { return util::read_json(std::cin, in_format); };
  }

  // write function
  std::function<void(const json &)> write;
  if (parser.has("outfile")) {
    write = [&parser, out_format](const json &data)
        { util::write_json(parser["outfile"], data, out_format); };
  } else {
    write = [out_format](const json &data)
        { util::write_json(std::cout, data, out_format); };
  }

  // run the read and write
  float rtime;
  json data = util::time_function_call(read, rtime);
  float wtime = util::time_function_call([&]() { write(data); });

  if (!parser.has("--quiet")) {
    std::cerr << "\n"
                 "Reading took " << rtime << " seconds\n"
                 "Writing took " << wtime << " seconds\n"
                 "\n";
  }

  return 0;
} // end of main()

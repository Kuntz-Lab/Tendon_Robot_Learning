#include "serialport/SerialPort.h"
#include "cliparser/CliParser.h"
#include <chrono>
#include <fstream>

void populate_parser(CliParser &parser) {
    parser.add_argflag("-b", "--byte");
    parser.set_description("--byte", "Input byte");
}

int main(int argCount, char* argList[]){
    unsigned char data=1500;
    CliParser parser;
    populate_parser(parser);
    parser.parse(argCount, argList);
    auto byte = parser.get("--byte",180);
    SerialPort myPort("/dev/ttyACM0");
    sleep(2);
    myPort.writeData(data,2);
    myPort.closePort();
}

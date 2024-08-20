SOURCE         += $(wildcard *.cpp)
SOURCE         += $(wildcard tests/*.cpp)
SOURCE         += $(wildcard ../src/tendon/*.cpp)

CXXFLAGS       += -std=c++17
CXXFLAGS       += -I../src
CXXFLAGS       += -isystem /usr/include/eigen3

LDFLAGS        +=
LDLIBS         +=

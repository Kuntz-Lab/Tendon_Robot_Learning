#ifndef ndi_ParseError_H
#define ndi_ParseError_H

#include <stdexcept>

namespace ndi {

class ParseError : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

} // end of namespace ndi

#endif // ndi_ParseError_H

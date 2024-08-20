#ifndef ndi_char_parse_H
#define ndi_char_parse_H

#include <QtGlobal>
#include <QByteArray>

#include <sstream>

#include <cstring> // for std::memcpy()
#include <cstdio>  // for std::snprintf()

namespace ndi {

// function declarations
inline quint32 hex_to_uint  (const QByteArray &hex);
inline float   text_to_float(const QByteArray &data);
inline quint32 bin_to_uint  (const QByteArray &data);
inline float   bin_to_float (const QByteArray &data);
inline QByteArray char_to_hex(char ch);

inline quint32 hex_to_uint(const QByteArray &hex) {
  bool ok;
  quint32 val = hex.toUInt(&ok, 16);
  if (!ok) {
    throw ParseError("Could not convert hex to unsigned integer");
  }
  return val;
}

inline float text_to_float(const QByteArray &data) {
  bool ok;
  auto val = data.toFloat(&ok);
  if (!ok) {
    throw ParseError("Could not convert to double precision");
  }
  return val;
}

inline quint32 bin_to_uint(const QByteArray &data) {
  quint32 val = 0;
  if (data.size() > 4) {
    throw ParseError("Too many bytes given to bin_to_uint()");
  }
  static unsigned char copy[4];
  std::memcpy(copy, data, data.size());
  for (decltype(data.size()) i = 0; i < data.size(); i++) {
    val += (quint32(copy[i]) << (8 * i));
  }
  return val;
}

inline float bin_to_float(const QByteArray &data) {
  if (sizeof(float) != sizeof(quint32)) {
    throw std::runtime_error("float is not 32-bits!!!");
  }
  quint32 intval = bin_to_uint(data);
  float fval;
  // just put the bytes of the unsigned integer into the float
  std::memcpy(&fval, &intval, sizeof(float));
  return fval;
}

inline QByteArray char_to_hex(char ch) {
  QByteArray val(2, 0);
  snprintf(val.data(), 3, "%02X", ch);
  return val;
}

} // end of namespace ndi

#endif // ndi_char_parse_H

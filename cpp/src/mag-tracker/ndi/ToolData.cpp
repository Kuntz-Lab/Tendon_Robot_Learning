#include "ToolData.h"
#include "NdiManager.h"
#include "ParseError.h"
#include "char_parse.h"


namespace ndi {


//
// =Public Static Methods=
//

ToolData ToolData::from_tx(const QByteArray &data) {
  ToolData obj;

  obj.missing  = false;
  obj.id       = data.left(2);
  auto payload = data.mid(2, data.size() - 18);
  obj.status   = hex_to_uint(data.mid(data.size() - 16, 8));
  obj.frame    = hex_to_uint(data.right(8));

  // TODO: interpret tool status

  if (payload == "MISSING") {
    obj.missing = true;
  } else {
    if (payload.size() != 51) {
      qDebug() << "payload.size(): " << payload.size();
      qDebug() << "payload:        " << payload;
      throw ParseError("TX payload not 51 bytes");
    }
    obj.orientation.setScalar(text_to_float(payload.left(6)));
    obj.orientation.setX(text_to_float(payload.mid( 6, 6)));
    obj.orientation.setY(text_to_float(payload.mid(12, 6)));
    obj.orientation.setZ(text_to_float(payload.mid(18, 6)));
    obj.orientation.normalize();
    obj.position.setX(text_to_float(payload.mid(24, 7)));
    obj.position.setY(text_to_float(payload.mid(31, 7)));
    obj.position.setZ(text_to_float(payload.mid(38, 7)));
    obj.position /= 1e5f; // convert to meters
    obj.error = text_to_float(payload.right(6));
  }

  return obj;
}

ToolData ToolData::from_bx(const QByteArray &data) {
  ToolData obj;

  // should be 42 characters
  if (data.size() != 42) {
    ParseError("BX data is not 42 bytes (" + std::to_string(data.size()) + ")");
  }

  // TODO: interpret tool status

  obj.id      = char_to_hex(data[0]);
  obj.missing = bool(data[1] & 0x06); // 2 means missing, 4 means disabled
  obj.orientation.setScalar(bin_to_float(data.mid(2, 4)));
  obj.orientation.setX(bin_to_float(data.mid( 6, 4)));
  obj.orientation.setY(bin_to_float(data.mid(10, 4)));
  obj.orientation.setZ(bin_to_float(data.mid(14, 4)));
  obj.position.setX(bin_to_float(data.mid(18, 4)));
  obj.position.setY(bin_to_float(data.mid(22, 4)));
  obj.position.setZ(bin_to_float(data.mid(26, 4)));
  obj.position /= 1e3f; // convert to meters
  obj.error  = bin_to_float(data.mid(30, 4));
  obj.status = bin_to_uint(data.mid(34, 4));
  obj.frame  = bin_to_uint(data.mid(38, 4));

  return obj;
}

} // end of namespace ndi

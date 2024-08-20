#include "haptic/HapticTransform.h"
#include <cpptoml/toml_conversions.h>
#include <cpptoml/cpptoml.h>

namespace haptic {

std::shared_ptr<cpptoml::table> HapticTransform::to_toml() const {
  auto container = cpptoml::make_table();

  auto tbl = cpptoml::make_table();
  container->insert("haptic_transform", tbl);
  tbl->insert("scale", this->scale);

  auto trans = cpptoml::make_array();
  tbl->insert("translation", trans);
  trans->push_back(this->translation.x());
  trans->push_back(this->translation.y());
  trans->push_back(this->translation.z());

  return container;
}

HapticTransform HapticTransform::from_toml(std::shared_ptr<cpptoml::table> tbl) {
  if (!tbl) { throw std::invalid_argument("null table given"); }

  HapticTransform transform_info;

  if (!tbl->contains("haptic_transform")) { return transform_info; }

  auto haptic_transform_tbl = tbl->get("haptic_transform")->as_table();
  if (!haptic_transform_tbl) {
    throw cpptoml::parse_exception(
        "Wrong type detected for 'haptic_transform': not a table");
  }

  if (haptic_transform_tbl->contains("translation")) {
    auto trans = haptic_transform_tbl->get("translation")->as_array();
    if (!trans) {
      throw cpptoml::parse_exception(
          "Wrong type detected for 'haptic-transform.translation': not an array");
    }
    auto point = cpptoml::to_point(trans);
    transform_info.translation.setX(point[0]);
    transform_info.translation.setY(point[1]);
    transform_info.translation.setZ(point[2]);
  }

  if (haptic_transform_tbl->contains("scale")) {
    auto scale_val = haptic_transform_tbl->get("scale")->as<double>();
    if (!scale_val) {
      throw cpptoml::parse_exception("Wrong type detected for scale");
    }
    transform_info.scale = scale_val->get();
  }

  return transform_info;
}

std::ostream& operator<<(std::ostream &out, const HapticTransform &t) {
  out << "HapticTransformer("
        << "transform: ["
          << t.translation.x() << ", "
          << t.translation.y() << ", "
          << t.translation.z() << "], "
        << "scale: " << t.scale << ")";
  return out;
}

} // end of namespace haptic

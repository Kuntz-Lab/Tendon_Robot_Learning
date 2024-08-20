#ifndef HAPTIC_TRANSFORM_H
#define HAPTIC_TRANSFORM_H

#include <QMatrix4x4>
#include <QVector3D>

#include <iostream>
#include <memory>

namespace cpptoml { class table; }

namespace haptic {

struct HapticTransform {
  QVector3D translation {0.0, 0.0, 0.0};
  double scale {1.0};

  QMatrix4x4 to_transform() const {
    QMatrix4x4 transform;
    transform.scale(scale);
    transform.translate(translation);
    return transform;
  }

  std::shared_ptr<cpptoml::table> to_toml() const;

  /** if there, then will extract.  if not there, will use defaults. */
  static HapticTransform from_toml(std::shared_ptr<cpptoml::table> tbl);
}; // end of struct HapticTransform

std::ostream& operator<<(std::ostream &out, const HapticTransform &t);

} // end of namespace haptic

#endif // HAPTIC_TRANSFORM_H

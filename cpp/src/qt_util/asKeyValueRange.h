/**
 * Author:       Michael Bentley
 * Created:      07 July 2021
 * Based from:
 *       https://www.kdab.com/qt-range-based-for-loops-and-structured-bindings/
 * Description:  Provides the asKeyValueRange<> class that can wrap around a
 *               QMap or a QHash that allows it to be used in a C++17-style
 *               range-based for loop
 * Example:
 *
 *   QMap<QString, int> mapping;
 *   for (auto [key, value] : asKeyValueRange(mapping)) {
 *     // ...
 *   }
 */

#ifndef qt_util_asKeyValueRange_H
#define qt_util_asKeyValueRange_H

namespace qt_util {

/** STL-style wrapper around a QMap or QHash
 * using keyValueBegin() and keyValueEnd().
 */
template <typename T>
class asKeyValueRange {
public:
  asKeyValueRange(T &data) : m_data{data} { }
  auto begin() { return m_data.keyValueBegin(); }
  auto end()   { return m_data.keyValueEnd(); }
private:
  T &m_data;
};

} // end of namespace qt_util

#endif // qt_util_asKeyValueRange_H

#include <QSerialPortInfo>
#include <QtGlobal>

#include <iostream>

int main(void) {
  auto ports = QSerialPortInfo::availablePorts();
  for (auto &port : ports) {
    std::cout << qPrintable(port.portName()) << std::endl;
  }
  return 0;
}

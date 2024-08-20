#include "SerialPort.h"
#include <chrono>
#include <thread>


SerialPort::SerialPort(std::string port_name="/dev/ttyACM0")
{
    serial.setPortName(port_name.c_str());
    serial.setBaudRate(QSerialPort::Baud9600);
    if (!serial.open(QIODevice::ReadWrite)) {
        std::cerr << "Could not open " << port_name.c_str() << std::endl;
    }
    else{
        std::cout<<"Port opened for reading and writing\n";
    }

}

SerialPort::~SerialPort()
{
    serial.close();
}

void SerialPort::writeData(const std::vector<std::string> msg,unsigned int sleep_time)
{

    for(auto i : msg){
        writeData(i.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
    }
}

void SerialPort::writeData (const char* msg)
{


        auto bytes_written = serial.write(msg);
        if (bytes_written <= -1) {
            std::cerr << "Failed to send message on serial port" << std::endl;
            return;
        }
        if (size_t(bytes_written) != strlen(msg)) {
            std::cerr << "Failed to send entire message" << std::endl;
            return;
        }
        if (!serial.waitForBytesWritten(5000)) {
            std::cerr << "Timeout for writing to serial port" << std::endl;
            return;
        }

        std::cout<<"Message sent over serial port is "<<msg<<std::endl;

}

void SerialPort::readData()
{
    std::cout<<"Reading data\n";
    QByteArray readData = serial.readAll();
    while (serial.waitForReadyRead(5000)){
      readData.append(serial.readAll());
    }
    if (serial.error() == QSerialPort::ReadError) {
      std::cout << "Failed to read from port " << serial.portName().toStdString()
                << ", error: " << serial.errorString().toStdString()
                << std::endl;
      return;
    } else if (serial.error() == QSerialPort::TimeoutError && readData.isEmpty()) {
      std::cout << "No data was currently available for reading from port "
                << serial.portName().toStdString()
                << std::endl;
      return;
    }
    std::cout << "Data successfully received from port "
              << serial.portName().toStdString()
              << std::endl;
    std::cout << "Data is " << readData.toStdString() << std::endl;
    std::cout<<"finished reading";
}

void SerialPort::parseData()
{

}

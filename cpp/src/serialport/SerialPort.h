#ifndef SERIALPORT_H
#define SERIALPORT_H

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <QCoreApplication>
#include <QtSerialPort>
#include <QString>

#include <iostream>
#include <chrono>
#include <thread>
#include <string.h>


class SerialPort
{
private:
    QSerialPort serial;
    std::vector<int> parsed_Data;
public:

    SerialPort(std::string port_name);
    void writeData(const std::vector<std::string>,unsigned int);
    void writeData(const char*);
    void readData();
    void parseData();
    ~SerialPort();

};

#endif // SERIALPORT_H

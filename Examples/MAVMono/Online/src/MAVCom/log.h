//
// Created by rasp on 1/23/18.
//

#ifndef C_UART_INTERFACE_EXAMPLE_LOG_H
#define C_UART_INTERFACE_EXAMPLE_LOG_H

#include <boost/filesystem.hpp>
#include <iostream>

#include <cpr/cpr.h>
#include <nlohmann/json.hpp>

class Log{
public:
    Log(std::string name);
    ~Log();
    void position_stream(double lat, double lon, double alt, double timestamp);
};
#endif //C_UART_INTERFACE_EXAMPLE_LOG_H

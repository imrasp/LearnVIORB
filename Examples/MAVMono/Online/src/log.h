//
// Created by rasp on 1/23/18.
//

#ifndef C_UART_INTERFACE_EXAMPLE_LOG_H
#define C_UART_INTERFACE_EXAMPLE_LOG_H

#include <boost/filesystem.hpp>
#include <iostream>

class Log{
public:
    Log(std::string name);
    ~Log();
};
#endif //C_UART_INTERFACE_EXAMPLE_LOG_H

//
// Created by Vilasinee Rasmeehirun on 29/3/2018 AD.
//

#ifndef C_UART_INTERFACE_EXAMPLE_TIME_RECORDER_H
#define C_UART_INTERFACE_EXAMPLE_TIME_RECORDER_H

#include <iostream>

#include <fstream>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <common/mavlink.h>
#include <queue>
#include "configParam.h"

struct time_data{
uint64_t time1, time2, time3, time4, time5;
};

class Time_Recorder{
public:
    Time_Recorder(string filename_, string record_path_);
    ~Time_Recorder();

    void start();
    void stop();
    void add_time_to_queue(uint64_t time1, uint64_t time2, uint64_t time3, uint64_t time4,
                           uint64_t time5);
    void write_time_from_queue();

private:
    ConfigParam *configParam;

    // mutex with condition signal
    pthread_cond_t unEmptyTime, emptyTime;
    pthread_mutex_t mutexTime;
    // queue for struc time
    std::queue<time_data> queueTime;
    string filename, record_path;
    bool active;
    bool time_to_exit;

    boost::thread threadRecordTime;
};

#endif //C_UART_INTERFACE_EXAMPLE_TIME_RECORDER_H

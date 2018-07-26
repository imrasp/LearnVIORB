//
// Created by rasp on 1/23/18.
//
#include <iostream>
#include <fstream>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <common/mavlink.h>
#include <queue>
#include "configParam.h"

#ifndef C_UART_INTERFACE_EXAMPLE_IMU_RECORDER_H
#define C_UART_INTERFACE_EXAMPLE_IMU_RECORDER_H
struct imu_data{
    uint64_t timestamp;
    double xacc, yacc, zacc, xgyro, ygyro, zgyro;
};

class IMU_Recorder{
public:
    IMU_Recorder(string filename_, string record_path_);
    ~IMU_Recorder();

    void start();
    void stop();
    void set_ref_time(mavlink_system_time_t system_time);
    uint64_t get_ns_time_ref_odroid(uint64_t time_ms);
    void add_imu_to_queue(uint64_t timestamp, double xacc, double yacc, double zacc,
    double xgyro, double ygyro, double zgyro);
    void write_imu_from_queue();

private:
    ConfigParam *configParam;

    // mutex with condition signal
    pthread_cond_t unEmptyIMU, emptyIMU;
    pthread_mutex_t mutexIMU;
    // queue for struc imu
    std::queue<imu_data> queueIMU;
    string filename, record_path;
    bool active;
    bool time_to_exit;

    uint64_t unix_time_ref, boot_time_ref, ref_boot_time_ms, ref_timestampcamera_ns;
    mavlink_system_time_t ref_system_time;

    boost::thread threadRecordIMU;
};
#endif //C_UART_INTERFACE_EXAMPLE_IMU_RECORDER_H

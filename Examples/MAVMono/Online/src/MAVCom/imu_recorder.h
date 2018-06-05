//
// Created by rasp on 1/23/18.
//
#include <iostream>
#include <fstream>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <common/mavlink.h>
#include "autopilot_interface.h"
#include "../../../src/MAV/configParam.h"

#ifndef C_UART_INTERFACE_EXAMPLE_IMU_RECORDER_H
#define C_UART_INTERFACE_EXAMPLE_IMU_RECORDER_H

using namespace MAV;
class IMU_Recorder{
public:
    IMU_Recorder(MAV::ConfigParam *configParam_, bool activate);
    ~IMU_Recorder();

    void start(Autopilot_Interface *autopilot_interface_);
    void stop();
    void set_ref_time(mavlink_system_time_t system_time);
    uint64_t get_ns_time_ref_odroid(uint64_t time_ms);
    void record();

private:
    MAV::ConfigParam *configParam;

    bool active;
    bool time_to_exit;
    Autopilot_Interface *autopilot_interface;
    uint64_t unix_time_ref, boot_time_ref, ref_boot_time_ms, ref_timestampcamera_ns;
    mavlink_system_time_t ref_system_time;

    boost::thread threadRecordIMU;
};
#endif //C_UART_INTERFACE_EXAMPLE_IMU_RECORDER_H

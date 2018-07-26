//
// Created by rasp on 29/4/2561.
//

#ifndef C_UART_IMU_LOCATION_MANAGER_H
#define C_UART_IMU_LOCATION_MANAGER_H

#include <stdio.h>
#include <iostream>
#include <cpr/cpr.h>
#include <nlohmann/json.hpp>
#include <boost/thread.hpp>

#include "geodetic_converter.cpp"
#include <common/mavlink.h>
#include "slam_interface.h"
#include "imu_recorder.h"

class Location_Manager{
public:
    Location_Manager(IMU_Recorder *imu_recorder_, SLAM_Interface *slam_interface_);
    ~Location_Manager();

    bool initializeGeodetic;
    double init_nedx, init_nedy, init_nedz;
    void set_initial_geodetic_pose();
    void get_NED_from_geodetic(double lat, double lon, double alt, float *x, float *y, float *z);

    void set_local_position(uint32_t timestamp, double x, double y, double z);
    void set_global_position(uint32_t timestamp, double lat, double lon, double alt);
    void set_imu_pose(uint32_t timestamp, mavlink_highres_imu_t imu);
    bool isGeodeticInitialize();

    double distanceInKmBetweenEarthCoordinates(double lat1, double lon1, double lat2, double lon2);

    double interpolate(uint32_t x1, uint32_t x2, uint32_t x_predict, double y1, double y2);
    void stream_global_position(uint32_t timestamp, double lat, double lon, double alt);

    void set_ref_time(mavlink_system_time_t system_time);
    void get_ns_ref_time(uint64_t time_ms);
private:
    geodetic_converter::GeodeticConverter *geodeticConverter;

    bool time_to_exit;
    double cx,cy,cz;
    double c_lat, c_lon, c_alt;
    uint32_t c_local_timestamp, c_global_timestamp;

    mavlink_system_time_t ref_system_time;

    pthread_mutex_t mutex_localpose, mutex_globalpose;
    IMU_Recorder *imu_recorder;
    SLAM_Interface *slam_interface;
};

#endif //C_UART_IMU_LOCATION_MANAGER_H

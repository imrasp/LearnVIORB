//
// Created by rasp on 24/6/2561.
//
#ifndef __SLAM_INTERFACE_H__
#define __SLAM_INTERFACE_H__


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>

#include"../../../../../include/System.h"
#include "../../../../../src/IMU/imudata.h"
#include "configParam.h"

using namespace std;

class Location_Manager;

class SLAM_Interface {
public:
    SLAM_Interface(ConfigParam *configParam_);
    ~SLAM_Interface();

    int start();
    int stop();

    void set_location_manager(Location_Manager *location_manager_);

    void add_imu_to_queue(uint64_t timestamp, double xacc, double yacc, double zacc, double xgyro, double ygyro, double zgyro);

    std::vector<ORB_SLAM2::IMUData> vimuData;

    void set_current_frame(cv::Mat frame, uint64_t timestamp);

private:
    ConfigParam *mavconfigParam;
    bool time_to_exit;

    boost::thread threadSLAM;
    ORB_SLAM2::System *SLAM;
    ORB_SLAM2::ConfigParam *config;

    Location_Manager *location_manager;

    // mutex with condition signal
    pthread_cond_t unEmptyIMU, emptyIMU;
    pthread_mutex_t mutexIMU;
    pthread_cond_t grabFrame, noFrame;
    pthread_mutex_t mutexFrame;

    //current frame
    cv::Mat c_frame;
    double c_frame_timestamp;

    bool bAccMultiply98;
    double imageMsgDelaySec;
    const double g3dm = 9.80665;

};

#endif
//
// Created by rasp on 24/6/2561.
//

#ifndef ORB_SLAM2_SLAM_INTERFACE_H
#define ORB_SLAM2_SLAM_INTERFACE_H

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <eigen3/Eigen/Core>

#include "camera_recorder.h"
#include "imu_recorder.h"
#include"../../../../../include/System.h"
#include "../../../../../src/IMU/imudata.h"
#include "../../../../../src/IMU/configparam.h"
#include "../../../../../src/MAV/configParam.h"

using namespace std;
using namespace MAV;

class SLAM_Interface {
public:
    SLAM_Interface(ConfigParam *configParam_);
    ~SLAM_Interface();

    int start(Camera_Recorder *camera_recorder, IMU_Recorder *imu_recorder, std::string configfile);
    int stop();
    void set_imu_pose(uint32_t timestamp, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro);

private:
    MAV::ConfigParam *mavconfigParam;
    bool time_to_exit;

    std::vector<ORB_SLAM2::IMUData> vimuData;

};

#endif //ORB_SLAM2_SLAM_INTERFACE_H

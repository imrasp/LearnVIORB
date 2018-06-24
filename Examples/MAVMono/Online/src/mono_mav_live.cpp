//
// Created by rasp on 17/4/2561.
//

#include<iostream>
#include<algorithm>
#include<fstream>
#include <iostream>
#include<chrono>
#include <unistd.h>

#include"../../../../include/System.h"

#include "../../../../src/IMU/imudata.h"
#include "../../../../src/IMU/configparam.h"
#include "../../../../src/MAV/configParam.h"

#include "MAVCom/mavlink_control.h"
#include "MAVCom/camera_recorder.h"
#include "MAVCom/log.h"
#include "MAVCom/imu_recorder.h"
#include "MAVCom/slam_interface.h"

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <eigen3/Eigen/Core>
#include <iostream>

using namespace std;

int main(int argc, char **argv);
int slamThread(std::string configfile);
inline bool exists_test (const std::string& name);

int main(int argc, char **argv) {
    try {
        std::cout << "\nConfigurating params \n";
        MAV::ConfigParam configParam(argv[1]);

        //Check is configuration file is exist or not
        {
            if(!exists_test(configParam.mission_route)) {
                cout << "Route file is not exist, Please upload route.";
                return 0;
            }
            if (!exists_test(configParam.vocabulary)) {
                cout << "Vocabulary file is not exist, Please contract admin.";
                return 0;
            }
        }

        std::cout << "\nCreating log folder \n";
        Log log(configParam.record_path);
        std::cout << "\nStart IMU recorder thread \n";
        IMU_Recorder imu_recorder(&configParam, true);
        std::cout << "\nStart Mavlink_Control\n";
        Mavlink_Control mavlinkControl(&configParam, &imu_recorder);
        Camera_Recorder cameraRecorder(&configParam);
        SLAM_Interface slam(&configParam);

        //create SLAM thread
        if(configParam.bEnableIMU) {
            int result = mavlinkControl.start();
            if (result == -1) {
                cout << "Mission failed! Location of first waypoint in the route is to far from current location.";
                return 0;
            }
        }

        if(configParam.bEnableCamera) {
            int con = cameraRecorder.start();
            if(!con){
                cout << "Camera is not working. Please turn on the camera.";
                return 0;
            }
        }

        if(configParam.bEnableSLAM) {
            slam.start(&cameraRecorder, &imu_recorder, argv[1]);
        }

        mavlinkControl.cmd();

        if(configParam.bEnableSLAM) slam.stop();

        if(configParam.bEnableCamera) cameraRecorder.stop();

        if(configParam.bEnableIMU) imu_recorder.stop();

        mavlinkControl.stop();

        cout << "Mission Complete!";
        return 1;
    }
    catch (int error) {
        fprintf(stderr, "threw exception %i \n", error);
        return error;
    }
}

inline bool exists_test (const std::string& name) {
    ifstream f(name.c_str());
    return f.good();
}


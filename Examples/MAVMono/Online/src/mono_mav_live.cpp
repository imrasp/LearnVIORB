//
// Created by rasp on 17/4/2561.
//

#include<iostream>
#include<algorithm>
#include<fstream>
#include <iostream>
#include<chrono>

#include"../../../../include/System.h"

#include "../../../../src/IMU/imudata.h"
#include "../../../../src/IMU/configparam.h"
#include "../../../../src/MAV/configParam.h"

#include "MAVCom/mavlink_control.h"
#include "MAVCom/camera_recorder.h"
#include "MAVCom/log.h"
#include "MAVCom/imu_recorder.h"

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <eigen3/Eigen/Core>
#include <iostream>

using namespace std;

int main(int argc, char **argv);
int slamThread(std::string configfile);

int main(int argc, char **argv) {
    try {

        bool bCamera = true;
        bool bIMU = true;
        bool bSLAM = false;

//        std::cout << "\nConfigurating params \n";
//        MAV::ConfigParam configParam(argv[1]);
//        std::cout << "\nCreating log folder \n";
//        Log log(configParam.record_path);
//        std::cout << "\nStart IMU recorder thread \n";
//        IMU_Recorder imu_recorder(&configParam, true);
//        std::cout << "\nStart Mavlink_Control\n";
//        Mavlink_Control mavlinkControl(&configParam, &imu_recorder);
//        Camera_Recorder cameraRecorder(&configParam);
//
//        //create SLAM thread
//
//        if(bIMU) mavlinkControl.start();
//
//        if(bCamera) cameraRecorder.start();
//
//        mavlinkControl.cmd();
//
//        if(bCamera) cameraRecorder.stop();
//
//        if(bIMU) imu_recorder.stop();
//
//        mavlinkControl.stop();

        return 0;
    }
    catch (int error) {
        fprintf(stderr, "threw exception %i \n", error);
        return error;
    }
}

int slamThread(std::string configfile){
//    std::cout << "Starting SLAM...\n";
//    // Create SLAM system. It initializes all system threads and gets ready to process frames.
//    ORB_SLAM2::System SLAM(configmav.vocabulary, argv[1], ORB_SLAM2::System::MONOCULAR, true);
//
//    std::cout << "Reading SLAM config file...\n";
//    ORB_SLAM2::ConfigParam config(argv[1]);
//    std::cout << "Finish reading SLAM config file...\n";
//    std::cout << "Checking for dataset...\n";
//    //Checking files
//    boost::filesystem::path dir(configmav.record_path + "/cam0");
//    if (!(boost::filesystem::exists(dir))) {
//        std::cout << "Doesn't Exists" << std::endl;
//        throw 1;
//    }
//    std::ifstream imu(configmav.record_path + "/imu0.csv");
//    if (!imu.is_open()) {
//        std::cout << "ERROR: Cannot Open IMU File" << std::endl;
//        throw 1;
//    }
//
//    //create result file
//    std::ofstream visionpose;
//    visionpose.open(configmav.record_path + "/slam_pose.csv");
//    // Read all images from folder order by name(default)
//    std::vector<cv::String> fn;
//    cv::glob(configmav.record_path + "/cam0/*.png", fn, false);
//
//    cv::Mat image;
//    double xgyro, ygyro, zgyro, ax, ay, az, timestamp;
//    std::string getval;
//    cv::Mat vision_estimated_pose;
//    static double startT=-1;
//
//    /**
//    * @brief added data sync
//    */
//    double imageMsgDelaySec = config.GetImageDelayToIMU();
//    bool bAccMultiply98 = config.GetAccMultiply9p8();
//    // 3dm imu output per g. 1g=9.80665 according to datasheet
//    const double g3dm = 9.80665;
//    bool isFirstFrame = false;
    return 0;
}

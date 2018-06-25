//
// Created by rasp on 17/4/2561.
//

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<cstdio>//to pause console screen
#include <typeinfo>


#include"../../../../include/System.h"

#include "../../../../src/IMU/imudata.h"
#include "../../../../src/IMU/configparam.h"
#include "../../../../src/MAV/configParam.h"

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <eigen3/Eigen/Core>

using namespace std;

int main(int argc, char **argv) {

    if(argc != 2)
    {
        cerr << endl << "Usage: ./ORB_SLAM2 Mono path_to_settings" << endl;
        return 1;
    }
    std::cout << "Reading MAV config file...\n";
    MAV::ConfigParam configmav(argv[1]);
   std::cout << "Starting SLAM...\n";
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(configmav.vocabulary, argv[1], ORB_SLAM2::System::MONOCULAR, true);

    std::cout << "Reading SLAM config file...\n";
    ORB_SLAM2::ConfigParam config(argv[1]);
    std::cout << "Finish reading SLAM config file...\n";
    std::cout << "Checking for dataset...\n";
    //Checking files
    boost::filesystem::path dir(configmav.record_path + "/cam0");
    if (!(boost::filesystem::exists(dir))) {
        std::cout << "Doesn't Exists" << std::endl;
        throw 1;
    }
    std::ifstream imu(configmav.record_path + "/imu0.csv");
    if (!imu.is_open()) {
        std::cout << "ERROR: Cannot Open IMU File" << std::endl;
        throw 1;
    }

    //create result file
    std::ofstream visionpose;
    visionpose.open(configmav.record_path + "/slam_pose.csv");
    // Read all images from folder order by name(default)
    std::vector<cv::String> fn;
    cv::glob(configmav.record_path + "/cam0/*.png", fn, false);

    cv::Mat image;
    double xgyro, ygyro, zgyro, ax, ay, az, timestamp;
    std::string getval;
    cv::Mat vision_estimated_pose;
    static double startT=-1;

    /**
    * @brief added data sync
    */
    double imageMsgDelaySec = config.GetImageDelayToIMU();
    bool bAccMultiply98 = config.GetAccMultiply9p8();
    // 3dm imu output per g. 1g=9.80665 according to datasheet
    const double g3dm = 9.80665;
    bool isFirstFrame = false;

    // load each frame with timestamp from name along with IMU data
    size_t count = fn.size(); //number of png files in images folder
    for (size_t i=0; i<count; i++) {
        std::string stimestamp = fn[i].substr(fn[i].find_last_of("/") + 1, 19);
        double timestamp_camera = std::stod(stimestamp) / 1e9;
        image = cv::imread(fn[i]);

        std::vector<ORB_SLAM2::IMUData> vimuData;

        while (imu.good()) {
            getline(imu, getval, ',');
            timestamp = atof(getval.c_str()) / 1e9; //cout << " timestamp : " << timestamp << endl;
            getline(imu, getval, ',');
            xgyro = atof(getval.c_str()); // cout << " xgyro : " << xgyro << endl;
            getline(imu, getval, ',');
            ygyro = atof(getval.c_str()); // cout << " ygyro : " << ygyro << endl;
            getline(imu, getval, ',');
            zgyro = atof(getval.c_str()); // cout << " zgyro : " << zgyro << endl;
            getline(imu, getval, ',');
            ax = atof(getval.c_str()); // cout << " xacc : " << xacc << endl;
            getline(imu, getval, ',');
            ay = atof(getval.c_str()); // cout << " yacc : " << yacc << endl;
            getline(imu, getval, '\n');
            az = atof(getval.c_str()); // cout << " zacc : " << zacc << endl;
            if (bAccMultiply98) {
                ax *= g3dm;
                ay *= g3dm;
                az *= g3dm;
            }

            if (timestamp >= timestamp_camera) {
                if(vimuData.size()==0) {
                    cout << "no imu message between images! \n";
                    break;
                }
                // Consider delay of image message

                if(startT<0)
                    startT = timestamp_camera;
                // Below to test relocalizaiton
                //if(imageMsg->header.stamp.toSec() > startT+25 && imageMsg->header.stamp.toSec() < startT+25.3)
                if(timestamp_camera < startT+config._testDiscardTime)
                    image = cv::Mat::zeros(image.rows,image.cols,image.type());

//                std::cout << "-------------------" << '\n';
//                std::cout << std::setprecision(19) << "Lastest IMU timestamp: " << timestamp << '\n';
//                std::cout << "Total Number of IMU: " << vimuData.size() << '\n';
//                std::cout << "-------------------" << '\n';

                //skip first frame
                if (!isFirstFrame) {
                    vimuData.clear();
                    isFirstFrame = true;
                } else {

                    // Check dis-continuity, tolerance 3 seconds
                    if(timestamp_camera - imageMsgDelaySec + 3.0 < vimuData.front()._t)
                    {
                        cout << "Data dis-continuity, > 3 seconds. Buffer cleared \n";
                        vimuData.clear();
                        continue;
                    }

                    // Pass the image to the SLAM system
                    vision_estimated_pose = SLAM.TrackMonoVI(image, vimuData, timestamp_camera - imageMsgDelaySec);
                    cout << "visionpose : " << vision_estimated_pose << endl;

                    if(!vision_estimated_pose.empty()) {
                        visionpose << ',' << vision_estimated_pose.at<double>(0,3)
                                   << ',' << vision_estimated_pose.at<double>(1,3)
                                   << ',' << vision_estimated_pose.at<double>(2,3) << '\n';

//                        float yaw,pitch,roll;
//                        boost::tie(yaw,pitch,roll) = quaternionToYawPitchRoll(vision_estimated_pose);
                    }
                    vimuData.clear();
                    // angular_velocity.x, angular_velocity.y, angular_velocity.z, linear_acceleration ax, ay, az, timestamp
                    ORB_SLAM2::IMUData imudata(xgyro, ygyro, zgyro, ax, ay, az, timestamp);
                    vimuData.push_back(imudata);
                }
                break;

            } else {
                ORB_SLAM2::IMUData imudata(xgyro, ygyro, zgyro, ax, ay, az, timestamp);
                vimuData.push_back(imudata);
            }
        }
    }

    bool bstop = false;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM(configmav.record_path + "/KeyFrameTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryNavState(configmav.record_path + "/KeyFrameNavStateTrajectory.txt");

    cout<<endl<<endl<<"press any key to shutdown"<<endl;
    getchar();

    // Stop all threads
    cout << "shuting down...";
//    SLAM.Shutdown();

    return 0;
}

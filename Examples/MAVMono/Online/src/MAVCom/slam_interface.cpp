//
// Created by rasp on 24/6/2561.
//

#include "slam_interface.h"

SLAM_Interface::SLAM_Interface(ConfigParam *mavconfigParam_) : mavconfigParam(mavconfigParam_) {
    time_to_exit = false;
}

SLAM_Interface::~SLAM_Interface() {

}

int SLAM_Interface::stop() {
    time_to_exit = true;

}

int SLAM_Interface::start(Camera_Recorder *camera_recorder, IMU_Recorder *imu_recorder, std::string configfile) {

    std::cout << "Starting SLAM...\n";
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(mavconfigParam->vocabulary, configfile, ORB_SLAM2::System::MONOCULAR, true);

    std::cout << "Reading SLAM config file...\n";
    ORB_SLAM2::ConfigParam config(configfile);
    std::cout << "Finish reading SLAM config file...\n";

    //create result file
    std::cout << "create SLAM result file \n";
    std::ofstream visionpose;
    visionpose.open(mavconfigParam->record_path + "/slam_pose.csv");

    /**
    * @brief added data sync
    */
    double imageMsgDelaySec = config.GetImageDelayToIMU();
    bool bAccMultiply98 = config.GetAccMultiply9p8();
    // 3dm imu output per g. 1g=9.80665 according to datasheet
    const double g3dm = 9.80665;
//    bool isFirstFrame = false;
    uint64_t timestampcamera_ns;
    cv::Mat matFrame;
    std::vector<ORB_SLAM2::IMUData> vimuData;

    std::cout << "grab first frame \n";
    while (!time_to_exit) {
        // grab image
        int OldPrio = 0;
        pthread_mutex_setprioceiling(&camera_recorder->_mutexFrameCam1Last, 2, &OldPrio);
        pthread_mutex_lock(&camera_recorder->_mutexFrameCam1Last);

//        pthread_cond_wait(&camera_recorder->grabaFrame, &camera_recorder->_mutexGrabFrame);
//        std::queue<cv::Mat> images = camera_recorder->copy_image_queue();
//        std::queue<uint64_t> times = camera_recorder->copy_time_queue();

//        matFrame = images.back();
//        timestampcamera_ns = times.back();
        matFrame = camera_recorder->matFrameForward;
        timestampcamera_ns = camera_recorder->timestampcamera_ns;
        std::cout << "grab frame for SLAM \n";
        pthread_mutex_unlock(&camera_recorder->_mutexFrameCam1Last);

        // grab IMUs
        std::queue<mavlink_highres_imu_t> imu = imu_recorder->copy_queue();
        std::cout << "processing IMU queue... \n";
        while (imu_recorder->get_ns_time_ref_odroid(imu.front().time_usec) <= timestampcamera_ns) {
            float xgyro = imu.front().xgyro;
            float ygyro = imu.front().ygyro;
            float zgyro = imu.front().zgyro;
            float xacc = imu.front().xacc;
            float yacc = imu.front().yacc;
            float zacc = imu.front().zacc;
            if (bAccMultiply98) {
                xacc *= g3dm;
                yacc *= g3dm;
                zacc *= g3dm;
            }
            ORB_SLAM2::IMUData imudata(xgyro, ygyro, zgyro, xacc, yacc, zacc, imu_recorder->get_ns_time_ref_odroid(imu.front().time_usec));
            vimuData.push_back(imudata);

            imu_recorder->write_first();
            imu.pop();

        }

        if (vimuData.size() == 0) {
            cout << "no imu message between images! \n";
            break;
        }

        // Check dis-continuity, tolerance 3 seconds
        if (timestampcamera_ns - imageMsgDelaySec + 3.0 < vimuData.front()._t) {
            cout << "Data dis-continuity, > 3 seconds. Buffer cleared \n";
            vimuData.clear();
            continue;
        }

        std::cout << "-------------------" << '\n';
//        std::cout << std::setprecision(19) << "Lastest IMU timestamp: " << vimuData.back().time_usec << '\n';
        std::cout << "Total Number of IMU: " << vimuData.size() << '\n';
        std::cout << "-------------------" << '\n';

        // Pass the image to the SLAM system
        cv::Mat vision_estimated_pose = SLAM.TrackMonoVI(matFrame, vimuData, timestampcamera_ns - imageMsgDelaySec);

//        if (!vision_estimated_pose.empty()) {
//            visionpose << ',' << vision_estimated_pose.at<double>(0, 3)
//                       << ',' << vision_estimated_pose.at<double>(1, 3)
//                       << ',' << vision_estimated_pose.at<double>(2, 3) << '\n';
        vimuData.clear();
    }

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM(mavconfigParam->record_path + "/KeyFrameTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryNavState(mavconfigParam->record_path + "/KeyFrameNavStateTrajectory.txt");

    // Stop all threads
    SLAM.Shutdown();

    return 1;
}



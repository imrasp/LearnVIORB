//
// Created by rasp on 24/6/2561.
//

#include "slam_interface.h"

SLAM_Interface::SLAM_Interface(ConfigParam *mavconfigParam_) : mavconfigParam(mavconfigParam_), time_to_exit(false) {

    unEmptyIMU = PTHREAD_COND_INITIALIZER;
    emptyIMU = PTHREAD_COND_INITIALIZER;
    mutexIMU = PTHREAD_MUTEX_INITIALIZER;
    grabFrame = PTHREAD_COND_INITIALIZER;
    noFrame = PTHREAD_COND_INITIALIZER;
    mutexFrame = PTHREAD_MUTEX_INITIALIZER;

    cout << "Starting SLAM..." << endl;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    SLAM = new ORB_SLAM2::System(mavconfigParam->vocabulary, mavconfigParam->setting, ORB_SLAM2::System::MONOCULAR, false);
    config = new ORB_SLAM2::ConfigParam(mavconfigParam->setting);

    imageMsgDelaySec = config->GetImageDelayToIMU();
    // ORBVIO::MsgSynchronizer msgsync(imageMsgDe laySec);
    bAccMultiply98 = config->GetAccMultiply9p8();


    cout << "Start SLAM thread..." << endl;
    threadSLAM = boost::thread(&SLAM_Interface::start, this);
}

SLAM_Interface::~SLAM_Interface() {
    stop();
    threadSLAM.join();
}

int SLAM_Interface::stop() {
    time_to_exit = true;


}

void SLAM_Interface::set_current_frame(cv::Mat frame, uint64_t timestamp) {
//    pthread_mutex_lock(&mutexFrame);
//    c_frame = frame.clone();
//    c_frame_timestamp = timestamp * 1e-9;
//    pthread_cond_signal(&grabFrame);
//    pthread_mutex_unlock(&mutexFrame);
}

void SLAM_Interface::add_imu_to_queue(uint64_t timestamp, double xacc, double yacc, double zacc,
                                    double xgyro, double ygyro, double zgyro) {
    if (bAccMultiply98) {
        xacc *= g3dm;
        yacc *= g3dm;
        zacc *= g3dm;
    }

    // add IMU to queue
    pthread_mutex_lock(&mutexIMU);
    ORB_SLAM2::IMUData imudata(xgyro, ygyro, zgyro, xacc, yacc, zacc, timestamp * 1e-9);
    vimuData.push_back(imudata);
    pthread_cond_signal(&unEmptyIMU);
    pthread_mutex_unlock(&mutexIMU);
}


int SLAM_Interface::start() {

    //create result file
    std::cout << "create SLAM result file \n";
    std::ofstream visionpose;
    visionpose.open(mavconfigParam->record_path + "/slam_pose.csv");

    while (!time_to_exit) {

        pthread_mutex_lock(&mutexFrame);
        pthread_mutex_lock(&mutexIMU);

        if (c_frame.rows == 0 || c_frame.cols == 0) {
            pthread_cond_wait(&grabFrame, &mutexIMU);
        }
        if (vimuData.size() == 0) {
            cout << "skip frame imu size = 0 \n";
            continue;
        }

        std::cout << "-------------------" << '\n';
        std::cout << std::setprecision(19) << "Frame timestamp: " << c_frame_timestamp << '\n';
        std::cout << std::setprecision(19) << "Lastest IMU timestamp: " << vimuData.back()._t << '\n';
        std::cout << "Total Number of IMU: " << vimuData.size() << '\n';
        std::cout << "-------------------" << '\n';

        cv::Mat vision_estimated_pose = SLAM->TrackMonoVI(c_frame, vimuData, c_frame_timestamp - imageMsgDelaySec);
        vimuData.clear();

        pthread_mutex_unlock(&mutexFrame);
        pthread_mutex_unlock(&mutexIMU);
    }

    // Save camera trajectory
    SLAM->SaveKeyFrameTrajectoryTUM(mavconfigParam->record_path + "/KeyFrameTrajectory.txt");
    SLAM->SaveKeyFrameTrajectoryNavState(mavconfigParam->record_path + "/KeyFrameNavStateTrajectory.txt");

    // Stop all threads
    SLAM->Shutdown();

    visionpose.close();

    return 1;
}

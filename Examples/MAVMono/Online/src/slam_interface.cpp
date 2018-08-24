//
// Created by rasp on 24/6/2561.
//

#include "slam_interface.h"
#include "location_manager.h"

SLAM_Interface::SLAM_Interface(ConfigParam *mavconfigParam_) : mavconfigParam(mavconfigParam_), time_to_exit(false) {

    unEmptyIMU = PTHREAD_COND_INITIALIZER;
    emptyIMU = PTHREAD_COND_INITIALIZER;
    mutexIMU = PTHREAD_MUTEX_INITIALIZER;
    grabFrame = PTHREAD_COND_INITIALIZER;
    noFrame = PTHREAD_COND_INITIALIZER;
    mutexFrame = PTHREAD_MUTEX_INITIALIZER;

    if (mavconfigParam->useSLAM || mavconfigParam->cameraid == 99) {
        cout << "Starting SLAM..." << endl;
        // Create SLAM system. It initializes all system threads and gets ready to process frames.
        SLAM = new ORB_SLAM2::System(mavconfigParam->vocabulary, mavconfigParam->setting, ORB_SLAM2::System::MONOCULAR,
                                     mavconfigParam->gui);
        config = new ORB_SLAM2::ConfigParam(mavconfigParam->setting);

        imageMsgDelaySec = config->GetImageDelayToIMU();
        // ORBVIO::MsgSynchronizer msgsync(imageMsgDe laySec);
        bAccMultiply98 = config->GetAccMultiply9p8();


        cout << "Start SLAM thread..." << endl;
        threadSLAM = boost::thread(&SLAM_Interface::start, this);
    }
}

SLAM_Interface::~SLAM_Interface() {
    stop();
}

void SLAM_Interface::set_location_manager(Location_Manager *location_manager_) {
    location_manager = location_manager_;
}

int SLAM_Interface::stop() {
    time_to_exit = true;

    // Save camera trajectory
    SLAM->SaveKeyFrameTrajectoryTUM(mavconfigParam->record_path + "/KeyFrameTrajectory.txt");
    SLAM->SaveKeyFrameTrajectoryNavState(mavconfigParam->record_path + "/KeyFrameNavStateTrajectory.txt");

    // Stop all threads
    SLAM->Shutdown();

    threadSLAM.join();
}

void SLAM_Interface::set_current_frame(cv::Mat frame, uint64_t timestamp) {
//    pthread_mutex_lock(&mutexFrame);
//    std::cout << "SLAM :: get frame \n";
    c_frame = frame.clone();
    c_frame_timestamp = timestamp * 1e-9;
    pthread_cond_signal(&grabFrame);
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
//    pthread_mutex_lock(&mutexIMU);
//    std::cout << "SLAM :: add IMU to queue \n";
    ORB_SLAM2::IMUData imudata(xgyro, ygyro, zgyro, xacc, yacc, zacc, timestamp * 1e-9);
    vimuData.push_back(imudata);
    pthread_cond_signal(&unEmptyIMU);
//    pthread_mutex_unlock(&mutexIMU);
}


int SLAM_Interface::start() {

    //create result file
    std::cout << "create SLAM result file \n";
    std::ofstream visionpose, processing_time;
    visionpose.open(mavconfigParam->record_path + "/slam_pose.csv");
    processing_time.open(mavconfigParam->record_path + "/processing_time.csv");
    processing_time << "t1" << "," << "t2" << "," << "dt" << "," << "state" << std::endl;

    while (!time_to_exit) {
        cout << "SLAM :: process frame \n";
            pthread_cond_wait(&grabFrame, &mutexFrame);
            pthread_cond_wait(&unEmptyIMU, &mutexIMU);
        if (vimuData.size() < 5) {
            cout << "skip frame imu size < 5 \n";
            continue;
        }
//        pthread_mutex_lock(&mutexFrame);
//        pthread_mutex_lock(&mutexIMU);
        cout << "SLAM :: TrackMonoVI \n";
        std::cout << "-------------------" << '\n';
        std::cout << std::setprecision(19) << "Frame timestamp: " << c_frame_timestamp << '\n';
        std::cout << std::setprecision(19) << "Lastest IMU timestamp: " << vimuData.back()._t << '\n';
        std::cout << "Total Number of IMU: " << vimuData.size() << '\n';
        std::cout << "-------------------" << '\n';

        uint64_t  cp_time1 = (boost::lexical_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count()));

        cv::Mat vision_estimated_pose = SLAM->TrackMonoVI(c_frame, vimuData, c_frame_timestamp - imageMsgDelaySec);
        vimuData.clear();

        uint64_t  cp_time2 = (boost::lexical_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count()));

//        pthread_mutex_unlock(&mutexFrame);
//        pthread_mutex_unlock(&mutexIMU);
        pthread_cond_signal(&noFrame);
        pthread_cond_signal(&emptyIMU);

        /* Tracking states
         SYSTEM_NOT_READY=-1,
         NO_IMAGES_YET=0,
         NOT_INITIALIZED=1,
         OK=2,
         LOST=3
         */
//        visionpose <<
        processing_time << cp_time1 << "," << cp_time2 << "," << cp_time2 - cp_time1 << "," << SLAM->get_state() << std::endl;

    }



    visionpose.close();

    return 1;
}

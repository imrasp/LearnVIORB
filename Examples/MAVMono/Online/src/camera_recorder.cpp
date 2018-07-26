//
// Created by rasp on 1/20/18.
//
#include "camera_recorder.h"

Camera_Recorder::Camera_Recorder(int camid_, int time_offset_, bool bViewer_, SLAM_Interface *slam_interface_, string record_path) : camid(camid_),
                                                                                time_offset(time_offset_),
                                                                                bViewer(bViewer_), slam_interface(slam_interface_) {
    lframe.open(record_path + "/frame_cam" + std::to_string(camid) + ".csv");
    lframe << "timestamp" << "\n";
}

Camera_Recorder::~Camera_Recorder() {
    stop();
}

void Camera_Recorder::initializeCamera() {
    stream1 = cv::VideoCapture(camid);
    if (!stream1.isOpened()) throw 30; //return 0;
//    query_maximum_resolution(&stream1, max_width, max_height);
//    max_width = 1280; max_height = 720;
//    max_width = 640; max_height = 480;
    max_width = 848;
    max_height = 480;

};

//find maximum resolution
void Camera_Recorder::query_maximum_resolution(cv::VideoCapture *camera, int &max_width, int &max_height) {
    // Save current resolution
    const int current_width = static_cast<int>(camera->get(CV_CAP_PROP_FRAME_WIDTH));
    const int current_height = static_cast<int>(camera->get(CV_CAP_PROP_FRAME_HEIGHT));

    // Get maximum resolution
    camera->set(CV_CAP_PROP_FRAME_WIDTH, 10000);
    camera->set(CV_CAP_PROP_FRAME_HEIGHT, 10000);
    max_width = static_cast<int>(camera->get(CV_CAP_PROP_FRAME_WIDTH));
    max_height = static_cast<int>(camera->get(CV_CAP_PROP_FRAME_HEIGHT));

    // Restore resolution
    camera->set(CV_CAP_PROP_FRAME_WIDTH, current_width);
    camera->set(CV_CAP_PROP_FRAME_HEIGHT, current_height);
}

// check is 2 frames is difference or not
double Camera_Recorder::frameDifference(cv::Mat &matFrameCurrent, cv::Mat &matFramePrevious) {
    double diff = 0.0;
    assert(matFrameCurrent.rows > 0 && matFrameCurrent.cols > 0);
    assert(
            matFrameCurrent.rows == matFramePrevious.rows
            && matFrameCurrent.cols == matFramePrevious.cols);
    assert(
            matFrameCurrent.type() == CV_8U && matFramePrevious.type() == CV_8U);
    for (int i = 0; i < matFrameCurrent.rows; i++) {
        for (int j = 0; j < matFrameCurrent.cols; j++) {
            diff += matFrameCurrent.at<cv::Vec3b>(i, j)[1] - matFramePrevious.at<cv::Vec3b>(i, j)[1];
        }
    }
    return diff;
}

void Camera_Recorder::cameraLoop() {
    int totalFrame = 0;

//    stream1.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('Y', 'U', 'Y', 'V') );
    stream1.set(CV_CAP_PROP_FRAME_WIDTH, max_width);
    stream1.set(CV_CAP_PROP_FRAME_HEIGHT, max_height);
//    stream1.set(CV_CAP_PROP_CONVERT_RGB , false);
    stream1.set(CV_CAP_PROP_FPS, configParam->fps);

    while (!time_to_exit) {
        std::cout << "";
        std::cout << "read frame \n";
        stream1 >> matFrameForward;
        timestampcamera_ns = (boost::lexical_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count())) - (time_offset * 1e9);
        //std::cout << "frame @ timestamp = " << timestampcamera_ns << std::endl;
//
        cv::cvtColor(matFrameForward, matFrameForward, CV_BGR2GRAY);
        matFrameForward.convertTo(matFrameForward, CV_8U);

//        if(slam_interface != nullptr)
//        slam_interface->set_current_frame(matFrameForward, timestampcamera_ns);

        pthread_mutex_lock(&_mutexFrameCam1Last);
        qFrame.push(matFrameForward);
        qTime.push(timestampcamera_ns);
        pthread_cond_signal(&frameQueueCondNotempty);
        pthread_mutex_unlock(&_mutexFrameCam1Last);

        if (bViewer) {
            std::cout << "read matFrameForward size : " << matFrameForward.size() << std::endl;
            cv::imshow("Camera", matFrameForward);
            if (cv::waitKey(1) >= 0) break;
        }

        totalFrame++;
    }
    std::cout << "#Frame = " << totalFrame << std::endl;
}

void Camera_Recorder::cameraRecord() {

    int totalRecord = 0;
    cv::Mat recFrameForward, lastestFrameForward;
    uint64_t timestampcamera;

    while (!time_to_exit || !qFrame.empty()) {
        std::cout << "";

        if (qFrame.empty()) {
            pthread_mutex_lock(&_mutexFrameCam1Last);
            pthread_cond_wait(&frameQueueCondNotempty, &_mutexFrameCam1Last);
            pthread_mutex_unlock(&_mutexFrameCam1Last);
        } else {
            recFrameForward = qFrame.front();
            timestampcamera = qTime.front();
            qFrame.pop();
            qTime.pop();

            imwrite("./record_data/cam0/" + std::to_string(timestampcamera) + ".png", recFrameForward);
            lframe << timestampcamera << "\n";
            totalRecord++;

            recFrameForward.copyTo(lastestFrameForward);
        }

    }
    std::cout << "#Record = " << totalRecord << std::endl;
}

void Camera_Recorder::start() {
    // initilize camera parameter
    initializeCamera();

    //create camera thread
    std::cout << "Start camera thread..." << std::endl;
    threadCamera = boost::thread(&Camera_Recorder::cameraLoop, this);

    // create record thread
    std::cout << "Start record thread..." << std::endl;
    threadRecord = boost::thread(&Camera_Recorder::cameraRecord, this);
}

void Camera_Recorder::stop() {
    //join thread
    time_to_exit = true;
    lframe.close();
    stream1.release();
    threadCamera.join();
    threadRecord.join();
    std::cout << "Finish recording frames." << std::endl;
}

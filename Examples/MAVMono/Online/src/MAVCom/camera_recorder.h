//
// Created by rasp on 1/20/18.
//

#ifndef C_UART_INTERFACE_EXAMPLE_CAMERA_RECORDER_H
#define C_UART_INTERFACE_EXAMPLE_CAMERA_RECORDER_H

#include <iostream>
#include <zconf.h>
#include <chrono>
#include <fstream>
#include <queue>

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../../../src/MAV/configParam.h"

using namespace MAV;
class Camera_Recorder {
public:
    Camera_Recorder(ConfigParam *configParam_);
    ~Camera_Recorder();

    int initializeCamera();
    void query_maximum_resolution(cv::VideoCapture *camera, int &max_width, int &max_height);
    double frameDifference(cv::Mat &matFrameCurrent, cv::Mat &matFramePrevious);
    void cameraRecord();
    void cameraLoop();
    void action();
    int start();
    void stop();

    cv::VideoCapture stream1;
    uint64_t timestampcamera_ns;
    cv::Mat matFrameForward;
    std::ofstream lframe;
    bool time_to_exit;
    int max_width, max_height;
    std::queue<uint64_t> qTime;
    std::queue<cv::Mat> qFrame;

    cv::Mat current_image;

    pthread_mutex_t _mutexGrabFrame =  PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t grabaFrame = PTHREAD_COND_INITIALIZER;

    std::queue<cv::Mat> copy_image_queue();
    std::queue<uint64_t> copy_time_queue();

    pthread_mutex_t _mutexFrameCam1Last = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t frameQueueCondNotempty = PTHREAD_COND_INITIALIZER;
    pthread_cond_t frameQueueCondEmpty = PTHREAD_COND_INITIALIZER;

private:
    ConfigParam *configParam;

    boost::thread threadCamera, threadRecord;

};
#endif //C_UART_INTERFACE_EXAMPLE_CAMERA_RECORDER_H

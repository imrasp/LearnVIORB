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

#include "src/configParam.h"
#include "src/slam_interface.h"


class Camera_Recorder {
public:
    Camera_Recorder(int camid, int time_offset, bool bViewer_, SLAM_Interface *slam_interface_, string record_path);
    ~Camera_Recorder();

    void initializeCamera();
    void query_maximum_resolution(cv::VideoCapture *camera, int &max_width, int &max_height);
    double frameDifference(cv::Mat &matFrameCurrent, cv::Mat &matFramePrevious);
    void cameraRecord();
    void cameraLoop();
    void action();
    void start();
    void stop();

    cv::VideoCapture stream1;
    uint64_t timestampcamera_ns;
    cv::Mat matFrameForward;
    std::ofstream lframe;
    bool time_to_exit;
    int max_width, max_height;
    std::queue<uint64_t> qTime;
    std::queue<cv::Mat> qFrame;
    bool bViewer;

private:
    ConfigParam *configParam;
    SLAM_Interface *slam_interface;
    int camid;
    int time_offset;
    string record_path;

    boost::thread threadCamera, threadRecord;
    pthread_mutex_t _mutexFrameCam1Last = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t frameQueueCondNotempty = PTHREAD_COND_INITIALIZER;
    pthread_cond_t frameQueueCondEmpty = PTHREAD_COND_INITIALIZER;

};
#endif //C_UART_INTERFACE_EXAMPLE_CAMERA_RECORDER_H

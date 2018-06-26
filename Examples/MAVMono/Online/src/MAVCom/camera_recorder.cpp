//
// Created by rasp on 1/20/18.
//
#include "camera_recorder.h"

Camera_Recorder::Camera_Recorder(ConfigParam *configParam_):configParam(configParam_){

}

Camera_Recorder::~Camera_Recorder() {}

int Camera_Recorder::initializeCamera() {
    stream1 = cv::VideoCapture(configParam->cameraid);
    if(!stream1.isOpened()) return 0;
//    query_maximum_resolution(&stream1, max_width, max_height);
//    max_width = 1280; max_height = 720;
//    max_width = 640; max_height = 480;
    max_width = 848; max_height = 480;

    lframe.open("./record_data/frame.csv");
    lframe << "timestamp" << "\n";
    return 1;
}

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

std::queue<cv::Mat> Camera_Recorder::copy_image_queue(){
    return qFrame;
}

std::queue<uint64_t> Camera_Recorder::copy_time_queue(){
    return qTime;
}
void Camera_Recorder::cameraLoop() {
    int totalFrame = 0;

    //stream1.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('Y', 'U', 'Y', 'V') );
    stream1.set(CV_CAP_PROP_FRAME_WIDTH, max_width);
    stream1.set(CV_CAP_PROP_FRAME_HEIGHT, max_height);
//    stream1.set(CV_CAP_PROP_CONVERT_RGB , false);
    stream1.set(CV_CAP_PROP_FPS, configParam->fps);

    while (!time_to_exit) {

        pthread_mutex_lock(&_mutexGrabFrame);
        stream1 >> matFrameForward;
        timestampcamera_ns = boost::lexical_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        //std::cout << "frame @ timestamp = " << timestampcamera_ns << std::endl;
        matFrameForward.convertTo(matFrameForward, CV_8U);
        cv::cvtColor(matFrameForward, matFrameForward, CV_BGR2GRAY);
        pthread_cond_signal(&grabaFrame);
        pthread_mutex_unlock(&_mutexGrabFrame);

        pthread_mutex_lock(&_mutexFrameCam1Last);
        qFrame.push(matFrameForward);
        qTime.push(timestampcamera_ns);

        pthread_cond_signal(&frameQueueCondNotempty);
        pthread_mutex_unlock(&_mutexFrameCam1Last);

//        std::cout << "read matFrameForward size : " << matFrameForward.size() << std::endl;
//        cv::imshow("Camera", matFrameForward);
//        if (cv::waitKey(1) >= 0) break;

        totalFrame++;
    }
    std::cout << "#Frame = " << totalFrame << std::endl;
}

void Camera_Recorder::cameraRecord() {

    int totalRecord = 0;
    cv::Mat recFrameForward, lastestFrameForward;
    uint64_t timestampcamera;

    while (!time_to_exit) {
//        if (matFrameForward.cols != max_width) continue;

        int OldPrio = 0;
        pthread_mutex_setprioceiling(&_mutexFrameCam1Last, 0, &OldPrio);
        pthread_mutex_lock(&_mutexFrameCam1Last);
        while (qFrame.empty()) {
            pthread_cond_wait(&frameQueueCondNotempty, &_mutexFrameCam1Last);
        }
        recFrameForward = qFrame.front();
        timestampcamera = qTime.front();

        qFrame.pop();
        qTime.pop();
        if(qFrame.empty()) {
            pthread_cond_signal(&frameQueueCondEmpty);
        }
        pthread_mutex_unlock(&_mutexFrameCam1Last);

        imwrite("./record_data/cam0/" + std::to_string(timestampcamera) + ".png", recFrameForward);
        lframe << timestampcamera << "\n";
        totalRecord++;

        recFrameForward.copyTo(lastestFrameForward);

    }
    std::cout << "#Record = " << totalRecord << std::endl;
}

int Camera_Recorder::start() {
    // initilize camera parameter
    int con = initializeCamera();
    if (!con) return 0;

    //create camera thread
    std::cout << "Start camera thread..." << std::endl;
    threadCamera = boost::thread(&Camera_Recorder::cameraLoop, this);

    // create record thread
    std::cout << "Start record thread..." << std::endl;
    threadRecord = boost::thread(&Camera_Recorder::cameraRecord, this);

    return 1;
}

void Camera_Recorder::stop() {
    //join thread
    time_to_exit = true;
    lframe.close();

    threadCamera.join();
    threadRecord.join();
    std::cout << "Finish recording frames." << std::endl;
}

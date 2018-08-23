//
// Created by rasp on 1/23/18.
// @djangogirl /Bangkok

#include "time_recorder.h"

Time_Recorder::Time_Recorder(string filename_, string record_path_) : time_to_exit(false), filename(filename_), record_path(record_path_) {
    unEmptyTime = PTHREAD_COND_INITIALIZER;
    emptyTime = PTHREAD_COND_INITIALIZER;
    mutexTime = PTHREAD_MUTEX_INITIALIZER;

    std::cout << "initialize time recorder for " << filename << " at " << record_path << std::endl;
}

Time_Recorder::~Time_Recorder() {
    stop();
}

void Time_Recorder::stop() {
    std::cout << "stop time record thread \n";
    time_to_exit = true;
    threadRecordTime.join();
    std::cout << "Finish recording IMU." << std::endl;
}


void Time_Recorder::start() {

    std::cout << "Start IMU record thread..." << std::endl;
    threadRecordTime = boost::thread(&Time_Recorder::write_time_from_queue, this);

}

void Time_Recorder::add_time_to_queue(uint64_t time1, uint64_t time2, uint64_t time3, uint64_t time4,
                                    uint64_t time5) {
//    std::cout << "add IMU to queue \n";
    time_data time;
    time.time1 = time1;
    time.time2 = time2;
    time.time3 = time3;
    time.time4 = time4;
    time.time5 = time5;

    // add IMU to queue
    pthread_mutex_lock(&mutexTime);
    queueTime.push(time);
    pthread_cond_signal(&unEmptyTime);
    pthread_mutex_unlock(&mutexTime);
}

void Time_Recorder::write_time_from_queue() {

    std::string sep = ",";
    ofstream datasettime;
    int count = 0;

//    std::cout << "creating files for recording messages with header\n";
    datasettime.open(record_path + "/" + filename);
    if (datasettime.bad()) throw 40;
    datasettime << "time1" << sep << "time2" << sep << "time3" << sep << "time4" << sep << "time5" << "\n";
//
    while (!time_to_exit ) {
        if (!queueTime.empty()) {

            datasettime << queueTime.front().time1 << sep
                       << queueTime.front().time2 << sep
                       << queueTime.front().time3 << sep
                       << queueTime.front().time4 << sep
                       << queueTime.front().time5 << endl;

            queueTime.pop();

            count++;
        } else {
            pthread_mutex_lock(&mutexTime);
            pthread_cond_wait(&unEmptyTime, &mutexTime);
            pthread_mutex_unlock(&mutexTime);
        }

    }
    datasettime.close();
    std::cout << " total time is " << count << " (" + filename + ") \n";
}
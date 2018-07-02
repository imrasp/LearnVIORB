//
// Created by rasp on 1/23/18.
// @djangogirl /Bangkok

#include "imu_recorder.h"

IMU_Recorder::IMU_Recorder(MAV::ConfigParam *configParam_, bool active_) : active(active_),time_to_exit(false),configParam(configParam_) {
    std::cout << "creating files for recording messages\n";
    if (configParam->gpstime) {
        datasetimu.open("./record_data/imu0.csv");
        datasetimu << "timestamp" << sep << "omega_x" << sep << "omega_y" << sep << "omega_z" << sep << "alpha_x" << sep
                   << "alpha_y" << sep << "alpha_z" << "\n";
    }
}

IMU_Recorder::~IMU_Recorder() {
    if (configParam->gpstime) {
        datasetimu.close();
    }
}

void IMU_Recorder::stop() {
    //std::cout << "stop imu record thread \n";
    time_to_exit = true;
    threadRecordIMU.join();
    std::cout << "Finish recording IMU." << std::endl;
}


void IMU_Recorder::start(Autopilot_Interface *autopilot_interface_) {
    autopilot_interface = autopilot_interface_;

    std::cout << "Start IMU record thread..." << std::endl;
    threadRecordIMU = boost::thread(&IMU_Recorder::record, this);

}

void IMU_Recorder::write_first(){
    datasetimu << pixhawk_to_unix_time_ns(autopilot_interface->queueIMU.front().time_usec) << sep
               << autopilot_interface->queueIMU.front().xgyro << sep
               << autopilot_interface->queueIMU.front().ygyro << sep
               << autopilot_interface->queueIMU.front().zgyro << sep
               << autopilot_interface->queueIMU.front().xacc << sep
               << autopilot_interface->queueIMU.front().yacc << sep
               << autopilot_interface->queueIMU.front().zacc << endl;

    autopilot_interface->queueIMU.pop();
}

std::queue<mavlink_highres_imu_t> IMU_Recorder::copy_queue(){
    return autopilot_interface->queueIMU;
}

uint64_t IMU_Recorder::pixhawk_to_unix_time_ns(uint64_t time_since_boot){
    uint64_t timestamp_ms = ref_system_time.time_unix_usec + (time_since_boot - (ref_system_time.time_boot_ms * 1000));
    return timestamp_ms * 1000;
}

void IMU_Recorder::record(){

    pthread_mutex_lock(&autopilot_interface->mutexIMU);
    pthread_cond_wait(&autopilot_interface->unEmptyIMU, &autopilot_interface->mutexIMU);
    pthread_mutex_unlock(&autopilot_interface->mutexIMU);


    while (!time_to_exit){// || !autopilot_interface->queueIMU.empty()) {
        pthread_mutex_lock(&autopilot_interface->mutexIMU);
        if(autopilot_interface->queueIMU.empty())
            pthread_cond_wait(&autopilot_interface->unEmptyIMU, &autopilot_interface->mutexIMU);

        if (configParam->gpstime) {
            write_first();
        }
    }
}

void IMU_Recorder::set_ref_time(mavlink_system_time_t system_time) {
    ref_system_time = system_time;

    ref_boot_time_ms = system_time.time_boot_ms;
    ref_unix_time = system_time.time_unix_usec;
    ref_timestampcamera_ns = boost::lexical_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}

uint64_t IMU_Recorder::get_ns_time_ref_odroid(uint64_t time_ms){
    return ref_timestampcamera_ns + (time_ms - ref_boot_time_ms)*1e3;
}


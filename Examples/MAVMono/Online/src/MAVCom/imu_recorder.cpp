//
// Created by rasp on 1/23/18.
// @djangogirl /Bangkok

#include "imu_recorder.h"
#include "location_manager.cpp"

IMU_Recorder::IMU_Recorder(MAV::ConfigParam *configParam_, bool active_) : active(active_),time_to_exit(false),configParam(configParam_) {

}

IMU_Recorder::~IMU_Recorder() {

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

void IMU_Recorder::record(){
    // write out queue
    std::string sep = ",";
    ofstream datasetimu, datasetimu2, datasetimu3, datasetimu4, datasetimu5, datasetgps, datasetgpsned, datasetOdometry, datasetAttitude;
    double gpsx, gpsy, gpsz;
    geodetic_converter::GeodeticConverter *geodeticConverter = new geodetic_converter::GeodeticConverter();

    std::cout << "creating files for recording messages\n";
    datasetimu2.open("./record_data/imu0_odroidunix.csv");
    datasetimu3.open("./record_data/imu0_pixhawkms.csv");
    datasetimu5.open("./record_data/imu0_odroidrefpixhawk.csv");
    if (configParam->gpstime) {
        datasetimu.open("./record_data/imu0.csv");
        //datasetimu4.open("./record_data/imu0_odroidpixhawk.csv");

        datasetgps.open("./record_data/gps0.csv");
        datasetgpsned.open("./record_data/gpsned.csv");
        datasetOdometry.open("./record_data/odometry.csv");
        datasetAttitude.open("./record_data/attitude.csv");
    }


    datasetimu2 << "timestamp" << sep << "omega_x" << sep << "omega_y" << sep << "omega_z" << sep << "alpha_x" << sep
               << "alpha_y" << sep << "alpha_z" << "\n";
    datasetimu3 << "timestamp" << sep << "omega_x" << sep << "omega_y" << sep << "omega_z" << sep << "alpha_x" << sep
                << "alpha_y" << sep << "alpha_z" << "\n";
    datasetimu5 << "timestamp" << sep << "omega_x" << sep << "omega_y" << sep << "omega_z" << sep << "alpha_x"
                << sep
                << "alpha_y" << sep << "alpha_z" << "\n";
    if (configParam->gpstime) {
        datasetimu << "timestamp" << sep << "omega_x" << sep << "omega_y" << sep << "omega_z" << sep << "alpha_x" << sep
                   << "alpha_y" << sep << "alpha_z" << "\n";
//    datasetimu4 << "timestamp" << sep << "omega_x" << sep << "omega_y" << sep << "omega_z" << sep << "alpha_x" << sep
//                << "alpha_y" << sep << "alpha_z" << "\n";

        datasetgps << "timestamp" << sep << "lat" << sep << "lon" << sep << "alt" << "\n";
        datasetgpsned << "timestamp" << sep << "gpsx" << sep << "gpsy" << sep << "gpsz" << "\n";
        datasetOdometry << "timestamp" << sep << "tx" << sep << "ty" << sep << "tz" << sep
                        << "qx" << sep << "qy" << sep << "qz" << sep << "qw" << "\n";
        datasetAttitude << "timestamp" << sep << "roll" << sep << "pitch" << sep << "yaw"
                        << sep << "x" << sep << "y" << sep << "z" << sep << "w" << "\n";
    }

    pthread_mutex_lock(&autopilot_interface->mutexIMU);
        pthread_cond_wait(&autopilot_interface->unEmptyIMU, &autopilot_interface->mutexIMU);
pthread_mutex_unlock(&autopilot_interface->mutexIMU);


    while (!time_to_exit){// || !autopilot_interface->queueIMU.empty()) {
        pthread_mutex_lock(&autopilot_interface->mutexIMU);
        if(autopilot_interface->queueIMU.empty())
            pthread_cond_wait(&autopilot_interface->unEmptyIMU, &autopilot_interface->mutexIMU);

        uint64_t timestamp_ms = ref_system_time.time_unix_usec + (autopilot_interface->queueIMU.front().time_usec -
                                                                  (ref_system_time.time_boot_ms * 1000));
        uint64_t timestamp_ns = timestamp_ms * 1000;

        datasetimu2 << autopilot_interface->queueIMUtime.front() << sep
                   << autopilot_interface->queueIMU.front().xgyro << sep
                   << autopilot_interface->queueIMU.front().ygyro << sep
                   << autopilot_interface->queueIMU.front().zgyro << sep
                   << autopilot_interface->queueIMU.front().xacc << sep
                   << autopilot_interface->queueIMU.front().yacc << sep
                   << autopilot_interface->queueIMU.front().zacc << endl;

        datasetimu3 << autopilot_interface->queueIMU.front().time_usec << sep
                   << autopilot_interface->queueIMU.front().xgyro << sep
                   << autopilot_interface->queueIMU.front().ygyro << sep
                   << autopilot_interface->queueIMU.front().zgyro << sep
                   << autopilot_interface->queueIMU.front().xacc << sep
                   << autopilot_interface->queueIMU.front().yacc << sep
                   << autopilot_interface->queueIMU.front().zacc << endl;

        datasetimu5 << autopilot_interface->queueIMUUnixRefTime.front() << sep
                    << autopilot_interface->queueIMU.front().xgyro << sep
                    << autopilot_interface->queueIMU.front().ygyro << sep
                    << autopilot_interface->queueIMU.front().zgyro << sep
                    << autopilot_interface->queueIMU.front().xacc << sep
                    << autopilot_interface->queueIMU.front().yacc << sep
                    << autopilot_interface->queueIMU.front().zacc << endl;

        if (configParam->gpstime) {
            datasetimu << timestamp_ns << sep
                       << autopilot_interface->queueIMU.front().xgyro << sep
                       << autopilot_interface->queueIMU.front().ygyro << sep
                       << autopilot_interface->queueIMU.front().zgyro << sep
                       << autopilot_interface->queueIMU.front().xacc << sep
                       << autopilot_interface->queueIMU.front().yacc << sep
                       << autopilot_interface->queueIMU.front().zacc << endl;

//        datasetimu4 << get_ns_time_ref_odroid(autopilot_interface->queueIMU.front().time_usec) << sep
//                    << autopilot_interface->queueIMU.front().xgyro << sep
//                    << autopilot_interface->queueIMU.front().ygyro << sep
//                    << autopilot_interface->queueIMU.front().zgyro << sep
//                    << autopilot_interface->queueIMU.front().xacc << sep
//                    << autopilot_interface->queueIMU.front().yacc << sep
//                    << autopilot_interface->queueIMU.front().zacc << endl;


//
            // record gps as a ground truth
            if(autopilot_interface->queueGPS.empty()) {
//
                uint64_t gps_timestamp_ms = ref_system_time.time_unix_usec + ((autopilot_interface->queueGPS.front().time_boot_ms - ref_system_time.time_boot_ms) * 1000);
                uint64_t gps_timestamp_ns = timestamp_ms * 1000;

                datasetgps << gps_timestamp_ns << sep
                           << autopilot_interface->queueGPS.front().lat << sep
                           << autopilot_interface->queueGPS.front().lon << sep
                           << autopilot_interface->queueGPS.front().alt << endl;

                if (!geodeticConverter->isInitialised()) {
                        geodeticConverter->initialiseReference(autopilot_interface->queueGPS.front().lat / 1e7, autopilot_interface->queueGPS.front().lon / 1e7,
                                                               autopilot_interface->queueGPS.front().alt / 1000);

                } else {
                    geodeticConverter->geodetic2Ned(autopilot_interface->queueGPS.front().lat / 1e7, autopilot_interface->queueGPS.front().lon / 1e7, (double)autopilot_interface->queueGPS.front().alt / 1000, &gpsx, &gpsy, &gpsz);

                    datasetgpsned << gps_timestamp_ns << sep
                                  << gpsx << sep << gpsy << sep << gpsz << endl;
                }

                autopilot_interface->queueGPS.pop();
                autopilot_interface->queueGPSUnixRefTime.pop();
            }
//
            // record imu position as a ground truth
            if(autopilot_interface->queueOdometry.empty()) {

                uint64_t gps_timestamp_ms = ref_system_time.time_unix_usec + (autopilot_interface->queueOdometry.front().time_usec - (ref_system_time.time_boot_ms * 1000));
                uint64_t gps_timestamp_ns = timestamp_ms * 1000;

                datasetOdometry << autopilot_interface->queueOdometryUnixRefTime.front() << sep
                           << autopilot_interface->queueOdometry.front().x << sep
                           << autopilot_interface->queueOdometry.front().y << sep
                           << autopilot_interface->queueOdometry.front().z << sep
                           << autopilot_interface->queueOdometry.front().q[0] << sep
                           << autopilot_interface->queueOdometry.front().q[1] << sep
                           << autopilot_interface->queueOdometry.front().q[2] << sep
                           << autopilot_interface->queueOdometry.front().q[3] << endl;

                autopilot_interface->queueOdometry.pop();
                autopilot_interface->queueOdometryUnixRefTime.pop();
            }
        }

        autopilot_interface->queueIMU.pop();
        autopilot_interface->queueIMUtime.pop();
        autopilot_interface->queueIMUUnixRefTime.pop();
        pthread_mutex_unlock(&autopilot_interface->mutexIMU);
    }
}

void IMU_Recorder::set_ref_time(mavlink_system_time_t system_time) {
    ref_system_time = system_time;

    ref_boot_time_ms = system_time.time_boot_ms;
    ref_timestampcamera_ns = boost::lexical_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}

uint64_t IMU_Recorder::get_ns_time_ref_odroid(uint64_t time_ms){
    return ref_timestampcamera_ns + (time_ms - ref_boot_time_ms)*1e3;
}


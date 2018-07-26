#include <iostream>
#include <fstream>

#include "src/mavlink_control.h"
#include "src/camera_recorder.h"
#include "src/log.h"
#include "src/imu_recorder.h"
#include "src/slam_interface.h"


using namespace std;

int main(int argc, char **argv);

inline bool exists_test(const std::string &name);

int main(int argc, char **argv) {
    bool bCamera = true;
    bool bIMU = true;
    bool bSLAM = true;

    try {

        Mavlink_Control *mavlinkControl;
        Camera_Recorder *cameraRecorder;
        SLAM_Interface *slam_interface;

        std::cout << "\n Configurating params \n";
        ConfigParam configParam(argv[1]);

        //Check is configuration file is exist or not
        {
            if (!exists_test(configParam.mission_route)) {
                cout << "Route file is not exist, Please upload route.";
                return 0;
            }
            if (!exists_test(configParam.vocabulary)) {
                cout << "Vocabulary file is not exist, Please contract admin.";
                return 0;
            }
        }

        slam_interface = new SLAM_Interface(&configParam);

        std::cout << "\nCreating log folder \n";
        Log log(configParam.record_path);

        Location_Manager *location_manager = new Location_Manager((bool) configParam.gps_position_update,
                                                                  (bool) configParam.slam_position_update,
                                                                  slam_interface, configParam.record_path);
        mavlinkControl = new Mavlink_Control(&configParam, location_manager);
        cameraRecorder = new Camera_Recorder(configParam.cameraid, configParam.camera_time_offset, false,
                                             slam_interface, configParam.record_path);
        mavlinkControl->start();
        cameraRecorder->start();

        mavlinkControl->cmd();

        cameraRecorder->stop();
        mavlinkControl->stop();


        return 0;
    }
    catch (int error) {
        fprintf(stderr, "threw exception %i \n", error);
        return error;
    }
}

inline bool exists_test(const std::string &name) {
    ifstream f(name.c_str());
    return f.good();
}


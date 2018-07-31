#include "configParam.h"

namespace MAV {
    int ConfigParam::cameraid;
    int ConfigParam::baudrate;
    int ConfigParam::fps;
    int ConfigParam::sec;
    int ConfigParam::gpstime;
    int ConfigParam::bEnableIMU;
    int ConfigParam::bEnableSLAM;
    int ConfigParam::bEnableCamera;
    double ConfigParam::camera_time_offset;
    double ConfigParam::xacc_offset;
    double ConfigParam::yacc_offset;
    double ConfigParam::zacc_offset;

    string ConfigParam::uart_name;
    string ConfigParam::setting;
    string ConfigParam::record_path;
    string ConfigParam::vocabulary;
    string ConfigParam::mission_route;

    ConfigParam::ConfigParam(std::string configfile) {
        std::cout << " setting : " << configfile << std::endl;
        cv::FileStorage fSettings(configfile, cv::FileStorage::READ);
        baudrate = fSettings["system.baudrate"];
        fSettings["system.uart_name"] >> uart_name;
        fSettings["system.record_path"] >> record_path;
        fSettings["system.vocabulary"] >> vocabulary;
            fSettings["system.route"] >> mission_route;
        sec = fSettings["system.sec"];
        cameraid = fSettings["system.cameraid"];
        fps = fSettings["system.fps"];
        gpstime = fSettings["system.gpstime"];
        bEnableIMU = fSettings["system.useIMU"];
        bEnableSLAM = fSettings["system.useSLAM"];
        bEnableCamera = fSettings["system.usecamera"];

        camera_time_offset = fSettings["system.camera_time_offset"];
        xacc_offset = fSettings["system.xacc_offset"];
        yacc_offset = fSettings["system.yacc_offset"];
        zacc_offset = fSettings["system.zacc_offset"];


        cout << "\n-------------------------------------" << endl;
        cout << "Parameters :" << endl;
        cout << "UART name " << uart_name << " baudrate " << baudrate << endl;
        cout << "Save result at " << record_path << endl;
        cout << "FPS : " << fps << endl;
        cout << "Camera : /dev/video" << cameraid << endl;
        cout << "Record time (sec.) : " << sec << endl;
        cout << "-------------------------------------\n" << endl;
    }

//    int ConfigParam::get_bEnableIMU(){
//            return bEnableIMU;
//    }

}
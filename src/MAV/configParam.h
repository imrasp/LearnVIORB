#ifndef ConfigParam_H

#define ConfigParam_H

#include <iostream>
#include <cstring>
#include <string>
#include <fstream>
#include <cstdlib>
#include <opencv2/opencv.hpp>

using namespace std;
namespace MAV {
    class ConfigParam {
    public:

        static string setting;

        static int cameraid;
        static int fps;
        static int baudrate;
        static int sec;
        static int gpstime;
        static string uart_name;
        static string mission_route;
        static string record_path;
        static string vocabulary;
        static int bEnableCamera;
        static int bEnableIMU;
        static int bEnableSLAM;

        static double camera_time_offset;
        static double xacc_offset;
        static double yacc_offset;
        static double zacc_offset;

        ConfigParam(std::string configfile);
//        int get_bEnableIMU();

    private:
    };
}

#endif

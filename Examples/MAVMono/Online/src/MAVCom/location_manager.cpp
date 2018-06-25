//
// Created by rasp on 25/6/2561.
//

#include "location_manager.h"

Location_Manager::Location_Manager(): initializeGeodetic(false) {
    geodeticConverter = new geodetic_converter::GeodeticConverter();
    cx = 0;
    cy = 0;
    cz = 0;
}

Location_Manager::~Location_Manager() {

}

void Location_Manager::set_initial_geodetic_pose(uint64_t timestamp, double lat, double lon, double alt) {

    if (cx != 0 || cy != 0 || cz != 0) {
        geodeticConverter->initialiseReference(lat / 10e7, lon / 10e7, alt / 10e7);

        // get initial position use for interpolation
        uint64_t i_time = c_local_timestamp;
        double i_x = cx;
        double i_y = cy;
        double i_z = cz;

        // get second position for interpolation
        uint64_t f_time;
        double f_x, f_y, f_z;
        while (cx != i_x || cy != i_y || cz != i_z) {
            f_time = c_local_timestamp;
            f_x = cx;
            f_y = cy;
            f_z = cz;
        }
        // perform interpolation
        init_nedx = interpolate(i_time, f_time, timestamp, i_x, f_x);
        init_nedy = interpolate(i_time, f_time, timestamp, i_y, f_y);
        init_nedz = interpolate(i_time, f_time, timestamp, i_z, f_z);

        initializeGeodetic = true;
    }
}

void Location_Manager::set_local_position(uint64_t timestamp, double x, double y, double z){
    c_local_timestamp = timestamp;
    cx = x;
    cy = y;
    cz = z;
}

double Location_Manager::interpolate(uint64_t x1, uint64_t x2, uint64_t x_predict, double y1, double y2){
    // predicted_y =
    return y2 + (x_predict -x2) * ((y2 - y1)/(x2 - x1));
}

bool Location_Manager::isGeodeticInitialize(){
    return initializeGeodetic;
}

void Location_Manager::get_NED_from_geodetic(double lat, double lon, double alt, double *x, double *y, double *z) {
    geodeticConverter->geodetic2Ned(lat, lon, alt, x, y, z);
    *x = *x + init_nedx;
    *y = *y + init_nedy;
    *z = *z + init_nedz;
}

double Location_Manager::distanceInKmBetweenEarthCoordinates(double lat1, double lon1, double lat2, double lon2) {
    double earthRadiusKm = 6371;

    double dLat = geodeticConverter->deg2Rad(lat2-lat1);
    double dLon = geodeticConverter->deg2Rad(lon2-lon1);

    lat1 = geodeticConverter->deg2Rad(lat1);
    lat2 = geodeticConverter->deg2Rad(lat2);

    double a = sin(dLat/2) * sin(dLat/2) +
               sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return earthRadiusKm * c;
}
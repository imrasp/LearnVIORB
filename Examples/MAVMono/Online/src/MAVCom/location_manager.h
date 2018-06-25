//
// Created by rasp on 29/4/2561.
//

#ifndef C_UART_IMU_LOCATION_MANAGER_H
#define C_UART_IMU_LOCATION_MANAGER_H

#include "geodetic_converter.cpp"

class Location_Manager{
public:
    Location_Manager();
    ~Location_Manager();

    bool initializeGeodetic;
    double init_nedx, init_nedy, init_nedz;
    void set_initial_geodetic_pose(uint64_t timestamp, double lat, double lon, double alt);
    void get_NED_from_geodetic(double lat, double lon, double alt, double *x, double *y, double *z);

    void set_local_position(uint64_t timestamp, double x, double y, double z);
    bool isGeodeticInitialize();

    double distanceInKmBetweenEarthCoordinates(double lat1, double lon1, double lat2, double lon2);

    double interpolate(uint64_t x1, uint64_t x2, uint64_t x_predict, double y1, double y2);

private:
    geodetic_converter::GeodeticConverter *geodeticConverter;
    double cx,cy,cz;
    uint64_t c_local_timestamp;
};

#endif //C_UART_IMU_LOCATION_MANAGER_H

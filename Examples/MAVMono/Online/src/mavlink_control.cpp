/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink_control.cpp
 *
 * @brief An example offboard control process via mavlink
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */



// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "mavlink_control.h"

Mavlink_Control::Mavlink_Control(ConfigParam *configParam_, Location_Manager *location_manager_) : configParam(configParam_), location_manager(location_manager_) {

    // --------------------------------------------------------------------------
    //   PORT and THREAD STARTUP
    // --------------------------------------------------------------------------

    /*
     * Instantiate a serial port object
     *
     * This object handles the opening and closing of the offboard computer's
     * serial port over which it will communicate to an autopilot.  It has
     * methods to read and write a mavlink_message_t object.  To help with read
     * and write in the context of pthreading, it gaurds port operations with a
     * pthread mutex lock.
     *
     */
    serial_port = new Serial_Port(configParam->uart_name.c_str(), configParam->baudrate);


    /*
     * Instantiate an autopilot interface object
     *
     * This starts two threads for read and write over MAVlink. The read thread
     * listens for any MAVlink message and pushes it to the current_messages
     * attribute.  The write thread at the moment only streams a position target
     * in the local NED frame (mavlink_set_position_target_local_ned_t), which
     * is changed by using the method update_setpoint().  Sending these messages
     * are only half the requirement to get response from the autopilot, a signal
     * to enter "offboard_control" mode is sent by using the enable_offboard_control()
     * method.  Signal the exit of this mode with disable_offboard_control().  It's
     * important that one way or another this program signals offboard mode exit,
     * otherwise the vehicle will go into failsafe.
     *
     */
    autopilot_interface = new Autopilot_Interface(serial_port, location_manager);

    /*
     * Setup interrupt signal handler
     *
     * Responds to early exits signaled with Ctrl-C.  The handler will command
     * to exit offboard mode if required, and close threads and the port.
     * The handler in this example needs references to the above objects.
     *
     */
    serial_port_quit = serial_port;
    autopilot_interface_quit = autopilot_interface;


}

Mavlink_Control::~Mavlink_Control() {
}

void Mavlink_Control::start() {
    /*
     * Start the port and autopilot_interface
     * This is where the port is opened, and read and write threads are started.
     */
    serial_port->start();
    autopilot_interface->start();


    if(configParam->gpstime) {
        //waiting for first gps message
        cout << "waiting for GPS signal \n";
        int count = 30;
        while (!location_manager->isGeodeticInitialize() && count > 0) {
            sleep(1);
            --count;
        }

        if (!location_manager->isGeodeticInitialize()) {
            throw 20; // gps not found
        }

        //check route
        std::cout << "start route check\n";
        int result = check_route();
        std::cout << "route checked \n";
    }

}

void Mavlink_Control::cmd() {
    // --------------------------------------------------------------------------
    //   START OFFBOARD MODE
    // --------------------------------------------------------------------------
    autopilot_interface->enable_offboard_control();

    // --------------------------------------------------------------------------
    //   RUN INITIAL COMMANDS to maintain the connection
    // --------------------------------------------------------------------------
    mavlink_set_position_target_local_ned_t sp;
    mavlink_set_position_target_local_ned_t ip = autopilot_interface->
            initial_position;
    set_position( ip.x , // [m]
                  ip.y , // [m]
                  ip.z , // [m]
                  sp         );
    autopilot_interface->update_setpoint(sp);

//    autopilot_interface->arm_control();

    usleep(100); // give some time to let it sink in
//    cout << " waiting for " << configParam->sec << " sec.\n";
//    // stack imu in queue for 60 seconds
//    sleep(configParam->sec);
    if(configParam->gpstime) {
        follow_route_file();
    } else {
//        // record for kalibr
//        cout << "start 1th hr record \n";
//        sleep(3600);
//        cout << "start 2nd hr record \n";
//        sleep(3600);
//        cout << "start 3rd hr record \n";
//        sleep(3600);
//        cout << "start 4th hr record \n";
//        sleep(3600);
//        cout << "Finish 4 hrs record! \n";
//        sleep(10);
    }



//    autopilot_interface->disarm_control();

// --------------------------------------------------------------------------
//   STOP OFFBOARD MODE
// --------------------------------------------------------------------------

    autopilot_interface->disable_offboard_control();

// --------------------------------------------------------------------------
//   END OF COMMANDS
// --------------------------------------------------------------------------

    return;
}

void Mavlink_Control::stop() {
    // --------------------------------------------------------------------------
    //   THREAD and PORT SHUTDOWN
    // --------------------------------------------------------------------------

    /*
     * Now that we are done we can stop the threads and close the port
     */
    autopilot_interface->stop();
    serial_port->stop();
}

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
Mavlink_Control::quit_handler(int sig) {
    printf("\n");
    printf("TERMINATING AT USER REQUEST\n");
    printf("\n");

    // autopilot interface
    try {
        autopilot_interface_quit->handle_quit(sig);
    }
    catch (int error) {}

    // serial port
    try {
        serial_port_quit->handle_quit(sig);
    }
    catch (int error) {}

    // end program here
    exit(0);

}

int Mavlink_Control::check_route(){
    ifstream input(configParam->mission_route);
    string line, temp;
    float lat = 0, lon = 0, alt = 0;

    // find first gps waypoint
    if (input.is_open()) {
        while (getline(input,line)) {
            cout << "read route line : " << line << endl;
            stringstream s (line);
            int i = 0;

            string mode = "";

            while(s>> temp) {
                if ( i == 0 && temp == "goto_gps" ) {
                    i++;
                } else if (i == 1) {
                    lat = stod(temp); i++;
                } else if (i == 2) {
                    lon = stod(temp); break;
//                } else if (i == 3) {
//                    alt = stod(temp); break;
                }
            }

            if(i != 0) break;

        }


    }

//    cout << "lat,lon =" << lat << ", " << lon << endl;
//    cout << "current lat, lon = " << autopilot_interface->current_messages.global_position_int.lat << ", " << autopilot_interface->current_messages.global_position_int.lon << endl;
    if(lat != 0){
        //check distant between current and first waypoint
        double distant = location_manager->distanceInKmBetweenEarthCoordinates(lat, lon, autopilot_interface->current_messages.global_position_int.lat, autopilot_interface->current_messages.global_position_int.lon);
        //if distant is longer than 50 meters in x y direction
        //return mission dinined because location of the route is to far from current location.
        if (distant > 50.0){
            throw 21; // route mismatch
        }

    }
}

int Mavlink_Control::follow_route_file(){
    cout << "read route from " << configParam->mission_route << endl;
    ifstream input(configParam->mission_route);
    string line, temp;
    double param1, param2;

    if (input.is_open()) {
        while (getline(input,line)) {
            cout << line << endl;
            stringstream s (line);
            int i = 0;
            string mode = "";

            // example : goto 9.0 8.0 7.0
            //          goto > i == 0
            //          9.0 > i == 1
            //          8.0 > i == 2
            //          7.0 > i == 3
            while(s>> temp) {
                cout << i << " : mode " << mode << " : " << temp << endl;
                if ( i == 0 && temp == "takeoff" ){
                    autopilot_interface->enable_takeoff(10.0, 0.5);
                    //mode = "takeoff";
                } else if ( i == 0 && temp == "arm" ){
                    autopilot_interface->arm_control();
                } else if ( i == 0 && temp == "disarm" ){
                    autopilot_interface->disarm_control();
                } else if ( i == 0 && temp == "land" ){
                    autopilot_interface->enable_land();
                }
//                else if ( i == 0 && temp == "update_GPS_pose" ){
//                    location_manager->setUpdateGPSPoseToMavlink(true);
//                } else if ( i == 0 && temp == "disable_update_GPS_pose" ){
//                    location_manager->setUpdateGPSPoseToMavlink(false);
//                } else if ( i == 0 && temp == "update_SLAM_pose" ){
//                    location_manager->setUpdateGPSPoseToMavlink(true);
//                } else if ( i == 0 && temp == "disable_update_SLAM_pose" ){
//                    location_manager->setUpdateGPSPoseToMavlink(true);
//                }
                else if ( i == 0 && temp == "hold" ){
                    mode = "hold"; i++;
                } else if ( i == 0 && temp == "hold" ){
                    mode = "print"; i++;
                } else if ( i == 0 && temp == "goto_ned" ){
                    mode = "gotoned"; i++;
                } else if ( i == 0 && temp == "goto_gps" ){
                    mode = "gotogps"; i++;
                } else if ( i == 0 && temp == "sleep" ) {
                    mode = "sleep"; i++;
                } else if ( i == 0 && temp == "gotonedoffset" ){
                    mode = "gotonedoffset"; i++;
                } else if ( i == 0 && temp == "findgps" ) {
                    int waiting_time = 60;
                    //Checking for GPS
                    while(autopilot_interface->current_messages.gps_raw_int.eph > 120 && autopilot_interface->current_messages.local_position_ned.z > -30 && waiting_time > 0){
                        sleep(1);
                        waiting_time--;
                    }
                    if(autopilot_interface->current_messages.gps_raw_int.eph > 120) return -1;
                } else if ( i != 0 ) { // hold and goto // 1 condition
                    if( i == 1 && mode == "hold" ){
                        autopilot_interface->enable_hold(stod(temp));
                    } else if( i == 1 && mode == "sleep" ){
                        cout << "sleep for " << stod(temp) << " sec. \n";
                        sleep(stod(temp));
                    } else if( i == 1 && mode == "print" ){
                        cout << temp << endl;
                    } else if ( i == 1 && (mode == "goto_ned" || mode == "goto_gps" || mode == "gotonedoffset" || mode == "takeoff")  ){
                        param1 = stod(temp); i++;
                    } else if (i != 1 ){
                        if( i == 2 && mode == "takeoff") {
//                            autopilot_interface->enable_takeoff(param1,stod(temp));
                        } else if( i == 2){
                            param2 = stod(temp); i++;
                        } else if( i == 3 && mode == "goto_ned" ){
                            autopilot_interface->goto_positon_ned(param1,param2,stod(temp));
                        } else if( i == 3 && mode == "goto_gps" ){
                            //Convert gps to ned

                            //fly in ned location
                            autopilot_interface->goto_positon_ned(param1,param2,stod(temp));
                        } else if( i == 3 && mode == "gotonedoffset" ){
                            autopilot_interface->goto_positon_offset_ned(param1,param2,stod(temp));
                        }
                    }
                }
            }
        }
    } else throw 22;
    input.close();
}




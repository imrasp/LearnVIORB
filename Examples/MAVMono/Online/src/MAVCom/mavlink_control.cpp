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

Mavlink_Control::Mavlink_Control(ConfigParam *configParam_, IMU_Recorder *imu_recorder_) : configParam(configParam_),
                                                                                           imu_recorder(imu_recorder_) {

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
    autopilot_interface = new Autopilot_Interface(serial_port);

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

    // --------------------------------------------------------------------------
    //   RUN INITIAL COMMANDS
    // --------------------------------------------------------------------------
    mavlink_set_position_target_local_ned_t sp;
    mavlink_set_position_target_local_ned_t ip = autopilot_interface->
            initial_position;
    autopilot_interface->update_setpoint(sp);

    // set time reference for imu data

    while (!autopilot_interface->bTimeRef) {


        pthread_mutex_lock(&autopilot_interface->mutexTimeRef);
        pthread_cond_wait(&autopilot_interface->timeRef, &autopilot_interface->mutexTimeRef);
        pthread_mutex_unlock(&autopilot_interface->mutexTimeRef);
        mavlink_system_time_t sys_time = autopilot_interface->current_messages.system_time;

        uint64_t current_unix_time = boost::lexical_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count());

        cout << "waiting for time reference\n";
        cout << "current_unix_time(" << current_unix_time <<
             ")  - sys_time.time_unix_usec(" << sys_time.time_unix_usec << ") = " <<
             abs(current_unix_time - sys_time.time_unix_usec) << "\n";

        if (current_unix_time - sys_time.time_unix_usec < 1e7) {
            imu_recorder->set_ref_time(autopilot_interface->current_messages.system_time);
            imu_recorder->start(autopilot_interface);
            cout << "set time referenced = " << autopilot_interface->bTimeRef << "!! \n";
            autopilot_interface->bTimeRef = true;
        } else if (!configParam->gpstime) {
            autopilot_interface->bTimeRef = true;
            imu_recorder->start(autopilot_interface);
            break;
        }
        pthread_cond_signal(&autopilot_interface->noTimeRef);
    }

}

void Mavlink_Control::cmd() {
    // --------------------------------------------------------------------------
    //   START OFFBOARD MODE
    // --------------------------------------------------------------------------
    autopilot_interface->enable_offboard_control();

    autopilot_interface->arm_control();

    usleep(100); // give some time to let it sink in
    cout << " waiting for " << configParam->sec << " sec.\n";
    // stack imu in queue for 60 seconds
    sleep(configParam->sec);

    autopilot_interface->disarm_control();

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



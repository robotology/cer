/*
* Copyright (C)2015  iCub Facility - Istituto Italiano di Tecnologia
* Author: Marco Randazzo
* email:  marco.randazzo@iit.it
* website: www.robotcub.org
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* http://www.robotcub.org/icub/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

#ifndef CONTROL_THREAD_H
#define CONTROL_THREAD_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/pids.h>
#include <string>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

#define MAX_LINEAR_VEL  0.4  // maximum linear  velocity (m/s)
#define MAX_ANGULAR_VEL 24.0 // maximum angular velocity (deg/s)


class ControlThread : public yarp::os::RateThread
{
private:
    Property            ctrl_options;

    double              thread_period;
    int                 base_control_type;
    int                 thread_timeout_counter;

protected:
    ResourceFinder            &rf;
    PolyDriver                *control_board_driver;
    BufferedPort<Bottle>      port_joystick_control;

    BufferedPort<Bottle>      port_debug_linear;
    BufferedPort<Bottle>      port_debug_angular;

    string                    remoteName;
    string                    localName;

    IPositionDirect*          iDir;
    IEncoders*                iEnc;
    
    double                    elong;
    double                    pitch;
    double                    roll;

    double                    max_elong;
    double                    min_elong;
    double                    max_alpha;

    double                    enc_init_elong;
    double                    enc_init_pitch;
    double                    enc_init_roll;


public:
    bool                      motors_enabled;

    ControlThread(unsigned int _period, ResourceFinder &_rf, Property options);

    virtual bool threadInit();
    virtual void threadRelease();
    virtual void afterStart(bool s);
    virtual void run();

    void printStats();
};

#endif

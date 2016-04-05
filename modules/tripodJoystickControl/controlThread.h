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

public:

    bool                      motors_enabled;

    ControlThread(unsigned int _period, ResourceFinder &_rf, Property options) :
               RateThread(_period),     rf(_rf),
               ctrl_options(options)
    {
        control_board_driver     = 0;
        thread_timeout_counter   = 0;
        iDir                     = 0;
        iEnc                     = 0;
        thread_period            = _period;

        remoteName               = ctrl_options.find("remote").asString();
        localName                = ctrl_options.find("local").asString();
        motors_enabled           = true;
    }

    virtual bool threadInit()
    {
        //open the joystick port
        port_joystick_control.open("/tripodJoystickCtrl/joystick:i");

        // open the control board driver
        yInfo("Opening the motors interface...\n");
        int trials = 0;
        double start_time = yarp::os::Time::now();
        Property control_board_options("(device remote_controlboard)");
        control_board_options.put("remote", remoteName.c_str());
        control_board_options.put("local", localName.c_str());

        do
        {
            double current_time = yarp::os::Time::now();

            //remove previously existing drivers
            if (control_board_driver)
            {
                delete control_board_driver;
                control_board_driver = 0;
            }

            //creates the new device driver
            control_board_driver = new PolyDriver(control_board_options);
            bool connected = control_board_driver->isValid();

            //check if the driver is connected
            if (connected) break;

            //check if the timeout (10s) is expired
            if (current_time - start_time > 10.0)
            {
                yError("It is not possible to instantiate the device driver. I tried %d times!", trials);
                if (control_board_driver)
                {
                    delete control_board_driver;
                    control_board_driver = 0;
                }
                return false;
            }

            yarp::os::Time::delay(0.5);
            trials++;
            yWarning("Unable to connect the device driver, trying again...");
        } while (true);

        control_board_driver->view(iDir);
        control_board_driver->view(iEnc);
        if (iDir == 0 || iEnc == 0)
        {
            yError() << "Failed to open interfaces";
            return false;
        }
        return true;
    }

    virtual void afterStart(bool s)
    {
        if (s)
            yInfo("Control thread started successfully");
        else
            yError("Control thread did not start");
    }

    virtual void run();

    void printStats();

    virtual void threadRelease()
    {
    }
};

#endif

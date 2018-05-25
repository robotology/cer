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
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <iostream>
#include <iomanip>
#include <string>

#include "controlThread.h"


using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

class CtrlModule: public RFModule
{
protected:
    ControlThread  *control_thr;
    Port            rpcPort;

public:
    CtrlModule() 
    {
        control_thr=0;
    }

    virtual bool configure(ResourceFinder &rf)
    {
        string slash = "/";
        string ctrlName;
        string robotName;
        string partName;
        string remoteName;
        string localName;

        Time::turboBoost();

        // get params from the RF
        ctrlName = rf.check("local", Value("tripodJoystickControl")).asString();
        robotName = rf.check("robot", Value("cer")).asString();
        partName = rf.check("part", Value("torso_tripod")).asString();

        remoteName = slash + robotName + slash + partName;
        localName = ctrlName;

        //reads the configuration file
        Property ctrl_options;

        string configFile = rf.findFile("from");
        if (configFile == "") //--from torsoJoystickControl.ini
        {
            yWarning("Cannot find .ini configuration file. By default I'm searching for torsoJoystickControl.ini");
            //return false;
        }
        else
        {
            ctrl_options.fromConfigFile(configFile.c_str());
        }

        ctrl_options.put("remote", remoteName.c_str());
        ctrl_options.put("local", localName.c_str());

        //check for robotInterface availablity
        yInfo("Checking for robotInterface availability");
        Port startport;
        startport.open(localName+"/robotInterfaceCheck:rpc");


        Bottle cmd; cmd.addString("is_ready");
        Bottle response;
        int rc_count = 0;
        int rp_count = 0;
        int rf_count = 0;
        double start_time = yarp::os::Time::now();
        bool not_yet_connected = true;

        bool skip_robot_interface_check = rf.check("skip_robot_interface_check");
        if (skip_robot_interface_check)
        {
            yInfo("skipping robotInterface check");
        }
        else
        {
            do
            {
                if (not_yet_connected)
                {
                    bool rc = yarp::os::Network::connect(localName + "/robotInterfaceCheck:rpc", "/" + robotName + "/robotInterface");
                    if (rc == false)
                    {
                        yWarning("Problems trying to connect to RobotInterface %d", rc_count++);
                        yarp::os::Time::delay(1.0);
                        continue;
                    }
                    else
                    {
                        not_yet_connected = false;
                        yDebug("Connection established with robotInterface");
                    }
                }

                bool rp = startport.write(cmd, response);
                if (rp == false)
                {
                    yWarning("Problems trying to connect to RobotInterface %d", rp_count++);
                    if (yarp::os::Time::now() - start_time > 30)
                    {
                        yError("Timeout expired while trying to connect to robotInterface");
                        return false;
                    }
                    yarp::os::Time::delay(1.0);
                    continue;
                }
                else
                {
                    if (response.get(0).asString() != "ok")
                    {
                        yWarning("RobotInterface is not ready yet, retrying... %d", rf_count++);
                        if (yarp::os::Time::now() - start_time > 30)
                        {
                            yError("Timeout expired while waiting for robotInterface availability");
                            return false;
                        }
                        yarp::os::Time::delay(1.0);
                        continue;
                    }
                    else
                    {
                        yInfo("RobotInterface is ready");
                        break;
                    }
                }
            } while (1);
        }

        //set the thread rate
        int rate = rf.check("rate",Value(10)).asInt();
        yInfo("baseCtrl thread rate: %d ms.",rate);

        //start the control thread
        control_thr = new ControlThread(rate, rf, ctrl_options);
        if (!control_thr->start())
        {
            delete control_thr;
            return false;
        }

        //check for debug mode
        if (rf.check("no_motors"))
        {
            this->control_thr->motors_enabled = false;
            yInfo() << "Motors disabled";
        }

        rpcPort.open((localName+"/rpc").c_str());
        attach(rpcPort);

        return true;
    }

    bool respond(const Bottle& command, Bottle& reply) 
    {
        reply.clear(); 
        reply.addString("Unknown command.");
        return true;
    }

    virtual bool close()
    {
        if (control_thr)
        {
            control_thr->stop();
            delete control_thr;
        }

        rpcPort.interrupt();
        rpcPort.close();

        return true;
    }

    virtual double getPeriod()    { return 0.1;  }
    virtual bool   updateModule()
    { 
        if (control_thr)
        {
            control_thr->printStats();
        }
        else
        {
            yDebug("* Motor thread:not running");
        }

        static int life_counter=0;
        yDebug( "* Life: %d\n", life_counter);
        life_counter++;

        return true;
    }
};



int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("cer");
    rf.setDefaultConfigFile("tripodJoystickCtrl.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        yInfo("Possible options: ");
        yInfo("'robot <name>' the robot name for remote connection.");
        yInfo("'local <name>' the local port name.");
        yInfo("'rate <r>' sets the threads rate (default 20ms).");
        yInfo("'no_motors' motor interface will not be opened.");
        yInfo("'joystick_connect' tries to automatically connect to the joystickCtrl output.");
        yInfo("'skip_robot_interface_check' does not connect to robotInterface/rpc (useful for simulator)");
        yInfo("'max_elong' <value> max tripod elongation");
        yInfo("'min_alpha' <value> min tripod elongation");
        yInfo("'max_alpha' <value> max alpha angle");

        yInfo("''");
        yInfo("example: tripodJoystickCtrl --robot SIM_CER_ROBOT --part torso --joystick_connect --skip_robot_interface_check ");
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
    {
        yError("Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    CtrlModule mod;

    return mod.runModule(rf);
}

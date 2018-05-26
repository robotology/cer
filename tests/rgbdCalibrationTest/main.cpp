/*
* Copyright (C)2017  iCub Facility - Istituto Italiano di Tecnologia
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
        //set the thread rate
        int rate = rf.check("rate",Value(10)).asInt();
        yInfo("baseCtrl thread rate: %d ms.",rate);

        //start the control thread
        control_thr = new ControlThread(rate, rf);
        if (!control_thr->start())
        {
            delete control_thr;
            return false;
        }

        rpcPort.open("/rgbdCalibrationTest/rpc");
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

    virtual double getPeriod()    { return 0.01;  }
    virtual bool   updateModule()
    { 
        if (control_thr)
        {
            control_thr->printStats();
        }
        else
        {
            yDebug("* thread:not running");
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
    rf.setDefaultConfigFile("rgbdCalibrationTest.ini");
    rf.configure(argc,argv);

    Network yarp;

    if (!yarp.checkNetwork())
    {
        yError("Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    CtrlModule mod;

    return mod.runModule(rf);
}

/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

#define _USE_MATH_DEFINES
#include <cmath>
#include <string>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <cer_kinematics/arm.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace cer::kinematics;


/****************************************************************/
class Controller : public RFModule
{
    PolyDriver drivers[4];
    RpcServer rpcPort;

public:
    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string robot=rf.check("robot",Value("cer")).asString().c_str();
        string arm_type=rf.check("arm-type",Value("left")).asString().c_str();

        rpcPort.open(("/cer_controller/"+arm_type+"/rpc").c_str());
        attach(rpcPort);        

        return true;
    }

    /****************************************************************/
    bool close()
    {
        rpcPort.close();
        return true;
    }

    /****************************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /****************************************************************/
    bool updateModule()
    {
        return true;
    }

    /****************************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        reply.addVocab(Vocab::encode("ack"));
        return true;
    }
};


/****************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("YARP server not available!");
        return 1;
    }
    
    ResourceFinder rf;
    rf.configure(argc,argv);

    Controller controller;
    return controller.runModule(rf);
}


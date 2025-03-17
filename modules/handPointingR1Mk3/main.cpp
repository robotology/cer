/*
 * SPDX-FileCopyrightText: 2006-2025 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/os/Log.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

#include "handPointing.h"

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("check Yarp network.\n");
        return -1;
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("handPointing_R1Mk3_SIM.ini");             //overridden by --from parameter
    rf.setDefaultContext("handPointing");                           //overridden by --context parameter
    rf.configure(argc,argv);
    HandPointing point;

    return point.runModule(rf);
}

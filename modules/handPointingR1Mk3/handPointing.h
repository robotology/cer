
/*
 * Copyright (C) 2006-2025 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */


#ifndef HAND_POINTING_H
#define HAND_POINTING_H

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <math.h>
#include "handPointingThread.h"
#include "goHomeRobot.h"


class HandPointing : public yarp::os::RFModule
{
protected:
    
    //Ports
    yarp::os::BufferedPort<yarp::os::Bottle> m_pointInputPort;
    yarp::os::BufferedPort<yarp::os::Bottle> m_goHomePort;

    //Callback thread
    HandPointingThread*      m_innerThread;
    
    //Callback
    GoHomeRobot*                   m_goHomeRobot;
    
    double                         m_period;

public:
    HandPointing();
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool close();
    virtual double getPeriod();
    virtual bool updateModule();
};

#endif

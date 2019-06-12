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
#include <yarp/sig/Image.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/pids.h>
#include <string>
#include <math.h>
#include <yarp/dev/IRGBDSensor.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

class ControlThread : public yarp::os::PeriodicThread
{
private:

protected:
    ResourceFinder            &m_rf;
    PolyDriver                m_driver;

    IRGBDSensor* iRGBD;
    int m_depth_width;
    int m_depth_height;
    yarp::sig::ImageOf<float> m_depth_image;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> m_gain_image;
    Port m_test_port;

    float m_real_value;


public:

    ControlThread(unsigned int _period, ResourceFinder &_rf);

    virtual bool threadInit();
    virtual void threadRelease();
    virtual void afterStart(bool s);
    virtual void run();
    
    void printStats();
};

#endif

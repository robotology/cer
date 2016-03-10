// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2015 iCub Facility
// Authors: Marco Randazzo <marco.randazzo@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include <boschIMU.h>

#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Log.h>
#include <iostream>
#include <string.h>
#include <time.h>
#include <stdlib.h>

using namespace std;
using namespace cer::dev;
using namespace yarp::os;
using namespace yarp::dev;

bool BoschIMU::open(yarp::os::Searchable& config)
{
    bool correct=true;
    //debug
    yTrace("\nParameters are:\n\t%s\n", config.toString().c_str());

    if(!config.check("baudrate"))
    {
        yError() << "Param 'baudrate' not found";
        return false;
    }

    if(!config.check("comport"))
    {
        yError() << "Param 'comport' not found";
        return false;
    }

    int period = config.check("period",Value(10),"Thread period in ms").asInt();
    setRate(period);

    Property prop;
    std::string ps = config.toString();
    prop.fromString(ps);
    prop.put("device", "serialport");

    //open the driver
    driver.open(prop);
    if (!driver.isValid())
    {
        yError() << "Error opening PolyDriver check parameters";
        return false;
    }

    //open the serial interface
    driver.view(pSerial);
    if (!pSerial)
    {
        yError("Error opening serial driver. Device not available");
        return false;
    }

    RateThread::start();
    return true;
}

bool BoschIMU::close()
{
    //stop the thread
    RateThread::stop();

    //stop the driver
    driver.close();

    return true;
}

bool BoschIMU::threadInit()
{
    timeStamp = yarp::os::Time::now();

    //start the transmission
    char buffer[128];
    buffer[0]= 0xAA;  // start byte
    buffer[1]= 0x01;  // read operation
    buffer[2]= 0x3D;  // operation mode register
    buffer[3]= 0x01;  // lenght to read (bytes?)

    if (pSerial)
    {
        bool ret = pSerial->send(buffer, 1);
        if (ret == false) { yError("BoschIMU problems starting the transmission"); return false; }
    }
    else
    {
        yError("BoschIMU pSerial == NULL");
        return false;
    }

    return true;
}

void BoschIMU::run()
{
    return;
    double timeNow=yarp::os::Time::now();
    mutex.wait();

    //if 100ms have passed since the last received message
    if (timeStamp+0.1<timeNow)
    {
//         status=IBattery::BATTERY_TIMEOUT;
    }

    //add checksum verification, if supported

    mutex.post();
}
bool BoschIMU::read(yarp::sig::Vector &out)
{
    return false;
};

bool BoschIMU::getChannels(int *nc)
{
    return false;
};

bool BoschIMU::calibrate(int ch, double v)
{
    return false;
};

void BoschIMU::threadRelease()
{
    yTrace("BoschIMU Thread released\n");
}


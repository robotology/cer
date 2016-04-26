/*
 * Copyright (C) 2016 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
 * CopyPolicy: LGPLv2.1+
 */

#ifndef OCULUS2CER_H
#define OCULUS2CER_H

#include <yarp/os/MonitorObject.h>
#include <yarp/os/Things.h>
#include <yarp/os/Bottle.h>

class Oculus2Cer : public yarp::os::MonitorObject
{
public:
    Oculus2Cer();
    virtual bool accept(yarp::os::Things& thing);
    virtual yarp::os::Things& update(yarp::os::Things& thing);
private:
    yarp::os::Things things;
    yarp::os::Bottle bottle;
};

#endif

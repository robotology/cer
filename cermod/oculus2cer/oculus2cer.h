/*
 * Copyright (C) 2016 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
 * CopyPolicy: LGPLv2.1+
 */

#ifndef OCULUS2CER_H
#define OCULUS2CER_H

#include <yarp/os/MonitorObject.h>

class Oculus2Cer : public yarp::os::MonitorObject
{
public:
    virtual yarp::os::Things& update(yarp::os::Things& thing);
};

#endif

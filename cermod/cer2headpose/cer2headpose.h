/*
 * Copyright (C) 2016 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
 * CopyPolicy: LGPLv2.1+
 */

#ifndef CER2HEADPOSE_H
#define CER2HEADPOSE_H

#include <yarp/os/MonitorObject.h>
#include <yarp/os/Things.h>
#include <yarp/sig/Vector.h>

class Cer2HeadPose : public yarp::os::MonitorObject
{
public:
    Cer2HeadPose();
    virtual bool accept(yarp::os::Things& thing);
    virtual yarp::os::Things& update(yarp::os::Things& thing);
private:
    yarp::os::Things things;
    yarp::sig::Vector vector;
};

#endif

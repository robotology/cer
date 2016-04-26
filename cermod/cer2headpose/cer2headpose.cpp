/*
 * Copyright (C) 2016 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
 * CopyPolicy: LGPLv2.1+
 */

#include "cer2headpose.h"

#include <yarp/os/Log.h>
#include <yarp/os/Value.h>


Cer2HeadPose::Cer2HeadPose()
{
    this->vector.resize(3);
    this->things.setPortWriter(&this->vector);
}

bool Cer2HeadPose::accept(yarp::os::Things& thing)
{
    yarp::sig::Vector *vp = thing.cast_as<yarp::sig::Vector>();
    if (vp == NULL) {
        yWarning("Cer2HeadPose: expected type Vector but got wrong data type!");
        return false;
    }
    if (vp->size() != 2) {
        yWarning("Cer2HeadPose: expected Vector of size 2 but got wrong size (%zd)!", vp->size());
        return false;
    }
    return true;
}
yarp::os::Things& Cer2HeadPose::update(yarp::os::Things& thing)
{
    yarp::sig::Vector *vp = thing.cast_as<yarp::sig::Vector>();
    yAssert(vp);

    yarp::sig::Vector &v = *vp;

    double pitch = - (v[0] + 10);
    double yaw   = v[1];

    this->vector[0] = pitch;
    this->vector[1] = 0.0;
    this->vector[2] = yaw;

    return this->things;
}

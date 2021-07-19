/*
 * Copyright (C) 2016 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
 * CopyPolicy: LGPLv2.1+
 */

#include "oculus2cer.h"

#include <yarp/os/Log.h>
#include <yarp/os/Value.h>


Oculus2Cer::Oculus2Cer()
{
    this->things.setPortWriter(&this->bottle);
}

bool Oculus2Cer::accept(yarp::os::Things& thing)
{
    yarp::os::Bottle* bt = thing.cast_as<yarp::os::Bottle>();
    if (bt == NULL) {
        yWarning("Oculus2Cer: expected type Bottle but got wrong data type!");
        return false;
    }
    if (bt->size() != 3) {
        yWarning("Oculus2Cer: expected Bottle of size 3 but got wrong size!");
        return false;
    }
    return true;
}

yarp::os::Things& Oculus2Cer::update(yarp::os::Things& thing)
{
    yarp::os::Bottle* bt = thing.cast_as<yarp::os::Bottle>();
    yAssert(bt);

    double pitch = bt->get(0).asFloat64();
    double yaw   = bt->get(2).asFloat64();

    this->bottle.clear();
    this->bottle.addFloat64(-pitch);
    this->bottle.addFloat64(yaw);

    return this->things;
}

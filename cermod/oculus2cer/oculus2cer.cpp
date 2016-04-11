/*
 * Copyright (C) 2016 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
 * CopyPolicy: LGPLv2.1+
 */

#include "oculus2cer.h"

#include <yarp/os/Bottle.h>
#include <yarp/os/Log.h>
#include <yarp/os/Things.h>
#include <yarp/os/Value.h>


yarp::os::Things& Oculus2Cer::update(yarp::os::Things& thing)
{
    yarp::os::Bottle* bt = thing.cast_as<yarp::os::Bottle>();
    if (bt == NULL) {
        yWarning("Oculus2Cer: expected type Bottle but got wrong data type!");
        return thing;
    }
    if (bt->size() != 3) {
        yWarning("Oculus2Cer: expected Bottle of size 3 but got wrong size!");
        return thing;
    }

    double pitch = bt->get(0).asDouble();
    double yaw   = bt->get(2).asDouble();

    bt->clear();
    bt->addDouble(-pitch);
    bt->addDouble(yaw);

    return thing;
}

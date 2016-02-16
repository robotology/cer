/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino <alberto.cardellino@iit.it>
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <faceDisplayServer.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace yarp::dev;
using namespace cer::dev;
using namespace yarp::os;
using namespace std;

/**
  * It reads the data from an analog sensor and sends them on one or more ports.
  * It creates one rpc port and its related handler for every output port.
  */

// Constructor used when there is only one output port
FaceDisplayServer::FaceDisplayServer()
{
    yTrace();
}

FaceDisplayServer::~FaceDisplayServer()
{
    yTrace();
    threadRelease();
}

bool FaceDisplayServer::attachAll(const PolyDriverList &analog2attach)
{
    yError() << "FaceDisplayServer: Attach metod is not yet implemented";
    return false;
}

bool FaceDisplayServer::detachAll()
{
    yError() << "FaceDisplayServer: Detach metod is not yet implemented";
    return false;
}

void FaceDisplayServer::setId(const std::string &id)
{
    sensorId=id;
}

std::string FaceDisplayServer::getId()
{
    return sensorId;
}

bool FaceDisplayServer::open(yarp::os::Searchable &config)
{
    Property params;
    params.fromString(config.toString().c_str());
    yTrace() << "AnalogServer params are: " << config.toString();

    if (!config.check("name"))
    {
        yError() << "AnalogServer: missing 'name' parameter. This parameter defines the name of ports opened by this device, i.e. '/robot/faceDisplay'. \n\
        Please correct the configuration file\n";
        return false;
    }
    else
         rpcPortName = config.find("name").asString();

    if (!config.check("file"))
    {
        yError() << "AnalogServer: missing 'file' parameter. This parameter defines the name of the device file to use in the system, i.e. '/dev/auxdisp'. \n\
        Please correct the configuration file\n";
        return false;
    }
    else
         deviceFileName = config.find("file").asString();

    if(!rpcPort.open(rpcPortName))
    {
        yError() << "Failed to open port " << rpcPortName.c_str();
        return false;
    }

    // open device file and init some stuff
    start();
    return true;
}

void FaceDisplayServer::onStop()
{
    yTrace();
    rpcPort.interrupt();
    rpcPort.close();
}

void FaceDisplayServer::run()
{
    yarp::os::Bottle command;
    yTrace() << "Started ... waiting for commands";
    while(!isStopping())
    {
        rpcPort.read(command);
        if(!isStopping())
            yDebug() << "FaceDisplayServer: Received command  '" << command.toString() << "'";
    }
}

bool FaceDisplayServer::close()
{
    yTrace();
    if(Thread::isRunning())
    {
        Thread::stop();
    }
    yTrace() << " @ line " << __LINE__;
    Thread::stop();
    detachAll();
    return true;
}

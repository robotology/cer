/*
# Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
* Author: Alberto Cardellino
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*
*/

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <faceDisplayClient.h>

using namespace cer::dev;
using namespace yarp::os;


bool cer::dev::FaceDisplayClient::open(yarp::os::Searchable &config)
{
    ConstString carrier = config.check("carrier", Value("udp"), "default carrier for streaming robot state").asString().c_str();

    local.clear();
    remote.clear();

    local  = config.find("local").asString().c_str();
    remote = config.find("remote").asString().c_str();

    if (local=="")
    {
        yError("FaceDisplayClient: 'local' parameter is missing");
        return false;
    }
    if (remote=="")
    {
        yError("FaceDisplayClient: 'remote' parameter is missing");
        return false;
    }

    if (!rpcPort.open(local.c_str()))
    {
        yError("FaceDisplayClient::open() error could not open rpc port %s, check network", local.c_str());
        return false;
    }

    bool ok=Network::connect(local.c_str(), remote.c_str());
    if (!ok)
    {
        yError("FaceDisplayClient::open() error could not connect to %s\n", remote.c_str());
        return false;
    }

    return true;
}

bool cer::dev::FaceDisplayClient::close()
{
    rpcPort.close();
    return true;
}

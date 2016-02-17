/*
# Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
* Author: Alberto Cardellino
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*
*/

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <faceDisplayClient.h>

#include <yarp/os/Time.h>

using namespace cer::dev;
using namespace yarp::os;

FaceDisplayClient::FaceDisplayClient()
{
    yTrace();
    local.clear();
    remote.clear();
}

FaceDisplayClient::~FaceDisplayClient()
{
    yTrace();
}


bool cer::dev::FaceDisplayClient::open(yarp::os::Searchable &config)
{
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

    int wait=2;
    if(config.check("test"))
    {
        yarp::os::Time::delay(wait);
        setFaceExpression(VOCAB_FACE_HAPPY);
        yarp::os::Time::delay(wait);
        setFaceExpression(VOCAB_FACE_SAD);
        yarp::os::Time::delay(wait);
        setFaceExpression(VOCAB_FACE_WARNING);

        yarp::os::Time::delay(wait);
        setImageFile("homer.bmp");
        yarp::os::Time::delay(wait);
        setImageFile("balette.bmp");
        yarp::os::Time::delay(wait);
        setImageFile("RobotE_PNG_80x32_16bit_01.bmp");
    }
    return true;
}

bool cer::dev::FaceDisplayClient::close()
{
    rpcPort.close();
    return true;
}

bool FaceDisplayClient::setFaceExpression(int faceId)
{
    yTrace() << "\n\tset face" << yarp::os::Vocab::decode(faceId);
    Bottle cmd;
    cmd.addVocab(VOCAB_FACE);
    cmd.addVocab(faceId);

    return rpcPort.write(cmd);
}

bool FaceDisplayClient::getFaceExpression(int* faceId)
{
    yTrace();
    yError() << "Not Yet Implemented FaceDisplayClient::getFaceExpression";
    return false;
}

bool FaceDisplayClient::setImageFile(std::string fileName)
{
    yTrace() << "\n\tset image" << fileName;
    Bottle cmd;
    cmd.addVocab(VOCAB_FILE);
    cmd.addString(fileName);

    return rpcPort.write(cmd);
}

bool FaceDisplayClient::getImageFile(std::string& fileName)
{
    yTrace();
    yError() << "Not Yet Implemented FaceDisplayClient::getFaceExpression";
    return false;
}















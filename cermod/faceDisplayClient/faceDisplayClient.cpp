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
        yError("FaceDisplayClient: could not open rpc port %s", local.c_str());
        return false;
    }

    if(! imagePort.open(local+"/image:o") )
    {
        yError() <<  "FaceDisplayClient: could not open streaming port" << local + "/image:o";
        return false;
    }

    bool ok=Network::connect(local.c_str(), (remote+"/rpc"));
    if (!ok)
    {
        yError("FaceDisplayClient: could not connect to %s\n", (remote+"/rpc").c_str());
        return false;
    }

    ok = Network::connect((local+"/image:o").c_str(), (remote+"/image:i").c_str());
    if (!ok)
    {
        yError("FaceDisplayClient: could not connect to %s\n", (remote+"/image:i").c_str());
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
    cmd.addVocab(VOCAB_SET);
    cmd.addVocab(VOCAB_FACE);
    cmd.addVocab(faceId);

    return rpcPort.write(cmd);
}

bool FaceDisplayClient::getFaceExpression(int* faceId)
{
    yTrace();
    Bottle cmd;
    cmd.addVocab(VOCAB_GET);
    cmd.addVocab(VOCAB_FACE);

    return rpcPort.write(cmd);
}

bool FaceDisplayClient::setImageFile(std::string fileName)
{
    Bottle cmd;
    cmd.addVocab(VOCAB_SET);
    cmd.addVocab(VOCAB_FILE);
    cmd.addString(fileName);

    return rpcPort.write(cmd);
}

bool FaceDisplayClient::getImageFile(std::string& fileName)
{
    yTrace();
    Bottle cmd;
    cmd.addVocab(VOCAB_GET);
    cmd.addVocab(VOCAB_FACE);
    return rpcPort.write(cmd);
}

bool FaceDisplayClient::setImage(yarp::sig::Image img)
{
    yTrace();
    yarp::sig::Image & imgSend =  imagePort.prepare();
    imgSend = img;
    imagePort.write();
    return true;
}

bool FaceDisplayClient::getImage(yarp::sig::Image* img)
{
    Bottle cmd;
    cmd.addVocab(VOCAB_GET);
    cmd.addVocab(VOCAB_IMAGE);

    bool ret =rpcPort.write(cmd);

    if(ret)
        *img = *imagePort.read();

    return ret;
}


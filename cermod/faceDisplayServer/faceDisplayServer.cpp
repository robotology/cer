/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino <alberto.cardellino@iit.it>
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include<opencv/cv.h>
#include<opencv/cxcore.h>
#include<opencv/highgui.h>

#include <faceDisplayServer.h>
#include <IFaceDisplayInterface.h>

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

    if (!config.check("display"))
    {
        yError() << "AnalogServer: missing 'display' parameter. This parameter defines the name of the device file to use in the system, i.e. '/dev/auxdisp'. \n\
        Please correct the configuration file\n";
        return false;
    }
    else
         deviceFileName = config.find("display").asString();

    if(!rpcPort.open(rpcPortName))
    {
        yError() << "Failed to open port " << rpcPortName.c_str();
        return false;
    }

    // open device file and init some stuff
    if ((fd=::open(deviceFileName.c_str(), O_RDWR)) < 0)
    {
        yError() << "Cannot open device file " << deviceFileName;
        return false;
    }

    // THIS SET THE DEAD TIME -- magic numbers from Francesco Diotalevi
    gen_reg.offset=CER_TIME;
    gen_reg.rw=WRITE_REGISTER;
    gen_reg.data=0x440063;
    ioctl (fd, IOC_GEN_REG, &gen_reg);

    // start the thread
    start();
    return true;
}

void FaceDisplayServer::onStop()
{
    yTrace();
    ::close(fd);
    rpcPort.interrupt();
    rpcPort.close();
}

void FaceDisplayServer::run()
{
    yarp::os::Bottle command;
    char imageFileName[255];

    int command_vocab;
    int param;

    while(!isStopping())
    {
        IplImage* img;
        yTrace() << "... waiting for commands";
        if(!rpcPort.read(command))
            continue;

        yTrace() << "Received command " << command.toString();


// Right now it supports commands both as a string, to be used with yarp write and as vocabs.
// The support of strings is probably gonna be dropped in the final version.
        if(command.get(0).isString())
        {
            if(command.get(0).asString() == yarp::os::ConstString("face"))
            {
                command_vocab = VOCAB_FACE;
                if(command.get(1).isString())
                {
                    if(command.get(1).asString() == yarp::os::ConstString("hap"))
                        param = VOCAB_FACE_HAPPY;
                    else
                    if(command.get(1).asString() == yarp::os::ConstString("sad"))
                        param = VOCAB_FACE_SAD;
                    else
                    if(command.get(1).asString() == yarp::os::ConstString("warn"))
                        param = VOCAB_FACE_WARNING;
                    else
                    {
                        yError() << "Received malformed command:" << command.toString();
                        yError() << "Value following vocab 'FACE' must be a supported vocab (" << yarp::os::Vocab::decode(VOCAB_FACE_HAPPY) <<  \
                                ", " << yarp::os::Vocab::decode(VOCAB_FACE_SAD) << ", " << yarp::os::Vocab::decode(VOCAB_FACE_WARNING) << ")";
                                continue;
                    }
                }
            }
            else
            {
                if(command.get(0).asString() == yarp::os::ConstString("file"))
                {
                    command_vocab = VOCAB_FILE;
                }
                else
                {
                    yError() << "Received malformed command:" << command.toString();
                    yError() << "First value must be a supported vocab (" << yarp::os::Vocab::decode(VOCAB_FACE) <<  \
                        ", " << yarp::os::Vocab::decode(VOCAB_FILE) << ")";
                        continue;
                }
            }
        }
        else
        {
            if(!command.get(0).isVocab())
            {
                yError() << "Received malformed command:" << command.toString();
                yError() << "First value must be a supported vocab (" << yarp::os::Vocab::decode(VOCAB_FACE) <<  \
                    ", " << yarp::os::Vocab::decode(VOCAB_FILE) << ")";
                continue;
            }
            command_vocab = command.get(0).asVocab();
            param = command.get(1).asVocab();
        }

        switch(command_vocab)
        {
            case VOCAB_FACE:
            {
//                 if(!command.get(1).isVocab())
//                 {
//                     yError() << "Received malformed command:" << command.toString();
//                     yError() << "Value following vocab 'FACE' must be a supported vocab (" << yarp::os::Vocab::decode(VOCAB_FACE_HAPPY) <<  \
//                         ", " << yarp::os::Vocab::decode(VOCAB_FACE_SAD) << ", " << yarp::os::Vocab::decode(VOCAB_FACE_WARNING) << ")";
//                         continue;
//                 }
                switch(param)
                {
                    case VOCAB_FACE_HAPPY:
                    {
                        snprintf(imageFileName, 255, "%s", "/home/linaro/AUXDISP/RobotE_PNG_80x32_16bit_04.bmp");
                    }
                    break;

                    case VOCAB_FACE_SAD:
                    {
                        snprintf(imageFileName, 255, "%s", "/home/linaro/AUXDISP/RobotE_PNG_80x32_16bit_02.bmp");
                    }
                    break;

                    case VOCAB_FACE_WARNING:
                    {
                        snprintf(imageFileName, 255, "%s", "/home/linaro/AUXDISP/RobotE_PNG_80x32_16bit_01.bmp");
                    }
                    break;

                    default:
                    {
                        yError() << "Received malformed command:" << command.toString();
                        yError() << "Unsupported face expression. Supported values are (" << yarp::os::Vocab::decode(VOCAB_FACE_HAPPY) <<  \
                            ", " << yarp::os::Vocab::decode(VOCAB_FACE_SAD) << ", " << yarp::os::Vocab::decode(VOCAB_FACE_WARNING) << ")";
                            continue;
                    }
                }
            }
            break;

            case VOCAB_FILE:
            {
                if(!command.get(1).isString())
                {
                    yError() << "Received malformed command:" << command.toString();
                    yError() << "Value following vocab 'FILE' must be a string containing file name with absolute path)";
                    continue;
                }
                snprintf(imageFileName, 255, "/home/linaro/AUXDISP/%s", command.get(1).asString().c_str());
            }
            break;

            default:
            {
                yError() << "Received malformed command:" << command.toString();
                yError() << "Unsupported command. Supported commands are (" << yarp::os::Vocab::decode(VOCAB_FACE) <<  \
                    ", " << yarp::os::Vocab::decode(VOCAB_FACE_SAD) << ", " << yarp::os::Vocab::decode(VOCAB_FILE) << ")";
                    continue;
            }
        }

        // load the image
        img = cvLoadImage(imageFileName, 1);

        if(!img)
        {
            yError() << "Cannot load image " << imageFileName;
            continue;
        }

        // Conver into BGR
        cvCvtColor(img,img,CV_BGR2RGB);

        // Set 24bpp otherwise exit
        if (img->depth==IPL_DEPTH_8U)
            ioctl(fd,IOC_SET_BPP,BPP24);

        if(-1 == ::write(fd, img->imageData, img->imageSize) )
            yError() << "Failed setting image to display";
    }
}

bool FaceDisplayServer::close()
{
    yTrace();
    if(Thread::isRunning())
    {
        Thread::stop();
    }
    Thread::stop();
    detachAll();
    ::close(fd);
    return true;
}

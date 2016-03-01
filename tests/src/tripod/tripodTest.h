//  Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
//  Authors: Alberto Cardellino <alberto.cardellino@iit.it>
//
//  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT


#ifndef TRIPOD_TEST_H
#define TRIPOD_TEST_H

#include <rtf/yarp/YarpTestCase.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <rtf/yarp/JointsPosMotion.h>

/**
* \ingroup cer-tests
* Test basic functionalities of CER tripod device
*
*  Accepts the following parameters:
* | Parameter name | Type   | Units | Default Value | Required | Description                                      | Notes |
* |:--------------:|:------:|:-----:|:-------------:|:--------:|:------------------------------------------------:|:-----:|
* | robotName      | string | -     | "cer"         | No       | The prefix name of the ports to connect to.      | -     |
*/

class TripodTest : public YarpTestCase
{
public:
    TripodTest();
    virtual ~TripodTest();

    virtual bool setup(yarp::os::Property& property);
    virtual void run();
    virtual void tearDown();

private:

    yarp::dev::PolyDriver           tripod;
    yarp::sig::Vector               joints;
    RTF::YARP::jointsPosMotion      *helper;

    int numberOfJoints;
    yarp::dev::IPositionControl *iPos;
    yarp::dev::IPositionDirect  *iDir;
    yarp::dev::IControlLimits   *iLim;
    yarp::dev::IControlMode2    *iMode;
    yarp::dev::IEncodersTimed   *iEnc;

    yarp::sig::Vector           encoders, targetPos;
    yarp::sig::Vector           timeStamps;
    yarp::sig::Vector           limitsMax, limitsMin;
};

#endif //TRIPOD_TEST_H

//  Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
//  Authors: Alberto Cardellino <alberto.cardellino@iit.it>
//
//  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT


#ifndef TRIPOD_TEST_H
#define TRIPOD_TEST_H

#include <yarp/rtf/TestCase.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <yarp/rtf/JointsPosMotion.h>

/**
* \ingroup cer-tests
* Test basic functionalities of CER tripod device
*
*  Accepts the following parameters:
* | Parameter name | Type   | Units | Default Value | Required | Description                                      | Notes |
* |:--------------:|:------:|:-----:|:-------------:|:--------:|:------------------------------------------------:|:-----:|
* | robotName      | string | -     | "cer"         | No       | The prefix name of the ports to connect to.      | -     |
* | posTolerance   | double | meter | 0.005         | No       | Max position error                               | one value for all joints  |
* | velTolerance   | double | m/s   | 0.01          | No       | Max velocity error                               | one value for all joints  |
*/

#define DEFAULT_POS_THRESHOLD  0.005  // 5mm
#define DEFAULT_VEL_THRESHOLD  0.01   // 1cm/s

class TripodTest : public yarp::rtf::TestCase
{
public:
    TripodTest();
    virtual ~TripodTest();

    virtual bool setup(yarp::os::Property& property);
    virtual void run();
    virtual void tearDown();

private:

    int     numberOfJoints;
    double  mutualPosTolerance;
    double  timeout;

    yarp::dev::PolyDriver           tripod;
    yarp::sig::Vector               joints;
    yarp::rtf::jointsPosMotion      *helper;

    yarp::dev::IPositionControl     *iPos;
    yarp::dev::IPositionDirect      *iDir;
    yarp::dev::IControlLimits       *iLim;
    yarp::dev::IControlMode2        *iMode;
    yarp::dev::IVelocityControl2    *iVel;
    yarp::dev::IEncodersTimed       *iEnc;

    yarp::sig::Vector           posTolerance;
    yarp::sig::Vector           velTolerance;
    yarp::sig::Vector           encoders, targetPos;
    yarp::sig::Vector           timeStamps;
    yarp::sig::Vector           limitsMax, limitsMin;

    std::vector<int>            modes;
    yarp::sig::Vector           refSpeeds;
    yarp::sig::Vector           encSpeeds;

    bool checkTimeout(double start);
    bool mutualTresholdCheck(yarp::sig::Vector data);
};

#endif //TRIPOD_TEST_H

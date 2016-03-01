
//  Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
//  Authors: Alberto Cardellino <alberto.cardellino@iit.it>
//
//  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT


#include <iostream>
#include <stdlib.h>
#include <rtf/TestAssert.h>
#include <rtf/dll/Plugin.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Property.h>
#include <yarp/os/LogStream.h>

#include <tripodTest.h>

using namespace RTF;
using namespace RTF::YARP;
using namespace yarp::os;


// prepare the plugin
PREPARE_PLUGIN(TripodTest)


/**************** Main Test ***********************/
TripodTest::TripodTest() : YarpTestCase("TripodTest")
{
    joints.resize(0);
    joints.zero();
}

TripodTest::~TripodTest() { }

bool TripodTest::setup(yarp::os::Property& property)
{
    yTrace() << "\nParameters are:\n\t" << property.toString();

    yarp::os::ConstString portPrefix("/tripodTest");
    yarp::os::ConstString robotName = property.check(yarp::os::ConstString("robotName"), yarp::os::Value("cer")).asString();

    yarp::os::Property prop;
    prop.put("device", "remote_controlboard");
    prop.put("local" , portPrefix);
    prop.put("remote", "/"+robotName+"/torso");

    if(!tripod.open(prop))
    {
        yError() << "Cannot connect to tripod device";
        return false;
    }

    // Check interfaces
    tripod.view(iPos);
    tripod.view(iDir);
    tripod.view(iEnc);
    tripod.view(iLim);
    tripod.view(iMode);

    RTF_ASSERT_ERROR_IF((!iPos || !iDir || !iMode || !iEnc || !iLim),
                        Asserter::format("Cannot get the required interfaces"));

    iPos->getAxes(&numberOfJoints);
        RTF_ASSERT_ERROR_IF((numberOfJoints != 3), Asserter::format("Number of joints is not 3, got %d", numberOfJoints) );

    joints.resize(numberOfJoints);
    joints[0] = 0;
    joints[1] = 1;
    joints[2] = 2;

    helper = new jointsPosMotion(&tripod, joints);

    std::vector<int> modes;
    for( int i=0; i< numberOfJoints; i++)
    {
        modes[i] = VOCAB_CM_POSITION_DIRECT;
    }
    iMode->setControlModes(modes.data());

    iMode->getControlModes(modes.data());
    for( int i=0; i< numberOfJoints; i++)
    {
        RTF_ASSERT_ERROR_IF(modes[i] != VOCAB_CM_POSITION_DIRECT,
                            Asserter::format("Cannot change control to position direct mode for joint %d", i));
    }

    encoders.resize(numberOfJoints);
    targetPos.resize(numberOfJoints);
    limitsMin.resize(numberOfJoints);
    limitsMax.resize(numberOfJoints);

    encoders.zero();
    targetPos.zero();

    for( int i=0; i< numberOfJoints; i++)
    {
        iLim->getLimits(i, limitsMin.data(), limitsMax.data() );
    }

    helper->setTolerance(0.01);  // 1cm tolerance
    return true;
}


void TripodTest::run()
{
    iEnc->getEncodersTimed(encoders.data(), timeStamps.data());




}

void TripodTest::tearDown()
{
    yTrace();
}

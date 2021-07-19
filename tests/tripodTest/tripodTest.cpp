
//  Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
//  Authors: Alberto Cardellino <alberto.cardellino@iit.it>
//
//  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT


#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <rtf/TestAssert.h>
#include <yarp/rtf/TestAsserter.h>
#include <rtf/dll/Plugin.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Property.h>
#include <yarp/os/LogStream.h>
#include <tripodTest.h>

using namespace RTF;
using namespace yarp::os;
using namespace yarp::rtf;


// prepare the plugin
PREPARE_PLUGIN(TripodTest)


bool TripodTest::mutualTresholdCheck(yarp::sig::Vector data)
{
    for(int idx=0; idx<data.size(); idx++)
    {
        for(int other=0; other<data.size(); other++)
        {
            if(fabs(data[idx]-data[other]) > mutualPosTolerance)
            {
                return false;
            }
        }
    }
    return true;
}

bool TripodTest::checkTimeout(double start)
{
    if((yarp::os::Time::now() - start) < timeout)
        return false;
    else
        return true;
}

/**************** Main Test ***********************/
TripodTest::TripodTest() : yarp::rtf::TestCase("TripodTest")
{
    joints.resize(0);
    joints.zero();
    timeout = 5;            //seconds
    mutualPosTolerance = DEFAULT_POS_THRESHOLD;
}

TripodTest::~TripodTest() { }

bool TripodTest::setup(yarp::os::Property& property)
{
    yTrace() << "\nParameters are:\n\t" << property.toString();

    // read param from config file
    std::string portPrefix("/tripodTest");
    std::string robotName = property.check("robotName", yarp::os::Value("cer")).asString();
    double pos_tolerance = property.check("posTolerance", yarp::os::Value(DEFAULT_POS_THRESHOLD)).asFloat64();
    double vel_tolerance = property.check("velTolerance", yarp::os::Value(DEFAULT_VEL_THRESHOLD)).asFloat64();
    mutualPosTolerance = pos_tolerance;

    // initialize device
    yarp::os::Property prop;
    prop.put("device", "remote_controlboard");
    prop.put("local" , portPrefix);
    prop.put("remote", "/"+robotName+"/torso_tripod");

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
    tripod.view(iVel);
    tripod.view(iMode);

    RTF_ASSERT_ERROR_IF( !(!iPos || !iDir || !iEnc || !iLim || !iVel || !iMode),
                         Asserter::format("Cannot get the required interfaces - iPos %d - iDir %d - iEnc %d - iLim %d - iVel %d - iMode %d",
                                                                              !!iPos, !!iDir, !! iEnc, !!iLim, !!iVel, !!iMode));

    iPos->getAxes(&numberOfJoints);
    RTF_ASSERT_ERROR_IF( !(numberOfJoints != 3), Asserter::format("Number of joints is not 3, got %d", numberOfJoints) );

    // resize and initialize vectors
    modes     .resize(numberOfJoints);
    joints    .resize(numberOfJoints);
    encoders  .resize(numberOfJoints);
    targetPos .resize(numberOfJoints);
    limitsMin .resize(numberOfJoints);
    limitsMax .resize(numberOfJoints);
    timeStamps.resize(numberOfJoints);
    refSpeeds .resize(numberOfJoints);
    encSpeeds .resize(numberOfJoints);
    posTolerance.resize(numberOfJoints);
    velTolerance.resize(numberOfJoints);

    encoders  .zero();
    targetPos .zero();
    timeStamps.zero();
    refSpeeds .zero();

    joints[0] = 0;
    joints[1] = 1;
    joints[2] = 2;

    // helper class
    helper = new jointsPosMotion(&tripod, joints);
    helper->setTolerance(pos_tolerance);  // 1cm tolerance

    // Configure tripod for testing
    for( int i=0; i< numberOfJoints; i++)
    {
        modes[i] = VOCAB_CM_POSITION;
        posTolerance[i] = pos_tolerance;
        velTolerance[i] = vel_tolerance;
        iPos->setRefSpeed(i, 0.1);
    }
    iMode->setControlModes(modes.data());

    yarp::os::Time::delay(1);
    iMode->getControlModes(modes.data());
    for( int i=0; i< numberOfJoints; i++)
    {
        iLim->getLimits(i, &limitsMin[i], &limitsMax[i] );
        RTF_ASSERT_ERROR_IF( !(modes[i] != VOCAB_CM_POSITION),
                             Asserter::format("Cannot change control to position mode for joint %d", i));
    }

    return true;
}

void TripodTest::run()
{
    yTrace();
    iEnc->getEncodersTimed(encoders.data(), timeStamps.data());

    /*
     *  Start the test in velocity mode
     */

    // Set target position as mid-travel
    for(int i=0; i< numberOfJoints; i++)
    {
        targetPos[i] = (limitsMin[i] + limitsMax[i]) / 2.0;
    }

    RTF_TEST_CHECK(helper->goTo(targetPos, &encoders),
                   Asserter::format("Going to middle position %.2lf --- %.2lf --- %.2lf -->", targetPos[0], targetPos[1], targetPos[2]));

    yarp::os::Time::delay(5);
    RTF_TEST_REPORT(Asserter::format("\n\tMoving the joints together and checking final position is within threshold of %.03lf[m]\n", posTolerance[0]) );
    iEnc->getEncodersTimed(encoders.data(), timeStamps.data());
    RTF_TEST_CHECK(mutualTresholdCheck(encoders),
                   Asserter::format("All motors are aligned to %0.4f", encoders[0]));

    // Set target position to maximum
    RTF_TEST_CHECK(helper->goTo(limitsMax, &encoders),
                   Asserter::format("Going to max limit position %.2lf --- %.2lf --- %.2lf -->", targetPos[0], targetPos[1], targetPos[2]));
    iEnc->getEncodersTimed(encoders.data(), timeStamps.data());
    RTF_TEST_CHECK(mutualTresholdCheck(encoders),
                   Asserter::format("All motors are aligned to %0.4f", encoders[0]));

    yarp::os::Time::delay(5);
    // Set target position to minimum
    RTF_TEST_CHECK(helper->goTo(limitsMin, &encoders),
                   Asserter::format("Going to min limit position %.2lf --- %.2lf --- %.2lf -->", targetPos[0], targetPos[1], targetPos[2]));
    iEnc->getEncodersTimed(encoders.data(), timeStamps.data());
    RTF_TEST_CHECK(mutualTresholdCheck(encoders),
                   Asserter::format("All motors are aligned to %0.4f", encoders[0]));

    iPos->positionMove(limitsMax.data());

    double start = yarp::os::Time::now();
    bool result_single, result_all = true;
    while( (yarp::os::Time::now() - start) < timeout)
    {
        iEnc->getEncoders(encoders.data());
        result_single = mutualTresholdCheck(encoders);
        result_all &= result_single;
        RTF_TEST_FAIL_IF(result_single, Asserter::format("All motors are aligned to %0.4f", encoders[0]));
    }
    RTF_TEST_CHECK(result_all, Asserter::format("All motors are aligned to %0.4f", encoders[0]));

    yarp::os::Time::delay(5);

    /*
     *  Start the test in velocity mode
     */

    // initialize moving the robot in the middle of range, so we can use velocity to move
    // the robot in either direction
    helper->goTo(targetPos);

    RTF_TEST_REPORT(Asserter::format("\n\tMoving the joints in velocity mode to HW limit.\n") );

    modes.resize(numberOfJoints);
    refSpeeds.resize(numberOfJoints);

    for(int i=0; i< numberOfJoints; i++)
    {
        modes[i] = VOCAB_CM_VELOCITY;
        iPos->setRefSpeed(i, 0.1);
    }
    iMode->setControlModes(modes.data());

    yarp::os::Time::delay(1);
    iMode->getControlModes(modes.data());
    for(int i=0; i< numberOfJoints; i++)
    {
        refSpeeds[i] = 0.1;
        RTF_ASSERT_ERROR_IF( !(modes[i] != VOCAB_CM_VELOCITY),
                             Asserter::format("Cannot change control to velocity mode for joint %d", i));
    }

    iVel->velocityMove(refSpeeds.data());

    yarp::sig::Vector prevEncs, hwLimit1, hwLimit2;
    prevEncs.resize(numberOfJoints);
    hwLimit1.resize(numberOfJoints);
    hwLimit2.resize(numberOfJoints);

    // test what tripod is doing -- exit at timeout
//     while( (yarp::os::Time::now() - start) < timeout)
    {
        // check tripod actually started moving:
        // wait until current position is different from initial position,
        // i.e. the motors actually moved a bit
        bool ret;
        while(TestAsserter::isApproxEqual(prevEncs, encoders, posTolerance))
        {
            prevEncs = encoders;
            yarp::os::Time::delay(0.05);
            iEnc->getEncoders(encoders.data());

            if(checkTimeout(start))
            {
                RTF_TEST_FAIL_IF(false, "Seems the tripod is not moving in velocity mode.");
                break;  // Shall I continue the test?
            }
        }

        // read current speed and check with the reference speed.
        iEnc->getEncoderSpeeds(encSpeeds.data());
        RTF_TEST_CHECK(TestAsserter::isApproxEqual(refSpeeds, encSpeeds, velTolerance),
                       Asserter::format("Motor moving at correct speed of %d.", refSpeeds[0]) );

        // wait until the motors stop moving, i.e. the tripod reaches HW limits
        while(TestAsserter::isApproxEqual(prevEncs, encoders, posTolerance) == false)
        {
            prevEncs = encoders;
            yarp::os::Time::delay(0.05);
            iEnc->getEncoders(encoders.data());
        }
        // store first HW limit in a tmp vector
        hwLimit1 = encoders;

        // TODO:: re-do the same thing we just did but in the opposite direction, then measure the span in meters
        // and compare it with the expected one.

    }

}

void TripodTest::tearDown()
{
    yTrace();
    tripod.close();
    if(helper)
        delete helper;

    helper = NULL;
    iPos = NULL;
    iDir = NULL;
    iLim = NULL;
    iEnc = NULL;
    iVel = NULL;
    iMode = NULL;
}

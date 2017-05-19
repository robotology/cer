
//  Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
//  Authors: Alberto Cardellino <alberto.cardellino@iit.it>
//
//  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT


#ifndef DISPLAY_TEST_H
#define DISPLAY_TEST_H

#include <yarp/sig/Image.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>

#include <IFaceDisplayInterface.h>

#include <yarp/rtf/TestCase.h>

/**
* \ingroup cer-tests
* Test basic functionalities of CER display device
*
*  Accepts the following parameters:
* | Parameter name | Type   | Units | Default Value      | Required | Description                                        | Notes |
* |:--------------:|:------:|:-----:|:------------------:|:--------:|:--------------------------------------------------:|:-----:|
* | local          | string | -     | /displayTest       | No       | The prefix name of the local ports to open.        | -     |
* | remote         | string | -     | /robot/faceDisplay | No       | The prefix name of the remote ports to connect to. | -     |
*/

class DisplayTest : public yarp::rtf::TestCase
{
public:
    DisplayTest();
    virtual ~DisplayTest();

    virtual bool setup(yarp::os::Property& property);
    virtual void run();
    virtual void tearDown();

private:
    yarp::sig::Image       image;
    yarp::dev::PolyDriver  display;
    cer::dev::IFaceDisplay *iDisp;
};

#endif //DISPLAY_TEST_H

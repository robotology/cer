
//  Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
//  Authors: Alberto Cardellino <alberto.cardellino@iit.it>
//
//  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT


#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <rtf/TestAssert.h>
#include <rtf/yarp/YarpTestAsserter.h>
#include <rtf/dll/Plugin.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Property.h>
#include <yarp/os/LogStream.h>

#include <displayTest.h>


using namespace RTF;
using namespace RTF::YARP;
using namespace yarp::os;
using namespace yarp::sig;


// prepare the plugin
PREPARE_PLUGIN(DisplayTest)


/**************** Main Test ***********************/
DisplayTest::DisplayTest() : YarpTestCase("DisplayTest")
{

}

DisplayTest::~DisplayTest() { }

bool DisplayTest::setup(yarp::os::Property& property)
{
    yTrace() << "\nParameters are:\n\t" << property.toString();

    // read param from config file
    yarp::os::ConstString local  = property.check("local",  yarp::os::Value("/displayTest")).asString();
    yarp::os::ConstString remote = property.check("remote", yarp::os::Value("/robot/faceDisplay")).asString();

    // initialize device
    yarp::os::Property prop;
    prop.put("device", "faceDisplayClient");
    prop.put("local" , local);
    prop.put("remote", remote);

    if(!display.open(prop))
    {
        yError() << "Cannot connect to display device";
        return false;
    }

    // Check interfaces
    display.view(iDisp);

    RTF_ASSERT_ERROR_IF( !(!iDisp),  Asserter::format("Cannot get the required interface"));

    return true;
}

void DisplayTest::run()
{
    yTrace();

    double wait = 1;
    yarp::os::Time::delay(wait);
    iDisp->setFaceExpression(VOCAB_FACE_HAPPY);
    yarp::os::Time::delay(wait);
    iDisp->setFaceExpression(VOCAB_FACE_SAD);
    yarp::os::Time::delay(wait);
    iDisp->setFaceExpression(VOCAB_FACE_WARNING);

    yarp::os::Time::delay(wait);
    iDisp->setImageFile("homer.bmp");
    yarp::os::Time::delay(wait);
    iDisp->setImageFile("balette.bmp");
    yarp::os::Time::delay(wait);
    iDisp->setImageFile("RobotE_PNG_80x32_16bit_01.bmp");


    // Creating an image to send
    yarp::sig::ImageOf<yarp::sig::PixelRgb> imgTest;

    imgTest.resize(80, 32);

    unsigned char *p;
    for(int base=0; base<80; base++)
    {
        p = imgTest.getRawImage();
        for(int r=0; r<32; r++)
        {
            for(int c=0; c<80; c++)
            {
                *p = (unsigned char) (base+r*8)%255;  p++;
                *p = (unsigned char) (base+c*3)%255;  p++;
                *p = (unsigned char)     base  %255;  p++;
            }
        }
        iDisp->setImage(imgTest);
        yarp::os::Time::delay(0.03);
    }
}

void DisplayTest::tearDown()
{
    yTrace();
    display.close();
    iDisp = NULL;
}

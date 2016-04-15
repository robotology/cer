/*
# Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
* Author:  Alberto Cardellino
* email:   alberto.cardellino@iit.it
*/

#ifndef CER_DEV_FACEDISPLAYCLIENT_H
#define CER_DEV_FACEDISPLAYCLIENT_H


#include <yarp/sig/Image.h>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Thread.h>

#include <IFaceDisplayInterface.h>

namespace cer {
    namespace dev {
        class FaceDisplayClient;
    }
}


#ifndef DOXYGEN_SHOULD_SKIP_THIS

#define DEFAULT_THREAD_PERIOD 20 //ms
/**
 *
 * The client side of any IAnalogSensor capable device.
 * Still single thread! concurrent access is unsafe.
 */
#endif /*DOXYGEN_SHOULD_SKIP_THIS*/

/**
*
* \section FaceDisplayClient Description of input parameters
*
*  This device will connect to the proper analogServer and read the data broadcasted making them available to use for the user application. It also made available some function to check and control the state of the remote sensor.
*
* Parameters accepted in the config argument of the open method:
* | Parameter name | Type   | Units | Default Value | Required  | Description   | Notes |
* |:--------------:|:------:|:-----:|:-------------:|:--------: |:-------------:|:-----:|
* | local          | string |       |               | Yes       | full name if the port opened by the device  | must start with a '/' character |
* | remote         | string |       |               | Yes       | full name of the port the device need to connect to | must start with a '/' character |
*  The device will create a port with name <local> and will connect to a port called <remote> at startup,
* ex: <b> /myModule/faceDisplayClient </b>, and will connect to a port called <b>  /myRobot/faceDisplayServer<b>.
*
**/
class cer::dev::FaceDisplayClient:  public yarp::dev::DeviceDriver,
                                    public cer::dev::IFaceDisplay,
                                    public yarp::os::Thread
{
#ifndef DOXYGEN_SHOULD_SKIP_THIS
protected:
    yarp::os::Port rpcPort;
    yarp::os::BufferedPort<yarp::sig::FlexImage>  imagePort;
    yarp::os::ConstString local;
    yarp::os::ConstString remote;
    std::string deviceId;
    std::string portName;
    int _rate;

#endif /*DOXYGEN_SHOULD_SKIP_THIS*/

public:

    FaceDisplayClient();
    ~FaceDisplayClient();

    /* DevideDriver methods */
    bool open(yarp::os::Searchable& config);
    bool close();

    bool threadInit();
    void run();

    bool setFaceExpression(int faceId);
    bool getFaceExpression(int *faceId);

    bool setImageFile(std::string fileName);
    bool getImageFile(std::string &fileName);

    virtual bool setImage(yarp::sig::Image  img);
    virtual bool getImage(yarp::sig::Image *img);
};

#endif // CER_DEV_FACEDISPLAYCLIENT_H

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino <alberto.cardellino@iit.it>
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef CER_DEV_FACEDISPLAYSERVER_H_
#define CER_DEV_FACEDISPLAYSERVER_H_

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <yarp/os/RateThread.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>


namespace cer{
    namespace dev{
        class FaceDisplayServer;
    }
}

#define DEFAULT_THREAD_PERIOD 20 //ms

/**
 *  @ingroup dev_impl_wrapper
 *
 * \section FaceDisplayServer Description of input parameters
 *
 *  It reads the data from an analog sensor and sends them on one or more ports.
 *  It creates one rpc port and its related handler for every output port..
 *
 * Parameters accepted in the config argument of the open method:
 * | Parameter name | Type    | Units          | Default Value | Required  | Description   | Notes |
 * |:--------------:|:------: |:--------------:|:-------------:|:--------: |:-------------:|:-----:|
 * |  name          | string  |  -             |   -           | Yes       | full name of the port opened by the device, like /robotName/deviceId/sensorType:o | must start with a '/' character |
 * |  file          | string  |  -             | /dev/auxdisp  | Yes       | device file to open  | The device file has to be created when inserting the kernel driver |
 *
 *
 * Configuration file using .ini format
 *
 * \code{.unparsed}
 * [ROS]
 * name          /faceDisplay
 * file          /dev/auxdisp
 * \endcode
 *
 * Configuration file using .xml format.
 *
 * \code{.unparsed}
 *
 *     <param name="name">  /faceDisplay         </param>
 *     <param name="file">  /dev/auxdisp         </param>
 * \endcode
 */

class cer::dev::FaceDisplayServer:  public yarp::os::Thread,
                                    public yarp::dev::DeviceDriver,
                                    public yarp::dev::IMultipleWrapper
{
public:
    // Constructor used by yarp factory
    FaceDisplayServer();

    ~FaceDisplayServer();

    bool open(yarp::os::Searchable &params);
    bool close();
    yarp::os::Bottle getOptions();

    void setId(const std::string &id);
    std::string getId();

    /**
      * Specify which analog sensor this thread has to read from.
      */
    bool attachAll(const yarp::dev::PolyDriverList &p);
    bool detachAll();

    void onStop();

    void run();

private:
#ifndef DOXYGEN_SHOULD_SKIP_THIS
    yarp::os::Port          rpcPort;
    yarp::os::ConstString   rpcPortName;
    yarp::os::ConstString   deviceFileName;

    int _rate;
    yarp::os::ConstString   sensorId;

#endif //DOXYGEN_SHOULD_SKIP_THIS
};

#endif  // CER_DEV_FACEDISPLAYSERVER_H_

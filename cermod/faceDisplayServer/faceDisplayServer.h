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
#include <yarp/sig/Image.h>

#include <unistd.h>
#include <stdint.h>

namespace cer{
    namespace dev{
        class FaceDisplayServer;
    }
}

// Values for kernel driver
#define IOC_MAGIC_NUMBER 0x98
#define IOC_GET_VER      _IOR(IOC_MAGIC_NUMBER, 0, unsigned int)
#define IOC_SET_FC       _IOW(IOC_MAGIC_NUMBER, 1, unsigned int)
#define IOC_GET_CTRL     _IOR(IOC_MAGIC_NUMBER, 2, unsigned int)
#define IOC_RESET_IP     _IOW(IOC_MAGIC_NUMBER, 3, unsigned int)
#define IOC_GEN_REG      _IOWR(IOC_MAGIC_NUMBER, 4, struct auxdisp_regs *)
#define IOC_SET_BPP      _IOW(IOC_MAGIC_NUMBER, 5, unsigned int)

// Registers
#define CER_VERSION 0x00
#define CER_CTRL    0x04
#define CER_INTMSK  0x08
#define CER_INT     0x0C
#define CER_RAWSTAT 0x10
#define CER_TIME    0x14
#define CER_IMPL    0x18

#define READ_REGISTER  0
#define WRITE_REGISTER 1

#define BPP48  (0)
#define BPP24  (1)
#define BPP16  (2)


typedef struct auxdisp_regs {
    uint32_t offset;
    char rw;
    uint32_t data;
} auxdisp_regs_t;

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
    yarp::sig::Image        image;
    yarp::os::ConstString   sensorId;

    int fd;                 // file descriptor for device file
    auxdisp_regs_t          gen_reg;

#endif //DOXYGEN_SHOULD_SKIP_THIS
};

#endif  // CER_DEV_FACEDISPLAYSERVER_H_

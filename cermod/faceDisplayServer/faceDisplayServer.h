/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino <alberto.cardellino@iit.it>
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef CER_DEV_FACEDISPLAYSERVER_H_
#define CER_DEV_FACEDISPLAYSERVER_H_

#include <unistd.h>
#include <stdint.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/LogStream.h>

#include <yarp/sig/Image.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/PolyDriver.h>


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

#define IMAGE_WIDTH    80
#define IMAGE_HEIGHT   32
#define IMAGE_BPP      24


typedef enum
{
    FACE_EXPR = 0,
    IMAGE_EXPR
} FaceExpression;

// To config the driver
typedef struct auxdisp_regs {
    uint32_t offset;
    char rw;
    uint32_t data;
} auxdisp_regs_t;

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

class ImagePort : public yarp::os::BufferedPort<yarp::sig::FlexImage>
{
private:
    int _fd;
    yarp::os::Semaphore *mutex;
    using BufferedPort<yarp::sig::FlexImage>::onRead;

public:

    ImagePort(yarp::os::Semaphore *_mutex) : mutex(_mutex)    { _fd =0; };

    bool init(int fd)   { _fd = fd; }

    virtual void onRead(yarp::sig::FlexImage& img)
    {
        // process data in b
        yDebug() <<  "Got a new image of size: w " << img.width() << " h " << img.height() << " bytesize " <<img.getRawImageSize();

        if( (img.width() != IMAGE_WIDTH) || (img.height() != IMAGE_HEIGHT) || img.getPixelCode() != VOCAB_PIXEL_RGB)
        mutex->wait();
        if(_fd)
        {
            if(-1 == ::write(_fd, img.getRawImage(), img.getRawImageSize()) )
                yError() << "Failed setting image to display";
        }
        else
            yError() << "Display not available or initted";
        mutex->post();
    }
};

class cer::dev::FaceDisplayServer:  public yarp::os::Thread,
                                    public yarp::dev::DeviceDriver
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

    void onStop();

    void run();

private:
#ifndef DOXYGEN_SHOULD_SKIP_THIS
    yarp::os::Port          rpcPort;
    ImagePort               imagePort;      // receive images to be displayed

    std::string             rootPath;
    std::string             deviceFileName;

    int _rate;
    yarp::os::Semaphore     mutex;
    yarp::sig::FlexImage    image;
    std::string             sensorId;

    int fd;                 // file descriptor for device file
    auxdisp_regs_t          gen_reg;

    // for self tests only
    int                     selfTest;
    int                     steps;
    int                     color_choise;

    // version2
    FaceExpression          faceExpression;
    cv::Mat                 face;
    cv::Mat                *currentEye;
    cv::Mat                *currentBars;
    cv::Mat                *currentFace;
    cv::Mat                 imgHappyEye;
    cv::Mat                 imgHappyBars;
    cv::Mat                 imgSadEye;
    cv::Mat                 imgSadBars;
    cv::Mat                 imageFromFile;
    double                  moveUp;
    double                  moveRight;
    float bar_offset_y, le_offset_x, le_offset_y, re_offset_x, re_offset_y;

    // Following values never change
    const int bar_offset_x;
    const float eyeWidth;
    const float eyeHeight;

    // limits WRT rest position
    const int min_offset_x;
    const int max_offset_x;
    const int min_offset_y;
    const int max_offset_y;


    // for timing measurement
    double                  _now, _last, _elapsedTime, _start;
    int                     imageIdx;
    int                     counterImages;
    std::vector<double>     cycleTime;
    void measureTiming();

#endif //DOXYGEN_SHOULD_SKIP_THIS
};

#endif  // CER_DEV_FACEDISPLAYSERVER_H_

/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino <alberto.cardellino@iit.it>
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/ImageFile.h>

#include <sys/ioctl.h>
#include <fcntl.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <faceDisplayServer.h>
#include <IFaceDisplayInterface.h>

using namespace yarp::dev;
using namespace cer::dev;
using namespace yarp::os;
using namespace std;

/**
  * It reads the data from an analog sensor and sends them on one or more ports.
  * It creates one rpc port and its related handler for every output port.
  */

// Constructor used when there is only one output port
FaceDisplayServer::FaceDisplayServer(): sensorId("low-res display"),
                                        selfTest(0),
                                        imagePort(&mutex)
{
    yTrace();
    color_choise = 0;
}

FaceDisplayServer::~FaceDisplayServer()
{
    yTrace();
    threadRelease();
}

void FaceDisplayServer::setId(const std::string &id)
{
    sensorId=id;
}

std::string FaceDisplayServer::getId()
{
    return sensorId;
}

bool FaceDisplayServer::open(yarp::os::Searchable &config)
{
    Property params;
    yarp::os::ConstString   rpcPortName, imagePortName;

    params.fromString(config.toString().c_str());
    yTrace() << "FaceDisplayServer params are: " << config.toString();

    rpcPortName   = config.check("name", yarp::os::Value("/robot/faceDisplay/rpc"), "Name of the port receiving input commands").toString();
    imagePortName = config.check("name", yarp::os::Value("/robot/faceDisplay/image:i"), "Name of the port receiving images").toString();

    deviceFileName = config.check("display", yarp::os::Value("/dev/auxdisp"), "Device file to use in the system, i.e. '/dev/auxdisp'").toString();
    rootPath = config.check("path", yarp::os::Value("/home/linaro/AUXDISP"), "Where images are located").toString();

    if(!rpcPort.open(rpcPortName))
    {
        yError() << "Failed to open port " << rpcPortName.c_str();
        return false;
    }

    imagePort.useCallback();                    // input images will go to onRead() callback
    if(!imagePort.open(imagePortName))
    {
        yError() << "Failed to open port " << imagePortName.c_str();
        return false;
    }

    // open device file and init some stuff
    if ((fd=::open(deviceFileName.c_str(), O_RDWR)) < 0)
    {
        yError() << "Cannot open device file " << deviceFileName.c_str();
        return false;
    }

    if (config.check("red"))
    {
        color_choise = 0;
        yInfo() << "RED";
    }

    if (config.check("green"))
    {
        color_choise = 1;
        yInfo() << "GREEN";
    }

    if (config.check("blue"))
    {
        color_choise = 2;
        yInfo() << "BLUE";
    }

    if (config.check("selfTest"))
    {
        selfTest = config.check("selfTest", yarp::os::Value(1), "Run one of the selt tests from (1 to 3)").asInt();
        yWarning() << "Device working in selfTest " << selfTest << " mode.";
    }

    // used in selfTestmode 2 for now, maybe useful also for automatic transitions later on ... ???
    steps = config.check("steps", yarp::os::Value(30), "Number of transition between start and end").asInt();

    if(steps <=0)
        yError() << "Number of steps must be a positive integer";

    // THIS SET THE DEAD TIME -- magic numbers from Francesco Diotalevi
    gen_reg.offset=CER_TIME;
    gen_reg.rw=WRITE_REGISTER;
    gen_reg.data=0x220063;
//     gen_reg.data=0x110063;
    ioctl(fd, IOC_GEN_REG, &gen_reg);

    // set 24 bit of color per pixel
    ioctl(fd,IOC_SET_BPP,BPP24);


    // clear display
    cv::Mat black(IMAGE_HEIGHT,IMAGE_WIDTH, CV_8UC3, cv::Scalar(0,0,0));
    mutex.wait();
    if(-1 == ::write(fd, black.data, black.total()*3) )
        yError() << "Failed setting image to display";
    mutex.post();
    imagePort.init(fd);

    // start the thread
    start();
    return true;
}

void FaceDisplayServer::onStop()
{
    yTrace();
    rpcPort.interrupt();
    rpcPort.close();
}


void FaceDisplayServer::measureTiming()
{
    static bool initted = false;

    _now = yarp::os::Time::now();

    if(!initted)
    {
        _last = _now;
        _start = _now;
        counterImages = 0;
        cycleTime.clear();

        initted = true;
        return;
    }

    counterImages++;
    _elapsedTime = _now - _last;
    cycleTime.push_back(_elapsedTime);
    _last = _now;

    // report images x sec
    if(_now -_start > 1.0f)
    {
        double _max=0, _min=cycleTime[0], _mean=0, _sum_deviation=0;

        for(int i=0; i<cycleTime.size(); i++)
        {
            _mean += cycleTime[i];
            if(cycleTime[i] > _max)
                _max = cycleTime[i];

            if(cycleTime[i] < _min)
                _min = cycleTime[i];
        }
        _mean=_mean/cycleTime.size();

        for(int i=0; i<cycleTime.size(); i++)
        {
            _sum_deviation += (cycleTime[i]-_mean)*(cycleTime[i]-_mean);
        }

        _sum_deviation = sqrt(_sum_deviation / cycleTime.size());

        yInfo() << "Images per secs: " << counterImages;
        yInfo() << "Time to write an image: Max " << _max << " Min " << _min;
        yInfo() << "Mean " << _mean << " deviation " << _sum_deviation;

        initted = false;
    }
}

void FaceDisplayServer::run()
{
    yarp::os::Bottle command;
    char imageFileName[255];

    int action, type, param;

    if(selfTest == 1)
    {
        // preload the images
        IplImage *img[10];

        img[0] = cvLoadImage(yarp::os::ConstString(rootPath + "/RobotE_PNG_80x32_16bit_01.bmp").c_str(), 1);
        img[1] = cvLoadImage(yarp::os::ConstString(rootPath + "/RobotE_PNG_80x32_16bit_02.bmp").c_str(), 1);
        img[2] = cvLoadImage(yarp::os::ConstString(rootPath + "/RobotE_PNG_80x32_16bit_03.bmp").c_str(), 1);
        img[3] = cvLoadImage(yarp::os::ConstString(rootPath + "/RobotE_PNG_80x32_16bit_03_clean.bmp").c_str(), 1);
        img[4] = cvLoadImage(yarp::os::ConstString(rootPath + "/RobotE_PNG_80x32_16bit_04.bmp").c_str(), 1);

        img[5] = cvLoadImage(yarp::os::ConstString(rootPath + "/RobotE_PNG_80x32_16bit_05.bmp").c_str(), 1);
        img[6] = cvLoadImage(yarp::os::ConstString(rootPath + "/Anger.bmp").c_str(), 1);
        img[7] = cvLoadImage(yarp::os::ConstString(rootPath + "/balette.bmp").c_str(), 1);
        img[8] = cvLoadImage(yarp::os::ConstString(rootPath + "/CCPP2.bmp").c_str(), 1);
        img[9] = cvLoadImage(yarp::os::ConstString(rootPath + "/CCPP4.bmp").c_str(), 1);


        // Convert into BGR
        for(int i=0; i<10; i++)
        {
            if(img[i] == NULL)
                yError() << "img not valid at index " << i;

            cvCvtColor(img[i], img[i], CV_BGR2RGB);

            if(img[i] == NULL)
                yError() << "img not valid at index " << i;
        }

        imageIdx = 0;
        measureTiming();

        while(!isStopping())
        {
            if(-1 == ::write(fd, img[imageIdx]->imageData, img[imageIdx]->imageSize) )
                yError() << "Failed setting image to display";

            imageIdx++;
            if(imageIdx >= 10)
                imageIdx = 0;

            measureTiming();
        }
        return;
    }


    if(selfTest == 2)
    {
        cv::Mat img(IMAGE_HEIGHT,IMAGE_WIDTH, CV_8UC3, cv::Scalar(0,0,0));
        cv::Mat black(IMAGE_HEIGHT,IMAGE_WIDTH, CV_8UC3, cv::Scalar(0,0,0));

        cout << "elemSize " << img.elemSize() << endl;
        cout << "total " << img.total() << endl;

        unsigned long int counter = 0;

        bool toggleColor = false;

        cv::Vec3b color;
        switch(color_choise)
        {
            case 0:
                color[0] = 255;
                color[1] = 0;
                color[2] = 0;
                break;

            case 1:
                color[0] = 0;
                color[1] = 255;
                color[2] = 0;
                break;

            case 2:
                color[0] = 0;
                color[1] = 0;
                color[2] = 255;
                break;

        }
        cv::Vec3b color_white( 0,  255, 0);
        cv::Vec3b color_black( 0, 0, 0);

        ::write(fd, black.data, black.total()*3);

        measureTiming();
        while(!isStopping())
        {
            for(int row=0; (row< img.rows) && (!isStopping()); row++)
            {
                for(int col=16*4; (col<img.cols) && (!isStopping()); col++)
                {
                    if(toggleColor)
                        img.at<cv::Vec3b>(row,col) = color_black;
                    else
                        img.at<cv::Vec3b>(row,col) = color;

                    if(-1 == ::write(fd, img.data, img.total()*3) )
                        yError() << "Failed setting image to display";

                    measureTiming();
                }
            }
            toggleColor = !toggleColor;
        }
        cout << "thread quittting" << endl;
        return;
    }


    if(selfTest == 3)
    {
        bool toggleImage = false;
        cv::Mat image_1  = cv::imread(yarp::os::ConstString(rootPath + "/fra.bmp").c_str(), CV_LOAD_IMAGE_COLOR);
        cv::Mat image_2  = cv::imread(yarp::os::ConstString(rootPath + "/homer.bmp").c_str(), CV_LOAD_IMAGE_COLOR);

        cv::Mat image_tmp   = cv::imread(yarp::os::ConstString(rootPath + "/homer.bmp").c_str(), CV_LOAD_IMAGE_COLOR);


        cv::cvtColor(image_1,image_1,CV_BGR2RGB);
        cv::cvtColor(image_2,image_2,CV_BGR2RGB);

        cv::Mat *image_init, *image_final;

        image_init  = &image_1;
        image_final = &image_2;

        if(-1 == ::write(fd, image_1.data, image_1.total()*3) )
            yError() << "Failed setting image to display";
        yarp::os::Time::delay(2);

        int step = 0;
        while(!isStopping())
        {
            for(int row=0; (row<image_tmp.rows) && (!isStopping()); row++)
            {
                for(int col=16*4; (col<image_tmp.cols) && (!isStopping()); col++)
                {
                    image_tmp.at<cv::Vec3b>(row,col)[0] = ((double) (image_final->at<cv::Vec3b>(row,col)[0] - image_init->at<cv::Vec3b>(row,col)[0]) * step/steps) + image_init->at<cv::Vec3b>(row,col)[0];
                    image_tmp.at<cv::Vec3b>(row,col)[1] = ((double) (image_final->at<cv::Vec3b>(row,col)[1] - image_init->at<cv::Vec3b>(row,col)[1]) * step/steps) + image_init->at<cv::Vec3b>(row,col)[1];
                    image_tmp.at<cv::Vec3b>(row,col)[2] = ((double) (image_final->at<cv::Vec3b>(row,col)[2] - image_init->at<cv::Vec3b>(row,col)[2]) * step/steps) + image_init->at<cv::Vec3b>(row,col)[2];
                }
            }

            if(-1 == ::write(fd, image_tmp.data, image_tmp.total()*3) )
                yError() << "Failed setting image to display";

            step++;

            if(step >= steps)
            {
                toggleImage = !toggleImage;

                if(toggleImage)
                {
                    image_init  = &image_2;
                    image_final = &image_1;
                }
                else
                {
                    image_init  = &image_1;
                    image_final = &image_2;
                }
                step = 0;
                yarp::os::Time::delay(2);
            }
        }
    }


    if(selfTest == 4)
    {
        yInfo() << "Running selfTest 4";

        // load the image
        IplImage* img;
        img = cvLoadImage("/home/linaro/AUXDISP/All_ins/All_greens-dx2sx-central.bmp", 1);

        if(!img)
        {
            yError() << "Cannot load image " << "/home/linaro/AUXDISP/All_ins/All_greens-dx2sx-central.bmp";
            return;
        }

        // Conver into BGR
        cvCvtColor(img,img,CV_BGR2RGB);

        int imageSize = IMAGE_WIDTH*IMAGE_HEIGHT*3;
        int rowSize = IMAGE_WIDTH*3;
        //  there is a dummy line between each image for easier understanding
        while(!isStopping())
        {
            for(int offset=0; offset<img->imageSize && !isStopping(); offset+= (imageSize+rowSize))
            {
                mutex.wait();
                if(-1 == ::write(fd, img->imageData+offset, imageSize) )
                    yError() << "Failed setting image to display";
                mutex.post();
                yarp::os::Time::delay(1.0);
            }
        }
    }


    if(selfTest == 5)
    {
        yInfo() << "Running selfTest 5";

        // load the image
        IplImage* img;
        img = cvLoadImage("/home/linaro/AUXDISP/All_ins/All_greens-up2down-central.bmp", 1);

        if(!img)
        {
            yError() << "Cannot load image " << "/home/linaro/AUXDISP/All_ins/All_greens-up2down-central.bmp";
            return;
        }

        // Conver into BGR
        cvCvtColor(img,img,CV_BGR2RGB);

        int imageSize = IMAGE_WIDTH*IMAGE_HEIGHT*3;
        int rowSize = IMAGE_WIDTH*3;
        //  there is a dummy line between each image for easier understanding
        while(!isStopping())
        {
            int centralOffset = 7*imageSize +7*rowSize;
            for(int offset=centralOffset; (offset<img->imageSize) && (!isStopping()); offset+= (imageSize+rowSize))
            {
                mutex.wait();
                if(-1 == ::write(fd, img->imageData+offset, imageSize) )
                    yError() << "Failed setting image to display";
                mutex.post();
                yarp::os::Time::delay(1.0);
            }

            for(int offset=centralOffset; (offset>0) && (!isStopping()); offset-= (imageSize+rowSize))
            {
                mutex.wait();
                if(-1 == ::write(fd, img->imageData+offset, imageSize) )
                    yError() << "Failed setting image to display";
                mutex.post();
                yarp::os::Time::delay(1.0);
            }
        }
    }


    if(selfTest == 6)
    {
        bool ok = true;

        // Create an empty black image
        cv::Mat face(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, cv::Scalar(0,0,0));

        // Load all required pieces of image
        cv::Mat imgHappyEye  = cv::imread(yarp::os::ConstString(rootPath + "/runtime/happyEye.bmp").c_str(), CV_LOAD_IMAGE_COLOR);
        cv::Mat imgHappyBars = cv::imread(yarp::os::ConstString(rootPath + "/runtime/happyBars.bmp").c_str(), CV_LOAD_IMAGE_COLOR);
        cv::Mat imgSadEye    = cv::imread(yarp::os::ConstString(rootPath + "/runtime/sadEye.bmp").c_str(), CV_LOAD_IMAGE_COLOR);
        cv::Mat imgSadBars   = cv::imread(yarp::os::ConstString(rootPath + "/runtime/sadBars.bmp").c_str(), CV_LOAD_IMAGE_COLOR);
        cv::Mat imgHappyEye_orig  = cv::imread(yarp::os::ConstString(rootPath + "/Robot_espressioni-01.bmp").c_str(), CV_LOAD_IMAGE_COLOR);

        if(imgHappyEye.empty() )
        {
            yError() << "FaceDisplayServer: Failed loading " << yarp::os::ConstString(rootPath + "/runtime/happyEye.bmp") << "image";
            ok = false;
        }

        if(imgHappyBars.empty() )
        {
            yError() << "FaceDisplayServer: Failed loading " << yarp::os::ConstString(rootPath + "/runtime/happyBars.bmp") << "image";
            ok = false;
        }

        if(imgSadEye.empty() )
        {
            yError() << "FaceDisplayServer: Failed loading " << yarp::os::ConstString(rootPath + "/runtime/sadEye.bmp") << "image";
            ok = false;
        }

        if(imgSadBars.empty() )
        {
            yError() << "FaceDisplayServer: Failed loading " << yarp::os::ConstString(rootPath + "/runtime/sadBars.bmp") << "image";
            ok = false;
        }

        if(!ok)
        {
            yError() << "FaceDisplayServer: Failed loading required set of images for the eyes.";
            return;
        }

        yInfo() << "imgFace      cols " << face.cols << " rows " << face.rows << " channels " << face.channels();
        yInfo() << "imgHappyEye  cols " << imgHappyEye.cols << " rows " << imgHappyEye.rows << " channels " << imgHappyEye.channels() << " tot " << imgHappyEye.total() << " elemSize " << imgHappyEye.elemSize();
        yInfo() << "imgHappyBars cols " << imgHappyBars.cols << " rows " << imgHappyBars.rows << " channels " << imgHappyBars.channels() << " tot " << imgHappyBars.total() << " elemSize " << imgHappyBars.elemSize();
        yInfo() << "imgHappyEye  cols " << imgHappyEye_orig.cols << " rows " << imgHappyEye_orig.rows << " channels " << imgHappyEye_orig.channels() << " tot " << imgHappyEye_orig.total() << " elemSize " << face.elemSize();

        cv::imwrite(yarp::os::ConstString(rootPath + "/runtime/runtime-HappyBars.bmp").c_str(), imgHappyBars);


        size_t eyeSize      = imgHappyEye.total()  * imgHappyEye.elemSize();
        size_t eyeLineSize  = imgHappyBars.cols    * imgHappyBars.elemSize();
        size_t barsSize     = imgHappyBars.total() * imgHappyBars.elemSize();

        size_t faceLineSize = face.cols            * face.elemSize();
        size_t faceSize     = face.total()         * face.elemSize();

        yInfo() << "EyeSize  is " << eyeSize;
        yInfo() << "BarsSize is " << barsSize;
        yInfo() << "faceSize is " << faceSize;

        float bar_offset_y, le_offset_x, le_offset_y, re_offset_x, re_offset_y;
        // Following values never change
        const int bar_offset_x  = 2;
        const float eyeWidth    = 21;
        const float eyeHeight   = 14;

        // limits WRT rest position
        const int min_offset_x = -3;
        const int max_offset_x = +3;
        const int min_offset_y = -7;
        const int max_offset_y = +7;

        //
        // Generating base image
        //

        // offset for rest position:
        le_offset_x  = 9;
        le_offset_y  = 9;
        re_offset_x  = 50;
        re_offset_y  = 9;
        bar_offset_y = 11;

        // Copy bars
        imgHappyBars.copyTo(face(cv::Rect(bar_offset_x, bar_offset_y, imgHappyBars.cols, imgHappyBars.rows)));

        // add eyes
        imgHappyEye.copyTo(face(cv::Rect(le_offset_x, le_offset_y, imgHappyEye.cols, imgHappyEye.rows)));
        imgHappyEye.copyTo(face(cv::Rect(re_offset_x, re_offset_y, imgHappyEye.cols, imgHappyEye.rows)));

        cv::imwrite(yarp::os::ConstString(rootPath + "/runtime/runtime-HappyEye.bmp").c_str(), face);
        mutex.wait();
        if(-1 == ::write(fd, face.data, faceSize) )
            yError() << "Failed setting image to display";
        mutex.post();


        while(!isStopping())
        {
            // moving eyes around
            for(int y=min_offset_y; y<max_offset_y && !isStopping(); y++)
            {
                for(int x=min_offset_x; x<max_offset_x && !isStopping(); x++)
                {
                    face = cv::Scalar(0,0,0);

                    face.zeros(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
                    // Copy bars
                    imgHappyBars.copyTo(face(cv::Rect(bar_offset_x, bar_offset_y+(y/2), imgHappyBars.cols, imgHappyBars.rows)));

                    // add eyes
                    imgHappyEye.copyTo(face(cv::Rect(le_offset_x+x, le_offset_y+y, imgHappyEye.cols, imgHappyEye.rows)));
                    imgHappyEye.copyTo(face(cv::Rect(re_offset_x+x, re_offset_y+y, imgHappyEye.cols, imgHappyEye.rows)));

                    cv::imwrite(yarp::os::ConstString(rootPath + "/runtime/runtime-HappyEye_"+yarp::os::Value(x).toString()+"_"+yarp::os::Value(y).toString()+".bmp").c_str(), face);
                    mutex.wait();
                    if(-1 == ::write(fd, face.data, faceSize) )
                        yError() << "Failed setting image to display";
                    mutex.post();

                    yarp::os::Time::delay(0.2);
                }
            }
        }
        return;
    }


    // normal workflow
    while(!isStopping())
    {
        IplImage* img;
        yTrace() << "\n\t... waiting for commands";
        if(!rpcPort.read(command))
            continue;

        yTrace() << "Received command " << command.toString();


        // Right now it supports commands both as a string, to be used with yarp write and as vocabs.
        // The support of strings is probably gonna be dropped in the final version.
        if(command.get(0).isString())
        {
            if(command.get(0).asString() == yarp::os::ConstString("set"))
            {
                action = VOCAB_SET;
                if(command.get(1).asString() == yarp::os::ConstString("face"))
                {
                    type = VOCAB_FACE;
                    if(command.get(2).isString())
                    {
                        if(command.get(2).asString() == yarp::os::ConstString("hap"))
                            param = VOCAB_FACE_HAPPY;
                        else
                        if(command.get(2).asString() == yarp::os::ConstString("sad"))
                            param = VOCAB_FACE_SAD;
                        else
                        if(command.get(2).asString() == yarp::os::ConstString("warn"))
                            param = VOCAB_FACE_WARNING;
                        else
                        {
                            yError() << "Received malformed command:" << command.toString();
                            yError() << "Value following vocab 'FACE' must be a supported vocab (" << yarp::os::Vocab::decode(VOCAB_FACE_HAPPY) <<  \
                                    ", " << yarp::os::Vocab::decode(VOCAB_FACE_SAD) << ", " << yarp::os::Vocab::decode(VOCAB_FACE_WARNING) << ")";
                                    continue;
                        }
                    }
                }
                else
                {
                    if(command.get(1).asString() == yarp::os::ConstString("file"))
                    {
                        type = VOCAB_FILE;
                    }
                    else
                    {
                        yError() << "Received malformed command:" << command.toString();
                        yError() << "First value must be a supported vocab (" << yarp::os::Vocab::decode(VOCAB_FACE) <<  \
                            ", " << yarp::os::Vocab::decode(VOCAB_FILE) << ")";
                            continue;
                    }
                }
            }
            if(command.get(0).asString() == yarp::os::ConstString("get"))
            {
                action = VOCAB_GET;
                if(command.get(1).asString() == yarp::os::ConstString("face"))
                    type = VOCAB_FACE;

                else if(command.get(1).asString() == yarp::os::ConstString("file"))
                    type = VOCAB_FILE;
                else
                {
                    yError() << "Received malformed command:" << command.toString();
                    yError() << "First value must be a supported vocab (" << yarp::os::Vocab::decode(VOCAB_FACE) <<  \
                        ", " << yarp::os::Vocab::decode(VOCAB_FILE) << ")";
                        continue;
                }
            }
        }
        else
        {
            if(!command.get(0).isVocab())
            {
                yError() << "Received malformed command:" << command.toString();
                yError() << "First value must be a supported vocab (" << yarp::os::Vocab::decode(VOCAB_FACE) <<  \
                    ", " << yarp::os::Vocab::decode(VOCAB_FILE) << ")";
                continue;
            }

            action = command.get(0).asVocab();
            type   = command.get(1).asVocab();
            param  = command.get(2).asVocab();
        }

        switch(action)
        {
            case VOCAB_SET:
            {
                switch(type)
                {
                    case VOCAB_FACE:
                    {
                        switch(param)
                        {
                            case VOCAB_FACE_HAPPY:
                            {
                                snprintf(imageFileName, 255, "%s", yarp::os::ConstString(rootPath + "/RobotE_PNG_80x32_16bit_04.bmp").c_str());
                            }
                            break;

                            case VOCAB_FACE_SAD:
                            {
                                snprintf(imageFileName, 255, "%s", yarp::os::ConstString(rootPath + "/RobotE_PNG_80x32_16bit_02.bmp").c_str());
                            }
                            break;

                            case VOCAB_FACE_WARNING:
                            {
                                snprintf(imageFileName, 255, "%s", yarp::os::ConstString(rootPath + "/RobotE_PNG_80x32_16bit_01.bmp").c_str());
                            }
                            break;

                            default:
                            {
                                yError() << "Received malformed command:" << command.toString();
                                yError() << "Unsupported face expression. Supported values are (" << yarp::os::Vocab::decode(VOCAB_FACE_HAPPY) <<  \
                                    ", " << yarp::os::Vocab::decode(VOCAB_FACE_SAD) << ", " << yarp::os::Vocab::decode(VOCAB_FACE_WARNING) << ")";
                                    continue;
                            }
                        }
                    }
                    break;

                    case VOCAB_FILE:
                    {
                        if(!command.get(2).isString())
                        {
                            yError() << "Received malformed command:" << command.toString();
                            yError() << "Value following vocab 'FILE' must be a string containing file name with absolute path)";
                            continue;
                        }
                        snprintf(imageFileName, 255, "%s/%s", rootPath.c_str(), command.get(2).asString().c_str());
                    }
                    break;

                    default:
                    {
                        yError() << "Received malformed SET command:" << command.toString();
                        yError() << "Unsupported command. Supported commands are (" << yarp::os::Vocab::decode(VOCAB_FACE) <<  \
                            ", " << yarp::os::Vocab::decode(VOCAB_IMAGE) << ", " << yarp::os::Vocab::decode(VOCAB_FILE) << ")";
                            continue;
                    }
                }
            }  break;
            // end VOCAB_SET

            case VOCAB_GET:
            {
                switch(type)
                {
                    case VOCAB_FACE:
                    {
                        yDebug() << "Received command: Get Face";
                    }
                    break;

                    case VOCAB_FILE:
                    {
                        yDebug() << " Received command: Get File";
    //                     snprintf(imageFileName, 255, "%s/%s", rootPath.c_str(), command.get(1).asString().c_str());
                    }
                    break;

                    case VOCAB_IMAGE:
                    {
                    yDebug() << "Received command: Get Image";

                    }
                    break;

                    default:
                    {
                        yError() << "Received malformed GET command:" << command.toString();
                        yError() << "Unsupported command. Supported commands are (" << yarp::os::Vocab::decode(VOCAB_FACE) <<  \
                            ", " << yarp::os::Vocab::decode(VOCAB_IMAGE) << ", " << yarp::os::Vocab::decode(VOCAB_FILE) << ")";
                            continue;
                    }
                }
            }
            break;// end VOCAB_GET

            default:
            {
                yError() << "Received malformed command:" << command.toString();
                yError() << "Unsupported command. Supported commands are (" << yarp::os::Vocab::decode(VOCAB_SET) <<  \
                    ", " << yarp::os::Vocab::decode(VOCAB_GET) << ")";
                    continue;
            }
        }


        if(action == VOCAB_SET)
        {
            // load the image
            img = cvLoadImage(imageFileName, 1);

            if(!img)
            {
                yError() << "Cannot load image " << imageFileName;
                continue;
            }

            // Conver into BGR
            cvCvtColor(img,img,CV_BGR2RGB);

            mutex.wait();
            if(-1 == ::write(fd, img->imageData, img->imageSize) )
                yError() << "Failed setting image to display";
            mutex.post();
        }
    }
}

bool FaceDisplayServer::close()
{
    yTrace();
    if(Thread::isRunning())
    {
        Thread::stop();
    }
    Thread::stop();
    cout << "Closing fd" << endl;
    ::close(fd);
    return true;
}

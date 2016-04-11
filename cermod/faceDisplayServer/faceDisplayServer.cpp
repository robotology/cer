/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino <alberto.cardellino@iit.it>
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
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


        yInfo() << "width is " << img[0]->width << "height" << img[0]->height;

        // Convert into BGR
        for(int i=0; i<10; i++)
        {
            if(img[i] == NULL)
                yError() << "img not valid at index " << i;

            cvCvtColor(img[i], img[i], CV_BGR2RGB);

            if(img[i] == NULL)
                yError() << "img not valid at index " << i;
        }


        double  last, elapsedTime, start;
        int     imageIdx            = 0;
        int     counterImages       = 0;
        std::vector<double> cycleTime;

        double  now                 = yarp::os::Time::now();
        start = now;
        while(!isStopping())
        {
            if(-1 == ::write(fd, img[imageIdx]->imageData, img[imageIdx]->imageSize) )
                yError() << "Failed setting image to display";

            imageIdx++;
            if(imageIdx >= 10)
                imageIdx = 0;

            now = yarp::os::Time::now();
            elapsedTime = now - last;
            cycleTime.push_back(elapsedTime);

//             yDebug() << "counterImages: "   << counterImages << " now " << now << "last" << last << "elapsedTime" << elapsedTime
//                                                              << "start" << start << "diff" << start-now;

            counterImages++;
            // report images x sec
            if(now-start > 1.0f)
            {
                double max=0, min=cycleTime[0], mean=0, sum_deviation=0;

                for(int i=0; i<cycleTime.size(); i++)
                {
                    mean += cycleTime[i];
                    if(cycleTime[i] > max)
                        max = cycleTime[i];

                    if(cycleTime[i] < min)
                        min = cycleTime[i];
                }
                mean=mean/cycleTime.size();

                for(int i=0; i<cycleTime.size(); i++)
                {
                    sum_deviation += (cycleTime[i]-mean)*(cycleTime[i]-mean);
                }

                sum_deviation = sqrt(sum_deviation / cycleTime.size());

                yInfo() << "Images per secs: " << counterImages;
                yInfo() << "Time to write an image: Max " << max << " Min " << min;
                yInfo() << "Mean " << mean << " deviation " << sum_deviation;

                counterImages = 0;
                cycleTime.clear();
                now  = yarp::os::Time::now();
                start = now;
            }
            last = now;
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
            if(command.get(0).asString() == yarp::os::ConstString("face"))
            {
                command_vocab = VOCAB_FACE;
                if(command.get(1).isString())
                {
                    if(command.get(1).asString() == yarp::os::ConstString("hap"))
                        param = VOCAB_FACE_HAPPY;
                    else
                    if(command.get(1).asString() == yarp::os::ConstString("sad"))
                        param = VOCAB_FACE_SAD;
                    else
                    if(command.get(1).asString() == yarp::os::ConstString("warn"))
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
                if(command.get(0).asString() == yarp::os::ConstString("file"))
                {
                    command_vocab = VOCAB_FILE;
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
        else
        {
            if(!command.get(0).isVocab())
            {
                yError() << "Received malformed command:" << command.toString();
                yError() << "First value must be a supported vocab (" << yarp::os::Vocab::decode(VOCAB_FACE) <<  \
                    ", " << yarp::os::Vocab::decode(VOCAB_FILE) << ")";
                continue;
            }
            command_vocab = command.get(0).asVocab();
            param = command.get(1).asVocab();
        }

        switch(command_vocab)
        {
            case VOCAB_FACE:
            {
//                 if(!command.get(1).isVocab())
//                 {
//                     yError() << "Received malformed command:" << command.toString();
//                     yError() << "Value following vocab 'FACE' must be a supported vocab (" << yarp::os::Vocab::decode(VOCAB_FACE_HAPPY) <<  \
//                         ", " << yarp::os::Vocab::decode(VOCAB_FACE_SAD) << ", " << yarp::os::Vocab::decode(VOCAB_FACE_WARNING) << ")";
//                         continue;
//                 }
                switch(param)
                {
                    case VOCAB_FACE_HAPPY:
                    {
                        snprintf(imageFileName, 255, "%s", "/home/linaro/AUXDISP/RobotE_PNG_80x32_16bit_04.bmp");
                    }
                    break;

                    case VOCAB_FACE_SAD:
                    {
                        snprintf(imageFileName, 255, "%s", "/home/linaro/AUXDISP/RobotE_PNG_80x32_16bit_02.bmp");
                    }
                    break;

                    case VOCAB_FACE_WARNING:
                    {
                        snprintf(imageFileName, 255, "%s", "/home/linaro/AUXDISP/RobotE_PNG_80x32_16bit_01.bmp");
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
                if(!command.get(1).isString())
                {
                    yError() << "Received malformed command:" << command.toString();
                    yError() << "Value following vocab 'FILE' must be a string containing file name with absolute path)";
                    continue;
                }
                snprintf(imageFileName, 255, "/home/linaro/AUXDISP/%s", command.get(1).asString().c_str());
            }
            break;

            default:
            {
                yError() << "Received malformed command:" << command.toString();
                yError() << "Unsupported command. Supported commands are (" << yarp::os::Vocab::decode(VOCAB_FACE) <<  \
                    ", " << yarp::os::Vocab::decode(VOCAB_FACE_SAD) << ", " << yarp::os::Vocab::decode(VOCAB_FILE) << ")";
                    continue;
            }
        }

        // load the image
        img = cvLoadImage(imageFileName, 1);

        if(!img)
        {
            yError() << "Cannot load image " << imageFileName;
            continue;
        }

        // Conver into BGR
        cvCvtColor(img,img,CV_BGR2RGB);

        // Set 24bpp otherwise exit
        if (img->depth==IPL_DEPTH_8U)
            ioctl(fd,IOC_SET_BPP,BPP24);

        if(-1 == ::write(fd, img->imageData, img->imageSize) )
            yError() << "Failed setting image to display";
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

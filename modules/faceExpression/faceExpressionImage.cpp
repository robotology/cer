#include <cmath>
#include <string>
#include "faceExpressionImage.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <yarp/os/Time.h>
#include <yarp/os/Searchable.h>
#include <yarp/math/Rand.h>


#define BLINK_STEP_NUM  10
#define FACE_WIDTH      80
#define FACE_HEIGHT     32

using namespace cv;
using namespace std;
using namespace yarp::os;
using namespace yarp::math;

FaceExpressionImageModule::FaceExpressionImageModule() :    th(0.2, imagePort),
                                                            imagePath(""),
                                                            blink_per_minute(20)
{

}

bool FaceExpressionImageModule::configure(ResourceFinder &rf)
{
    // Check input params
    if(!rf.check("path"))
    {
        string imagePath_test = rf.findFileByName("images/blink_1.bmp");
        size_t last = imagePath_test.rfind("/");
        imagePath = imagePath_test.substr(0, last);
    }
    else
    {
        imagePath = rf.find("path").asString();
    }

    // Open ports
    if(!rpcPort  .open("/" + getName() + "/rpc") )
    {
        yError() << "Cannot open RPC port";
        return false;
    }

    if(!imagePort.open("/" + getName() + "/image:o") )
    {
        yError() << "Cannot open image streaming port";
        return false;
    }

    if(!attach(rpcPort))
    {
        yError() << "RFModule cannot read from RPC port (" << rpcPort.getName() << ")";
    }

    // Load basic images
    th.blinkEye.push_back(cv::imread(std::string(imagePath + "/blink_1.bmp").c_str(),  CV_LOAD_IMAGE_COLOR));
    th.blinkEye.push_back(cv::imread(std::string(imagePath + "/blink_2.bmp").c_str(),  CV_LOAD_IMAGE_COLOR));
    th.blinkEye.push_back(cv::imread(std::string(imagePath + "/blink_3.bmp").c_str(),  CV_LOAD_IMAGE_COLOR));
    th.blinkEye.push_back(cv::imread(std::string(imagePath + "/blink_4.bmp").c_str(),  CV_LOAD_IMAGE_COLOR));
    th.blinkEye.push_back(cv::imread(std::string(imagePath + "/blink_5.bmp").c_str(),  CV_LOAD_IMAGE_COLOR));
    th.blinkEye.push_back(cv::imread(std::string(imagePath + "/blink_6.bmp").c_str(),  CV_LOAD_IMAGE_COLOR));

    // Load image for bars
    th.blinkBar = imread(std::string(imagePath + "/happyBar.bmp").c_str(), CV_LOAD_IMAGE_COLOR);

    // Load image for 'nose' bar
    th.noseBar = cv::imread(std::string(imagePath + "/noseBar.bmp").c_str(), CV_LOAD_IMAGE_COLOR);
    th.noseBar0_len = th.noseBar.cols;

    // Verify all images are loaded correctly
    bool ok = true;
    for(int i=0; i < th.blinkEye.size(); i++)
    {
        if(th.blinkEye[i].empty() )
        {
            yError() << "Image number " << i << "required for blink sequence was not found.";
            ok = false;
        }
    }

    if(th.blinkBar.empty() )
    {
        yError() << "Image required for hear bars was not found.";
        ok = false;
    }

    if(th.noseBar.empty() )
    {
        yError() << "Image required for nose was not found.";
        ok = false;
    }


    // Quit if something wrong!!
    if(!ok) return false;

    // Check for optional params
    th.hearBar0_x       = rf.check("hearBar0_x",      Value( 1), "horizontal offset from left border of outer ear bar, in pixels from upper left corner of the image, starting from 0").asInt();
    th.hearBar0_y       = rf.check("hearBar0_y",      Value( 6), "vertical offset from bottom border of outer ear bar, in pixels from upper left corner of the image, starting from 0").asInt();
    th.hearBar0_minLen  = rf.check("hearBar0_minLen", Value( 3), "minimum length  of outer ear bar").asInt();
    th.hearBar0_maxLen  = rf.check("hearBar0_maxLen", Value(18), "maximum length  of outer ear bar").asInt();
    th.hearBar0_len     = rf.check("hearBar0_len",    Value(10), "starting length of outer ear bar").asInt();

    th.hearBar1_x       = rf.check("hearBar1_x",      Value( 3), "horizontal offset from left border of inner ear bar, in pixels from upper left corner of the image, starting from 0").asInt();
    th.hearBar1_y       = rf.check("hearBar1_y",      Value( 6), "vertical offset from bottom border of inner ear bar, in pixels from upper left corner of the image, starting from 0").asInt();
    th.hearBar1_minLen  = rf.check("hearBar1_minLen", Value( 4), "minimum length  of inner ear bar").asInt();
    th.hearBar1_maxLen  = rf.check("hearBar1_maxLen", Value(19), "maximum length  of inner ear bar").asInt();
    th.hearBar1_len     = rf.check("hearBar1_len",    Value(11), "starting length of inner ear bar").asInt();

    th.hearBarR0_x = FACE_WIDTH - 1 - th.hearBar0_x;
    th.hearBarR1_x = FACE_WIDTH - 1 - th.hearBar1_x;

    th.threadInit();

    return true;
}

bool FaceExpressionImageModule::respond(const Bottle& command, Bottle& reply)
{
    int cmd = command.get(0).asVocab();

    yTrace() << "Received command " << command.toString();
    switch(cmd)
    {
        case VOCAB_AUDIO_START:
        {
            th.activateBars(true);
        }
        break;

        case VOCAB_AUDIO_STOP:
        {
            th.activateBars(false);
            th.step();
        }
        break;

        case VOCAB_TALK_START:
        {
            th.activateTalk(true);
        }
        break;

        case VOCAB_TALK_STOP:
        {
            th.activateTalk(false);
            th.step();
        }
        break;

        case VOCAB_BLINK:
        {
            th.activateBlink(true);
        }
        break;

        case VOCAB_RESET:
        {
			th.activateDraw(true);
            th.threadRelease();
        }
        break;

        case VOCAB_BLACK_RESET:
        {
            th.activateDraw(false);
            th.blackReset();
        }
        break;

        default:
        {
            yError() << "Unknown command " << command.toString();
        }
        break;
    }
    reply.clear();
    reply.addVocab(yarp::os::Vocab::encode("ok"));
    return true;
}

bool FaceExpressionImageModule::interruptModule()
{
    th.activateBars(false);
    th.activateBlink(false);
    th.threadRelease();
    return true;
}

bool FaceExpressionImageModule::close()
{
    rpcPort.close();
    imagePort.close();
    return true;
}

double FaceExpressionImageModule::getPeriod()
{

    return 0.1;
}

bool FaceExpressionImageModule::updateModule()
{
    bool doubleBlink = 0;

    period_percent = ((float)rand() / (float)RAND_MAX);
    period_sec = 60 / (blink_per_minute + 6 * period_percent) / 0.1;

    static int shall_blink = 0;
    float r;

    shall_blink++;

    if(shall_blink >= period_sec)
    {
        th.activateBlink(true);
        r = ((float)rand() / (float)RAND_MAX) * 100;
        r < 30 ? doubleBlink = true : doubleBlink = false;
        shall_blink = 0;
    }

    if(th.doBlink || th.doBars || th.doTalk)
        th.step();

    if(doubleBlink)
    {
        th.activateBlink(true);
        th.step();
    }
    return true;
}


BlinkThread::BlinkThread(unsigned int _period, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >  &imagePort):
                                                PeriodicThread((double)_period/1000.0),
                                                port(imagePort),
                                                barWidth  ( 1),
                                                eyeWidth  (21),
                                                eyeHeight (14),
                                                leftEye_x ( 9), leftEye_y (9),
                                                rightEye_x(50), rightEye_y(9),
                                                noseBar0_x(36), noseBar0_y(16), noseBar0_len( 8), noseBar0_maxLen(8),
                                                hearBar0_x( 1), hearBar0_y( 6), hearBar0_len(14), hearBar0_minLen(3), hearBar0_maxLen(18),
                                                hearBar1_x( 3), hearBar1_y( 6), hearBar1_len(16), hearBar1_minLen(4), hearBar1_maxLen(19),
                                                faceWidth (FACE_WIDTH), faceHeight(FACE_HEIGHT),
                                                blackBar(32, 1, CV_8UC3, cv::Scalar(0,0,0)),
                                                face(FACE_HEIGHT, FACE_WIDTH, CV_8UC3, cv::Scalar(0,0,0)),
                                                faceRest(FACE_HEIGHT, FACE_WIDTH, CV_8UC3, cv::Scalar(0,0,0)),
                                                faceBlack(FACE_HEIGHT, FACE_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0)),
                                                index(0), doBlink(false), doBars(false), doTalk(false), doDraw(true)
{
    indexes[ 0] = 0;
    indexes[ 1] = 1;
    indexes[ 2] = 2;
    indexes[ 3] = 3;
    indexes[ 4] = 4;
    indexes[ 5] = 5;
    indexes[ 6] = 4;
    indexes[ 7] = 3;
    indexes[ 8] = 2;
    indexes[ 9] = 1;
    indexes[10] = 0;

    delays[ 0] = 0.045;
    delays[ 1] = 0.045;
    delays[ 2] = 0.045;
    delays[ 3] = 0.015;
    delays[ 4] = 0.015;
    delays[ 5] = 0.015;
    delays[ 6] = 0.065;
    delays[ 7] = 0.065;
    delays[ 8] = 0.065;
    delays[ 9] = 0.065;
    delays[10] = 0.065;

    Rand::init();
}

void BlinkThread::afterStart(bool s) { }

void BlinkThread::activateBars(bool activate)
{
    mutex.lock();
    doBars = activate;
    mutex.unlock();
}

void BlinkThread::activateTalk(bool activate)
{
    mutex.lock();
    doTalk = activate;
    mutex.unlock();
}

void BlinkThread::activateBlink(bool activate)
{
    mutex.lock();
    doBlink = activate;
    mutex.unlock();
}

bool BlinkThread::threadInit()
{
    //
    // Generate base image
    //

    // Reset image to black
    faceRest.setTo(cv::Scalar(0,0,0));

    // Copy eyes
    ((Mat)blinkEye[indexes[0]]).copyTo(faceRest(cv::Rect(leftEye_x,  leftEye_y,  eyeWidth, eyeHeight)));
    ((Mat)blinkEye[indexes[0]]).copyTo(faceRest(cv::Rect(rightEye_x, rightEye_y, eyeWidth, eyeHeight)));

    //
    // Add bars:
    //

    // Get color from image
    barColor = Scalar(blinkBar.at<cv::Vec3b>(0, 0));

    // Create a Mat for maximum bar size
    blinkBar.resize(hearBar1_maxLen, barWidth);
    blinkBar.setTo(barColor);

    // Left side
    blinkBar(Rect(0, 0, barWidth, hearBar0_len)).  copyTo  (faceRest(cv::Rect(hearBar0_x, faceHeight - hearBar0_y - hearBar0_len, barWidth, hearBar0_len)));
    blinkBar(Rect(0, 0, barWidth, hearBar1_len)).  copyTo  (faceRest(cv::Rect(hearBar1_x, faceHeight - hearBar1_y - hearBar1_len, barWidth, hearBar1_len)));

    // Right side
    blinkBar(Rect(0, 0, barWidth, hearBar0_len)).  copyTo  (faceRest(cv::Rect(hearBarR0_x, faceHeight - hearBar0_y - hearBar0_len, barWidth, hearBar0_len)));
    blinkBar(Rect(0, 0, barWidth, hearBar1_len)).  copyTo  (faceRest(cv::Rect(hearBarR1_x, faceHeight - hearBar1_y - hearBar1_len, barWidth, hearBar1_len)));

    // Add nose
    noseBar.copyTo(faceRest(cv::Rect(noseBar0_x, noseBar0_y, noseBar.cols, noseBar.rows)));

    faceRest.copyTo(face);

    yarp::sig::ImageOf<yarp::sig::PixelRgb> &img = port.prepare();
    img.setExternal(faceRest.data, faceWidth, faceHeight);
    port.writeStrict();

    return true;
}

void BlinkThread::run()
{
	if (doDraw==false)
	{
		return;
	}
	
    mutex.lock();
    // Compute hear bars sizes
    // Left side
    float percentage = 0.5;

    //
    // Add bars
    //

    do
    {
        if(doBars)
        {
            percentage = (float)rand() / (float)RAND_MAX;
        }
        updateBars(percentage);

        if(doBlink)
        {
            index++;
            updateBlink(index);

            if(index >= BLINK_STEP_NUM)
            {
                index = 0;
                doBlink = false;
            }
        }

        yarp::sig::ImageOf<yarp::sig::PixelRgb> &img = port.prepare();
        if(doTalk)
        {
            updateTalk();
            img.setExternal(faceMouth.data, faceWidth, faceHeight);
        }
        else
        {
            img.setExternal(face.data, faceWidth, faceHeight);
        }
        port.writeStrict();

        if(doBlink)
            yarp::os::Time::delay(delays[index]);
    }
    while(doBlink);
    
    mutex.unlock();
}

bool BlinkThread::updateBars(float percentage)
{
    hearBar0_len = hearBar0_minLen + (hearBar0_maxLen - hearBar0_minLen) *  percentage;
    hearBar1_len = hearBar1_minLen + (hearBar1_maxLen - hearBar1_minLen) *  percentage;

    // Reset bars to black
    blackBar(Rect(0, 0, barWidth, 32)).  copyTo  (face(cv::Rect(hearBar0_x,  0, barWidth, 32)));
    blackBar(Rect(0, 0, barWidth, 32)).  copyTo  (face(cv::Rect(hearBar1_x,  0, barWidth, 32)));
    blackBar(Rect(0, 0, barWidth, 32)).  copyTo  (face(cv::Rect(hearBarR0_x, 0, barWidth, 32)));
    blackBar(Rect(0, 0, barWidth, 32)).  copyTo  (face(cv::Rect(hearBarR1_x, 0, barWidth, 32)));

    // Left side
    blinkBar(Rect(0, 0, barWidth, hearBar0_len)).  copyTo  (face(cv::Rect(hearBar0_x, faceHeight - hearBar0_y - hearBar0_len, barWidth, hearBar0_len)));
    blinkBar(Rect(0, 0, barWidth, hearBar1_len)).  copyTo  (face(cv::Rect(hearBar1_x, faceHeight - hearBar1_y - hearBar1_len, barWidth, hearBar1_len)));

    // Right side
    blinkBar(Rect(0, 0, barWidth, hearBar0_len)).  copyTo  (face(cv::Rect(hearBarR0_x, faceHeight - hearBar0_y - hearBar0_len, barWidth, hearBar0_len)));
    blinkBar(Rect(0, 0, barWidth, hearBar1_len)).  copyTo  (face(cv::Rect(hearBarR1_x, faceHeight - hearBar1_y - hearBar1_len, barWidth, hearBar1_len)));

    return true;
}

bool BlinkThread::updateBlink(int index)
{
    // Copy eyes
    ((Mat)blinkEye[indexes[index]]).copyTo(face(cv::Rect(leftEye_x,  leftEye_y,  eyeWidth, eyeHeight)));
    ((Mat)blinkEye[indexes[index]]).copyTo(face(cv::Rect(rightEye_x, rightEye_y, eyeWidth, eyeHeight)));

    // Add nose
    noseBar.copyTo(face(cv::Rect(noseBar0_x, noseBar0_y, noseBar.cols, noseBar.rows)));
    return true;
}

void BlinkThread::updateTalk()
{
    face.copyTo(faceMouth);
    int pixels=faceHeight>>1;
    int y=faceHeight-2;
    for (int x=(faceWidth-pixels)>>1; x<(faceWidth+pixels)>>1; x++)
    {
        int y_=y+round(Rand::scalar(-1,1));
        faceMouth.at<cv::Vec3b>(y_,x)[0]=0;
        faceMouth.at<cv::Vec3b>(y_,x)[1]=255;
        faceMouth.at<cv::Vec3b>(y_,x)[2]=0;
    }
}

void BlinkThread::threadRelease()
{
    mutex.lock();
    yarp::sig::ImageOf<yarp::sig::PixelRgb> &img = port.prepare();
    img.setExternal(faceRest.data, faceWidth, faceHeight);
    port.writeStrict();
    mutex.unlock();
}

void BlinkThread::activateDraw(bool activate)
{
    mutex.lock();
    doDraw = activate;
    mutex.unlock();
}

void BlinkThread::blackReset()
{
    mutex.lock();
    yarp::sig::ImageOf<yarp::sig::PixelRgb> &img = port.prepare();
    img.setExternal(faceBlack.data, faceWidth, faceHeight);
    port.writeStrict();
    mutex.unlock();
}

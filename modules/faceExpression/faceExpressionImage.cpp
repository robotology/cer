#include "faceExpressionImage.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <yarp/os/Time.h>
#include <yarp/os/Searchable.h>


#define BLINK_STEP_NUM  10
#define FACE_WIDTH      80
#define FACE_HEIGHT     32

using namespace cv;
using namespace std;
using namespace yarp::os;

FaceExpressionImageModule::FaceExpressionImageModule() :    th(0.2, imagePort),
                                                            imagePath("/home/linaro/Anand"),
                                                            blink_per_minute(20)
{

}

bool FaceExpressionImageModule::configure(ResourceFinder &rf)
{
    // Check input params
    if(rf.check("path"))
        imagePath = rf.find(ConstString("path")).asString();

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

    Network::connect("/" + getName() + "/image:o", "/yarpview/img:i");

    // Load basic images
    th.blinkEye.push_back(cv::imread(yarp::os::ConstString(imagePath + "/blink_1.bmp").c_str(),  CV_LOAD_IMAGE_COLOR));
    th.blinkEye.push_back(cv::imread(yarp::os::ConstString(imagePath + "/blink_2.bmp").c_str(),  CV_LOAD_IMAGE_COLOR));
    th.blinkEye.push_back(cv::imread(yarp::os::ConstString(imagePath + "/blink_3.bmp").c_str(),  CV_LOAD_IMAGE_COLOR));
    th.blinkEye.push_back(cv::imread(yarp::os::ConstString(imagePath + "/blink_4.bmp").c_str(),  CV_LOAD_IMAGE_COLOR));
    th.blinkEye.push_back(cv::imread(yarp::os::ConstString(imagePath + "/blink_5.bmp").c_str(),  CV_LOAD_IMAGE_COLOR));
    th.blinkEye.push_back(cv::imread(yarp::os::ConstString(imagePath + "/blink_6.bmp").c_str(),  CV_LOAD_IMAGE_COLOR));

    // Load image for bars
    th.blinkBar = imread(yarp::os::ConstString(imagePath + "/happyBar.bmp").c_str(), CV_LOAD_IMAGE_COLOR);

    // Load image for 'nose' bar
    th.noseBar = cv::imread(yarp::os::ConstString(imagePath + "/noseBar.bmp").c_str(), CV_LOAD_IMAGE_COLOR);
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
        }
        break;

        case VOCAB_BLINK:
        {
            th.activateBlink(true);
        }
        break;

        case VOCAB_RESET:
        {
            th.threadRelease();
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
    yInfo() << "period_sec "<<  period_sec;



    static int ciao = 0;
    float r;


    ciao++;
    yInfo() << "RRRR "<<  r << " ciao " << ciao;


    if(ciao >= period_sec)
    {
        th.activateBlink(true);

        ciao = 0;

        r = ((float)rand() / (float)RAND_MAX) * 100;
        r < 30 ? doubleBlink = true : doubleBlink = false;
    }

    if(th.doBlink || th.doBars)
        th.step();

    if(doubleBlink)
    {
        yError() << "Double blink!!";
        th.activateBlink(true);
        th.step();
    }
    return true;
}


BlinkThread::BlinkThread(unsigned int _period, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >  &imagePort):
                                                RateThread(_period),
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
                                                index(0), doBlink(false), doBars(false)
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

    hearBarR0_x = FACE_WIDTH - 1 - hearBar0_x;
    hearBarR1_x = FACE_WIDTH - 1 - hearBar1_x;
}

void BlinkThread::afterStart(bool s)
{
    yDebug() << "After start";
}

void BlinkThread::activateBars(bool activate)
{
    doBars = activate;
}

void BlinkThread::activateBlink(bool activate)
{
    doBlink = activate;
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

    // Set current hear bars size as from original image
//     hearBar0_len = blinkBar.rows;
//     hearBar1_len = blinkBar.rows +1;

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
}

void BlinkThread::run()
{
    yTrace();
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
            updateBars(percentage);
        }

        if(doBlink)
        {
            yTrace() << __LINE__;
            index++;
            updateBlink(index);

            if(index >= BLINK_STEP_NUM)
            {
                index = 0;
                doBlink = false;
            }
        }

        yarp::sig::ImageOf<yarp::sig::PixelRgb> &img = port.prepare();
        img.setExternal(face.data, faceWidth, faceHeight);
        port.writeStrict();

        if(doBlink)
            yarp::os::Time::delay(delays[index]);
    }
    while(doBlink);
}

bool BlinkThread::updateBars(float percentage)
{
    hearBar0_len = hearBar0_minLen + (hearBar0_maxLen - hearBar0_minLen) *  percentage;
    hearBar1_len = hearBar1_minLen + (hearBar1_maxLen - hearBar1_minLen) *  percentage;

    yDebug() << "% " << percentage <<  " ** len : " << hearBar0_len << " ** " << hearBar1_len;

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
}

bool BlinkThread::updateBlink(int index)
{
    // Copy eyes
    ((Mat)blinkEye[indexes[index]]).copyTo(face(cv::Rect(leftEye_x,  leftEye_y,  eyeWidth, eyeHeight)));
    ((Mat)blinkEye[indexes[index]]).copyTo(face(cv::Rect(rightEye_x, rightEye_y, eyeWidth, eyeHeight)));

    // Add nose
    noseBar.copyTo(face(cv::Rect(noseBar0_x, noseBar0_y, noseBar.cols, noseBar.rows)));
}


void BlinkThread::threadRelease()
{
    yarp::sig::ImageOf<yarp::sig::PixelRgb> &img = port.prepare();
    img.setExternal(faceRest.data, faceWidth, faceHeight);
    port.writeStrict();
}


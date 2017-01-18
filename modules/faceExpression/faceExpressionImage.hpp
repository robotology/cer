#include <string>
#include <iostream>

#include <opencv/cv.h>
#include <opencv2/core/mat.hpp>

#include <yarp/sig/Image.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>


#define VOCAB_AUDIO_START       VOCAB4('a','s','t','a')
#define VOCAB_AUDIO_STOP        VOCAB4('a','s','t','o')
#define VOCAB_BLINK             VOCAB4('b','l','i','n')
#define VOCAB_RESET             VOCAB3('r','s','t')

class BlinkThread : public yarp::os::RateThread
{
public:
    BlinkThread(unsigned int _period, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >  &imagePort);

    cv::Mat                 face;
    cv::Mat                 faceRest;
    cv::Mat                 noseBar;
    cv::Mat                 blinkBar;
    cv::Mat                 blackBar;
    std::vector<cv::Mat>    blinkEye;

    cv::Scalar              barColor;   // Color of bar as loaded from file

    // Offset values for placing stuff and size
    int barWidth;
    int eyeWidth;
    int eyeHeight;
    int faceWidth, faceHeight;
    int hearBar0_y, hearBar0_x, hearBar0_minLen, hearBar0_maxLen;
    int hearBar1_y, hearBar1_x, hearBar1_minLen, hearBar1_maxLen;
    int noseBar0_y, noseBar0_x, noseBar0_maxLen;

    int hearBarR0_x;    // Shall be constant as well, but cannot determine size at compile time
    int hearBarR1_x;

    // Where to place the eyes
    float leftEye_x,  leftEye_y;
    float rightEye_x, rightEye_y;

    // Actual size of bars, changing
    int noseBar0_len;
    int hearBar0_len;
    int hearBar1_len;

    bool configure(yarp::os::ResourceFinder &_rf, yarp::os::Property options);
    bool threadInit();
    void threadRelease();
    void afterStart(bool s);
    void run();

    void activateBlink(bool activate);
    void activateBars (bool activate);

    bool updateBars(float percentage);
    bool updateBlink(int index);

    bool doBlink;
    bool doBars;

private:
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >  &port;
    int index;
    int indexes[11];
    float delays[11];
};

class FaceExpressionImageModule: public yarp::os::RFModule
{
private:
    BlinkThread             th;
    yarp::os::Port          rpcPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >    imagePort;

    yarp::os::ConstString   imagePath;

    float period_percent;
    float blink_per_minute;
    float period_sec;

public:
    FaceExpressionImageModule();
    bool configure(yarp::os::ResourceFinder &rf);
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    bool close();
    bool interruptModule();
    double getPeriod();
    bool   updateModule();
};


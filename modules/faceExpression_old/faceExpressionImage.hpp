#include <mutex>
#include <string>
#include <iostream>

#include <opencv2/core/version.hpp>
#include <opencv2/core/mat.hpp>

#include <yarp/sig/Image.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>


#define VOCAB_AUDIO_START       yarp::os::createVocab('a','s','t','a')
#define VOCAB_AUDIO_STOP        yarp::os::createVocab('a','s','t','o')
#define VOCAB_TALK_START        yarp::os::createVocab('t','s','t','a')
#define VOCAB_TALK_STOP         yarp::os::createVocab('t','s','t','o')
#define VOCAB_BLINK             yarp::os::createVocab('b','l','i','n')
#define VOCAB_RESET             yarp::os::createVocab('r','s','t')
#define VOCAB_BLACK_RESET       yarp::os::createVocab('b','l','c','k')

class BlinkThread : public yarp::os::PeriodicThread
{
public:
    BlinkThread(unsigned int _period, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >  &imagePort);

    cv::Mat                 face;
    cv::Mat                 faceRest;
    cv::Mat                 faceBlack;
    cv::Mat                 faceMouth;
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

    bool threadInit()  override;
    void threadRelease()  override;
    void afterStart(bool s)  override;
    void run() override;

    void activateBlink(bool activate);
    void activateBars (bool activate);
    void activateTalk (bool activate);
    void activateDraw (bool activate);

    bool updateBars(float percentage);
    bool updateBlink(int index);
    void updateTalk();
    void blackReset();

    bool doBlink;
    bool doBars;
    bool doTalk;
    bool doDraw;

private:
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >  &port;
    int index;
    int indexes[11];
    float delays[11];
    std::mutex mtx;
};

class FaceExpressionImageModule: public yarp::os::RFModule
{
private:
    BlinkThread             th;
    yarp::os::Port          rpcPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >    imagePort;

    std::string imagePath;

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


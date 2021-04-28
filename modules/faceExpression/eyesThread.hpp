#ifndef EYES_THREAD_HPP
#define EYES_THREAD_HPP

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
#include <yarp/dev/audioRecorderStatus.h>
#include <yarp/dev/audioPlayerStatus.h>

#define VOCAB_AUDIO_START       yarp::os::createVocab('a','s','t','a')
#define VOCAB_AUDIO_STOP        yarp::os::createVocab('a','s','t','o')
#define VOCAB_TALK_START        yarp::os::createVocab('t','s','t','a')
#define VOCAB_TALK_STOP         yarp::os::createVocab('t','s','t','o')
#define VOCAB_BLINK             yarp::os::createVocab('b','l','i','n')
#define VOCAB_RESET             yarp::os::createVocab('r','s','t')
#define VOCAB_BLACK_RESET       yarp::os::createVocab('b','l','c','k')

class EyesThread : public yarp::os::PeriodicThread
{
public:
    EyesThread(yarp::os::ResourceFinder& _rf, std::string _moduleName, double _period, cv::Mat& _image, std::mutex& _mutex);

    yarp::os::ResourceFinder& m_rf;
    std::mutex& m_mutex;
    std::string m_imagePath;
    cv::Mat&                m_face;
    std::string m_moduleName;

    cv::Mat                 faceRest;
    cv::Mat                 faceBlack;
    cv::Mat                 noseBar;
    cv::Mat                 blinkBar;
    cv::Mat                 blackBar;
    std::vector<cv::Mat>    blinkEye;

    double                  m_last_blink = 0;
    cv::Scalar              barColor;

    // Offset values for placing stuff and size
    int eyeWidth=21;
    int eyeHeight=14;
    
    // Where to place the eyes
    float leftEye_x=9;
    float leftEye_y=9;
    float rightEye_x=50;
    float rightEye_y=9;

    // Nose
    int noseBar0_len=8;
    int noseBar0_x=36;
    int noseBar0_y=16;
    int noseBar0_maxLen8;

    bool threadInit()  override;
    void threadRelease()  override;
    void afterStart(bool s)  override;
    void run() override;

    void activateBlink(bool activate);
    bool updateBlink(int index);
    void reset();

    bool m_doBlink;

private:
    int index=0;
    int indexes[11];
    float delays[11];
};

#endif

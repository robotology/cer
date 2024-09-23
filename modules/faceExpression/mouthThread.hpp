#ifndef MOUTH_THREAD_HPP
#define MOUTH_THREAD_HPP

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
#include <yarp/sig/AudioRecorderStatus.h>
#include <yarp/sig/AudioPlayerStatus.h>

#define VOCAB_AUDIO_START       yarp::os::createVocab32('a','s','t','a')
#define VOCAB_AUDIO_STOP        yarp::os::createVocab32('a','s','t','o')
#define VOCAB_TALK_START        yarp::os::createVocab32('t','s','t','a')
#define VOCAB_TALK_STOP         yarp::os::createVocab32('t','s','t','o')
#define VOCAB_BLINK             yarp::os::createVocab32('b','l','i','n')
#define VOCAB_RESET             yarp::os::createVocab32('r','s','t')
#define VOCAB_BLACK_RESET       yarp::os::createVocab32('b','l','c','k')

class MouthThread : public yarp::os::PeriodicThread
{
public:
    MouthThread(yarp::os::ResourceFinder& _rf, std::string _moduleName, double _period, cv::Mat& _image, std::mutex& _mutex);

private:
    yarp::os::ResourceFinder& m_rf;
    yarp::os::BufferedPort<yarp::sig::AudioPlayerStatus > m_audioPlayPort;
    std::mutex&             m_drawing_mutex;
    std::recursive_mutex    m_methods_mutex;
    std::string             m_imagePath;
    std::string             m_moduleName;

    cv::Mat&                m_face;
    cv::Mat                 m_defaultPlainMouth;
    cv::Mat                 m_blackMouth;

    size_t                  m_mouth_w=16;
    size_t                  m_mouth_h=3;

    bool                    m_doTalk = false;
    bool                    m_audioIsPlaying = false;
    bool                    m_drawEnable = true;

    cv::Scalar              m_mouthDefaultColor = cv::Scalar(0, 128, 0);
    cv::Scalar              m_mouthCurrentColor = cv::Scalar(0, 128, 0);

    void updateTalk();
    void clearWithBlack();

public:
    bool threadInit()  override;
    void threadRelease()  override;
    void afterStart(bool s)  override;
    void run() override;

    void activateTalk(bool activate);
    void resetToDefault();
    void enableDrawing(bool activate);
    void setColor(float vr, float vg, float vb);

};

#endif

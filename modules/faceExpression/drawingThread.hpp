#ifndef DRAWING_THREAD_HPP
#define DRAWING_THREAD_HPP

#include <mutex>
#include <string>
#include <iostream>
#include "utils.hpp"

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

class DrawingThread : public yarp::os::PeriodicThread
{
public:
    DrawingThread(yarp::os::ResourceFinder& _rf, std::string _moduleName, double _period, cv::Mat& _image, std::mutex& _mutex);

private:
    yarp::os::ResourceFinder& m_rf;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> m_imageOutPort;
    std::mutex&             m_mutex;
    cv::Mat& m_image;
    std::string m_moduleName;

public:
    bool threadInit()  override;
    void threadRelease()  override;
    void afterStart(bool s)  override;
    void run() override;

    void blackReset();
};

#endif

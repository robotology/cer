#ifndef FACE_EXPRESSION_IMAGE_HPP
#define FACE_EXPRESSION_IMAGE_HPP

#include <mutex>
#include <string>
#include <iostream>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/ResourceFinder.h>
#include "drawingThread.hpp"
#include "earsThread.hpp"
#include "mouthThread.hpp"
#include "eyesThread.hpp"

#define VOCAB_AUDIO_START       yarp::os::createVocab('a','s','t','a')
#define VOCAB_AUDIO_STOP        yarp::os::createVocab('a','s','t','o')
#define VOCAB_TALK_START        yarp::os::createVocab('t','s','t','a')
#define VOCAB_TALK_STOP         yarp::os::createVocab('t','s','t','o')
#define VOCAB_BLINK             yarp::os::createVocab('b','l','i','n')
#define VOCAB_RESET             yarp::os::createVocab('r','s','t')
#define VOCAB_BLACK_RESET       yarp::os::createVocab('b','l','c','k')

class FaceExpressionImageModule: public yarp::os::RFModule
{
private:
    DrawingThread*           m_thread_output = nullptr;
    MouthThread*             m_thread_mouth = nullptr;
    EyesThread*              m_thread_eyes = nullptr;
    EarsThread*              m_thread_ears = nullptr;

    std::mutex                                                 m_mutex;
    yarp::os::Port                                             m_rpcPort;
    cv::Mat                                                    m_image;

public:
    FaceExpressionImageModule();
    bool configure(yarp::os::ResourceFinder &rf);
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    bool close();
    bool interruptModule();
    double getPeriod();
    bool   updateModule();
};

#endif

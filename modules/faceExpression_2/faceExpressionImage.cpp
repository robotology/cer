#include <cmath>
#include <string>
#include "faceExpressionImage.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <yarp/os/Time.h>
#include <yarp/os/Searchable.h>
#include <yarp/math/Rand.h>


#define BLINK_STEP_NUM  10

using namespace cv;
using namespace std;
using namespace yarp::os;
using namespace yarp::math;

FaceExpressionImageModule::FaceExpressionImageModule()
{
    m_image.create(FACE_HEIGHT, FACE_WIDTH, CV_8UC3);
}

bool FaceExpressionImageModule::configure(ResourceFinder &rf)
{
    // Open ports
    if(!m_rpcPort  .open("/" + getName() + "/rpc") )
    {
        yError() << "Cannot open RPC port";
        return false;
    }

    if(!attach(m_rpcPort))
    {
        yError() << "RFModule cannot read from RPC port (" << m_rpcPort.getName() << ")";
    }

    if (1) m_thread_output = new DrawingThread(rf, 0.033, m_image, m_mutex);
    if (1) m_thread_eyes   = new EyesThread(rf, 0.020, m_image, m_mutex);
    if (1) m_thread_ears   = new EarsThread(rf, 0.020, m_image, m_mutex);
    if (1) m_thread_mouth  = new MouthThread(rf, 0.020, m_image, m_mutex);

    if (m_thread_output) 
        if (!m_thread_output->start())
            return false;
    if (m_thread_eyes)
        if (!m_thread_eyes->start())
            return false;
    if (m_thread_ears)
        if (!m_thread_ears->start())
            return false;
    if (m_thread_mouth)
        if (!m_thread_mouth->start())
            return false;

    yInfo() << "FaceExpressionImage module started";
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
            if (m_thread_ears) m_thread_ears->activateBars(true);
        }
        break;

        case VOCAB_AUDIO_STOP:
        {
            if (m_thread_ears) m_thread_ears->activateBars(false);
        }
        break;

        case VOCAB_TALK_START:
        {
            if (m_thread_mouth) m_thread_mouth->activateTalk(true);
        }
        break;

        case VOCAB_TALK_STOP:
        {
            if (m_thread_mouth) m_thread_mouth->activateTalk(false);
        }
        break;

        case VOCAB_BLINK:
        {
            if(m_thread_eyes) m_thread_eyes->activateBlink(true);
        }
        break;

        case VOCAB_RESET:
        {
            //th.activateDraw(true);@@@@@@@
            //th.threadRelease();
        }
        break;

        case VOCAB_BLACK_RESET:
        {
            //th.activateDraw(false);@@@@@@@
            //th.blackReset();
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
    return true;
}

bool FaceExpressionImageModule::close()
{
    m_rpcPort.close();
    return true;
}

double FaceExpressionImageModule::getPeriod()
{
    return 0.1;
}

bool FaceExpressionImageModule::updateModule()
{
    return true;
}

#include <cmath>
#include <string>
#include "earsThread.hpp"
#include "utils.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <yarp/os/Time.h>
#include <yarp/os/Searchable.h>
#include <yarp/math/Rand.h>

using namespace cv;
using namespace std;
using namespace yarp::os;
using namespace yarp::math;

EarsThread::EarsThread(ResourceFinder& _rf, double _period, cv::Mat& _image, std::mutex& _mutex) :
    PeriodicThread(_period),
    m_face(_image),
    m_mutex(_mutex),
    m_rf(_rf)
{
}

void EarsThread::afterStart(bool s) { }

void EarsThread::activateBars(bool activate)
{
    lock_guard<mutex> lg(m_mutex);
    m_doBars = activate;
}

bool EarsThread::threadInit()
{
    // Check for optional params
    earBar0_x = m_rf.check("earBar0_x", Value(1), "horizontal offset from left border of outer ear bar, in pixels from upper left corner of the image, starting from 0").asInt();
    earBar0_y = m_rf.check("earBar0_y", Value(6), "vertical offset from bottom border of outer ear bar, in pixels from upper left corner of the image, starting from 0").asInt();
    earBar0_minLen = m_rf.check("earBar0_minLen", Value(3), "minimum length  of outer ear bar").asInt();
    earBar0_maxLen = m_rf.check("earBar0_maxLen", Value(18), "maximum length  of outer ear bar").asInt();
    earBar0_len = m_rf.check("earBar0_len", Value(10), "starting length of outer ear bar").asInt();

    earBar1_x = m_rf.check("earBar1_x", Value(3), "horizontal offset from left border of inner ear bar, in pixels from upper left corner of the image, starting from 0").asInt();
    earBar1_y = m_rf.check("earBar1_y", Value(6), "vertical offset from bottom border of inner ear bar, in pixels from upper left corner of the image, starting from 0").asInt();
    earBar1_minLen = m_rf.check("earBar1_minLen", Value(4), "minimum length  of inner ear bar").asInt();
    earBar1_maxLen = m_rf.check("earBar1_maxLen", Value(19), "maximum length  of inner ear bar").asInt();
    earBar1_len = m_rf.check("earBar1_len", Value(11), "starting length of inner ear bar").asInt();

    earBarR0_x = FACE_WIDTH - 1 - earBar0_x;
    earBarR1_x = FACE_WIDTH - 1 - earBar1_x;


    m_audioRecPort.open("/ears:i");

    // Get color from image
    barColor = Scalar(0,255,0);

    // Create a Mat for maximum bar size
    earBar.create(earBar1_maxLen, barWidth, CV_8UC3);
    earBar.setTo(barColor);

    blackBar.create(32, 32, CV_8UC3); //@@@@@@@
    blackBar.setTo(Scalar(0, 0, 0));

    return true;
}

void EarsThread::run()
{
    float percentage = 0.5;

    yarp::dev::audioRecorderStatus* Rstatus = m_audioRecPort.read(false);
    if (Rstatus)
    {
        m_audioIsRecording=Rstatus->enabled;
    }

    if(1/*m_doBars && m_audioIsRecording()*/)
    {
        percentage = (float)rand() / (float)RAND_MAX;
    }
    else
    {
        percentage = 1.0;
    }
    updateBars(percentage);
}

bool EarsThread::updateBars(float percentage)
{
    lock_guard<mutex> lg(m_mutex);

    earBar0_len = earBar0_minLen + (earBar0_maxLen - earBar0_minLen) *  percentage;
    earBar1_len = earBar1_minLen + (earBar1_maxLen - earBar1_minLen) *  percentage;

    // Reset bars to black
    blackBar(Rect(0, 0, barWidth, 32)).copyTo  (m_face(cv::Rect(earBar0_x,  0, barWidth, 32)));
    blackBar(Rect(0, 0, barWidth, 32)).copyTo  (m_face(cv::Rect(earBar1_x,  0, barWidth, 32)));
    blackBar(Rect(0, 0, barWidth, 32)).copyTo  (m_face(cv::Rect(earBarR0_x, 0, barWidth, 32)));
    blackBar(Rect(0, 0, barWidth, 32)).copyTo  (m_face(cv::Rect(earBarR1_x, 0, barWidth, 32)));

    // Left side
    earBar(Rect(0, 0, barWidth, earBar0_len)).copyTo  (m_face(cv::Rect(earBar0_x, FACE_HEIGHT - earBar0_y - earBar0_len, barWidth, earBar0_len)));
    earBar(Rect(0, 0, barWidth, earBar1_len)).copyTo  (m_face(cv::Rect(earBar1_x, FACE_HEIGHT - earBar1_y - earBar1_len, barWidth, earBar1_len)));

    // Right side
    earBar(Rect(0, 0, barWidth, earBar0_len)).copyTo  (m_face(cv::Rect(earBarR0_x, FACE_HEIGHT - earBar0_y - earBar0_len, barWidth, earBar0_len)));
    earBar(Rect(0, 0, barWidth, earBar1_len)).copyTo  (m_face(cv::Rect(earBarR1_x, FACE_HEIGHT - earBar1_y - earBar1_len, barWidth, earBar1_len)));

    return true;
}

void EarsThread::threadRelease()
{
    m_audioRecPort.close();
}

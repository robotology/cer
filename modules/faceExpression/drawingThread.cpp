#include <cmath>
#include <string>
#include "drawingThread.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

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

DrawingThread::DrawingThread(ResourceFinder& _rf, double _period, cv::Mat& _image, std::mutex& _mutex):
               PeriodicThread(_period),
               m_image(_image),
               m_mutex(_mutex),
               m_rf(_rf)
{
}

void DrawingThread::afterStart(bool s) { }

bool DrawingThread::threadInit()
{
    m_imageOutPort.open("/face:o");
    return true;
}

void DrawingThread::run()
{
    lock_guard<mutex> lg(m_mutex);

    yarp::sig::ImageOf<yarp::sig::PixelRgb> &img = m_imageOutPort.prepare();
    img.setExternal(m_image.data, FACE_WIDTH, FACE_HEIGHT);
    m_imageOutPort.writeStrict();
}

void DrawingThread::threadRelease()
{
    m_imageOutPort.close();
/*    lock_guard<mutex> lg(m_mutex);
    yarp::sig::ImageOf<yarp::sig::PixelRgb> &img = m_imagePort.prepare();
    img.setExternal(faceRest.data, faceWidth, faceHeight);
    m_imagePort.writeStrict();*/
}
/*
void DrawingThread::blackReset()
{
    lock_guard<mutex> lg(m_mutex);
    yarp::sig::ImageOf<yarp::sig::PixelRgb> &img = m_imagePort.prepare();
    img.setExternal(faceBlack.data, faceWidth, faceHeight);
    m_imagePort.writeStrict();
}
*/
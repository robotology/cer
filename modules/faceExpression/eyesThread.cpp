#include <cmath>
#include <string>
#include "utils.hpp"
#include "eyesThread.hpp"

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

EyesThread::EyesThread(ResourceFinder& _rf, double _period, cv::Mat& _image, std::mutex& _mutex) :
    PeriodicThread(_period),
    m_face(_image),
    m_mutex(_mutex),
    m_rf(_rf)
{
    //indexes
    indexes[0] = 0;
    indexes[1] = 1;
    indexes[2] = 2;
    indexes[3] = 3;
    indexes[4] = 4;
    indexes[5] = 5;
    indexes[6] = 4;
    indexes[7] = 3;
    indexes[8] = 2;
    indexes[9] = 1;
    indexes[10] = 0;

    //delays
    delays[0] = 0.045;
    delays[1] = 0.045;
    delays[2] = 0.045;
    delays[3] = 0.015;
    delays[4] = 0.015;
    delays[5] = 0.015;
    delays[6] = 0.065;
    delays[7] = 0.065;
    delays[8] = 0.065;
    delays[9] = 0.065;
    delays[10] = 0.065;


/*
    // Load image for bars
    th.blinkBar = imread(std::string(m_imagePath + "/happyBar.bmp").c_str(), cv::IMREAD_COLOR);

    if (th.blinkBar.empty())
    {
        yError() << "Image required for hear bars was not found.";
        ok = false;
    }
*/

}

void EyesThread::afterStart(bool s) { }

void EyesThread::activateBlink(bool activate)
{
    lock_guard<mutex> lg(m_mutex);
    m_doBlink = activate;
}

bool EyesThread::threadInit()
{
    if (!getPath(m_rf, m_imagePath)) return false;

    //load eyes
    bool ok = true;
    blinkEye.push_back(cv::imread(std::string(m_imagePath + "/blink_1.bmp").c_str(), cv::IMREAD_COLOR));
    blinkEye.push_back(cv::imread(std::string(m_imagePath + "/blink_2.bmp").c_str(), cv::IMREAD_COLOR));
    blinkEye.push_back(cv::imread(std::string(m_imagePath + "/blink_3.bmp").c_str(), cv::IMREAD_COLOR));
    blinkEye.push_back(cv::imread(std::string(m_imagePath + "/blink_4.bmp").c_str(), cv::IMREAD_COLOR));
    blinkEye.push_back(cv::imread(std::string(m_imagePath + "/blink_5.bmp").c_str(), cv::IMREAD_COLOR));
    blinkEye.push_back(cv::imread(std::string(m_imagePath + "/blink_6.bmp").c_str(), cv::IMREAD_COLOR));
    for (int i = 0; i < blinkEye.size(); i++)
    {
        if (blinkEye[i].empty())
        {
            yError() << "Image number " << i << "required for blink sequence was not found.";
            ok = false;
        }
    }
    // Load nose
    noseBar = cv::imread(std::string(m_imagePath + "/noseBar.bmp").c_str(), cv::IMREAD_COLOR);
    noseBar0_len = noseBar.cols;

    if (noseBar.empty())
    {
        yError() << "Image required for nose was not found.";
        ok = false;
    }

    // Quit if something wrong!!
    if (!ok)
    {
        yError() << "I am searching in path: " << m_imagePath << ", use option --path to change the path";
        return false;
    }


    // Reset image to black
    faceRest.create(FACE_HEIGHT, FACE_WIDTH, CV_8UC3);
    faceRest.setTo(cv::Scalar(0,0,0));

    // Copy eyes
    ((Mat)blinkEye[indexes[0]]).copyTo(faceRest(cv::Rect(leftEye_x,  leftEye_y,  eyeWidth, eyeHeight)));
    ((Mat)blinkEye[indexes[0]]).copyTo(faceRest(cv::Rect(rightEye_x, rightEye_y, eyeWidth, eyeHeight)));


    // Get color from image
    //barColor = Scalar(blinkBar.at<cv::Vec3b>(0, 0));

    // Create a Mat for maximum bar size @@@@@@
    //blinkBar.resize(hearBar1_maxLen, barWidth);
    //blinkBar.setTo(barColor);

    // Add nose
    noseBar.copyTo(faceRest(cv::Rect(noseBar0_x, noseBar0_y, noseBar.cols, noseBar.rows)));

    faceRest.copyTo(m_face);

    return true;
}

void EyesThread::run()
{
    bool  doubleBlink = false;
    float blink_per_minute = 20;

    float period_percent = ((float)rand() / (float)RAND_MAX);
    float period_sec = 60 / (blink_per_minute + 6 * period_percent) / 0.1;

    static int shall_blink = 0;
    float r;

    shall_blink++;

    if (shall_blink >= period_sec)
    {
        m_doBlink = true;
        r = ((float)rand() / (float)RAND_MAX) * 100;
        r < 30 ? doubleBlink = true : doubleBlink = false;
        shall_blink = 0;
    }

    if (doubleBlink)
    {
        m_doBlink = true;
    }

    do
    {
        if(m_doBlink)
        {
            index++;
            updateBlink(index);

            if(index >= BLINK_STEP_NUM)
            {
                index = 0;
                m_doBlink = false;
            }
        }

        if(m_doBlink)
        {
            yarp::os::Time::delay(delays[index]);
        }
    }
    while(m_doBlink);
}

bool EyesThread::updateBlink(int index)
{
    lock_guard<mutex> lg(m_mutex);
    // Copy eyes
    ((Mat)blinkEye[indexes[index]]).copyTo(m_face(cv::Rect(leftEye_x,  leftEye_y,  eyeWidth, eyeHeight)));
    ((Mat)blinkEye[indexes[index]]).copyTo(m_face(cv::Rect(rightEye_x, rightEye_y, eyeWidth, eyeHeight)));

    // Add nose
    noseBar.copyTo(m_face(cv::Rect(noseBar0_x, noseBar0_y, noseBar.cols, noseBar.rows)));

    return true;
}

void EyesThread::threadRelease()
{
}

/*
* Copyright (C)2015  iCub Facility - Istituto Italiano di Tecnologia
* Author: Marco Randazzo
* email:  marco.randazzo@iit.it
* website: www.robotcub.org
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* http://www.robotcub.org/icub/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

#include <sstream>
#include <iomanip>

#include "controlThread.h"


void ControlThread::run()
{
    iRGBD->getDepthImage(m_depth_image);
    if (m_depth_image.getRawImage() == 0)
    {
        yDebug() << "invalid image received";
        return;
    }

    if (m_depth_image.width() != m_depth_width ||
        m_depth_image.height() != m_depth_height)
    {
        yDebug() << "invalid image size";
        return;
    }
    float *center_value_p = (float*)(m_depth_image.getPixelAddress(m_depth_width/2, m_depth_height/2));
    float center_value = *center_value_p;

    if (m_gain_image.width() == 0)
    {
        m_gain_image.copy(m_depth_image);
    }
    for (int iy = 0; iy < m_depth_image.height(); iy++)
    {
        for (int ix = 0; ix < m_depth_image.width(); ix++)
        {
            float* pixel_depth_p = (float*)m_depth_image.getPixelAddress(ix, iy);
            float  pixel_depth = *pixel_depth_p;
            
            float  error_gain = 1.0;
            if (m_real_value!=-1)
            {error_gain = pixel_depth / m_real_value;}
            else
            {error_gain = pixel_depth / center_value;}
            
            //yDebug() << error;
            yarp::sig::PixelRgb& rgb_p = m_gain_image.safePixel(ix, iy);
            const double MAX_VAL = 1.5;
            double m = 255/(1-MAX_VAL);
            double q = -MAX_VAL * m;
               
            if (error_gain > 1.0)
            {
                if (error_gain>MAX_VAL) error_gain = MAX_VAL;

                rgb_p.r = (char)(255 * 1);
                rgb_p.g = (char)(m*error_gain+q);
                rgb_p.b = (char)(m*error_gain+q);
            }
            else
            {
                error_gain=1/error_gain;
                if (error_gain>MAX_VAL) error_gain = MAX_VAL;                
                rgb_p.r = (char)(m*error_gain+q);
                rgb_p.g = (char)(m*error_gain+q);
                rgb_p.b = (char)(255 * 1);
            }
            if (pixel_depth == 0.0)
            {
                rgb_p.r = 0;
                rgb_p.g = 0;
                rgb_p.b = 0;
            }
         
            if (error_gain < 0)
            {
                rgb_p.r = 0;
                rgb_p.g = 0;
                rgb_p.b = 0;
            }
        }
    }
    m_test_port.write(m_gain_image);
}

void ControlThread::printStats()
{

}

bool ControlThread::threadInit()
{
    Time::turboBoost();

    Property prop;
    if (!m_rf.check("RGBD_SENSOR_CLIENT"))
    {
        yError() << "missing RGBD_SENSOR_CLIENT section in configuration file!";
        return false;
    }
    prop.fromString(m_rf.findGroup("RGBD_SENSOR_CLIENT").toString());
    prop.put("device", "RGBDSensorClient");

    m_driver.open(prop);
    if (!m_driver.isValid())
    {
        yError("Error opening PolyDriver check parameters");
        return false;
    }
    m_driver.view(iRGBD);
    if (!iRGBD)
    {
        yError("Error opening iRGBD interface. Device not available");
        return false;
    }

    m_real_value = -1;
    m_real_value = (float)(m_rf.find("real_value").asDouble());
    m_depth_width = iRGBD->getDepthWidth();
    m_depth_height = iRGBD->getDepthHeight();

    m_test_port.open("/rgbCalibrationTest/test:o");

    return true;
}

void ControlThread::afterStart(bool s)
{
    if (s)
        yInfo("Control thread started successfully");
    else
        yError("Control thread did not start");
}

ControlThread::ControlThread(unsigned int _period, ResourceFinder &_rf) :
RateThread(_period), m_rf(_rf)
{
   
}

void ControlThread::threadRelease()
{
    m_test_port.close();
}

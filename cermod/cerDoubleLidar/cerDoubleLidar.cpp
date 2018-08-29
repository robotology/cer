/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <cerDoubleLidar.h>

#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/ResourceFinder.h>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <limits>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>

#ifndef DEG2RAD
#define DEG2RAD M_PI/180.0
#endif


using namespace std;
using namespace cer::dev;

//-------------------------------------------------------------------------------------

bool cerDoubleLidar::open(yarp::os::Searchable& config)
{
    yInfo() << "Scan started successfully";

    yInfo() << "Device info:"   << m_info;
    yInfo("max_angle %f, min_angle %f", m_max_angle, m_min_angle);
    yInfo("resolution %f",              m_resolution);
    yInfo("sensors %d",                 m_sensorsNum);
    PeriodicThread::start();
    return true;
}

bool cerDoubleLidar::close()
{
    PeriodicThread::stop();
    yInfo() << "rpLidar closed";
    return true;
}

bool cerDoubleLidar::getDistanceRange(double& min, double& max)
{
    LockGuard guard(m_mutex);
    min = m_min_distance;
    max = m_max_distance;
    return true;
}

bool cerDoubleLidar::setDistanceRange(double min, double max)
{
    LockGuard guard(m_mutex);
    m_min_distance = min;
    m_max_distance = max;
    return true;
}

bool cerDoubleLidar::getScanLimits(double& min, double& max)
{
    LockGuard guard(m_mutex);
    min = m_min_angle;
    max = m_max_angle;
    return true;
}

bool cerDoubleLidar::setScanLimits(double min, double max)
{
    LockGuard guard(m_mutex);
    yWarning("setScanLimits not yet implemented");
    return true;
}

bool cerDoubleLidar::getHorizontalResolution(double& step)
{
    LockGuard guard(m_mutex);
    step = m_resolution;
    return true;
}

bool cerDoubleLidar::setHorizontalResolution(double step)
{
    LockGuard guard(m_mutex);
    m_resolution = step;
    return true;
}

bool cerDoubleLidar::getScanRate(double& rate)
{
    LockGuard guard(m_mutex);
    yWarning("getScanRate not yet implemented");
    return true;
}

bool cerDoubleLidar::setScanRate(double rate)
{
    LockGuard guard(m_mutex);
    yWarning("setScanRate not yet implemented");
    return false;
}


bool cerDoubleLidar::getRawData(yarp::sig::Vector &out)
{
    LockGuard guard(m_mutex);
    out           = m_laser_data;
    m_device_status = yarp::dev::IRangefinder2D::DEVICE_OK_IN_USE;
    return true;
}

bool cerDoubleLidar::getLaserMeasurement(std::vector<LaserMeasurementData> &data)
{
    LockGuard guard(m_mutex);
    size_t size = m_laser_data.size();
    data.resize(size);

    if (m_max_angle < m_min_angle) { yError() << "getLaserMeasurement failed"; return false; }

    double laser_angle_of_view = m_max_angle - m_min_angle;

    for (size_t i = 0; i < size; i++)
    {
        double angle = (i / double(size)*laser_angle_of_view + m_min_angle)* DEG2RAD;
        data[i].set_polar(m_laser_data[i], angle);
    }
    m_device_status = yarp::dev::IRangefinder2D::DEVICE_OK_IN_USE;

    return true;
}
bool cerDoubleLidar::getDeviceStatus(Device_status &status)
{
    LockGuard guard(m_mutex);
    status = m_device_status;
    return true;
}

bool cerDoubleLidar::threadInit()
{

    return true;
}

//#define DEBUG_LOCKING 1
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof (_Array[0]))
#endif

void cerDoubleLidar::run()
{
    return;
}

void cerDoubleLidar::threadRelease()
{
#ifdef LASER_DEBUG
    yDebug("RpLidar Thread releasing...");
    yDebug("... done.");
#endif

    return;
}

bool cerDoubleLidar::getDeviceInfo(std::string &device_info)
{
    LockGuard guard(m_mutex);
    device_info = std::string("Laser:\n") + m_info;
    return true;
}

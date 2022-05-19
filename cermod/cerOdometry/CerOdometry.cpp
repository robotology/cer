/*
* Copyright (C)2021 Istituto Italiano di Tecnologia
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

#include "CerOdometry.h"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/math/Rand.h>
#include <iostream>

using namespace cer::dev;

namespace {
YARP_LOG_COMPONENT(CERODOM, "cer.devices.CerOdometry")
}

CerOdometry::CerOdometry():
        yarp::os::PeriodicThread(m_period)
{
    encvel_estimator =new iCub::ctrl::AWLinEstimator(2,1.0);
    encw_estimator = new iCub::ctrl::AWLinEstimator(1, 1.0);
    enc.resize(2);
    encv.resize(2);
    yCTrace(CERODOM);
    yCInfo(CERODOM);
}

CerOdometry::~CerOdometry()
{
    yCTrace(CERODOM);
    close();
}

bool CerOdometry::close()
{
    detach();
    return true;
}

bool CerOdometry::getOdometry(yarp::dev::OdometryData& odom)
{
    std::lock_guard lock(m_odometry_mutex);
    odom.odom_x = m_odometryData.odom_x;
    odom.odom_y = m_odometryData.odom_y;
    odom.odom_theta  = m_odometryData.odom_theta;
    odom.base_vel_x = m_odometryData.base_vel_x;
    odom.base_vel_y = m_odometryData.base_vel_y;
    odom.base_vel_theta = m_odometryData.base_vel_theta;
    odom.odom_vel_x  = m_odometryData.odom_vel_x;
    odom.odom_vel_y = m_odometryData.odom_vel_y;
    odom.odom_vel_theta = m_odometryData.odom_vel_theta;
    return true;
}

bool CerOdometry::resetOdometry()
{
    if (ienc) {
        ienc->getEncoder(0, &encL_offset);
        ienc->getEncoder(1, &encR_offset);
    }
    m_odometryData.odom_x=0;
    m_odometryData.odom_y=0;
    if (encvel_estimator) {
        encvel_estimator->reset();
    }
    yCInfo(CERODOM,"Odometry reset done");
    return true;
}

bool CerOdometry::open(yarp::os::Searchable& config)
{
    yarp::os::Network yarp;

    if (!yarp.checkNetwork())
    {
        yCError(CERODOM) << "yarp network not found";
        std::cerr << "Sorry YARP network does not seem to be available, is the yarp server available?\n";
        return -1;
    }

    if (!config.check("period"))
    {
        yCWarning(CERODOM, "using default period of 0.01");
    } else {
        m_period = config.find("period").asFloat64();
    }

    //reset odometry
    resetOdometry();
    //get robot geometry

    if (!config.check("geom_r"))
    {
        yCError(CERODOM, "Missing param geom_r");
        return false;
    }
    geom_r = config.find("geom_r").asFloat64();

    if (!config.check("geom_l"))
    {
        yCError(CERODOM, "Missing param geom_l");
        return false;
    }
    geom_L = config.find("geom_l").asFloat64();

    yarp::os::PeriodicThread::setPeriod(m_period);
    return true;
}

void CerOdometry::compute()
{
    std::lock_guard lock(m_odometry_mutex);
    if (ienc) {
        //read the encoders (deg)
        ienc->getEncoder(0, &encL);
        ienc->getEncoder(1, &encR);

        //read the speeds (deg/s)
        ienc->getEncoderSpeed(0, &velL);
        ienc->getEncoderSpeed(1, &velR);

        //remove the offset and convert in radians
        enc[0] = (encL - encL_offset) * 0.0174532925;
        enc[1] = (encR - encR_offset) * 0.0174532925;

        //estimate the speeds
        iCub::ctrl::AWPolyElement el;
        el.data = enc;
        el.time = yarp::os::Time::now();
        encv = encvel_estimator->estimate(el);

        //compute the orientation.
        m_odometryData.odom_theta = (geom_r / geom_L) * (-enc[0] + enc[1]);

        iCub::ctrl::AWPolyElement el2;
        el2.data = yarp::sig::Vector(1, m_odometryData.odom_theta);
        el2.time = yarp::os::Time::now();
        yarp::sig::Vector vvv;
        vvv.resize(1, 0.0);
        vvv = encw_estimator->estimate(el2);

        //build the kinematics matrix
        /*yarp::sig::Matrix kin;
        kin.resize(3,2);
        kin.zero();
        kin(0, 0) = cos(odom_theta) / 2;
        kin(0, 1) = cos(odom_theta) / 2;
        kin(0, 2) = sin(odom_theta) / 2;
        kin(1, 0) = sin(odom_theta) / 2;
        kin(1, 1) = 1 / (2 * geom_L);
        kin(1, 2) = 1 / (2 * geom_L);

        yarp::sig::Vector odom_cart_vels;  //velocities expressed in the world reference frame
        yarp::sig::Vector base_cart_vels; //velocities expressed in the base reference frame
        odom_cart_vels  = kin*encv;
        base_cart_vels = kin*encv; //@@@

        base_vel_x     = base_cart_vels[1];
        base_vel_y     = base_cart_vels[0];
        base_vel_lin   = sqrt(base_vel_x*base_vel_x + base_vel_y*base_vel_y);
        base_vel_theta = base_cart_vels[2];

        odom_vel_x      = odom_cart_vels[1];
        odom_vel_y      = odom_cart_vels[0];
        odom_vel_theta  = odom_cart_vels[2];
        */


        m_odometryData.base_vel_x = geom_r / 2 * encv[0] + geom_r / 2 * encv[1];
        m_odometryData.base_vel_y = 0;
        base_vel_lin = fabs(m_odometryData.base_vel_x);
        m_odometryData.base_vel_theta = vvv[0];///-(geom_r / geom_L) * encv[0] + (geom_r / geom_L) * encv[1];


        m_odometryData.odom_vel_x = m_odometryData.base_vel_x * cos(m_odometryData.odom_theta);
        m_odometryData.odom_vel_y = m_odometryData.base_vel_x * sin(m_odometryData.odom_theta);
        m_odometryData.odom_vel_theta = m_odometryData.base_vel_theta;

        //the integration step
        double period = el.time - last_time;
        m_odometryData.odom_x = m_odometryData.odom_x + (m_odometryData.odom_vel_x * period);
        m_odometryData.odom_y = m_odometryData.odom_y + (m_odometryData.odom_vel_y * period);

        //compute traveled distance (odometer)
        traveled_distance = traveled_distance + fabs(base_vel_lin * period);
        traveled_angle = traveled_angle + fabs(m_odometryData.base_vel_theta * period);

        //convert from radians back to degrees
        m_odometryData.odom_theta *= RAD2DEG;
        m_odometryData.base_vel_theta *= RAD2DEG;
        m_odometryData.odom_vel_theta *= RAD2DEG;
        traveled_angle *= RAD2DEG;

        last_time = yarp::os::Time::now();
    } else {
        yCError(CERODOM) << "iencoder interface not valid";
    }
}

bool CerOdometry::attach(yarp::dev::PolyDriver *driver) {
    if (!driver->isValid())
    {
        yCError(CERODOM) << "not valid poly driver";
        return false;
    }
    if(!driver->view(ienc)){
        yCError(CERODOM) << "iencoder device has not been viewed";
        return false;
    }
    int axesNumber;
    ienc->getAxes(&axesNumber);
    if(axesNumber != 2){
        yCError(CERODOM) << "failed to get correct number of axes";
        return false;
    }

    bool b = yarp::os::PeriodicThread::start();
    return b;
}

bool CerOdometry::detach() {
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }
    ienc = nullptr;
    return true;
}

bool CerOdometry::threadInit() {
    return true;
}

void CerOdometry::threadRelease() {
}

void CerOdometry::run() {
    compute();
}


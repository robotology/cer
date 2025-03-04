/*
 * Copyright (C) 2013-2020 Fondazione Istituto Italiano di Tecnologia
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <cstdio>
#include <string>
#include "cerDoubleLidar.h"

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <math.h>
#include <limits>
#include <cmath>

#ifndef DEG2RAD
#define DEG2RAD M_PI/180.0
#endif

#ifndef RAD2DEG
#define RAD2DEG 180/M_PI
#endif

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace cer::dev;

YARP_LOG_COMPONENT(CER_DOUBLE_LIDAR, "cer.devices.cerDoubleLidar")

bool cerDoubleLidar::LaserCfg_t::loadConfig(yarp::os::Searchable& config)
{
    std::string key;
    if(laser_id ==front)
    {
        key="LASERFRONT-CFG";
    }
    else
    {
        key="LASERBACK-CFG";
    }

    yarp::os::Searchable& l_config = config.findGroup(key);
    if (l_config.check("pose")==false) {yCError(CER_DOUBLE_LIDAR) << "missing pose"; return false; }
    Bottle & b_pose = l_config.findGroup("pose");
    if(b_pose.size()!= 5)
    {
        yCError(CER_DOUBLE_LIDAR) << "wrong size of pose ("<<b_pose.size()<<"). It should be <x y x theta>";
        return false;
    }
    pose.x = b_pose.get(1).asFloat64();
    pose.y = b_pose.get(2).asFloat64();
    pose.z = b_pose.get(3).asFloat64();
    pose.theta = b_pose.get(4).asFloat64();


    if (l_config.check("sensorName")==false) {yCError(CER_DOUBLE_LIDAR) << "missing sensorName"; return false; }
    Bottle & b_sensorName = l_config.findGroup("sensorName");
    sensorName = b_sensorName.get(1).asString();


    //DEBUG
    yDebug() << key << "x:" << pose.x << "y:" << pose.y << "z:" << pose.z << "t:" << pose.theta << sensorName;

    return true;

}


cerDoubleLidar::cerDoubleLidar():
    m_inited(false),
    m_lFrontCfg(LaserCfg_t::LaserId::front),
    m_lBackCfg(LaserCfg_t::LaserId::back)
{;}

cerDoubleLidar::~cerDoubleLidar() {}

bool cerDoubleLidar::getLasersInterfaces(void)
{
    if(m_driver_laserFront == nullptr)
    {
        yCError(CER_DOUBLE_LIDAR) << "cannot find laserFront device";
        return false;
    }
    else
    {
        yCInfo(CER_DOUBLE_LIDAR) << "laserFront device found. OK";
    }

    if(m_driver_laserBack == nullptr)
    {
        yCError(CER_DOUBLE_LIDAR) << "cannot find laserBack device";
        return false;
    }
    else
    {
        yCInfo(CER_DOUBLE_LIDAR) << "laserBack device found. OK";
    }

    if(!m_driver_laserFront->view(m_ILaserFrontData))
    {
        yCError(CER_DOUBLE_LIDAR) << "cannot get interface of laser front";
        return false;
    }
    else
    {
        yCInfo(CER_DOUBLE_LIDAR) << "get interface of laser front. OK";
    }

    if(!m_driver_laserBack->view(m_ILaserBackData))
    {
        yCError(CER_DOUBLE_LIDAR) << "cannot get interface of laser Back";
        return false;
    }
    else
    {
        yCInfo(CER_DOUBLE_LIDAR) << "get interface of laser Back. OK";
    }

    return true;
}

bool cerDoubleLidar::attachAll(const PolyDriverList &p)
{
    if(p.size()!=2)
    {
        yCError(CER_DOUBLE_LIDAR) << "attach with wrong number of drivers";
        return false;
    }

    for(int i=0; i< p.size(); i++)
    {
        if(p[i]->key == m_lFrontCfg.sensorName)
            m_driver_laserFront = p[i]->poly;
        else if(p[i]->key == m_lBackCfg.sensorName)
            m_driver_laserBack = p[i]->poly;
        else
        {
             yCError(CER_DOUBLE_LIDAR) << "attach: the driver called" << p[i]->key << "not belong to my configuration";
             return false;
        }
    }


    if(!getLasersInterfaces())
        return false;

    if(!verifyLasersConfigurations())
        return false;

    m_inited = true;
    return true;
}

bool cerDoubleLidar::detachAll(void)
{
    m_driver_laserFront = nullptr;
    m_driver_laserBack = nullptr;
    m_ILaserFrontData = nullptr;
    m_ILaserBackData = nullptr;
    m_inited=false;
    return true;
}

bool cerDoubleLidar::open(yarp::os::Searchable& config)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    yDebug() << "open()";

     //parse all the parameters related to the linear/angular range of the sensor
     yDebug() << config.toString();
    if (this->parseConfiguration(config) == false)
    {
        yCError(CER_DOUBLE_LIDAR) << "error parsing parameters";
        return false;
    }

    if(!m_lFrontCfg.loadConfig(config))
    {
        yCError(CER_DOUBLE_LIDAR) << "m_lFrontCfg.loadConfig() failed";
        return false;
    }
    yDebug() << "x:" << m_lFrontCfg.pose.x << "y:" << m_lFrontCfg.pose.y << "z:" << m_lFrontCfg.pose.z << "t:" << m_lFrontCfg.pose.theta << m_lFrontCfg.sensorName;

    if(!m_lBackCfg.loadConfig(config))
    {
        yCError(CER_DOUBLE_LIDAR) << "m_lBackCfg.loadConfig() failed";
        return false;
    }
    yDebug() << "x:" << m_lBackCfg.pose.x << "y:" << m_lBackCfg.pose.y << "z:" << m_lBackCfg.pose.z << "t:" << m_lBackCfg.pose.theta << m_lBackCfg.sensorName;

    //currently if z values differs, than return error
    if(m_lFrontCfg.pose.z != m_lBackCfg.pose.z)
    {
        yCWarning(CER_DOUBLE_LIDAR) << "poses of laser front and back have different z values";
    }

    return true;
}

bool cerDoubleLidar::close()
{
    detachAll();
    return true;
}

bool cerDoubleLidar::verifyLasersConfigurations(void)
{
    double min_angle[2], max_angle[2];
    double min_dist[2], max_dist[2];
    double resolution[2];

    {
        if(!m_ILaserFrontData->getScanLimits(min_angle[0], max_angle[0]))
        {
            yCError(CER_DOUBLE_LIDAR) << "error in getScanLimits for front laser";
            return false;
        }

        if(!m_ILaserBackData->getScanLimits(min_angle[1], max_angle[1]))
        {
            yCError(CER_DOUBLE_LIDAR) << "error in getScanLimits for back laser";
            return false;
        }

        /*
        if( (min_angle[0] != min_angle[1]) || (max_angle[0] != max_angle[1]) )
        {
            yCError(CER_DOUBLE_LIDAR) << "front and back laser differ in getScanLimits";
            return false;
        }*/
    }

    {
        if(!m_ILaserFrontData->getDistanceRange(min_dist[0], max_dist[0]))
        {
            yCError(CER_DOUBLE_LIDAR) << "error in getDistanceRange for front laser";
            return false;
        }

        if(!m_ILaserBackData->getDistanceRange(min_dist[1], max_dist[1]))
        {
            yCError(CER_DOUBLE_LIDAR) << "error in getDistanceRange for back laser";
            return false;
        }

        /*
        if( (min_dist[0] != min_dist[1]) || (max_dist[0] != max_dist[1]) )
        {
            yCError(CER_DOUBLE_LIDAR) << "front and back laser differ in getDistanceRange";
            return false;
        }*/
   }

    if(!m_ILaserFrontData->getHorizontalResolution(resolution[0]))
    {
        yCError(CER_DOUBLE_LIDAR) << "error getting resolution for front laser";
        return false;
    }

    if(!m_ILaserBackData->getHorizontalResolution(resolution[1]))
    {
        yCError(CER_DOUBLE_LIDAR) << "error getting resolution for back laser";
        return false;
    }

    if(m_resolution != resolution[0] ||
       m_resolution != resolution[1])
    {
        yCError(CER_DOUBLE_LIDAR) << "front laser, back laser and user configuration are different!" << m_resolution << resolution[0] << resolution[1];
        return false;
    }

    return true;

}

ReturnValue cerDoubleLidar::setDistanceRange(double min, double max)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    if(!m_inited)
        return ReturnValue::return_code::return_value_error_generic;
    double curmin, curmax;
    if(!m_ILaserFrontData->getDistanceRange(curmin, curmax))
        return ReturnValue::return_code::return_value_error_generic;

    if(!m_ILaserFrontData->setDistanceRange(min, max))
        return ReturnValue::return_code::return_value_error_generic;

    if(!m_ILaserBackData->setDistanceRange(min,max))
    {
        m_ILaserFrontData->setDistanceRange(curmin, curmax);
        return ReturnValue::return_code::return_value_error_generic;
    }
    return ReturnValue_ok;
}

ReturnValue cerDoubleLidar::setScanLimits(double min, double max)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    if(!m_inited)
        return ReturnValue::return_code::return_value_error_generic;

    double curmin, curmax;
    if(!m_ILaserFrontData->getScanLimits(curmin, curmax))
        return ReturnValue::return_code::return_value_error_generic;

    if(!m_ILaserFrontData->setScanLimits(min, max))
        return ReturnValue::return_code::return_value_error_generic;

    if(!m_ILaserBackData->setScanLimits(min,max))
    {
        m_ILaserFrontData->setScanLimits(curmin, curmax);
        return ReturnValue::return_code::return_value_error_generic;
    }
    return ReturnValue_ok;
}

ReturnValue cerDoubleLidar::setHorizontalResolution(double step)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    if(!m_inited)
        return ReturnValue::return_code::return_value_error_generic;

    double curstep;
    if(!m_ILaserFrontData->getHorizontalResolution(curstep))
        return ReturnValue::return_code::return_value_error_generic;

    if(!m_ILaserFrontData->setHorizontalResolution(step))
        return ReturnValue::return_code::return_value_error_generic;

    if(!m_ILaserBackData->setHorizontalResolution(step))
    {
        m_ILaserFrontData->setHorizontalResolution(curstep);
        return ReturnValue::return_code::return_value_error_generic;
    }
    return ReturnValue_ok;
}

ReturnValue cerDoubleLidar::setScanRate(double rate)
{
   std::lock_guard<std::mutex> guard(m_mutex);
    if(!m_inited)
        return ReturnValue::return_code::return_value_error_generic;

    double currate;
    if(!m_ILaserFrontData->getScanRate(currate))
        return ReturnValue::return_code::return_value_error_generic;

    if(!m_ILaserFrontData->setScanRate(rate))
        return ReturnValue::return_code::return_value_error_generic;

    if(!m_ILaserBackData->setScanRate(rate))
    {
        m_ILaserFrontData->setScanRate(currate);
        return ReturnValue::return_code::return_value_error_generic;
    }
    return ReturnValue_ok;
}

void cerDoubleLidar::calculate(int sensNum, double distance, double x_off, double y_off, double t_off)
{
    if (distance == std::numeric_limits<double>::infinity())
    {
        //if we received an infinity, put it in the right slot
        double angle = (sensNum*m_resolution) + t_off *RAD2DEG;
        int newSensNum= (double)(angle /m_resolution);
        if (newSensNum >= (int)m_sensorsNum) newSensNum-= (int) m_sensorsNum;
        if (newSensNum < 0)    newSensNum+= (int)m_sensorsNum;

        //assignment on empty slots
        if (std::isnan(m_laser_data[newSensNum]))
        { m_laser_data[newSensNum] = std::numeric_limits<double>::infinity();}
    }
    else if (std::isnan(distance))
    {
        //if we received an NaN, just skip it
    }
    else
    {
        //if we received a valid value, process it and put it in the right slot
        double angle_input = (sensNum*m_resolution);
        double angle_rad = angle_input*DEG2RAD;

    #ifdef DO_NOTHING_DEBUG
        x_off=0;
        y_off=0;
        t_off=0;
    #endif

        //calculate vertical and horizontal components of input angle
        double Ay = (sin(angle_rad+t_off)*distance);
        double Ax = (cos(angle_rad+t_off)*distance);

        //calculate vertical and horizontal components of new angle with offset.
        double By = Ay + y_off;
        double Bx = Ax + x_off;

        double betarad = atan2(By,Bx); //the output is -pi +pi
        double beta2 = betarad*RAD2DEG; //the output is -180 +180

        //compute the new slot
        int newSensNum= (double)(beta2/m_resolution);
        if (newSensNum >= (int)m_sensorsNum) newSensNum-= (int)m_sensorsNum;
        if (newSensNum < 0)   newSensNum+=(int) m_sensorsNum;

        //compute the distance
        double newdistance = std::sqrt((Bx*Bx)+(By*By));

        //assignment
        m_laser_data[newSensNum] = newdistance;
    }

}

ReturnValue cerDoubleLidar::getRawData(yarp::sig::Vector &out, double* timestamp)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if(!m_inited)
    {
        return ReturnValue::return_code::return_value_error_generic;
    }
    yarp::sig::Vector dataFront;
    yarp::sig::Vector dataBack;
    double            timestampFront=0;
    double            timestampBack=0;

    for(size_t i=0; i<m_sensorsNum; i++)
    {
         m_laser_data[i] = std::nan("");
    }

    if(!m_ILaserFrontData->getRawData(dataFront, &timestampFront))
    {
        out = m_laser_data;
        m_device_status = yarp::dev::IRangefinder2D::Device_status::DEVICE_GENERAL_ERROR;
        return ReturnValue::return_code::return_value_error_generic;
    }

    if(!m_ILaserBackData->getRawData(dataBack, &timestampBack))
    {
        out = m_laser_data;
        m_device_status = yarp::dev::IRangefinder2D::Device_status::DEVICE_GENERAL_ERROR;
        return ReturnValue::return_code::return_value_error_generic;
    }
    m_device_status = yarp::dev::IRangefinder2D::Device_status::DEVICE_OK_IN_USE;

    for(size_t i=0; i<m_sensorsNum; i++)
    {
     //   yDebug() << i << m_sensorsNum;
        //When hardware has problems, it gives me 0.0, so I skip it
        if(dataFront[i]!=0.0)
        {
            calculate(i, dataFront[i], m_lFrontCfg.pose.x, m_lFrontCfg.pose.y, m_lFrontCfg.pose.theta);
        }
        if(dataBack[i]!=0.0)
        {
            calculate(i, dataBack[i], m_lBackCfg.pose.x, m_lBackCfg.pose.y, m_lBackCfg.pose.theta);
        }
    }

    //For now, I am using the front lidar timestamp as the timestamp for the doubleLidar.
    //I think that it does not worth to compute the mean value. It can be eventually improved in the future.
    m_timestamp.update(timestampFront);
    if (timestamp!=nullptr)
    {
        *timestamp = m_timestamp.getTime();
    }

    // applyLimitsOnLaserData();
    out = m_laser_data;
    return ReturnValue_ok;
}

ReturnValue cerDoubleLidar::getLaserMeasurement(std::vector<LaserMeasurementData> &data, double* timestamp)
{
    yCError(CER_DOUBLE_LIDAR) << "getLaserMeasurement not yet implemented";
    if (timestamp != nullptr)
    {
        *timestamp = m_timestamp.getTime();
    }
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

bool cerDoubleLidar::acquireDataFromHW()
{
    //this is empty because no thread is needed, we just take data in getRawData()
    return true;
}

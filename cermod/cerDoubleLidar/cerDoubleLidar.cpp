/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include <cstdio>
#include <string>
#include "cerDoubleLidar.h"

#include <yarp/os/LogStream.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/ResourceFinder.h>
#include <math.h>
#include <cmath>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace cer::dev;

bool LaserCfg_t::loadConfig(yarp::os::Searchable& config)
{
    std::string key;
    if(laser==front)
    {
		key="LASERFRONT-CFG";
    }
    else
    {
		key="LASERBACK-CFG";
    }
    
    yarp::os::Searchable& l_config = config.findGroup(key);
    if (l_config.check("pose")==false) {yError() << "cerDoubleLidar: missing pose"; return false; }
    Bottle & b_pose = l_config.findGroup("pose");
    if(b_pose.size()!= 5)
    {
		    yError() << "cerDoubleLidar: wrong size of pose ("<<b_pose.size()<<"). It should be <x y x theta>";
		    return false; 
    }
    pose.x = b_pose.get(1).asFloat64();
    pose.y = b_pose.get(2).asFloat64();
    pose.z = b_pose.get(3).asFloat64();
    pose.theta = b_pose.get(4).asFloat64();


    if (l_config.check("file")==false) {yError() << "cerDoubleLidar: missing file"; return false; }
    Bottle & b_filename = l_config.findGroup("file");
    fileCfgName = b_filename.get(1).asString();

    if (l_config.check("sensorName")==false) {yError() << "cerDoubleLidar: missing sensorName"; return false; }
    Bottle & b_sensorName = l_config.findGroup("sensorName");
    sensorName = b_sensorName.get(1).asString();


    //DEBUG
    yDebug() << key << "x:" << pose.x << "y:" << pose.y << "z:" << pose.z << "t:" << pose.theta << fileCfgName << sensorName;

    return true;

}


cerDoubleLidar::cerDoubleLidar():
    m_driver_laserFront(nullptr),
    m_dev_laserFront(nullptr),
    m_driver_laserBack(nullptr),
    m_dev_laserBack(nullptr),
    m_samples(0),
    m_resolution(1),
    m_inited(false),
    m_onSimulator(true),
    m_lFrontCfg(LaserCfg_t::Laser::front),
    m_lBackCfg(LaserCfg_t::Laser::back)
{;}

cerDoubleLidar::~cerDoubleLidar() {}

bool cerDoubleLidar::getLasersInterfaces(void)
{
    if(m_driver_laserFront == nullptr)
    {
        yError() << "cerDoubleLidar: cannot find laserFront device";
        return false;
    }
    else
    {
        yInfo() << "****cerDoubleLidar: laserFront device found. OK";
    }

    if(m_driver_laserBack == nullptr)
    {
        yError() << "cerDoubleLidar: cannot find laserBack device";
        return false;
    }
    else
    {
        yInfo() << "****cerDoubleLidar: laserBack device found. OK";
    }

    if(!m_driver_laserFront->view(m_dev_laserFront))
    {
        yError() << "cerDoubleLidar: cannot get interface of laser front";
        return false;
    }
    else
    {
        yInfo() << "****cerDoubleLidar: get interface of laser front. OK";
    }



    if(!m_driver_laserBack->view(m_dev_laserBack))
    {
        yError() << "cerDoubleLidar: cannot get interface of laser Back";
        return false;
    }
    else
    {
        yInfo() << "****cerDoubleLidar: get interface of laser Back. OK";
    }

    return true;
}

bool cerDoubleLidar::attachAll(const PolyDriverList &p)
{
    if(p.size()!=2)
    {
        yError() << "cerDoubleLidar attach with wrong num of drivers";
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
             yError() << "cerDoubleLidar::attach: the driver called" << p[i]->key << "not belong to my configuration";
             return false;
        }
    }


   if(!getLasersInterfaces())
        return false;

    if(!verifyLasersConfigurations())
        return false;

    return true;
}

bool cerDoubleLidar::detachAll(void)
{
    m_inited=false;
}


bool cerDoubleLidar::createLasersDevices(void)
{
    ResourceFinder rf;

    Property laserF_prop;
    if(!laserF_prop.fromConfigFile(rf.findFileByName(m_lFrontCfg.fileCfgName)))
    {
        yError() << "cerDoubleLidar: cannot load file " << m_lFrontCfg.fileCfgName;
        return false;
    }

    m_driver_laserFront = new PolyDriver(laserF_prop);
    if(m_driver_laserFront == nullptr)
    {
        yError() << "cerDoubleLidar: cannot cannot create device for laser front";
        return false;
    }


    Property laserB_prop;
    if(!laserB_prop.fromConfigFile(rf.findFileByName(m_lBackCfg.fileCfgName)))
    {
        yError() << "cerDoubleLidar: cannot load file " << m_lBackCfg.fileCfgName;
        return false;
    }

    m_driver_laserBack = new PolyDriver(laserB_prop);
    if(m_driver_laserBack == nullptr)
    {
        yError() << "cerDoubleLidar: cannot cannot create device for laser back";
        return false;
    }

    if(!m_driver_laserFront->open(laserF_prop))
    {
        yError() << "cerDoubleLidar: cannot open laser Front";
        return false;
    }
    yError() << "cerDoubleLidar: cannot opened laser Front! OK";

    if(!m_driver_laserBack->open(laserB_prop))
    {
        yError() << "cerDoubleLidar: cannot open laser Back";
        return false;
    }
    yError() << "cerDoubleLidar:  opened laser Back! OK";

    if(!getLasersInterfaces())
        return false;

    m_inited=true;
    return true;
}

bool cerDoubleLidar::open(yarp::os::Searchable& config)
{
    yarp::os::LockGuard guard(m_mutex);

    if(!m_lFrontCfg.loadConfig(config))
    {
        return false;
    }
    yDebug() << "x:" << m_lFrontCfg.pose.x << "y:" << m_lFrontCfg.pose.y << "z:" << m_lFrontCfg.pose.z << "t:" << m_lFrontCfg.pose.theta << m_lBackCfg.sensorName;

    if(!m_lBackCfg.loadConfig(config))
    {
	    return false;
    }
    yDebug() << "x:" << m_lBackCfg.pose.x << "y:" << m_lBackCfg.pose.y << "z:" << m_lBackCfg.pose.z << "t:" << m_lBackCfg.pose.theta << m_lBackCfg.sensorName;

    //currently if z values differs, than return error
    if(m_lFrontCfg.pose.z != m_lBackCfg.pose.z)
    {
        yError() << "cerDoubleLidar: poses of laser front and back have different z values";
        return false;
    }

    if (config.check("subdevice"))
        m_onSimulator=false;
    else
        m_onSimulator=true; //than I need that gazebo calls my attach function

    bool ret = true;
    //If I'm on robot I need to create the device of both lasers, while I'm onSimulator I need to wait the attach of gazebo.
    if(false==m_onSimulator)
    {
        ret = createLasersDevices();
    }
    return ret;
}

bool cerDoubleLidar::close()
{
    m_inited=false;
    //if I'm on simulator I did not create subdevice
    if(m_onSimulator)
        return true;

    if(nullptr!=m_driver_laserFront)
    {
        m_driver_laserFront->close();
        delete m_dev_laserFront;
        m_dev_laserFront=nullptr;
    }

    if(nullptr!=m_driver_laserBack)
    {
        m_driver_laserBack->close();
        delete m_driver_laserBack;
        m_driver_laserBack=nullptr;
    }

    return true;
}

bool cerDoubleLidar::verifyLasersConfigurations(void)
{
    double minFront, maxFront, minBack, maxBack;
    if(!m_dev_laserFront->getScanLimits(minFront, maxFront))
    {
        yError() << "cerDoubleLidar: error getting scan limits for front laser";
        return false;
    }

    if(!m_dev_laserBack->getScanLimits(minBack, maxBack))
    {
        yError() << "cerDoubleLidar: error getting scan limits for back laser";
        return false;
    }

    if( (minFront != minBack) || (maxFront != maxBack) )
    {
        yError() << "cerDoubleLidar: front and back laser differ in scan limits";
        return false;
    }

    double resolutionFront, resolutionBack;
    if(!m_dev_laserFront->getHorizontalResolution(resolutionFront))
    {
        yError() << "cerDoubleLidar: error getting resolution for front laser";
        return false;
    }

    if(!m_dev_laserBack->getHorizontalResolution(resolutionBack))
    {
        yError() << "cerDoubleLidar: error getting resolution for back laser";
        return false;
    }

    if(resolutionFront != resolutionBack)
    {
        yError() << "cerDoubleLidar: front and back laser differ in resolution";
        return false;
    }

    //here I'm sure front and back have same config
    m_samples = (maxFront -minFront) /resolutionFront;

    m_resolution = resolutionFront;

    m_inited = true;

    return true;

}

bool cerDoubleLidar::getDistanceRange(double& min, double& max)
{
    yarp::os::LockGuard guard(m_mutex);
    if(!m_inited)
        return false;
    return m_dev_laserFront->getDistanceRange(min, max);
}

bool cerDoubleLidar::setDistanceRange(double min, double max)
{
    yarp::os::LockGuard guard(m_mutex);
    if(!m_inited)
        return false;
    double curmin, curmax;
    if(!m_dev_laserFront->getDistanceRange(curmin, curmax))
        return false;

    if(!m_dev_laserFront->setDistanceRange(min, max))
        return false;

    if(!m_dev_laserBack->setDistanceRange(min,max))
    {
        m_dev_laserFront->setDistanceRange(curmin, curmax);
        return false;
    }
    return true;
}

bool cerDoubleLidar::getScanLimits(double& min, double& max)
{
    yarp::os::LockGuard guard(m_mutex);
    if(!m_inited)
        return false;
    return m_dev_laserFront->getScanLimits(min, max);
    return true;
}

bool cerDoubleLidar::setScanLimits(double min, double max)
{
    yarp::os::LockGuard guard(m_mutex);
    if(!m_inited)
        return false;

    double curmin, curmax;
    if(!m_dev_laserFront->getScanLimits(curmin, curmax))
        return false;

    if(!m_dev_laserFront->setScanLimits(min, max))
        return false;

    if(!m_dev_laserBack->setScanLimits(min,max))
    {
        m_dev_laserFront->setScanLimits(curmin, curmax);
        return false;
    }
    return true;
}

bool cerDoubleLidar::getHorizontalResolution(double& step)
{
    yarp::os::LockGuard guard(m_mutex);
    if(!m_inited)
        return false;
    return m_dev_laserFront->getHorizontalResolution(step);
}

bool cerDoubleLidar::setHorizontalResolution(double step)
{
    yarp::os::LockGuard guard(m_mutex);
    if(!m_inited)
        return false;

    double curstep;
    if(!m_dev_laserFront->getHorizontalResolution(curstep))
        return false;

    if(!m_dev_laserFront->setHorizontalResolution(step))
        return false;

    if(!m_dev_laserBack->setHorizontalResolution(step))
    {
        m_dev_laserFront->setHorizontalResolution(curstep);
        return false;
    }
    return true;
}

bool cerDoubleLidar::getScanRate(double& rate)
{
    yarp::os::LockGuard guard(m_mutex);
    if(!m_inited)
        return false;

    return (m_dev_laserFront->getScanRate(rate));

}

bool cerDoubleLidar::setScanRate(double rate)
{
   yarp::os::LockGuard guard(m_mutex);
    if(!m_inited)
        return false;

    double currate;
    if(!m_dev_laserFront->getScanRate(currate))
        return false;

    if(!m_dev_laserFront->setScanRate(rate))
        return false;

    if(!m_dev_laserBack->setScanRate(rate))
    {
        m_dev_laserFront->setScanRate(currate);
        return false;
    }
    return true;
}

#define PI 3.14159265

static double convertAngle_user2Hw(double userAngle)
{
    double hwAngle =  userAngle + 90.0;
    if(hwAngle>360)
        hwAngle = hwAngle - 360.0;
    return hwAngle;
}

static double convertAngle_hw2user(double hwAngle)
{
    double userAngle = hwAngle-90.0;
    if(userAngle<0)
        userAngle = userAngle+360.0;
    return userAngle;
}

static inline double convertAngle_degree2rad(double angle)
{
    return angle*PI/180.0;
}


static inline double convertAngle_rad2degree(double angle)
{
    return angle*180.0/PI;
}


void cerDoubleLidar::calculate(int sensNum, double distance, int &newSensNum, double &newdistance, double x_off, double y_off, double t_off)
{
    //calculate the input angle in degree
    double angle_input = (sensNum*m_resolution);
    double angle_rad = convertAngle_degree2rad(angle_input);

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
    double beta2 = convertAngle_rad2degree(betarad); //the output is -180 +180
    
    //compute the new slot
    newSensNum= (double)(beta2/m_resolution);
    if (newSensNum > 720) newSensNum-= 720;
    if (newSensNum < 0)   newSensNum+= 720;
    
    //compute the distance
    newdistance = std::sqrt((Bx*Bx)+(By*By));

}



bool cerDoubleLidar::getRawData(yarp::sig::Vector &out)
{
    yarp::os::LockGuard guard(m_mutex);

    if(!m_inited)
    {
		return false;
	}
    yarp::sig::Vector dataFront;
    yarp::sig::Vector dataBack;
    
    //if(out.size() != m_samples)
        out.resize(m_samples, INFINITY);
    
    if(!m_dev_laserFront->getRawData(dataFront))
        return false;
    if(!m_dev_laserBack->getRawData(dataBack))
        return false;
    
    for(int i=0; i<m_samples; i++)
    {
         out[i] = INFINITY;
    }

    for(int i=0; i<m_samples; i++)
    {
        double newvalue;
        int newindex;
       
        //When hardware has problems, it gives me 0.0, so I skip it
        if((dataFront[i]!= INFINITY)&& (dataFront[i]!=0.0))
        {
			    calculate(i, dataFront[i], newindex, newvalue, m_lFrontCfg.pose.x, m_lFrontCfg.pose.y, m_lFrontCfg.pose.theta);
			    out[newindex] = newvalue;
	    }
        if((dataBack[i] != INFINITY)&& (dataBack[i]!=0.0))
        {
                calculate(i, dataBack[i], newindex, newvalue, m_lBackCfg.pose.x, m_lBackCfg.pose.y, m_lBackCfg.pose.theta);
                out[newindex] = newvalue;
	    }
    }
    
    return true;
}

bool cerDoubleLidar::getLaserMeasurement(std::vector<LaserMeasurementData> &data)
{
	yError() << "getLaserMeasurement ot yet implemented";
	return false;
	
	//Some transformation is missing in the following piece of code
    yarp::os::LockGuard guard(m_mutex);
    
    if(!m_inited)
        return false;
    std::vector<LaserMeasurementData> dataFront;
    std::vector<LaserMeasurementData> dataBack;
    if(data.size() != m_samples)
        data.resize(m_samples);
    
    if(!m_dev_laserFront->getLaserMeasurement(dataFront))
        return false;
    if(!m_dev_laserBack->getLaserMeasurement(dataBack))
        return false;
    
    for(int i=0; i<m_samples; i++)
    {
        double rhoFront, thetaFront, rhoBack, thetaBack;
        dataFront[i].get_polar(rhoFront, thetaFront);
        dataBack[i].get_polar(rhoBack, thetaBack);
        
        if(rhoFront!= INFINITY)
            data[i].set_polar(rhoFront, thetaFront);
        else  if(rhoBack != INFINITY)
                data[i].set_polar(rhoBack, thetaBack);
        else
            data[i].set_polar(INFINITY, i*m_resolution);
    }

    return true;
}

bool cerDoubleLidar::getDeviceStatus(Device_status &status)
{
    yarp::os::LockGuard guard(m_mutex);
    if(!m_inited)
        return false;
    Device_status statusFront, statusBack;
    if(! m_dev_laserFront->getDeviceStatus(statusFront))
        return false;
    if(! m_dev_laserBack->getDeviceStatus(statusBack))
        return false;
    if(statusFront == statusBack)
        status = statusFront;
    else
    {
        //TODO
        yError() << "cerDoubleLidar: the status of laser front (" << statusFront << ") differs from the status of laser back (" << statusBack << ")";
        return false;
    }

    return true;
}

bool cerDoubleLidar::getDeviceInfo (std::string &device_info)
{
    yarp::os::LockGuard guard(m_mutex);
    return true;
}


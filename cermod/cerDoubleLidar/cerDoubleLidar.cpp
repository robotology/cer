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
    {key="LASERFRONT-CFG";}
    else
    {key="LASERBACK-CFG";}

    yarp::os::Searchable& l_config = config.findGroup(key);
    if (l_config.check("pose")==false) {yError() << "cerDoubleLidar: missing pose"; return false; }
    Bottle & b_pose = l_config.findGroup("pose");
    if(b_pose.size()!= 4)
        yError() << "cerDoubleLidar: wrong size of pose";
    pose.x = b_pose.get(1).asDouble();
    pose.y = b_pose.get(2).asDouble();
    pose.z = b_pose.get(3).asDouble();

    if (l_config.check("file")==false) {yError() << "cerDoubleLidar: missing file"; return false; }
    Bottle & b_filename = l_config.findGroup("file");
    fileCfgName = b_filename.get(1).asString();

    if (l_config.check("sensorName")==false) {yError() << "cerDoubleLidar: missing sensorName"; return false; }
    Bottle & b_sensorName = l_config.findGroup("sensorName");
    sensorName = b_sensorName.get(1).asString();


    //DEBUG
    yError() << key << pose.x << pose.y << pose.z << fileCfgName << sensorName;

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
        yError() << "****cerDoubleLidar: finded laserFront device. OK";
    }

    if(m_driver_laserBack == nullptr)
    {
        yError() << "cerDoubleLidar: cannot find laserBack device";
        return false;
    }
    else
    {
        yError() << "****cerDoubleLidar: finded laserBack device. OK";
    }

    if(!m_driver_laserFront->view(m_dev_laserFront))
    {
        yError() << "cerDoubleLidar: cannot get interface of laser front";
        return false;
    }
    else
    {
        yError() << "****cerDoubleLidar: get interface of laser front. OK";
    }



    if(!m_driver_laserBack->view(m_dev_laserBack))
    {
        yError() << "cerDoubleLidar: cannot get interface of laser Back";
        return false;
    }
    else
    {
        yError() << "****cerDoubleLidar: get interface of laser Back. OK";
    }

    return true;
}

bool cerDoubleLidar::attachAll(const PolyDriverList &p)
{
    if(p.size()!=2)
    {
        yError() << "cerDoubleLidar attach whin wrong num of drivers";
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
            return false;
    if(!m_lBackCfg.loadConfig(config))
        return false;

    //currently if z values differs, than return error
    if(m_lFrontCfg.pose.z != m_lBackCfg.pose.z)
    {
        yError() << "cerDoubleLidar: poses of laser front and back have differnt z values";
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


void cerDoubleLidar::calculate(int sensNum, double distance, bool front, int &newSensNum, double &newdistance)
{
    //calculate the input angle in degree
    double angle_input = (sensNum*m_resolution);

    //converto fro user pace to hw space and to rad
    double hw_input_angle = convertAngle_user2Hw(angle_input);
    double angle_rad = convertAngle_degree2rad(hw_input_angle);

    //calculate vertical and horizontal components of input angle
    double Ay = std::abs(sin(angle_rad)*distance);
    double Ax = std::abs(cos(angle_rad)*distance);

    //calculate vertical and horizontal components of new angle with offset.
    //Note: sum laserpose.x to Ay is not an error! its dipend on the orienattion of axis.
    //I'm not sure about the use of abs in  std::abs(m_laserFrontPose.y). to be tested.
    double By, Bx;
    if(front)
    {
        By = Ay + std::abs(m_lFrontCfg.pose.x);
        Bx = Ax + std::abs(m_lFrontCfg.pose.y);
    }
    else
    {
        By = Ay + std::abs(m_lBackCfg.pose.x);
        Bx = Ax + std::abs(m_lBackCfg.pose.y);
    }

    double betarad = atan(By/Bx);
    double beta = convertAngle_rad2degree(betarad);

    //atan has codominio = (-PI/2 , PI/2), but since the input is only >0 than the atan output is in (0, PI/2).
    //Now I need to normalize the angle according to the input angle.
    double beta2;

    if(hw_input_angle>=90.0 && hw_input_angle<180)
        beta2=180-beta;
    else if(hw_input_angle >=180 && hw_input_angle <270)
        beta2= beta+180;
    else if(hw_input_angle >=270)
        beta2=360-beta;
    else
        beta2=beta;


    //now I check the difference between the input angle and the calculated angle in order to understand if I need to change the slot (sensorNum)
    double diff=beta2-hw_input_angle;
    newSensNum= sensNum+ round(diff);
    if(newSensNum>=m_samples)
    {
        newSensNum = newSensNum-m_samples;
        yError() << "cerDoubleLidar::calculate...something stange has been happened";
    }

    newdistance = std::sqrt((Bx*Bx)+(By*By));


//     //------debug stuff----
//     std::string res= "==>OK";
//     if(std::abs(diff)>m_resolution)
//         res="==> change slot!!!";
//
//     std::string laser="front";
//     if(!front)
//         laser="back";
//     //yError() << laser << "ANGLE_in=" << angle_input << "ANGLE=" << angle << "BETA="<<beta  <<"BETA2=" << beta2 <<  "  angle2=" <<angle2 << "DIFF="<< diff<<  res;
//
//     double beta3 = convertAngle_hw2user(beta2);
//
//     //newSensNum=round(beta3/m_resolution); per semplificare i calcoli la calcolo cosi:
//
//     yError() << laser << "INPUT_A=" <<angle_input << "("<<sensNum<<")"<< "ORIGINAL_D=" << distance<< "ORIGINAL_A=" << hw_input_angle << "NEW_DIST="<< newdistance << "NEW_ang=" << beta2  << "OUPUT_A="<< beta3 << "sensnum=" << newSensNum << "DIFF=" << diff <<  res;
//

}



bool cerDoubleLidar::getRawData(yarp::sig::Vector &out)
{
    yarp::os::LockGuard guard(m_mutex);

    if(!m_inited)
        return false;
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
        double tmp;
        int newindex;
        if(dataFront[i]!= INFINITY)
            calculate(i, dataFront[i], true, newindex, tmp);
        else if(dataBack[i] != INFINITY)
            calculate(i, dataBack[i], false, newindex, tmp);
        else
        {
            tmp = INFINITY;
            newindex = i;
        }
        out[newindex] = tmp;
    }
    
    return true;
}

bool cerDoubleLidar::getLaserMeasurement(std::vector<LaserMeasurementData> &data)
{
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


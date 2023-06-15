/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
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

#include "goHomeRobot.h"

YARP_LOG_COMPONENT(GO_HOME_ROBOT, "cer.goHomeRobot")

bool GoHomeRobot::configure(yarp::os::ResourceFinder &rf)
{
    std::string robot=rf.check("robot",yarp::os::Value("cer")).asString();

    // ----------- Polydriver config ----------- //
    yarp::os::Property goHomeProp;

    bool okGoHome{rf.check("GO_HOME_CLIENT")};
    if(!okGoHome)
    {
        yCError(GO_HOME_ROBOT,"GO_HOME_CLIENT section missing in ini file. Using default values.");

        // defaults
        goHomeProp.put("device","remote_controlboard");
        goHomeProp.put("local","/pointHandTransform/goHomeRArm");
        goHomeProp.put("remote","/cer/right_arm");
        if (!m_drivers[0].open(goHomeProp))
        {
            yCError(GO_HOME_ROBOT,"Unable to connect to %s",("/"+robot+"/right_arm").c_str());
            close();
            return false;
        }
        goHomeProp.clear();
        goHomeProp.put("device","remote_controlboard");
        goHomeProp.put("local","/pointHandTransform/goHomeLArm");
        goHomeProp.put("remote","/cer/left_arm");
        if (!m_drivers[1].open(goHomeProp))
        {
            yCError(GO_HOME_ROBOT,"Unable to connect to %s",("/"+robot+"/left_arm").c_str());
            close();
            return false;
        }
        goHomeProp.clear();
        goHomeProp.put("device","remote_controlboard");
        goHomeProp.put("local","/pointHandTransform/goHomeHead");
        goHomeProp.put("remote","/cer/head");
        if (!m_drivers[2].open(goHomeProp))
        {
            yCError(GO_HOME_ROBOT,"Unable to connect to %s",("/"+robot+"/head").c_str());
            close();
            return false;
        }
    }
    else
    {
        yarp::os::Searchable& goHome_config = rf.findGroup("GO_HOME_CLIENT");
        if(goHome_config.check("device")) {goHomeProp.put("device", goHome_config.find("device").asString());}
        if(goHome_config.check("local_r_arm")) {goHomeProp.put("local", goHome_config.find("local_r_arm").asString());}
        if(goHome_config.check("remote_r_arm")) {goHomeProp.put("remote", goHome_config.find("remote_r_arm").asString());}
        if (!m_drivers[0].open(goHomeProp))
        {
            yCError(GO_HOME_ROBOT,"Unable to connect to %s", goHome_config.find("remote_r_arm").asString().c_str());
            close();
            return false;
        }
        goHomeProp.clear();
        if(goHome_config.check("device")) {goHomeProp.put("device", goHome_config.find("device").asString());}
        if(goHome_config.check("local_l_arm")) {goHomeProp.put("local", goHome_config.find("local_l_arm").asString());}
        if(goHome_config.check("remote_l_arm")) {goHomeProp.put("remote", goHome_config.find("remote_l_arm").asString());}
        if (!m_drivers[1].open(goHomeProp))
        {
            yCError(GO_HOME_ROBOT,"Unable to connect to %s", goHome_config.find("remote_l_arm").asString().c_str());
            close();
            return false;
        }
        goHomeProp.clear();
        if(goHome_config.check("device")) {goHomeProp.put("device", goHome_config.find("device").asString());}
        if(goHome_config.check("local_head")) {goHomeProp.put("local", goHome_config.find("local_head").asString());}
        if(goHome_config.check("remote_head")) {goHomeProp.put("remote", goHome_config.find("remote_head").asString());}
        if (!m_drivers[2].open(goHomeProp))
        {
            yCError(GO_HOME_ROBOT,"Unable to connect to %s", goHome_config.find("remote_head").asString().c_str());
            close();
            return false;
        }
    }

    m_drivers[0].view(m_iRemCal[0]);
    m_drivers[1].view(m_iRemCal[1]);
    m_drivers[2].view(m_iRemCal[2]);
    if(!m_iRemCal[0] || !m_iRemCal[1] || !m_iRemCal[2])
    {
        yCError(GO_HOME_ROBOT,"Error opening iRemoteCalibrator interfaces. Devices not available");
        return false;
    }

    return true;
}

void GoHomeRobot::onRead(yarp::os::Bottle &b)
{
    //add a check if the robot is already moving
    backToHome();
}

void GoHomeRobot::backToHome()
{
    bool okRightArm = m_iRemCal[0]->homingWholePart();
    bool okLeftArm = m_iRemCal[1]->homingWholePart();
    bool okHead = m_iRemCal[2]->homingWholePart();
    if (!okRightArm || !okLeftArm || !okHead ) 
    {
        // providing better feedback to user by verifying if the calibrator device was set or not
        if (!okRightArm)  
        {
            bool isCalib = false;
            m_iRemCal[0]->isCalibratorDevicePresent(&isCalib);
            if (!isCalib)
            { yCError(GO_HOME_ROBOT, "Error with part: right_arm. No calibrator device was configured to perform this action, please verify that the wrapper config file for the part has the 'Calibrator' keyword in the attach phase"); }
            else
            { yCError(GO_HOME_ROBOT, "Error with part: right_arm. The remote calibrator reported that something went wrong during the calibration procedure"); }
        }
        if (!okRightArm)  
        {
            bool isCalib = false;
            m_iRemCal[1]->isCalibratorDevicePresent(&isCalib);
            if (!isCalib)
            { yCError(GO_HOME_ROBOT, "Error with part: left_arm. No calibrator device was configured to perform this action, please verify that the wrapper config file for the part has the 'Calibrator' keyword in the attach phase"); }
            else
            { yCError(GO_HOME_ROBOT, "Error with part: left_arm. The remote calibrator reported that something went wrong during the calibration procedure"); }
        }
        if (!okRightArm) 
        {
            bool isCalib = false;
            m_iRemCal[2]->isCalibratorDevicePresent(&isCalib);
            if (!isCalib)
            { yCError(GO_HOME_ROBOT, "Error with part: head. No calibrator device was configured to perform this action, please verify that the wrapper config file for the part has the 'Calibrator' keyword in the attach phase"); }
            else
            { yCError(GO_HOME_ROBOT, "Error with part: head. The remote calibrator reported that something went wrong during the calibration procedure"); }
        }
    }
    
}



void GoHomeRobot::close()
{
    if(m_drivers[0].isValid())
    {
        m_drivers[0].close();
    }

    if(m_drivers[1].isValid())
    {
        m_drivers[1].close();
    }

    if(m_drivers[2].isValid())
    {
        m_drivers[2].close();
    }
}

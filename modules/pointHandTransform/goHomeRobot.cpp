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

    m_drivers[0].view(m_iremcal[0]);
    m_drivers[1].view(m_iremcal[1]);
    m_drivers[2].view(m_iremcal[2]);
    if(!m_iremcal[0] || !m_iremcal[1] || !m_iremcal[2])
    {
        yCError(GO_HOME_ROBOT,"Error opening iRemoteCalibrator interfaces. Devices not available");
        return false;
    }

    m_drivers[0].view(m_iposctrl[0]);
    m_drivers[1].view(m_iposctrl[1]);
    m_drivers[2].view(m_iposctrl[2]);
    if(!m_iposctrl[0] || !m_iposctrl[1] || !m_iposctrl[2])
    {
        yCError(GO_HOME_ROBOT,"Error opening iPositionControl interfaces. Devices not available");
        return false;
    }

    m_drivers[0].view(m_ictrlmode[0]);
    m_drivers[1].view(m_ictrlmode[1]);
    m_drivers[2].view(m_ictrlmode[2]);
    if(!m_ictrlmode[0] || !m_ictrlmode[1] || !m_ictrlmode[2])
    {
        yCError(GO_HOME_ROBOT,"Error opening iControlMode interfaces. Devices not available");
        return false;
    }


    // ----------- Port config ----------- //
    if(okGoHome)
    {
        yarp::os::Searchable& goHome_config = rf.findGroup("GO_HOME_CLIENT");
        if (rf.check("arms_out_port")) {m_armsOutPortName = goHome_config.find("arms_out_port").asString();}
        if (rf.check("head_out_port")) {m_headOutPortName = goHome_config.find("head_out_port").asString();}
    }

    //open target ports
    if(!m_armsOutPort.open(m_armsOutPortName)){
        yCError(GO_HOME_ROBOT) << "Cannot open armsOutOut port with name" << m_armsOutPortName;
        return false;
    }

     if(!m_headOutPort.open(m_headOutPortName)){
        yCError(GO_HOME_ROBOT) << "Cannot open headOutOut port with name" << m_headOutPortName;
        return false;
    }

    return true;
}

void GoHomeRobot::onRead(yarp::os::Bottle &b)
{
    // ---- stop the actual controllers for arms and head ----- //
    yarp::os::Bottle&  toSend = m_armsOutPort.prepare();
    toSend.clear();
    toSend.addString("(stop)");
    m_armsOutPort.write();

    yarp::os::Bottle&  toSend1 = m_headOutPort.prepare();
    toSend1.clear();
    toSend1.addString("(stop)");
    m_headOutPort.write();

    backToHome();

}

void GoHomeRobot::backToHome()
{    

    // ------ set all the parts to Position control mode ------ //

    int NUMBER_OF_JOINTS;
    //  right arm
    m_iposctrl[0]->getAxes(&NUMBER_OF_JOINTS);
    for (int i_joint=0; i_joint < NUMBER_OF_JOINTS; i_joint++){ m_ictrlmode[0]->setControlMode(i_joint, VOCAB_CM_POSITION); }
    //  left arm
    m_iposctrl[1]->getAxes(&NUMBER_OF_JOINTS);
    for (int i_joint=0; i_joint < NUMBER_OF_JOINTS; i_joint++){ m_ictrlmode[1]->setControlMode(i_joint, VOCAB_CM_POSITION); }
    //  head
    m_iposctrl[2]->getAxes(&NUMBER_OF_JOINTS);
    for (int i_joint=0; i_joint < NUMBER_OF_JOINTS; i_joint++){ m_ictrlmode[2]->setControlMode(i_joint, VOCAB_CM_POSITION); }
    
    
    // ------ go home command  ------ //
    
    bool okRightArm = m_iremcal[0]->homingWholePart();
    bool okLeftArm = m_iremcal[1]->homingWholePart();
    bool okHead = m_iremcal[2]->homingWholePart();
    if (!okRightArm || !okLeftArm || !okHead ) 
    {
        // providing better feedback to user by verifying if the calibrator device was set or not. (SIM_CER_ROBOT does not have a calibrator!)
        if (!okRightArm)  
        {
            bool isCalib = false;
            m_iremcal[0]->isCalibratorDevicePresent(&isCalib);
            if (!isCalib)
            { yCError(GO_HOME_ROBOT, "Error with part: right_arm. No calibrator device was configured to perform this action, please verify that the wrapper config file for the part has the 'Calibrator' keyword in the attach phase"); }
            else
            { yCError(GO_HOME_ROBOT, "Error with part: right_arm. The remote calibrator reported that something went wrong during the calibration procedure"); }
        }
        if (!okRightArm)  
        {
            bool isCalib = false;
            m_iremcal[1]->isCalibratorDevicePresent(&isCalib);
            if (!isCalib)
            { yCError(GO_HOME_ROBOT, "Error with part: left_arm. No calibrator device was configured to perform this action, please verify that the wrapper config file for the part has the 'Calibrator' keyword in the attach phase"); }
            else
            { yCError(GO_HOME_ROBOT, "Error with part: left_arm. The remote calibrator reported that something went wrong during the calibration procedure"); }
        }
        if (!okRightArm) 
        {
            bool isCalib = false;
            m_iremcal[2]->isCalibratorDevicePresent(&isCalib);
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

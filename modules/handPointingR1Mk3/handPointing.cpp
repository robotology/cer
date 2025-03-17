/*
 * SPDX-FileCopyrightText: 2006-2025 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "handPointing.h"

YARP_LOG_COMPONENT(HAND_POINTING, "cer.handPointing")

HandPointing::HandPointing() :
    m_period(1.0)
{
}

bool HandPointing::configure(yarp::os::ResourceFinder &rf)
{
    if(rf.check("period")){m_period = rf.find("period").asFloat32();}

    double threadPeriod = 0.02;
    if(rf.check("thread_period")){threadPeriod = rf.find("thread_period").asFloat32();}
    
    // --------- Temp RGBDSensor config --------- //
    yarp::dev::PolyDriver tempPoly;
    yarp::dev::IRGBDSensor* tempIRGBD{nullptr};
    yarp::os::Property rgbdProp;

    //defaults
    rgbdProp.put("device", RGBDClient);
    rgbdProp.put("localImagePort", RGBDLocalImagePort);
    rgbdProp.put("localDepthPort", RGBDLocalDepthPort);
    rgbdProp.put("localRpcPort", RGBDLocalRpcPort);
    rgbdProp.put("remoteImagePort", RGBDRemoteImagePort);
    rgbdProp.put("remoteDepthPort", RGBDRemoteDepthPort);
    rgbdProp.put("remoteRpcPort", RGBDRemoteRpcPort);
    rgbdProp.put("ImageCarrier", RGBDImageCarrier);
    rgbdProp.put("DepthCarrier", RGBDDepthCarrier);
    bool okRgbdRf = rf.check("RGBD_SENSOR_CLIENT");
    if(!okRgbdRf)
    {
        yCWarning(HAND_POINTING,"RGBD_SENSOR_CLIENT section missing in ini file. Using default values");
    }
    else
    {
        yarp::os::Searchable& rgbd_config = rf.findGroup("RGBD_SENSOR_CLIENT");
        if(rgbd_config.check("device")) {rgbdProp.put("device", rgbd_config.find("device").asString());}
        if(rgbd_config.check("localImagePort")) {rgbdProp.put("localImagePort", rgbd_config.find("localImagePort").asString());}
        if(rgbd_config.check("localDepthPort")) {rgbdProp.put("localDepthPort", rgbd_config.find("localDepthPort").asString());}
        if(rgbd_config.check("localRpcPort")) {rgbdProp.put("localRpcPort", rgbd_config.find("localRpcPort").asString());}
        if(rgbd_config.check("remoteImagePort")) {rgbdProp.put("remoteImagePort", rgbd_config.find("remoteImagePort").asString());}
        if(rgbd_config.check("remoteDepthPort")) {rgbdProp.put("remoteDepthPort", rgbd_config.find("remoteDepthPort").asString());}
        if(rgbd_config.check("remoteRpcPort")) {rgbdProp.put("remoteRpcPort", rgbd_config.find("remoteRpcPort").asString());}
        if(rgbd_config.check("ImageCarrier")) {rgbdProp.put("ImageCarrier", rgbd_config.find("ImageCarrier").asString());}
        if(rgbd_config.check("DepthCarrier")) {rgbdProp.put("DepthCarrier", rgbd_config.find("DepthCarrier").asString());}
    }
    tempPoly.open(rgbdProp);
    if(!tempPoly.isValid())
    {
        yCError(HAND_POINTING,"Error opening PolyDriver check parameters");
        return false;
    }
    tempPoly.view(tempIRGBD);
    if(!tempIRGBD)
    {
        yCError(HAND_POINTING,"Error opening iRGBD interface. Device not available");
        return false;
    }

    tempPoly.close(); 

    
    // --------- Thread initialization --------- //
    m_innerThread = new HandPointingThread(threadPeriod,rf);
    bool threadOk = m_innerThread->start();
    if (!threadOk){
        return false;
    }

    std::string posName = "/handPointing/clicked_point:i";
    if(rf.check("clicked_point_port")){posName = rf.find("clicked_point_port").asString();}
    m_pointInputPort.useCallback(*m_innerThread);
    bool ret = m_pointInputPort.open(posName);
    if (!ret)
    {
        yCError(HAND_POINTING, "Unable to open module ports");
        return false;
    }

    // --------- goHome config --------- //
    m_goHomeRobot = new GoHomeRobot();
    bool okHome{m_goHomeRobot->configure(rf)};
    if(!okHome)
    {
        yCError(HAND_POINTING,"GoHomeRobot configuration failed");
        return false;
    }

    std::string homeName = "/handPointing/go_home:i";
    if(rf.check("go_home_port")){homeName = rf.find("go_home_port").asString();}
    m_goHomePort.useCallback(*m_goHomeRobot);
    bool ret1 = m_goHomePort.open(homeName);
    if (!ret1)
    {
        yCError(HAND_POINTING, "Unable to open module ports");
        return false;
    }

    return true;
}

bool HandPointing::close()
{
    m_innerThread->stop();
    delete m_innerThread;
    m_innerThread =NULL;
    m_pointInputPort.close();

    m_goHomeRobot->close();
    m_goHomePort.close();

    return true;
}

double HandPointing::getPeriod()
{
    return m_period;
}

bool HandPointing::updateModule()
{
    if (isStopping())
    {
        if (m_innerThread) m_innerThread->stop();
        return false;
    }

    return true;
}

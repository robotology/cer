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


#include "pointHandTransform.h"

YARP_LOG_COMPONENT(POINT_HAND_TRANSFORM, "cer.pointHandTransform")

PointHandTransform::PointHandTransform() :
    m_period(1.0)
{
}

bool PointHandTransform::configure(yarp::os::ResourceFinder &rf)
{
    if(rf.check("period")){m_period = rf.find("period").asFloat32();}

    double threadPeriod = 0.02;
    if(rf.check("thread_period")){threadPeriod = rf.find("thread_period").asFloat32();}
    
    // --------- Temp RGBDSensor config --------- //
    yarp::dev::PolyDriver tempPoly;
    yarp::dev::IRGBDSensor* tempIRGBD{nullptr};
    yarp::os::Property rgbdProp;
    // Prepare default prop object
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
        yCWarning(POINT_HAND_TRANSFORM,"RGBD_SENSOR_CLIENT section missing in ini file. Using default values");
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
        yCError(POINT_HAND_TRANSFORM,"Error opening PolyDriver check parameters");
        return false;
    }
    tempPoly.view(tempIRGBD);
    if(!tempIRGBD)
    {
        yCError(POINT_HAND_TRANSFORM,"Error opening iRGBD interface. Device not available");
        return false;
    }
    double horizFOV, verFOV;
    tempIRGBD->getRgbFOV(horizFOV,verFOV);
    int imgHeight = tempIRGBD->getRgbHeight();
    int imgWidth = tempIRGBD->getRgbWidth();

    tempPoly.close(); // Temporary polydriver closed

    
    // Thread initialization
    m_innerThread = new PointHandTransformThread(threadPeriod,rf);
    bool threadOk = m_innerThread->start();
    if (!threadOk){
        return false;
    }

    std::string posName = "/pointHandTransform/clicked_point:i";
    if(rf.check("clicked_point_port")){posName = rf.find("clicked_point_port").asString();}
    m_pointInputPort.useCallback(*m_innerThread);
    bool ret = m_pointInputPort.open(posName);
    if (!ret)
    {
        yCError(POINT_HAND_TRANSFORM, "Unable to open module ports");
        return false;
    }

    return true;
}

bool PointHandTransform::close()
{
    m_innerThread->stop();
    delete m_innerThread;
    m_innerThread =NULL;

    m_pointInputPort.close();

    return true;
}

double PointHandTransform::getPeriod()
{
    return m_period;
}

bool PointHandTransform::updateModule()
{
    if (isStopping())
    {
        if (m_innerThread) m_innerThread->stop();
        return false;
    }

    return true;
}

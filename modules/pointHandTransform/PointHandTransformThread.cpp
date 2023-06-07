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



#define _USE_MATH_DEFINES

#include "PointHandTransformThread.h"
#include <chrono>
#include <cmath>
#include <list>

YARP_LOG_COMPONENT(POINT_HAND_TRANSFORM_THREAD, "cer.pointHandTransform.PointHandTransformThread")
bool print{true};


PointHandTransformThread::PointHandTransformThread(double _period, yarp::os::ResourceFinder &rf):
    PeriodicThread(_period),
    TypedReaderCallback(),
    m_rf(rf)
{
    m_depth_width = 0;
    m_depth_height = 0;
    m_base_frame_id = "/base_frame";
    m_camera_frame_id = "/depth_camera_frame";
    m_targetOutPortName = "/pointHandTransform/target:o";
    m_pose_param = "xyz_pose";
    m_solver_config_param = "no_heave";
    m_torso_heave_param = 0.0;
    m_arm_heave_param = 0.2;
}

bool PointHandTransformThread::threadInit()
{
#ifdef HANDTRANSFORM_DEBUG
    yCDebug(POINT_HAND_TRANSFORM_THREAD, "thread initialising...\n");
#endif

    // --------- Generic config --------- //
    if (m_rf.check("target_point_port")) {m_targetOutPortName = m_rf.find("target_point_port").asString();}
    
    // --------- Frame config -------- //
    bool okFrameConfig = m_rf.check("FRAME_CONFIG");
    if(okFrameConfig)
    {
        yarp::os::Searchable& frame_config = m_rf.findGroup("FRAME_CONFIG");
        if (frame_config.check("base_frame_id")) {m_base_frame_id = frame_config.find("base_frame_id").asString();}
        if (frame_config.check("camera_frame_id")) {m_camera_frame_id = frame_config.find("camera_frame_id").asString();}
    }

    // --------- RGBDSensor config --------- //
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
    bool okRgbdRf = m_rf.check("RGBD_SENSOR_CLIENT");
    if(!okRgbdRf)
    {
        yCWarning(POINT_HAND_TRANSFORM_THREAD,"RGBD_SENSOR_CLIENT section missing in ini file. Using default values");
    }
    else
    {
        yarp::os::Searchable& rgbd_config = m_rf.findGroup("RGBD_SENSOR_CLIENT");
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

    m_rgbdPoly.open(rgbdProp);
    if(!m_rgbdPoly.isValid())
    {
        yCError(POINT_HAND_TRANSFORM_THREAD,"Error opening PolyDriver check parameters");
        return false;
    }
    m_rgbdPoly.view(m_iRgbd);
    if(!m_iRgbd)
    {
        yCError(POINT_HAND_TRANSFORM_THREAD,"Error opening iRGBD interface. Device not available");
        return false;
    }
   

    // --------- TransformClient config ---------- //
    yarp::os::Property tcProp;
    // Prepare default prop object
    tcProp.put("device", "frameTransformClient");
    tcProp.put("ft_client_prefix", "/pointHandTransform");
    tcProp.put("local_rpc", "/pointHandTransform/ftClient.rpc");
    bool okTransformRf = m_rf.check("TRANSFORM_CLIENT");
    if(!okTransformRf)
    {
        yCWarning(POINT_HAND_TRANSFORM_THREAD,"TRANSFORM_CLIENT section missing in ini file Using default values");
        tcProp.put("filexml_option","ftc_yarp_only.xml");
    }
    else {
        yarp::os::Searchable &tf_config = m_rf.findGroup("TRANSFORM_CLIENT");
        if (tf_config.check("ft_client_prefix")) {
            tcProp.put("ft_client_prefix", tf_config.find("ft_client_prefix").asString());
        }
        if (tf_config.check("ft_server_prefix")) {
            tcProp.put("ft_server_prefix", tf_config.find("ft_server_prefix").asString());
        }
        if(tf_config.check("filexml_option") && !(tf_config.check("testxml_from") || tf_config.check("testxml_context")))
        {
            tcProp.put("filexml_option", tf_config.find("filexml_option").asString());
        }
        else if(!tf_config.check("filexml_option") && (tf_config.check("testxml_from") && tf_config.check("testxml_context")))
        {
            tcProp.put("testxml_from", tf_config.find("testxml_from").asString());
            tcProp.put("testxml_context", tf_config.find("testxml_context").asString());
        }
        else
        {
            yCError(POINT_HAND_TRANSFORM_THREAD,"TRANSFORM_CLIENT is missing information about the frameTransformClient device configuration. Check your config. RETURNING");
            return false;
        }
    }
    m_tcPoly.open(tcProp);
    if(!m_tcPoly.isValid())
    {
        yCError(POINT_HAND_TRANSFORM_THREAD,"Error opening PolyDriver check parameters");
        return false;
    }
    m_tcPoly.view(m_iTc);
    if(!m_iTc)
    {
        yCError(POINT_HAND_TRANSFORM_THREAD,"Error opening iFrameTransform interface. Device not available");
        return false;
    }

    //get parameters data from the camera
    m_depth_width = m_iRgbd->getRgbWidth();
    m_depth_height = m_iRgbd->getRgbHeight();
    bool propintr  = m_iRgbd->getDepthIntrinsicParam(m_propIntrinsics);
    if(!propintr){
        return false;
    }
    yCInfo(POINT_HAND_TRANSFORM_THREAD) << "Depth Intrinsics:" << m_propIntrinsics.toString();
    m_intrinsics.fromProperty(m_propIntrinsics);

    if(!m_targetOutPort.open(m_targetOutPortName)){
        yCError(POINT_HAND_TRANSFORM_THREAD) << "Cannot open targetOut port with name" << m_targetOutPortName;
        return false;
    }

    // --------- Kinematic solver config ---------- //
    bool okSolverConfig = m_rf.check("KIN_SOLVER_CONFIG");
    if(okSolverConfig)
    {
        yarp::os::Searchable& solver_config = m_rf.findGroup("KIN_SOLVER_CONFIG");
        if (solver_config.check("pose")) {m_pose_param = solver_config.find("pose").asString();}
        if (solver_config.check("configuration")) {m_solver_config_param = solver_config.find("configuration").asString();}
        if (solver_config.check("torso_heave")) {m_torso_heave_param = solver_config.find("torso_heave").asFloat32();}
        if (solver_config.check("lower_arm_heave")) {m_arm_heave_param = solver_config.find("lower_arm_heave").asFloat32();}
        if (solver_config.check("ee_quaternion1") && solver_config.check("ee_quaternion2") && solver_config.check("ee_quaternion3") && solver_config.check("ee_quaternion4"))
            {m_ee_quaternion_param = {solver_config.find("ee_quaternion1").asFloat32(),
             m_ee_quaternion_param[1] = solver_config.find("ee_quaternion2").asFloat32(),
             m_ee_quaternion_param[2] = solver_config.find("ee_quaternion3").asFloat32(),
             m_ee_quaternion_param[3] = solver_config.find("ee_quaternion4").asFloat32()};}
    }

#ifdef HANDTRANSFORM_DEBUG
    yCDebug(POINT_HAND_TRANSFORM_THREAD, "... done!\n");
#endif

    return true;
}

void PointHandTransformThread::run()
{
    bool depth_ok = m_iRgbd->getDepthImage(m_depth_image);
    if (!depth_ok)
    {
        yCDebug(POINT_HAND_TRANSFORM_THREAD, "getDepthImage failed");
        return;
    }
    if (m_depth_image.getRawImage()==nullptr)
    {
        yCDebug(POINT_HAND_TRANSFORM_THREAD, "invalid image received");
        return;
    }

    bool rgb_ok = m_iRgbd->getRgbImage(m_rgbImage);
    if (!rgb_ok)
    {
        yCDebug(POINT_HAND_TRANSFORM_THREAD, "getRgbImage failed");
        return;
    }
    if (m_rgbImage.getRawImage()==nullptr)
    {
        yCDebug(POINT_HAND_TRANSFORM_THREAD, "invalid image received");
        return;
    }

    if (m_depth_image.width()!=m_depth_width ||
        m_depth_image.height()!=m_depth_height)
    {
        yCDebug(POINT_HAND_TRANSFORM_THREAD,"invalid image size: (%lu %lu) vs (%d %d)",m_depth_image.width(),m_depth_image.height(),m_depth_width,m_depth_height);
        return;
    }

    //we compute the transformation matrix from the camera to the base reference frame

    bool frame_exists = m_iTc->getTransform(m_camera_frame_id, m_base_frame_id, m_transform_mtrx);
    if (!frame_exists)
    {
        yCWarning(POINT_HAND_TRANSFORM_THREAD, "Unable to found m matrix");
    }
  

}


void PointHandTransformThread::onRead(yarp::os::Bottle &b)
{

    yCInfo(POINT_HAND_TRANSFORM_THREAD,"Received: %s",b.toString().c_str());

    if(b.size() == 2)
    {
        double u = b.get(0).asFloat32();
        double v = b.get(1).asFloat32();
        yarp::sig::Vector tempPoint(4,1.0);
        tempPoint[0] = (u - m_intrinsics.principalPointX) / m_intrinsics.focalLengthX * m_depth_image.pixel(u, v);
        tempPoint[1] = (v - m_intrinsics.principalPointY) / m_intrinsics.focalLengthY * m_depth_image.pixel(u, v);
        tempPoint[2] = m_depth_image.pixel(u, v);
        yarp::sig::Vector v2 = m_transform_mtrx*tempPoint;
        
        yarp::os::Bottle&  toSend = m_targetOutPort.prepare();
        toSend.clear();
        toSend.addString("askRequest");

        yarp::os::Bottle& parameterList = toSend.addList();
        parameterList.addString("parameters");
        yarp::os::Bottle& modeList = parameterList.addList();
        modeList.addString("mode");
        modeList.addString(m_pose_param + "+" + m_solver_config_param + "+forward_diff");
        yarp::os::Bottle& torsoHeaveList = parameterList.addList();
        torsoHeaveList.addString("torso_heave");
        torsoHeaveList.addFloat32(m_torso_heave_param);
        yarp::os::Bottle& armHeaveList = parameterList.addList();
        armHeaveList.addString("lower_arm_heave");
        armHeaveList.addFloat32(m_arm_heave_param);

        yarp::os::Bottle& tempList = toSend.addList();
        tempList.addString("target");
        yarp::os::Bottle& targetList = tempList.addList();
        targetList.addFloat32(v2[0]);
        targetList.addFloat32(v2[1]);
        targetList.addFloat32(v2[2]);
        targetList.addFloat32(m_ee_quaternion_param[0]);
        targetList.addFloat32(m_ee_quaternion_param[1]);
        targetList.addFloat32(m_ee_quaternion_param[2]);
        targetList.addFloat32(m_ee_quaternion_param[3]);

        m_targetOutPort.write();

        yCInfo(POINT_HAND_TRANSFORM_THREAD,"Sent to solver: %s",toSend.toString().c_str());
    }
    else{
        yCError(POINT_HAND_TRANSFORM_THREAD,"The input bottle has the wrong number of elements");
    }
    

}

void PointHandTransformThread::threadRelease()
{
#ifdef HANDTRANSFORM_DEBUG
    yCDebug(POINT_HAND_TRANSFORM_THREAD, "Thread releasing...");
#endif

    if(m_rgbdPoly.isValid())
        m_rgbdPoly.close();

    if(m_tcPoly.isValid())
        m_tcPoly.close();
    
    if(!m_targetOutPort.isClosed()){
        m_targetOutPort.close();
    }

    yCInfo(POINT_HAND_TRANSFORM_THREAD, "Thread released");

#ifdef HANDTRANSFORM_DEBUG
    yCDebug(POINT_HAND_TRANSFORM_THREAD, "... done.");
#endif

    return;
}

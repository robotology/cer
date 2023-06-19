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

#include "pointHandTransformThread.h"
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
    m_base_frame_id = "base_link";
    m_camera_frame_id = "depth_center";
    m_r_shoulder_frame_id = "r_shoulder_link";
    m_l_shoulder_frame_id = "l_shoulder_link";
    m_r_targetOutPortName = "/pointHandTransform/r_target:o";
    m_l_targetOutPortName = "/pointHandTransform/l_target:o";
    m_gazeTargetOutPortName = "/pointHandTransform/gazeTarget:o";
    m_getArmHomePortName = "/pointHandTransform/get_arm_home:o";
    m_pose_param = "xyz_pose";
    m_solver_config_param = "no_torso_no_heave";
    m_torso_heave_param = 0.0;
    m_arm_heave_param = 0.02;
    m_reach_radius = 0.6;
    m_gaze_target_type = "image";
}

bool PointHandTransformThread::threadInit()
{
#ifdef HANDTRANSFORM_DEBUG
    yCDebug(POINT_HAND_TRANSFORM_THREAD, "thread initialising...\n");
#endif

    // --------- Generic config --------- //
    if (m_rf.check("r_target_point_port")) {m_r_targetOutPortName = m_rf.find("r_target_point_port").asString();}
    if (m_rf.check("l_target_point_port")) {m_l_targetOutPortName = m_rf.find("l_target_point_port").asString();}
    if (m_rf.check("gaze_target_point_port")) {m_gazeTargetOutPortName = m_rf.find("gaze_target_point_port").asString();}
    if (m_rf.check("get_arm_home_port")) {m_getArmHomePortName = m_rf.find("get_arm_home_port").asString();}
    
    // --------- Frame config -------- //
    bool okFrameConfig = m_rf.check("FRAME_CONFIG");
    if(okFrameConfig)
    {
        yarp::os::Searchable& frame_config = m_rf.findGroup("FRAME_CONFIG");
        if (frame_config.check("base_frame_id")) {m_base_frame_id = frame_config.find("base_frame_id").asString();}
        if (frame_config.check("camera_frame_id")) {m_camera_frame_id = frame_config.find("camera_frame_id").asString();}
        if (frame_config.check("r_shoulder_frame_id")) {m_r_shoulder_frame_id = frame_config.find("r_shoulder_frame_id").asString();}
        if (frame_config.check("l_shoulder_frame_id")) {m_l_shoulder_frame_id = frame_config.find("l_shoulder_frame_id").asString();}
        if (frame_config.check("torso_frame_id")) {m_torso_frame_id = frame_config.find("torso_frame_id").asString();}
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

    //open target ports
    if(!m_r_targetOutPort.open(m_r_targetOutPortName)){
        yCError(POINT_HAND_TRANSFORM_THREAD) << "Cannot open r_targetOut port with name" << m_r_targetOutPortName;
        return false;
    }

    if(!m_l_targetOutPort.open(m_l_targetOutPortName)){
        yCError(POINT_HAND_TRANSFORM_THREAD) << "Cannot open l_targetOut port with name" << m_l_targetOutPortName;
        return false;
    }

    if(!m_gazeTargetOutPort.open(m_gazeTargetOutPortName)){
        yCError(POINT_HAND_TRANSFORM_THREAD) << "Cannot open gazeTargetOut port with name" << m_gazeTargetOutPortName;
        return false;
    }

    if(!m_getArmHomePort.open(m_getArmHomePortName)){
        yCError(POINT_HAND_TRANSFORM_THREAD) << "Cannot open gazeTargetOut port with name" << m_getArmHomePortName;
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

    // --------- Reachability radius ---------- //
    bool okRadius = m_rf.check("REACH_RADIUS");
    if(!okRadius)
    {
        yCWarning(POINT_HAND_TRANSFORM_THREAD,"REACH_RADIUS section missing in ini file Using default values");
    }
    else {
        yarp::os::Searchable &radius_config = m_rf.findGroup("REACH_RADIUS");
        if (radius_config.check("r")) {
            m_reach_radius = radius_config.find("r").asFloat32();
        }
    }

    // --------- Gaze Controller Config ---------- //
    bool okGazeCtrl = m_rf.check("GAZE_CONFIG");
    if(!okGazeCtrl)
    {
        yCWarning(POINT_HAND_TRANSFORM_THREAD,"GAZE_CONFIG section missing in ini file Using default values");
    }
    else {
        yarp::os::Searchable &gaze_config = m_rf.findGroup("GAZE_CONFIG");
        if (gaze_config.check("target_type")) {
            m_gaze_target_type = gaze_config.find("target_type").asString();
        }
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

    // compute the transformation matrix from the camera to the base reference frame
    bool base_frame_exists = m_iTc->getTransform(m_camera_frame_id, m_base_frame_id, m_transform_mtrx_camera);
    if (!base_frame_exists)
    {
        yCWarning(POINT_HAND_TRANSFORM_THREAD, "Unable to found m matrix (camera-base)");
    }

    // compute the transformation matrix from the right shoulder to the base reference frame
    bool r_shoulder_frame_exists = m_iTc->getTransform(m_r_shoulder_frame_id, m_base_frame_id, m_r_transform_mtrx_shoulder);
    if (!r_shoulder_frame_exists)
    {
        yCWarning(POINT_HAND_TRANSFORM_THREAD, "Unable to found m matrix (r_shoulder-base)");
    }

    // compute the transformation matrix from the left shoulder to the base reference frame
    bool l_shoulder_frame_exists = m_iTc->getTransform(m_l_shoulder_frame_id, m_base_frame_id, m_l_transform_mtrx_shoulder);
    if (!l_shoulder_frame_exists)
    {
        yCWarning(POINT_HAND_TRANSFORM_THREAD, "Unable to found m matrix (l_shoulder-base)");
    }

}

yarp::sig::Vector& PointHandTransformThread::reachablePoint(const yarp::sig::Vector& v0 , const yarp::sig::Vector& v1 , const yarp::sig::Vector& vSC, yarp::sig::Vector& vreach )
{
    //retrieve the coordinates of the interception between the line passing from point v0 to point v1, and a sphere with center in vSC

    double exp1 = (v1[1]-v0[1])/(v1[0]-v0[0]);
    double exp2 = (v1[2]-v0[2])/(v1[0]-v0[0]);
    double exp3 = v0[1] - exp1*v0[0];
    double exp4 = v0[2] - exp2*v0[0];

    double a_eq { 1 + pow(exp1, 2.0) + pow(exp2, 2.0) };
    double b_eq { -2*vSC[0] + 2*exp1*exp3 + 2*exp2*exp4 - 2*exp1*vSC[1] - 2*exp2*vSC[2] };
    double c_eq { pow(vSC[0],2.0) + pow(vSC[1],2.0) + pow(vSC[2],2.0) + pow(exp3,2.0) + pow(exp4,2.0) - 2*vSC[1]*exp3 - 2*vSC[2]*exp4 - pow(m_reach_radius,2.0) };

    vreach[0] = (-b_eq+sqrt(pow(b_eq,2.0)-4*a_eq*c_eq))/(2*a_eq);
    vreach[1] = vreach[0]*exp1 + exp3;
    vreach[2] = vreach[0]*exp2 + exp4;

    yCInfo(POINT_HAND_TRANSFORM_THREAD, "Reachable point defined");
    return vreach;
}

void PointHandTransformThread::onRead(yarp::os::Bottle &b)
{

    yCInfo(POINT_HAND_TRANSFORM_THREAD,"Received: %s",b.toString().c_str());

    if(b.size() == 2)
    {

        //transforming the clicked point to the target end-effector position
        double u = b.get(0).asFloat32();
        double v = b.get(1).asFloat32();
        yarp::sig::Vector tempPoint(4,1.0);
        tempPoint[0] = (u - m_intrinsics.principalPointX) / m_intrinsics.focalLengthX * m_depth_image.pixel(u, v);
        tempPoint[1] = (v - m_intrinsics.principalPointY) / m_intrinsics.focalLengthY * m_depth_image.pixel(u, v);
        tempPoint[2] = m_depth_image.pixel(u, v);
        
        //decide whether to use the right or left arm depending on the position of the point wrt to the torso
        yarp::sig::Matrix transform_mtrx_torso;
        bool torso_frame_exists = m_iTc->getTransform(m_camera_frame_id, m_torso_frame_id, transform_mtrx_torso);
        if (!torso_frame_exists)
        {
            yCError(POINT_HAND_TRANSFORM_THREAD, "Unable to found m matrix (camera-torso)");
        }
        yarp::sig::Vector vTorso = transform_mtrx_torso*tempPoint; //clicked point in point cloud coordinates wrt torso frame
        
        std::string armUsed;
        yarp::sig::Matrix* matrixShoulderUsed;
        yarp::os::BufferedPort<yarp::os::Bottle>* targetPortUsed;
        if(vTorso[1]>0) 
        {
            armUsed = "left-arm";
            matrixShoulderUsed = &m_l_transform_mtrx_shoulder;
            targetPortUsed = &m_l_targetOutPort;
        }
        else 
        {
            armUsed = "right-arm";
            matrixShoulderUsed = &m_r_transform_mtrx_shoulder;
            targetPortUsed = &m_r_targetOutPort;
        }

        

        yarp::sig::Vector v1 = m_transform_mtrx_camera*tempPoint; //clicked point in point cloud coordinates wrt base frame

        yarp::sig::Vector tempOrig {0.0, 0.0, 0.0, 1.0};
        yarp::sig::Vector v0 = (*matrixShoulderUsed)*tempOrig; //shoulder frame origin wrt to base frame 

        yarp::sig::Vector vTarget {0.0, 0.0, 0.0};
        vTarget = reachablePoint(v0, v1, v0, vTarget);
    
        //ee target output
        yarp::os::Bottle&  toSend = (*targetPortUsed).prepare();
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
        targetList.addFloat32(vTarget[0]);
        targetList.addFloat32(vTarget[1]);
        targetList.addFloat32(vTarget[2]);
        targetList.addFloat32(m_ee_quaternion_param[0]);
        targetList.addFloat32(m_ee_quaternion_param[1]);
        targetList.addFloat32(m_ee_quaternion_param[2]);
        targetList.addFloat32(m_ee_quaternion_param[3]);

        //gaze target output
        yarp::os::Bottle&  toSend1 = m_gazeTargetOutPort.prepare();
        toSend1.clear();
        if (m_gaze_target_type == "image")
        {
            yarp::os::Bottle& controlFrameList = toSend1.addList();
            controlFrameList.addString("control-frame");
            controlFrameList.addString("depth");
            yarp::os::Bottle& targetTypeList = toSend1.addList();
            targetTypeList.addString("target-type");
            targetTypeList.addString(m_gaze_target_type);
            yarp::os::Bottle& targetLocationList = toSend1.addList();
            targetLocationList.addString("target-location");
            yarp::os::Bottle& targetList1 = targetLocationList.addList();
            targetList1.addFloat32(u);
            targetList1.addFloat32(v);
        }
        else if (m_gaze_target_type == "cartesian") // TO BE FIXED !!
        {
            yarp::os::Bottle& controlFrameList = toSend1.addList();
            controlFrameList.addString("control-frame");
            controlFrameList.addString("gaze");
            yarp::os::Bottle& targetTypeList = toSend1.addList();
            targetTypeList.addString("target-type");
            targetTypeList.addString(m_gaze_target_type);
            yarp::os::Bottle& targetLocationList = toSend1.addList();
            targetLocationList.addString("target-location");

            yarp::sig::Matrix transform_mtrx_gaze;
            bool gaze_frame_exists = m_iTc->getTransform(m_camera_frame_id, "gaze", transform_mtrx_gaze);
            if (!gaze_frame_exists)
            {
                yCError(POINT_HAND_TRANSFORM_THREAD, "Unable to found m matrix (camera-gaze)");
            }
            yarp::sig::Vector vGaze = transform_mtrx_gaze*tempPoint; //clicked point wrt gaze frame

            yarp::os::Bottle& targetList1 = targetLocationList.addList();
            targetList1.addFloat32(vGaze[0]);
            targetList1.addFloat32(vGaze[1]);
            targetList1.addFloat32(vGaze[2]);
        }
        else 
        {
            yCWarning(POINT_HAND_TRANSFORM_THREAD,"Invalid or no Gaze Target Type defined. Gaze Controller not able to move head");
        }

        //output to goHome part
        yarp::os::Bottle&  toSend2 = m_getArmHomePort.prepare();
        toSend2.clear();
        toSend2.addString("goHome");
        if      (armUsed == "left-arm")  {toSend2.addString("right-arm"); }
        else if (armUsed == "right-arm") {toSend2.addString("left-arm");  }
        else {yCError(POINT_HAND_TRANSFORM_THREAD,"Neither the left nor the right arm is used"); }
        
        // ----- write ports ----- //
        m_getArmHomePort.write();
        yCInfo(POINT_HAND_TRANSFORM_THREAD,"Sent home other arm");

        m_gazeTargetOutPort.write();
        (*targetPortUsed).write();
        yCInfo(POINT_HAND_TRANSFORM_THREAD,"Sent to arm: %s. Sent to gaze: %s",toSend.toString().c_str(),toSend1.toString().c_str());

        
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
    
    if(!m_r_targetOutPort.isClosed()){
        m_r_targetOutPort.close();
    }
    
    if(!m_l_targetOutPort.isClosed()){
        m_l_targetOutPort.close();
    }
    
    if(!m_gazeTargetOutPort.isClosed()){
        m_gazeTargetOutPort.close();
    }
    
    if(!m_getArmHomePort.isClosed()){
        m_getArmHomePort.close();
    }

    yCInfo(POINT_HAND_TRANSFORM_THREAD, "Thread released");

#ifdef HANDTRANSFORM_DEBUG
    yCDebug(POINT_HAND_TRANSFORM_THREAD, "... done.");
#endif

    return;
}

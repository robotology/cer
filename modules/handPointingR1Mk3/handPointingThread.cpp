/*
 * SPDX-FileCopyrightText: 2006-2025 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <yarp/os/Log.h>
#include <yarp/os/LogComponent.h>
#include <yarp/math/Math.h>

#define _USE_MATH_DEFINES

#include "handPointingThread.h"

YARP_LOG_COMPONENT(HAND_POINTING_THREAD, "cer.handPointing.HandPointingThread")


HandPointingThread::HandPointingThread(double _period, yarp::os::ResourceFinder &rf):
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
    m_gazeTargetOutPortName = "/handPointing/gazeTarget:o";
    m_getArmHomePortName = "/handPointing/get_arm_home:o";
    m_reach_radius = 0.6;
    m_gaze_target_type = "image";
}

bool HandPointingThread::threadInit()
{
#ifdef HANDPOINT_DEBUG
    yCDebug(HAND_POINTING_THREAD, "thread initialising...\n");
#endif

    // --------- Generic config --------- //
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
        yCWarning(HAND_POINTING_THREAD,"RGBD_SENSOR_CLIENT section missing in ini file. Using default values");
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
        yCError(HAND_POINTING_THREAD,"Error opening PolyDriver check parameters");
        return false;
    }
    m_rgbdPoly.view(m_iRgbd);
    if(!m_iRgbd)
    {
        yCError(HAND_POINTING_THREAD,"Error opening iRGBD interface. Device not available");
        return false;
    }
   
    //get parameters data from the camera
    m_depth_width = m_iRgbd->getRgbWidth();
    m_depth_height = m_iRgbd->getRgbHeight();
    bool propintr  = m_iRgbd->getDepthIntrinsicParam(m_propIntrinsics);
    if(!propintr){
        return false;
    }
    yCInfo(HAND_POINTING_THREAD) << "Depth Intrinsics:" << m_propIntrinsics.toString();
    m_intrinsics.fromProperty(m_propIntrinsics);

    //open target ports
    if(!m_gazeTargetOutPort.open(m_gazeTargetOutPortName)){
        yCError(HAND_POINTING_THREAD) << "Cannot open gazeTargetOut port with name" << m_gazeTargetOutPortName;
        return false;
    }

    if(!m_getArmHomePort.open(m_getArmHomePortName)){
        yCError(HAND_POINTING_THREAD) << "Cannot open gazeTargetOut port with name" << m_getArmHomePortName;
        return false;
    }

    // --------- Reachability radius ---------- //
    bool okRadius = m_rf.check("REACH_RADIUS");
    if(!okRadius)
    {
        yCWarning(HAND_POINTING_THREAD,"REACH_RADIUS section missing in ini file Using default values");
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
        yCWarning(HAND_POINTING_THREAD,"GAZE_CONFIG section missing in ini file Using default values");
    }
    else {
        yarp::os::Searchable &gaze_config = m_rf.findGroup("GAZE_CONFIG");
        if (gaze_config.check("target_type")) {
            m_gaze_target_type = gaze_config.find("target_type").asString();
        }
    }

    // --------- Cartesian controller config --------- //

    // Check if we have a group for the cartesian controller
    bool okCartesianControllerConfig = m_rf.check("CARTESIAN_CONTROLLER_CONFIG");

    if(!okCartesianControllerConfig)
    {
        yCError(HAND_POINTING_THREAD,"CARTESIAN_CONTROLLER_CONFIG mandatory section missing in ini file");
        return false;
    }

    yarp::os::Searchable &group = m_rf.findGroup("CARTESIAN_CONTROLLER_CONFIG");

    // Check that both cartesian remote port and cartesian local port are present
    if(!group.check("cartesian_remote_port_right"))
    {
        yCError(HAND_POINTING_THREAD, "CARTESIAN_CONTROLLER_CONFIG section missing cartesian_remote_port_right in ini file");
        return false;
    }

    if(!group.check("cartesian_remote_port_left"))
    {
        yCError(HAND_POINTING_THREAD, "CARTESIAN_CONTROLLER_CONFIG section missing cartesian_remote_port_left in ini file");
        return false;
    }

    if(!group.check("cartesian_local_port_right"))
    {
        yCError(HAND_POINTING_THREAD, "CARTESIAN_CONTROLLER_CONFIG section missing cartesian_local_port_right in ini file");
        return false;
    }

    if(!group.check("cartesian_local_port_left"))
    {
        yCError(HAND_POINTING_THREAD, "CARTESIAN_CONTROLLER_CONFIG section missing cartesian_local_port_left in ini file");
        return false;
    }

    if(!group.check("cartesian_traj_duration"))
    {
        yCError(HAND_POINTING_THREAD, "CARTESIAN_CONTROLLER_CONFIG section missing cartesian_traj_duration in ini file");
        return false;
    }

    // Set the value for the cartesian remote and local port
    const auto cartesian_remote_port_right = group.find("cartesian_remote_port_right").asString();
    const auto cartesian_remote_port_left = group.find("cartesian_remote_port_left").asString();
    const auto cartesian_local_port_right = group.find("cartesian_local_port_right").asString();
    const auto cartesian_local_port_left = group.find("cartesian_local_port_left").asString();
    m_traj_duration = group.find("cartesian_traj_duration").asFloat64();
    
    // Open the rpc ports
    if(!m_rpc_port_right.open(cartesian_local_port_right))
    {
        yCError(HAND_POINTING_THREAD, "Unable to open cartesian_local_port_right");
        return false;
    }

    if(!m_rpc_port_left.open(cartesian_local_port_left))
    {
        yCError(HAND_POINTING_THREAD, "Unable to open cartesian_local_port_left");
        return false;
    }

    // Attach the rpc port to the service for the right arm
    if(!m_service_right.yarp().attachAsClient(m_rpc_port_right))
    {
        yCError(HAND_POINTING_THREAD, "Unable to attach service as client for the right arm rpc");
        return false;
    }

    // // Set the streaming mode to false
    // if(m_service_right.yarp().setStreamingMode(false))
    // {
    //     yCError(HAND_POINTING_THREAD, "Unable to set streaming mode for the right arm service");
    //     return false;
    // }

    // Attach the rpc port to the service for the left arm
    if(!m_service_left.yarp().attachAsClient(m_rpc_port_left))
    {
        yCError(HAND_POINTING_THREAD, "Unable to attach service as client for the left arm rpc");
        return false;
    }

    // // Set the streaming mode to false
    // if(m_service_left.yarp().setStreamingMode(false))
    // {
    //     yCError(HAND_POINTING_THREAD, "Unable to set streaming mode for the left arm service");
    //     return false;
    // }

    // Make the connection to the server.
    if (!yarp::os::NetworkBase::connect(cartesian_local_port_right, cartesian_remote_port_right, "tcp", true))
    {
        yCError(HAND_POINTING_THREAD,"::configure(). Error: cannot connect to the RPC server.");
        return false;
    }

    // Make the connection to the server.
    if (!yarp::os::NetworkBase::connect(cartesian_local_port_left, cartesian_remote_port_left, "tcp", true))
    {
        yCError(HAND_POINTING_THREAD,"::configure(). Error: cannot connect to the RPC server.");
        return false;
    }

    // --------- Frame Transform config --------- //
    // Initialize ROS 2 context first
    try {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
            yCInfo(HAND_POINTING_THREAD, "ROS 2 initialized successfully");
        }
        
        m_node = std::make_shared<rclcpp::Node>("hand_pointing_tf_client");
        m_tf_buffer = std::make_shared<tf2_ros::Buffer>(m_node->get_clock());
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
        
        // Create a thread for ros2 spinning
        std::thread([this](){
            rclcpp::spin(m_node);
        }).detach();
    }
    catch (const std::exception& e) {
        yCError(HAND_POINTING_THREAD, "Failed to initialize ROS 2: %s", e.what());
        return false;
    }


#ifdef HANDPOINT_DEBUG
    yCDebug(HAND_POINTING_THREAD, "... done!\n");
#endif

    return true;
}

void HandPointingThread::run()
{
    bool depth_ok = m_iRgbd->getDepthImage(m_depth_image);
    if (!depth_ok)
    {
        yCDebug(HAND_POINTING_THREAD, "getDepthImage failed");
        return;
    }
    if (m_depth_image.getRawImage()==nullptr)
    {
        yCDebug(HAND_POINTING_THREAD, "invalid image received");
        return;
    }

    bool rgb_ok = m_iRgbd->getRgbImage(m_rgbImage);
    if (!rgb_ok)
    {
        yCDebug(HAND_POINTING_THREAD, "getRgbImage failed");
        return;
    }
    if (m_rgbImage.getRawImage()==nullptr)
    {
        yCDebug(HAND_POINTING_THREAD, "invalid image received");
        return;
    }

    if (m_depth_image.width()!=m_depth_width ||
        m_depth_image.height()!=m_depth_height)
    {
        yCDebug(HAND_POINTING_THREAD,"invalid image size: (%lu %lu) vs (%d %d)",m_depth_image.width(),m_depth_image.height(),m_depth_width,m_depth_height);
        return;
    }

    // compute the transformation matrix from the camera to the base reference frame

    bool base_frame_exists = getTransformTF2(m_base_frame_id,m_camera_frame_id, m_transform_mtrx_camera);
    if (!base_frame_exists)
    {
        yCWarning(HAND_POINTING_THREAD, "Unable to found m matrix (camera-base)");
    }

    // compute the transformation matrix from the right shoulder to the base reference frame
    bool r_shoulder_frame_exists = getTransformTF2(m_base_frame_id,m_r_shoulder_frame_id, m_r_transform_mtrx_shoulder);
    if (!r_shoulder_frame_exists)
    {
        yCWarning(HAND_POINTING_THREAD, "Unable to found m matrix (r_shoulder-base)");
    }

    // compute the transformation matrix from the left shoulder to the base reference frame
    bool l_shoulder_frame_exists = getTransformTF2(m_base_frame_id,m_l_shoulder_frame_id, m_l_transform_mtrx_shoulder);
    if (!l_shoulder_frame_exists)
    {
        yCWarning(HAND_POINTING_THREAD, "Unable to found m matrix (l_shoulder-base)");
    }

}

yarp::sig::Vector& HandPointingThread::reachablePoint(const yarp::sig::Vector& v0 , const yarp::sig::Vector& v1 , const yarp::sig::Vector& vSC, yarp::sig::Vector& vreach )
{
    //retrieve the coordinates of the interception between the line passing from point v0 to point v1, and a sphere with center in vSC

    double exp1 = (v1[1]-v0[1])/(v1[0]-v0[0]);
    double exp2 = (v1[2]-v0[2])/(v1[0]-v0[0]);
    double exp3 = v0[1] - exp1*v0[0];
    double exp4 = v0[2] - exp2*v0[0];

    double d = sqrt(pow((v1[0]-v0[0]),2.0) + pow((v1[1]-v0[1]),2.0) + pow((v1[2]-v0[2]),2.0)); //object distance from robot
    d = d>0.4 ? d : 0.4;
    double r = d > m_reach_radius ? m_reach_radius : d;
    

    double a_eq { 1 + pow(exp1, 2.0) + pow(exp2, 2.0) };
    double b_eq { -2*vSC[0] + 2*exp1*exp3 + 2*exp2*exp4 - 2*exp1*vSC[1] - 2*exp2*vSC[2] };
    double c_eq { pow(vSC[0],2.0) + pow(vSC[1],2.0) + pow(vSC[2],2.0) + pow(exp3,2.0) + pow(exp4,2.0) - 2*vSC[1]*exp3 - 2*vSC[2]*exp4 - pow(r,2.0) };

    vreach[0] = (-b_eq+sqrt(pow(b_eq,2.0)-4*a_eq*c_eq))/(2*a_eq);
    vreach[1] = vreach[0]*exp1 + exp3;
    vreach[2] = vreach[0]*exp2 + exp4;

    yCInfo(HAND_POINTING_THREAD, "Reachable point defined");
    return vreach;
}

void HandPointingThread::onRead(yarp::os::Bottle &b)
{

    yCInfo(HAND_POINTING_THREAD,"Received: %s",b.toString().c_str());

    if(b.size() == 2)
    {

        //transforming the clicked point to the target end-effector position
        double u = b.get(0).asFloat32();
        double v = b.get(1).asFloat32();
        yarp::sig::Vector tempPoint(4,1.0);

        // We are averaging the depth value in a 10x10 box around the centre
        double av_pixel_value = average_depth(u,v,10);

        tempPoint[0] = (u - m_intrinsics.principalPointX) / m_intrinsics.focalLengthX * av_pixel_value;
        tempPoint[1] = (v - m_intrinsics.principalPointY) / m_intrinsics.focalLengthY * av_pixel_value;
        tempPoint[2] = av_pixel_value;
        
        yDebug() << "Clicked point in camera coordinates:" << tempPoint[0] << "," << tempPoint[1] << "," << tempPoint[2];

        //decide whether to use the right or left arm depending on the position of the point wrt to the torso
        yarp::sig::Matrix transform_mtrx_torso;
        bool torso_frame_exists = getTransformTF2(m_torso_frame_id,m_camera_frame_id, transform_mtrx_torso);
        if (!torso_frame_exists)
        {
            yCError(HAND_POINTING_THREAD, "Unable to found m matrix (camera-torso)");
            return;
        }
        yarp::sig::Vector vTorso = transform_mtrx_torso*tempPoint; //clicked point in point cloud coordinates wrt torso frame
        
        std::string armUsed;
        yarp::sig::Matrix* matrixShoulderUsed;
        if(vTorso[1]>0) 
        {
            armUsed = "left-arm";
            matrixShoulderUsed = &m_l_transform_mtrx_shoulder;
        }
        else 
        {
            armUsed = "right-arm";
            matrixShoulderUsed = &m_r_transform_mtrx_shoulder;
        }

        yarp::sig::Vector v1 = m_transform_mtrx_camera*tempPoint; //clicked point in point cloud coordinates wrt base frame

        yarp::sig::Vector vTarget {0.0, 0.0, 0.0};

        if (m_reach_radius == 0.0)
            vTarget = v1;
        else
        {
            yarp::sig::Vector tempOrig {0.0, 0.0, 0.0, 1.0};
            yarp::sig::Vector v0 = (*matrixShoulderUsed)*tempOrig; //shoulder frame origin wrt to base frame
            vTarget = reachablePoint(v0, v1, v0, vTarget);
        }
    
        // Construct the orientation of the target so that the end effector is aligned with the target

        // Extract the translation part (position of shoulder origin in base frame coordinates)
        yarp::sig::Vector shoulder_position_in_base(3);
        shoulder_position_in_base[0] = (*matrixShoulderUsed)(0, 3);
        shoulder_position_in_base[1] = (*matrixShoulderUsed)(1, 3);
        shoulder_position_in_base[2] = (*matrixShoulderUsed)(2, 3);

        // Calculate the vector from the shoulder position (in base frame) to the target position (in base frame)
        // Note: vTarget is 4D [x,y,z,1], shoulder_position_in_base is 3D [x,y,z]
        yarp::sig::Vector shoulder_to_target_in_base = vTarget.subVector(0, 2) - shoulder_position_in_base;

        yDebug() << "Shoulder position (base frame): " << shoulder_position_in_base.toString();
        yDebug() << "Target position (base frame): " << vTarget.subVector(0, 2).toString();
        yDebug() << "Vector from shoulder to target (base frame):" << shoulder_to_target_in_base.toString();

        // Get a rotation matrix that is collinear with the vector from the shoulder to the target
        yarp::sig::Matrix aligned_rotation_matrix = getAlignedRotation(shoulder_to_target_in_base);

        // Convert rotation matrix to rpy for logging
        yarp::sig::Vector rpy = yarp::math::dcm2rpy(aligned_rotation_matrix);

        // Convert rotation matrix to quaternion
        auto [qx,qy,qz,qw] = rot2quats(aligned_rotation_matrix);

        yInfo() << "Moving arm" << armUsed << "to" << vTarget[0] << "," <<  vTarget[1] << "," << vTarget[2];
        yInfo() << "With orientation (quats)" << qx << "," << qy << "," << qz << "," << qw;
        yInfo() << "With orientation (rpy)" << rpy[0] << "," << rpy[1] << "," << rpy[2];
        if(armUsed == "left-arm")
        {
            if(!m_service_left.go_to_pose(vTarget[0], vTarget[1], vTarget[2], qx, qy, qz, qw, m_traj_duration))
            {
                yCWarning(HAND_POINTING_THREAD,"Error in sending command to move the arm");
            }
        }
        else if(armUsed == "right-arm")
        {
            if(!m_service_right.go_to_pose(vTarget[0], vTarget[1], vTarget[2], qx, qy, qz, qw, m_traj_duration))
            {
                yCWarning(HAND_POINTING_THREAD,"Error in sending command to move the arm");
            }
        }
        else
        {
            yCError(HAND_POINTING_THREAD,"I don't have more than two arms, sorry");
        }

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
            bool gaze_frame_exists = getTransformTF2(m_camera_frame_id, "gaze", transform_mtrx_gaze);
            if (!gaze_frame_exists)
            {
                yCError(HAND_POINTING_THREAD, "Unable to found m matrix (camera-gaze)");
            }
            yarp::sig::Vector vGaze = transform_mtrx_gaze*tempPoint; //clicked point wrt gaze frame

            yarp::os::Bottle& targetList1 = targetLocationList.addList();
            targetList1.addFloat32(vGaze[0]);
            targetList1.addFloat32(vGaze[1]);
            targetList1.addFloat32(vGaze[2]);
        }
        else 
        {
            yCWarning(HAND_POINTING_THREAD,"Invalid or no Gaze Target Type defined. Gaze Controller not able to move head");
        }

        //output to goHome part
        if(armUsed == "left-arm")
        {
            if(!m_service_right.go_home())
            {
                yCWarning(HAND_POINTING_THREAD,"Error in sending command to move the right arm home");
            }
        }
        else if(armUsed == "right-arm")
        {
            if(!m_service_left.go_home())
            {
                yCWarning(HAND_POINTING_THREAD,"Error in sending command to move the left arm home");
            }
        }
        else
        {
            yCError(HAND_POINTING_THREAD,"I don't have more than two arms, sorry");
        }

        // yarp::os::Bottle&  toSend2 = m_getArmHomePort.prepare();
        // toSend2.clear();
        // toSend2.addString("goHome");
        // yarp::os::Bottle& getArmHomeList = toSend2.addList();
        // if      (armUsed == "left-arm")  {getArmHomeList.addString("right-arm"); }
        // else if (armUsed == "right-arm") {getArmHomeList.addString("left-arm");  }
        // else {yCError(HAND_POINTING_THREAD,"Neither the left nor the right arm is used"); }
        
        // ----- write ports ----- //
        m_getArmHomePort.write();
        yCInfo(HAND_POINTING_THREAD,"Sent home other arm");

        m_gazeTargetOutPort.write();
    }
    else{
        yCError(HAND_POINTING_THREAD,"The input bottle has the wrong number of elements");
    }
    

}

void HandPointingThread::threadRelease()
{
#ifdef HANDPOINT_DEBUG
    yCDebug(HAND_POINTING_THREAD, "Thread releasing...");
#endif

    if(m_rgbdPoly.isValid())
        m_rgbdPoly.close();

    if(m_tcPoly.isValid())
        m_tcPoly.close();
       
    if(!m_gazeTargetOutPort.isClosed()){
        m_gazeTargetOutPort.close();
    }
    
    if(!m_getArmHomePort.isClosed()){
        m_getArmHomePort.close();
    }

    yCInfo(HAND_POINTING_THREAD, "Thread released");

    if (m_node) {
        rclcpp::shutdown();
    }

#ifdef HANDPOINT_DEBUG
    yCDebug(HAND_POINTING_THREAD, "... done.");
#endif

    return;
}

double HandPointingThread::average_depth(int u, int v, int max_offset)
{
    double av_pixel_value = 0;
    int n_valid_pixels = 0;
    for (int offset_u = -max_offset/2; offset_u <max_offset/2; offset_u++)
    {
        for (int offset_v = -max_offset/2; offset_v < max_offset/2; offset_v++)
        {
            auto pixel = m_depth_image.pixel(u+offset_u,v+offset_v);                 
            if (pixel != 0)
            {
                n_valid_pixels += 1;
                av_pixel_value = av_pixel_value*(n_valid_pixels-1)/n_valid_pixels + pixel/n_valid_pixels;
            }
        }
    }

    return av_pixel_value;
}

bool HandPointingThread::getTransformTF2(const std::string& target_frame, const std::string& source_frame, 
    yarp::sig::Matrix& transform_matrix)
{
    try {
        // Look up the transform
        // yDebug() << "Looking up transform from" << source_frame << "to" << target_frame;
        geometry_msgs::msg::TransformStamped transform = 
        m_tf_buffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero);

        // Convert to YARP matrix format (4x4 homogeneous transformation matrix)
        transform_matrix.resize(4, 4);

        // Extract rotation as quaternion
        double qx = transform.transform.rotation.x;
        double qy = transform.transform.rotation.y;
        double qz = transform.transform.rotation.z;
        double qw = transform.transform.rotation.w;

        // Convert quaternion to rotation matrix (3x3 part)
        double x2 = qx * qx;
        double y2 = qy * qy;
        double z2 = qz * qz;
        double xy = qx * qy;
        double xz = qx * qz;
        double yz = qy * qz;
        double wx = qw * qx;
        double wy = qw * qy;
        double wz = qw * qz;

        transform_matrix(0,0) = 1.0 - 2.0 * (y2 + z2);
        transform_matrix(0,1) = 2.0 * (xy - wz);
        transform_matrix(0,2) = 2.0 * (xz + wy);
        transform_matrix(1,0) = 2.0 * (xy + wz);
        transform_matrix(1,1) = 1.0 - 2.0 * (x2 + z2);
        transform_matrix(1,2) = 2.0 * (yz - wx);
        transform_matrix(2,0) = 2.0 * (xz - wy);
        transform_matrix(2,1) = 2.0 * (yz + wx);
        transform_matrix(2,2) = 1.0 - 2.0 * (x2 + y2);

        // Set translation vector
        transform_matrix(0,3) = transform.transform.translation.x;
        transform_matrix(1,3) = transform.transform.translation.y;
        transform_matrix(2,3) = transform.transform.translation.z;

        // Set bottom row for homogeneous matrix
        transform_matrix(3,0) = 0.0;
        transform_matrix(3,1) = 0.0;
        transform_matrix(3,2) = 0.0;
        transform_matrix(3,3) = 1.0;

        return true;
    }
    catch (const tf2::TransformException& ex) {
        yCError(HAND_POINTING_THREAD, "Failed to get transform: %s", ex.what());
        return false;
    }
}

yarp::sig::Matrix HandPointingThread::getAlignedRotation(const yarp::sig::Vector &v0) const
{
    // Let's first define the frame of the end effector
    // The end effector frame z axis is aligned with the direction of the vector v0
    yarp::sig::Vector ee_z = v0 / yarp::math::norm(v0);

    // We take the negative of the z axis to have the end effector pointing towards the target
    ee_z[0] = -ee_z[0];
    ee_z[1] = -ee_z[1];
    ee_z[2] = -ee_z[2];

    // Define primary up vector and alternative right vector
    const yarp::sig::Vector world_y_axis {0.0, 1.0, 0.0}; // Primary up vector
    const yarp::sig::Vector world_x_axis {1.0, 0.0, 0.0}; // Alternative reference vector
    const double epsilon = 1e-6; // Tolerance for parallelism check

    yarp::sig::Vector ee_x(3);
    yarp::sig::Vector ee_y(3);

    // Check if ee_z is aligned with the world Y-axis (up_vector)
    if (std::abs(yarp::math::dot(world_y_axis,ee_z)) > (1.0 - epsilon))
    {
        // ee_z is aligned with world Y. Use world X as the reference to find ee_y.
        yCDebug(HAND_POINTING_THREAD, "ee_z is aligned with world Y, using alternative reference.");
        ee_y = yarp::math::cross(ee_z, world_x_axis);
        ee_y = ee_y / yarp::math::norm(ee_y); // Normalize ee_y
        ee_x = yarp::math::cross(ee_y, ee_z);
        ee_x = ee_x / yarp::math::norm(ee_x); // Normalize ee_x
    }
    else
    {
        // ee_z is not aligned with world Y. Use standard Gram-Schmidt with world Y.
        // Orthogonalize world_y_axis with respect to ee_z to get ee_y
        ee_y = world_y_axis - (world_y_axis * ee_z) * ee_z;
        ee_y = ee_y / yarp::math::norm(ee_y); // Normalize ee_y

        // Calculate ee_x as the cross product of ee_y and ee_z
        ee_x = yarp::math::cross(ee_y, ee_z);
        ee_x = ee_x / yarp::math::norm(ee_x); // Normalize ee_x (should be already normalized if ee_y and ee_z are orthonormal)
    }

        // Now we can construct the rotation matrix
        yarp::sig::Matrix rotation_matrix(3, 3);
        rotation_matrix(0, 0) = ee_x[0];
        rotation_matrix(1, 0) = ee_x[1];
        rotation_matrix(2, 0) = ee_x[2];
        rotation_matrix(0, 1) = ee_y[0];
        rotation_matrix(1, 1) = ee_y[1];
        rotation_matrix(2, 1) = ee_y[2];
        rotation_matrix(0, 2) = ee_z[0];
        rotation_matrix(1, 2) = ee_z[1];
        rotation_matrix(2, 2) = ee_z[2];

        return rotation_matrix;
}

std::tuple<double,double,double,double> HandPointingThread::rot2quats(const yarp::sig::Matrix& rot) const
{
    // Then we can convert the rotation matrix to a quaternion
    double qw = sqrt(1.0+ rot(0, 0) + rot(1, 1) + rot(2, 2)) / 2.0;
    double qx = (rot(2, 1) - rot(1, 2)) / (4.0 * qw);
    double qy = (rot(0, 2) - rot(2, 0)) / (4.0 * qw);
    double qz = (rot(1, 0) - rot(0, 1)) / (4.0 * qw);

    return {qx, qy, qz, qw};
}     
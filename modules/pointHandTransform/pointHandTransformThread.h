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

#ifndef POINT_HAND_TRANSFORM_THREAD_H
#define POINT_HAND_TRANSFORM_THREAD_H

#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/sig/IntrinsicParams.h>
#include <yarp/sig/PointCloud.h>
#include <yarp/sig/PointCloudUtils.h>
#include <yarp/math/Math.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <math.h>
#include <mutex>
#include <algorithm>

//Defaults
// RGBD sensor
#define RGBDClient            "RGBDSensorClient"
#define RGBDLocalImagePort    "/clientRgbPort:i"
#define RGBDLocalDepthPort    "/clientDepthPort:i"
#define RGBDLocalRpcPort      "/clientRpcPort"
#define RGBDRemoteImagePort   "/SIM_CER_ROBOT/depthCamera/rgbImage:o"
#define RGBDRemoteDepthPort   "/SIM_CER_ROBOT/depthCamera/depthImage:o"
#define RGBDRemoteRpcPort     "/SIM_CER_ROBOT/depthCamera/rpc:i"
#define RGBDImageCarrier      "mjpeg"
#define RGBDDepthCarrier      "fast_tcp"


class PointHandTransformThread : public yarp::os::PeriodicThread, public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
protected:
    //Devices related attributes
    yarp::dev::PolyDriver            m_rgbdPoly;
    yarp::dev::IRGBDSensor*          m_iRgbd{nullptr};
    yarp::dev::PolyDriver            m_tcPoly;
    yarp::dev::IFrameTransform*      m_iTc{nullptr};

    //Computation related attributes
    int    m_depth_width;
    int    m_depth_height;  
    std::string                  m_base_frame_id;
    std::string                  m_camera_frame_id;
    std::string                  m_r_shoulder_frame_id;
    std::string                  m_l_shoulder_frame_id;
    std::string                  m_torso_frame_id;
    yarp::sig::Matrix            m_transform_mtrx_camera;
    yarp::sig::Matrix            m_r_transform_mtrx_shoulder;
    yarp::sig::Matrix            m_l_transform_mtrx_shoulder;
    yarp::os::Property           m_propIntrinsics;
    yarp::sig::FlexImage         m_rgbImage;
    yarp::sig::ImageOf<float>    m_depth_image;
    yarp::sig::IntrinsicParams   m_intrinsics;
    
    //Kinematic Solver Parameters
    std::string                  m_pose_param;
    std::string                  m_solver_config_param;
    double                       m_torso_heave_param;
    double                       m_arm_heave_param;
    std::vector<double>          m_ee_quaternion_param {0,0,0,1};

    //Ports
    std::string                                m_r_targetOutPortName;
    std::string                                m_l_targetOutPortName;
    std::string                                m_gazeTargetOutPortName;
    std::string                                m_getArmHomePortName;
    yarp::os::BufferedPort<yarp::os::Bottle>   m_r_targetOutPort;
    yarp::os::BufferedPort<yarp::os::Bottle>   m_l_targetOutPort;
    yarp::os::BufferedPort<yarp::os::Bottle>   m_gazeTargetOutPort;
    yarp::os::BufferedPort<yarp::os::Bottle>   m_getArmHomePort;

    //Others
    yarp::os::ResourceFinder    &m_rf;
    double                      m_reach_radius;
    std::string                 m_gaze_target_type;

public:
    //Contructor and distructor
    PointHandTransformThread(double _period, yarp::os::ResourceFinder &rf);
    ~PointHandTransformThread() = default;

    //methods inherited from PeriodicThread
    virtual void run() override;
    virtual bool threadInit() override;
    virtual void threadRelease() override;

    //Port inherited from TypedReaderCallback
    using TypedReaderCallback<yarp::os::Bottle>::onRead;
    void onRead(yarp::os::Bottle& b) override;

private:
    yarp::sig::Vector& reachablePoint(const yarp::sig::Vector& v0 , const yarp::sig::Vector& v1 , const yarp::sig::Vector& vSC, yarp::sig::Vector& vreach );
};

#endif

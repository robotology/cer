/*
 * SPDX-FileCopyrightText: 2006-2025 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef HAND_POINTING_THREAD_H
#define HAND_POINTING_THREAD_H

#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/os/RpcClient.h>
#include <yarp/sig/IntrinsicParams.h>
#include <yarp/sig/PointCloud.h>
#include <yarp/sig/PointCloudUtils.h>
#include <yarp/math/Math.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <gb-ergocub-cartesian-service/ergoCubCartesianService.h>

// Use ros2 for the transform client
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <math.h>

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


class HandPointingThread : public yarp::os::PeriodicThread, public yarp::os::TypedReaderCallback<yarp::os::Bottle>
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
    
    //Ports
    std::string                                m_gazeTargetOutPortName;
    std::string                                m_getArmHomePortName;
    yarp::os::BufferedPort<yarp::os::Bottle>   m_gazeTargetOutPort;
    yarp::os::BufferedPort<yarp::os::Bottle>   m_getArmHomePort;

    // Cartesian controller

    // Cartesian controller port
    yarp::os::RpcClient m_rpc_port_left;
    yarp::os::RpcClient m_rpc_port_right;

    // Cartesian controller interface
    ergoCubCartesianService m_service_left;
    ergoCubCartesianService m_service_right;

    // Cartesian controller parameters
    double m_traj_duration;

    //Others
    yarp::os::ResourceFinder    &m_rf;
    double                      m_reach_radius;
    std::string                 m_gaze_target_type;

public:
    //Contructor and distructor
    HandPointingThread(double _period, yarp::os::ResourceFinder &rf);
    ~HandPointingThread() = default;

    //methods inherited from PeriodicThread
    virtual void run() override;
    virtual bool threadInit() override;
    virtual void threadRelease() override;

    //Port inherited from TypedReaderCallback
    using TypedReaderCallback<yarp::os::Bottle>::onRead;
    void onRead(yarp::os::Bottle& b) override;

    bool getTransformTF2(const std::string& target_frame,
                         const std::string& source_frame,
                         yarp::sig::Matrix& transform);

    std::tuple<double,double,double,double> rot2quats(const yarp::sig::Matrix& rot) const;

private:
    yarp::sig::Vector& reachablePoint(const yarp::sig::Vector& v0 , const yarp::sig::Vector& v1 , const yarp::sig::Vector& vSC, yarp::sig::Vector& vreach );
    double average_depth(int u,int v,int offset);

    // Return a rotation matrix that is aligned with the input vector
    yarp::sig::Matrix getAlignedRotation(const yarp::sig::Vector& v0) const;

    //Tf2 members
    std::shared_ptr<rclcpp::Node> m_node;
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

};

#endif

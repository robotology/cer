/*
* Copyright (C)2015  iCub Facility - Istituto Italiano di Tecnologia
* Author: Marco Randazzo
* email:  marco.randazzo@iit.it
* website: www.robotcub.org
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* http://www.robotcub.org/icub/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <yarp/math/Math.h>
#include <yarp/os/Stamp.h>
#include <string>
#include <math.h>
#include <yarp/os/Node.h>
#include <yarp/os/Publisher.h>
#include "include/nav_msgs_Odometry.h"
#include "include/geometry_msgs_PolygonStamped.h"
#include "include/geometry_msgs_TransformStamped.h"
#include "include/tf_tfMessage.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

#ifndef M_PI
#define M_PI 3.14159265
#endif

class Odometry
{
private:
    Property ctrl_options;
    yarp::os::Semaphore   mutex;
    yarp::os::Stamp       timeStamp;

    //encoder variables
    double              encL_offset;
    double              encR_offset;

    double              encL;
    double              encR;

    //measured motor velocity
    double              velL;
    double              velR;

    //estimated motor velocity
    double              velL_est;
    double              velR_est;
    iCub::ctrl::AWLinEstimator      *encvel_estimator;

    //robot geometry
    double              geom_r;
    double              geom_L;

    //thread period
    double              period;

    //ros
    bool                enable_ROS;
    yarp::os::Publisher<nav_msgs_Odometry>          rosPublisherPort_odometry;
    std::string                                     odometry_frame_id;
    std::string                                     child_frame_id;
    std::string                                     rosNodeName;
    std::string                                     rosTopicName_odometry;
    yarp::os::Node                                  *rosNode;
    yarp::os::NetUint32                             rosMsgCounter;

    yarp::os::Publisher<geometry_msgs_PolygonStamped>          rosPublisherPort_footprint;
    double                                                     footprint_diameter;
    std::string                                                rosTopicName_footprint;
    geometry_msgs_PolygonStamped                               footprint;
    std::string                                                footprint_frame_id;

    yarp::os::Publisher<tf_tfMessage>                          rosPublisherPort_tf;

    yarp::sig::Vector enc;
    yarp::sig::Vector encv;

public:
    //estimated cartesian velocity in the fixed odometry reference frame (world)
    double              odom_vel_x;
    double              odom_vel_y;
    double              odom_vel_lin;
    double              odom_vel_theta;

    //estimated cartesian velocity in the base relative reference frame
    double              base_vel_x;
    double              base_vel_y;
    double              base_vel_lin;
    double              base_vel_theta;

    //estimated odometer 
    double              traveled_distance;
    double              traveled_angle;

    //estimated cartesian pos in the fixed odometry reference frame
    double              odom_x;
    double              odom_y;
    double              odom_theta;

private:
    ResourceFinder                  &rf;
    PolyDriver                      *control_board_driver;
    BufferedPort<Bottle>            port_odometry;
    BufferedPort<Bottle>            port_odometer;
    BufferedPort<Bottle>            port_vels;

    string                          localName;
    IEncoders                       *ienc;

public:
    Odometry(unsigned int _period, ResourceFinder &_rf, Property options, PolyDriver* _driver); 
    ~Odometry();
    bool reset_odometry();
    bool open();
    void compute();
    void printStats();
    void close();

};

#endif

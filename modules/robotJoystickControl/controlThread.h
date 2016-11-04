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

#ifndef CONTROL_THREAD_H
#define CONTROL_THREAD_H

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/pids.h>
#include <string>
#include <math.h>
#include "ros_messages/visualization_msgs_Marker.h"
#include "ros_messages/visualization_msgs_MarkerArray.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

#define MAX_LINEAR_VEL  0.4  // maximum linear  velocity (m/s)
#define MAX_ANGULAR_VEL 24.0 // maximum angular velocity (deg/s)


class ControlThread : public yarp::os::RateThread
{
private:
    Property            ctrl_options;

    double              thread_period;
    int                 base_control_type;
    int                 thread_timeout_counter;

protected:
    ResourceFinder            &rf;
    PolyDriver                *driver_head;
    PolyDriver                *driver_torso_tripod;
    PolyDriver                *driver_torso_equiv;
    PolyDriver                *driver_left_hand;
    PolyDriver                *driver_right_hand;

    Publisher<visualization_msgs_MarkerArray> rosPublisherPort;
    Node *rosNode;

    BufferedPort<Bottle>      port_joystick_control;

    string                    robotName;
    string                    localName;
    bool                      first_run;
    bool                      error_status;

    //connection to the robot
    RpcClient                 robotCmdPort_larm;
    RpcClient                 robotCmdPort_rarm;

    BufferedPort<Bottle>      robotStatusPort_larm;
    BufferedPort<Bottle>      robotStatusPort_rarm;

    BufferedPort<Property>    robotTargetPort_larm;
    BufferedPort<Property>    robotTargetPort_rarm;
    BufferedPort<Bottle>      robotCmdPort_base;

    IPositionControl*         interface_torso_tripod_iPos;
    IVelocityControl*         interface_torso_tripod_iVel;
    IPositionDirect*          interface_torso_tripod_iDir;
    IEncoders*                interface_torso_tripod_iEnc;
    IControlMode2*            interface_torso_tripod_iCmd;

    IPositionControl*         interface_torso_equiv_iPos;
    IVelocityControl*         interface_torso_equiv_iVel;
    IControlMode2*            interface_torso_equiv_iCmd;

    IControlMode2*            interface_left_hand_iCmd;
    IVelocityControl*         interface_left_hand_iVel;

    IControlMode2*            interface_right_hand_iCmd;
    IVelocityControl*         interface_right_hand_iVel;

    IPositionControl*         interface_head_iPos;
    IControlMode2*            interface_head_iCmd;
    IVelocityControl*         interface_head_iVel;

    yarp::sig::Vector         initial_xyz_left;
    yarp::sig::Vector         initial_rpy_left;
    yarp::sig::Vector         initial_xyz_right;
    yarp::sig::Vector         initial_rpy_right;
    yarp::sig::Vector         current_xyz_left;
    yarp::sig::Vector         current_rpy_left;
    yarp::sig::Vector         current_xyz_right;
    yarp::sig::Vector         current_rpy_right;

    double                    torso_elong;
    double                    torso_pitch;
    double                    torso_roll;
    double                    torso_max_elong;
    double                    torso_min_elong;
    double                    torso_max_alpha;

public:
    bool                      motors_enabled;

    ControlThread(unsigned int _period, ResourceFinder &_rf, Property options);

    virtual bool threadInit();
    virtual void threadRelease();
    virtual void afterStart(bool s);
    virtual void run();
    
    void printStats();
    void goToPose(string arm_type, const yarp::sig::Vector &xd, const yarp::sig::Vector &od);
    void velMoveHandler(const bool b, std::vector<int> joints, double speed, IControlMode2* imod, IVelocityControl* ivel);
    void reachingHandler(string arm_type, const bool b, const yarp::sig::Vector &pos, const yarp::sig::Vector &rpy);
    void saturate(double& v, double sat_lim);
    void updateRVIZ(const yarp::sig::Vector &xd, const yarp::sig::Vector &od);
    void getCartesianArmPositions();

    void option1(double* axis);
    void option2(double* axis);
    void option3(double* axis);
    void option4(double* axis);

    int get_status();
};

#endif

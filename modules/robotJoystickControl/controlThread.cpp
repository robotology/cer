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

#include <sstream>
#include <iomanip>
#include <yarp/math/Math.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <limits>
#include "controlThread.h"

using namespace yarp::sig;
using namespace yarp::math;

void ControlThread::updateRVIZ(const Vector &xd, const Vector &od)
{
    double yarpTimeStamp = yarp::os::Time::now();
    uint64_t time;
    uint64_t nsec_part;
    uint64_t sec_part;
    TickTime ret;
    time = (uint64_t)(yarpTimeStamp * 1000000000UL);
    nsec_part = (time % 1000000000UL);
    sec_part = (time / 1000000000UL);
    if (sec_part > std::numeric_limits<unsigned int>::max())
    {
        yWarning() << "Timestamp exceeded the 64 bit representation, resetting it to 0";
        sec_part = 0;
    }

    visualization_msgs_MarkerArray& markerarray = rosPublisherPort.prepare();
    markerarray.markers.clear();
    visualization_msgs_Marker marker;
    marker.header.frame_id = "mobile_base_body_link";
    marker.header.stamp.sec = (yarp::os::NetUint32) sec_part;
    marker.header.stamp.nsec = (yarp::os::NetUint32) nsec_part;
    marker.ns = "cer-teleop_namespace";
    marker.type = visualization_msgs_Marker::SPHERE;
    marker.action = visualization_msgs_Marker::ADD;

    //center
    Quaternion q;
    q.fromRotationMatrix(axis2dcm(od));
    marker.id = 1;
    marker.pose.position.x = xd[0];
    marker.pose.position.y = xd[1];
    marker.pose.position.z = xd[2];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    markerarray.markers.push_back(marker);

    /*
    //x
    marker.id = 2;
    marker.pose.orientation.x = q[3];
    marker.pose.orientation.y = q[0];
    marker.pose.orientation.z = q[1];
    marker.pose.orientation.w = q[2];
    marker.scale.x = 0.5;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    markerarray.markers.push_back(marker);

    //y
    marker.id = 3;
    marker.pose.orientation.x = q[3];
    marker.pose.orientation.y = q[0];
    marker.pose.orientation.z = q[1];
    marker.pose.orientation.w = q[2];
    marker.scale.x = 0.05;
    marker.scale.y = 0.5;
    marker.scale.z = 0.05;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    markerarray.markers.push_back(marker);

    //z
    marker.id = 4;
    marker.pose.orientation.x = q[3];
    marker.pose.orientation.y = q[0];
    marker.pose.orientation.z = q[1];
    marker.pose.orientation.w = q[2];
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    markerarray.markers.push_back(marker);
    */
    rosPublisherPort.write();
}

void ControlThread::reachingHandler(string arm_type, const bool b, const Vector &pos, const Vector &rpy)
{
    ///yDebug() << "reaching handler";
    //return;

    Vector xd(4, 0.0);
    xd[0] = (pos[0]);
    xd[1] = (pos[1]);
    xd[2] = (pos[2]);
    xd[3] = 1.0;

    Vector drpy(3);
    drpy[0] = (rpy[0]);
    drpy[1] = (rpy[1]);
    drpy[2] = (rpy[2]);

    Vector ax(4, 0.0), ay(4, 0.0), az(4, 0.0);
    ax[0] = 1.0; ax[3] = drpy[2] * ((arm_type == "right") ? +1.0 : -1.0);
    ay[1] = 1.0; ay[3] = drpy[1] * ((arm_type == "right") ? -1.0 : +1.0);
    az[2] = 1.0; az[3] = drpy[0] * ((arm_type == "right") ? -1.0 : +1.0);

    Matrix Rd = axis2dcm(ax)*axis2dcm(ay)*axis2dcm(az);

    Vector od = dcm2axis(Rd);

    goToPose(arm_type, xd, od);
    //updateGazebo(xd, od);
    
    updateRVIZ(xd, od);

   /* if (c0 != 0)
    {
        stopReaching();
    }*/
}

void ControlThread::velMoveHandler(const bool b, std::vector<int> joints, double speed, IControlMode2* imod, IVelocityControl* ivel)
{
    for (size_t i = 0; i < joints.size(); i++)
    {
        imod->setControlMode(joints[i], VOCAB_CM_VELOCITY);
        if (b)
        {
            ivel->velocityMove(joints[i], speed);
        }
        else
        {
            ivel->velocityMove(joints[i], -speed);
        }
    }

}

void ControlThread::saturate(double& v, double sat_lim)
{
    if (v < -sat_lim) v = -sat_lim;
    if (v > sat_lim) v = sat_lim;
}

#define AXIS_NUM               10
#define AXIS_DUMMY              0
#define AXIS_L_SHIFT            4
#define AXIS_R_SHIFT            6
#define AXIS_LEFT_HORIZONTAL    2
#define AXIS_LEFT_VERTICAL      1
#define AXIS_RIGHT_HORIZONTAL   3
#define AXIS_RIGHT_VERTICAL     5
#define AXIS_DIGITAL_HORIZONTAL 7
#define AXIS_DIGITAL_VERTICAL   8

void ControlThread::option4(double* axis)
{
    std::vector<int> hand_joints(2);
    hand_joints[0] = 0;
    hand_joints[1] = 1;

    std::vector<int> torso_yaw(1);
    torso_yaw[0] = 3;

    if (motors_enabled)
    {
        //right_hand
        double val0 = axis[AXIS_RIGHT_VERTICAL];     //right_hand u/d
        if (val0 > 0.1 || val0 < -0.1) velMoveHandler(val0>0, hand_joints, 50.0, interface_right_hand_iCmd, interface_right_hand_iVel);

        //left_hand
        double val1 = axis[AXIS_LEFT_VERTICAL];      //left_hand u/d
        if (val1 > 0.1 || val1 < -0.1) velMoveHandler(val1>0, hand_joints, 50.0, interface_left_hand_iCmd, interface_left_hand_iVel);

        double val2 = axis[AXIS_RIGHT_HORIZONTAL];   //torso yaw
        if (val2 > 80 || val2 < -80) velMoveHandler(val2>0, torso_yaw, 15.0, interface_torso_equiv_iCmd, interface_torso_equiv_iVel);
    }
}

void ControlThread::option3(double* axis)
{
    //right_arm
    double val0 = axis[AXIS_RIGHT_VERTICAL];     //right_arm u/d
    double val1 = axis[AXIS_RIGHT_HORIZONTAL];   //right_arm l/r
    double val2 = axis[AXIS_DIGITAL_HORIZONTAL]; //right_arm f/b
    double val3 = axis[AXIS_LEFT_HORIZONTAL];    //right_arm rot u/d
    double val4 = axis[AXIS_LEFT_VERTICAL];      //right_arm rot l/r
    double val5 = axis[AXIS_DIGITAL_VERTICAL];   //right_arm rot f/b
    saturate(val0, 100);
    saturate(val1, 100);
    saturate(val2, 100);
    saturate(val3, 100);
    saturate(val4, 100);
    saturate(val5, 100);

    double gain_0 = 0.000025;
    double gain_1 = 0.000025;
    double gain_2 = 0.000025;
    double gain_3 = 0.00005;
    double gain_4 = 0.00005;
    double gain_5 = 0.00005;

    current_xyz_right[0] += val0 * gain_0;
    current_xyz_right[1] += val1 * gain_1;
    current_xyz_right[2] += val2 * gain_2;
    current_rpy_right[0] += val3 * gain_3;
    current_rpy_right[1] += val4 * gain_4;
    current_rpy_right[2] += val5 * gain_5;
    reachingHandler("right_arm", 0, current_xyz_right, current_rpy_right);
}

void ControlThread::option2(double* axis)
{
    //left_arm
    double val0 = axis[AXIS_RIGHT_VERTICAL];     //left_arm u/d
    double val1 = axis[AXIS_RIGHT_HORIZONTAL];   //left_arm l/r
    double val2 = axis[AXIS_DIGITAL_HORIZONTAL]; //left_arm f/b
    double val3 = axis[AXIS_LEFT_HORIZONTAL];    //left_arm rot u/d
    double val4 = axis[AXIS_LEFT_VERTICAL];      //left_arm rot l/r
    double val5 = axis[AXIS_DIGITAL_VERTICAL];   //left_arm rot f/b
    saturate(val0, 100);
    saturate(val1, 100);
    saturate(val2, 100);
    saturate(val3, 100);
    saturate(val4, 100);
    saturate(val5, 100);

    static double x;
    static double y;
    static double z;
    static double rx;
    static double ry;
    static double rz;

    double gain_0 = 0.000025;
    double gain_1 = 0.000025;
    double gain_2 = 0.000025;
    double gain_3 = 0.00005;
    double gain_4 = 0.00005;
    double gain_5 = 0.00005;

    current_xyz_left[0] += val0 * gain_0;
    current_xyz_left[1] += val1 * gain_1;
    current_xyz_left[2] += val2 * gain_2;
    current_rpy_left[0] += val3 * gain_3;
    current_rpy_left[1] += val4 * gain_4;
    current_rpy_left[2] += val5 * gain_5;
    reachingHandler("left_arm", 0, current_xyz_left, current_rpy_left);
}

void ControlThread::option1(double* axis)
{
    //torso and base
    double val0 = axis[AXIS_RIGHT_VERTICAL];     //torso up/downn
    double val1 = axis[AXIS_DIGITAL_HORIZONTAL]; //torso l/r
    double val2 = axis[AXIS_DIGITAL_VERTICAL];   //torso f/b

    double val3 = axis[AXIS_RIGHT_HORIZONTAL];   //base turn
    double val4 = axis[AXIS_LEFT_HORIZONTAL];    //head
    double val5 = axis[AXIS_LEFT_VERTICAL];      //base f/b
    double val6 = axis[AXIS_L_SHIFT];

    saturate(val0, 100);
    saturate(val1, 100);
    saturate(val2, 100);
    saturate(val3, 100);
    saturate(val4, 100);
    saturate(val5, 100);

    double gain_0 = 0.0001;
    double gain_1 = 0.0001;
    double gain_2 = 0.0002;
    double gain_4 = 0.0001;

    val0 = val0*gain_0;
    val1 = val1*gain_1;
    val2 = val2*gain_2;
    val4 = val4*gain_4;

    //double torso_velA = val2 + val0 + 0;
    //double torso_velB = val2 - val0 / 2 + val1 / 1.732050808;
    //double torso_velC = val2 - val0 / 2 - val1 / 1.732050808;
    double torso_velA = val2;// + val0 + 0;
    double torso_velB = val2;// - val0 / 2 + val1 / 1.732050808;
    double torso_velC = val2;// - val0 / 2 - val1 / 1.732050808;


    //torso control
    if (motors_enabled &&
    fabs(torso_velA) >  0.001 &&
    fabs(torso_velB) >  0.001 &&
    fabs(torso_velC) >  0.001)
    {
        int mods[3];
        interface_torso_tripod_iCmd->getControlModes(mods);
        if (mods[0] != VOCAB_CM_VELOCITY)
        {
            interface_torso_tripod_iCmd->setControlMode(0, VOCAB_CM_VELOCITY);
            yarp::os::Time::delay(0.005);
        }
        if (mods[1] != VOCAB_CM_VELOCITY)
        {
            interface_torso_tripod_iCmd->setControlMode(1, VOCAB_CM_VELOCITY);
            yarp::os::Time::delay(0.005);
        }
        if (mods[2] != VOCAB_CM_VELOCITY)
        {
            interface_torso_tripod_iCmd->setControlMode(2, VOCAB_CM_VELOCITY);
            yarp::os::Time::delay(0.005);
        }
        double torso_vels[3];
        torso_vels[0] = torso_velA;
        torso_vels[1] = torso_velB;
        torso_vels[2] = torso_velC;
        interface_torso_tripod_iVel->velocityMove(torso_vels);
    }
    

    //base control
    if (motors_enabled)
    {
        Bottle& b = robotCmdPort_base.prepare();
        b.clear();
        b.addInt(3);
        b.addDouble(val5); //x_lin_speed
        b.addDouble(0.0); //y_lin_speed
        b.addDouble(val3); //ang_speed
        b.addDouble(100.0); //gain
        robotCmdPort_base.write();
    }
}

void ControlThread::getCartesianArmPositions(bool blocking)
{
    Bottle *sl = this->robotStatusPort_larm.read(blocking);
    Bottle *sr = this->robotStatusPort_rarm.read(blocking);

    if (sl)
    {
        initial_xyz_left.resize(3);
        initial_rpy_left.resize(3);
        current_xyz_left.resize(3);
        current_rpy_left.resize(3);
        current_xyz_left[0] = initial_xyz_left[0] = sl->get(0).asDouble();
        current_xyz_left[1] = initial_xyz_left[1] = sl->get(1).asDouble();
        current_xyz_left[2] = initial_xyz_left[2] = sl->get(2).asDouble();
        Vector vi(4), vo(3);
        vi[0] = sl->get(3).asDouble();
        vi[1] = sl->get(4).asDouble();
        vi[2] = sl->get(5).asDouble();
        vi[3] = 0;
#if 1
        double norm = sqrt((vi[0] * vi[0]) + (vi[1] * vi[1]) + (vi[2] * vi[2]));
        vi[0] = vi[0] / norm;
        vi[1] = vi[1] / norm;
        vi[2] = vi[2] / norm;
        vi[3] = norm;
        Matrix m = yarp::math::axis2dcm(vi);
        vo = yarp::math::dcm2rpy(m);
#else
        vo[0] = vi[0]; vo[1] = vi[1]; vo[2] = vi[2];
#endif 
        current_rpy_left[0] = initial_rpy_left[0] = vo[0];
        current_rpy_left[1] = initial_rpy_left[1] = vo[1];
        current_rpy_left[2] = initial_rpy_left[2] = vo[2];
    }
    if (sr)
    {
        initial_xyz_right.resize(3);
        initial_rpy_right.resize(3);
        current_xyz_right.resize(3);
        current_rpy_right.resize(3);
        current_xyz_right[0] = initial_xyz_right[0] = sr->get(0).asDouble();
        current_xyz_right[1] = initial_xyz_right[1] = sr->get(1).asDouble();
        current_xyz_right[2] = initial_xyz_right[2] = sr->get(2).asDouble();
        Vector vi(4), vo(3);
        vi[0] = sr->get(3).asDouble();
        vi[1] = sr->get(4).asDouble();
        vi[2] = sr->get(5).asDouble();
#if 1
        double norm = sqrt((vi[0] * vi[0]) + (vi[1] * vi[1]) + (vi[2] * vi[2]));
        vi[0] = vi[0] / norm;
        vi[1] = vi[1] / norm;
        vi[2] = vi[2] / norm;
        vi[3] = norm;
        Matrix m = yarp::math::axis2dcm(vi);
        vo = yarp::math::dcm2rpy(m);
#else
        vo[0] = vi[0]; vo[1] = vi[1]; vo[2] = vi[2];
#endif 
        current_rpy_right[0] = initial_rpy_right[0] = vo[0];
        current_rpy_right[1] = initial_rpy_right[1] = vo[1];
        current_rpy_right[2] = initial_rpy_right[2] = vo[2];
    }
}

#define LEFT_AND_RIGHT_BUTTONS_PUSHED (fabs(axis[AXIS_L_SHIFT]) > 10 && fabs(axis[AXIS_R_SHIFT]) > 10)
#define LEFT_BUTTON_PUSHED            (fabs(axis[AXIS_L_SHIFT]) > 10 && fabs(axis[AXIS_R_SHIFT]) < 10)
#define RIGHT_BUTTON_PUSHED           (fabs(axis[AXIS_L_SHIFT]) < 10 && fabs(axis[AXIS_R_SHIFT]) > 10)
#define NO_BUTTONS_PUSHED             (fabs(axis[AXIS_L_SHIFT]) < 10 && fabs(axis[AXIS_R_SHIFT]) < 10)

//------------------------------------------------------------------------------------------------
void ControlThread::run()
{
    double pidout_linear_speed  = 0;
    double pidout_angular_speed = 0;
    double pidout_direction     = 0;
    double axis[20];
    static int lb = 0;
    static int lo = 0;
    static int ro = 0;
    static int no = 0;

    if (first_run)
    {
        yInfo() << "Waiting for initial position...";
        Bottle* br = 0;
        Bottle* bl = 0;
        int timeout = 0;
        do
        {
            bl = robotStatusPort_larm.read(false);
            br = robotStatusPort_rarm.read(false);
            timeout++;
            yarp::os::Time::delay(1.0);
            if      (bl == 0 && br != 0) yWarning("Cartesian controller left arm not ready yet");
            else if (bl != 0 && br == 0) yWarning("Cartesian controller right arm not ready yet");
            else if (bl == 0 && br == 0) yWarning("Cartesian controller left arm and right arm not ready yet");
            if (timeout >= 30)
            {
                yError() << "Unable to connect to cartesian controller, terminating....";
                error_status = true;
                return;
            }
        }
        while (bl==0 || br==0);

        getCartesianArmPositions(true);
        first_run = false;
        yInfo() << "...done";
    }

    Bottle *b = this->port_joystick_control.read(false);
    if (b)
    {
        for (int j = 0; j < AXIS_NUM; j++)
        {
            axis[j] = b->get(j).asDouble();
        }

        if (LEFT_AND_RIGHT_BUTTONS_PUSHED) { lb++; lo = 0; ro = 0; no = 0; }
        else if (LEFT_BUTTON_PUSHED)       { lb=0; lo++;   ro = 0; no = 0; }
        else if (RIGHT_BUTTON_PUSHED)      { lb=0; lo = 0; ro++  ; no = 0; }
        else if (NO_BUTTONS_PUSHED)        { lb=0; lo = 0; ro = 0; no++;   }

        controlling = 0;
        if (LEFT_AND_RIGHT_BUTTONS_PUSHED && lb>10)
        {
            controlling = 1;
            option1(axis);
            getCartesianArmPositions(false);
            return;
        }
        else if (LEFT_BUTTON_PUSHED && lo>10)
        {
            controlling = 2;
            option2(axis);
            return;
        }
        else if (RIGHT_BUTTON_PUSHED && ro>10)
        {
            controlling = 3;
            option3(axis);
            return;
        }
        else if (NO_BUTTONS_PUSHED && no>10)
        {
            controlling = 4;
            option4(axis); //hands and torso yaw
            getCartesianArmPositions(false);
            return;
        }
    }
    else 
    {
        //yDebug() <<" empty, nothing received from joy port";
        return;
    }

}

void ControlThread::printStats()
{
   // ostringstream stats;
   // stats<<setprecision(3)<<"elong: "<<elong<<" pitch: "<<pitch<<" roll: "<<roll;
   // yDebug()<<stats.str();
}


robot_status ControlThread::get_status()
{
    robot_status current_status;
    if (first_run == false)
    {
        current_status.controlling = controlling;
        current_status.left_arm_xyz = current_xyz_left;
        current_status.right_arm_xyz = current_xyz_right;
    }

    if (error_status)
    {
        current_status.status = -1;
    }
    else
    {
        current_status.status = 0;
    }
    return current_status;
}

bool ControlThread::threadInit()
{
    error_status = false;
    bool autoconnect = false;
    rosNode = new yarp::os::Node(localName);
    if (!rosPublisherPort.topic(localName+"_marker"))
    {
        yError()<<"Unable to publish data on " << localName << "_marker" << " topic";
        yWarning("Check your yarp-ROS network configuration");
        return false;
    }

    //open the joystick port
    port_joystick_control.open(localName + "/joystick:i");

        //try to connect to joystickCtrl output
        if (rf.check("autoconnect"))
        {
            autoconnect = true;
            int joystick_trials = 0; 
            do
            {
                yarp::os::Time::delay(1.0);
                if (yarp::os::Network::connect("/joystickCtrl:o", localName+"/joystick:i"))
                    {
                        yInfo("Joystick has been automatically connected");
                        break;
                    }
                else
                    {
                        yWarning("Unable to find the joystick port, retrying (%d/5)...",joystick_trials);
                        joystick_trials++;
                    }

                if (joystick_trials>=5)
                    {
                        yError("Unable to find the joystick port, giving up");
                        break;
                    }
            }
            while (1);
        }
        
    // open the control board driver
    yInfo("Opening the motors interface...\n");
    int trials = 0;
    double start_time = yarp::os::Time::now();

    Property torso_tripod_control_board_options("(device remote_controlboard)");
    torso_tripod_control_board_options.put("remote", "/"+robotName+"/torso_tripod");
    torso_tripod_control_board_options.put("local", localName + "/torso_tripod");

    Property torso_equiv_control_board_options("(device remote_controlboard)");
    torso_equiv_control_board_options.put("remote", "/"+robotName+"/torso");
    torso_equiv_control_board_options.put("local", localName + "/torso");

    Property left_hand_control_board_options("(device remote_controlboard)");
    left_hand_control_board_options.put("remote", "/" + robotName + "/left_hand");
    left_hand_control_board_options.put("local", localName + "/left_hand");

    Property right_hand_control_board_options("(device remote_controlboard)");
    right_hand_control_board_options.put("remote", "/" + robotName + "/right_hand");
    right_hand_control_board_options.put("local", localName + "/right_hand");

    Property head_control_board_options("(device remote_controlboard)");
    head_control_board_options.put("remote", "/" + robotName + "/head");
    head_control_board_options.put("local", localName + "/head");
    do
    {
        double current_time = yarp::os::Time::now();

        //remove previously existing drivers
        if (driver_torso_tripod)
        {
            delete driver_torso_tripod;
            driver_torso_tripod = 0;
        }
        if (driver_torso_equiv)
        {
            delete driver_torso_equiv;
            driver_torso_equiv = 0;
        }
        if (driver_head)
        {
            delete driver_head;
            driver_head = 0;
        }
        if (driver_left_hand)
        {
            delete driver_left_hand;
            driver_left_hand = 0;
        }
        if (driver_right_hand)
        {
            delete driver_right_hand;
            driver_right_hand = 0;
        }
        //creates the new device driver
        driver_torso_tripod = new PolyDriver(torso_tripod_control_board_options);
        driver_torso_equiv = new PolyDriver(torso_equiv_control_board_options);
        driver_left_hand = new PolyDriver  (left_hand_control_board_options);
        driver_right_hand = new PolyDriver (right_hand_control_board_options);
        driver_head = new PolyDriver(head_control_board_options);
        bool connected = true;
        connected &= driver_torso_tripod->isValid();
        connected &= driver_torso_equiv->isValid();
        connected &= driver_left_hand->isValid();
        connected &= driver_right_hand->isValid();
        connected &= driver_head->isValid();

        //check if the driver is connected
        if (connected) break;

        //check if the timeout (10s) is expired
        if (current_time - start_time > 10.0)
        {
            yError("It is not possible to instantiate the device driver. I tried %d times!", trials);
            if (driver_torso_tripod)
            {
                delete driver_torso_tripod;
                driver_torso_tripod = 0;
            }
            if (driver_head)
            {
                delete driver_head;
                driver_head = 0;
            }
            if (driver_torso_equiv)
            {
                delete driver_torso_equiv;
                driver_torso_equiv = 0;
            }
            if (driver_left_hand)
            {
                delete driver_left_hand;
                driver_left_hand = 0;
            }
            if (driver_right_hand)
            {
                delete driver_right_hand;
                driver_right_hand = 0;
            }
            return false;
        }

        yarp::os::Time::delay(0.5);
        trials++;
        yWarning("Unable to connect the device driver, trying again...");
    } while (true);

    robotCmdPort_larm.open(localName + "/larm/cmd:rpc");
    robotCmdPort_rarm.open(localName + "/rarm/cmd:rpc");
    robotTargetPort_larm.open(localName + "/larm/target:o");
    robotTargetPort_rarm.open(localName + "/rarm/target:o");
    robotStatusPort_larm.open(localName + "/larm/status:i");
    robotStatusPort_rarm.open(localName + "/rarm/status:i");
    robotCmdPort_base.open(localName + "/base/cmd:o");

    if (autoconnect)
    {
        bool base_connected = yarp::os::Network::connect(localName + "/base/cmd:o", "/baseControl/joystick:i");
        bool left_cartesian_arm_connected = yarp::os::Network::connect("/cer_reaching-controller/left/state:o", localName + "/larm/status:i");
        bool right_cartesian_arm_connected = yarp::os::Network::connect("/cer_reaching-controller/right/state:o", localName + "/rarm/status:i");
        bool left_arm_output_connected = yarp::os::Network::connect(localName + "/larm/target:o", "/cer_reaching-controller/left/target:i");
        bool right_arm_output_connected = yarp::os::Network::connect(localName + "/rarm/target:o", "/cer_reaching-controller/right/target:i");
        if (base_connected == false)
        {
            yError() << "Failed to open mobile base interfaces";
        }
        if (left_cartesian_arm_connected == false)
        {
            yError() << "Failed to open left_arm  cartesian interfaces";
        }
        if (right_cartesian_arm_connected == false)
        {
            yError() << "Failed to open right_arm cartesian interfaces";
        }
        if (left_arm_output_connected == false)
        {
            yError() << "Failed to open left_arm motor interfaces";
        }
        if (right_arm_output_connected == false)
        {
            yError() << "Failed to open right_arm motor interfaces";
        }
    }

    driver_torso_tripod->view(interface_torso_tripod_iDir);
    driver_torso_tripod->view(interface_torso_tripod_iVel);
    driver_torso_tripod->view(interface_torso_tripod_iEnc);
    driver_torso_tripod->view(interface_torso_tripod_iPos);
    driver_torso_tripod->view(interface_torso_tripod_iCmd);

    driver_torso_equiv->view(interface_torso_equiv_iPos);
    driver_torso_equiv->view(interface_torso_equiv_iCmd);
    driver_torso_equiv->view(interface_torso_equiv_iVel);

    driver_head->view(interface_head_iPos);
    driver_head->view(interface_head_iCmd);
    driver_head->view(interface_head_iVel);

    if (interface_torso_tripod_iDir == 0 || interface_torso_tripod_iEnc == 0 || interface_torso_tripod_iVel == 0 || interface_torso_tripod_iCmd == 0)
    {
        yError() << "Failed to open torso_tripod interfaces";
        return false;
    }
    if (interface_torso_equiv_iPos == 0 || interface_torso_equiv_iVel == 0 || interface_torso_equiv_iCmd == 0)
    {
        yError() << "Failed to open torso_equiv interfaces";
        return false;
    }
    if (interface_head_iPos == 0 || interface_head_iVel == 0 || interface_head_iCmd == 0)
    {
        yError() << "Failed to open head interfaces";
        return false;
    }
    driver_right_hand->view(interface_right_hand_iVel);
    driver_right_hand->view(interface_right_hand_iCmd);
    if (interface_right_hand_iVel == 0 || interface_right_hand_iCmd == 0)
    {
        yError() << "Failed to open right_hand interfaces";
        return false;
    }
    driver_left_hand->view(interface_left_hand_iVel);
    driver_left_hand->view(interface_left_hand_iCmd);
    if (interface_left_hand_iVel == 0 || interface_left_hand_iCmd == 0)
    {
        yError() << "Failed to open left_hand interfaces";
        return false;
    }

    yarp::os::Time::delay(1.0);
    interface_torso_equiv_iVel->setRefAcceleration(0, 10000000);
    interface_torso_equiv_iVel->setRefAcceleration(1, 10000000);
    interface_torso_equiv_iVel->setRefAcceleration(2, 10000000);
    interface_torso_equiv_iVel->setRefAcceleration(3, 10000000);

    interface_torso_tripod_iVel->setRefAcceleration(0, 10000000);
    interface_torso_tripod_iVel->setRefAcceleration(1, 10000000);
    interface_torso_tripod_iVel->setRefAcceleration(2, 10000000);

    interface_left_hand_iVel->setRefAcceleration(0, 10000000);
    interface_left_hand_iVel->setRefAcceleration(1, 10000000);

    interface_right_hand_iVel->setRefAcceleration(0, 10000000);
    interface_right_hand_iVel->setRefAcceleration(1, 10000000);

    interface_head_iVel->setRefAcceleration(0, 10000000);
    interface_head_iVel->setRefAcceleration(1, 10000000);
    return true;
}

void ControlThread::afterStart(bool s)
{
    if (s)
        yInfo("Control thread started successfully");
    else
        yError("Control thread did not start");
}

ControlThread::ControlThread(unsigned int _period, ResourceFinder &_rf, Property options) :
RateThread(_period), rf(_rf),
ctrl_options(options)
{
    first_run = true;
    driver_head = 0;
    driver_torso_tripod = 0;
    driver_torso_equiv = 0;
    driver_left_hand = 0;
    driver_right_hand = 0;
    thread_timeout_counter = 0;

    interface_torso_tripod_iDir = 0;
    interface_torso_tripod_iPos = 0;
    interface_torso_tripod_iEnc = 0;
    interface_torso_tripod_iVel = 0;
    interface_torso_tripod_iCmd = 0;

    interface_torso_equiv_iPos = 0;
    interface_torso_equiv_iVel = 0;
    interface_torso_equiv_iCmd = 0;

    interface_left_hand_iVel = 0;
    interface_left_hand_iCmd = 0;

    interface_right_hand_iVel = 0;
    interface_right_hand_iCmd = 0;

    interface_head_iVel = 0;
    interface_head_iCmd = 0;

    thread_period = _period;

    robotName = ctrl_options.find("robot").asString();
    localName = ctrl_options.find("local").asString();

    motors_enabled = false;

    torso_max_elong = rf.check("max_torso_elong", Value(0.2)).asDouble();
    torso_min_elong = rf.check("min_torso_elong", Value(0.0)).asDouble();
    torso_max_alpha = rf.check("max_torso_alpha", Value(15)).asDouble();

    if (rf.check("no_motors"))
    {
        yInfo("'no_motors' option found. Skipping motor control part.");
        motors_enabled = false;
    }
    if (rf.check("motors_enabled"))
    {
        yInfo("'motors_enabled' option found. Enabling motors.");
        motors_enabled = true;
    }
}

/**********************************************************/
void ControlThread::goToPose(string arm_type, const Vector &xd, const Vector &od)
{
    Vector od_ = od;
    od_ *= od_[3]; od_.pop_back();

    Vector payLoad;
    double torso_heave = 0.07;
    double wrist_heave = 0.02;
    payLoad = cat(payLoad, xd);
    payLoad = cat(payLoad, od_);

    Bottle target;
    target.addList().read(payLoad);

    Bottle params;
    Bottle &bLoad = params.addList().addList();
    bLoad.addString("mode");
    //string mode = "full_pose+no_torso_no_heave";
    string mode = "xyz_pose+no_torso_no_heave+no_torso_heave"; //no_torso_heave
    bLoad.addString(mode);

    if (arm_type == "left_arm")
    {
        Property &prop = robotTargetPort_larm.prepare();
        prop.clear();
        prop.put("parameters", params.get(0));
        prop.put("target", target.get(0));
        string s = prop.toString(); //debug only
        if (motors_enabled)
        {
            robotTargetPort_larm.write();
        }
    }
    else if (arm_type == "right_arm")
    {
        Property &prop = robotTargetPort_rarm.prepare();
        prop.clear();
        prop.put("parameters", params.get(0));
        prop.put("target", target.get(0));
        string s = prop.toString(); //debug only
        if (motors_enabled)
        {
            robotTargetPort_rarm.write();
        }
    }
    else
    {
        yDebug("should never reach here");
    }
}

void ControlThread::threadRelease()
{
    if (rosNode)
    {
        delete rosNode;
        rosNode = 0;
    }

    if (interface_torso_tripod_iCmd)
    {
        interface_torso_tripod_iCmd->setControlMode(0, VOCAB_CM_POSITION);
        interface_torso_tripod_iCmd->setControlMode(1, VOCAB_CM_POSITION);
        interface_torso_tripod_iCmd->setControlMode(2, VOCAB_CM_POSITION);
    }

    port_joystick_control.interrupt();
    port_joystick_control.close();

    robotStatusPort_larm.interrupt();
    robotStatusPort_larm.close();
    robotStatusPort_rarm.interrupt();
    robotStatusPort_rarm.close();

    robotTargetPort_larm.interrupt();
    robotTargetPort_larm.close();
    robotTargetPort_rarm.interrupt();
    robotTargetPort_rarm.close();

    robotCmdPort_larm.interrupt();
    robotCmdPort_larm.close();
    robotCmdPort_rarm.interrupt();
    robotCmdPort_rarm.close();
    robotCmdPort_base.interrupt();
    robotCmdPort_base.close();

    yInfo() << "Thread stopped";
}

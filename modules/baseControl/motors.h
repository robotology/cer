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

#ifndef MOTORS_CTRL_H
#define MOTORS_CTRL_H

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
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/pids.h>
#include <string>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

#define DEFAULT_MAX_LINEAR_VEL  0.42  // maximum linear  velocity (m/s)
#define DEFAULT_MAX_ANGULAR_VEL 24.0  // maximum angular velocity (deg/s)

class MotorControl
{
private:
    Property ctrl_options;

    double              thread_period;

    int                 board_control_modes[3];
    int                 thread_timeout_counter;

    int                 command_received;
    int                 auxiliary_received;
    int                 joystick_received;

    int                 mov_timeout_counter;
    int                 aux_timeout_counter;
    int                 joy_timeout_counter;

    //movement control variables (input from external)
    double              joy_linear_speed;
    double              joy_angular_speed;
    double              joy_desired_direction;
    double              joy_pwm_gain;

    double              cmd_linear_speed;
    double              cmd_angular_speed;
    double              cmd_desired_direction;
    double              cmd_pwm_gain;

    double              aux_linear_speed;
    double              aux_angular_speed;
    double              aux_desired_direction;
    double              aux_pwm_gain;

    //motor variables
    double              F_L;
    double              F_R;
    double              max_linear_vel;
    double              max_angular_vel;

protected:
    ResourceFinder            &rf;
    PolyDriver                *control_board_driver;
    BufferedPort<Bottle>      port_movement_control;
    BufferedPort<Bottle>      port_auxiliary_control;
    BufferedPort<Bottle>      port_joystick_control;

    int                motors_filter_enabled;
    string             localName;

    IPidControl       *ipid;
    IVelocityControl  *ivel;
    IEncoders         *ienc;
    IAmplifierControl *iamp;
    IOpenLoopControl  *iopl;
    IControlMode2     *icmd;

public:

    MotorControl(unsigned int _period, ResourceFinder &_rf, Property options, PolyDriver* _driver);
    ~MotorControl();
    bool set_control_velocity();
    bool set_control_openloop();
    bool set_control_idle();

    bool   open();
    void read_inputs(double *linear_speed,double *angular_speed,double *desired_direction, double *pwm_gain);
    void execute_none();
    void execute_openloop(double appl_linear_speed, double appl_angular_speed);
    void execute_speed(double appl_linear_speed, double appl_angular_speed);
    void decouple(double appl_linear_speed, double appl_angular_speed);
    void close();
    bool check_motors_on();
    void updateControlMode();
    void printStats();
    void set_motors_filter(int b) {motors_filter_enabled=b;}
 
    void read_percent_polar (const Bottle *b, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain);
    void read_percent_cart  (const Bottle *b, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain);
    void read_speed_polar   (const Bottle *b, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain);
    void read_speed_cart    (const Bottle *b, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain);
    double get_max_linear_vel()   {return max_linear_vel;}
    double get_max_angular_vel()  {return max_angular_vel;}
};

#endif

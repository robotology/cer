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

#include "controlThread.h"

void ControlThread::run()
{
    double pidout_linear_speed  = 0;
    double pidout_angular_speed = 0;
    double pidout_direction     = 0;

    Bottle *b = this->port_joystick_control.read(false);
    if (b)
    {
        double val0 = b->get(1).asDouble();
        double val1 = b->get(2).asDouble();
        double val2 = b->get(3).asDouble();
        if (val1 > 100) val1 = 100;
        if (val1 < -100) val1 = -100;
        if (val2 > 100) val2 = 100;
        if (val2 < -100) val2 = -100;
        pitch = val1/100.0*25.0 * sin(val0 / 180.0 * 3.14);
        roll  = val1/100.0*25.0 * cos(val0 / 180.0 * 3.14);
        elong = elong + val2 / 200000.0;
        if (elong > max_elong) elong = max_elong;
        if (elong < min_elong) elong = min_elong;
    }

    double enc_elong = 0;
    double enc_roll = 0;
    double enc_pitch = 0;

    iEnc->getEncoder(0, &enc_elong);
    iEnc->getEncoder(0, &enc_roll);
    iEnc->getEncoder(0, &enc_pitch);

    //-----------

    if (elong < min_elong)
    {
        elong = min_elong;
        yDebug() << "Out of limits elong" << elong;
    }
    if (elong > max_elong)
    {
        elong = max_elong;
        yDebug() << "Out of limits elong" << elong;
    }
    if (motors_enabled) iDir->setPosition(0, elong);
    
    //-----------

    if (pitch < -max_alpha)
    {
        pitch = -max_alpha;
        yDebug() << "Out of limits elong" << pitch;
    }
    if (pitch > max_alpha)
    {
        pitch = max_alpha;
        yDebug() << "Out of limits elong" << pitch;
    }
    if (motors_enabled) iDir->setPosition(1, pitch);

    //-----------

    if (roll < -max_alpha)
    {
        roll = -max_alpha;
        yDebug() << "Out of limits elong" << roll;
    }
    if (roll > max_alpha)
    {
        roll = max_alpha;
        yDebug() << "Out of limits elong" << roll;
    }
    if (motors_enabled) iDir->setPosition(2, roll);

}

void ControlThread::printStats()
{
    char buff[255];
    sprintf(buff, "elong: %+3.3f  pitch:%+3.3f  roll:%+3.3f ", elong, pitch, roll);
    yDebug() << buff;
}

bool ControlThread::threadInit()
{
    //open the joystick port
    port_joystick_control.open("/tripodJoystickCtrl/joystick:i");

    // open the control board driver
    yInfo("Opening the motors interface...\n");
    int trials = 0;
    double start_time = yarp::os::Time::now();
    Property control_board_options("(device remote_controlboard)");
    control_board_options.put("remote", remoteName.c_str());
    control_board_options.put("local", localName.c_str());

    do
    {
        double current_time = yarp::os::Time::now();

        //remove previously existing drivers
        if (control_board_driver)
        {
            delete control_board_driver;
            control_board_driver = 0;
        }

        //creates the new device driver
        control_board_driver = new PolyDriver(control_board_options);
        bool connected = control_board_driver->isValid();

        //check if the driver is connected
        if (connected) break;

        //check if the timeout (10s) is expired
        if (current_time - start_time > 10.0)
        {
            yError("It is not possible to instantiate the device driver. I tried %d times!", trials);
            if (control_board_driver)
            {
                delete control_board_driver;
                control_board_driver = 0;
            }
            return false;
        }

        yarp::os::Time::delay(0.5);
        trials++;
        yWarning("Unable to connect the device driver, trying again...");
    } while (true);

    control_board_driver->view(iDir);
    control_board_driver->view(iEnc);
    if (iDir == 0 || iEnc == 0)
    {
        yError() << "Failed to open interfaces";
        return false;
    }
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
    control_board_driver = 0;
    thread_timeout_counter = 0;
    iDir = 0;
    iEnc = 0;
    thread_period = _period;

    remoteName = ctrl_options.find("remote").asString();
    localName = ctrl_options.find("local").asString();
    motors_enabled = true;

    max_elong = rf.check("max_elong", Value(0.2)).asDouble();
    min_elong = rf.check("min_elong", Value(0.0)).asDouble();
    max_alpha = rf.check("max_alpha", Value(25)).asDouble();

    if (rf.check("no_motors"))
    {
        yInfo("'no_motors' option found. Skipping motor control part.");
        motors_enabled = false;
    }
}

void ControlThread::threadRelease()
{
    yInfo() << "Thread stopped";
}
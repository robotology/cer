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

#include "controlThread.h"

void ControlThread::run()
{
    double pidout_linear_speed  = 0;
    double pidout_angular_speed = 0;
    double pidout_direction     = 0;

    Bottle *b = this->port_joystick_control.read(false);
    if (b)
    {
        double val0 = b->get(8).asDouble(); 
        double val1 = b->get(7).asDouble(); 
        double val2 = b->get(5).asDouble(); //elong
        
        if (fabs(val0) < 20 && fabs(val1) < 20 && fabs(val2) < 20) return;

        if (val0 > 100) val0 = 100;
        if (val0 < -100) val0 = -100;
        if (val1 > 100) val1 = 100;
        if (val1 < -100) val1 = -100;
        if (val2 > 100) val2 = 100;
        if (val2 < -100) val2 = -100;
        val0=val0/10000;
        val1=val1/10000;
        val2=val2/5000;
    
        double vel1= val2 + val0          + 0;
        double vel2= val2 - val0/2      + val1/1.732050808;
        double vel3= val2 - val0/2      - val1/1.732050808;
    
        //yDebug() << vel1 <<vel2 << vel3;
    
        double vels[3];
        vels[0]=vel1;
        vels[1]=vel2;
        vels[2]=vel3;
        int mods[3];
        
        if (motors_enabled)
        {
		  if (1)
		  {	
	          iCmd -> getControlModes(mods);
              if (mods[0] != VOCAB_CM_VELOCITY)
              {
	    		  iCmd->setControlMode(0,VOCAB_CM_VELOCITY);
	    		  yarp::os::Time::delay(0.005);
	    	  }
	    	  if (mods[1] != VOCAB_CM_VELOCITY)
              {
	    		  iCmd->setControlMode(1,VOCAB_CM_VELOCITY);
	    		  yarp::os::Time::delay(0.005);
	    	  }
	    	  if (mods[2] != VOCAB_CM_VELOCITY)
              {
	    		  iCmd->setControlMode(2,VOCAB_CM_VELOCITY);
	    		  yarp::os::Time::delay(0.005);
	    	  }
	      }		  
          iVel -> velocityMove(vels);
        }    
    }
    else 
    {
		//yDebug() <<" empty";
		return;
	}

}

void ControlThread::printStats()
{
   // ostringstream stats;
   // stats<<setprecision(3)<<"elong: "<<elong<<" pitch: "<<pitch<<" roll: "<<roll;
   // yDebug()<<stats.str();
}

bool ControlThread::threadInit()
{
    //open the joystick port
    port_joystick_control.open("/torsoJoystickControl/joystick:i");

        //try to connect to joystickCtrl output
        if (rf.check("joystick_connect"))
        {
            int joystick_trials = 0; 
            do
            {
                yarp::os::Time::delay(1.0);
                if (yarp::os::Network::connect("/joystickCtrl:o","/torsoJoystickControl/joystick:i"))
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
    control_board_driver->view(iVel);
    control_board_driver->view(iEnc);
    control_board_driver->view(iPos);
    control_board_driver->view(iCmd);
    if (iDir == 0 || iEnc == 0 || iVel == 0 || iCmd == 0)
    {
        yError() << "Failed to open interfaces";
        return false;
    }

    yarp::os::Time::delay(1.0);
    iEnc->getEncoder(0, &enc_init_elong);
    iEnc->getEncoder(1, &enc_init_roll);
    iEnc->getEncoder(2, &enc_init_pitch);
    iVel ->setRefAcceleration (0,10000000);
    iVel ->setRefAcceleration (1,10000000);
    iVel ->setRefAcceleration (2,10000000);

    yDebug() << "Initial vals" << enc_init_elong << enc_init_roll << enc_init_pitch;
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
    iPos = 0;
    iEnc = 0;
    thread_period = _period;

    remoteName = ctrl_options.find("remote").asString();
    localName = ctrl_options.find("local").asString();
    motors_enabled = true;

    max_elong = rf.check("max_elong", Value(0.2)).asDouble();
    min_elong = rf.check("min_elong", Value(0.0)).asDouble();
    max_alpha = rf.check("max_alpha", Value(15)).asDouble();

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

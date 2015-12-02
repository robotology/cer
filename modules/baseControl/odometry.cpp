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

#include "odometry.h"

#define RAD2DEG 180.0/3.14159
#define DEG2RAD 3.14159/180.0

bool Odometry::reset_odometry()
{
    ienc->getEncoder(0,&encL_offset);
    ienc->getEncoder(1,&encR_offset);
    odom_x=0;
    odom_y=0;
    encvel_estimator->reset();
    yInfo("Odometry reset done");
    return true;
}

void Odometry::printStats()
{
    mutex.wait();
    //yInfo (stdout,"Odometry Thread: Curr motor velocities: %+3.3f %+3.3f %+3.3f\n", velA, velB, velC);
    yInfo("* Odometry Thread:");
    yInfo("enc1:%+9.1f enc2:%+9.1f enc3:%+9.1f ******** env1:%+9.3f env2:%+9.3f env3:%+9.3f",
    enc[0]*57, enc[1]*57, enc[2]*57, encv[0]*57, encv[1]*57, encv[2]*57);
    
    yInfo("ivlx:%+9.3f ivlx:%+9.3f             ******** ovlx:%+9.3f ovly:%+9.3f ovlt:%+9.3f ******** x: %+5.3f y: %+5.3f t: %+5.3f",
    base_vel_lin, base_vel_theta, odom_vel_x, odom_vel_y, odom_vel_theta, odom_x, odom_y, odom_theta);
    mutex.post();
}

void Odometry::close()
{    
    port_odometry.interrupt();
    port_odometry.close();
    port_odometer.interrupt();
    port_odometer.close();
    port_vels.interrupt();
    port_vels.close();
}

Odometry::~Odometry()
{
    close();
}

Odometry::Odometry(unsigned int _period, ResourceFinder &_rf, Property options, PolyDriver* _driver):rf(_rf)
{
    ctrl_options = options;
    period = _period;
    control_board_driver= _driver;
    odom_x=0;
    odom_y=0;
    odom_theta=0;

    odom_vel_x=0;
    odom_vel_y=0;
    odom_vel_theta = 0;

    base_vel_lin=0;
    base_vel_theta=0;

    traveled_distance=0;
    traveled_angle=0;
    geom_r = 62.5 / 1000.0;     //m
    geom_L = 297.16/1000.0;   //m
    encvel_estimator =new iCub::ctrl::AWLinEstimator(2,1.0);
    enc.resize(2);
    encv.resize(2);
    localName = ctrl_options.find("local").asString();
}

bool Odometry::open()
{
    // open the control board driver
    yInfo("Opening the motors interface...");

    Property control_board_options("(device remote_controlboard)");
    if (!control_board_driver)
    {
        yError("ERROR: control board driver not ready!");
            return false;
    }
    // open the interfaces for the control boards
    bool ok = true;
    ok = ok & control_board_driver->view(ienc);
    if(!ok)
    {
        yError("ERROR: one or more devices has not been viewed\nreturning...");
        //return false;
    }
    // open control input ports
    port_odometry.open((localName+"/odometry:o").c_str());
    port_odometer.open((localName+"/odometer:o").c_str());
    port_vels.open((localName+"/velocity:o").c_str());

    //reset odometry
    reset_odometry();

    return true;
}

void Odometry::compute()
{
    mutex.wait();

    //read the encoders (deg)
    ienc->getEncoder(0,&encL);
    ienc->getEncoder(1,&encR);
        
    //read the speeds (deg/s)
    ienc->getEncoderSpeed(0,&velL);
    ienc->getEncoderSpeed(1,&velR);
        
    //remove the offset and convert in radians
    enc[0]= -(encL - encL_offset) * 0.0174532925; 
    enc[1]= -(encR - encR_offset) * 0.0174532925;
       
    //estimate the speeds
    iCub::ctrl::AWPolyElement el;
    el.data=enc;
    el.time=Time::now();
    encv= encvel_estimator->estimate(el);

    //build the kinematics matrix
    yarp::sig::Matrix kin;
    kin.resize(2,3);
    kin.zero();
    kin(0, 0) = cos(odom_theta) / 2;
    kin(0, 1) = cos(odom_theta) / 2;
    kin(0, 2) = sin(odom_theta) / 2;
    kin(1, 0) = sin(odom_theta) / 2;
    kin(1, 1) = 1 / (2 * geom_L);
    kin(1, 2) = 1 / (2 * geom_L);

    yarp::sig::Vector odom_cart_vels;  //velocities expressed in the world reference frame
    yarp::sig::Vector base_cart_vels; //velocities expressed in the base reference frame
    odom_cart_vels  = kin*encv;
    base_cart_vels = kin*encv; //@@@

    base_vel_lin   = base_cart_vels[1];
    double dummy   = base_cart_vels[0];
    base_vel_theta = base_cart_vels[2];
    
    odom_vel_x      = odom_cart_vels[1];
    odom_vel_y      = odom_cart_vels[0];
    odom_vel_theta  = odom_cart_vels[2];
  
    //the integration step
    odom_x=odom_x + (odom_vel_x * period/1000.0);
    odom_y=odom_y + (odom_vel_y * period/1000.0);

    //compute traveled distance (odometer)
    traveled_distance = traveled_distance + fabs(base_vel_lin   * period/1000.0);
    traveled_angle    = traveled_angle    + fabs(base_vel_theta * period/1000.0);

    //convert from radians back to degrees
    odom_theta       *= RAD2DEG;
    base_vel_theta   *= RAD2DEG;
    odom_vel_theta   *= RAD2DEG;
    traveled_angle   *= RAD2DEG;

    mutex.post();

    timeStamp.update();
    if (port_odometry.getOutputCount()>0)
    {
        port_odometry.setEnvelope(timeStamp);
        Bottle &b=port_odometry.prepare();
        b.clear();
        b.addDouble(odom_x);
        b.addDouble(odom_y);
        b.addDouble(odom_theta);
        b.addDouble(odom_vel_x);
        b.addDouble(odom_vel_y);
        b.addDouble(odom_vel_theta);
        port_odometry.write();
    }

    if (port_odometer.getOutputCount()>0)
    {
        port_odometer.setEnvelope(timeStamp);
        Bottle &t=port_odometer.prepare();
        t.clear();
        t.addDouble(traveled_distance);
        t.addDouble(traveled_angle);
        port_odometer.write();
    }

    if (port_vels.getOutputCount()>0)
    {
        port_vels.setEnvelope(timeStamp);
        Bottle &v=port_vels.prepare();
        v.clear();
        v.addDouble(base_vel_lin);
        v.addDouble(base_vel_theta);
        port_vels.write();
    }
}

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

#ifndef IDLE_MOTIONS_H
#define IDLE_MOTIONS_H

#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <map>
#include <cmath>


using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

class IdleMotions : public RFModule
{
private:
    //Polydriver
    PolyDriver          m_drivers[4];
    IControlMode*       m_ictrlmode[4];  
    IPositionControl*   m_iposctrl[4]; 

    //Port
    RpcServer           m_rpc_port;
    string              m_rpc_port_name;

    double              m_period;
    string              m_robot;
    string              m_script_name;
    map<int, string>    m_motions;
    int                 m_number_of_possible_motions;
    bool                m_use_ctpservice;
    
public:
    //Constructor/Distructor
    IdleMotions();
    ~IdleMotions() = default;

    //Internal methods
    virtual bool configure(ResourceFinder &rf);
    virtual bool close();
    virtual double getPeriod();
    virtual bool updateModule();
    virtual bool respond(const Bottle &cmd, Bottle &reply);

    void setCtrlMode(const int part, int ctrlMode);

    string move();
    void arms_home();
    void head_home();
    void torso_home();
    void go_home();
    void stretch_right_arm();
    void stretch_left_arm();
    void stretch_shoulders();
    void stretch_back();
    void look_gripper();
    void look_watch();

};

#endif
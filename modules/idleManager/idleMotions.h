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
#include <yarp/os/PeriodicThread.h>
#include <map>
#include <cmath>


using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

class IdleMotions : public PeriodicThread, public TypedReaderCallback<Bottle>
{
private:
    //Polydriver
    PolyDriver          m_drivers[4];
    IControlMode*       m_ictrlmode[4];  
    IPositionControl*   m_iposctrl[4];

    string              m_robot;
    string              m_script_name;
    map<int, string>    m_motions;
    int                 m_number_of_possible_motions;
    bool                m_use_ctpservice;
    int                 m_min_idle_time_s;
    double              m_last_movement = 0;
    bool                m_dont_move;

    string              m_port_to_gaze_controller;
    string              m_port_of_gaze_controller;

    BufferedPort<Bottle> m_input_port;
    string              m_input_port_name;

    ResourceFinder&     m_rf;
    
public:
    //Constructor/Distructor
    IdleMotions(ResourceFinder &_rf, double _period = 0.1);
    ~IdleMotions() = default;

    //Internal methods
    bool threadInit() override;
    void threadRelease() override;
    void run() override;

    using TypedReaderCallback<Bottle>::onRead;
    void onRead(Bottle& b) override;

    void setCtrlMode(const int part, int ctrlMode);
    bool doMotion(int motion_number = -1);
    void dontMove(){m_dont_move=true;}
    void nowYouCanMove(){m_dont_move=false;}

    string move(int motion_number = -1);
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


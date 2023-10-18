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

#include "idleMotions.h"

YARP_LOG_COMPONENT(IDLE_MOTIONS, "r1_obr.idleManager.idleMotions")


IdleMotions::IdleMotions(ResourceFinder &_rf, double _period) :
    PeriodicThread(_period),
    m_rf(_rf), 
    m_dont_move(false)
{
    m_robot = "cer";
    m_script_name = "/home/user1/robotology/cer/app/idleManager/scripts/r1_idleMotions.sh";
    m_input_port_name = "/idleManager/idleMotions:i";
    m_use_ctpservice = true;
    m_min_idle_time_s = 20;
    m_port_to_gaze_controller="/eyeContactManager/control:o";
    m_port_of_gaze_controller="/cer_gaze-controller/target:i";
};


// --------------------------------------------------------------- //
bool IdleMotions::threadInit()
{
    if(m_rf.check("robot")) {m_robot = m_rf.find("robot").asString();}
    if(m_rf.check("min_idle_time")) {m_min_idle_time_s = m_rf.find("min_idle_time").asInt32();}
    if(m_rf.check("input_port")) {m_input_port_name = m_rf.find("input_port").asString();}

    
    if(m_rf.check("use_ctpservice_or_setpos")) 
    {
        m_use_ctpservice = !(m_rf.find("use_ctpservice_or_setpos").asString() != "ctpservice");
    }

    if(m_use_ctpservice)
    {
        Searchable& ctps_config = m_rf.findGroup("USE_CTPSERVICES");
        if(ctps_config.check("ctps_script_name")) {m_script_name = ctps_config.find("ctps_script_name").asString();}
    }
    
    // Polydriver config
    Property prop;

    prop.put("device","remote_controlboard");
    prop.put("local","/idleMotions/right_arm");
    prop.put("remote","/"+m_robot+"/right_arm");
    if (!m_drivers[0].open(prop))
    {
        yCError(IDLE_MOTIONS,"Unable to connect to %s",("/"+m_robot+"/right_arm").c_str());
        return false;
    }
    prop.clear();
    prop.put("device","remote_controlboard");
    prop.put("local","/idleMotions/left_arm");
    prop.put("remote","/"+m_robot+"/left_arm");
    if (!m_drivers[1].open(prop))
    {
        yCError(IDLE_MOTIONS,"Unable to connect to %s",("/"+m_robot+"/left_arm").c_str());
        return false;
    }
    prop.clear();
    prop.put("device","remote_controlboard");
    prop.put("local","/idleMotions/head");
    prop.put("remote","/"+m_robot+"/head");
    if (!m_drivers[2].open(prop))
    {
        yCError(IDLE_MOTIONS,"Unable to connect to %s",("/"+m_robot+"/head").c_str());
        return false;
    }
    prop.clear();
    prop.put("device","remote_controlboard");
    prop.put("local","/idleMotions/torso");
    prop.put("remote","/"+m_robot+"/torso");
    if (!m_drivers[3].open(prop))
    {
        yCError(IDLE_MOTIONS,"Unable to connect to %s",("/"+m_robot+"/torso").c_str());
        return false;
    }

    m_drivers[0].view(m_iposctrl[0]);
    m_drivers[1].view(m_iposctrl[1]);
    m_drivers[2].view(m_iposctrl[2]);
    m_drivers[3].view(m_iposctrl[3]);
    if(!m_iposctrl[0] || !m_iposctrl[1] || !m_iposctrl[2] || !m_iposctrl[3] )
    {
        yCError(IDLE_MOTIONS,"Error opening iPositionControl interfaces. Devices not available");
        return false;
    }

    m_drivers[0].view(m_ictrlmode[0]);
    m_drivers[1].view(m_ictrlmode[1]);
    m_drivers[2].view(m_ictrlmode[2]);
    m_drivers[3].view(m_ictrlmode[3]);
    if(!m_ictrlmode[0] || !m_ictrlmode[1] || !m_ictrlmode[2] || !m_ictrlmode[3])
    {
        yCError(IDLE_MOTIONS,"Error opening iControlMode interfaces. Devices not available");
        return false;
    }

    if (m_use_ctpservice)
    {
        //Motions names definition
        if(!m_rf.check("MOTIONS"))
        {
            yCError(IDLE_MOTIONS,"MOTIONS section missing in ini file. Please list motions name in ini file.");
            return false;
        }
        else
        {
            Searchable& config = m_rf.findGroup("MOTIONS");
            int idx {0};
            while (true) 
            {
                if (!config.check(to_string(idx)))
                    break;

                string motion_name = config.find(to_string(idx)).asString();           

                m_motions.insert({idx, motion_name });
                idx++;
            }
            if (idx==0)
            {
                yCError(IDLE_MOTIONS,"No motion name listed in MOTIONS section. Please list motions names in ini file.");
                return false;
            }
            else
                m_number_of_possible_motions = idx;
        }
    }

    //Opening input Port
    if (!m_input_port.open(m_input_port_name))
    {
        yCError(IDLE_MOTIONS, "Unable to open input port");
        return false;
    }
    m_input_port.useCallback(*this);

    Network::connect(m_port_to_gaze_controller,m_input_port_name); //listening to the commands for the gaze controller

    m_last_movement = Time::now();

    return true;
}


// --------------------------------------------------------------- //
void IdleMotions::onRead(Bottle &b)
{
    m_last_movement = Time::now();
}


// --------------------------------------------------------------- //
void IdleMotions::setCtrlMode(const int part, int ctrlMode)
{
    int NUMBER_OF_JOINTS;
    m_iposctrl[part]->getAxes(&NUMBER_OF_JOINTS);
    for (int i_joint=0; i_joint < NUMBER_OF_JOINTS; i_joint++){ m_ictrlmode[part]->setControlMode(i_joint, ctrlMode); } 
}


// --------------------------------------------------------------- //
bool IdleMotions::doMotion(int motion_number)
{      
    for (int i = 0 ; i<=3 ; i++)
    {
        setCtrlMode(i, VOCAB_CM_POSITION);
    }
    
    if(m_use_ctpservice)
    {
        if(motion_number == -1)
            motion_number = rand() % m_number_of_possible_motions;
        string motion_name = m_motions.at(motion_number);
        string command = m_script_name + " " + motion_name + " &";
        yCInfo(IDLE_MOTIONS, "Executing: %s", command.c_str() );

        int result = system(command.c_str());

        if(result != 0)
            return false;
    }
    else
    {
        string motion_name = move(motion_number);
        if (motion_name == "OutOfBounds")
            return false;

        yCInfo(IDLE_MOTIONS, "Motion: %s", motion_name.c_str() );
    }

    // for (int i = 0 ; i<=3 ; i++)
    // {
    //     setCtrlMode(i, VOCAB_CM_POSITION_DIRECT);
    // }

    return true;
}

// --------------------------------------------------------------- //
void IdleMotions::threadRelease()
{
    if(m_drivers[0].isValid())
    {
        m_drivers[0].close();
    }

    if(m_drivers[1].isValid())
    {
        m_drivers[1].close();
    }

    if(m_drivers[2].isValid())
    {
        m_drivers[2].close();
    }

    if(m_drivers[3].isValid())
    {
        m_drivers[3].close();
    }

    yCInfo(IDLE_MOTIONS, "Thread released");
}


// --------------------------------------------------------------- //
void IdleMotions::run()
{
    int extra_idle_time_s = rand() % 10;
    int period_s = m_min_idle_time_s + extra_idle_time_s;

    if (!m_dont_move && Time::now()-m_last_movement >= period_s)
    {
        //Deactivating the control of the head via gaze-controller
        Network::disconnect(m_port_to_gaze_controller,m_port_of_gaze_controller);

        doMotion();

        //Re-activating the control of the head via gaze-controller
        Network::connect(m_port_to_gaze_controller,m_port_of_gaze_controller);

        m_last_movement = Time::now();
    }
}
